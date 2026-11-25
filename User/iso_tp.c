/**
  ******************************************************************************
  * @file    iso_tp.c
  * @brief   ISO 15765-2 传输层：多帧接收重组 + 流控帧 + 单帧发送
  *
  * ★ 做什么
  *   把 CAN 上一条超过 8 字节的报文拆开收、拼回完整一包交给上层。
  *   发送方向只实现单帧，理由见 iso_tp.h 抬头。
  *
  * ★ 为什么单独一个文件
  *   UDS 下周才写，但 ISO-TP 现在就能独立验证：只要"256 字节原样收回来"成立，
  *   UDS 就只剩业务逻辑。两层分开，出错时不用猜是分帧错了还是服务写错了。
  *
  * ★ 影响什么
  *   占 CAN1 的 RX0 中断和 288 字节静态 RAM；不改任何现有 CAN 发送行为。
  *
  * ★ 怎么验证
  *   开 TEST_ISO_TP_DUMP，PC 发一条 256 字节报文，比对串口打回的长度和 CRC16。
  ******************************************************************************
  */

#include "iso_tp.h"

#if (CAN_EN != 0)

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"          /* xTaskGetTickCount：只在任务侧用，中断里不碰 */
#include "ProbeConfig.h"
#include "Crc16.h"
#include "Serial.h"
#include "can_tx.h"        /* 2026-11-24：命令帧 0x201 的分发 */

volatile uint32_t g_isotp_rx_sf   = 0u;
volatile uint32_t g_isotp_rx_msg  = 0u;
volatile uint32_t g_isotp_rx_drop = 0u;
volatile uint32_t g_isotp_fc_tx   = 0u;
volatile uint32_t g_isotp_drop_why = 0u;

/* ★ 2026-11-24 加：给 UDS 0x22 读的两级快照
 * · s_done_* 存最新一包，g_isotp_prev_* 存它之前那一包
 * · 为什么要两级：读 0x22 的请求本身也是一包，会把最新那包顶掉
 */
static uint16_t s_done_len = 0u;
static uint16_t s_done_crc = 0u;
volatile uint16_t g_isotp_prev_len = 0u;
volatile uint16_t g_isotp_prev_crc = 0u;

/* ★ 最近 4 次丢包的原因（2026-11-23 加）
 * · 一次事件常常连记两条（比如"等超时"之后"残余连续帧涌进来"各记一次），
 *   只留最后一个会把第一条的原因盖掉 —— 上一轮就是这么丢了线索
 * · 4 个字节的环，够看清一次事件的来龙去脉 */
volatile uint8_t  g_isotp_why_ring[4];
volatile uint8_t  g_isotp_why_cnt = 0u;

/*==============================================================================
 *  ★ 记一次丢包
 *
 *  ★ 为什么集中成一个函数：
 *    原来每个丢包点自己写 "why=N; drop++"，两个语句分散在 8 个地方，
 *    想加"历史记录"就得改 8 遍。收成一处之后，以后加计数维度只改这里。
 *
 *  ★ 为什么可以被中断调用：只做自增和赋值，没有循环、没有阻塞
 *============================================================================*/
static void IsoTp_NoteDrop(uint8_t why)
{
	g_isotp_drop_why = why;
	g_isotp_why_ring[g_isotp_why_cnt & 0x03u] = why;
	g_isotp_why_cnt++;
	g_isotp_rx_drop++;
}

/* ★ 接收状态机
 * · IDLE：等首帧或单帧
 * · RECV：已经收到首帧，正在按序号收连续帧
 * · 只有这两个状态，因为发送方向没有多帧，不需要 TX 状态机 */
typedef enum
{
	ISOTP_ST_IDLE = 0,
	ISOTP_ST_RECV = 1
} IsoTpState_t;

static IsoTpState_t s_state;
static uint16_t s_total;      /* 首帧里声明的总长度（同时只有一个包在收） */
static uint16_t s_got;        /* 已经收进来的字节数 */
static uint8_t  s_sn;         /* 下一个应该收到的连续帧序号（0~15） */
static uint8_t  s_bs_left;    /* 本块还允许收几个连续帧 */

/* ★ 双缓冲（2026-11-24 加）
 * · 做什么：两块 288 字节接收缓冲，一块在收、一块在处理，互不覆盖
 * · 为什么：原来只有一块。Uds_Poll 取走包之后到 Release 之前这块一直被占着，
 *   而响应是在 Release 之前发出去的 —— PC 收到响应马上发下一块，
 *   撞上"缓冲还占着"就整包丢掉（why=5），PC 只能干等 1.5 秒超时重试。
 *   311 块里丢了 37 次，光重试就吃掉 55 秒。
 * · 影响什么：RAM 多占 288 字节，ZI 从 17236 涨到约 17524，20 KB 里还放得下
 * · 怎么验证：跑 download_test.py，失败次数应当从 37 降到 0
 */
static uint8_t  s_buf[2][ISOTP_RX_BUF];
static uint16_t s_len[2];      /* 每个槽里那一包的长度 */
static volatile uint8_t s_ready_mask;  /* bit0 = 槽0有包待取，bit1 = 槽1有包待取 */
static uint8_t  s_fill;        /* 正在填的槽号，只有多帧接收时有效 */
static uint8_t  s_take_slot;   /* 最近一次 Take 给的槽，Release 用它 */

/* ★ 2026-11-24 审计修：加到达序号，保证"先到先处理"
 * · s_ready_mask 只记"哪些槽有包"，不记到达顺序。原来 Take 固定优先槽 0，
 *   于是"槽 1 先到、槽 0 后到"时会先处理后到的那包 —— 对按块顺序写 Flash 的 OTA 是块序错乱
 * · 只有两包，比较序号就够；(int16_t)(a-b) < 0 处理 uint16 回绕
 * · 只在中断里写（收齐那一刻），Take 里读 */
static volatile uint16_t s_slot_seq[2];
static volatile uint16_t s_seq_next;

/* ★ 挑一个可用的接收槽
 * · 返回 0 或 1 表示可用，0xFF 表示两个槽都占着
 * · 正在填的那个槽也算占着，否则多帧收到一半会把自己覆盖掉
 * · 只在中断里调，所以不需要加临界区
 */
static uint8_t IsoTp_PickSlot(void)
{
	uint8_t busy = s_ready_mask;

	if (s_state == ISOTP_ST_RECV)
	{
		busy = (uint8_t)(busy | (uint8_t)(1u << s_fill));
	}

	if ((busy & 0x01u) == 0u)
	{
		return 0u;
	}

	if ((busy & 0x02u) == 0u)
	{
		return 1u;
	}

	return 0xFFu;
}

/* ★ 超时判据：连续多少次轮询"字节数没变"就认为对面掉线
 * · 不用时间戳，所以中断里一个时间函数都不用调，也不依赖 FreeRTOS 的 tick
 * · 代价是精度受轮询周期限制，判据由 ISOTP_POLL_MS 换算而来 */
static uint32_t s_progress_ms;      /* 上一次"字节数有进展"的时刻 */
static uint16_t s_got_last;         /* 上一轮轮询时的 s_got */
static volatile uint8_t s_progress_reset;  /* 1 = 刚收到首帧，超时要重新计时 */

/*==============================================================================
 *  ★ 发一个流控帧
 *
 *  ★ 谁会调它：
 *    · 收到首帧后回 CTS，告诉对面"可以开始发连续帧了"
 *    · 每收满 BS 个连续帧再回一次 CTS，这是给接收侧留的节流点
 *    · 报文太长或缓冲被占，回 OVFLW / WAIT 让对面停
 *
 *  ★ 可以在中断里调：只走 CAN_Transmit，不打印、不阻塞。
 *============================================================================*/
static void IsoTp_SendFc(uint8_t fs)
{
	uint8_t f[8];

	f[0] = (uint8_t)(ISOTP_PCI_FC | (fs & 0x0Fu));
	f[1] = (uint8_t)ISOTP_BS;
	f[2] = (uint8_t)ISOTP_STMIN;
	f[3] = 0u;
	f[4] = 0u;
	f[5] = 0u;
	f[6] = 0u;
	f[7] = 0u;

	if (CAN_TX_SendStd(ISOTP_ID_RSP, f, 8u) != 0u)
	{
		g_isotp_fc_tx++;
	}
}

/*==============================================================================
 *  ★ 中断里处理一帧
 *
 *  ★ 为什么全部在中断里做完：
 *    · 一帧最多搬 7 个字节，比"存队列 + 任务取"更省 RAM 也更省时间
 *    · F103 只有 20 KB RAM，开队列又要占一块，不划算
 *
 *  ★ 为什么 dlc 必须等于 8：
 *    本协议不裁短帧。少一个字节就凑不满 7 字节数据，硬解会错位。
 *    宁可丢掉并记一次 drop，也不猜。
 *============================================================================*/
static void IsoTp_OnFrame(uint16_t id, const uint8_t *d, uint8_t dlc)
{
	uint8_t i;
	uint16_t total;

	/* 只认自己那条 ID，其它报文（0x101/0x102/0x201）不归这里管 */
	if (id != ISOTP_ID_REQ)
	{
		return;
	}

	if (dlc < 8u)
	{
		IsoTp_NoteDrop(1u);
		return;
	}

	/* ---- 单帧：一帧装得下，直接就是一包完整报文 ---- */
	if ((d[0] & 0xF0u) == ISOTP_PCI_SF)
	{
		uint8_t len = (uint8_t)(d[0] & 0x0Fu);

		if ((len == 0u) || (len > 7u))
		{
			IsoTp_NoteDrop(2u);
			return;
		}

		/* 正在收多帧时来了单帧：协议不允许交错，丢新的 */
		if (s_state != ISOTP_ST_IDLE)
		{
			IsoTp_NoteDrop(3u);
			return;
		}

		/* 单帧也要挑槽（2026-11-24 改）
		 * 原来是"有包待取就丢"，现在只要另一块空着就收下 —— 这就是双缓冲的意义 */
		{
			uint8_t slot = IsoTp_PickSlot();

			if (slot == 0xFFu)
			{
				IsoTp_NoteDrop(3u);
				return;
			}

			for (i = 0u; i < len; i++)
			{
				s_buf[slot][i] = d[1u + i];
			}

			s_len[slot]  = (uint16_t)len;
			s_seq_next++;
			s_slot_seq[slot] = s_seq_next;
			s_ready_mask = (uint8_t)(s_ready_mask | (uint8_t)(1u << slot));
		}

		g_isotp_rx_sf++;

		return;
	}

	/* ---- 首帧：后面还有连续帧 ---- */
	if ((d[0] & 0xF0u) == ISOTP_PCI_FF)
	{
		total = (uint16_t)(((uint16_t)(d[0] & 0x0Fu) << 8) | (uint16_t)d[1]);

		/* 声明长度不合法，或者比缓冲还大：回 OVFLW 让对面停，自己回 IDLE */
		if ((total < 8u) || (total > ISOTP_RX_BUF))
		{
			IsoTp_NoteDrop(4u);
			IsoTp_SendFc(ISOTP_FC_OVFLW);
			s_state = ISOTP_ST_IDLE;
			return;
		}

		/* 已经有一个多帧包在收：协议不允许交错，丢新的 */
		if (s_state == ISOTP_ST_RECV)
		{
			IsoTp_NoteDrop(5u);
			return;
		}

		/* 挑一个空槽（2026-11-24 改）：两块都占着才丢 */
		{
			uint8_t slot = IsoTp_PickSlot();

			if (slot == 0xFFu)
			{
				IsoTp_NoteDrop(5u);
				return;
			}

			s_fill = slot;
		}

		/* 首帧自带 6 个数据字节 */
		for (i = 0u; i < 6u; i++)
		{
			s_buf[s_fill][i] = d[2u + i];
		}

		s_total     = total;
		s_got       = 6u;
		s_sn        = 1u;              /* 第一个连续帧序号是 1，不是 0 */
		s_bs_left        = (uint8_t)ISOTP_BS;
		s_got_last       = 6u;
		s_progress_reset = 1u;      /* ★ 让任务侧把超时重新计时（理由见 IsoTp_Poll） */
		s_state          = ISOTP_ST_RECV;

		IsoTp_SendFc(ISOTP_FC_CTS);

		return;
	}

	/* ---- 连续帧 ---- */
	if ((d[0] & 0xF0u) == ISOTP_PCI_CF)
	{
		uint8_t  sn = (uint8_t)(d[0] & 0x0Fu);
		uint16_t n;

		/* 没在收就收到连续帧：说明前面丢了首帧，直接丢 */
		if (s_state != ISOTP_ST_RECV)
		{
			IsoTp_NoteDrop(6u);
			return;
		}

		/* 序号对不上（丢帧 / 重帧 / 乱序）：整包作废
		 * 不做"按序号插回去"——CAN 是顺序总线，跳号就说明中间真丢了，
		 * 硬拼出来的包是不可信的，宁可让上层从头再来 */
		if (sn != s_sn)
		{
			IsoTp_NoteDrop(7u);
			s_state   = ISOTP_ST_IDLE;
			s_bs_left = 0u;
			return;
		}

		/* 这一帧实际能装几个字节：最后一帧可能不满 7 个 */
		n = (uint16_t)(s_total - s_got);
		if (n > 7u)
		{
			n = 7u;
		}

		for (i = 0u; i < n; i++)
		{
			s_buf[s_fill][s_got + i] = d[1u + i];
		}
		s_got = (uint16_t)(s_got + n);

		s_sn = (uint8_t)((s_sn + 1u) & 0x0Fu);

		/* 收齐了 */
		if (s_got >= s_total)
		{
			s_len[s_fill] = s_total;
			s_seq_next++;
			s_slot_seq[s_fill] = s_seq_next;
			s_ready_mask  = (uint8_t)(s_ready_mask | (uint8_t)(1u << s_fill));
			s_state       = ISOTP_ST_IDLE;
			s_bs_left     = 0u;
			g_isotp_rx_msg++;
			return;
		}

		/* 收满一块就再回一次流控，给接收侧一个节流点 */
		if (s_bs_left > 0u)
		{
			s_bs_left--;
		}
		if (s_bs_left == 0u)
		{
			s_bs_left = ISOTP_BS;
			IsoTp_SendFc(ISOTP_FC_CTS);
		}

		/* 有进展留到任务侧判：中断里只改 s_got，不碰时间也不碰 s_got_last */

		return;
	}

	/* ---- 流控帧 / 未知类型：我们不做多帧发送，没有可回的 ---- */
	if ((d[0] & 0xF0u) == ISOTP_PCI_FC)
	{
		return;
	}

	g_isotp_rx_drop++;
}

/*==============================================================================
 *  ★ CAN1 RX0 中断
 *
 *  ★ 为什么把 while 写成循环而不是只读一帧：
 *    一次中断到来时 FIFO 里可能已经排了 3 帧，只读一帧会留下老数据，
 *    下一次中断的时序就对不上了。
 *
 *  ★ 为什么这里不调任何 FreeRTOS 接口：
 *    不调就不会踩 configMAX_SYSCALL_INTERRUPT_PRIORITY 的限制，
 *    也就不需要为这个中断专门在 FreeRTOSConfig.h 里算优先级预算。
 *============================================================================*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg rx;
	uint8_t  d[8];
	uint8_t  i;

	while (CAN_MessagePending(CAN1, CAN_FIFO0) > 0u)
	{
		/* 读邮箱本身就把中断标志清掉了，不用另外 CAN_ClearITPendingBit */
		CAN_Receive(CAN1, CAN_FIFO0, &rx);

		for (i = 0u; i < 8u; i++)
		{
			d[i] = rx.Data[i];
		}

		IsoTp_OnFrame(rx.StdId, d, (uint8_t)rx.DLC);

		/* ★ 2026-11-24 加：命令帧走另一条路
		 * · 0x201 不是 ISO-TP 报文，是 8 字节裸帧，IsoTp_OnFrame 会直接丢掉
		 * · 放在它后面调，两者靠 ID 分流，谁也抢不到谁 */
		CAN_CmdOnFrame(rx.StdId, d, (uint8_t)rx.DLC);
	}
}

/*==============================================================================
 *  ★ 初始化：打开 FIFO0 中断
 *============================================================================*/
void IsoTp_Init(void)
{
	NVIC_InitTypeDef nvic;

	s_state      = ISOTP_ST_IDLE;
	s_ready_mask  = 0u;
	s_fill        = 0u;
	s_take_slot   = 0u;
	s_slot_seq[0] = 0u;
	s_slot_seq[1] = 0u;
	s_seq_next    = 0u;
	s_got        = 0u;
	s_progress_ms    = 0u;
	s_got_last       = 0u;
	s_progress_reset = 0u;

	/* FIFO0 有待收报文就进中断 */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

	/* ★ 优先级 6：比采集任务低，比串口打印高，和现有中断不打架
	 *   · 本中断不调 FreeRTOS 接口，所以不占用 configMAX_SYSCALL 的名额 */
	nvic.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 6u;
	nvic.NVIC_IRQChannelSubPriority        = 0u;
	nvic.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&nvic);
}

/*==============================================================================
 *  ★ 取一包收完的报文，直接把内部缓冲的指针给出去
 *
 *  ★ 为什么不拷贝：
 *    上层（UDS）如果自己再开一块 288 字节的缓冲，这块钱在 F103 的 20 KB 里
 *    白花，而数据本来就在接收缓冲里放着，拷一遍只为"看起来封装得好"不值。
 *
 *  ★ 代价：调用者必须用完就 Release
 *    · 不 Release，那个槽会一直被占着，两个槽都占满时下一包才会被挡掉
 *    · 2026-11-24 改成双缓冲后，处理一块的工夫另一块还能接新包，
 *      不再出现"响应已经发出、缓冲还占着、新包被整包丢掉"那个窗口
 *============================================================================*/
const uint8_t *IsoTp_Take(uint16_t *len)
{
	uint8_t slot;

	if (s_ready_mask == 0u)
	{
		return 0;
	}

	/* ★ 2026-11-24 审计修：按到达序号取，不再固定优先槽 0
	 * · 原来槽 1 先到、槽 0 后到时，会先处理后到的那包 —— OTA 块序会乱
	 * · 两包都在时比序号；只有一包时直接取它 */
	if ((s_ready_mask & 0x03u) == 0x03u)
	{
		slot = ((int16_t)(s_slot_seq[0] - s_slot_seq[1]) < 0) ? 0u : 1u;
	}
	else
	{
		slot = ((s_ready_mask & 0x01u) != 0u) ? 0u : 1u;
	}
	s_take_slot = slot;

	/* ★ 2026-11-24 加：把这一包滚进两级快照，供 UDS 0x22 读
	 * · 记"上一条"而不是"最后一条"：读 0x22 的请求本身也是一个 ISO-TP 包，
	 *   会把它顶掉；只有"上一条"才是被测的那一包
	 * · 为什么不用串口打印当判据：那行要在 task_KEY 里忙等约 8 ms，
	 *   而且回归会被迫依赖"设备打了什么字"，观测通道一换就断
	 * · CRC 每包都算：258 字节约 2300 次循环，10 包每秒只占 0.2% CPU */
	g_isotp_prev_len = s_done_len;
	g_isotp_prev_crc = s_done_crc;
	s_done_len = s_len[slot];
	s_done_crc = Crc16_Kermit(s_buf[slot], (uint32_t)s_len[slot]);

#if (TEST_ISO_TP_DUMP != 0)
	{
		uint16_t crc = s_done_crc;      /* 上面已算过，不重复算 */
		uint16_t i;

		/* 只打长度 + CRC16 + 头 16 字节，不打 288 字节全量：
		 * 全量十六进制在 115200 下要 50 ms，会把 PROC 那一拍拖长 */
		Serial_Printf("[ISOTP] n=%u crc=%04X head=", (unsigned)s_len[slot], (unsigned)crc);

		for (i = 0u; (i < s_len[slot]) && (i < 16u); i++)
		{
			Serial_Printf("%02X ", (unsigned)s_buf[slot][i]);
		}

		Serial_Printf("\r\n");
	}
#endif

	if (len != 0)
	{
		*len = s_len[slot];
	}

	return s_buf[slot];
}

/* ★ 2026-11-24 审计修：读改写必须互斥
 * · s_ready_mask 是 uint8_t，&= 会编译成 LDRB / BIC / STRB 三步。
 *   中断若在 LDRB 与 STRB 之间把另一槽置位，任务的 STRB 会覆盖掉那个新位，
 *   那一包永远不会被 Take，而 IsoTp_PickSlot 又认为该槽空闲 —— 后续新包直接盖掉它的数据
 * · 关中断只有几条指令，对 CAN 收发的影响在纳秒级
 */
void IsoTp_Release(void)
{
	__disable_irq();
	s_ready_mask = (uint8_t)(s_ready_mask & (uint8_t)~(uint8_t)(1u << s_take_slot));
	__enable_irq();
}

/*==============================================================================
 *  ★ 发一个单帧
 *============================================================================*/
uint8_t IsoTp_SendSf(uint16_t id, const uint8_t *data, uint8_t len)
{
	uint8_t f[8];
	uint8_t i;

	if ((len == 0u) || (len > 7u) || (data == 0))
	{
		return 0u;
	}

	f[0] = (uint8_t)(ISOTP_PCI_SF | len);
	for (i = 0u; i < len; i++)
	{
		f[1u + i] = data[i];
	}
	for (i = (uint8_t)(1u + len); i < 8u; i++)
	{
		f[i] = 0u;                   /* 没用到的字节填 0，不留旧数据 */
	}

	return CAN_TX_SendStd(id, f, 8u);
}

/*==============================================================================
 *  ★ 任务侧轮询：只做超时检查
 *
 *  ★ 为什么取包不在这里做：
 *    取包是上层（UDS）的事，传输层只负责"把一包攒好、把超时管好"。
 *    两件事分开之后，上层想什么时候取就什么时候取，不用迁就轮询节拍。
 *============================================================================*/
void IsoTp_Poll(void)
{
#if (TEST_ISO_TP_DUMP != 0)
	/* ★ 丢包现场取证（2026-11-23 加）
	 * · 只在计数变化时打一行，不刷屏
	 * · 打的是"为什么丢"：why 的含义见 iso_tp.h
	 * · 有这一行才能在传整包失败时判断是分帧层丢了，还是 UDS 层没回 */
	{
		static uint32_t last_drop = 0u;

		if (g_isotp_rx_drop != last_drop)
		{
			last_drop = g_isotp_rx_drop;

			uint8_t k;
			uint8_t n_show = (g_isotp_why_cnt < 4u) ? g_isotp_why_cnt : 4u;

			Serial_Printf("[ISOTP] DROP n=%lu state=%u got=%u/%u ready=%u why=",
			              (unsigned long)g_isotp_rx_drop,
			              (unsigned)s_state,
			              (unsigned)s_got,
			              (unsigned)s_total,
			              (unsigned)s_ready_mask);

			/* 把最近几次的原因都列出来（老的在前），一次事件的两条都能看见 */
			for (k = 0u; k < n_show; k++)
			{
				uint8_t idx = (uint8_t)((g_isotp_why_cnt - n_show + k) & 0x03u);

				Serial_Printf("%u ", (unsigned)g_isotp_why_ring[idx]);
			}

			Serial_Printf("\r\n");
		}
	}
#endif

	uint32_t now = (uint32_t)xTaskGetTickCount();

	if (s_state != ISOTP_ST_RECV)
	{
		return;
	}

	/* 新的一包刚开始：把"最后一次有进展"的时刻重置到现在
	 * ★ 用中断置的标志来做，而不是比较 got 有没有变：
	 *   上一次报文如果正好中止在 got=6，新首帧又把 got 设成 6，
	 *   只比数值会漏判，于是这条新报文一上来就被判超时 */
	if (s_progress_reset != 0u)
	{
		s_progress_reset = 0u;
		s_progress_ms    = now;
		s_got_last       = s_got;
		return;
	}

	if (s_got != s_got_last)
	{
		s_got_last    = s_got;
		s_progress_ms = now;
		return;
	}

	if ((now - s_progress_ms) > (uint32_t)ISOTP_N_CR_MS)
	{
		/* 等到一半没动静：整包作废，回 IDLE 等下一轮
		 * 中断里不管这个，因为中断只知道"又收到一帧"，
		 * 判断"多久没收到"要跨时间看，那只有任务侧能做 */
		s_state   = ISOTP_ST_IDLE;
		s_bs_left = 0u;
		IsoTp_NoteDrop(8u);
	}
}

#endif /* CAN_EN */
