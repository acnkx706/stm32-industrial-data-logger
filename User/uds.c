/**
  ******************************************************************************
  * @file    uds.c
  * @brief   UDS 诊断服务层（ISO 14229 子集）—— CAN-OTA 的业务层
  ******************************************************************************
  * ★ 做什么
  *   收一条完整请求，按服务号分派，回一条肯定或否定响应。
  *   本周只做三个：0x10 会话控制、0x3E 保活、0x11 复位（受会话限制）。
  *   其余服务先回"不支持"，等下一周逐个填进来。
  *
  * ★ 为什么不一次写完
  *   10/3E/11 全是纯内存操作，错了也不会弄坏任何东西；31/34/36/37 会碰 W25Q64
  *   暂存区，一旦写坏要擦整片才能恢复。先让前三个跑通，把"分派 + 响应 + 超时"
  *   这套骨架验证掉，后面每个服务都只是往表里加一格。
  *
  * ★ 影响什么
  *   0x28 关掉周期报文之后，网关会看不到数据；5 秒不发 3E 会自动恢复。
  *   0x11 复位只在编程会话里允许，避免记录途中把文件写坏。
  *
  * ★ 怎么验证
  *   PC 发 3E 00 -> 7E 00；发 10 03 -> 50 03 00 32 13 88；
  *   发 22 F1 90 -> 7F 22 11；发 11 01（默认会话）-> 7F 11 7F。
  ******************************************************************************
  */

#include "uds.h"

#if (CAN_EN != 0)

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

#include "iso_tp.h"
#include "MyRTC.h"        /* 种子的熵源之一：RTC 秒数 */
#include "ota_stage.h"    /* 暂存区擦除与校验（0x31 用） */
#include "boot_flag.h"    /* 37 收尾时置升级标志（BOOT_FLAG_DR / MAGIC） */
#include "Serial.h"
#include "ProbeConfig.h"

volatile uint32_t g_uds_req_cnt    = 0u;
volatile uint32_t g_uds_nrc_cnt    = 0u;
volatile uint32_t g_uds_s3_timeout = 0u;

static uint8_t  s_session;        /* 当前会话，见 uds.h 的三个 UDS_SESSION_xxx */
static uint32_t s_s3_tick;        /* 上一次收到请求的时刻（毫秒） */
static uint8_t  s_comm_off;       /* 1 = 0x28 让我们停掉周期报文 */
static uint8_t  s_dtc_off;        /* 1 = 0x85 让我们停掉故障记录 */

/* ---- 0x27 安全访问的状态 ---- */
static uint8_t  s_seed[4];        /* 上一次发出去的种子 */
static uint8_t  s_seed_ok;        /* 1 = 发过种子，正等密钥 */
static uint8_t  s_unlocked;       /* 1 = 已通过安全访问 */
static uint8_t  s_fail_cnt;       /* 连续送错密钥的次数 */
static uint32_t s_lock_tick;      /* 锁到哪个时刻（毫秒），0 = 没锁 */
static uint32_t s_seed_seq;       /* 种子里的递增计数器，保证同一秒内也不同 */

/* ---- 34/36/37 下载状态 ---- */
static uint8_t  s_dl_active;      /* 1 = 34 已受理、37 还没来 */
static uint32_t s_dl_size;        /* 34 里声明的整包长度 */
static uint32_t s_dl_off;         /* 已经收进来多少字节 */
static uint16_t s_dl_crc;         /* CRC-A：逐块累加，用来发现传输错误 */
/* ★ 2026-11-24 加：0x22 的实现定义在文件末尾，这里先声明 */
static void Uds_Service_ReadDid(const uint8_t *d, uint16_t n);
static uint8_t  s_dl_seq;         /* 下一个期望的块序号（1 起，0xFF 后回 0x00） */

/* ★ 2026-11-23：这里原来还有个 s_dtc_off（0x85 用），先删掉
 * · 变量只写不读，编译器直接给 #550-D 警告
 * · 等下周真做 0x85 时再加回来，那时候它会被读到 */

#if (TEST_ISO_TP_DUMP != 0)
/*==============================================================================
 *  ★ 测试期把每条收/发都打出来
 *
 *  ★ 为什么要打发送方向：
 *    PC 只能往总线上发帧，看不到回程（F407 那条命令通道目前只发不收）。
 *    所以验证响应只能看这台设备的串口。数据本身就是"一条报文"，打出来正好对照。
 *
 *  ★ 正式版必须关（TEST_ISO_TP_DUMP = 0）：一行约 30 字符，115200 下约 3 ms
 *============================================================================*/
static void Uds_Dump(const char *dir, const uint8_t *d, uint8_t n)
{
	uint8_t i;

	Serial_Printf("[UDS] %s ", dir);

	for (i = 0u; i < n; i++)
	{
		Serial_Printf("%02X ", (unsigned)d[i]);
	}

	Serial_Printf("\r\n");
}
#endif

/*==============================================================================
 *  ★ 回肯定响应
 *
 *  ★ 规则：响应的第一个字节永远是 SID + 0x40，第二个字节原样回子功能
 *    · 50 = 10 + 40，7E = 3E + 40，51 = 11 + 40
 *  ★ 为什么最多只带 7 个字节：
 *    ISO-TP 的发送方向只实现了单帧，一帧 7 个数据字节。
 *    UDS 的响应确实都短，这条限制目前不会碰到。
 *============================================================================*/
static void Uds_Positive(uint8_t sid, const uint8_t *data, uint8_t n)
{
	uint8_t r[7];
	uint8_t i;

	if (n > 6u)
	{
		n = 6u;
	}

	r[0] = (uint8_t)(sid + 0x40u);
	for (i = 0u; i < n; i++)
	{
		r[1u + i] = data[i];
	}

	(void)IsoTp_SendSf(ISOTP_ID_RSP, r, (uint8_t)(1u + n));

#if (TEST_ISO_TP_DUMP != 0)
	Uds_Dump("TX+", r, (uint8_t)(1u + n));
#endif
}

/*==============================================================================
 *  ★ 回否定响应：0x7F + 请求的 SID + 否定码
 *
 *  ★ 为什么必须把 SID 一起回：
 *    Tester 可能同时挂着好几条请求，只回一个否定码它不知道是哪条失败了。
 *============================================================================*/
static void Uds_Negative(uint8_t sid, uint8_t nrc)
{
	uint8_t r[3];

	r[0] = 0x7Fu;
	r[1] = sid;
	r[2] = nrc;

	(void)IsoTp_SendSf(ISOTP_ID_RSP, r, 3u);

	g_uds_nrc_cnt++;

#if (TEST_ISO_TP_DUMP != 0)
	Uds_Dump("TX-", r, 3u);
#endif
}

/*==============================================================================
 *  ★ 0x10 诊断会话控制
 *
 *  ★ 响应格式：50 <子功能> <P2 高> <P2 低> <P2* 高> <P2* 低>
 *    · P2 和 P2* 是告诉 Tester "我多久会给响应"，必须和真实行为一致
 *    · 报小了 Tester 会提前超时，报大了它白等
 *
 *  ★ 为什么切会话不改别的状态：
 *    会话只是一个"允许做什么"的门槛，真正的门槛检查放在各服务入口，
 *    集中在一处判断比散在各处可靠。
 *============================================================================*/
static void Uds_Service_Session(const uint8_t *d, uint16_t n)
{
	uint8_t sf;
	uint8_t r[5];

	if (n < 2u)
	{
		Uds_Negative(UDS_SID_SESSION, UDS_NRC_INCORRECT_LENGTH);
		return;
	}

	sf = (uint8_t)(d[1] & 0x7Fu);      /* 最高位是"不要回响应"，先掩掉 */

	if ((sf != UDS_SESSION_DEFAULT) &&
	    (sf != UDS_SESSION_PROGRAM) &&
	    (sf != UDS_SESSION_EXTENDED))
	{
		Uds_Negative(UDS_SID_SESSION, UDS_NRC_SUBFUNC_NOT_SUPPORTED);
		return;
	}

	s_session = sf;
	s_s3_tick = xTaskGetTickCount();

	/* ★ 切会话必须清安全状态（ISO 14229 的要求，也是实测逼出来的）
	 *   · 回默认会话要连"已解锁"一起清：否则一次拿到的高权限会一直跟着，越权
	 *   · 任何切换都要清掉待用的种子：跨会话的种子不该还能接着用
	 *   · 这条是 2026-11-23 测出来的：脚本重复跑时残留的种子让第二次
	 *     "未求种子先送密钥" 从 0x24 变成了 0x35，看着像密钥校验错了 */
	s_seed_ok = 0u;
	s_dl_active = 0u;

	if (sf == UDS_SESSION_DEFAULT)
	{
		s_unlocked = 0u;
		s_fail_cnt = 0u;
	}

	r[0] = sf;
	r[1] = (uint8_t)((UDS_P2_MS >> 8) & 0xFFu);
	r[2] = (uint8_t)(UDS_P2_MS & 0xFFu);
	r[3] = (uint8_t)((UDS_P2STAR_MS >> 8) & 0xFFu);
	r[4] = (uint8_t)(UDS_P2STAR_MS & 0xFFu);

	/* 子功能最高位置了 1 就是"别回响应"，工业上叫 suppressPositiveResponse */
	if ((d[1] & 0x80u) == 0u)
	{
		Uds_Positive(UDS_SID_SESSION, r, 5u);
	}
}

/*==============================================================================
 *  ★ 造一个种子
 *
 *  ★ 熵源用两个：
 *    · RTC 的秒数：跨次上电都不同
 *    · 一个递增计数器：保证同一秒内连求两次也不一样
 *    没有计数器的话，同一秒内的两次请求会拿到同一个种子，重放就成立了。
 *
 *  ★ 不是密码学随机
 *    这里只做乘法散列，不是 CSPRNG。安全性靠"种子一次性 + 三次锁定"，
 *    不靠种子的不可预测性。这个设备要防的是产线误刷，不是定向攻击。
 *============================================================================*/
static void Uds_MakeSeed(uint8_t *s)
{
    uint32_t t;
    uint32_t x;

    s_seed_seq++;

    t = (uint32_t)MyRTC_GetUnix();
    if (t == 0u)
    {
        /* RTC 还没配好时退回系统节拍，至少保证每次不同 */
        t = (uint32_t)xTaskGetTickCount();
    }

    x = t ^ (s_seed_seq * 2654435761u);
    x ^= (x >> 13);
    x *= 0x5BD1E995u;
    x ^= (x >> 15);

    s[0] = (uint8_t)(x & 0xFFu);
    s[1] = (uint8_t)((x >> 8) & 0xFFu);
    s[2] = (uint8_t)((x >> 16) & 0xFFu);
    s[3] = (uint8_t)((x >> 24) & 0xFFu);
}

/*==============================================================================
 *  ★ 由种子算密钥（Tester 那边必须用同一套算法）
 *
 *  ★ 这不是密码学
 *    两轮"异或 + 循环移位 + 相加"的混合，作用是让"改种子的一个字节"扩散到
 *    密钥的全部四个字节，不至于一眼看出关系。
 *    ⚠️ 它挡不住真正会反汇编的人。要防篡改得上签名验签（非对称算法），
 *       那是另一个量级的工程，也不在这个项目的目标里。
 *============================================================================*/
static void Uds_CalcKey(const uint8_t *seed, uint8_t *key)
{
    uint16_t x = (uint16_t)((uint16_t)seed[0] | ((uint16_t)seed[1] << 8));
    uint16_t y = (uint16_t)((uint16_t)seed[2] | ((uint16_t)seed[3] << 8));
    uint16_t a;
    uint16_t b;
    uint8_t  i;

    for (i = 0u; i < 2u; i++)
    {
        x = (uint16_t)(x ^ 0x5A3Cu);
        y = (uint16_t)(y ^ 0xA5C3u);

        a = (uint16_t)((uint16_t)(x << 5) | (uint16_t)(x >> 11));
        b = (uint16_t)((uint16_t)(y << 5) | (uint16_t)(y >> 11));

        x = (uint16_t)(a + y);
        y = (uint16_t)(b ^ x);
    }

    key[0] = (uint8_t)(x & 0xFFu);
    key[1] = (uint8_t)((x >> 8) & 0xFFu);
    key[2] = (uint8_t)(y & 0xFFu);
    key[3] = (uint8_t)((y >> 8) & 0xFFu);
}

/*==============================================================================
 *  ★ 0x27 安全访问（种子-密钥）
 *
 *  ★ 流程：27 01 求种子 -> 67 01 <4字节> -> 27 02 <4字节密钥> -> 67 02
 *    · 子功能奇数 = 求种子，偶数 = 送密钥，这是标准约定
 *    · 种子一次性：用过一次就作废，防止重放
 *    · 连错 3 次锁 10 分钟，锁定期内连求种子也拒
 *
 *  ★ 为什么"没求种子就送密钥"要单独报 0x24
 *    如果不查这一步，Tester 可能拿上一次残留的种子算密钥送进来，
 *    看起来"通了"，其实中间错过了一个步骤，出问题时查不出来。
 *============================================================================*/
static void Uds_Service_Security(const uint8_t *d, uint16_t n)
{
    uint8_t  sf;
    uint8_t  r[5];
    uint8_t  key[4];
    uint32_t now;
    uint8_t  i;
    uint8_t  same;

    if (n < 2u)
    {
        Uds_Negative(UDS_SID_SECURITY, UDS_NRC_INCORRECT_LENGTH);
        return;
    }

    sf  = (uint8_t)(d[1] & 0x7Fu);
    now = xTaskGetTickCount();

    /* ---- 锁定期内什么都不许 ---- */
    if (s_lock_tick != 0u)
    {
        if ((int32_t)(now - s_lock_tick) < 0)
        {
            Uds_Negative(UDS_SID_SECURITY, UDS_NRC_EXCEED_ATTEMPTS);
            return;
        }

        /* 到期：解锁并把失败计数清零，重新给三次机会 */
        s_lock_tick = 0u;
        s_fail_cnt  = 0u;
    }

    /* ---- 27 01 求种子 ---- */
    if (sf == 0x01u)
    {
        if (n != 2u)
        {
            Uds_Negative(UDS_SID_SECURITY, UDS_NRC_INCORRECT_LENGTH);
            return;
        }

        if (s_unlocked != 0u)
        {
            /* ★ 已经解锁就回全 0 种子，这是标准里"我已经开了"的信号
             *   · 不这么回的话 Tester 会以为还要再来一轮，白白多一轮握手 */
            r[0] = 0x01u;
            r[1] = 0u; r[2] = 0u; r[3] = 0u; r[4] = 0u;
            Uds_Positive(UDS_SID_SECURITY, r, 5u);
            return;
        }

        Uds_MakeSeed(s_seed);
        s_seed_ok = 1u;

        r[0] = 0x01u;
        r[1] = s_seed[0];
        r[2] = s_seed[1];
        r[3] = s_seed[2];
        r[4] = s_seed[3];
        Uds_Positive(UDS_SID_SECURITY, r, 5u);
        return;
    }

    /* ---- 27 02 送密钥 ---- */
    if (sf == 0x02u)
    {
        if (n != 6u)
        {
            Uds_Negative(UDS_SID_SECURITY, UDS_NRC_INCORRECT_LENGTH);
            return;
        }

        if (s_seed_ok == 0u)
        {
            Uds_Negative(UDS_SID_SECURITY, UDS_NRC_REQUEST_SEQUENCE_ERROR);
            return;
        }

        /* 种子用过就作废，无论对不对 */
        s_seed_ok = 0u;

        Uds_CalcKey(s_seed, key);

        same = 1u;
        for (i = 0u; i < 4u; i++)
        {
            if (d[2u + i] != key[i])
            {
                same = 0u;
                break;
            }
        }

        if (same != 0u)
        {
            s_unlocked = 1u;
            s_fail_cnt = 0u;
            r[0] = 0x02u;
            Uds_Positive(UDS_SID_SECURITY, r, 1u);
            return;
        }

        /* 错了：记账，满三次就锁 */
        s_fail_cnt++;

        if (s_fail_cnt >= 3u)
        {
            s_lock_tick = now + (uint32_t)UDS_LOCK_MS;
            Uds_Negative(UDS_SID_SECURITY, UDS_NRC_EXCEED_ATTEMPTS);
            return;
        }

        Uds_Negative(UDS_SID_SECURITY, UDS_NRC_INVALID_KEY);
        return;
    }

    Uds_Negative(UDS_SID_SECURITY, UDS_NRC_SUBFUNC_NOT_SUPPORTED);
}

/*==============================================================================
 *  ★ 0x28 通信控制：让总线安静下来
 *
 *  ★ 请求：28 <子功能> <通信类型>
 *    · 子功能 00 允许收发 / 01 只收 / 02 只发 / 03 收发全停
 *    · 通信类型 bit0 = 普通报文，bit1 = 网络管理报文
 *
 *  ★ 为什么刷写前必须关：
 *    本设备每 100 ms 发两帧业务数据（0x101/0x102），刷写时这些帧会和固件
 *    数据抢总线。500 kbps 下 20 帧/秒只占 5% 带宽，看起来无所谓，
 *    但真正的问题是"一边修路一边放车"——升级中途业务帧插进来，
 *    排查问题时很难分清是传输错了还是业务帧挤的。
 *
 *  ★ 为什么只支持通信类型 0x01：
 *    本设备没有网络管理报文，报别的类型没有意义。宁可直接拒绝，
 *    也不要"接受但什么都不做"——那会让 Tester 以为已经生效了。
 *============================================================================*/
static void Uds_Service_CommControl(const uint8_t *d, uint16_t n)
{
	uint8_t sf;
	uint8_t type;

	if (n != 3u)
	{
		Uds_Negative(UDS_SID_COMM_CTRL, UDS_NRC_INCORRECT_LENGTH);
		return;
	}

	sf   = (uint8_t)(d[1] & 0x7Fu);
	type = d[2];

	if (sf > 0x03u)
	{
		Uds_Negative(UDS_SID_COMM_CTRL, UDS_NRC_SUBFUNC_NOT_SUPPORTED);
		return;
	}

	if (type != 0x01u)
	{
		/* 只认"普通报文"这一种，别的类型直接说越界 */
		Uds_Negative(UDS_SID_COMM_CTRL, UDS_NRC_REQUEST_OUT_OF_RANGE);
		return;
	}

	/* ★ 默认会话里不许关总线
	 *   · 关掉周期报文意味着网关那边会判定"设备离线"，这是危险操作
	 *   · 标准也要求 28 服务在非默认会话里才允许 */
	if (s_session == UDS_SESSION_DEFAULT)
	{
		Uds_Negative(UDS_SID_COMM_CTRL, UDS_NRC_SERVICE_NOT_IN_SESSION);
		return;
	}

	/* 0x00 允许收发 / 0x01 只收 / 0x02 只发 / 0x03 全停
	 * 只有 0x03 会真正停掉我们的周期发送；其余三种都算"允许发" */
	s_comm_off = (sf == 0x03u) ? 1u : 0u;

	if ((d[1] & 0x80u) == 0u)
	{
		uint8_t echo = sf;

		Uds_Positive(UDS_SID_COMM_CTRL, &echo, 1u);
	}
}

/*==============================================================================
 *  ★ 0x85 故障记录开关
 *
 *  ★ 请求：85 <子功能>，01 = 打开记录，02 = 关闭记录
 *
 *  ★ 为什么不关就会出问题：
 *    刷写期间设备不采集、SD 也停了，本工程的故障统计会把"数据流断了"
 *    记成故障，写一堆假的故障记录到卡里。刷完一看，全是升级当天的假故障。
 *
 *  ★ 和 0x28 同一个前提：默认会话不许改，因为改完忘了恢复会影响正常记录
 *============================================================================*/
static void Uds_Service_ControlDtc(const uint8_t *d, uint16_t n)
{
	uint8_t sf;

	if (n != 2u)
	{
		Uds_Negative(UDS_SID_CONTROL_DTC, UDS_NRC_INCORRECT_LENGTH);
		return;
	}

	if (s_session == UDS_SESSION_DEFAULT)
	{
		Uds_Negative(UDS_SID_CONTROL_DTC, UDS_NRC_SERVICE_NOT_IN_SESSION);
		return;
	}

	sf = (uint8_t)(d[1] & 0x7Fu);

	if (sf == 0x01u)
	{
		s_dtc_off = 0u;
	}
	else if (sf == 0x02u)
	{
		s_dtc_off = 1u;
	}
	else
	{
		Uds_Negative(UDS_SID_CONTROL_DTC, UDS_NRC_SUBFUNC_NOT_SUPPORTED);
		return;
	}

	if ((d[1] & 0x80u) == 0u)
	{
		uint8_t echo = sf;

		Uds_Positive(UDS_SID_CONTROL_DTC, &echo, 1u);
	}
}

/*==============================================================================
 *  ★ 0x31 例程控制：擦除暂存区 / 校验暂存镜像
 *
 *  ★ 请求：31 <子功能> <例程ID 高> <例程ID 低>
 *    子功能 01 启动 / 02 停止 / 03 查结果；本实现只支持 01
 *    例程 FF00 = 擦除暂存区，0202 = 校验暂存镜像
 *  ★ 响应：71 <子功能> <例程ID 高> <例程ID 低> <状态>
 *
 *  ★ 为什么必须先回 0x78
 *    擦 27 个扇区要 1.2~10.8 秒，而 UDS 的 P2 只有 50 ms。
 *    不先回一句"我在忙"，Tester 50 ms 就判超时，整个流程失败。
 *
 *  ★ 为什么擦到一半还要再回 0x78
 *    Tester 收到 0x78 之后只放宽到 P2* = 5 秒。27 个扇区最坏 10.8 秒，
 *    所以每擦一批就补一句，保证任何时刻都不会等超过 5 秒。
 *
 *  ⚠️ 已知代价：擦除期间 PROC 被阻塞（最长约 10 秒），采集任务会因为
 *     队列写满而丢这段时间的数据。正式产品应当把擦除挪到独立低优先级
 *     任务，并在升级前显式暂停采集；本版先接受这个代价，因为升级本身
 *     就是一次计划内的停机。
 *============================================================================*/
static void Uds_Service_Routine(const uint8_t *d, uint16_t n)
{
	uint8_t  sf;
	uint16_t rid;
	uint8_t  r[4];
	uint8_t  status;
	uint8_t  left;

	if (n < 4u)
	{
		Uds_Negative(UDS_SID_ROUTINE, UDS_NRC_INCORRECT_LENGTH);
		return;
	}

	sf  = (uint8_t)(d[1] & 0x7Fu);
	rid = (uint16_t)(((uint16_t)d[2] << 8) | (uint16_t)d[3]);

	if (sf != 0x01u)
	{
		Uds_Negative(UDS_SID_ROUTINE, UDS_NRC_SUBFUNC_NOT_SUPPORTED);
		return;
	}

	/* ★ 擦 Flash 是危险操作，必须先过安全访问 */
	if (s_unlocked == 0u)
	{
		Uds_Negative(UDS_SID_ROUTINE, UDS_NRC_SECURITY_DENIED);
		return;
	}

	if (rid == 0xFF00u)
	{
		/* ---- 擦除暂存区 ---- */
		Uds_Negative(UDS_SID_ROUTINE, UDS_NRC_RESPONSE_PENDING);

		/* 顺序：先让头失效，再擦镜像区（理由见 OtaStage_InvalidateHdr） */
		if ((OtaStage_InvalidateHdr() != 0u) || (OtaStage_EraseStart() != 0u))
		{
			status = OTA_STATUS_NO_W25;
		}
		else
		{
			left = OtaStage_EraseMore(5u);

			while (left > 0u)
			{
				Uds_Negative(UDS_SID_ROUTINE, UDS_NRC_RESPONSE_PENDING);
				left = OtaStage_EraseMore(5u);
			}

			status = OTA_STATUS_OK;
		}
	}
	else if (rid == 0x0202u)
	{
		/* ---- 校验暂存镜像 ---- */
		Uds_Negative(UDS_SID_ROUTINE, UDS_NRC_RESPONSE_PENDING);
		status = OtaStage_CheckIntegrity();
	}
	else
	{
		Uds_Negative(UDS_SID_ROUTINE, UDS_NRC_REQUEST_OUT_OF_RANGE);
		return;
	}

	r[0] = sf;
	r[1] = d[2];
	r[2] = d[3];
	r[3] = status;

	Uds_Positive(UDS_SID_ROUTINE, r, 4u);
}

/*==============================================================================
 *  ★ 0x34 请求下载：协商块长，进入"准备收数据"状态
 *
 *  ★ 请求：34 <数据格式> <地址长度格式> <地址 4 字节> <长度 4 字节>
 *    · 数据格式 00 = 不压缩（本实现不支持压缩，报 0x31）
 *    · 地址长度格式 44 = 4 字节地址 + 4 字节长度（本实现只认这一种）
 *  ★ 响应：74 <长度格式> <块长 2 字节>
 *    · 74 20 01 02 → 0x0102 = 258，即"每块最多 258 字节"
 *    · 258 = 2（SID + 块序号）+ 256（正好一个 W25 页）
 *
 *  ★ 地址字段为什么忽略：
 *    本实现的落点是固定的暂存区（W25Q64 0x7E1000），不由 Tester 指定。
 *    收下但不校验，是为了兼容那些习惯填 App 基地址的工具。
 *============================================================================*/
static void Uds_Service_RequestDownload(const uint8_t *d, uint16_t n)
{
	uint32_t size;
	uint8_t  r[3];

	if (n != 11u)
	{
		Uds_Negative(UDS_SID_REQ_DOWNLOAD, UDS_NRC_INCORRECT_LENGTH);
		return;
	}

	if (d[1] != 0x00u)
	{
		Uds_Negative(UDS_SID_REQ_DOWNLOAD, UDS_NRC_REQUEST_OUT_OF_RANGE);
		return;
	}

	if (d[2] != 0x44u)
	{
		Uds_Negative(UDS_SID_REQ_DOWNLOAD, UDS_NRC_REQUEST_OUT_OF_RANGE);
		return;
	}

	if (s_session != UDS_SESSION_PROGRAM)
	{
		Uds_Negative(UDS_SID_REQ_DOWNLOAD, UDS_NRC_SERVICE_NOT_IN_SESSION);
		return;
	}

	if (s_unlocked == 0u)
	{
		Uds_Negative(UDS_SID_REQ_DOWNLOAD, UDS_NRC_SECURITY_DENIED);
		return;
	}

	/* 长度字段是大端 4 字节（UDS 全线大端，和我们自己 CAN 帧的小端相反） */
	size = ((uint32_t)d[7] << 24) | ((uint32_t)d[8] << 16) |
	       ((uint32_t)d[9] << 8)  | (uint32_t)d[10];

	if ((size == 0u) || (size > (uint32_t)OTA_STAGE_IMG_MAX))
	{
		Uds_Negative(UDS_SID_REQ_DOWNLOAD, UDS_NRC_REQUEST_OUT_OF_RANGE);
		return;
	}

	s_dl_size   = size;
	s_dl_off    = 0u;
	s_dl_crc    = 0u;
	s_dl_seq    = 1u;
	s_dl_active = 1u;

	r[0] = 0x20u;      /* 高 4 位 = 块长字段占几个字节，2 个 */
	r[1] = 0x01u;
	r[2] = 0x02u;      /* 0x0102 = 258 */

	Uds_Positive(UDS_SID_REQ_DOWNLOAD, r, 3u);
}

/*==============================================================================
 *  ★ 0x36 传输数据：收一块、写一页、累加 CRC
 *
 *  ★ 请求：36 <块序号> <数据...>
 *  ★ 响应：76 <块序号>
 *
 *  ★ 块序号的两条规则（背下来，面试常问）：
 *    · 从 0x01 开始，不是 0x00
 *    · 到 0xFF 之后绕回 0x00，不是 0x01
 *    本实现用 uint8_t 自增，天然满足这两条。
 *
 *  ★ 为什么块序号错了要整包拒收而不是补发：
 *    CAN 是顺序总线，跳号说明中间真丢了。硬拼出来的镜像 CRC 一定对不上，
 *    但那时候已经写了半张 Flash，浪费的是擦除时间。早拒早省事。
 *============================================================================*/
static void Uds_Service_TransferData(const uint8_t *d, uint16_t n)
{
	uint8_t  seq;
	uint16_t len;
	uint8_t  echo;

	if (n < 3u)
	{
		Uds_Negative(UDS_SID_TRANSFER, UDS_NRC_INCORRECT_LENGTH);
		return;
	}

	if (s_dl_active == 0u)
	{
		Uds_Negative(UDS_SID_TRANSFER, UDS_NRC_REQUEST_SEQUENCE_ERROR);
		return;
	}

	seq = d[1];
	len = (uint16_t)(n - 2u);

	/* 258 的上限扣掉 SID 和块序号，数据最多 256 字节 */
	if (len > 256u)
	{
		Uds_Negative(UDS_SID_TRANSFER, UDS_NRC_INCORRECT_LENGTH);
		return;
	}

	if (seq != s_dl_seq)
	{
		Uds_Negative(UDS_SID_TRANSFER, UDS_NRC_WRONG_BLOCK_SEQ);
		return;
	}

	if ((s_dl_off + (uint32_t)len) > s_dl_size)
	{
		Uds_Negative(UDS_SID_TRANSFER, UDS_NRC_REQUEST_OUT_OF_RANGE);
		return;
	}

	if (OtaStage_WriteImage(s_dl_off, &d[2], (uint32_t)len) != 0u)
	{
		/* 写 W25 失败：可能是芯片不在位、也可能擦除没做干净 */
		Uds_Negative(UDS_SID_TRANSFER, UDS_NRC_CONDITIONS_NOT_CORRECT);
		return;
	}

	s_dl_crc = OtaStage_CrcUpdate(s_dl_crc, &d[2], (uint32_t)len);
	s_dl_off += (uint32_t)len;
	s_dl_seq++;                     /* 0xFF 之后自然回到 0x00，符合标准 */

	echo = seq;
	Uds_Positive(UDS_SID_TRANSFER, &echo, 1u);
}

/*==============================================================================
 *  ★ 0x37 请求退出传输：核对 CRC-A，写就绪头，置升级标志
 *
 *  ★ 请求：37 <期望 CRC 高字节> <期望 CRC 低字节>
 *    ⚠️ 这两个字节是本实现加的（标准里 37 的参数是 OEM 自定义的"传输参数记录"）。
 *       为什么必须有：不核对 CRC 就等于"只要字节数够了就当成功"，
 *       而总线翻一位、W25 写错一页，字节数是照样够的。
 *  ★ 响应：77
 *
 *  ★ 为什么这里只置标志不自己复位：
 *    标准流程是 37 → 31 校验 → 11 复位。让 Tester 决定什么时候复位，
 *    设备不该替它做决定。Boot 看到升级标志 + 就绪头才会装。
 *============================================================================*/
static void Uds_Service_TransferExit(const uint8_t *d, uint16_t n)
{
	uint16_t want;

	if (n != 3u)
	{
		Uds_Negative(UDS_SID_TRANSFER_EXIT, UDS_NRC_INCORRECT_LENGTH);
		return;
	}

	if (s_dl_active == 0u)
	{
		Uds_Negative(UDS_SID_TRANSFER_EXIT, UDS_NRC_REQUEST_SEQUENCE_ERROR);
		return;
	}

	want = (uint16_t)(((uint16_t)d[1] << 8) | (uint16_t)d[2]);

	if (s_dl_off != s_dl_size)
	{
		/* 字节数不够就退出：说明中间丢块了，不许装 */
		Uds_Negative(UDS_SID_TRANSFER_EXIT, UDS_NRC_REQUEST_SEQUENCE_ERROR);
		return;
	}

	if (want != s_dl_crc)
	{
		/* CRC-A 不符：传输层就错了，连暂存头都不写，免得 Boot 拿去装 */
		s_dl_active = 0u;
		Uds_Negative(UDS_SID_TRANSFER_EXIT, UDS_NRC_REQUEST_OUT_OF_RANGE);
		return;
	}

	if (OtaStage_WriteReady(s_dl_size, s_dl_crc) != 0u)
	{
		s_dl_active = 0u;
		Uds_Negative(UDS_SID_TRANSFER_EXIT, UDS_NRC_CONDITIONS_NOT_CORRECT);
		return;
	}

	s_dl_active = 0u;

	/* ★ 置升级标志：下次复位 Boot 才会停在升级模式，把暂存那份装进内部 Flash
	 *   · App 工程里没有 boot_flag.c（只有头文件），所以这里照
	 *     freertos_demo.c 的 App_RequestUpgrade() 自己写一遍：
	 *     开时钟 -> 解锁备份域 -> 写魔数 -> 回读确认
	 *   · 回读这一步不能省：备份域没解锁时写会静默失败，
	 *     标志没进去而 37 却回了成功，Boot 就会照常跳回 App，白刷一场 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	BKP_WriteBackupRegister(BOOT_FLAG_DR, BOOT_FLAG_MAGIC);

	if (BKP_ReadBackupRegister(BOOT_FLAG_DR) != BOOT_FLAG_MAGIC)
	{
		Uds_Negative(UDS_SID_TRANSFER_EXIT, UDS_NRC_CONDITIONS_NOT_CORRECT);
		return;
	}

	Uds_Positive(UDS_SID_TRANSFER_EXIT, 0, 0u);
}

/*==============================================================================
 *  ★ 0x3E 保活：Tester 定期发一句"我还在"
 *
 *  ★ 为什么需要它：
 *    会话是"租来的"，不是"买下来的"。超过 S3（5 秒）不发 3E 就自动退回默认会话，
 *    这样 Tester 崩了、线拔了，设备自己会回到安全状态，不需要人来收拾。
 *============================================================================*/
static void Uds_Service_TesterPresent(const uint8_t *d, uint16_t n)
{
	uint8_t sf = 0u;

	if (n >= 2u)
	{
		sf = (uint8_t)(d[1] & 0x7Fu);
	}

	if ((n >= 2u) && (sf != 0u))
	{
		Uds_Negative(UDS_SID_TESTER_PRESENT, UDS_NRC_SUBFUNC_NOT_SUPPORTED);
		return;
	}

	s_s3_tick = xTaskGetTickCount();

	/* 只有不要求"别回"的时候才回
	 * ★ 响应必须原样回子功能：3E 00 -> 7E 00，不是 7E
	 *   · 只回 7E 的话 Tester 按"响应缺子功能"判长度不符，会当失败的
	 *   · 这条 2026-11-23 实测抓到过一次，当时就是漏了这一个字节 */
	if ((n < 2u) || ((d[1] & 0x80u) == 0u))
	{
		uint8_t echo = (n >= 2u) ? (uint8_t)(d[1] & 0x7Fu) : 0u;

		Uds_Positive(UDS_SID_TESTER_PRESENT, &echo, 1u);
	}
}

/*==============================================================================
 *  ★ 0x11 ECU 复位
 *
 *  ★ 为什么只在编程会话里允许：
 *    默认会话就是"正在干活"。这时候复位会把 SD 卡上那条正在写的记录劈成两半，
 *    而本工程的设计目标第一条就是"任意时刻断电已落盘数据不丢、文件不坏"。
 *    刷写流程里复位发生在 37 之后，那时数据早就落盘了。
 *============================================================================*/
static void Uds_Service_EcuReset(const uint8_t *d, uint16_t n)
{
	uint8_t sf;

	if (n < 2u)
	{
		Uds_Negative(UDS_SID_ECU_RESET, UDS_NRC_INCORRECT_LENGTH);
		return;
	}

	sf = (uint8_t)(d[1] & 0x7Fu);

	if (sf != 0x01u)
	{
		Uds_Negative(UDS_SID_ECU_RESET, UDS_NRC_SUBFUNC_NOT_SUPPORTED);
		return;
	}

	if (s_session != UDS_SESSION_PROGRAM)
	{
		Uds_Negative(UDS_SID_ECU_RESET, UDS_NRC_SERVICE_NOT_IN_SESSION);
		return;
	}

	/* ★ 复位也要过安全访问这一关
	 *   · 严格按 ISO 14229，11 服务并不强制要求先过 27；这里加上是因为
	 *     本工程的复位会中断 SD 记录，属于"会丢东西"的操作
	 *   · 真实刷写顺序里 27 排在 11 之前，所以正常流程不受影响 */
	if (s_unlocked == 0u)
	{
		Uds_Negative(UDS_SID_ECU_RESET, UDS_NRC_SECURITY_DENIED);
		return;
	}

	Uds_Positive(UDS_SID_ECU_RESET, &sf, 1u);

	/* ★ 先等这一帧真的发上总线再复位
	 *   · 复位一瞬间就把 CAN 外设关了，帧可能还躺在邮箱里没出去
	 *   · 5 ms 足够 500 kbps 把一帧发完（一帧约 0.26 ms），不需要等得更精细 */
	vTaskDelay(5u / portTICK_PERIOD_MS);

	NVIC_SystemReset();
}

/*==============================================================================
 *  ★ 分派
 *
 *  ★ 顺序上的讲究：先查长度再查会话
 *    长度不对是"报文本身坏了"，跟当前在哪个会话无关，应当先回 0x13。
 *    反过来会让 Tester 以为要切会话，多做一步无用功。
 *============================================================================*/
static void Uds_Dispatch(const uint8_t *d, uint16_t n)
{
	uint8_t sid = d[0];

	g_uds_req_cnt++;

	/* ★ S3 定时器由"任何请求"重置，不是只有 3E 才算（ISO 14229 的规定）
	 *   · 原来只在 0x10 和 0x3E 里刷，结果连续下载 5 秒后会话自己超时，
	 *     下载状态被清，第 35 块开始全部回 0x24
	 *   · 现实里的 Tester 会定期发 3E，但"靠 Tester 兜着"不该是设备的正确性前提
	 *   · 这条是 2026-11-23 传整包时测出来的 */
	s_s3_tick = xTaskGetTickCount();

#if (TEST_ISO_TP_DUMP != 0)
	Uds_Dump("RX ", d, (uint8_t)n);
#endif

	switch (sid)
	{
	case UDS_SID_SESSION:
		Uds_Service_Session(d, n);
		break;

	case UDS_SID_TESTER_PRESENT:
		Uds_Service_TesterPresent(d, n);
		break;

	case UDS_SID_ECU_RESET:
		Uds_Service_EcuReset(d, n);
		break;

	/* ---- 下面这几个还没实现，但要让 Tester 知道"是不支持，不是丢了" ---- */
	case UDS_SID_SECURITY:
		Uds_Service_Security(d, n);
		break;

	case UDS_SID_COMM_CTRL:
		Uds_Service_CommControl(d, n);
		break;

	case UDS_SID_ROUTINE:
		Uds_Service_Routine(d, n);
		break;

	case UDS_SID_REQ_DOWNLOAD:
		Uds_Service_RequestDownload(d, n);
		break;

	case UDS_SID_TRANSFER:
		Uds_Service_TransferData(d, n);
		break;

	case UDS_SID_TRANSFER_EXIT:
		Uds_Service_TransferExit(d, n);
		break;

	case UDS_SID_READ_DID:
		Uds_Service_ReadDid(d, n);
		break;

	case UDS_SID_CONTROL_DTC:
		Uds_Service_ControlDtc(d, n);
		Uds_Negative(sid, UDS_NRC_SERVICE_NOT_SUPPORTED);
		break;

	default:
		Uds_Negative(sid, UDS_NRC_SERVICE_NOT_SUPPORTED);
		break;
	}
}

/*==============================================================================
 *  ★ 初始化
 *============================================================================*/
void Uds_Init(void)
{
	s_session = UDS_SESSION_DEFAULT;
	s_s3_tick = 0u;
	s_comm_off = 0u;
	s_dtc_off  = 0u;

	s_seed_ok   = 0u;
	s_unlocked  = 0u;
	s_fail_cnt  = 0u;
	s_lock_tick = 0u;
	s_seed_seq  = 0u;

	s_dl_active = 0u;
	s_dl_size   = 0u;
	s_dl_off    = 0u;
	s_dl_crc    = 0u;
	s_dl_seq    = 0u;

	g_uds_req_cnt    = 0u;
	g_uds_nrc_cnt    = 0u;
	g_uds_s3_timeout = 0u;
}

/*==============================================================================
 *  ★ 轮询：取一包 → 分派；顺便看 S3 超时
 *============================================================================*/
void Uds_Poll(void)
{
	const uint8_t *p;
	uint16_t       n;

	/* ---- ① 有请求就处理 ---- */
	p = IsoTp_Take(&n);
	if (p != 0)
	{
		if (n > 0u)
		{
			Uds_Dispatch(p, n);
		}

		/* ★ 处理完必须交还，否则下一包会被"缓冲占着"挡掉 */
		IsoTp_Release();
	}

	/* ---- ② S3 超时：5 秒没有请求就退回默认会话 ---- */
	if (s_session != UDS_SESSION_DEFAULT)
	{
		if ((xTaskGetTickCount() - s_s3_tick) > (UDS_S3_MS / portTICK_PERIOD_MS))
		{
			s_session = UDS_SESSION_DEFAULT;
			g_uds_s3_timeout++;

			/* 退回默认会话时把状态都恢复：
			 * · 总线恢复发送，否则网关会一直以为设备离线
			 * · 故障记录恢复，否则刷写中途 Tester 掉了，设备就再也不记故障了
			 * · 安全访问也一起清（★ 2026-11-23 补）：
			 *   原来这里只清会话，`s_unlocked` 还留着 ⇒ 会话超时后
			 *   照样能擦 Flash。这和"退回默认会话必须清解锁"那条规矩自相矛盾，
			 *   是一个真越权：Tester 只要不吭声就能永久保留权限 */
			s_comm_off  = 0u;
			s_dtc_off   = 0u;
			s_dl_active = 0u;
			s_unlocked  = 0u;
			s_seed_ok   = 0u;
		}
	}
}

/*==============================================================================
 *  ★ 给上层用的查询
 *============================================================================*/
uint8_t Uds_GetSession(void)
{
	return s_session;
}

uint8_t Uds_CommDisabled(void)
{
	return s_comm_off;
}

uint8_t Uds_IsUnlocked(void)
{
	return s_unlocked;
}

/* ★ 读一条 ISO-TP 传输层的统计（0x22 ReadDataByIdentifier，2026-11-24 加）
 *
 * ★ 为什么加这个服务
 *   原来 ISO-TP 组回归的判据是"设备往串口打了一行 [ISOTP] n=.. crc=.."。
 *   那一行要在 task_KEY 里忙等约 8 ms，而且把测试和"打印格式"绑死了。
 *   改走 UDS 之后 dump 可以关，回归也不再依赖设备往串口写什么。
 *
 * ★ 为什么回"上一条"而不是"最后一条"
 *   读 0x22 的这条请求本身就是一条 ISO-TP 报文，取包时会把它顶成"最后一条"。
 *   只有"上一条"才是被测的那一包。
 *
 * ★ 报文格式
 *   请求 22 F1 A0
 *   肯定 62 F1 A0 <len_hi> <len_lo> <crc_hi> <crc_lo>（7 字节，正好是单帧上限）
 *   否定 7F 22 13（长度不够）/ 7F 22 31（别的 DID，本项目只认 F1A0）
 */
static void Uds_Service_ReadDid(const uint8_t *d, uint16_t n)
{
	uint8_t  r[6];
	uint16_t did;

	if (n < 3u)
	{
		Uds_Negative(UDS_SID_READ_DID, UDS_NRC_INCORRECT_LENGTH);
		return;
	}

	did = (uint16_t)(((uint16_t)d[1] << 8) | (uint16_t)d[2]);

	if (did != UDS_DID_ISOTP_STAT)
	{
		Uds_Negative(UDS_SID_READ_DID, UDS_NRC_REQUEST_OUT_OF_RANGE);
		return;
	}

	r[0] = (uint8_t)(did >> 8);
	r[1] = (uint8_t)(did & 0xFFu);
	r[2] = (uint8_t)(g_isotp_prev_len >> 8);
	r[3] = (uint8_t)(g_isotp_prev_len & 0xFFu);
	r[4] = (uint8_t)(g_isotp_prev_crc >> 8);
	r[5] = (uint8_t)(g_isotp_prev_crc & 0xFFu);

	Uds_Positive(UDS_SID_READ_DID, r, 6u);
}

uint8_t Uds_DtcDisabled(void)
{
	return s_dtc_off;
}

#endif /* CAN_EN */
