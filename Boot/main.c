#include "stm32f10x.h"
#include "boot_config.h"
#include "boot_jump.h"
#include "boot_flag.h"
#include "boot_upgrade.h"
#include "boot_w25.h"           /* ★ W25Q64 只读驱动（2026-10-14 加） */
#include "boot_golden.h"        /* ★ 金副本恢复（2026-10-14 加，G5） */
#include "boot_param.h"         /* ★ 4KB 参数区（2026-11-07，U1/U2） */
#include "Serial.h"

/*==============================================================================
 *  ★ Bootloader 主流程（2026-10-13 建，当日三次迭代）
 *
 *  ★ 三步走：
 *    · 第一步：纯跳转（分区 + 有效性检查 + 跳）—— 已完成
 *    · 第二步：升级请求标志（BKP_DR2 魔数）—— 已完成
 *    · 第三步：串口收固件（协议 + 擦写 + CRC）—— 本次完成
 *
 *  ★ 本文件只干一件事：决定"该跳 App，还是该进升级模式"
 *    · 别的都在各自文件里：
 *      · boot_jump.c，有效性检查 + 跳转
 *      · boot_flag.c，升级标志读写
 *      · boot_flash.c，Flash 擦 / 写 / 回读
 *      · boot_upgrade.c，协议状态机（第三步的主体）
 *    → 为什么非要分开：这些每一条都能单独验证
 *      混在一起出问题就分不清是谁的错
 *============================================================================*/

/*==============================================================================
 *  ★ 粗延时（只在开机判按键这一处用，别的地方一律不许用）
 *
 *  ★ 标定：72MHz，volatile uint32_t 空循环每圈约 4 个周期
 *    → 18000 圈 ≈ 1ms（和老版 main.c 里那个粗延时同一口径）
 *
 *  ⚠️ 绝对不许把它用到 boot_upgrade.c 的接收循环里
 *    · 原因：115200 下11.52 字节/ms，而环形缓冲只有 64 字节
 *      → 1ms 的延时就会让缓冲开始堆积，再大就必丢字节
 *    · 这里能用，是因为这是开机时的一次性动作
 *      此刻串口上什么都没在传
 *============================================================================*/
static void Boot_DelayMs(uint16_t ms)
{
	volatile uint32_t d;

	while (ms > 0u)
	{
		for (d = 0u; d < 18000u; d++) { }
		ms--;
	}
}


/*==============================================================================
 *  ★ 有没有人按住按键要强制进 Bootloader（2026-10-13 深夜加）
 *
 *  ★ 为什么要读两次，而不是读一次：
 *    · 机械按键有抖动，按下/松开的瞬间电平会来回跳
 *    · 只读一次，可能正好读到"抖成低电平"的那一瞬 → 漏判
 *       （手按住的那 20ms 里读到一次 0，就白按了）
 *    → 隔约 20ms 再读一次，两次都为"按下"才算数
 *      （这就是最朴素的消抖，和 App 的 Key.c 状态机同一个道理）
 *
 *  ★ 为什么不按住的正常开机只花几微秒：
 *    · 第一次读就是 0，立刻 return，连那 20ms 都不花
 *      → 正常开机一点也不变慢
 *============================================================================*/
static uint8_t Boot_KeyForcesUpgrade(void)
{
	GPIO_InitTypeDef gi;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	gi.GPIO_Mode  = GPIO_Mode_IPD;          /* ★ 下拉：空闲 0，按下 1（见 boot_config.h） */
	gi.GPIO_Pin   = BOOT_KEY_PINS;
	gi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BOOT_KEY_GPIO, &gi);

	/* ★ 第一次读，没按住就直接放行。 */
	if (BOOT_KEY_ANY_PRESSED() == 0u)
	{
		return 0u;
	}

	/* ★ 按住了一次，等一下再确认（消抖） */
	Boot_DelayMs(20u);

	return BOOT_KEY_ANY_PRESSED();
}


int main(void)
{
	uint8_t by_key;                     /* ★ 有人按住键吗（最后一条自救路） */
	uint8_t by_flag;                    /* ★ 有人显式请求升级吗。 */
	uint8_t app_ok;                     /* ★ App 现在有效吗。 */
	uint8_t by_rb;                      /* ★ 有人请求回退吗（2026-10-14 加） */
	uint8_t iwdg_rst;                   /* ★ 上次复位是狗咬的吗。 */
	uint8_t by_iwdg_stop = 0u;          /* ★ 回退过一次还是被狗咬 ⇒ 停在升级模式（2026-10-19 修审计 B1）
	                                     *   ★ 必须带 = 0u，它只在 by_key == 0u 里被赋值
	                                     *     而 :401 是无条件读它 ⇒ 否则读到的是栈上垃圾值 */
	ParamRec_t prm;                     /* ★ 4KB 参数区记录（2026-11-07） */
	uint8_t    prm_ok;
	uint8_t    prm_saved = 1u;
	uint8_t    trial_fail = 0u;         /* ★ image_ok=0 试用超限 */

	Serial_Init();

	Serial_SendString("\r\n=== BOOT ===\r\n");

	/* ★ 原来这里是一行 printf 带格式，现在拆成多次调用
	 *   ★ 为什么值得啰嗦：
	 *     · 保留 printf 要花 3,664 字节（当时占 8KB 的 44.7%
	 *        现在 Boot 是 16KB ⇒ 约占 22%）
	 *     · 拆开只多几行，但省下整个格式化库
	 *   ★ 而且这条日志只在启动时打一次
	 *     不用担心"拆开会不会变慢" */
	Serial_SendString("BOOT: app area = 0x");
	Serial_SendHex((uint32_t)APP_BASE, 8u);          /* ★ 对应 %08X。 */
	Serial_SendString(" - 0x");
	Serial_SendHex((uint32_t)APP_END, 8u);
	Serial_SendString("  (");
	Serial_SendU32((uint32_t)(APP_SIZE / 1024u));    /* ★ 对应 %u。 */
	Serial_SendString(" KB)\r\n");

	/*==========================================================================
	 *  ★ 看门狗现场取证（2026-10-13 深夜加）
	 *
	 *  ★ 为什么要有这一段：
	 *    · 评审文档说IWDG 软复位后不停，每 6.55 秒咬一次
	 *    · 而实测 22 秒一次都没咬（旧 Boot 升级模式里）
	 *    → "没咬"是事实，但"为什么"还没被证明
	 *    → 只有读寄存器能把这一步钉死
	 *
	 *  ★ 怎么读（判据写在 boot_config.h 的 BOOT_IWDG_PROBE 旁边）：
	 *    · PR == 04→ IWDG 寄存器活过了软复位
	 *      → 那"没咬"只能是计数没在跑（即 enable 被复位清了）
	 *    · PR == 00→ 整个 IWDG 外设被软复位清了
	 *      → 这就是最干净的解释，而且坑 1 在纯软件 IWDG 下不成立
	 *    ⚠️ 只在"软复位进来的那次"才有意义
	 *      → 上电复位（拔电重上）那次 PR 本来就会是 0，别误读
	 *========================================================================*/
#if BOOT_IWDG_PROBE
	Serial_SendString("BOOT: IWDG probe PR=");
	Serial_SendHex((uint32_t)IWDG->PR, 2u);
	Serial_SendString(" RLR=");
	Serial_SendHex((uint32_t)IWDG->RLR, 4u);
	Serial_SendString(" SR=");
	Serial_SendHex((uint32_t)IWDG->SR, 2u);
	/* ⚠️ 这条串口输出必须是纯 ASCII
	 *   · 踩过：原来这里写了中文，armcc 报
	 *     #870-D: invalid multibyte character sequence
	 *   · 原因：AC5 按本地编码解读字符串字面量
	 *     而我们的源码是 UTF-8→ 中文被解成非法字节
	 *   → 本工程的规矩（App 那边也是）：
	 *     注释可以随便写中文，字符串字面量一律 ASCII */
	Serial_SendString("  (PR: 00=reset-cleared  04=set-by-App ; RLR default 0FFF)\r\n");
#endif

	/*==========================================================================
	 *  ★ 金副本开机自检（2026-10-14 补，教训换来的）
	 *
	 *  ★ 为什么必须有这一段：
	 *    · 加完金副本之后，启动日志和旧版一模一样
	 *      （因为四条判据都不成立时它一句话都不说）
	 *    · 结果，用户烧了新 Boot，无法确认烧进去了没
	 *      → "什么也没出现"，分不清是"对"还是"没生效"
	 *    · 教训，加了功能就要让它看得见
	 *      —— 和 CRC selftest / IWDG probe 同一个道理
	 *
	 *  ★ 它一次回答三个问题：
	 *    · 新 Boot 跑没跑 → 有这行 = 跑了
	 *    · W25Q64 在不在 → W25 ok / NOT FOUND
	 *    · 备份有没有 → backup crc=xxxx / backup none
	 *  ★ 代价，约 5ms（两次 SPI 读），开机一次，无所谓
	 *========================================================================*/
	if (BootW25_Init() != 0u)
	{
		uint16_t bcrc = 0u;

		Serial_SendString("GOLD: W25 ok, backup ");
		if (BootGolden_BackupInfo(&bcrc, 0) != 0u)
		{
			Serial_SendString("crc=");
			Serial_SendHex((uint32_t)bcrc, 4u);
			Serial_SendString("\r\n");
		}
		else
		{
			Serial_SendString("none\r\n");
		}
	}
	else
	{
		Serial_SendString("GOLD: W25 NOT FOUND\r\n");
	}

	/*==========================================================================
	 *  ★ 判据①：有人按住按键吗（2026-10-13 深夜加）
	 *
	 *  ★ 为什么它必须排在最前面：
	 *    · 另外两条判据都有前提：
	 *      · 判据② 依赖 BKP；本板有电池，但换电池 / 电池耗尽时仍会丢
	 *      · 判据③ 依赖"App 是坏的"，而"有效但卡死"的 App 它判不出来
	 *    · 只有这一条，不依赖任何存储，也不依赖 App 的好坏
	 *      → 它是人的明确意图，直接覆盖一切自动判断
	 *    · 这正是"最后一条自救路"的含义
	 *
	 *  ★ 时序：必须在还没跳 App 之前读
	 *    （这里正是，而且此刻 GPIOB 还没人用过，App 的 Key_Init 还没跑）
	 *========================================================================*/
	by_key = Boot_KeyForcesUpgrade();

	/*==========================================================================
	 *  ★ 判据②：升级请求标志（第二步，2026-10-13 加）
	 *
	 *  ★ 流程：
	 *    · App 收到升级命令 → 写 BKP_DR2 = 0xA5C3 + 软复位
	 *    · 软复位后CPU 从 0x08000000 起，也就是从我们这里起
	 *    · 我们一读标志，就知道"该进升级模式了"
	 *========================================================================*/
	BootFlag_Init();
	by_flag = BootFlag_IsUpgradeRequested();
	by_rb   = BootFlag_IsRollbackRequested();   /* ★ 回退标志（BKP_DR3） */

	/*==========================================================================
	 *  ★ 2026-11-07：4KB 参数区（U1/U2）
	 *
	 *  ★ 先读参数；没有就迁移 BKP 的一次性标志。
	 *  ★ BKP 继续当"旧 App 的邮箱"：
	 *    · 旧 App 仍写 BKP_DR2/DR3/DR6；
	 *    · Boot 每次启动把它们折进参数区，然后清掉 BKP；
	 *    · 这样旧 App 不改也能用新 Boot。
	 *  ★ 参数区写失败 → 保留 BKP，下一次还能重试。
	 *========================================================================*/
	prm_ok = Param_Load(&prm);

	if (prm_ok == 0u)
	{
		Param_Default(&prm);
		if (by_flag != 0u) { prm.flags |= PARAM_FLAG_UPGRADE; }
		if (by_rb   != 0u) { prm.flags |= PARAM_FLAG_ROLLBACK; }
		prm.iwdg_retry = (uint32_t)BootFlag_GetIwdgRetry();
		prm_saved = Param_Save(&prm);
	}
	else
	{
		uint8_t dirty = 0u;

		if ((by_flag != 0u) && ((prm.flags & PARAM_FLAG_UPGRADE) == 0u))
		{
			prm.flags |= PARAM_FLAG_UPGRADE;
			dirty = 1u;
		}
		if ((by_rb != 0u) && ((prm.flags & PARAM_FLAG_ROLLBACK) == 0u))
		{
			prm.flags |= PARAM_FLAG_ROLLBACK;
			dirty = 1u;
		}
		if ((uint32_t)BootFlag_GetIwdgRetry() > prm.iwdg_retry)
		{
			prm.iwdg_retry = (uint32_t)BootFlag_GetIwdgRetry();
			dirty = 1u;
		}
		if (dirty != 0u) { prm_saved = Param_Save(&prm); }
	}

	if (prm_saved != 0u)
	{
		/* ★ 已折进参数区 ⇒ 清 BKP，防下一次重复折叠 */
		BootFlag_ClearUpgrade();
		BootFlag_ClearRollback();
		BootFlag_ClearIwdgRetry();
	}

	by_flag = ((prm.flags & PARAM_FLAG_UPGRADE)  != 0u) ? 1u : 0u;
	by_rb   = ((prm.flags & PARAM_FLAG_ROLLBACK) != 0u) ? 1u : 0u;

	Serial_SendString("PARAM: img=");
	Serial_SendU32(prm.image_ok);
	Serial_SendString(" boot=");
	Serial_SendU32(prm.boot_count);
	Serial_SendString(" seq=");
	Serial_SendU32(prm.seq);
	Serial_SendString(" flags=");
	Serial_SendHex(prm.flags, 2u);
	Serial_SendString(" min=");
	Serial_SendU32(prm.img_min_ver);          /* ★ U5：防回滚下限 */
	Serial_SendString("\r\n");

	/*==========================================================================
	 *  ★ 判据③：App 有没有效（第三步，2026-10-13 加）
	 *
	 *  ★ 为什么必须有这一条，只有上面那条不行
	 *         —— 这是"安全模型"的承重墙
	 *
	 *    · 上面那条存在备份寄存器里
	 *      → 而板子虽有电池，换电池 / 电池耗尽时 BKP 仍会清零
	 *    · 于是危险的场景：
	 *      · 升级升到一半，断电（此时 App 区是半成品）
	 *      · 重新上电，BKP 没了，我们以为没人要升级
	 *        → 直接去跳 App
	 *      · 而"半成品"里旧固件的向量表可能还是好的
	 *        → 有效性检查会通过 → 按旧向量表跳进新代码，必炸
	 *
	 *  ★ 所以要加一条不依赖任何易失存储的硬判据：
	 *    · App 无效= 第 0 页被擦过（全 0xFF）
	 *      → 栈顶指针不在 RAM 里 → 检查必然失败
	 *    · 这个事实写在 Flash 里，断电也成立，不靠电池
	 *
	 *  ★ 三条是"或"的关系：
	 *    · ① 有人按住键 → 进（最后一条自救路，见 boot_config.h）
	 *    · ② 有人请求升级（BKP 标志）→ 进
	 *    · ③ App 是坏的 → 也必须进（不然它就没救了）
	 *    · 否则半成品状态下重启，板子永远回不来
	 *      （而进了升级模式，PC 连上重发一次就能安全重来）
	 *
	 *  ★ 为什么①要排在最前面：
	 *    · 它是唯一一条不依赖任何存储 / 不依赖 App 好坏的路
	 *    · 按键按住 = 人的明确意图，优先于一切自动判断
	 *========================================================================*/
	app_ok = Boot_IsAppValid();

	/*==========================================================================
	 *  ★ 2026-11-07（U3）：App 无效时，先看 W25 暂存区有没有一份完整镜像
	 *
	 *  ★ 什么场景：升级装到一半断电（第 0 页还没写）
	 *    · 老路径：停在升级模式，等人接电脑重发
	 *    · 现在：暂存区那份还在且校验通过 ⇒ 自己装回去，直接开机
	 *
	 *  ★ 为什么只在没有人为干预时做：
	 *    · 按住按键 / BKP 写着升级请求 = 人的明确意图
	 *    · 自动装回会把这个意图抢走，那是最不该发生的事
	 *========================================================================*/
	if ((app_ok == 0u) && (by_key == 0u))
	{
		if (Up_RecoverFromStage() != 0u)
		{
			app_ok = Boot_IsAppValid();      /* ★ 装完必须重判，别用旧值 */
			/* ★ 升级请求已经兑现：清标志，也清本地变量
			 *   · 不清的话，下面的升级模式判断还会命中，
			 *     装好了却又停回升级模式 —— 等于白装 */
			by_flag = 0u;
			BootFlag_ClearUpgrade();
		}
	}

	/*==========================================================================
	 *  ★ 复位原因（2026-10-14 加，金副本回退的判据③）
	 *
	 *  ★ 必须在这里、且只读一次：
	 *    · 它读完顺手会把标志清掉
	 *      → 再读第二次就永远返回 0 了
	 *========================================================================*/
	iwdg_rst = BootGolden_ResetWasIwdg();

	/* ★ 2026-11-07：image_ok=0 的试用计数
	 *   · 只在"App 有效、没按键、还没确认"时写；
	 *   · image_ok=1 后正常启动根本不写参数区 ⇒ 不磨 10k 寿命。 */
	if ((by_key == 0u) && (app_ok != 0u) && (prm.image_ok == 0u))
	{
		prm.boot_count++;
		if (Param_Save(&prm) == 0u)
		{
			Serial_SendString("PARAM: trial save FAILED\r\n");
		}
		if (prm.boot_count > PARAM_BOOT_TRIAL_MAX)
		{
			trial_fail = 1u;
		}
	}

	/*==========================================================================
	 *  ★ 金副本恢复（2026-10-14 加，G5，本次的核心）
	 *
	 *  ★ 它补上了最后一块拼图：
	 *    · 在这之前，按键自救只能把人送进 Bootloader
	 *      → 进去之后还是需要一台 PC + 一个固件
	 *    · 有了它，Boot 自己就能把上一版装回来
	 *
	 *  ★ 四条判据（顺序 = 优先级）：
	 *    · ① R 回退标志（BKP_DR3）—— 人明确要求
	 *    · ② App 无效（第 0 页被擦 / 半成品）
	 *    · ③ 上次复位是 IWDG 咬的 ——
	 *      "App 卡死"的硬件级证据
	 *      （这一条CRC 判据抓不到
	 *        "镜像完好，但功能跑飞"，CRC 完全看不出来）
	 *    · ④ App 区 CRC ≠ 备份头里的 CRC —— 镜像坏了
	 *
	 *  ⚠️ 三条安全护栏（一条都不能省）：
	 *    · 按键优先，有人按住键 = 他要的是"给我进升级模式"
	 *      → 绝不能被"自动恢复"抢走
	 *    · 没有备份 → 一条都不恢复，直接往下走
	 *      → 这样绝不会因为"没备份"把板子卡死
	 *    · 恢复失败也往下走
	 *      → App 有效就跳，无效就进升级模式（原来的逻辑）
	 *========================================================================*/
	if (by_key == 0u)                   /* ★ 护栏①：按键优先于一切自动判断。 */
	{
		uint8_t     need = 0u;
		const char *why  = "";

		if (by_rb != 0u)
		{
			need = 1u; why = "R flag";
		}
		else if (app_ok == 0u)
		{
			need = 1u; why = "app invalid";
		}
		else if (iwdg_rst != 0u)
		{
			/* ★ 2026-10-18 改（⑥ 用户拍板 A）：IWDG 回退加次数上限
			 *
			 *  ★ 为什么：判据⑤原来是"被狗咬过 ⇒ 无条件换回旧版"
			 *    · 偶发被咬（SD 卡顿 4 秒正好撞上）⇒ 镜像好的也被换
			 *    · 旧版也咬狗 ⇒ 每次开机擦写 112 KB的循环
			 *
			 *  ★ 现在的规则：
			 *    · 第 1 次 ⇒ 照旧自动回退（自愈优先）
			 *    · 第 2 次 ⇒ 不再擦写，停在升级模式等人
			 *      （回退都救不回 ⇒ 问题不在版本上）
			 *
			 *  ★ 怎么"停在升级模式"（2026-10-19 改：用独立局部变量）：
			 *    · 原来想把 app_ok 置 0，可 :401 判的是
			 *      当场重读的 Boot_IsAppValid()，⇒ 置 0根本没人看
			 *      （而 :292 那次读早就过去了，这正是 B1 的原因）
			 *    · 现在改用 by_iwdg_stop，自己的变量，自己的判据
			 *      ⇒ :401 多一个分支，而日志说得是实话 */
			if (prm.iwdg_retry >= 1u)
			{
				by_iwdg_stop = 1u;
				prm.iwdg_retry = 0u;
				(void)Param_Save(&prm);
				Serial_SendString("G: IWDG again -> NO rollback, stay in upgrade\r\n");
			}
			else
			{
				prm.iwdg_retry++;
				(void)Param_Save(&prm);        /* ★ 先记一笔，再回退。 */
				need = 1u; why = "IWDG reset";
			}
		}
		else if (trial_fail != 0u)
		{
			/* ★ 2026-11-07：image_ok=0 且连续启动超过 3 次 ⇒ 回退
			 *   · 第一次回退；恢复后 iwdg_retry 保留 1；
			 *   · 恢复的旧版再失败 ⇒ 停升级模式，不无限擦写。 */
			if (prm.iwdg_retry >= 1u)
			{
				by_iwdg_stop = 1u;
				prm.iwdg_retry = 0u;
				(void)Param_Save(&prm);
				Serial_SendString("G: trial again -> NO rollback, stay in upgrade\r\n");
			}
			else
			{
				prm.iwdg_retry++;
				(void)Param_Save(&prm);
				need = 1u; why = "trial";
			}
		}
		/* ⚠️ 这里原来还有第 ④ 条，2026-10-14 当场删掉了
		 *
		 *  ★ 原判据，"App 区 CRC ≠ 备份头里的 CRC" → 恢复
		 *
		 *  ★ 它和用户的设计直接打架：
		 *    · 用户的设计，备份 = 上一版
		 *    · 而升级之后，App 区是新版，备份是旧版
		 *      → 两者本来就该不一样
		 *    → 于是每次升级成功后，下次开机都会被自动换回旧版
		 *      （现象，"PASS 了，但跑起来还是老版本
		 *        而且怎么升级都没用"，极难查）
		 *
		 *  ★ 它还和判据③功能重叠：
		 *    · 镜像真坏了 → 要么App 无效，要么IWDG 咬
		 *    → 这两条都会先抓到，不需要重复一遍
		 *
		 *  ★ 教训（比这个 bug 本身值钱）：
		 *    ★ 两个各自都合理的设计，拼在一起会互相打架
		 *    → "备份是上一版" 和 "CRC 必须一致"
		 *      不可能同时成立，加规则前必须先想它和已有规则的相互作用 */

		if (need != 0u)
		{
			/* ★ 2026-10-19：B2 已用临时强制分支上板验证通过
			 *   · 做法：临时置一个 volatile 标志 ⇒ 强制走"W25 读不到"那条出口
			 *   · 判据：发 R 后第 1 次复位 ⇒ G: trigger R flag + G: no W25Q64
			 *     ，再按一次 NRST ⇒不再出现 G: trigger R flag
			 *     （改前会一直出现 —— 那就是 DR3 粘滞的指纹）
			 *   · 证据，证据日志\B2-验证-20261019.log
			 *   ★ 临时开关已删除，下面就是正常逻辑 */

			/* ⚠️ 这几条字符串故意写得极短
			 *   · 理由，见 boot_golden.c 里那段"实测超了 176 字节"的记录
			 *   · Boot 只有 8KB，日志也要算字节 */
			Serial_SendString("G: trigger ");
			Serial_SendString(why);
			Serial_SendString("\r\n");

			/* ★ 2026-10-19 修（审计 B2，DR3 粘滞吞掉升级请求）：
			 *   ★ DR3 是一次性指令，不是状态
			 *   · 原来只有"W25 在 + 恢复失败"那一条出口会清它
			 *     ⇒ W25 不在位时 DR3 会一直挂着
			 *   · 后果链条：
			 *       下次发 U ⇒ Boot 先命中 by_rb ⇒ 去"回退"
			 *       → 恢复成功 ⇒ 顺手把 DR2 一起清掉
			 *       ⇒ 升级请求被吞，现象是"怎么升级都没用"
			 *   · 规矩改成，只要决定去恢复，这笔请求就当场用掉
			 *     （成功 / 失败 / W25 不在位，三条出口一视同仁）
			 *   · 放在"先看备份"之前也没关系：
			 *     万一恢复中途断电，第 0 页已被擦成 0xFF
			 *     ⇒ 下次开机命中"App 无效"，照样会再去恢复 */
			if (by_rb != 0u)
			{
				prm.flags &= ~PARAM_FLAG_ROLLBACK;
				(void)Param_Save(&prm);
			}

			if (BootW25_Init() == 0u)
			{
				Serial_SendString("G: no W25Q64\r\n");
				if (trial_fail != 0u) { by_iwdg_stop = 1u; }
			}
			else if (BootGolden_Restore() != 0u)
			{
				/* ★ 恢复出来的旧版也要走一次"试用→确认"：
				 *   image_ok=0，boot_count=0；保留 iwdg_retry 作为恢复次数上限。 */
				(void)Param_OnNewImage();

				/* ★ DR3 已经在"决定要恢复"那一刻清掉了，这里只剩 DR2。 */
				BootFlag_ClearUpgrade();

				Serial_SendString("G: restored -> jump\r\n");
				while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }

				Boot_JumpToApp();       /* ★ 不会返回（恢复时已复检有效） */
			}
			else
			{
				/* ★ 护栏③：恢复不成就照旧往下走
				 *   → App 有效就跳，无效就进升级模式， */
				Serial_SendString("G: cannot restore\r\n");
				if (trial_fail != 0u) { by_iwdg_stop = 1u; }
				/* ★ DR3 已在上面统一清掉，这里不用再清
				 *   （2026-10-19 修 B2：原来只在这一条出口清，漏了"W25 不在位"） */
			}
		}
	}

	/* ★ 2026-10-18 修（复盘 B1，变砖级）：
	 *   原来这里用的是 main.c:247 采来的旧快照app_ok
	 *   · 恢复流程第 0 页擦完之后才可能失败（boot_golden.c:171）
	 *   · 而那时 app_ok 还是 1，⇒ 这三个条件全假
	 *     → 走到下面 :417 ⇒ 跳进已被擦成 0xFF 的 App 区
	 *       ⇒ SP = 0xFFFFFFFF ⇒ 立刻 HardFault
	 *   · 现象：串口全静默，看着像砖，只能按住键复位救
	 *   ⇒ 改成现在重新问一次
	 *     Boot_IsAppValid() 只读头两个字，很便宜
	 *     ★ 判据失效就停在升级模式等 PC，绝不跳空地 */
	if ((by_key != 0u) || (by_flag != 0u) || (by_iwdg_stop != 0u) || (Boot_IsAppValid() == 0u))
	{
		if (by_key != 0u)
		{
			/* ★ 这一条是本次新加的
			 *   ★ 打两行：
			 *     · 第一行坐实"是按键触发的"，而不是标志/App 坏了
			 *     · 第二行告诉人怎么出去，不然他只会看到"停在升级模式" */
			Serial_SendString("BOOT: KEY HELD at reset -> FORCED upgrade mode\r\n");
			Serial_SendString("BOOT: (release the key; send 'C' to boot the App)\r\n");
		}
		else if (by_flag != 0u)
		{
			Serial_SendString("BOOT: UPGRADE FLAG detected -> STAY in upgrade mode\r\n");

			/* ★ 故意不在这里清除标志 —— 这是有意为之
			 *   · 好处：升级中途断电 / 复位
			 *     → 重新上电还在这里，可以接着升级
			 *   · 真正清标志的地方
			 *     → boot_upgrade.c 里"校验通过，写完第 0 页"之后 */
			Serial_SendString("BOOT: (flag KEPT on purpose - cleared after upgrade OK)\r\n");
		}
		else if (by_iwdg_stop != 0u)
		{
			/* ★ 这一条只说事实（2026-10-19 修审计 B1 新增）
			 *   ★ 不冒充升级标志：
			 *     · BKP_DR2 这会儿真的是 0
			 *       ⇒ 所以绝不说"检测到升级标志"
			 *     · 也不说"标志故意留着"，那是上面 by_flag 那条分支的事
			 *   ★ 它只回答两个问题：
			 *     · 为什么停= 回退过一次，结果还是被狗咬
			 *     · 怎么出去= 发固件，或者按 C 跳回 App
			 *   ★ 字符串必须纯 ASCII（规矩见本文件 :140-146） */
			Serial_SendString("BOOT: IWDG again after rollback -> STAY in upgrade mode\r\n");
			Serial_SendString("BOOT: (send firmware to upgrade, or 'C' to boot the App)\r\n");
		}
		else
		{
			/* ★ 这条分支就是本次新加的
			 *   ★ 老版本这里是 BOOT: NO VALID APP -> halted + for(;;){}
			 *   · 为什么必须改掉：
			 *     · 狗可能还在跑（上电复位才停它，见 boot_config.h）
			 *       → 那个 for(;;){} 会变成"每 6.55 秒重启一次"
			 *     · 而更重要的：停在这里什么也做不了
			 *       断电重来还是停在这里，只能拔电 + 重新烧录
			 *     → 改成"进升级等待循环"
			 *       PC 一连上就能把固件灌进去 */
			Serial_SendString("BOOT: NO VALID APP (page0 empty/corrupt)\r\n");
			Serial_SendString("BOOT: -> entering upgrade mode (this is the only way back)\r\n");
		}

		Serial_SendString("BOOT: waiting for firmware...\r\n");

		/* ★ 进升级引擎 —— 它永不返回，出口只有"跳 App"
		 *   · 它会逐轮喂狗（修掉那个 6.55 秒重启循环）
		 *   · 它自己管 C 取消，管协议，管 Flash，管 CRC */
		BootUpgrade_Run();
	}

	/*==========================================================================
	 *  ★ 到这里 = 两个条件都不成立：
	 *    · 没人请求升级
	 *    · App 有效
	 *  → 正常开机，跳
	 *========================================================================*/
	Serial_SendString("BOOT: no upgrade request -> normal boot\r\n");
	Serial_SendString("BOOT: app valid -> jumping\r\n");

	/* ★ 等串口真的发完再跳
	 *  ★ 为什么必须等：
	 *    · Serial_SendByte() 等的是 TXE（发送寄存器空）
	 *    · 而最后一个字节可能还在移位寄存器里
	 *      → 一跳过去，App 的 Serial_Init() 就把它冲掉了
	 *      → 结果是那行日志时有时无，很难查
	 *  ★ TC（Transmission Complete）才是"真的发完了" */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }

	Boot_JumpToApp();               /* ★ 不会返回。 */

	/* ★ 万一真的回来了 → 停住，总比跑飞好
	 *   ★ 但别忘了喂狗，否则又变成 6.55 秒重启 */
	Serial_SendString("BOOT: !!! returned from App -> halted\r\n");
	for (;;) { BOOT_FEED_IWDG(); }
}
