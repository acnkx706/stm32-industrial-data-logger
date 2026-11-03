#include "stm32f10x.h"
#include "boot_config.h"
#include "boot_jump.h"

/*==============================================================================
 *  ★ Bootloader 核心：App 有效性检查 + 跳转（2026-10-13 建）
 *        设计参考 stm32-bootloader-master（Akos Pasztor）
 *============================================================================*/

/* ★ 函数指针类型，用来"取地址然后跳过去"。 */
typedef void (*Boot_AppEntry_t)(void);


/* ★ 检查 App 是否看起来有效
 *   只读两个字，极便宜，但能挡住三种坏情况
 *   判据详见 boot_jump.h。 */
uint8_t Boot_IsAppValid(void)
{
	/* ★ App 的首字 = 初始栈顶指针（由链接器写进去的） */
	uint32_t sp = *(__IO uint32_t *)APP_BASE;

	/* ★ 第二个字 = 复位向量= App 第一条指令的地址。 */
	uint32_t pc = *(__IO uint32_t *)(APP_BASE + 4u);

	if (BOOT_IS_SP_VALID(sp) == 0) { return 0u; }
	if (BOOT_IS_PC_VALID(pc) == 0) { return 0u; }

	return 1u;
}


/* ★ 跳到 App，五步，顺序不能换。 */
void Boot_JumpToApp(void)
{
	uint32_t        appSp    = *(__IO uint32_t *)APP_BASE;
	Boot_AppEntry_t appEntry = (Boot_AppEntry_t)(*(__IO uint32_t *)(APP_BASE + 4u));
	uint8_t         i;

	/* ★ 2026-10-18 加：跳转前的最后一道闸
	 *
	 *  ★ 为什么必须有：
	 *    · 本函数拿到两个数就跳，不做任何校验
	 *    · 而它的后果是不可恢复的
	 *      __set_MSP(0xFFFFFFFF) ⇒ 立刻 HardFault
	 *      ⇒ 而这时候中断已经全关了
	 *      ⇒ 连一句报错都打不出来⇒ 看上去就是砖
	 *
	 *  ★ 为什么现在加：
	 *    · 上游每一处都以为自己检查过了
	 *    · 而 2026-10-18 的复盘（B1）证明上游会漏
	 *      （旧快照 app_ok 那一条就是）
	 *    ⇒ 这里再问一次：它是唯一"跳"的地方
	 *
	 *  ★ 无效时怎么做：
	 *    · 停住 + 喂狗，不跳、也不让它反复复位
	 *    · 为什么喂狗：不喂 ⇒ 6.55s 后 IWDG 复位
	 *      ⇒ 起来又走到这 ⇒ 复位循环，每一次都可能触发擦写
	 *    · 停在半路，人还能按住键重来
	 *
	 *  ★ 为什么不在这里直接进升级模式：
	 *    · 那要给本文件加 boot_upgrade 的依赖
	 *    · 而且走到这一步本来就不该发生，属于"兜底"
	 *      ⇒ 兜底只做最保守的事 */
	if (Boot_IsAppValid() == 0u)
	{
		for (;;) { BOOT_FEED_IWDG(); }
	}

	/* ---- ① 关掉所有中断 + 清掉所有挂起 -----------------------------
	 *  ★ 为什么必须做：
	 *    · Bootloader 里可能开过中断（比如串口）
	 *    · 而跳过去之后，向量表才刚换
	 *      如果这时还有中断挂起 → 用旧的向量表去响应
	 *      → 直接跑飞 / HardFault
	 *  ★ ICER = 关中断使能，ICPR = 清挂起位
	 *     STM32F103 有 8 个（32 位 × 8 = 60 个中断）。 */
	__disable_irq();
	for (i = 0u; i < 8u; i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFFu;
		NVIC->ICPR[i] = 0xFFFFFFFFu;
	}

	/* ---- ② 关掉 SysTick -----------------------------------------------
	 *  ★ SysTick 是内核外设，它不属于 NVIC 的中断使能位
	 *  ★ 所以ICER 关不掉它，必须单独关 */
	SysTick->CTRL = 0u;
	SysTick->LOAD = 0u;
	SysTick->VAL  = 0u;

	/* ---- ③ 把向量表搬到 App 那边 --------------------------------------
	 *  ★ SCB->VTOR 是 Cortex-M3 的向量表偏移寄存器
	 *  ★ 这是整个方案能成立的硬件基础 ——
	 *    · 没有它 → 跳过去后中断还找 Bootloader 的表，必崩 */
	SCB->VTOR = APP_BASE;

	/* ---- ④ 把栈指针设成 App 的首字 ------------------------------------ */
	__set_MSP(appSp);

	/* ---- ④.5 跳之前先喂一次狗（2026-10-13 加）--------------------
	 *  ★ 为什么这一步不能省：
	 *    · App 的 WD_Init() 在很后面才调用
	 *      （它故意放在所有耗时初始化之后，见 main.c:534）
	 *    · 而这中间SD 初始化可能耗好几秒
	 *    · 如果刚才在 Bootloader 里一直没喂
	 *      → App 还没走到 WD_Init()，狗就先咬了
	 *    → 喂这一口，等于把完整的 6.55 秒交给 App
	 *  ★ 注：0xAAAA 在狗没启动时无害（不会把它启动起来） */
	BOOT_FEED_IWDG();

	/* ---- ⑤ 跳 -------------------------------------------------------- */
	appEntry();

	/* ★ 正常情况下不可能到这里（App 是死循环）
	 *   ★ 但万一 App 真的返回了 → 停住，总比跑飞好 */
	for (;;) { }
}
