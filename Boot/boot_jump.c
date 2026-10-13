#include "stm32f10x.h"
#include "boot_config.h"
#include "boot_jump.h"

/*==============================================================================
 *  ★★★★ Bootloader 核心：App 有效性检查 + 跳转（2026-10-13 建 ✓）
 *        设计参考 `stm32-bootloader-master`（★ Akos Pasztor ✓）
 *============================================================================*/

/* ★ 函数指针类型 ✗ 用来"取地址然后跳过去" ✓ */
typedef void (*Boot_AppEntry_t)(void);


/* ★★★ 检查 App 是否【看起来有效】✗✓
 *   只读两个字 ✗ 【极便宜 ✗ 但能挡住三种坏情况】✓✓
 *   判据详见 boot_jump.h ✓ */
uint8_t Boot_IsAppValid(void)
{
	/* ★ App 的首字 = 【初始栈顶指针】（★ 由链接器写进去的 ✓）*/
	uint32_t sp = *(__IO uint32_t *)APP_BASE;

	/* ★★ 第二个字 = 【复位向量】= App 第一条指令的地址 ✓ */
	uint32_t pc = *(__IO uint32_t *)(APP_BASE + 4u);

	if (BOOT_IS_SP_VALID(sp) == 0) { return 0u; }
	if (BOOT_IS_PC_VALID(pc) == 0) { return 0u; }

	return 1u;
}


/* ★★★★ 跳到 App ✗ 五步 ✗ 顺序不能换 ✓ */
void Boot_JumpToApp(void)
{
	uint32_t        appSp    = *(__IO uint32_t *)APP_BASE;
	Boot_AppEntry_t appEntry = (Boot_AppEntry_t)(*(__IO uint32_t *)(APP_BASE + 4u));
	uint8_t         i;

	/* ---- ① 关掉【所有】中断 + 清掉所有挂起 ✗✓ -----------------------------
	 *  ★ 为什么【必须】做 ✗：
	 *    · ★ Bootloader 里可能开过中断（★ 比如串口 ✓）
	 *    · ★★ 而【跳过去之后 ✗ 向量表才刚换 ✗
	 *      如果这时【还有中断挂起】✗ → 【用【旧的】向量表去响应】✓
	 *      → ★★★ 【直接跑飞 / HardFault】✓✓
	 *  ★★ ICER = 关中断使能 ✗ ICPR = 清挂起位 ✓
	 *     STM32F103 有 8 个（32 位 × 8 = 60 个中断）✓ */
	__disable_irq();
	for (i = 0u; i < 8u; i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFFu;
		NVIC->ICPR[i] = 0xFFFFFFFFu;
	}

	/* ---- ② 关掉 SysTick ✗✓ -----------------------------------------------
	 *  ★ SysTick 是【内核外设】✗ 它【不属于 NVIC 的中断使能位】✓
	 *  ★★ 所以【ICER 关不掉它 ✗ 必须【单独关】】✓✓ */
	SysTick->CTRL = 0u;
	SysTick->LOAD = 0u;
	SysTick->VAL  = 0u;

	/* ---- ③ 把向量表搬到 App 那边 ✗✓ --------------------------------------
	 *  ★ `SCB->VTOR` 是 Cortex-M3 的【向量表偏移寄存器】✓
	 *  ★★ 这是【整个方案能成立的【硬件基础】】✗✓ ——
	 *    · ★ 没有它 ✗ → 【跳过去后【中断还找 Bootloader 的表】✗ 必崩】✓✓ */
	SCB->VTOR = APP_BASE;

	/* ---- ④ 把栈指针设成 App 的首字 ✗✓ ------------------------------------ */
	__set_MSP(appSp);

	/* ---- ④.5 【跳之前先喂一次狗】✗✓（2026-10-13 加 ✓）--------------------
	 *  ★★ 为什么【这一步不能省】✗：
	 *    · ★ App 的 `WD_Init()` 【在【很后面】才调用】✓
	 *      （★ 它故意放在【所有耗时初始化之后】✗ 见 `main.c:534` ✓）
	 *    · ★★ 而【这中间【SD 初始化可能耗好几秒】】✗
	 *    · ★★★ 如果【刚才在 Bootloader 里【一直没喂**✗
	 *      → 【App 还没走到 `WD_Init()` ✗ 狗就先咬了】✓✓
	 *    → ★★★★ 【喂这一口 ✗ 等于【把完整的 6.55 秒【交给 App】】✓✓
	 *  ★ 注：★ `0xAAAA` 在【狗没启动时【无害】✓（★ 不会把它启动起来 ✓）*/
	BOOT_FEED_IWDG();

	/* ---- ⑤ 跳 ✗✓ -------------------------------------------------------- */
	appEntry();

	/* ★★ 正常情况下【不可能到这里】✗（★ App 是死循环 ✓）
	 *   ★ 但万一 App 真的返回了 ✗ → 【停住 ✗ 总比跑飞好】✓✓ */
	for (;;) { }
}
