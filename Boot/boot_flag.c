#include "stm32f10x.h"
#include "boot_flag.h"

/*==============================================================================
 *  ★★★★ 升级请求标志的实现（2026-10-13 建 ✓）理由详见 boot_flag.h
 *============================================================================*/

/* ★ 解锁备份域 ✗✓
 *  ★ 做法【照抄 App 的 `MyRTC.c:28-29`】✗（★ 那边也是这么干的 ✓）：
 *    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
 *    PWR_BackupAccessCmd(ENABLE);
 *
 *  ★★ 为什么【两步都要 ✗✓】：
 *    · ★ 【PWR 时钟】✗ —— ★ `PWR_BackupAccessCmd` 这个函数本身要它 ✓
 *    · ★★ 【BKP 时钟】✗ —— ★ 读写 `BKP_DRx` 寄存器要它 ✓
 *    · ★★★ 【解锁】✗ —— ★★ 【STM32 默认【禁止】写备份域】✓
 *      → 【不调它 ✗ `BKP_WriteBackupRegister` 【静默失败】】✓✓
 *        （★ 不报错 ✗ 但值【没写进去】✗ 很难查 ✓）*/
void BootFlag_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
}


uint8_t BootFlag_IsUpgradeRequested(void)
{
	return (BKP_ReadBackupRegister(BOOT_FLAG_DR) == BOOT_FLAG_MAGIC) ? 1u : 0u;
}


void BootFlag_ClearUpgrade(void)
{
	/* ★ 写 0 ✗ 而不是写"另一个魔数" ✓
	 *   ★★ 理由：★ 0 【不可能**等于 `BOOT_FLAG_MAGIC` ✓
	 *     → 【判断逻辑【最简单 ✗ 不会出错】】✓✓ */
	BKP_WriteBackupRegister(BOOT_FLAG_DR, 0x0000u);
}


void BootFlag_RequestUpgrade(void)
{
	BKP_WriteBackupRegister(BOOT_FLAG_DR, BOOT_FLAG_MAGIC);
}


/* ---- 回退请求（★ 2026-10-14 加 ✓）---------------------------------------- */

uint8_t BootFlag_IsRollbackRequested(void)
{
	return (BKP_ReadBackupRegister(BOOT_ROLLBACK_DR) == BOOT_ROLLBACK_MAGIC) ? 1u : 0u;
}


void BootFlag_ClearRollback(void)
{
	/* ★ 同 `BootFlag_ClearUpgrade` ✗ 写 0 ✗ 【不可能等于魔数】✓ */
	BKP_WriteBackupRegister(BOOT_ROLLBACK_DR, 0x0000u);
}
