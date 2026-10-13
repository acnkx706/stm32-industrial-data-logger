#include "stm32f10x.h"
#include "boot_flash.h"

/*==============================================================================
 *  ★★★★ Flash 擦写实现（2026-10-13 建 ✓）理由详见 boot_flash.h
 *============================================================================*/

/* ★★★ 一次"写一页"的开场白 ✗ 三处都要做 ✗ 抽出来 ✓
 *   · ★ `__disable_irq()`   ✗ 见 boot_flash.h ① ✓
 *   · ★★ `FLASH_Unlock()`   ✗ 不解锁【编程/擦除会【被忽略】】✓
 *   · ★★★ 清三个标志 ✗ 尤其 **PGERR**
 *     → 【它【是"粘"的 ✗ 上一次的错【不清掉 ✗ 会一直报错】】✓✓ */
static void Flash_Begin(void)
{
	__disable_irq();
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
}

/* ★★ 收场 ✗ 加一次状态判读 ✓ */
static void Flash_End(void)
{
	FLASH_Lock();
	__enable_irq();
}

/* ★★★ 看一眼"刚才那次操作到底成功没有" ✗✓
 *   ★ 返回 0 = 成功 ✓；非 0 = 对应的错误码 ✓
 *   ★★ 为什么【两个都查**✗：
 *     · ★ `GetBank1Status()` ✗ —— ★【看 BSY / PGERR / WRPRTERR 的【合成结果】】✓
 *     · ★★ `GetFlagStatus()`  ✗ —— ★【单独看【写保护】✗ 因为它【最需要区别对待】】✓✓
 *       （★ 写保护 = 【配置问题 ✗ 重试一万次也没用】✗ 值得单独报 ✓）*/
static uint8_t Flash_CheckStatus(uint8_t err_generic, uint8_t err_wrp)
{
	if (FLASH_GetFlagStatus(FLASH_FLAG_WRPRTERR) != RESET)
	{
		return err_wrp;
	}
	if (FLASH_GetBank1Status() != FLASH_COMPLETE)
	{
		return err_generic;
	}
	return BOOTFLASH_OK;
}


/*==============================================================================
 *  ★★★★ 擦一个页
 *============================================================================*/
uint8_t BootFlash_ErasePage(uint32_t addr)
{
	uint8_t ok;

	Flash_Begin();

	/* ★ `FLASH_ErasePage` 内部【自己会等 BSY ✗ 并返回结果】✓
	 *   ★★ 但仍要【再查一遍标志**✗ 因为【返回值【只反映"完成没完成"】✓
	 *     → 【写保护 / 编程错【要靠标志位看】】✓✓ */
	if (FLASH_ErasePage(addr) != FLASH_COMPLETE)
	{
		ok = BOOTFLASH_ERR_ERASE;
	}
	else
	{
		ok = Flash_CheckStatus(BOOTFLASH_ERR_ERASE, BOOTFLASH_ERR_ERASE_WRP);
	}

	Flash_End();
	return ok;
}


/*==============================================================================
 *  ★★★★ 写满一页（1024 字节）
 *
 *  ★★ 为什么【按【半字】写**✗✓（★ 这是 F103 的硬约束 ✓）：
 *    · ★ F103 的 Flash 编程【最小单位 = 16 位（半字）】✗✓
 *    · ★★ 【没有"写一个字节"**✗ 也没有"写一个字"（32 位）✓
 *      （★ 那是 F4 才有的 PSIZE 配置 ✓）
 *    · ★★★ 【地址【必须是偶数】】✗ 且每次 **+2** ✓
 *  ★★★ 我们的起始地址 `0x08002000` 【天然偶对齐**✓
 *    但【循环里【还是要按 i+1 取高字节】✗ 不能想当然 ✓✓
 *
 *  ★★★★ 为什么【不用 `memcpy` 式的"按字搬"**✗：
 *    · ★ 缓冲是 `uint8_t[]` ✗ 【不保证 4 字节对齐】✓
 *    · ★★ 硬转 `uint32_t*` 去读 ✗ → 【非对齐访问 ✗ 在 CM3 上【直接 HardFault】】✓✓
 *    → ★★★ 【老老实实【一次两个字节 ✗ 拼成半字】**✓✓
 *============================================================================*/
uint8_t BootFlash_WritePage(uint32_t addr, const uint8_t *buf)
{
	uint8_t  ok = BOOTFLASH_OK;
	uint16_t i;
	uint16_t half;

	Flash_Begin();

	for (i = 0u; i < (uint16_t)BOOT_FLASH_PAGE_SIZE; i += 2u)
	{
		/* ★ 小端拼接 ✗：低字节在前 ✓
		 *   （★ 和"内存里看到的样子"一致 ✗ 也是 Flash 期望的顺序 ✓）*/
		half = (uint16_t)((uint16_t)buf[i] | ((uint16_t)buf[i + 1u] << 8));

		if (FLASH_ProgramHalfWord(addr + (uint32_t)i, half) != FLASH_COMPLETE)
		{
			ok = BOOTFLASH_ERR_WRITE;
			break;
		}

		/* ★★ 每个半字都查一遍 ✗
		 *   ★ 为什么【不只在最后查**✗：★【出错后【必须【立刻停 ✗
		 *     否则【一个坏半字后面【全是无效编程 ✗ 白白多花 1.4 秒】】✓✓ */
		ok = Flash_CheckStatus(BOOTFLASH_ERR_WRITE, BOOTFLASH_ERR_WRITE_WRP);
		if (ok != BOOTFLASH_OK)
		{
			break;
		}

		/* ★ 清掉"这次成功"留下的 EOP ✗
		 *   （★ 不清✗ 下一个半字会【误判成"上次有错"✗ 或者更糟 ✗ 被忽略 ✓）*/
		FLASH_ClearFlag(FLASH_FLAG_EOP);
	}

	Flash_End();
	return ok;
}


/*==============================================================================
 *  ★★★★ 回读比对
 *============================================================================*/
uint8_t BootFlash_VerifyPage(uint32_t addr, const uint8_t *buf)
{
	/* ★ 用 `volatile` 读 ✗ 防止编译器"聪明地"把它优化掉 ✓
	 *   ★★ 这是【真实存在的坑**✗：★ 非 volatile 的指针读 Flash ✗
	 *     【理论上编译器可以假设"它没被改过"✗ 从而【把整个循环删掉】】✓✓ */
	const volatile uint8_t *flash = (const volatile uint8_t *)addr;
	uint16_t i;

	for (i = 0u; i < (uint16_t)BOOT_FLASH_PAGE_SIZE; i++)
	{
		if (flash[i] != buf[i])
		{
			return BOOTFLASH_ERR_VERIFY;
		}
	}
	return BOOTFLASH_OK;
}
