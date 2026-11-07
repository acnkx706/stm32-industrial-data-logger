#include "stm32f10x.h"
#include "boot_config.h"
#include "boot_flash.h"
#include "boot_param.h"

#include <stddef.h>
#include <string.h>

/*==============================================================================
 *  ★ 4KB 参数区实现（U1/U2，2026-11-07）
 *
 *  ★ 为什么 valid 用半字 0xFF01：
 *    · STM32F1 的 Flash 编程最小单位是 16 位，没有"写一个字节"；
 *    · 擦除态 0xFFFF，写 0xFF01 ⇒ 只把低字节 0xFF 变成 0x01，
 *      高字节仍是 0xFF；这就是"0xFF→0x01 单次写"的正确实现。
 *  ★ 为什么先写 1KB 再写 valid：
 *    · 1KB 页一次擦、一次写、一次回读；
 *    · valid 最后写 ⇒ 任意时刻断电，这一槽要么无效，要么完整。
 *============================================================================*/

static uint8_t s_page[BOOT_FLASH_PAGE_SIZE];   /* ★ 1KB 页缓冲，静态，不占栈 */
static uint8_t s_active = 0u;                  /* ★ 0=A, 1=B */
static uint8_t s_have   = 0u;                  /* ★ 1 = s_active 是合法槽 */

static uint16_t Param_Crc16(const uint8_t *p, uint32_t n)
{
	uint16_t crc = 0u;
	uint32_t i;
	uint8_t  b;

	for (i = 0u; i < n; i++)
	{
		crc ^= (uint16_t)p[i];
		for (b = 0u; b < 8u; b++)
		{
			if ((crc & 1u) != 0u) { crc = (uint16_t)((crc >> 1) ^ 0x8408u); }
			else                  { crc = (uint16_t)(crc >> 1); }
		}
	}
	return crc;
}

/* ★ 只把 valid 半字从 0xFFFF 写成 0xFF01，并回读。
 *   ★ 为什么直接调 FLASH_ProgramHalfWord：
 *     BootFlash_WritePage() 是"整页写"，会把 valid 一起写成 0xFFFF；
 *     最后这一步必须单独做，才能保证"valid 最后写"。 */
static uint8_t Param_ProgramValid(uint32_t addr)
{
	uint8_t ok = 1u;

	__disable_irq();
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	if (FLASH_ProgramHalfWord(addr, PARAM_VALID) != FLASH_COMPLETE)
	{
		ok = 0u;
	}
	else if (*(volatile uint16_t *)addr != PARAM_VALID)
	{
		ok = 0u;                                /* ★ 回读，防"返回 OK 但没写进去" */
	}

	FLASH_Lock();
	__enable_irq();
	return ok;
}

static uint8_t Param_SlotValid(const ParamRec_t *r)
{
	if (r->valid != PARAM_VALID) { return 0u; }
	if (r->magic != PARAM_MAGIC) { return 0u; }
	if (r->crc != Param_Crc16((const uint8_t *)r, (uint32_t)offsetof(ParamRec_t, crc)))
	{
		return 0u;
	}
	return 1u;
}

void Param_Default(ParamRec_t *r)
{
	memset(r, 0, sizeof(*r));
	r->magic    = PARAM_MAGIC;
	r->version  = PARAM_VERSION;
	r->image_ok = 1u;                          /* ★ 迁移时把当前 App 当好的 */
	r->valid    = PARAM_ERASED;
}

uint8_t Param_Load(ParamRec_t *out)
{
	const ParamRec_t *a = (const ParamRec_t *)PARAM_SLOT_A;
	const ParamRec_t *b = (const ParamRec_t *)PARAM_SLOT_B;
	uint8_t va = Param_SlotValid(a);
	uint8_t vb = Param_SlotValid(b);

	if ((va != 0u) && (vb != 0u))
	{
		/* ★ 两个都合法 ⇒ 取 seq 大的；seq 相同取 A（不应该发生） */
		if (b->seq > a->seq) { *out = *b; s_active = 1u; }
		else                 { *out = *a; s_active = 0u; }
		s_have = 1u;
		return 1u;
	}
	if (va != 0u) { *out = *a; s_active = 0u; s_have = 1u; return 1u; }
	if (vb != 0u) { *out = *b; s_active = 1u; s_have = 1u; return 1u; }

	Param_Default(out);
	s_have = 0u;
	return 0u;
}

uint8_t Param_Get(ParamRec_t *out)
{
	return Param_Load(out);
}

uint8_t Param_Save(ParamRec_t *in)
{
	ParamRec_t rec;
	uint32_t   target;
	uint32_t   vaddr;
	uint32_t   seq;

	/* ★ 目标槽：有合法槽就写另一个；没有就写 A */
	target = (s_have != 0u) ? ((s_active == 0u) ? PARAM_SLOT_B : PARAM_SLOT_A)
	                        : PARAM_SLOT_A;

	rec = *in;
	rec.magic   = PARAM_MAGIC;
	rec.version = PARAM_VERSION;
	seq         = (s_have != 0u) ? (uint16_t)(in->seq + 1u) : 1u;
	rec.seq     = (uint32_t)seq;
	rec.crc     = Param_Crc16((const uint8_t *)&rec, (uint32_t)offsetof(ParamRec_t, crc));
	rec.valid   = PARAM_ERASED;                /* ★ 先写 0xFFFF，最后单独写 0xFF01 */

	memset(s_page, 0xFF, sizeof(s_page));
	memcpy(s_page, &rec, sizeof(rec));

	if (BootFlash_ErasePage(target) != BOOTFLASH_OK) { return 0u; }
	if (BootFlash_WritePage(target, s_page) != BOOTFLASH_OK) { return 0u; }
	if (BootFlash_VerifyPage(target, s_page) != BOOTFLASH_OK) { return 0u; }

	vaddr = target + (uint32_t)offsetof(ParamRec_t, valid);
	if (Param_ProgramValid(vaddr) == 0u) { return 0u; }

	s_active = (target == PARAM_SLOT_A) ? 0u : 1u;
	s_have   = 1u;

	in->seq   = rec.seq;
	in->crc   = rec.crc;
	in->valid = PARAM_VALID;
	return 1u;
}

uint8_t Param_SetFlag(uint32_t flag, uint8_t on)
{
	ParamRec_t r;

	(void)Param_Get(&r);
	if (on != 0u) { r.flags |= flag; }
	else          { r.flags &= ~flag; }
	return Param_Save(&r);
}

uint8_t Param_ClearFlags(void)
{
	ParamRec_t r;

	(void)Param_Get(&r);
	r.flags = 0u;
	return Param_Save(&r);
}

uint8_t Param_OnUpgrade(void)
{
	ParamRec_t r;

	(void)Param_Get(&r);
	r.image_ok   = 0u;
	r.boot_count = 0u;
	r.flags      = 0u;
	r.iwdg_retry = 0u;                         /* ★ 新升级，重新给满机会 */
	return Param_Save(&r);
}

uint8_t Param_OnNewImage(void)
{
	ParamRec_t r;

	(void)Param_Get(&r);
	r.image_ok   = 0u;
	r.boot_count = 0u;
	r.flags      = 0u;
	/* ★ 保留 iwdg_retry：恢复次数上限跨"恢复→试用"保留 */
	return Param_Save(&r);
}

uint8_t Param_ConfirmImage(void)
{
	ParamRec_t r;

	(void)Param_Get(&r);

	if ((r.image_ok == 1u) && (r.boot_count == 0u) &&
	    (r.flags == 0u) && (r.iwdg_retry == 0u))
	{
		return 1u;                             /* ★ 已确认 ⇒ 只读不写 */
	}

	r.image_ok   = 1u;
	r.boot_count = 0u;
	r.flags      = 0u;
	r.iwdg_retry = 0u;                         /* ★ 确认成功，清恢复次数 */
	return Param_Save(&r);
}

/*==============================================================================
 *  ★ 防回滚计数器（U5，2026-11-07）
 *
 *  ★ 语义：img_min_ver = "从此以后不再接受比它小的镜像"
 *  ★ 为什么只增不减：递减就等于没有防回滚，攻击者/误操作把计数器写小即可
 *  ★ 为什么升级成功才更新：装到一半失败时，旧版本还得能装回去
 *============================================================================*/
uint32_t Param_GetMinVer(void)
{
	ParamRec_t r;

	(void)Param_Get(&r);
	return r.img_min_ver;
}

uint8_t Param_SetMinVer(uint32_t ver)
{
	ParamRec_t r;

	(void)Param_Get(&r);

	/* ★ 只往上走：比现有值小就直接算成功，不写 flash */
	if (ver <= r.img_min_ver)
	{
		return 1u;
	}

	r.img_min_ver = ver;
	return Param_Save(&r);
}
