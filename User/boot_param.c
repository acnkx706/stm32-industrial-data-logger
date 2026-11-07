#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "boot_param.h"

#include <stddef.h>
#include <string.h>

/*==============================================================================
 *  ★ 4KB 参数区（App 侧，U1/U2，2026-11-07）
 *
 *  ★ 和 Boot/boot_param.c 的逻辑必须一致：
 *    · 同一套地址、同一个结构、同一个 CRC、同一个 valid=0xFF01；
 *    · 区别只在 Flash 驱动：Boot 用 boot_flash.c，App 直接用标准库。
 *
 *  ★ App 只做两件事：
 *    · Param_ConfirmImage()：POST 通过后写 image_ok=1、boot_count=0；
 *    · Param_SetFlag()/Param_ClearFlags()：升级/回退请求（可选，BKP 仍保留）。
 *============================================================================*/

static uint8_t s_page[PARAM_SLOT_SIZE];        /* ★ 1KB 页缓冲，静态 */
static uint8_t s_active = 0u;
static uint8_t s_have   = 0u;

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
	r->image_ok = 1u;
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
	uint8_t    ok = 1u;
	uint16_t   i;

	target = (s_have != 0u) ? ((s_active == 0u) ? PARAM_SLOT_B : PARAM_SLOT_A)
	                        : PARAM_SLOT_A;

	rec = *in;
	rec.magic   = PARAM_MAGIC;
	rec.version = PARAM_VERSION;
	seq         = (s_have != 0u) ? (in->seq + 1u) : 1u;
	rec.seq     = seq;
	rec.crc     = Param_Crc16((const uint8_t *)&rec, (uint32_t)offsetof(ParamRec_t, crc));
	rec.valid   = PARAM_ERASED;

	memset(s_page, 0xFF, sizeof(s_page));
	memcpy(s_page, &rec, sizeof(rec));

	__disable_irq();
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	if (FLASH_ErasePage(target) != FLASH_COMPLETE)
	{
		ok = 0u;
	}

	if (ok != 0u)
	{
		for (i = 0u; i < (uint16_t)PARAM_SLOT_SIZE; i += 2u)
		{
			uint16_t half = (uint16_t)((uint16_t)s_page[i] |
			                           ((uint16_t)s_page[i + 1u] << 8));

			if (FLASH_ProgramHalfWord(target + (uint32_t)i, half) != FLASH_COMPLETE)
			{
				ok = 0u;
				break;
			}
			FLASH_ClearFlag(FLASH_FLAG_EOP);
		}
	}

	if (ok != 0u)
	{
		for (i = 0u; i < (uint16_t)PARAM_SLOT_SIZE; i++)
		{
			if (*(volatile uint8_t *)(target + (uint32_t)i) != s_page[i])
			{
				ok = 0u;
				break;
			}
		}
	}

	if (ok != 0u)
	{
		vaddr = target + (uint32_t)offsetof(ParamRec_t, valid);
		if (FLASH_ProgramHalfWord(vaddr, PARAM_VALID) != FLASH_COMPLETE)
		{
			ok = 0u;
		}
		else if (*(volatile uint16_t *)vaddr != PARAM_VALID)
		{
			ok = 0u;
		}
	}

	FLASH_Lock();
	__enable_irq();

	if (ok == 0u) { return 0u; }

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

uint8_t Param_OnNewImage(void)
{
	ParamRec_t r;

	(void)Param_Get(&r);
	r.image_ok   = 0u;
	r.boot_count = 0u;
	r.flags      = 0u;
	return Param_Save(&r);
}

uint8_t Param_ConfirmImage(void)
{
	ParamRec_t r;

	(void)Param_Get(&r);
	r.image_ok   = 1u;
	r.boot_count = 0u;
	r.flags      = 0u;
	return Param_Save(&r);
}
