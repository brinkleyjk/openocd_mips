/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2011 by Drasko DRASKOVIC                                *
 *   drasko.draskovic@gmail.com                                            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef MIPS32_H
#define MIPS32_H

#include "target.h"
#include "mips32_pracc.h"

#define MIPS32_COMMON_MAGIC		0xB320B320

/**
 * Memory segments (32bit kernel mode addresses)
 * These are the traditional names used in the 32-bit universe.
 */
#define KUSEG			0x00000000
#define KSEG0			0x80000000
#define KSEG1			0xa0000000
#define KSEG2			0xc0000000
#define KSEG3			0xe0000000

/** Returns the kernel segment base of a given address */
#define KSEGX(a)		((a) & 0xe0000000)

/** CP0 CONFIG regites fields */
#define MIPS32_CONFIG0_KU_SHIFT 25
#define MIPS32_CONFIG0_KU_MASK (0x7 << MIPS32_CONFIG0_KU_SHIFT)

#define MIPS32_CONFIG0_K0_SHIFT 0
#define MIPS32_CONFIG0_K0_MASK (0x7 << MIPS32_CONFIG0_K0_SHIFT)

#define MIPS32_CONFIG0_K23_SHIFT 28
#define MIPS32_CONFIG0_K23_MASK (0x7 << MIPS32_CONFIG0_K23_SHIFT)

#define MIPS32_CONFIG0_AR_SHIFT 10
#define MIPS32_CONFIG0_AR_MASK (0x7 << MIPS32_CONFIG0_AR_SHIFT)

#define MIPS32_CONFIG1_DL_SHIFT 10
#define MIPS32_CONFIG1_DL_MASK (0x7 << MIPS32_CONFIG1_DL_SHIFT)

#define MIPS32_ARCH_REL1 0x0
#define MIPS32_ARCH_REL2 0x1

#define MIPS32_SCAN_DELAY_LEGACY_MODE 2000000
 
/* offsets into mips32 core register cache */
enum {
	MIPS32_PC = 37,
	MIPS32NUMCOREREGS
};

enum mips32_isa_mode {
	MIPS32_ISA_MIPS32 = 0,
	MIPS32_ISA_MIPS16E = 1,
};

enum micro_mips_enabled {
	MIPS32_ONLY = 0,
	MICRO_MIPS_ONLY = 1,
	MICRO_MIPS32_16_ONRESET_MIPS32 = 2,
	MICRO_MIPS32_16_ONRESET_MIPS16 = 3,
};

struct mips32_comparator {
	int used;
	uint32_t bp_value;
	uint32_t reg_address;
};



struct mips32_common {
	uint32_t common_magic;
	void *arch_info;
	struct reg_cache *core_cache;
	struct mips_ejtag ejtag_info;
	uint32_t core_regs[MIPS32NUMCOREREGS];
	enum mips32_isa_mode isa_mode;
	enum micro_mips_enabled mmips;

	/* working area for fastdata access */
	struct working_area *fast_data_area;

	int bp_scanned;
	int num_inst_bpoints;
	int num_data_bpoints;
	int num_inst_bpoints_avail;
	int num_data_bpoints_avail;
	struct mips32_comparator *inst_break_list;
	struct mips32_comparator *data_break_list;

	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target *target, int num);
	int (*write_core_reg)(struct target *target, int num);
};

static inline struct mips32_common *
target_to_mips32(struct target *target)
{
	return target->arch_info;
}

struct mips32_core_reg {
	uint32_t num;
	struct target *target;
	struct mips32_common *mips32_common;
};

struct mips32_algorithm {
	int common_magic;
	enum mips32_isa_mode isa_mode;
};

#define zero	0

#define AT	1

#define v0 	2
#define v1	3

#define a0	4
#define a1	5
#define a2	6
#define	a3	7
#define t0	8
#define t1	9
#define t2	10
#define t3	11
#define t4	12
#define t5	13
#define t6	14
#define t7	15
#define ta0	12	/* alias for $t4 */
#define ta1	13	/* alias for $t5 */
#define ta2	14	/* alias for $t6 */
#define ta3	15	/* alias for $t7 */

#define s0	16
#define s1	17
#define s2	18
#define s3	19
#define s4	20
#define s5	21
#define s6	22
#define s7	23
#define s8	30		/* == fp */

#define t8	24
#define t9	25
#define k0	26
#define k1	27

#define gp	28

#define sp	29
#define fp	30
#define ra	31

#define ALL 0
#define INST 1
#define DATA 2
#define ALLNOWB 3
#define DATANOWB 4
#define L2 5

/*
 * MIPS32 Config0 Register  (CP0 Register 16, Select 0)
 */
#define CFG0_M          0x80000000      /* Config1 implemented */
#define CFG0_BE         0x00008000      /* Big Endian */
#define CFG0_ATMASK     0x00006000      /* Architecture type: */
#define CFG0_AT_M32     (0<<13)         /* MIPS32 */
#define CFG0_AT_M64_A32 (1<<13)         /* MIPS64, 32-bit addresses */
#define CFG0_AT_M64_A64 (2<<13)         /* MIPS64, 64-bit addresses */
#define CFG0_AT_RES     (3<<13)
#define CFG0_ARMASK     0x00001c00
#define CFG0_ARSHIFT    10
#define CFG0_MTMASK     0x00000380
#define CFG0_MT_NONE    (0<<7)
#define CFG0_MT_TLB     (1<<7)
#define CFG0_MT_BAT     (2<<7)
#define CFG0_MT_NONSTD  (3<<7)
#define CFG0_VI         0x00000008      /* Icache is virtual */
#define CFG0_K0MASK     0x00000007      /* KSEG0 coherency algorithm */

/*
 * MIPS32 Config1 Register (CP0 Register 16, Select 1)
 */
#define CFG1_M          0x80000000      /* Config2 implemented */
#define CFG1_MMUSMASK   0x7e000000      /* mmu size - 1 */
#define CFG1_MMUSSHIFT  25
#define CFG1_ISMASK     0x01c00000      /* icache lines 64<<n */
#define CFG1_ISSHIFT    22
#define CFG1_ILMASK     0x00380000      /* icache line size 2<<n */
#define CFG1_ILSHIFT    19
#define CFG1_IAMASK     0x00070000      /* icache ways - 1 */
#define CFG1_IASHIFT    16
#define CFG1_DSMASK     0x0000e000      /* dcache lines 64<<n */
#define CFG1_DSSHIFT    13
#define CFG1_DLMASK     0x00001c00      /* dcache line size 2<<n */
#define CFG1_DLSHIFT    10

/*
 * MIPS32 Config1 Register (CP0 Register 16, Select 1)
 */
#define CFG1_M          0x80000000      /* Config2 implemented */
#define CFG1_MMUSMASK   0x7e000000      /* mmu size - 1 */
#define CFG1_MMUSSHIFT  25
#define CFG1_ISMASK     0x01c00000      /* icache lines 64<<n */
#define CFG1_ISSHIFT    22
#define CFG1_ILMASK     0x00380000      /* icache line size 2<<n */
#define CFG1_ILSHIFT    19
#define CFG1_IAMASK     0x00070000      /* icache ways - 1 */
#define CFG1_IASHIFT    16
#define CFG1_DSMASK     0x0000e000      /* dcache lines 64<<n */
#define CFG1_DSSHIFT    13
#define CFG1_DLMASK     0x00001c00      /* dcache line size 2<<n */
#define CFG1_DLSHIFT    10
#define CFG1_DAMASK     0x00000380      /* dcache ways - 1 */
#define CFG1_DASHIFT    7
#define CFG1_C2         0x00000040      /* Coprocessor 2 present */
#define CFG1_MD         0x00000020      /* MDMX implemented */
#define CFG1_PC         0x00000010      /* performance counters implemented */
#define CFG1_WR         0x00000008      /* watch registers implemented */
#define CFG1_CA         0x00000004      /* compression (mips16) implemented */
#define CFG1_EP         0x00000002      /* ejtag implemented */
#define CFG1_FP         0x00000001      /* fpu implemented */

/*
 * MIPS32r2 Config2 Register (CP0 Register 16, Select 2)
 */
#define CFG2_M          0x80000000      /* Config3 implemented */
#define CFG2_TUMASK     0x70000000      /* tertiary cache control */
#define CFG2_TUSHIFT    28
#define CFG2_TSMASK     0x0f000000      /* tcache sets per wway 64<<n */
#define CFG2_TSSHIFT    24
#define CFG2_TLMASK     0x00f00000      /* tcache line size 2<<n */
#define CFG2_TLSHIFT    20
#define CFG2_TAMASK     0x000f0000      /* tcache ways - 1 */
#define CFG2_TASHIFT    16
#define CFG2_SUMASK     0x0000f000      /* secondary cache control */
#define CFG2_SUSHIFT    12
#define CFG2_SSMASK     0x00000f00      /* scache sets per wway 64<<n */
#define CFG2_SSSHIFT    8
#define CFG2_SLMASK     0x000000f0      /* scache line size 2<<n */
#define CFG2_SLSHIFT    4
#define CFG2_SAMASK     0x0000000f      /* scache ways - 1 */
#define CFG2_SASHIFT    0

/*
 * MIPS32r2 Config3 Register (CP0 Register 16, Select 3)
 */
#define CFG3_M          0x80000000      /* Config4 implemented */
#define CFG3_ISAONEXC   0x00010000      /* ISA mode on exception entry */
#define CFG3_DSPP       0x00000400      /* DSP ASE present */
#define CFG3_LPA        0x00000080      /* Large physical addresses */
#define CFG3_VEIC       0x00000040      /* Vectored external i/u controller */
#define CFG3_VI         0x00000020      /* Vectored i/us */
#define CFG3_SP         0x00000010      /* Small page support */
#define CFG3_MT         0x00000004      /* MT ASE present */
#define CFG3_SM         0x00000002      /* SmartMIPS ASE */
#define CFG3_TL         0x00000001      /* Trace Logic */

/*
 * Cache operations
 */
#define Index_Invalidate_I               0x00        /* 0       0 */
#define Index_Writeback_Inv_D            0x01        /* 0       1 */
#define Index_Writeback_Inv_T            0x02        /* 0       2 */
#define Index_Writeback_Inv_S            0x03        /* 0       3 */
#define Index_Load_Tag_I                 0x04        /* 1       0 */
#define Index_Load_Tag_D                 0x05        /* 1       1 */
#define Index_Load_Tag_T                 0x06        /* 1       2 */
#define Index_Load_Tag_S                 0x07        /* 1       3 */
#define Index_Store_Tag_I                0x08        /* 2       0 */
#define Index_Store_Tag_D                0x09        /* 2       1 */
#define Index_Store_Tag_T                0x0A        /* 2       2 */
#define Index_Store_Tag_S                0x0B        /* 2       3 */
#define Hit_Invalidate_I                 0x10        /* 4       0 */
#define Hit_Invalidate_D                 0x11        /* 4       1 */
#define Hit_Invalidate_T                 0x12        /* 4       2 */
#define Hit_Invalidate_S                 0x13        /* 4       3 */
#define Fill_I                           0x14        /* 5       0 */
#define Hit_Writeback_Inv_D              0x15        /* 5       1 */
#define Hit_Writeback_Inv_T              0x16        /* 5       2 */
#define Hit_Writeback_Inv_S              0x17        /* 5       3 */
#define Hit_Writeback_D                  0x19        /* 6       1 */
#define Hit_Writeback_T                  0x1A        /* 6       1 */
#define Hit_Writeback_S                  0x1B        /* 6       3 */
#define Fetch_Lock_I                     0x1C        /* 7       0 */
#define Fetch_Lock_D                     0x1D        /* 7       1 */

/*
 * MIPS32 Coprocessor 0 register numbers
 */
#define C0_INDEX        0
#define C0_INX          0
#define C0_RANDOM       1
#define C0_RAND         1
#define C0_ENTRYLO0     2
#define C0_TLBLO0       2
#define C0_ENTRYLO1     3
#define C0_TLBLO1       3
#define C0_CONTEXT      4
#define C0_CTXT         4
#define C0_PAGEMASK     5
#define C0_PAGEGRAIN    5,1
#define C0_WIRED        6
#define C0_HWRENA       7
#define C0_BADVADDR     8
#define C0_VADDR        8
#define C0_COUNT        9
#define C0_ENTRYHI      10
#define C0_TLBHI        10
#define C0_COMPARE      11
#define C0_STATUS       12
#define C0_SR           12
#define C0_INTCTL       12,1
#define C0_SRSCTL       12,2
#define C0_SRSMAP       12,3
#define C0_CAUSE        13
#define C0_CR           13
#define C0_EPC          14
#define C0_PRID         15
#define C0_EBASE        15,1
#define C0_CONFIG       16
#define C0_CONFIG0      16,0
#define C0_CONFIG1      16,1
#define C0_CONFIG2      16,2
#define C0_CONFIG3      16,3
#define C0_LLADDR       17
#define C0_WATCHLO      18
#define C0_WATCHHI      19
#define C0_DEBUG        23
#define C0_DEPC         24
#define C0_PERFCNT      25
#define C0_ERRCTL       26
#define C0_CACHEERR     27
#define C0_TAGLO        28
#define C0_ITAGLO       28
#define C0_DTAGLO       28,2
#define C0_TAGLO2       28,4
#define C0_DATALO       28,1
#define C0_IDATALO      28,1
#define C0_DDATALO      28,3
#define C0_DATALO2      28,5
#define C0_TAGHI        29
#define C0_ITAGHI		29
#define C0_DATAHI       29,1
#define C0_ERRPC        30
#define C0_DESAVE       31

#define MIPS32_OP_ADDIU 0x21
#define MIPS32_OP_ANDI	0x0C
#define MIPS32_OP_BEQ	0x04
#define MIPS32_OP_BGTZ	0x07
#define MIPS32_OP_BNE	0x05
#define MIPS32_OP_ADDI	0x08
#define MIPS32_OP_AND	0x24
#define MIPS32_OP_CACHE	0x2F
#define MIPS32_OP_COP0	0x10
#define MIPS32_OP_EXT   0x1F
#define MIPS32_OP_J		0x02
#define MIPS32_OP_JR	0x08
#define MIPS32_OP_LUI	0x0F
#define MIPS32_OP_LW	0x23
#define MIPS32_OP_LBU	0x24
#define MIPS32_OP_LHU	0x25
#define MIPS32_OP_MFHI	0x10
#define MIPS32_OP_MTHI	0x11
#define MIPS32_OP_MFLO	0x12
#define MIPS32_OP_MUL   0x2
#define MIPS32_OP_MTLO	0x13
#define MIPS32_OP_RDHWR 0x3B
#define MIPS32_OP_SB	0x28
#define MIPS32_OP_SH	0x29
#define MIPS32_OP_SW	0x2B
#define MIPS32_OP_ORI	0x0D
#define MIPS32_OP_XORI	0x0E
#define MIPS32_OP_XOR	0x26
#define MIPS32_OP_SLTU  0x2B
#define MIPS32_OP_SLLV  0x04
#define MIPS32_OP_SRL	0x03
#define MIPS32_OP_SYNCI	0x1F

#define MIPS32_OP_REGIMM	0x01
#define MIPS32_OP_SDBBP		0x3F
#define MIPS32_OP_SPECIAL	0x00
#define MIPS32_OP_SPECIAL2	0x07
#define MIPS32_OP_SPECIAL3	0x1F

#define MIPS32_COP0_MF	0x00
#define MIPS32_COP0_MT	0x04

#define MIPS32_R_INST(opcode, rs, rt, rd, shamt, funct) \
	(((opcode) << 26) | ((rs) << 21) | ((rt) << 16) | ((rd) << 11) | ((shamt) << 6) | (funct))
#define MIPS32_I_INST(opcode, rs, rt, immd) \
	(((opcode) << 26) | ((rs) << 21) | ((rt) << 16) | (immd))
#define MIPS32_J_INST(opcode, addr)	(((opcode) << 26) | (addr))

#define MIPS32_NOP						0
#define MIPS32_ADD(dst, src, tar)		MIPS32_R_INST(0, src, tar, dst,0, 32)
#define MIPS32_ADDI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ADDI, src, tar, val)
#define MIPS32_ADDIU(tar, src, val)		MIPS32_I_INST(9, src, tar, val)
#define MIPS32_ADDU(dst, src, tar)		MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_ADDIU)
#define MIPS32_AND(reg, off, val)		MIPS32_R_INST(0, off, val, reg, 0, MIPS32_OP_AND)
#define MIPS32_ANDI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ANDI, src, tar, val)
#define MIPS32_B(off)					MIPS32_BEQ(0, 0, off)
#define MIPS32_BEQ(src, tar, off)		MIPS32_I_INST(MIPS32_OP_BEQ, src, tar, off)
#define MIPS32_BGTZ(reg, off)			MIPS32_I_INST(MIPS32_OP_BGTZ, reg, 0, off)
#define MIPS32_BNE(src, tar, off)		MIPS32_I_INST(MIPS32_OP_BNE, src, tar, off)
#define MIPS32_CACHE(op, off, base)		MIPS32_I_INST(MIPS32_OP_CACHE, base, op, off)
#define MIPS32_EXT(dst, src, shf, sz)	MIPS32_R_INST(MIPS32_OP_EXT, src, dst, (sz-1), shf, 0)
#define MIPS32_J(tar)				    MIPS32_J_INST(MIPS32_OP_J, tar)
#define MIPS32_JR(reg)					MIPS32_R_INST(0, reg, 0, 0, 0, MIPS32_OP_JR)
#define MIPS32_MFC0(gpr, cpr, sel)		MIPS32_R_INST(MIPS32_OP_COP0, MIPS32_COP0_MF, gpr, cpr, 0, sel)
#define MIPS32_MOVE(dst, src)			MIPS32_R_INST(17, 16, 0, src, dst, 6)
#define MIPS32_MTC0(gpr, cpr, sel)		MIPS32_R_INST(MIPS32_OP_COP0, MIPS32_COP0_MT, gpr, cpr, 0, sel)
#define MIPS32_LBU(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LBU, base, reg, off)
#define MIPS32_LHU(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LHU, base, reg, off)
#define MIPS32_LUI(reg, val)			MIPS32_I_INST(MIPS32_OP_LUI, 0, reg, val)
#define MIPS32_LW(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LW, base, reg, off)
#define MIPS32_MFLO(reg)				MIPS32_R_INST(0, 0, 0, reg, 0, MIPS32_OP_MFLO)
#define MIPS32_MFHI(reg)				MIPS32_R_INST(0, 0, 0, reg, 0, MIPS32_OP_MFHI)
#define MIPS32_MTLO(reg)				MIPS32_R_INST(0, reg, 0, 0, 0, MIPS32_OP_MTLO)
#define MIPS32_MTHI(reg)				MIPS32_R_INST(0, reg, 0, 0, 0, MIPS32_OP_MTHI)
#define MIPS32_MUL(dst, src, t)			MIPS32_R_INST(28, src, t, dst, 0, MIPS32_OP_MUL)
#define MIPS32_OR(dst, src, val)		MIPS32_R_INST(0, src, val, dst, 0, 37)
#define MIPS32_ORI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ORI, src, tar, val)
#define MIPS32_XORI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_XORI, src, tar, val)
#define MIPS32_RDHWR(tar, dst)			MIPS32_R_INST(MIPS32_OP_SPECIAL3, 0, tar, dst, 0, MIPS32_OP_RDHWR)
#define MIPS32_SB(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SB, base, reg, off)
#define MIPS32_SH(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SH, base, reg, off)
#define MIPS32_SW(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SW, base, reg, off)
#define MIPS32_XOR(reg, val1, val2)		MIPS32_R_INST(0, val1, val2, reg, 0, MIPS32_OP_XOR)
#define MIPS32_SRL(reg, src, off)		MIPS32_R_INST(0, 0, src, reg, off, MIPS32_OP_SRL)
#define MIPS32_SLTU(dst, tar, src)		MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_SLTU)
#define MIPS32_SLLV(dst, tar, src)		MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_SLLV)
#define MIPS32_SYNCI(off, base)			MIPS32_I_INST(MIPS32_OP_REGIMM, base, MIPS32_OP_SYNCI, off)

#define MIPS32_SYNC			0xF
#define MIPS32_SYNCI_STEP	0x1	/* reg num od address step size to be used with synci instruction */

/**
 * Cache operations definietions
 * Operation field is 5 bits long :
 * 1) bits 1..0 hold cache type
 * 2) bits 4..2 hold operation code
 */
#define MIPS32_CACHE_D_HIT_WRITEBACK ((0x1 << 0) | (0x6 << 2))
#define MIPS32_CACHE_I_HIT_INVALIDATE ((0x0 << 0) | (0x4 << 2))

/* ejtag specific instructions */
#define MIPS32_DRET					0x4200001F
#define MIPS32_SDBBP				0x7000003F	/* MIPS32_J_INST(MIPS32_OP_SPECIAL2, MIPS32_OP_SDBBP) */
#define MIPS16_SDBBP				0xE801
//#define MICRO_MIPS32_SDBBP			0x0000DB7C
#define MICRO_MIPS32_SDBBP			0x000046C0
#define MICRO_MIPS_SDBBP			0x46C0

extern const struct command_registration mips32_command_handlers[];

int mips32_arch_state(struct target *target);

int mips32_init_arch_info(struct target *target,
		struct mips32_common *mips32, struct jtag_tap *tap);

int mips32_restore_context(struct target *target);
int mips32_save_context(struct target *target);

struct reg_cache *mips32_build_reg_cache(struct target *target);

int mips32_run_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		uint32_t entry_point, uint32_t exit_point,
		int timeout_ms, void *arch_info);

int mips32_configure_break_unit(struct target *target);

int mips32_enable_interrupts(struct target *target, int enable);

int mips32_examine(struct target *target);

int mips32_register_commands(struct command_context *cmd_ctx);

int mips32_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class);
int mips32_checksum_memory(struct target *target, uint32_t address,
		uint32_t count, uint32_t *checksum);
int mips32_blank_check_memory(struct target *target,
		uint32_t address, uint32_t count, uint32_t *blank);

int mips32_mark_reg_invalid (struct target *, int);
#endif	/*MIPS32_H*/
