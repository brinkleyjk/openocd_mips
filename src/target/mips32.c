/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                             *
 *   oyvind.harboe@zylin.com                                               *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "mips32.h"
#include "breakpoints.h"
#include "algorithm.h"
#include "register.h"

static const char *mips_isa_strings[] = {
    "MIPS32", "MIPS16"
};

static const struct {
    unsigned option;
    const char *arg;
} invalidate_cmd[5] = {
    { ALL, "all", },
    { INST, "inst", },
    { DATA, "data", },
    { ALLNOWB, "allnowb", },
    { DATANOWB, "datanowb", },
};

static const struct {
    unsigned id;
    const char *name;
} mips32_regs[MIPS32NUMCOREREGS] = {
    { 0, "zero", },
    { 1, "at", },
    { 2, "v0", },
    { 3, "v1", },
    { 4, "a0", },
    { 5, "a1", },
    { 6, "a2", },
    { 7, "a3", },
    { 8, "t0", },
    { 9, "t1", },
    { 10, "t2", },
    { 11, "t3", },
    { 12, "t4", },
    { 13, "t5", },
    { 14, "t6", },
    { 15, "t7", },
    { 16, "s0", },
    { 17, "s1", },
    { 18, "s2", },
    { 19, "s3", },
    { 20, "s4", },
    { 21, "s5", },
    { 22, "s6", },
    { 23, "s7", },
    { 24, "t8", },
    { 25, "t9", },
    { 26, "k0", },
    { 27, "k1", },
    { 28, "gp", },
    { 29, "sp", },
    { 30, "fp", },
    { 31, "ra", },

    { 32, "status", },
    { 33, "lo", },
    { 34, "hi", },
    { 35, "badvaddr", },
    { 36, "cause", },
    { 37, "pc" },
};

 static const struct {
    unsigned reg;
    const char *name;
} mips32_dsp_regs[MIPS32NUMDSPREGS] = {
	{ 0, "hi1"},
	{ 1, "hi2"},
	{ 2, "hi3"},
	{ 3, "lo1"},
	{ 4, "lo2"},
	{ 5, "lo3"},
	{ 6, "control"},
};

/* number of mips dummy fp regs fp0 - fp31 + fsr and fir
 * we also add 18 unknown registers to handle gdb requests */

#define MIPS32NUMFPREGS (34 + 18)

static uint8_t mips32_gdb_dummy_fp_value[] = {0, 0, 0, 0};

static struct reg mips32_gdb_dummy_fp_reg = {
    .name = "GDB dummy floating-point register",
    .value = mips32_gdb_dummy_fp_value,
    .dirty = 0,
    .valid = 1,
    .size = 32,
    .arch_info = NULL,
};

/* WAYS MAPPING */
static const int wayTable[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};       /* field->ways mapping */
static const int setTableISDS[] = {64,128,256,512,1024,2048,4096,32,		  /* field->sets mapping */
16*1024, 32*1024, 64*1024, 128*1024, 256*1024, 512*1024, 1024*1024, 2048*1024};
static const int setTable[] = {64,128,256,512,1024,2048,4096,8192,			  /* field->sets mapping */
16*1024, 32*1024, 64*1024, 128*1024, 256*1024, 512*1024, 1024*1024, 2048*1024};

/* BPL */
static const int bplTable[] = {0,4,8,16,32,64,128,256,512,1024,2048,4*1024,8*1024,16*1024,32*1024,64*1024}; /* field->bytes per line */
static const int bplbitTable[] = {0,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};									/* field->bits in bpl */

static int mips32_get_core_reg(struct reg *reg)
{
    int retval;
    struct mips32_core_reg *mips32_reg = reg->arch_info;
    struct target *target = mips32_reg->target;
    struct mips32_common *mips32_target = target_to_mips32(target);

    if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

    retval = mips32_target->read_core_reg(target, mips32_reg->num);

    return retval;
}

static int mips32_set_core_reg(struct reg *reg, uint8_t *buf)
{
    struct mips32_core_reg *mips32_reg = reg->arch_info;
    struct target *target = mips32_reg->target;
    uint32_t value = buf_get_u32(buf, 0, 32);

    if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

    buf_set_u32(reg->value, 0, 32, value);
    reg->dirty = 1;
    reg->valid = 1;

    return ERROR_OK;
}

static int mips32_read_core_reg(struct target *target, int num)
{
    uint32_t reg_value;

    /* get pointers to arch-specific information */
    struct mips32_common *mips32 = target_to_mips32(target);

    if ((num < 0) || (num >= MIPS32NUMCOREREGS))
		return ERROR_COMMAND_SYNTAX_ERROR;

    reg_value = mips32->core_regs[num];
    buf_set_u32(mips32->core_cache->reg_list[num].value, 0, 32, reg_value);
    mips32->core_cache->reg_list[num].valid = 1;
    mips32->core_cache->reg_list[num].dirty = 0;

    return ERROR_OK;
}

static int mips32_write_core_reg(struct target *target, int num)
{
    uint32_t reg_value;

    /* get pointers to arch-specific information */
    struct mips32_common *mips32 = target_to_mips32(target);

    if ((num < 0) || (num >= MIPS32NUMCOREREGS))
		return ERROR_COMMAND_SYNTAX_ERROR;

    reg_value = buf_get_u32(mips32->core_cache->reg_list[num].value, 0, 32);
    mips32->core_regs[num] = reg_value;
    LOG_DEBUG("write core reg %i value 0x%" PRIx32 "", num , reg_value);
    mips32->core_cache->reg_list[num].valid = 1;
    mips32->core_cache->reg_list[num].dirty = 0;

    return ERROR_OK;
}

int mips32_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
			    int *reg_list_size, enum target_register_class reg_class)
{
    /* get pointers to arch-specific information */
    struct mips32_common *mips32 = target_to_mips32(target);
    int i;

	/* include floating point registers */
	*reg_list_size = MIPS32NUMCOREREGS + MIPS32NUMFPREGS;

    *reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

    for (i = 0; i < MIPS32NUMCOREREGS; i++)
		(*reg_list)[i] = &mips32->core_cache->reg_list[i];

    /* add dummy floating points regs */
    for (i = MIPS32NUMCOREREGS; i < (MIPS32NUMCOREREGS + MIPS32NUMFPREGS); i++)
		(*reg_list)[i] = &mips32_gdb_dummy_fp_reg;

    return ERROR_OK;
}

int mips32_save_context(struct target *target)
{
    int i;

    /* get pointers to arch-specific information */
    struct mips32_common *mips32 = target_to_mips32(target);
    struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

    /* read core registers */
    int retval = mips32_pracc_read_regs(ejtag_info, mips32->core_regs);
	if (retval != ERROR_OK) {
		LOG_DEBUG ("mips32_pracc_read_regs failed");
		return retval;
	}
	
    for (i = 0; i < MIPS32NUMCOREREGS; i++) {
		if (!mips32->core_cache->reg_list[i].valid) {
			retval = mips32->read_core_reg(target, i);
			if (retval != ERROR_OK) {
				LOG_DEBUG ("mips32->read_core_reg failed");
				return retval;
			}
		}
    }

    return ERROR_OK;
}

int mips32_restore_context(struct target *target)
{
    int i;

    /* get pointers to arch-specific information */
    struct mips32_common *mips32 = target_to_mips32(target);
    struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

    for (i = 0; i < MIPS32NUMCOREREGS; i++) {
	if (mips32->core_cache->reg_list[i].dirty)
	    mips32->write_core_reg(target, i);
    }

    /* write core regs */
    mips32_pracc_write_regs(ejtag_info, mips32->core_regs);

    return ERROR_OK;
}

int mips32_arch_state(struct target *target)
{
    struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	int retval;

	/* Read config3 to determine if core is MIPS32/MIPS16 */
	uint32_t config3;
	uint32_t cp0_reg = 16;
	uint32_t cp0_sel = 3;
	retval = mips32_cp0_read(ejtag_info, &config3, cp0_reg, cp0_sel);
	if (retval != ERROR_OK) {
		LOG_DEBUG ("reading config3 register failed");
		return retval;
	}

	mips32->dsp_implemented = ((config3 & CFG3_DSPP) >>  10);
	mips32->dsp_rev = ((config3 & CFG3_DSP_REV) >>  11);
	mips32->mmips = ((config3 & CFG3_ISA_MODE) >>  14);

    LOG_USER("target halted in %s mode due to %s, pc: 0x%8.8" PRIx32 "",
	     mips_isa_strings[mips32->isa_mode],
	     debug_reason_name(target),
	     buf_get_u32(mips32->core_cache->reg_list[MIPS32_PC].value, 0, 32));

    return ERROR_OK;
}

static const struct reg_arch_type mips32_reg_type = {
    .get = mips32_get_core_reg,
    .set = mips32_set_core_reg,
};

struct reg_cache *mips32_build_reg_cache(struct target *target)
{
    /* get pointers to arch-specific information */
    struct mips32_common *mips32 = target_to_mips32(target);

    int num_regs = MIPS32NUMCOREREGS;
    struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
    struct reg_cache *cache = malloc(sizeof(struct reg_cache));
    struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
    struct mips32_core_reg *arch_info = malloc(sizeof(struct mips32_core_reg) * num_regs);
    int i;

    register_init_dummy(&mips32_gdb_dummy_fp_reg);

    /* Build the process context cache */
    cache->name = "mips32 registers";
    cache->next = NULL;
    cache->reg_list = reg_list;
    cache->num_regs = num_regs;
    (*cache_p) = cache;
    mips32->core_cache = cache;

    for (i = 0; i < num_regs; i++) {
		arch_info[i].num = mips32_regs[i].id;
		arch_info[i].target = target;
		arch_info[i].mips32_common = mips32;

		reg_list[i].name = mips32_regs[i].name;
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].type = &mips32_reg_type;
		reg_list[i].arch_info = &arch_info[i];
    }

    return cache;
}

int mips32_init_arch_info(struct target *target, struct mips32_common *mips32, struct jtag_tap *tap)
{
    target->arch_info = mips32;
    mips32->common_magic = MIPS32_COMMON_MAGIC;
    mips32->fast_data_area = NULL;

    /* has breakpoint/watchpint unit been scanned */
    mips32->bp_scanned = 0;
    mips32->data_break_list = NULL;

    mips32->ejtag_info.tap = tap;
    mips32->read_core_reg = mips32_read_core_reg;
    mips32->write_core_reg = mips32_write_core_reg;

    mips32->ejtag_info.scan_delay = 2000000;	/* Initial default value */
    mips32->ejtag_info.mode = 0;			/* Initial default value */

    return ERROR_OK;
}

/* run to exit point. return error if exit point was not reached. */
static int mips32_run_and_wait(struct target *target, uint32_t entry_point,
			       int timeout_ms, uint32_t exit_point, struct mips32_common *mips32)
{
    uint32_t pc;
    int retval;

    /* This code relies on the target specific  resume() and  poll()->debug_entry()
     * sequence to write register values to the processor and the read them back */
    retval = target_resume(target, 0, entry_point, 0, 1);
    if (retval != ERROR_OK)
		return retval;

    retval = target_wait_state(target, TARGET_HALTED, timeout_ms);
    /* If the target fails to halt due to the breakpoint, force a halt */
    if (retval != ERROR_OK || target->state != TARGET_HALTED) {
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;
		retval = target_wait_state(target, TARGET_HALTED, 500);
		if (retval != ERROR_OK)
			return retval;

		return ERROR_TARGET_TIMEOUT;
    }

    pc = buf_get_u32(mips32->core_cache->reg_list[MIPS32_PC].value, 0, 32);
    if (exit_point && (pc != exit_point)) {
		LOG_DEBUG("failed algorithm halted at 0x%" PRIx32 " ", pc);
		return ERROR_TARGET_TIMEOUT;
    }

    return ERROR_OK;
}

int mips32_run_algorithm(struct target *target, int num_mem_params,
			 struct mem_param *mem_params, int num_reg_params,
			 struct reg_param *reg_params, uint32_t entry_point,
			 uint32_t exit_point, int timeout_ms, void *arch_info)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips32_algorithm *mips32_algorithm_info = arch_info;
	enum mips32_isa_mode isa_mode = mips32->isa_mode;

	uint32_t context[MIPS32NUMCOREREGS];
	int i;
	int retval = ERROR_OK;

	LOG_DEBUG("Running algorithm");

	/* NOTE: mips32_run_algorithm requires that each algorithm uses a software breakpoint
	 * at the exit point */

	if (mips32->common_magic != MIPS32_COMMON_MAGIC) {
		LOG_ERROR("current target isn't a MIPS32 target");
		return ERROR_TARGET_INVALID;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* refresh core register cache */
	for (i = 0; i < MIPS32NUMCOREREGS; i++) {
		if (!mips32->core_cache->reg_list[i].valid)
			mips32->read_core_reg(target, i);

		context[i] = buf_get_u32(mips32->core_cache->reg_list[i].value, 0, 32);
	}

	for (i = 0; i < num_mem_params; i++) {
		retval = target_write_buffer(target, mem_params[i].address,
									 mem_params[i].size, mem_params[i].value);
		if (retval != ERROR_OK)
			return retval;
	}

	for (i = 0; i < num_reg_params; i++) {
		struct reg *reg = register_get_by_name(mips32->core_cache, reg_params[i].reg_name, 0);

		if (!reg) {
			LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (reg->size != reg_params[i].size) {
			LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
					  reg_params[i].reg_name);

			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		mips32_set_core_reg(reg, reg_params[i].value);
	}

	mips32->isa_mode = mips32_algorithm_info->isa_mode;

	retval = mips32_run_and_wait(target, entry_point, timeout_ms, exit_point, mips32);
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction != PARAM_OUT) {
			retval = target_read_buffer(target, mem_params[i].address, mem_params[i].size,
																	mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	for (i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction != PARAM_OUT) {
			struct reg *reg = register_get_by_name(mips32->core_cache, reg_params[i].reg_name, 0);
			if (!reg) {
				LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			if (reg->size != reg_params[i].size) {
				LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
									reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			buf_set_u32(reg_params[i].value, 0, 32, buf_get_u32(reg->value, 0, 32));
		}
	}

	/* restore everything we saved before */
	for (i = 0; i < MIPS32NUMCOREREGS; i++) {
		uint32_t regvalue;
		regvalue = buf_get_u32(mips32->core_cache->reg_list[i].value, 0, 32);
		if (regvalue != context[i]) {
			LOG_DEBUG("restoring register %s with value 0x%8.8" PRIx32,
								mips32->core_cache->reg_list[i].name, context[i]);

			buf_set_u32(mips32->core_cache->reg_list[i].value,
									0, 32, context[i]);
			mips32->core_cache->reg_list[i].valid = 1;
			mips32->core_cache->reg_list[i].dirty = 1;
		}
	}

	mips32->isa_mode = isa_mode;

	return ERROR_OK;
}

int mips32_examine(struct target *target)
{
    struct mips32_common *mips32 = target_to_mips32(target);

    if (!target_was_examined(target)) {
		target_set_examined(target);

		/* we will configure later */
		mips32->bp_scanned = 0;
		mips32->num_inst_bpoints = 0;
		mips32->num_data_bpoints = 0;
		mips32->num_inst_bpoints_avail = 0;
		mips32->num_data_bpoints_avail = 0;
    }

    return ERROR_OK;
}

static int mips32_configure_ibs(struct target *target)
{
    struct mips32_common *mips32 = target_to_mips32(target);
    struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
    int retval, i;
    uint32_t bpinfo;

    /* get number of inst breakpoints */
    retval = target_read_u32(target, ejtag_info->ejtag_ibs_addr, &bpinfo);
    if (retval != ERROR_OK)
		return retval;

    mips32->num_inst_bpoints = (bpinfo >> 24) & 0x0F;
    mips32->num_inst_bpoints_avail = mips32->num_inst_bpoints;
    mips32->inst_break_list = calloc(mips32->num_inst_bpoints,
									 sizeof(struct mips32_comparator));

    for (i = 0; i < mips32->num_inst_bpoints; i++)
		mips32->inst_break_list[i].reg_address =
			ejtag_info->ejtag_iba0_addr +
			(ejtag_info->ejtag_iba_step_size * i);

    /* clear IBIS reg */
    retval = target_write_u32(target, ejtag_info->ejtag_ibs_addr, 0);
    return retval;
}

static int mips32_configure_dbs(struct target *target)
{
    struct mips32_common *mips32 = target_to_mips32(target);
    struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
    int retval, i;
    uint32_t bpinfo;

    /* get number of data breakpoints */
    retval = target_read_u32(target, ejtag_info->ejtag_dbs_addr, &bpinfo);
    if (retval != ERROR_OK)
		return retval;

    mips32->num_data_bpoints = (bpinfo >> 24) & 0x0F;
    mips32->num_data_bpoints_avail = mips32->num_data_bpoints;
    mips32->data_break_list = calloc(mips32->num_data_bpoints,
									 sizeof(struct mips32_comparator));

    for (i = 0; i < mips32->num_data_bpoints; i++)
		mips32->data_break_list[i].reg_address =
			ejtag_info->ejtag_dba0_addr +
			(ejtag_info->ejtag_dba_step_size * i);

    /* clear DBIS reg */
    retval = target_write_u32(target, ejtag_info->ejtag_dbs_addr, 0);
    return retval;
}

int mips32_configure_break_unit(struct target *target)
{
    /* get pointers to arch-specific information */
    struct mips32_common *mips32 = target_to_mips32(target);
    struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
    int retval;
    uint32_t dcr;

    if (mips32->bp_scanned)
		return ERROR_OK;

    /* get info about breakpoint support */
    retval = target_read_u32(target, EJTAG_DCR, &dcr);
    if (retval != ERROR_OK)
		return retval;

	/* EJTAG 2.0 defines IB and DB bits in IMP instead of DCR. */
	if (ejtag_info->ejtag_version == EJTAG_VERSION_20) {
		ejtag_info->debug_caps = dcr & EJTAG_DCR_ENM;
		if (!(ejtag_info->impcode & EJTAG_V20_IMP_NOIB))
			ejtag_info->debug_caps |= EJTAG_DCR_IB;
		if (!(ejtag_info->impcode & EJTAG_V20_IMP_NODB))
			ejtag_info->debug_caps |= EJTAG_DCR_DB;
	} else
		/* keep  debug caps for later use */
		ejtag_info->debug_caps = dcr & (EJTAG_DCR_ENM
				| EJTAG_DCR_IB | EJTAG_DCR_DB);

    if (ejtag_info->debug_caps & EJTAG_DCR_IB) {
		retval = mips32_configure_ibs(target);
		if (retval != ERROR_OK)
			return retval;
    }

    if (ejtag_info->debug_caps & EJTAG_DCR_DB) {
		retval = mips32_configure_dbs(target);
		if (retval != ERROR_OK)
			return retval;
    }

    /* check if target endianness settings matches debug control register */
    if (((ejtag_info->debug_caps & EJTAG_DCR_ENM) && (target->endianness == TARGET_LITTLE_ENDIAN)) ||
		(!(ejtag_info->debug_caps & EJTAG_DCR_ENM) && (target->endianness == TARGET_BIG_ENDIAN))){
		LOG_WARNING("DCR endianness settings does not match target settings");
		LOG_WARNING("Config file does not match DCR endianness");
	}

    LOG_DEBUG("DCR 0x%" PRIx32 " numinst %i numdata %i", dcr, mips32->num_inst_bpoints,
			  mips32->num_data_bpoints);

    mips32->bp_scanned = 1;

    return ERROR_OK;
}

int mips32_enable_interrupts(struct target *target, int enable)
{
    int retval;
    int update = 0;
    uint32_t dcr;

    /* read debug control register */
    retval = target_read_u32(target, EJTAG_DCR, &dcr);
    if (retval != ERROR_OK)
		return retval;

    if (enable) {
		if (!(dcr & EJTAG_DCR_INTE)) {
			/* enable interrupts */
			dcr |= EJTAG_DCR_INTE;
			update = 1;
		}
    } else {
		if (dcr & EJTAG_DCR_INTE) {
			/* disable interrupts */
			dcr &= ~EJTAG_DCR_INTE;
			update = 1;
		}
    }

    if (update) {
		retval = target_write_u32(target, EJTAG_DCR, dcr);
		if (retval != ERROR_OK)
			return retval;
    }

    return ERROR_OK;
}

int mips32_checksum_memory(struct target *target, uint32_t address,
			   uint32_t count, uint32_t *checksum)
{
    struct working_area *crc_algorithm;
    struct reg_param reg_params[2];
    struct mips32_algorithm mips32_info;

    /* see contib/loaders/checksum/mips32.s for src */

    static const uint32_t mips_crc_code[] = {
		0x248C0000,		/* addiu	$t4, $a0, 0 */
		0x24AA0000,		/* addiu	$t2, $a1, 0 */
		0x2404FFFF,		/* addiu	$a0, $zero, 0xffffffff */
		0x10000010,		/* beq		$zero, $zero, ncomp */
		0x240B0000,		/* addiu	$t3, $zero, 0 */

		/* nbyte: */
		0x81850000,		/* lb		$a1, ($t4) */
		0x218C0001,		/* addi		$t4, $t4, 1 */
		0x00052E00,		/* sll		$a1, $a1, 24 */
		0x3C0204C1,		/* lui		$v0, 0x04c1 */
		0x00852026,		/* xor		$a0, $a0, $a1 */
		0x34471DB7,		/* ori		$a3, $v0, 0x1db7 */
		0x00003021,		/* addu		$a2, $zero, $zero */
		/* loop: */
		0x00044040,		/* sll		$t0, $a0, 1 */
		0x24C60001,		/* addiu	$a2, $a2, 1 */
		0x28840000,		/* slti		$a0, $a0, 0 */
		0x01074826,		/* xor		$t1, $t0, $a3 */
		0x0124400B,		/* movn		$t0, $t1, $a0 */
		0x28C30008,		/* slti		$v1, $a2, 8 */
		0x1460FFF9,		/* bne		$v1, $zero, loop */
		0x01002021,		/* addu		$a0, $t0, $zero */

		/* ncomp: */
		0x154BFFF0,		/* bne		$t2, $t3, nbyte */
		0x256B0001,		/* addiu	$t3, $t3, 1 */
		0x7000003F,		/* sdbbp */
    };

    /* make sure we have a working area */
    if (target_alloc_working_area(target, sizeof(mips_crc_code), &crc_algorithm) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

    /* convert flash writing code into a buffer in target endianness */
	uint8_t mips_crc_code_8[sizeof(mips_crc_code)];
	target_buffer_set_u32_array(target, mips_crc_code_8,
					ARRAY_SIZE(mips_crc_code), mips_crc_code);

	target_write_buffer(target, crc_algorithm->address, sizeof(mips_crc_code), mips_crc_code_8);

    mips32_info.common_magic = MIPS32_COMMON_MAGIC;
    mips32_info.isa_mode = MIPS32_ISA_MIPS32;

    init_reg_param(&reg_params[0], "a0", 32, PARAM_IN_OUT);
    buf_set_u32(reg_params[0].value, 0, 32, address);

    init_reg_param(&reg_params[1], "a1", 32, PARAM_OUT);
    buf_set_u32(reg_params[1].value, 0, 32, count);

    int timeout = (20000 * (1 + (count / (1024 * 1024))) * 2);

    int retval = target_run_algorithm(target, 0, NULL, 2, reg_params,
				  crc_algorithm->address, crc_algorithm->address + (sizeof(mips_crc_code)-4), timeout,
				  &mips32_info);

	if (retval == ERROR_OK)
		*checksum = buf_get_u32(reg_params[0].value, 0, 32);

    destroy_reg_param(&reg_params[0]);
    destroy_reg_param(&reg_params[1]);

    target_free_working_area(target, crc_algorithm);

    return retval;
}

/** Checks whether a memory region is zeroed. */
int mips32_blank_check_memory(struct target *target,
			      uint32_t address, uint32_t count, uint32_t *blank)
{
    struct working_area *erase_check_algorithm;
    struct reg_param reg_params[3];
    struct mips32_algorithm mips32_info;

    static const uint32_t erase_check_code[] = {
		/* nbyte: */
		0x80880000,		/* lb		$t0, ($a0) */
		0x00C83024,		/* and		$a2, $a2, $t0 */
		0x24A5FFFF,		/* addiu	$a1, $a1, -1 */
		0x14A0FFFC,		/* bne		$a1, $zero, nbyte */
		0x24840001,		/* addiu	$a0, $a0, 1 */
		0x7000003F		/* sdbbp */
    };

    /* make sure we have a working area */
    if (target_alloc_working_area(target, sizeof(erase_check_code), &erase_check_algorithm) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* convert erase check code into a buffer in target endianness */
	uint8_t erase_check_code_8[sizeof(erase_check_code)];
	target_buffer_set_u32_array(target, erase_check_code_8,
					ARRAY_SIZE(erase_check_code), erase_check_code);

	target_write_buffer(target, erase_check_algorithm->address, sizeof(erase_check_code), erase_check_code_8);

    mips32_info.common_magic = MIPS32_COMMON_MAGIC;
    mips32_info.isa_mode = MIPS32_ISA_MIPS32;

    init_reg_param(&reg_params[0], "a0", 32, PARAM_OUT);
    buf_set_u32(reg_params[0].value, 0, 32, address);

    init_reg_param(&reg_params[1], "a1", 32, PARAM_OUT);
    buf_set_u32(reg_params[1].value, 0, 32, count);

    init_reg_param(&reg_params[2], "a2", 32, PARAM_IN_OUT);
    buf_set_u32(reg_params[2].value, 0, 32, 0xff);

    int retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
								  erase_check_algorithm->address,
								  erase_check_algorithm->address + (sizeof(erase_check_code)-4),
								  10000, &mips32_info);
	if (retval == ERROR_OK)
		*blank = buf_get_u32(reg_params[2].value, 0, 32);

    destroy_reg_param(&reg_params[0]);
    destroy_reg_param(&reg_params[1]);
    destroy_reg_param(&reg_params[2]);

    target_free_working_area(target, erase_check_algorithm);

    return retval;
}

static int mips32_verify_pointer(struct command_context *cmd_ctx,
				 struct mips32_common *mips32)
{
    if (mips32->common_magic != MIPS32_COMMON_MAGIC) {
		command_print(cmd_ctx, "target is not an MIPS32");
		return ERROR_TARGET_INVALID;
    }
    return ERROR_OK;
}

uint32_t DetermineCpuTypeFromPrid(uint32_t prid, uint32_t config, uint32_t config1) {

	uint32_t cpuType;
	/* Determine CPU type from PRID. */
	if (((prid >> 16) & 0xff) == 16)
		/* Altera */
		return (uint32_t)MIPS_MP32;

	if (((prid >> 16) & 0xff) == 2)
		/* Broadcom */
		return (uint32_t) MIPS_BCM;

	if (((prid >> 16) & 0xff) == 3) {
		/* AMD Alchemy processors */
		switch ((prid >> 24) & 0xff)
		{
			case 0x00:
				cpuType = MIPS_AU1000;
				break;

			case 0x01:
				cpuType = MIPS_AU1500;
				break;

			case 0x02:
				cpuType = MIPS_AU1100;
				break;

			case 0x03:
				cpuType = MIPS_AU1550;
				break;

			case 0x04:
				cpuType = MIPS_AU1200;
				break;

			default:
				cpuType = CPUTYPE_UNKNOWN;
				break;
		} /* end of switch */

		return cpuType;
	}

	switch ((prid >> 8) & 0xff)
	{    /* MIPS Technologies cores */
		case 0x80:
			cpuType = MIPS_4Kc;
			break;

		case 0x81:
			if (config1 & 1)
				cpuType = MIPS_5Kf;       /* fpu present */
			else 
				cpuType = MIPS_5Kc;
			break;

		case 0x82:
			cpuType = MIPS_20Kc;
			break;

		case 0x83:
			if ((config >> 20) & 1)
				cpuType = MIPS_4Kp;
			else
				cpuType = MIPS_4Km;
			break;
					   
		case 0x84:
		case 0x90:
			cpuType = MIPS_4KEc;
			break;

		case 0x85:
		case 0x91:
			if ((config >> 20) & 1) 
				cpuType = MIPS_4KEp;
			else
				cpuType = MIPS_4KEm;
			break;
					   
		case 0x86:
			cpuType = MIPS_4KSc;
			break;

		case 0x87:
			cpuType = MIPS_M4K;
			break;

		case 0x88:
			cpuType = MIPS_25Kf;
			break;

		case 0x89:
			if (config1 & 1)
				cpuType = MIPS_5KEf;       /* fpu present */
			else
				cpuType = MIPS_5KEc;
			break;

		case 0x92:
			cpuType = MIPS_4KSd;
			break;

		case 0x93:
			if (config1 & 1)
				cpuType = MIPS_24Kf;       /* fpu present */
			else
				cpuType = MIPS_24Kc;
			break;

		case 0x95:
			if (config1 & 1)
				cpuType = MIPS_34Kf;       /* fpu present */
			else {
				/* In MT with a single-threaded FPU, Config1.FP may be 0   */
				/* even though an FPU exists.  Scan all TC contexts and if */
				/* any have Config1.FP, then set processor to 100Kf.       */
				/* skip it for now										   */
				cpuType = MIPS_34Kc;
			}
			break;

		case 0x96:
			if (config1 & 1)
				cpuType = MIPS_24KEf;       /* fpu present */
			else
				cpuType = MIPS_24KEc;
			break;

		case 0x97:
			if (config1 & 1)
				cpuType = MIPS_74Kf;       /* fpu present */
			else 
				cpuType = MIPS_74Kc;
			break;

		case 0x99:
			if (config1 & 1) 
				cpuType = MIPS_1004Kf;       /* fpu present */
			else {
				/* In MT with a single-threaded FPU, Config1.FP may be 0   */
				/* even though an FPU exists.  Scan all TC contexts and if */
				/* any have Config1.FP, then set processor to 100Kf.	   */
				/* skip it for now										   */
				cpuType = MIPS_1004Kc;
			}
			break;

		case 0x9A:
			if (config1 & 1)
				cpuType = MIPS_1074Kf;       /* fpu present */
			else {
				/* In MT with a single-threaded FPU, Config1.FP may be 0   */
				/* even though an FPU exists.  Scan all TC contexts and if */
				/* any have Config1.FP, then set processor to 100Kf.	   */
				/* skip it for now										   */
				cpuType = MIPS_1074Kc;
			}
			break;

		case 0x9B:
			cpuType = MIPS_M14K;
			break;

		case 0x9C:
			if (config1& 1)
				cpuType = MIPS_M14Kf;       /* fpu present */
			else
				cpuType = MIPS_M14Kc;
			break;
			   
		case 0x9D:
			if (config1 & 1)
				cpuType = MIPS_M14KEf;
			else
				cpuType = MIPS_M14KE;
			break;

		case 0x9E:
			if (config1 & 1)
				cpuType = MIPS_M14KEcf;
			else 
				cpuType = MIPS_M14KEc;
			break;

		case 0xA0:
			cpuType = MIPS_INTERAPTIV;
			break;

		case 0xA1:
			cpuType = MIPS_INTERAPTIV_CM;
			break;

		case 0xA2:
			cpuType = MIPS_PROAPTIV;
			break;

		case 0xA3:
			cpuType = MIPS_PROAPTIV_CM;
			break;

		case 0xA6:
			cpuType = MIPS_M5100;
			break;

		case 0xA7:
			cpuType = MIPS_M5150;
			break;

		case 0xA8:
			cpuType = MIPS_P5600;
			break;

		case 0xA9:
			cpuType = MIPS_I5500;
			break;

		default:
			cpuType = CPUTYPE_UNKNOWN;
			break;
	} /* end of switch */
	
	return (cpuType);
}

/**
 * MIPS32 targets expose command interface
 * to manipulate CP0 registers
 */
COMMAND_HANDLER(mips32_handle_cp0_command)
{
    int retval;
    struct target *target = get_current_target(CMD_CTX);
    struct mips32_common *mips32 = target_to_mips32(target);
    struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

    retval = mips32_verify_pointer(CMD_CTX, mips32);
    if (retval != ERROR_OK)
		return retval;

    if (target->state != TARGET_HALTED) {
		command_print(CMD_CTX, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
    }

    /* two or more argument, access a single register/select (write if third argument is given) */
    if (CMD_ARGC < 2){
		uint32_t value;

		if (CMD_ARGC == 0){
			for (int i = 0; i < MIPS32NUMCP0REGS; i++){
				retval = mips32_cp0_read(ejtag_info, &value, mips32_cp0_regs[i].reg, mips32_cp0_regs[i].sel);
				if (retval != ERROR_OK) {
					command_print(CMD_CTX, "couldn't access reg %s", mips32_cp0_regs[i].name);
					return ERROR_OK;
				}

				command_print(CMD_CTX, "%*s: 0x%8.8x", 14, mips32_cp0_regs[i].name, value);
			}
		}
		else {

			for (int i = 0; i < MIPS32NUMCP0REGS; i++){
				/* find register name */
				if (strcmp(mips32_cp0_regs[i].name, CMD_ARGV[0]) == 0){
					retval = mips32_cp0_read(ejtag_info, &value, mips32_cp0_regs[i].reg, mips32_cp0_regs[i].sel);
					command_print(CMD_CTX, "0x%8.8x", value);
					return ERROR_OK;
				}
			}

			LOG_ERROR("BUG: register '%s' not found", CMD_ARGV[0]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}
    else {
		if (CMD_ARGC == 2) {
			uint32_t value;
			char tmp = *CMD_ARGV[0];

			if (isdigit (tmp) == false) {
				
				for (int i = 0; i < MIPS32NUMCP0REGS; i++){
					/* find register name */
					if (strcmp(mips32_cp0_regs[i].name, CMD_ARGV[0]) == 0){
						COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);
						retval = mips32_cp0_write(ejtag_info, value, mips32_cp0_regs[i].reg, mips32_cp0_regs[i].sel);
						return ERROR_OK;
					}
				}

				LOG_ERROR("BUG: register '%s' not found", CMD_ARGV[0]);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
			else {
				uint32_t cp0_reg, cp0_sel;

				COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], cp0_reg);
				COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], cp0_sel);

				retval = mips32_cp0_read(ejtag_info, &value, cp0_reg, cp0_sel);
				if (retval != ERROR_OK) {
					command_print(CMD_CTX,
								  "couldn't access reg %" PRIi32,
								  cp0_reg);
					return ERROR_OK;
				}

				command_print(CMD_CTX, "cp0 reg %" PRIi32 ", select %" PRIi32 ": %8.8" PRIx32,
							  cp0_reg, cp0_sel, value);
			}
		} else if (CMD_ARGC == 3) {
			uint32_t cp0_reg, cp0_sel;
			uint32_t value;

			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], cp0_reg);
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], cp0_sel);
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], value);
			retval = mips32_cp0_write(ejtag_info, value, cp0_reg, cp0_sel);
			if (retval != ERROR_OK) {
				command_print(CMD_CTX,
							  "couldn't access cp0 reg %" PRIi32 ", select %" PRIi32,
							  cp0_reg,  cp0_sel);
				return ERROR_OK;
			}
			command_print(CMD_CTX, "cp0 reg %" PRIi32 ", select %" PRIi32 ": %8.8" PRIx32,
						  cp0_reg, cp0_sel, value);
		}
    }

    return ERROR_OK;
}

/**
 * MIPS32/microMips targets expose command interface
 * to manipulate DSP registers if supported.
 */
COMMAND_HANDLER(mips32_handle_dsp_command)
{
    int retval;
    struct target *target = get_current_target(CMD_CTX);
    struct mips32_common *mips32 = target_to_mips32(target);
    struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

    retval = mips32_verify_pointer(CMD_CTX, mips32);
    if (retval != ERROR_OK)
		return retval;

    if (target->state != TARGET_HALTED) {
		command_print(CMD_CTX, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
    }

	/* Check if DSP access supported or not */
	if ((mips32->mmips != MIPS32_ONLY) && (mips32->dsp_implemented == DSP_NOT_IMP)) {

		/* Issue Error Message */
		command_print(CMD_CTX, "DSP not implemented by this processor");
		return ERROR_OK;
	}

	if (mips32->dsp_rev != DSP_REV2) {
		command_print(CMD_CTX, "only DSP Rev 2 supported by this processor");
		return ERROR_OK;
	}

    /* two or more argument, access a single register/select (write if third argument is given) */
    if (CMD_ARGC < 2){
		uint32_t value;

		if (CMD_ARGC == 0){
			value = 0;
			for (int i = 0; i < MIPS32NUMDSPREGS; i++){
				retval = mips32_pracc_read_dsp_regs(ejtag_info, &value, mips32_dsp_regs[i].reg);
				if (retval != ERROR_OK) {
					command_print(CMD_CTX, "couldn't access reg %s", mips32_dsp_regs[i].name);
					return retval;
				}
				command_print(CMD_CTX, "%*s: 0x%8.8x", 7, mips32_dsp_regs[i].name, value);
			}
		}
		else {
			value = 0;
			for (int i = 0; i < MIPS32NUMDSPREGS; i++){
				/* find register name */
				if (strcmp(mips32_dsp_regs[i].name, CMD_ARGV[0]) == 0){
					retval = mips32_pracc_read_dsp_regs(ejtag_info, &value, mips32_dsp_regs[i].reg);
					command_print(CMD_CTX, "0x%8.8x", value);
					return retval;
				}
			}

			LOG_ERROR("BUG: register '%s' not found", CMD_ARGV[0]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}
    else {
		if (CMD_ARGC == 2) {
			uint32_t value;
			char tmp = *CMD_ARGV[0];

			if (isdigit (tmp) == false) {
				
				for (int i = 0; i < MIPS32NUMCP0REGS; i++){
					/* find register name */
					if (strcmp(mips32_dsp_regs[i].name, CMD_ARGV[0]) == 0){
						COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);
						retval = mips32_pracc_write_dsp_regs (ejtag_info, value, mips32_dsp_regs[i].reg);
						return retval;
					}
				}

				LOG_ERROR("BUG: register '%s' not found", CMD_ARGV[0]);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
		} else if (CMD_ARGC == 3) {
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
    }

    return ERROR_OK;
}

/**
 * MIPS32 targets expose command interface
 * to query core and provide configuration
 * details.
 */
COMMAND_HANDLER(mips32_handle_cpuinfo_command)
{
    int retval;
    struct target *target = get_current_target(CMD_CTX);
    struct mips32_common *mips32 = target_to_mips32(target);
    struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

	CPU_INFO info;

	/* Will be added in the future */
#if 0
	/* MMU types */
	uint32_t MMU_TLB = 0;
	uint32_t MMU_BAT = 0;
	uint32_t MMU_FMT = 0;
	uint32_t MMU_RPU = 0;
	uint32_t MMU_TLB_RPU = 0;
#endif

	char text[40]={0};
	uint32_t ways, sets, bpl;

	uint32_t    prid; /* cp0 PRID - 15, 0 */
	uint32_t  config; /*	cp0 config - 16, 0 */
	uint32_t config1; /*	cp0 config - 16, 1 */
	uint32_t config2; /*	cp0 config - 16, 2 */
	uint32_t config3; /*	cp0 config - 16, 3 */
	uint32_t config4; /*	cp0 config - 16, 4 */
	uint32_t config5; /*	cp0 config - 16, 5 */
	uint32_t config7; /*	cp0 config - 16, 7 */

	if (target->state != TARGET_HALTED) {
		command_print(CMD_CTX, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	/* No arg.s for now */
	if (CMD_ARGC >= 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Read PRID and config registers */
	if ((retval = mips32_cp0_read(ejtag_info, &prid, 15, 0)) != ERROR_OK)
		return retval;

	/* Read Config, Config(1,2,3,5 and 7) registers */
	if ((retval = mips32_cp0_read(ejtag_info, &config, 16, 0))!= ERROR_OK)
		return retval;

	if ((retval = mips32_cp0_read(ejtag_info, &config1, 16, 1))!= ERROR_OK)
		return retval;

	if ((retval = mips32_cp0_read(ejtag_info, &config2, 16, 2))!= ERROR_OK)
		return retval;

	if ((retval = mips32_cp0_read(ejtag_info, &config3, 16, 3))!= ERROR_OK)
		return retval;

	if ((retval = mips32_cp0_read(ejtag_info, &config4, 16, 4))!= ERROR_OK)
		return retval;

	if ((retval = mips32_cp0_read(ejtag_info, &config5, 16, 5))!= ERROR_OK)
		return retval;

	if ((retval = mips32_cp0_read(ejtag_info, &config7, 16, 7))!= ERROR_OK)
		return retval;

	/* Read and store dspase and mtase (and other) architecture extension presence bits */
	info.dspase = (config3 & 0x00000400) ? 1 : 0;		/* dsp ase */
	info.mtase  = (config3 & 0x00000004) ? 1 : 0;		/* multithreading */

	/* If multithreading then need to get more info */
	/* TODO List */
#if 0
	if (info.mtase) {

	}
#endif

	info.smase  = (config3 & 0x00000002) ? 1 : 0;		/* smartmips ase */
	info.m16ase = (config1 & 0x00000004) ? 1 : 0;       /* mips16(e) ase */
	info.micromipsase = ((config3 >> 14) & 0x3) != 0;
	info.vzase = (config3 & (1<<23)) ? 1 : 0;			/* VZ */

	/* Check if Virtualization supported */
		/* TODO List */
#if 0
	if (info.vzase) {
		/* Core supports Virtualization - now get Guest Info */
		uint32_t width;
		uint32_t guestCtl0;

		if ((retval = mips32_cp0_read(ejtag_info, &questCt10, 12, 6)) != ERROR_OK)
			return retval;

		cpuinfo.guestCtl1Present = GUESTCTL0_G1_READ(guestCtl0);
		
		if ((err = DetermineGuestIdWidth(handle, &width)) != HDI_SUCCESS) {
			return err;
		}
		cpuinfo.vzGuestIdWidth = width;
   }
#endif

	/* MIPS� SIMD Architecture (MSA) */
	info.msa = (config3 & 0x10000000) ? 1 : 0;

	info.mvh = (config5 & (1<<5)) ? 1 : 0;		/* mvh */

	/* MMU Supported */
	info.mmuType = (config >> 7) & 7;
	info.tlbEntries = 0;
#if 0
	/*MMU types */
	if (info.mmuType == )
		MMU_TLB =  ((MMUTYPE == 1) || (MMUTYPE == 4));

	if (cpuinfo.mmuType)
		MMU_BAT =  (MMUTYPE == 2);

	if (cpuinfo.mmuType)
		MMU_FMT =  (MMUTYPE == 3);

	if (cpuinfo.mmuType)
		MMU_RPU =  ((MMUTYPE == 3) && info.vzase);

	if (cpuinfo.mmuType)
		MMU_TLB_RPU = (MMU_TLB || MMU_RPU);
#endif

	/* If release 2 of Arch. then get exception base info */
	if (((config >> 10) & 7) != 0) {    /* release 2 */
		uint32_t  ebase;
		if ((retval = mips32_cp0_read(ejtag_info, &ebase, 15, 1))!= ERROR_OK)
			return retval;

		info.cpuid = (uint32_t)(ebase & 0x1ff);
	} else {
		info.cpuid = 0;
	}

	info.cpuType = DetermineCpuTypeFromPrid(prid, config, config1);

	/* Determine Core info */
	switch (info.cpuType) {
      case MIPS_4Kc:
		  info.cpuCore = MIPS_4Kc;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "4Kc");
		  break;

      case MIPS_4Km:
		  info.cpuCore = MIPS_4Km;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "4Km");
		  break;

      case MIPS_4Kp:
		  info.cpuCore = MIPS_4Kp;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "4Kp");
		  break;

      case MIPS_4KEc:
		  info.cpuCore = MIPS_4KEc;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "4KEc");		  
		  break;

      case MIPS_4KEm:
		  info.cpuCore = MIPS_4KEm;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "4KEm");
		  break;
      case MIPS_4KEp:
		  info.cpuCore = MIPS_4KEp;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "4KEp");
		  break;

      case MIPS_4KSc:
		  info.cpuCore = MIPS_4KSc;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "4KSc");
		  break;

      case MIPS_4KSd:
		  info.cpuCore = MIPS_4KSd;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "4KSd");
		  break;

      case MIPS_M4K:
		  info.cpuCore = MIPS_M4K;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "4K");
		  break;

      case MIPS_24Kc:
		  info.cpuCore = MIPS_24Kc;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "24Kc");
		  break;

      case MIPS_24Kf:
		  info.cpuCore = MIPS_24Kf;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "24Kf");
		  break;

      case MIPS_24KEc:
		  info.cpuCore = MIPS_24KEc;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "24KEc");
		  break;

      case MIPS_24KEf:
		  info.cpuCore = MIPS_24KEf;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "24KEf");
		  break;

      case MIPS_34Kc:
		  info.cpuCore = MIPS_34Kc;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "34Kc");
		  break;

      case MIPS_34Kf:
		  info.cpuCore = MIPS_34Kf;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "3Kf ");
		  break;

      case MIPS_5Kc:
		  info.cpuCore = MIPS_5Kc;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS64;
		  strcpy(text, "5Kc");
		  break;

      case MIPS_5Kf:
		  info.cpuCore = MIPS_5Kf;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS64;
		  strcpy(text, "5Kf");
		  break;

      case MIPS_5KEc:
		  info.cpuCore = MIPS_5KEc;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS64;
		  strcpy(text, "5KEc");
		  break;

      case MIPS_5KEf:
		  info.cpuCore = MIPS_5KEf;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS64;
		  strcpy(text, "5KEf");
		  break;

      case MIPS_20Kc:
		  info.cpuCore = MIPS_20Kc;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS64;
		  strcpy(text, "20Kc");
		  break;

      case MIPS_25Kf:
		  info.cpuCore = MIPS_25Kf;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS64;
		  strcpy(text, "25Kf");
		  break;

      case MIPS_AU1000:
		  info.cpuCore = MIPS_AU1000;
		  info.vendor = ALCHEMY_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "AU1000");
		  break;
      case MIPS_AU1100:
		  info.cpuCore = MIPS_AU1100;
		  info.vendor = ALCHEMY_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "AU1100");
		  break;

      case MIPS_AU1200:
		  info.cpuCore = MIPS_AU1200;
		  info.vendor = ALCHEMY_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "AU1200");
		  break;

      case MIPS_AU1500:
		  info.cpuCore = MIPS_AU1500;
		  info.vendor = ALCHEMY_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "AU1500");
		  break;

      case MIPS_AU1550:
		  info.cpuCore = MIPS_AU1550;
		  info.vendor = ALCHEMY_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "AU1550");
		  break;

      case MIPS_74Kc:
		  info.cpuCore = MIPS_74Kc;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "74Kc");
		  break;

      case MIPS_74Kf:
		  info.cpuCore = MIPS_74Kf;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "74Kf");
		  break;

      case MIPS_84Kc:
		  info.cpuCore = MIPS_84Kc;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "84Kc");
		  break;

      case MIPS_84Kf:
		  info.cpuCore = MIPS_84Kf;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "84Kf");
		  break;

      case MIPS_M14K:
		  info.cpuCore = MIPS_M14K;
		  info.vendor = MIPS_CORE;
		  /*		  if ((err = GetM14KInstSet(handle, &info.instSet)) != SUCCESS) return err; */
		  strcpy(text, "M14K");
		  break;

      case MIPS_M14Kc:
		  info.cpuCore = MIPS_M14Kc;
		  info.vendor = MIPS_CORE;
		  /*		  if ((err = GetM14KInstSet(handle, &info.instSet)) != SUCCESS) return err; */
		  strcpy(text, "M14Kc");
		  break;

      case MIPS_M14Kf:
		  info.cpuCore = MIPS_M14Kf;
		  info.vendor = MIPS_CORE;
		  /*		  if ((err = GetM14KInstSet(handle, &info.instSet)) != SUCCESS) return err; */
		  strcpy(text, "M14Kf");
		  break;

      case MIPS_M14KE:
		  info.cpuCore = MIPS_M14KE;
		  info.vendor = MIPS_CORE;
		  /*		  if ((err = GetM14KInstSet(handle, &info.instSet)) != SUCCESS) return err; */
		  strcpy(text, "microAptiv_UC");
		  break;

      case MIPS_M14KEf:
		  info.cpuCore = MIPS_M14KEf;
		  info.vendor = MIPS_CORE;
		  /*		  if ((err = GetM14KInstSet(handle, &info.instSet)) != SUCCESS) return err; */
		  strcpy(text, "microAptiv_UCF");
		  break;

      case MIPS_M14KEc:
		  info.cpuCore = MIPS_M14KEc;
		  info.vendor = MIPS_CORE;
		  /*		  if ((err = GetM14KInstSet(handle, &info.instSet)) != SUCCESS) return err; */
		  strcpy(text, "microAptiv_UP");
		  break;

      case MIPS_M14KEcf:
		  info.cpuCore = MIPS_M14KEcf;
		  info.vendor = MIPS_CORE;
		  /*		  if ((err = GetM14KInstSet(handle, &info.instSet)) != SUCCESS) return err; */
		  strcpy(text, "microAptiv_UPF");
		  break;

      case MIPS_M5100:
		  info.cpuCore = MIPS_M5100;
		  info.vendor = MIPS_CORE;
		  /*		  if ((err = GetM14KInstSet(handle, &info.instSet)) != SUCCESS) return err; */
		  strcpy(text, "M5100");
		  break;

      case MIPS_M5150:
		  info.cpuCore = MIPS_M5150;
		  info.vendor = MIPS_CORE;
		  /*		  if ((err = GetM14KInstSet(handle, &info.instSet)) != SUCCESS) return err; */
		  strcpy(text, "M5150");
		  break;

      case MIPS_BCM:
		  info.cpuCore = MIPS_BCM;
		  info.vendor = BROADCOM_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "BCM");
		  break;

      case MIPS_MP32:
		  info.cpuCore = MIPS_MP32;
		  info.vendor = ALTERA_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "MP32");
		  break;

      case MIPS_1004Kc:
		  info.cpuCore = MIPS_1004Kc;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "1004Kc");
		  break;

      case MIPS_1004Kf:
		  info.cpuCore = MIPS_1004Kf;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "1004Kf");
		  break;

      case MIPS_1074Kc:
		  info.cpuCore = MIPS_1074Kc;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "1074Kc");
		  break;

      case MIPS_1074Kf:
		  info.cpuCore = MIPS_1074Kf;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "1074Kf");
		  break;

      case MIPS_PROAPTIV:
		  info.cpuCore = MIPS_PROAPTIV;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "PROAPTIV");
		  break;

      case MIPS_PROAPTIV_CM:
		  info.cpuCore = MIPS_PROAPTIV_CM;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "PROAPTIV_CM");
		  break;

      case MIPS_INTERAPTIV:
		  info.cpuCore = MIPS_INTERAPTIV;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "INTERAPTIV");
		  break;

      case MIPS_INTERAPTIV_CM:
		  info.cpuCore = MIPS_INTERAPTIV_CM;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "INTERAPTIV_CM");
		  break;

      case MIPS_P5600:
		  info.cpuCore = MIPS_P5600;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "P5600");
		  break;

      case MIPS_I5500:
		  info.cpuCore = MIPS_I5500;
		  info.vendor = MIPS_CORE;
		  info.instSet = MIPS32;
		  strcpy(text, "I5500");
		  break;
	}

	/* Determine Instr Cache Size */
	ways = wayTable[(config1 >> CFG1_IASHIFT) & 7];
	sets = setTableISDS[(config1 >> CFG1_ISSHIFT) & 7];
	bpl  = bplTable[(config1 >> CFG1_ILSHIFT) & 7];
	info.iCacheSize = ways*sets*bpl;

	/* Determine data cache size */
	ways = wayTable[(config1 >>  CFG1_DASHIFT) & 7];
	sets = setTableISDS[(config1 >> CFG1_DSSHIFT) & 7];
	bpl  = bplTable[(config1 >> CFG1_DLSHIFT) & 7];
	info.dCacheSize = ways*sets*bpl;

	/* Display Core Type info */
	LOG_USER ("cpuCore: MIPS_%s", &text[0]);

	LOG_USER ("cputype: %d", info.cpuType);

	/* Display Core Vendor ID */
	switch (info.vendor) {
		case MIPS_CORE:
			strcpy(text, "MIPS");
			break;

		case ALCHEMY_CORE:
			strcpy(text, "Alchemy");
			break;

		case BROADCOM_CORE:
			strcpy(text, "Broadcom");
			break;

		case ALTERA_CORE:
			strcpy(text, "Altera");
			break;

		default:
			sprintf (text, "Unknown CPU vendor code %u.", ((prid & 0x00ffff00) >> 16));
			break;
	}

	/* Display Core Vendor */
    LOG_USER (" vendor: %s", &text[0]);
    LOG_USER ("  cpuid: %d", info.cpuid);
	switch ((((config3 & 0x0000C000) >>  14)))
	{
		case 0:
			strcpy (text, "MIPS32");
			break;
		case 1:
			strcpy (text, "microMIPS");
			break;
		case 2:
			strcpy (text, "MIPS32 (at reset) and microMIPS");
			break;

		case 3:
			strcpy (text, "microMIPS (at reset) and MIPS32");
			break;
	}

	/* Display Instruction Set Info */
	LOG_USER ("instr Set: %s", &text[0]);

	LOG_USER ("Instr Cache: %d", info.iCacheSize);
	LOG_USER (" Data Cache: %d", info.dCacheSize);

	LOG_USER ("Max Number of Instr Breakpoints: %d", mips32->num_inst_bpoints);
	LOG_USER ("Max Number of  Data Breakpoints: %d", mips32->num_data_bpoints);

	if (info.dspase)
		strcpy(text, "true");
	else
		strcpy(text, "false");

    LOG_USER ("dsp: %s", &text[0]);

	/* MIPS� SIMD Architecture (MSA) */
	if (info.msa)
		strcpy(text, "true");
	else
		strcpy(text, "false");

    LOG_USER ("msa: %s", &text[0]);

	/*Move To/From High COP0 (MTHC0/MFHC0) instructions are implemented. */
	if (info.mvh)
		strcpy(text, "true");
	else
		strcpy(text, "false");

    LOG_USER ("mvh: %s", &text[0]);

	/*
	  {cmPresent 0}
	  {cpuCore MIPS_microAptiv_UC}
	  {cpuid 0}
	  {cpuType 4194336}
	  {dCache 0x0}
	  {dspase 1}
	  {ejtagVersion 5.0}
	  {iCache 0x0}
	  {idcode 0x1}
	  {impcode 0xa1004000}
	  {instSet microMIPS (at reset) and MIPS32}
	  {micromipsase 1}
	  {mmutype FIXED_MAPPING}
	  {msaPresent 0}
	  {mtase 0}
	  {numShadowRegs 0}
	  {prid 0x1e019d28}
	  {rtl 0x28}
	  {tlbEntries 0}
	  {vendor MIPS}
	  {vzase 0}
	  {vzGuestId 0}
	  {bkptRangePresent 0x3000f}
	  {cbtrig 0}
	  {daSampling 1}
	  {numDataBkpts 2}
	  {numInstBkpts 4}
	  {profiling 1}
	  {hwoffchip 1}
	  {hwonchip 0}
	  {iFlowtrace 1}
	  {offchip 1}
	  {onchip 0}
	  {pcTraceForcedOn 1}
	  {tcbCpuBits 0}
	  {tcbrev 2}
	  {tcbVmodes 0}
	  */
	return ERROR_OK;
}

COMMAND_HANDLER(mips32_handle_invalidate_cache_command)
{
    int retval = -1;
    struct target *target = get_current_target(CMD_CTX);
    struct mips32_common *mips32 = target_to_mips32(target);
    struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	int i = 0;

	char *cache_msg[] = {"instr", "data", "L23", NULL, NULL, NULL};

    if (target->state != TARGET_HALTED){
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (CMD_ARGC >= 2){
		LOG_DEBUG("ERROR_COMMAND_SYNTAX_ERROR");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

    if (CMD_ARGC == 1) {
		/* PARSE command options - all/inst/data/allnowb/datanowb */
		for (i = 0; i < 5 ; i++) {
			if (strcmp(CMD_ARGV[0], invalidate_cmd[i].arg) == 0){
				switch (invalidate_cmd[i].option) {
					case ALL:
						LOG_INFO ("clearing %s cache", cache_msg[1]);
						retval = mips32_pracc_invalidate_cache(target, ejtag_info, 0, 0, 0, invalidate_cmd[1].option);
						if (retval != ERROR_OK)
							return (retval);
						
#if 0
						/* Disable for now - need to add L2 code */
						LOG_INFO ("clearing %s cache", cache_msg[2]);
						retval = mips32_pracc_invalidate_cache(target, ejtag_info, 0, 0, 0, L2);
						if (retval != ERROR_OK)
							return (retval);
#endif

						LOG_INFO ("clearing %s cache", cache_msg[0]);
						retval = mips32_pracc_invalidate_cache(target, ejtag_info, 0, 0, 0, invalidate_cmd[0].option);
						if (retval != ERROR_OK)
							return (retval);
						break;

					case INST:
						LOG_INFO ("clearing %s cache", cache_msg[0]);
						retval = mips32_pracc_invalidate_cache(target, ejtag_info, 0, 0, 0, invalidate_cmd[1].option);
						if (retval != ERROR_OK)
							return (retval);
						break;

					case DATA:
						LOG_INFO ("clearing %s cache", cache_msg[1]);
						retval = mips32_pracc_invalidate_cache(target, ejtag_info, 0, 0, 0, invalidate_cmd[2].option);
						if (retval != ERROR_OK)
							return (retval);
						break;

					case ALLNOWB:
						LOG_INFO ("clearing %s cache no writeback", cache_msg[1]);
						retval = mips32_pracc_invalidate_cache(target, ejtag_info, 0, 0, 0, invalidate_cmd[3].option);
						if (retval != ERROR_OK)
							return (retval);

#if 0
						/* Disable for now - need to add L2 code */
						LOG_INFO ("clearing %s cache no writeback", cache_msg[2]);
						retval = mips32_pracc_invalidate_cache(target, ejtag_info, 0, 0, 0, L2);
						if (retval != ERROR_OK)
							return (retval);
#endif

						LOG_INFO ("clearing %s cache", cache_msg[0]);
						retval = mips32_pracc_invalidate_cache(target, ejtag_info, 0, 0, 0, invalidate_cmd[0].option);
						if (retval != ERROR_OK)
							return (retval);
						break;

					case DATANOWB:
						LOG_INFO ("clearing %s cache no writeback", cache_msg[0]);
						retval = mips32_pracc_invalidate_cache(target, ejtag_info, 0, 0, 0, invalidate_cmd[4].option);
						if (retval != ERROR_OK)
							return (retval);
						break;

					default:
						LOG_INFO ("Invalid option");
						retval = ERROR_FAIL;
				}

				if (retval == ERROR_FAIL){
					return ERROR_FAIL;
				}
				else
					break;
			}
		}
	} else {
		/* default is All */
		LOG_INFO ("clearing %s cache", cache_msg[1]);
		retval = mips32_pracc_invalidate_cache(target, ejtag_info, 0, 0, 0, invalidate_cmd[1].option);
		if (retval != ERROR_OK)
			return (retval);
						
#if 0
		/* Disable for now - need to add L2 code */
		LOG_INFO ("clearing %s cache", cache_msg[2]);
		retval = mips32_pracc_invalidate_cache(target, ejtag_info, 0, 0, 0, L2);
		if (retval != ERROR_OK)
			return (retval);
#endif

		LOG_INFO ("clearing %s cache", cache_msg[0]);
		retval = mips32_pracc_invalidate_cache(target, ejtag_info, 0, 0, 0, invalidate_cmd[0].option);
		if (retval != ERROR_OK)
			return (retval);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(mips32_handle_scan_delay_command)
{
    struct target *target = get_current_target(CMD_CTX);
    struct mips32_common *mips32 = target_to_mips32(target);
    struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

    if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], ejtag_info->scan_delay);
    else if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

    command_print(CMD_CTX, "scan delay: %d nsec", ejtag_info->scan_delay);
    if (ejtag_info->scan_delay >= MIPS32_SCAN_DELAY_LEGACY_MODE) {
		ejtag_info->mode = 0;
		command_print(CMD_CTX, "running in legacy mode");
    } else {
		ejtag_info->mode = 1;
		command_print(CMD_CTX, "running in fast queued mode");
    }

    return ERROR_OK;
}

extern int mips_ejtag_get_impcode(struct mips_ejtag *ejtag_info, uint32_t *impcode);
COMMAND_HANDLER(mips32_handle_ejtag_reg_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

	uint32_t idcode;
	uint32_t impcode;
    uint32_t ejtag_ctrl;
    int retval;

	retval = mips_ejtag_get_idcode(ejtag_info, &idcode);
	retval = mips_ejtag_get_impcode (ejtag_info, &impcode);
    mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	retval = mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	LOG_USER ("       idcode: 0x%8.8x", idcode);
	LOG_USER ("      impcode: 0x%8.8x", impcode);
	LOG_USER ("ejtag control: 0x%8.8x", ejtag_ctrl);

#if 0
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], ejtag_info->scan_delay);
	else if (CMD_ARGC > 1)
			return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD_CTX, "scan delay: %d nsec", ejtag_info->scan_delay);
	if (ejtag_info->scan_delay >= MIPS32_SCAN_DELAY_LEGACY_MODE) {
		ejtag_info->mode = 0;
		command_print(CMD_CTX, "running in legacy mode");
	} else {
		ejtag_info->mode = 1;
		command_print(CMD_CTX, "running in fast queued mode");
	}
#endif
	return ERROR_OK;
}

COMMAND_HANDLER(mips32_handle_pic32_reset_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

    uint32_t ejtag_ctrl;
    int retval;

	LOG_INFO("Using MTAP reset to reset processor...");

	/* use microchip specific MTAP reset */
	mips_ejtag_set_instr(ejtag_info, MTAP_SW_MTAP);
	mips_ejtag_set_instr(ejtag_info, MTAP_COMMAND);

	mips_ejtag_drscan_8_out(ejtag_info, MCHP_ASERT_RST);
	mips_ejtag_drscan_8_out(ejtag_info, MCHP_DE_ASSERT_RST);
	mips_ejtag_set_instr(ejtag_info, MTAP_SW_ETAP);

#if 0
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], ejtag_info->scan_delay);
	else if (CMD_ARGC > 1)
			return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD_CTX, "scan delay: %d nsec", ejtag_info->scan_delay);
	if (ejtag_info->scan_delay >= MIPS32_SCAN_DELAY_LEGACY_MODE) {
		ejtag_info->mode = 0;
		command_print(CMD_CTX, "running in legacy mode");
	} else {
		ejtag_info->mode = 1;
		command_print(CMD_CTX, "running in fast queued mode");
	}
#endif
	return ERROR_OK;
}

static const struct command_registration mips32_exec_command_handlers[] = {
    {
		.name = "cp0",
		.handler = mips32_handle_cp0_command,
		.mode = COMMAND_EXEC,
		.usage = "[[reg_name|regnum select] [value]]",
		.help = "display/modify cp0 register",
    },
	{
		.name = "cpuinfo",
		.handler = mips32_handle_cpuinfo_command,
		.mode = COMMAND_EXEC,
		.help = "cpuinfo displays information for the current CPU core.",
		.usage = "cpuinfo",
	},
	{
		.name = "invalidate",
		.handler = mips32_handle_invalidate_cache_command,
		.mode = COMMAND_ANY,
		.help = "Invalidate either or both of the instruction and data caches.",
		.usage = "[all|inst|data|allnowb|datanowb]",
	},
    {
		.name = "scan_delay",
		.handler = mips32_handle_scan_delay_command,
		.mode = COMMAND_ANY,
		.help = "display/set scan delay in nano seconds",
		.usage = "[value]",
    },
    {
		.name = "dsp",
		.handler = mips32_handle_dsp_command,
		.mode = COMMAND_ANY,
		.help = "display/set DSP registers",
		.usage = "[value]",
    },
	{
		.name = "ejtag_reg",
		.handler = mips32_handle_ejtag_reg_command,
		.mode = COMMAND_ANY,
		.help = "read ejtag registers",
		.usage = "[value]",
	},
	{
		.name = "pic32_reset",
		.handler = mips32_handle_pic32_reset_command,
		.mode = COMMAND_ANY,
		.help = "force pic32 reset",
		.usage = "[value]",
	},
    COMMAND_REGISTRATION_DONE
};

const struct command_registration mips32_command_handlers[] = {
    {
		.name = "mips32",
		.mode = COMMAND_ANY,
		.help = "mips32 command group",
		.usage = "",
		.chain = mips32_exec_command_handlers,
    },
    COMMAND_REGISTRATION_DONE
};
