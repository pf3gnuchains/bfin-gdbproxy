/* Copyright (C) 2008 Analog Devices, Inc.

   This file is subject to the terms and conditions of the GNU
   General Public License as published by the Free Software
   Foundation; either version 2, or (at your option) any later
   version.  See the file COPYING for more details.

   This file is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   Non-GPL License is also available.  Please contact
   <david.babicz@analog.com> for more information.
 
   Implementation of `Blackfin' target for the GDB proxy server.
   Written by Jie Zhang <jie.zhang@analog.com>.  */


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif /* HAVE_CONFIG_H */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <stdarg.h>
#include <getopt.h>

#include "gdbproxy.h"

#include "part.h"
#include "chain.h"
#include "tap.h"
#include "state.h"
#include "cmd.h"
#include "jtag.h"


/* MMRs definitions */

#define DBGCTL_SRAM_INIT		0x1000
#define DBGCTL_WAKEUP			0x0800
#define DBGCTL_SYSRST			0x0400
#define DBGCTL_ESSTEP			0x0200
#define DBGCTL_EMUDATSZ_32		0x0000
#define DBGCTL_EMUDATSZ_40		0x0080
#define DBGCTL_EMUDATSZ_48		0x0100
#define DBGCTL_EMUDATSZ_MASK		0x0180
#define DBGCTL_EMUIRLPSZ_2		0x0040
#define DBGCTL_EMUIRSZ_64		0x0000
#define DBGCTL_EMUIRSZ_48		0x0010
#define DBGCTL_EMUIRSZ_32		0x0020
#define DBGCTL_EMUIRSZ_MASK		0x0030
#define DBGCTL_EMPEN			0x0008
#define DBGCTL_EMEEN			0x0004
#define DBGCTL_EMFEN			0x0002
#define DBGCTL_EMPWR			0x0001

#define DBGSTAT_LPDEC1			0x8000
#define DBGSTAT_CORE_FAULT		0x4000
#define DBGSTAT_IDLE			0x2000
#define DBGSTAT_IN_RESET		0x1000
#define DBGSTAT_LPDEC0			0x0800
#define DBGSTAT_BIST_DONE		0x0400
#define DBGSTAT_EMUCAUSE_MASK		0x03c0
#define DBGSTAT_EMUACK			0x0020
#define DBGSTAT_EMUREADY		0x0010
#define DBGSTAT_EMUDIOVF		0x0008
#define DBGSTAT_EMUDOOVF		0x0004
#define DBGSTAT_EMUDIF			0x0002
#define DBGSTAT_EMUDOF			0x0001

#define EMUCAUSE_EMUEXCPT		0x0
#define EMUCAUSE_EMUIN			0x1
#define EMUCAUSE_WATCHPOINT		0x2
#define EMUCAUSE_PM0_OVERFLOW		0x4
#define EMUCAUSE_PM1_OVERFLOW		0x5
#define EMUCAUSE_SINGLE_STEP		0x8

const static char *emucause_infos[] = {
  "EMUEXCPT instruction was executed",
  "EMUIN pin was asserted",
  "watchpoint event occured",
  "unknown emucause 0x3",
  "performance monitor 0 overflowed",
  "performance monitor 1 overflowed",
  "unknown emucause 0x6",
  "unknown emucause 0x7",
  "emulation single step",
};

#define WPIACTL				0xffe07000
#define WPIACTL_WPAND			0x02000000
#define WPIACTL_EMUSW5			0x01000000
#define WPIACTL_EMUSW4			0x00800000
#define WPIACTL_WPICNTEN5		0x00400000
#define WPIACTL_WPICNTEN4		0x00200000
#define WPIACTL_WPIAEN5			0x00100000
#define WPIACTL_WPIAEN4			0x00080000
#define WPIACTL_WPIRINV45		0x00040000
#define WPIACTL_WPIREN45		0x00020000
#define WPIACTL_EMUSW3			0x00010000
#define WPIACTL_EMUSW2			0x00008000
#define WPIACTL_WPICNTEN3		0x00004000
#define WPIACTL_WPICNTEN2		0x00002000
#define WPIACTL_WPIAEN3			0x00001000
#define WPIACTL_WPIAEN2			0x00000800
#define WPIACTL_WPIRINV23		0x00000400
#define WPIACTL_WPIREN23		0x00000200
#define WPIACTL_EMUSW1			0x00000100
#define WPIACTL_EMUSW0			0x00000080
#define WPIACTL_WPICNTEN1		0x00000040
#define WPIACTL_WPICNTEN0		0x00000020
#define WPIACTL_WPIAEN1			0x00000010
#define WPIACTL_WPIAEN0			0x00000008
#define WPIACTL_WPIRINV01		0x00000004
#define WPIACTL_WPIREN01		0x00000002
#define WPIACTL_WPPWR			0x00000001
#define WPIA0				0xffe07040

#define WPDACTL				0xffe07100
#define WPDACTL_WPDACC1_R		0x00002000
#define WPDACTL_WPDACC1_W		0x00001000
#define WPDACTL_WPDACC1_A		0x00003000
#define WPDACTL_WPDSRC1_1		0x00000800
#define WPDACTL_WPDSRC1_0		0x00000400
#define WPDACTL_WPDSRC1_A		0x00000c00
#define WPDACTL_WPDACC0_R		0x00000200
#define WPDACTL_WPDACC0_W		0x00000100
#define WPDACTL_WPDACC0_A		0x00000300
#define WPDACTL_WPDSRC0_1		0x00000080
#define WPDACTL_WPDSRC0_0		0x00000040
#define WPDACTL_WPDSRC0_A		0x000000c0
#define WPDACTL_WPDCNTEN1		0x00000020
#define WPDACTL_WPDCNTEN0		0x00000010
#define WPDACTL_WPDAEN1			0x00000008
#define WPDACTL_WPDAEN0			0x00000004
#define WPDACTL_WPDRINV01		0x00000002
#define WPDACTL_WPDREN01		0x00000001
#define WPDA0				0xffe07140

#define WPSTAT				0xffe07200
#define WPSTAT_STATDA1			0x00000080
#define WPSTAT_STATDA0			0x00000040
#define WPSTAT_STATIA5			0x00000020
#define WPSTAT_STATIA4			0x00000010
#define WPSTAT_STATIA3			0x00000008
#define WPSTAT_STATIA2			0x00000004
#define WPSTAT_STATIA1			0x00000002
#define WPSTAT_STATIA0			0x00000001

#define SWRST				0xffc00100

#define SYSCR				0xffc00104
#define SYSCR_COREB_SRAM_INIT		0x0020
#define SYSCR_BOOT_ON_SWRST		0x0010

#define DMA_CONFIG_FLOW_STOP		0x0000
#define DMA_CONFIG_NDSIZE_0		0x0000
#define DMA_CONFIG_DI_EN		0x0080
#define DMA_CONFIG_DI_SEL		0x0040
#define DMA_CONFIG_SYNC			0x0020
#define DMA_CONFIG_DMA2D		0x0010
#define DMA_CONFIG_WDSIZE_8		0x0000
#define DMA_CONFIG_WDSIZE_16		0x0004
#define DMA_CONFIG_WDSIZE_32		0x0008
#define DMA_CONFIG_WDSIZE_MASK		0x000c
#define DMA_CONFIG_WNR			0x0002
#define DMA_CONFIG_DMAEN		0x0001

#define DMA_IRQ_STATUS_DMA_RUN		0x0008
#define DMA_IRQ_STATUS_DFETCH		0x0004
#define DMA_IRQ_STATUS_DMA_ERR		0x0002
#define DMA_IRQ_STATUS_DMA_DONE		0x0001

#define EBIU_SDGCTL			0xffc00a10
#define EBIU_SDBCTL			0xffc00a14
#define EBIU_SDRRC			0xffc00a18
#define EBIU_SDSTAT			0xffc00a1c

#define EBIU_DDRCTL0			0xffc00a20
#define EBIU_DDRCTL1			0xffc00a24
#define EBIU_DDRCTL2			0xffc00a28
#define EBIU_DDRCTL3			0xffc00a2c
#define EBIU_RSTCTL			0xffc00a3c

#define SICA_SYSCR			0xffc00104
#define SICA_SYSCR_COREB_SRAM_INIT	0x0020

#define DMEM_CONTROL			0xffe00004
#define DCPLB_ADDR0			0xffe00100
#define DCPLB_DATA0			0xffe00200
#define IMEM_CONTROL			0xffe01004
#define ICPLB_ADDR0			0xffe01100
#define ICPLB_DATA0			0xffe01200

#define ENICPLB				0x00000002
#define IMC				0x00000004

#define ENDCPLB				0x00000002
#define DMC				0x0000000c
#define ACACHE_BSRAM			0x00000008
#define ACACHE_BCACHE			0x0000000c

#define PAGE_SIZE_MASK			0x00030000
#define PAGE_SIZE_4MB			0x00030000
#define PAGE_SIZE_1MB			0x00020000
#define PAGE_SIZE_4KB			0x00010000
#define PAGE_SIZE_1KB			0x00000000
#define CPLB_L1_AOW			0x00008000
#define CPLB_WT				0x00004000
#define CPLB_L1_CHBL			0x00001000
#define CPLB_MEM_LEV			0x00000200
#define CPLB_LRUPRIO			0x00000100
#define CPLB_DIRTY			0x00000080
#define CPLB_SUPV_WR			0x00000010
#define CPLB_USER_WR			0x00000008
#define CPLB_USER_RD			0x00000004
#define CPLB_LOCK			0x00000002
#define CPLB_VALID			0x00000001

#define L1_DMEMORY	(PAGE_SIZE_1MB | CPLB_SUPV_WR | CPLB_USER_WR | CPLB_USER_RD | CPLB_VALID)
#define SDRAM_DNON_CHBL (PAGE_SIZE_4MB | CPLB_SUPV_WR | CPLB_USER_WR | CPLB_USER_RD | CPLB_VALID)
#define SDRAM_DGEN_WB	(PAGE_SIZE_4MB | CPLB_L1_CHBL | CPLB_DIRTY | CPLB_SUPV_WR | CPLB_USER_WR | CPLB_USER_RD | CPLB_VALID)
#define SDRAM_DGEN_WT	(PAGE_SIZE_4MB | CPLB_L1_CHBL | CPLB_WT | CPLB_L1_AOW | CPLB_SUPV_WR | CPLB_USER_WR | CPLB_USER_RD | CPLB_VALID)
#define L1_IMEMORY	(PAGE_SIZE_1MB | CPLB_USER_RD | CPLB_VALID)
#define SDRAM_INON_CHBL	(PAGE_SIZE_4MB | CPLB_USER_RD | CPLB_VALID)
#define SDRAM_IGENERIC	(PAGE_SIZE_4MB | CPLB_L1_CHBL | CPLB_MEM_LEV | CPLB_USER_RD | CPLB_VALID)
/* The following DCPLB DATA are used by gdbproxy to prevent DCPLB missing exception.  */
#define DNON_CHBL_4MB	(PAGE_SIZE_4MB | CPLB_SUPV_WR | CPLB_USER_WR | CPLB_USER_RD | CPLB_VALID)
#define DNON_CHBL_1MB	(PAGE_SIZE_1MB | CPLB_SUPV_WR | CPLB_USER_WR | CPLB_USER_RD | CPLB_VALID)
#define DNON_CHBL_4KB	(PAGE_SIZE_4KB | CPLB_SUPV_WR | CPLB_USER_WR | CPLB_USER_RD | CPLB_VALID)
#define DNON_CHBL_1KB	(PAGE_SIZE_1KB | CPLB_SUPV_WR | CPLB_USER_WR | CPLB_USER_RD | CPLB_VALID)

#define CACHE_LINE_BYTES		32

#define DTEST_COMMAND			0xffe00300
#define ITEST_COMMAND			0xffe01300

#define BFIN_DCPLB_NUM			16
#define BFIN_ICPLB_NUM			16


/* Misc macros and definitions */

#define RP_BFIN_MAX_HWBREAKPOINTS	6
#define RP_BFIN_MAX_HWWATCHPOINTS	2

#define MAX_BREAKPOINT_LEN		4

/* The boards we support.  */
typedef enum _bfin_board
{
  UNKNOWN_BOARD,
  BF527_EZKIT,
  BF533_EZKIT,
  BF533_STAMP,
  BF537_EZKIT,
  BF537_STAMP,
  BF538F_EZKIT,
  BF548_EZKIT,
  BF561_EZKIT,
} bfin_board;

#define IDCODE_SCAN			0
#define DBGSTAT_SCAN			1
#define DBGCTL_SCAN			2
#define EMUIR_SCAN			3
#define EMUDAT_SCAN			4
#define EMUPC_SCAN			5
#define BYPASS				6

const static char *scans[] = {
  "IDCODE_SCAN",
  "DBGSTAT_SCAN",
  "DBGCTL_SCAN",
  "EMUIR_SCAN",
  "EMUDAT_SCAN",
  "EMUPC_SCAN",
  "BYPASS",
};

#define INSN_NOP			0x00000000
#define INSN_RTE			0x00140000
#define INSN_CSYNC			0x00230000
#define INSN_SSYNC			0x00240000
#define INSN_ILLEGAL			0xffffffff

#define UPDATE				0
#define RUNTEST				1

/* Convert part number to thread ID.
   Core A is assigned the thread ID 1.
   Core B is assigned the thread ID 2.  */
#define THREAD_ID(n) \
  (cpu->chain->parts->len - (n))
/* Convert thread id to part number.  */
#define PART_NO(n) \
  (cpu->chain->parts->len - (n))

#define ALL_THREADS	-1
#define ANY_THREAD	0

#define INVALID_CORE	-1
#define ALL_CORES	(cpu->chain->parts->len + 1)
#define ANY_CORE	(cpu->chain->parts->len)

#define CACHE_DISABLED 0
#define WRITE_THROUGH 1
#define WRITE_BACK 2

#define WPDA_DISABLE			0
#define WPDA_WRITE			1
#define WPDA_READ			2
#define WPDA_ALL			3

/* High-Nibble: group code, low nibble: register code.  */
#define T_REG_R				0x00
#define T_REG_P				0x10
#define T_REG_I				0x20
#define T_REG_B				0x30
#define T_REG_L				0x34
#define T_REG_M				0x24
#define T_REG_A				0x40

enum core_regnum
{
  REG_R0 = T_REG_R, REG_R1, REG_R2, REG_R3, REG_R4, REG_R5, REG_R6, REG_R7,
  REG_P0 = T_REG_P, REG_P1, REG_P2, REG_P3, REG_P4, REG_P5, REG_SP, REG_FP,
  REG_I0 = T_REG_I, REG_I1, REG_I2, REG_I3,
  REG_M0 = T_REG_M, REG_M1, REG_M2, REG_M3,
  REG_B0 = T_REG_B, REG_B1, REG_B2, REG_B3,
  REG_L0 = T_REG_L, REG_L1, REG_L2, REG_L3,
  REG_A0x = T_REG_A, REG_A0w, REG_A1x, REG_A1w,
  REG_ASTAT = 0x46,
  REG_RETS = 0x47,
  REG_LC0 = 0x60, REG_LT0, REG_LB0, REG_LC1, REG_LT1, REG_LB1,
  REG_CYCLES, REG_CYCLES2,
  REG_USP = 0x70, REG_SEQSTAT, REG_SYSCFG,
  REG_RETI, REG_RETX, REG_RETN, REG_RETE, REG_EMUDAT,
};

#define CLASS_MASK		0xf0
#define GROUP(x)		(((x) & CLASS_MASK) >> 4)
#define DREG_P(x)		(((x) & CLASS_MASK) == T_REG_R)
#define PREG_P(x)		(((x) & CLASS_MASK) == T_REG_P)

/* This enum was copied from GDB.  */
enum gdb_regnum
{
  /* Core Registers */
  BFIN_R0_REGNUM = 0,
  BFIN_R1_REGNUM,
  BFIN_R2_REGNUM,
  BFIN_R3_REGNUM,
  BFIN_R4_REGNUM,
  BFIN_R5_REGNUM,
  BFIN_R6_REGNUM,
  BFIN_R7_REGNUM,
  BFIN_P0_REGNUM,
  BFIN_P1_REGNUM,
  BFIN_P2_REGNUM,
  BFIN_P3_REGNUM,
  BFIN_P4_REGNUM,
  BFIN_P5_REGNUM,
  BFIN_SP_REGNUM,
  BFIN_FP_REGNUM,
  BFIN_I0_REGNUM,
  BFIN_I1_REGNUM,
  BFIN_I2_REGNUM,
  BFIN_I3_REGNUM,
  BFIN_M0_REGNUM,
  BFIN_M1_REGNUM,
  BFIN_M2_REGNUM,
  BFIN_M3_REGNUM,
  BFIN_B0_REGNUM,
  BFIN_B1_REGNUM,
  BFIN_B2_REGNUM,
  BFIN_B3_REGNUM,
  BFIN_L0_REGNUM,
  BFIN_L1_REGNUM,
  BFIN_L2_REGNUM,
  BFIN_L3_REGNUM,
  BFIN_A0_DOT_X_REGNUM,
  BFIN_AO_DOT_W_REGNUM,
  BFIN_A1_DOT_X_REGNUM,
  BFIN_A1_DOT_W_REGNUM,
  BFIN_ASTAT_REGNUM,
  BFIN_RETS_REGNUM,
  BFIN_LC0_REGNUM,
  BFIN_LT0_REGNUM,
  BFIN_LB0_REGNUM,
  BFIN_LC1_REGNUM,
  BFIN_LT1_REGNUM,
  BFIN_LB1_REGNUM,
  BFIN_CYCLES_REGNUM,
  BFIN_CYCLES2_REGNUM,
  BFIN_USP_REGNUM,
  BFIN_SEQSTAT_REGNUM,
  BFIN_SYSCFG_REGNUM,
  BFIN_RETI_REGNUM,
  BFIN_RETX_REGNUM,
  BFIN_RETN_REGNUM,
  BFIN_RETE_REGNUM,

  /* Pseudo Registers */
  BFIN_PC_REGNUM,
  BFIN_CC_REGNUM,
  BFIN_TEXT_ADDR,		/* Address of .text section.  */
  BFIN_TEXT_END_ADDR,		/* Address of the end of .text section.  */
  BFIN_DATA_ADDR,		/* Address of .data section.  */

  BFIN_FDPIC_EXEC_REGNUM,
  BFIN_FDPIC_INTERP_REGNUM,

  /* MMRs */
  BFIN_IPEND_REGNUM,

  /* LAST ENTRY SHOULD NOT BE CHANGED.  */
  BFIN_NUM_REGS			/* The number of all registers.  */
};

/* Map gdb register number to core register number.  */
static int map_gdb_core[] = {
  /* Core Registers */
  REG_R0, REG_R1, REG_R2, REG_R3, REG_R4, REG_R5, REG_R6, REG_R7,
  REG_P0, REG_P1, REG_P2, REG_P3, REG_P4, REG_P5, REG_SP, REG_FP,
  REG_I0, REG_I1, REG_I2, REG_I3, REG_M0, REG_M1, REG_M2, REG_M3,
  REG_B0, REG_B1, REG_B2, REG_B3, REG_L0, REG_L1, REG_L2, REG_L3,
  REG_A0x, REG_A0w, REG_A1x, REG_A1w, REG_ASTAT, REG_RETS,
  REG_LC0, REG_LT0, REG_LB0, REG_LC1, REG_LT1, REG_LB1,
  REG_CYCLES, REG_CYCLES2,
  REG_USP, REG_SEQSTAT, REG_SYSCFG, REG_RETI, REG_RETX, REG_RETN, REG_RETE,

  /* Pseudo Registers */
  REG_RETE, -1 /* REG_CC */ , -1, -1, -1, -1, -1,

  /* MMRs */
  -1				/* REG_IPEND */
};


/* Target methods */

static void bfin_help (const char *prog_name);
static int bfin_open (int argc,
		      char *const argv[],
		      const char *prog_name, log_func log_fn);
static void bfin_close (void);
static int bfin_connect (char *status_string,
			 size_t status_string_size, int *can_restart);
static int bfin_disconnect (void);
static void bfin_kill (void);
static int bfin_restart (void);
static void bfin_stop (void);
static int bfin_set_gen_thread (rp_thread_ref *thread);
static int bfin_set_ctrl_thread (rp_thread_ref *thread);
static int bfin_is_thread_alive (rp_thread_ref *thread, int *alive);
static int bfin_read_registers (uint8_t *data_buf,
				uint8_t *avail_buf,
				size_t buf_size, size_t *read_size);
static int bfin_write_registers (uint8_t *data_buf, size_t write_size);
static int bfin_read_single_register (unsigned int reg_no,
				      uint8_t *data_buf,
				      uint8_t *avail_buf,
				      size_t buf_size, size_t *read_size);
static int bfin_write_single_register (unsigned int reg_no,
				       uint8_t *data_buf, size_t write_size);
static int bfin_read_mem (uint64_t addr,
			  uint8_t *data_buf,
			  size_t req_size, size_t *actual_size);
static int bfin_write_mem (uint64_t addr,
			   uint8_t *data_buf, size_t req_sise);
static int bfin_resume_from_current (int step, int sig);
static int bfin_resume_from_addr (int step, int sig, uint64_t addr);
static int bfin_go_waiting (int sig);
static int bfin_wait_partial (int first,
			      char *status_string,
			      size_t status_string_len,
			      out_func out, int *implemented, int *more);
static int bfin_wait (char *status_string,
		      size_t status_string_len,
		      out_func out, int *implemented);
static int bfin_process_query (unsigned int *mask,
			       rp_thread_ref *arg, rp_thread_info *info);
static int bfin_list_query (int first,
			    rp_thread_ref *arg,
			    rp_thread_ref *result,
			    size_t max_num, size_t *num, int *done);
static int bfin_current_thread_query (rp_thread_ref *thread);
static int bfin_offsets_query (uint64_t *text,
			       uint64_t *data, uint64_t *bss);
static int bfin_crc_query (uint64_t addr, size_t len, uint32_t *val);
static int bfin_raw_query (char *in_buf, char *out_buf, size_t out_buf_size);
static int bfin_threadinfo_query (int first,
				  char *out_buf, size_t out_buf_size);
static int bfin_threadextrainfo_query (rp_thread_ref *thread,
				       char *out_buf, size_t out_buf_size);
static int bfin_packetsize_query (char *out_buf, size_t out_buf_size);
static int bfin_add_break (int type, uint64_t addr, unsigned int len);
static int bfin_remove_break (int type, uint64_t addr, unsigned int len);

/* Global target descriptor */
rp_target bfin_target = {
  NULL,
  "bfin",
  "Blackfin JTAG target",
  NULL,
  bfin_help,
  bfin_open,
  bfin_close,
  bfin_connect,
  bfin_disconnect,
  bfin_kill,
  bfin_restart,
  bfin_stop,
  bfin_set_gen_thread,
  bfin_set_ctrl_thread,
  bfin_is_thread_alive,
  bfin_read_registers,
  bfin_write_registers,
  bfin_read_single_register,
  bfin_write_single_register,
  bfin_read_mem,
  bfin_write_mem,
  bfin_resume_from_current,
  bfin_resume_from_addr,
  bfin_go_waiting,
  bfin_wait_partial,
  bfin_wait,
  bfin_process_query,
  bfin_list_query,
  bfin_current_thread_query,
  bfin_offsets_query,
  bfin_crc_query,
  bfin_raw_query,
  bfin_add_break,
  bfin_remove_break,
  bfin_threadinfo_query,
  bfin_threadextrainfo_query,
  bfin_packetsize_query,
};
static char default_jtag_connect[] = "cable gnICE ftdi-mpsse 0456:F000";

typedef struct _bfin_swbp
{
  struct _bfin_swbp *next;
  uint32_t address;
  uint8_t old_data[MAX_BREAKPOINT_LEN];
  int core;
} bfin_swbp;

typedef struct _bfin_l1_map
{
  uint32_t l1;
  uint32_t l1_data_a;
  uint32_t l1_data_a_end;
  uint32_t l1_data_a_cache;
  uint32_t l1_data_a_cache_end;
  uint32_t l1_data_b;
  uint32_t l1_data_b_end;
  uint32_t l1_data_b_cache;
  uint32_t l1_data_b_cache_end;
  uint32_t l1_code;
  uint32_t l1_code_end;
  uint32_t l1_code_cache;
  uint32_t l1_code_cache_end;
  uint32_t l1_code_rom;
  uint32_t l1_code_rom_end;
  uint32_t l1_scratch;
  uint32_t l1_scratch_end;
  uint32_t l1_end;
} bfin_l1_map;

static bfin_l1_map bf52x_l1_map = {
  .l1			= 0xff800000,
  .l1_data_a		= 0xff800000,
  .l1_data_a_end	= 0xff804000,
  .l1_data_a_cache	= 0xff804000,
  .l1_data_a_cache_end	= 0xff808000,
  .l1_data_b		= 0xff900000,
  .l1_data_b_end	= 0xff904000,
  .l1_data_b_cache	= 0xff904000,
  .l1_data_b_cache_end	= 0xff908000,
  .l1_code		= 0xffa00000,
  .l1_code_end		= 0xffa0c000,
  .l1_code_cache	= 0xffa10000,
  .l1_code_cache_end	= 0xffa14000,
  .l1_scratch		= 0xffb00000,
  .l1_scratch_end	= 0xffb01000,
  .l1_end		= 0xffc00000,
};
static bfin_l1_map bf533_l1_map = {
  .l1			= 0xff800000,
  .l1_data_a		= 0xff800000,
  .l1_data_a_end	= 0xff804000,
  .l1_data_a_cache	= 0xff804000,
  .l1_data_a_cache_end	= 0xff808000,
  .l1_data_b		= 0xff900000,
  .l1_data_b_end	= 0xff904000,
  .l1_data_b_cache	= 0xff904000,
  .l1_data_b_cache_end	= 0xff908000,
  .l1_code		= 0xffa00000,
  .l1_code_end		= 0xffa10000,
  .l1_code_cache	= 0xffa10000,
  .l1_code_cache_end	= 0xffa14000,
  .l1_scratch		= 0xffb00000,
  .l1_scratch_end	= 0xffb01000,
  .l1_end		= 0xffc00000,
};
static bfin_l1_map bf537_l1_map = {
  .l1			= 0xff800000,
  .l1_data_a		= 0xff800000,
  .l1_data_a_end	= 0xff804000,
  .l1_data_a_cache	= 0xff804000,
  .l1_data_a_cache_end	= 0xff808000,
  .l1_data_b		= 0xff900000,
  .l1_data_b_end	= 0xff904000,
  .l1_data_b_cache	= 0xff904000,
  .l1_data_b_cache_end	= 0xff908000,
  .l1_code		= 0xffa00000,
  .l1_code_end		= 0xffa0c000,
  .l1_code_cache	= 0xffa10000,
  .l1_code_cache_end	= 0xffa14000,
  .l1_scratch		= 0xffb00000,
  .l1_scratch_end	= 0xffb01000,
  .l1_end		= 0xffc00000,
};
static bfin_l1_map bf538_l1_map = {
  .l1			= 0xff800000,
  .l1_data_a		= 0xff800000,
  .l1_data_a_end	= 0xff804000,
  .l1_data_a_cache	= 0xff804000,
  .l1_data_a_cache_end	= 0xff808000,
  .l1_data_b		= 0xff900000,
  .l1_data_b_end	= 0xff904000,
  .l1_data_b_cache	= 0xff904000,
  .l1_data_b_cache_end	= 0xff908000,
  .l1_code		= 0xffa00000,
  .l1_code_end		= 0xffa10000,
  .l1_code_cache	= 0xffa10000,
  .l1_code_cache_end	= 0xffa14000,
  .l1_scratch		= 0xffb00000,
  .l1_scratch_end	= 0xffb01000,
  .l1_end		= 0xffc00000,
};
static bfin_l1_map bf54x_l1_map = {
  .l1			= 0xff800000,
  .l1_data_a		= 0xff800000,
  .l1_data_a_end	= 0xff804000,
  .l1_data_a_cache	= 0xff804000,
  .l1_data_a_cache_end	= 0xff808000,
  .l1_data_b		= 0xff900000,
  .l1_data_b_end	= 0xff904000,
  .l1_data_b_cache	= 0xff904000,
  .l1_data_b_cache_end	= 0xff908000,
  .l1_code		= 0xffa00000,
  .l1_code_end		= 0xffa08000,
  .l1_code_cache	= 0xffa10000,
  .l1_code_cache_end	= 0xffa14000,
  .l1_code_rom		= 0xffa14000,
  .l1_code_rom_end	= 0xffa24000,
  .l1_scratch		= 0xffb00000,
  .l1_scratch_end	= 0xffb01000,
  .l1_end		= 0xffc00000,
};
static bfin_l1_map bf561_a_l1_map = {
  .l1			= 0xff800000,
  .l1_data_a		= 0xff800000,
  .l1_data_a_end	= 0xff804000,
  .l1_data_a_cache	= 0xff804000,
  .l1_data_a_cache_end	= 0xff808000,
  .l1_data_b		= 0xff900000,
  .l1_data_b_end	= 0xff904000,
  .l1_data_b_cache	= 0xff904000,
  .l1_data_b_cache_end	= 0xff908000,
  .l1_code		= 0xffa00000,
  .l1_code_end		= 0xffa10000,
  .l1_code_cache	= 0xffa10000,
  .l1_code_cache_end	= 0xffa14000,
  .l1_scratch		= 0xffb00000,
  .l1_scratch_end	= 0xffb01000,
  .l1_end		= 0xffc00000,
};
static bfin_l1_map bf561_b_l1_map = {
  .l1			= 0xff400000,
  .l1_data_a		= 0xff400000,
  .l1_data_a_end	= 0xff404000,
  .l1_data_a_cache	= 0xff404000,
  .l1_data_a_cache_end	= 0xff408000,
  .l1_data_b		= 0xff500000,
  .l1_data_b_end	= 0xff504000,
  .l1_data_b_cache	= 0xff504000,
  .l1_data_b_cache_end	= 0xff508000,
  .l1_code		= 0xff600000,
  .l1_code_end		= 0xff610000,
  .l1_code_cache	= 0xff610000,
  .l1_code_cache_end	= 0xff614000,
  .l1_scratch		= 0xff700000,
  .l1_scratch_end	= 0xff701000,
  .l1_end		= 0xff800000,
};

typedef struct _bfin_mem_map
{
  uint32_t sdram;
  uint32_t sdram_end;
  uint32_t async_mem;
  uint32_t flash;
  uint32_t flash_end;
  uint32_t async_mem_end;
  uint32_t boot_rom;
  uint32_t boot_rom_end;
  uint32_t l2_sram;
  uint32_t l2_sram_end;
  uint32_t l1;
  uint32_t l1_end;
  uint32_t sysmmr;
  uint32_t coremmr;
} bfin_mem_map;

static bfin_mem_map bf52x_mem_map = {
  .sdram		= 0,
  .sdram_end		= 0x08000000,
  .async_mem		= 0x20000000,
  .flash		= 0x20000000,
  .flash_end		= 0x20100000,
  .async_mem_end	= 0x20400000,
  .boot_rom		= 0xef000000,
  .boot_rom_end		= 0xef004000,
  .l1			= 0xff800000,
  .l1_end		= 0xffc00000,
  .sysmmr		= 0xffc00000,
  .coremmr		= 0xffe00000,
};
static bfin_mem_map bf533_mem_map = {
  .sdram		= 0,
  .sdram_end		= 0x04000000,
  .async_mem		= 0x20000000,
  .flash		= 0x20000000,
  .flash_end		= 0x20100000,
  .async_mem_end	= 0x20400000,
  .boot_rom		= 0xef000000,
  .boot_rom_end		= 0xef000400,
  .l1			= 0xff800000,
  .l1_end		= 0xffc00000,
  .sysmmr		= 0xffc00000,
  .coremmr		= 0xffe00000,
};
static bfin_mem_map bf537_mem_map = {
  .sdram		= 0,
  .sdram_end		= 0x04000000,
  .async_mem		= 0x20000000,
  .flash		= 0x20000000,
  .flash_end		= 0x20100000,
  .async_mem_end	= 0x20400000,
  .boot_rom		= 0xef000000,
  .boot_rom_end		= 0xef000800,
  .l1			= 0xff800000,
  .l1_end		= 0xffc00000,
  .sysmmr		= 0xffc00000,
  .coremmr		= 0xffe00000,
};
static bfin_mem_map bf538_mem_map = {
  .sdram		= 0,
  .sdram_end		= 0x08000000,
  .async_mem		= 0x20000000,
  .flash		= 0x20000000,
  .flash_end		= 0x20100000,
  .async_mem_end	= 0x20400000,
  .boot_rom		= 0xef000000,
  .boot_rom_end		= 0xef000400,
  .l1			= 0xff800000,
  .l1_end		= 0xffc00000,
  .sysmmr		= 0xffc00000,
  .coremmr		= 0xffe00000,
};
static bfin_mem_map bf54x_mem_map = {
  .sdram		= 0,
  .sdram_end		= 0x04000000,
  .async_mem		= 0x20000000,
  .flash		= 0x20000000,
  .flash_end		= 0x24000000,
  .async_mem_end	= 0x30000000,
  .boot_rom		= 0xef000000,
  .boot_rom_end		= 0xef001000,
  .l2_sram		= 0xfeb00000,
  .l2_sram_end		= 0xfeb20000,
  .l1			= 0xff800000,
  .l1_end		= 0xffc00000,
  .sysmmr		= 0xffc00000,
  .coremmr		= 0xffe00000,
};
static bfin_mem_map bf561_mem_map = {
  .sdram		= 0,
  .sdram_end		= 0x04000000,
  .async_mem		= 0x20000000,
  .flash		= 0x20000000,
  .flash_end		= 0x24000000,
  .async_mem_end	= 0x30000000,
  .boot_rom		= 0xef000000,
  .boot_rom_end		= 0xef000800,
  .l2_sram		= 0xfeb00000,
  .l2_sram_end		= 0xfeb20000,
  .l1			= 0xff400000,
  .l1_end		= 0xffc00000,
  .sysmmr		= 0xffc00000,
  .coremmr		= 0xffe00000,
};

typedef struct _bfin_dma
{
  uint32_t next_desc_ptr;
  uint32_t start_addr;
  uint16_t config;
  uint16_t x_count;
  uint16_t x_modify;
  uint16_t y_count;
  uint16_t y_modify;
  uint32_t curr_desc_ptr;
  uint32_t curr_addr;
  uint16_t irq_status;
  uint16_t peripheral_map;
  uint16_t curr_x_count;
  uint16_t curr_y_count;
} bfin_dma;

typedef struct _bfin_hwwps
{
  uint32_t addr;
  size_t len;
  int mode;
} bfin_hwwps;

typedef struct _bfin_cplb_entry
{
  uint32_t addr;
  uint32_t data;
} bfin_cplb_entry;

typedef struct _bfin_core
{
  int scan;

  char *name;

  unsigned int is_locked:1;
  unsigned int is_running:1;
  unsigned int is_interrupted:1;
  unsigned int is_stepping:1;
  unsigned int is_corefault:1;

  unsigned int leave_stopped:1;
  unsigned int status_pending_p:1;
  unsigned int pending_is_breakpoint:1;

  unsigned int l1_data_a_cache_enabled:1;
  unsigned int l1_data_b_cache_enabled:1;
  unsigned int l1_code_cache_enabled:1;

  unsigned int dmem_control_valid_p:1;
  unsigned int imem_control_valid_p:1;
  unsigned int dcplbs_valid_p:1;
  unsigned int icplbs_valid_p:1;

  int pending_signal;
  uint32_t pending_stop_pc;

  bfin_l1_map l1_map;
  uint32_t registers[BFIN_NUM_REGS];
  uint32_t wpiactl;
  uint32_t wpdactl;
  uint32_t wpstat;
  uint16_t dbgctl;
  uint16_t dbgstat;
  uint32_t emuir_a;
  uint32_t emuir_b;
  uint32_t emudat_out;
  uint32_t emudat_in;
  uint32_t emupc;
  uint32_t hwbps[RP_BFIN_MAX_HWBREAKPOINTS];
  bfin_hwwps hwwps[RP_BFIN_MAX_HWWATCHPOINTS];
  uint32_t dmem_control;
  uint32_t imem_control;
  bfin_cplb_entry dcplbs[BFIN_DCPLB_NUM];
  bfin_cplb_entry icplbs[BFIN_ICPLB_NUM];
} bfin_core;

typedef struct _bfin_sdram_config
{
  uint16_t sdrrc;
  uint16_t sdbctl;
  uint32_t sdgctl;
} bfin_sdram_config;

static bfin_sdram_config bf527_ezkit_sdram_config = {
  .sdrrc = 0x0407,
  .sdbctl = 0x0025,
  .sdgctl = 0x0091998d,
};

static bfin_sdram_config bf533_ezkit_sdram_config = {
  .sdrrc = 0x01a0,
  .sdbctl = 0x0025,
  .sdgctl = 0x0091998d,
};

static bfin_sdram_config bf533_stamp_sdram_config = {
  .sdrrc = 0x01a0,
  .sdbctl = 0x0025,
  .sdgctl = 0x0091998d,
};

static bfin_sdram_config bf537_ezkit_sdram_config = {
  .sdrrc = 0x03a0,
  .sdbctl = 0x0025,
  .sdgctl = 0x0091998d,
};

static bfin_sdram_config bf538f_ezkit_sdram_config = {
  .sdrrc = 0x03f6,
  .sdbctl = 0x0025,
  .sdgctl = 0x0091998d,
};

static bfin_sdram_config bf561_ezkit_sdram_config = {
  .sdrrc = 0x01cf,
  .sdbctl = 0x0013,
  .sdgctl = 0x0091998d,
};

typedef struct _bfin_ddr_config
{
  uint32_t ddrctl0;
  uint32_t ddrctl1;
  uint32_t ddrctl2;
} bfin_ddr_config;

static bfin_ddr_config bf548_ezkit_ddr_config = {
  .ddrctl0 = 0x218A8411,
  .ddrctl1 = 0x20022222,
  .ddrctl2 = 0x00000021,
};

typedef struct _bfin_cpu
{
  chain_t *chain;
  bfin_swbp *swbps;
  bfin_board board;
  bfin_mem_map mem_map;
  bfin_sdram_config *sdram_config;
  bfin_ddr_config *ddr_config;
  uint32_t mdma_s0;
  uint32_t mdma_d0;

  /* The core will never be locked.  */
  int core_a;

  int general_core;
  int continue_core;

  bfin_core cores[0];
} bfin_cpu;

int big_endian = 0;
int debug_mode = 0;

static log_func bfin_log;
static bfin_cpu *cpu = NULL;
static int bfin_force_range_wp = 0;
static int bfin_unlock_on_connect = 0;
static int bfin_unlock_on_load = 0;
static int bfin_auto_switch = 1;
static struct timespec bfin_loop_wait_first_ts = {0, 50000000};
static struct timespec bfin_loop_wait_ts = {0, 10000000};
static struct timespec bfin_emu_wait_ts = {0, 5000000};
static int bfin_init_sdram = 0;
static int bfin_reset = 0;
static int bfin_enable_dcache = CACHE_DISABLED;
static int bfin_enable_icache = 0;


/* Local functions */

static char *bfin_out_treg (char *in, unsigned int reg_no);
static char *bfin_out_treg_value (char *in, unsigned int reg_no,
				  uint32_t value);

static uint32_t
gen_move (enum core_regnum dest, enum core_regnum src)
{
  uint32_t insn;

  insn = 0x3000;
  insn |= src & 0xf;
  insn |= (dest & 0xf) << 3;
  insn |= GROUP (src) << 6;
  insn |= GROUP (dest) << 9;

  return insn << 16;
}

static uint32_t
gen_ldstidxi (enum core_regnum reg,
	      enum core_regnum ptr, int32_t offset, int w, int sz)
{
  uint32_t insn;

  insn = 0xe4000000;
  insn |= (reg & 0xf) << 16;
  insn |= (ptr & 0xf) << 19;

  switch (sz)
    {
    case 0:
      offset >>= 2;
      break;
    case 1:
      offset >>= 1;
      break;
    case 2:
      break;
    default:
      abort ();
    }
  if (offset > 32767 || offset < -32768)
    abort ();
  insn |= offset & 0xffff;

  insn |= w << 25;
  insn |= sz << 22;

  return insn;
}

static uint32_t
gen_load32_offset (enum core_regnum dest, enum core_regnum base, int32_t offset)
{
  return gen_ldstidxi (dest, base, offset, 0, 0);
}

static uint32_t
gen_store32_offset (enum core_regnum base, int32_t offset, enum core_regnum src)
{
  return gen_ldstidxi (src, base, offset, 1, 0);
}

static uint32_t
gen_load16z_offset (enum core_regnum dest, enum core_regnum base, int32_t offset)
{
  return gen_ldstidxi (dest, base, offset, 0, 1);
}

static uint32_t
gen_store16_offset (enum core_regnum base, int32_t offset, enum core_regnum src)
{
  return gen_ldstidxi (src, base, offset, 1, 1);
}

static uint32_t
gen_load8z_offset (enum core_regnum dest, enum core_regnum base, int32_t offset)
{
  return gen_ldstidxi (dest, base, offset, 0, 2);
}

static uint32_t
gen_store8_offset (enum core_regnum base, int32_t offset, enum core_regnum src)
{
  return gen_ldstidxi (src, base, offset, 1, 2);
}

static uint32_t
gen_ldst (enum core_regnum reg,
	  enum core_regnum ptr, int post_dec, int w, int sz)
{
  uint32_t insn;

  insn = 0x9000;
  insn |= reg & 0xf;
  insn |= (ptr & 0xf) << 3;
  insn |= post_dec << 7;
  insn |= w << 9;
  insn |= sz << 10;

  return insn << 16;
}

static uint32_t
gen_load32pi (enum core_regnum dest, enum core_regnum base)
{
  return gen_ldst (dest, base, 0, 0, 0);
}

static uint32_t
gen_store32pi (enum core_regnum base, enum core_regnum src)
{
  return gen_ldst (src, base, 0, 1, 0);
}

static uint32_t
gen_load16zpi (enum core_regnum dest, enum core_regnum base)
{
  return gen_ldst (dest, base, 0, 0, 1);
}

static uint32_t
gen_store16pi (enum core_regnum base, enum core_regnum src)
{
  return gen_ldst (src, base, 0, 1, 1);
}

static uint32_t
gen_load8zpi (enum core_regnum dest, enum core_regnum base)
{
  return gen_ldst (dest, base, 0, 0, 2);
}

static uint32_t
gen_store8pi (enum core_regnum base, enum core_regnum src)
{
  return gen_ldst (src, base, 0, 1, 2);
}

static uint32_t
gen_load32 (enum core_regnum dest, enum core_regnum base)
{
  return gen_ldst (dest, base, 2, 0, 0);
}

static uint32_t
gen_store32 (enum core_regnum base, enum core_regnum src)
{
  return gen_ldst (src, base, 2, 1, 0);
}

static uint32_t
gen_load16z (enum core_regnum dest, enum core_regnum base)
{
  return gen_ldst (dest, base, 2, 0, 1);
}

static uint32_t
gen_store16 (enum core_regnum base, enum core_regnum src)
{
  return gen_ldst (src, base, 2, 1, 1);
}

static uint32_t
gen_load8z (enum core_regnum dest, enum core_regnum base)
{
  return gen_ldst (dest, base, 2, 0, 2);
}

static uint32_t
gen_store8 (enum core_regnum base, enum core_regnum src)
{
  return gen_ldst (src, base, 2, 1, 2);
}

/* op
   0  prefetch
   1  flushinv
   2  flush
   3  iflush  */
static uint32_t
gen_flush_insn (enum core_regnum addr, int op, int post_modify)
{
  uint32_t insn;

  insn = 0x0240;
  insn |= addr & 0xf;
  insn |= op << 3;
  insn |= post_modify << 5;

  return insn << 16;
}

static uint32_t
gen_iflush (enum core_regnum addr)
{
  return gen_flush_insn (addr, 3, 0);
}

static uint32_t
gen_iflush_pm (enum core_regnum addr)
{
  return gen_flush_insn (addr, 3, 1);
}

static uint32_t
gen_flush (enum core_regnum addr)
{
  return gen_flush_insn (addr, 2, 0);
}

static uint32_t
gen_flush_pm (enum core_regnum addr)
{
  return gen_flush_insn (addr, 2, 1);
}

static uint32_t
gen_flushinv (enum core_regnum addr)
{
  return gen_flush_insn (addr, 1, 0);
}

static uint32_t
gen_flushinv_pm (enum core_regnum addr)
{
  return gen_flush_insn (addr, 1, 1);
}

static uint32_t
gen_prefetch (enum core_regnum addr)
{
  return gen_flush_insn (addr, 0, 0);
}

static uint32_t
gen_prefetch_pm (enum core_regnum addr)
{
  return gen_flush_insn (addr, 0, 1);
}

static tap_register *
register_init_value (tap_register *tr, uint64_t value)
{
  int i;

  assert (tr->len <= 64);

  for (i = 0; i < tr->len; i++)
    tr->data[i] = (value >> (tr->len - i - 1)) & 1;

  return tr;
}

static uint64_t
register_value (tap_register *tr)
{
  uint64_t v = 0;
  int i;

  assert (tr->len <= 64);

  for (i = 0; i < tr->len; i++)
    v |= tr->data[i] << (tr->len - i - 1);

  return v;
}

static void
scan_select (int scan)
{
  int i;
  int changed;

  changed = 0;
  for (i = 0; i < cpu->chain->parts->len; i++)
    if (cpu->cores[i].scan != scan)
      {
	part_set_instruction (cpu->chain->parts->parts[i], scans[scan]);
	cpu->cores[i].scan = scan;
	changed = 1;
      }

  if (changed)
    chain_shift_instructions_mode (cpu->chain, 0, 1, EXITMODE_UPDATE);
}

static void
core_scan_select (int core, int scan)
{
  int i;
  int changed;

  assert (core >= 0 && core < cpu->chain->parts->len);

  changed = 0;

  if (cpu->cores[core].scan != scan)
    {
      part_set_instruction (cpu->chain->parts->parts[core], scans[scan]);
      cpu->cores[core].scan = scan;
      changed = 1;
    }

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (i != core && cpu->cores[i].scan != BYPASS)
      {
	part_set_instruction (cpu->chain->parts->parts[i], scans[BYPASS]);
	cpu->cores[i].scan = BYPASS;
	changed = 1;
      }

  if (changed)
    chain_shift_instructions_mode (cpu->chain, 0, 1, EXITMODE_UPDATE);
}

static void
dbgctl_set (uint16_t v, int runtest)
{
  part_t *part;
  int i;

  scan_select (DBGCTL_SCAN);

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      part = cpu->chain->parts->parts[i];
      cpu->cores[i].dbgctl |= v;
      register_init_value (part->active_instruction->data_register->in,
			   cpu->cores[i].dbgctl);
    }

  chain_shift_data_registers_mode (cpu->chain, 0, 1,
				   runtest ? EXITMODE_IDLE : EXITMODE_UPDATE);
}

static void
core_dbgctl_set (int core, uint16_t v, int runtest)
{
  part_t *part;

  assert (core >= 0 && core < cpu->chain->parts->len);

  core_scan_select (core, DBGCTL_SCAN);

  part = cpu->chain->parts->parts[core];
  cpu->cores[core].dbgctl |= v;
  register_init_value (part->active_instruction->data_register->in,
		       cpu->cores[core].dbgctl);

  chain_shift_data_registers_mode (cpu->chain, 0, 1,
				   runtest ? EXITMODE_IDLE : EXITMODE_UPDATE);
}

static void
dbgctl_clear (uint16_t v, int runtest)
{
  part_t *part;
  int i;

  scan_select (DBGCTL_SCAN);

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      part = cpu->chain->parts->parts[i];
      cpu->cores[i].dbgctl &= ~v;
      register_init_value (part->active_instruction->data_register->in,
			   cpu->cores[i].dbgctl);
    }

  chain_shift_data_registers_mode (cpu->chain, 0, 1,
				   runtest ? EXITMODE_IDLE : EXITMODE_UPDATE);
}

static void
core_dbgctl_clear (int core, uint16_t v, int runtest)
{
  part_t *part;

  assert (core >= 0 && core < cpu->chain->parts->len);

  core_scan_select (core, DBGCTL_SCAN);

  part = cpu->chain->parts->parts[core];
  cpu->cores[core].dbgctl &= ~v;
  register_init_value (part->active_instruction->data_register->in,
		       cpu->cores[core].dbgctl);

  chain_shift_data_registers_mode (cpu->chain, 0, 1,
				   runtest ? EXITMODE_IDLE : EXITMODE_UPDATE);
}

static void
dbgstat_get (void)
{
  tap_register *r;
  int i;

  scan_select (DBGSTAT_SCAN);

  chain_shift_data_registers_mode (cpu->chain, 1, 1, EXITMODE_UPDATE);
  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      r = cpu->chain->parts->parts[i]->active_instruction->data_register->out;
      cpu->cores[i].dbgstat = register_value (r);
    }
}

static void
core_dbgstat_get (int core)
{
  tap_register *r;

  assert (core >= 0 && core < cpu->chain->parts->len);

  core_scan_select (core, DBGSTAT_SCAN);

  chain_shift_data_registers_mode (cpu->chain, 1, 1, EXITMODE_UPDATE);

  r = cpu->chain->parts->parts[core]->active_instruction->data_register->out;
  cpu->cores[core].dbgstat = register_value (r);
}

static void
emupc_get (void)
{
  tap_register *r;
  int i;

  scan_select (EMUPC_SCAN);

  chain_shift_data_registers_mode (cpu->chain, 1, 1, EXITMODE_UPDATE);
  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      r = cpu->chain->parts->parts[i]->active_instruction->data_register->out;
      cpu->cores[i].emupc = register_value (r);
    }
}

static uint32_t
core_emupc_get (int core)
{
  tap_register *r;

  assert (core >= 0 && core < cpu->chain->parts->len);

  core_scan_select (core, EMUPC_SCAN);

  chain_shift_data_registers_mode (cpu->chain, 1, 1, EXITMODE_UPDATE);

  r = cpu->chain->parts->parts[core]->active_instruction->data_register->out;
  cpu->cores[core].emupc = register_value (r);

  return cpu->cores[core].emupc;
}

static void
dbgstat_clear_ovfs (void)
{
  tap_register *r;
  int i;

  scan_select (DBGSTAT_SCAN);

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      cpu->cores[i].dbgstat |= DBGSTAT_EMUDIOVF | DBGSTAT_EMUDOOVF;
      r = cpu->chain->parts->parts[i]->active_instruction->data_register->in;
      register_init_value (r, cpu->cores[i].dbgstat);
      cpu->cores[i].dbgstat &= ~(DBGSTAT_EMUDIOVF | DBGSTAT_EMUDOOVF);
    }

  chain_shift_data_registers_mode (cpu->chain, 0, 1, EXITMODE_UPDATE);
}

static void
core_dbgstat_clear_ovfs (int core)
{
  tap_register *r;

  assert (core >= 0 && core < cpu->chain->parts->len);

  core_scan_select (core, DBGSTAT_SCAN);

  cpu->cores[core].dbgstat |= DBGSTAT_EMUDIOVF | DBGSTAT_EMUDOOVF;
  r = cpu->chain->parts->parts[core]->active_instruction->data_register->in;
  register_init_value (r, cpu->cores[core].dbgstat);
  cpu->cores[core].dbgstat &= ~(DBGSTAT_EMUDIOVF | DBGSTAT_EMUDOOVF);

  chain_shift_data_registers_mode (cpu->chain, 0, 1, EXITMODE_UPDATE);
}

static void
dbgstat_show (const char *id)
{
  int i;

  assert (id != NULL);

  dbgstat_get ();
  for (i = 0; i < cpu->chain->parts->len; i++)
    bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	      "<%s> [%d] DBGSTAT [0x%08X]", id, i, cpu->cores[i].dbgstat);
}

static void
core_dbgstat_show (int core, const char *id)
{
  assert (id != NULL);

  core_dbgstat_get (core);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "<%s> [%d] DBGSTAT [0x%08X]", id, core, cpu->cores[core].dbgstat);
}

static void
wait_emuready (void)
{
  int emuready;
  int i;
  int waited = 0;

try_again:

  dbgstat_get ();
  emuready = DBGSTAT_EMUREADY;
  for (i = 0; i < cpu->chain->parts->len; i++)
    emuready &= cpu->cores[i].dbgstat & DBGSTAT_EMUREADY;

  if (waited)
    assert (emuready);

  if (!emuready)
    {
      nanosleep (&bfin_emu_wait_ts, NULL);
      waited = 1;
      goto try_again;
    }
}

static void
core_wait_emuready (int core)
{
  int emuready;
  int waited = 0;

try_again:

  core_dbgstat_get (core);
  emuready = cpu->cores[core].dbgstat & DBGSTAT_EMUREADY;

  if (waited)
    assert (emuready);

  if (!emuready)
    {
      nanosleep (&bfin_emu_wait_ts, NULL);
      waited = 1;
      goto try_again;
    }
}

static void
wait_in_reset (void)
{
  int in_reset;
  int i;
  int waited = 0;

try_again:

  dbgstat_get ();
  in_reset = DBGSTAT_IN_RESET;
  for (i = 0; i < cpu->chain->parts->len; i++)
    in_reset &= cpu->cores[i].dbgstat & DBGSTAT_IN_RESET;

  if (waited)
    assert (in_reset);

  if (!in_reset)
    {
      nanosleep (&bfin_emu_wait_ts, NULL);
      waited = 1;
      goto try_again;
    }
}

static void
core_wait_in_reset (int core)
{
  int in_reset;
  int waited = 0;

try_again:

  core_dbgstat_get (core);
  in_reset = cpu->cores[core].dbgstat & DBGSTAT_IN_RESET;

  if (waited)
    assert (in_reset);

  if (!in_reset)
    {
      nanosleep (&bfin_emu_wait_ts, NULL);
      waited = 1;
      goto try_again;
    }
}

static void
wait_reset (void)
{
  int in_reset;
  int i;
  int waited = 0;

try_again:

  dbgstat_get ();
  in_reset = 0;
  for (i = 0; i < cpu->chain->parts->len; i++)
    in_reset |= cpu->cores[i].dbgstat & DBGSTAT_IN_RESET;

  if (waited)
    assert (!in_reset);

  if (in_reset)
    {
      nanosleep (&bfin_emu_wait_ts, NULL);
      waited = 1;
      goto try_again;
    }
}

static void
core_wait_reset (int core)
{
  int in_reset;
  int waited = 0;

try_again:

  core_dbgstat_get (core);
  in_reset = cpu->cores[core].dbgstat & DBGSTAT_IN_RESET;

  if (waited)
    assert (!in_reset);

  if (in_reset)
    {
      nanosleep (&bfin_emu_wait_ts, NULL);
      waited = 1;
      goto try_again;
    }
}

static void
emuir_set (uint32_t *insn, int runtest)
{
  part_t *part;
  int i;

  scan_select (EMUIR_SCAN);

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      part = cpu->chain->parts->parts[i];
      register_init_value (part->active_instruction->data_register->in,
			   insn[i]);
      cpu->cores[i].emuir_a = insn[i];
    }

  chain_shift_data_registers_mode (cpu->chain, 0, 1,
				   runtest ? EXITMODE_IDLE : EXITMODE_UPDATE);
  if (runtest)
    wait_emuready ();
}

static void
emuir_set_same (uint32_t insn, int runtest)
{
  part_t *part;
  int i;

  scan_select (EMUIR_SCAN);

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      part = cpu->chain->parts->parts[i];
      register_init_value (part->active_instruction->data_register->in, insn);
      cpu->cores[i].emuir_a = insn;
    }

  chain_shift_data_registers_mode (cpu->chain, 0, 1,
				   runtest ? EXITMODE_IDLE : EXITMODE_UPDATE);
  if (runtest)
    wait_emuready ();
}

static void
core_emuir_set (int core, uint32_t insn, int runtest)
{
  part_t *part;
  int *changed;
  int scan_changed;
  int i;

  assert (core >= 0 && core < cpu->chain->parts->len);

  changed = (int *) malloc (cpu->chain->parts->len *sizeof (int));

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (i == core && cpu->cores[i].emuir_a != insn)
      {
	cpu->cores[i].emuir_a = insn;
	changed[i] = 1;
      }
    else if (i != core && cpu->cores[i].emuir_a != INSN_NOP)
      {
	cpu->cores[i].emuir_a = INSN_NOP;
	changed[i] = 1;
      }
    else
      changed[i] = 0;

  scan_changed = 0;

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (changed[i] && cpu->cores[i].scan != EMUIR_SCAN)
      {
	part_set_instruction (cpu->chain->parts->parts[i], scans[EMUIR_SCAN]);
	cpu->cores[i].scan = EMUIR_SCAN;
	scan_changed = 1;
      }
    else if (!changed[i] && cpu->cores[i].scan != BYPASS)
      {
	part_set_instruction (cpu->chain->parts->parts[i], scans[BYPASS]);
	cpu->cores[i].scan = BYPASS;
	scan_changed = 1;
      }

  if (scan_changed)
    chain_shift_instructions_mode (cpu->chain, 0, 1, EXITMODE_UPDATE);

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (changed[i])
      {
	part = cpu->chain->parts->parts[i];
	register_init_value (part->active_instruction->data_register->in,
			     cpu->cores[i].emuir_a);
      }

  free (changed);

  chain_shift_data_registers_mode (cpu->chain, 0, 1,
				   runtest ? EXITMODE_IDLE : EXITMODE_UPDATE);
  if (runtest)
    core_wait_emuready (core);
}

static void
emuir_set_2 (uint32_t *insn1, uint32_t *insn2, int runtest)
{
  part_t *part;
  int i;

  scan_select (EMUIR_SCAN);

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      part = cpu->chain->parts->parts[i];
      register_init_value (part->active_instruction->data_register->in,
			   insn2[i]);
      cpu->cores[i].emuir_b = insn2[i];
    }

  chain_shift_data_registers_mode (cpu->chain, 0, 1, EXITMODE_UPDATE);

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      part = cpu->chain->parts->parts[i];
      register_init_value (part->active_instruction->data_register->in,
			   insn1[i]);
      cpu->cores[i].emuir_a = insn1[i];
    }

  chain_shift_data_registers_mode (cpu->chain, 0, 1,
				   runtest ? EXITMODE_IDLE : EXITMODE_UPDATE);
  if (runtest)
    wait_emuready ();
}

static void
emuir_set_same_2 (uint32_t insn1, uint32_t insn2, int runtest)
{
  part_t *part;
  int i;

  scan_select (EMUIR_SCAN);

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      part = cpu->chain->parts->parts[i];
      register_init_value (part->active_instruction->data_register->in,
			   insn2);
      cpu->cores[i].emuir_b = insn2;
    }

  chain_shift_data_registers_mode (cpu->chain, 0, 1, EXITMODE_UPDATE);

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      part = cpu->chain->parts->parts[i];
      register_init_value (part->active_instruction->data_register->in,
			   insn1);
      cpu->cores[i].emuir_a = insn1;
    }

  chain_shift_data_registers_mode (cpu->chain, 0, 1,
				   runtest ? EXITMODE_IDLE : EXITMODE_UPDATE);
  if (runtest)
    wait_emuready ();
}

static void
core_emuir_set_2 (int core, uint32_t insn1, uint32_t insn2, int runtest)
{
  part_t *part;
  int *changed;
  int scan_changed;
  int i;

  assert (core >= 0 && core < cpu->chain->parts->len);

  changed = (int *) malloc (cpu->chain->parts->len * sizeof (int));

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (i == core
	&& (cpu->cores[i].emuir_a != insn1 || cpu->cores[i].emuir_b != insn2))
      {
	cpu->cores[i].emuir_a = insn1;
	cpu->cores[i].emuir_b = insn2;
	changed[i] = 1;
      }
    else if (i != core && cpu->cores[i].emuir_a != INSN_NOP)
      {
	cpu->cores[i].emuir_a = INSN_NOP;
	changed[i] = 1;
      }
    else
      changed[i] = 0;

  scan_changed = 0;

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (changed[i] && cpu->cores[i].scan != EMUIR_SCAN)
      {
	part_set_instruction (cpu->chain->parts->parts[i], scans[EMUIR_SCAN]);
	cpu->cores[i].scan = EMUIR_SCAN;
	scan_changed = 1;
      }
    else if (!changed[i] && cpu->cores[i].scan != BYPASS)
      {
	part_set_instruction (cpu->chain->parts->parts[i], scans[BYPASS]);
	cpu->cores[i].scan = BYPASS;
	scan_changed = 1;
      }

  if (scan_changed)
    chain_shift_instructions_mode (cpu->chain, 0, 1, EXITMODE_UPDATE);

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (changed[i] && i == core)
      {
	part = cpu->chain->parts->parts[i];
	register_init_value (part->active_instruction->data_register->in,
			     insn2);
	chain_shift_data_registers_mode (cpu->chain, 0, 1, EXITMODE_UPDATE);

	register_init_value (part->active_instruction->data_register->in,
			     insn1);
      }
    else if (changed[i] && i != core)
      {
	part = cpu->chain->parts->parts[i];
	register_init_value (part->active_instruction->data_register->in,
			     cpu->cores[i].emuir_a);
      }

  free (changed);

  chain_shift_data_registers_mode (cpu->chain, 0, 1,
				   runtest ? EXITMODE_IDLE : EXITMODE_UPDATE);
  if (runtest)
    core_wait_emuready (core);
}


static uint32_t
core_emudat_get (int core, int runtest)
{
  tap_register *r;

  if (runtest)
    {
      assert (tap_state (cpu->chain) & TAPSTAT_IDLE);
      chain_clock (cpu->chain, 0, 0, 1);
      core_wait_emuready (core);
    }

  core_scan_select (core, EMUDAT_SCAN);

  chain_shift_data_registers_mode (cpu->chain, 1, 1, EXITMODE_UPDATE);
  r = cpu->chain->parts->parts[core]->active_instruction->data_register->out;
  return register_value (r);
}

static void
core_emudat_set (int core, uint32_t value, int runtest)
{
  tap_register *r;

  core_scan_select (core, EMUDAT_SCAN);

  r = cpu->chain->parts->parts[core]->active_instruction->data_register->in;
  register_init_value (r, value);
  chain_shift_data_registers_mode (cpu->chain, 0, 1,
				   runtest ? EXITMODE_IDLE : EXITMODE_UPDATE);
  if (runtest)
    core_wait_emuready (core);
}

/* Forward declarations */
static void register_set (enum core_regnum reg, uint32_t *value);
static void core_register_set (int core, enum core_regnum reg,
			       uint32_t value);

static void
register_get (enum core_regnum reg, uint32_t *value)
{
  tap_register *r;
  int i;
  uint32_t *r0 = NULL;

  if (DREG_P (reg) || PREG_P (reg) || reg == REG_USP)
    emuir_set_same (gen_move (REG_EMUDAT, reg), RUNTEST);
  else
    {
      r0 = (uint32_t *)malloc (cpu->chain->parts->len * sizeof (uint32_t));
      if (!r0)
	abort ();

      register_get (REG_R0, r0);
      dbgctl_set (DBGCTL_EMUIRLPSZ_2, UPDATE);
      emuir_set_same_2 (gen_move (REG_R0, reg),
			gen_move (REG_EMUDAT, REG_R0), RUNTEST);
      dbgctl_clear (DBGCTL_EMUIRLPSZ_2, UPDATE);
    }

  scan_select (EMUDAT_SCAN);
  chain_shift_data_registers_mode (cpu->chain, 1, 1, EXITMODE_UPDATE);
  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      r = cpu->chain->parts->parts[i]->active_instruction->data_register->out;
      value[i] = register_value (r);
    }

  if (!DREG_P (reg) && !PREG_P (reg) && reg != REG_USP)
    {
      register_set (REG_R0, r0);
      free (r0);
    }
}

static uint32_t
core_register_get (int core, enum core_regnum reg)
{
  tap_register *r;
  uint32_t r0 = 0;

  if (DREG_P (reg) || PREG_P (reg) || reg == REG_USP)
    core_emuir_set (core, gen_move (REG_EMUDAT, reg), RUNTEST);
  else
    {
      r0 = core_register_get (core, REG_R0);
      core_dbgctl_set (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
      core_emuir_set_2 (core, gen_move (REG_R0, reg),
			gen_move (REG_EMUDAT, REG_R0), RUNTEST);
      core_dbgctl_clear (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
    }

  core_scan_select (core, EMUDAT_SCAN);
  chain_shift_data_registers_mode (cpu->chain, 1, 1, EXITMODE_UPDATE);
  r = cpu->chain->parts->parts[core]->active_instruction->data_register->out;

  if (!DREG_P (reg) && !PREG_P (reg) && reg != REG_USP)
    core_register_set (core, REG_R0, r0);

  return register_value (r);
}

static void
register_set (enum core_regnum reg, uint32_t *value)
{
  tap_register *r;
  int i;
  uint32_t *r0 = NULL;

  if (!DREG_P (reg) && !PREG_P (reg) && reg != REG_USP)
    {
      r0 = (uint32_t *)malloc (cpu->chain->parts->len * sizeof (uint32_t));
      if (!r0)
	abort ();

      register_get (REG_R0, r0);
    }

  scan_select (EMUDAT_SCAN);
  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      r = cpu->chain->parts->parts[i]->active_instruction->data_register->in;
      register_init_value (r, value[i]);
    }
  chain_shift_data_registers_mode (cpu->chain, 0, 1, EXITMODE_UPDATE);

  if (DREG_P (reg) || PREG_P (reg) || reg == REG_USP)
    emuir_set_same (gen_move (reg, REG_EMUDAT), RUNTEST);
  else
    {
      dbgctl_set (DBGCTL_EMUIRLPSZ_2, UPDATE);
      emuir_set_same_2 (gen_move (REG_R0, REG_EMUDAT),
			gen_move (reg, REG_R0), RUNTEST);
      dbgctl_clear (DBGCTL_EMUIRLPSZ_2, UPDATE);
      register_set (REG_R0, r0);
      free (r0);
    }
}

static void
register_set_same (enum core_regnum reg, uint32_t value)
{
  tap_register *r;
  int i;
  uint32_t *r0 = NULL;

  if (!DREG_P (reg) && !PREG_P (reg) && reg != REG_USP)
    {
      r0 = (uint32_t *)malloc (cpu->chain->parts->len * sizeof (uint32_t));
      if (!r0)
	abort ();

      register_get (REG_R0, r0);
    }

  scan_select (EMUDAT_SCAN);
  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      r = cpu->chain->parts->parts[i]->active_instruction->data_register->in;
      register_init_value (r, value);
    }
  chain_shift_data_registers_mode (cpu->chain, 0, 1, EXITMODE_UPDATE);

  if (DREG_P (reg) || PREG_P (reg) || reg == REG_USP)
    emuir_set_same (gen_move (reg, REG_EMUDAT), RUNTEST);
  else
    {
      dbgctl_set (DBGCTL_EMUIRLPSZ_2, UPDATE);
      emuir_set_same_2 (gen_move (REG_R0, REG_EMUDAT),
			gen_move (reg, REG_R0), RUNTEST);
      dbgctl_clear (DBGCTL_EMUIRLPSZ_2, UPDATE);
      register_set (REG_R0, r0);
      free (r0);
    }
}

static void
core_register_set (int core, enum core_regnum reg, uint32_t value)
{
  tap_register *r;
  uint32_t r0 = 0;

  if (!DREG_P (reg) && !PREG_P (reg) && reg != REG_USP)
    r0 = core_register_get (core, REG_R0);

  core_scan_select (core, EMUDAT_SCAN);

  cpu->cores[core].emudat_in = value;
  r = cpu->chain->parts->parts[core]->active_instruction->data_register->in;
  register_init_value (r, value);

  chain_shift_data_registers_mode (cpu->chain, 0, 1, EXITMODE_UPDATE);

  if (DREG_P (reg) || PREG_P (reg) || reg == REG_USP)
    core_emuir_set (core, gen_move (reg, REG_EMUDAT), RUNTEST);
  else
    {
      core_dbgctl_set (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
      core_emuir_set_2 (core, gen_move (REG_R0, REG_EMUDAT),
			gen_move (reg, REG_R0), RUNTEST);
      core_dbgctl_clear (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
      core_register_set (core, REG_R0, r0);
    }
}

static void
wpu_init (void)
{
  uint32_t *p0, *r0;
  uint32_t wpiactl, wpdactl;
  int i;

  p0 = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));
  r0 = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));

  if (!p0 || !r0)
    abort ();

  register_get (REG_P0, p0);
  register_get (REG_R0, r0);

  register_set_same (REG_P0, WPIACTL);

  wpiactl = WPIACTL_WPPWR;

  register_set_same (REG_R0, wpiactl);

  emuir_set_same (gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);

  wpiactl |= WPIACTL_EMUSW5 | WPIACTL_EMUSW4 | WPIACTL_EMUSW3;
  wpiactl |= WPIACTL_EMUSW2 | WPIACTL_EMUSW1 | WPIACTL_EMUSW0;

  register_set_same (REG_R0, wpiactl);
  emuir_set_same (gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);

  wpdactl = WPDACTL_WPDSRC1_A | WPDACTL_WPDSRC0_A;

  register_set_same (REG_R0, wpdactl);
  emuir_set_same (gen_store32_offset (REG_P0, WPDACTL - WPIACTL, REG_R0), RUNTEST);

  register_set_same (REG_R0, 0);
  emuir_set_same (gen_store32_offset (REG_P0, WPSTAT - WPIACTL, REG_R0), RUNTEST);

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      cpu->cores[i].wpiactl = wpiactl;
      cpu->cores[i].wpdactl = wpdactl;
    }

  register_set (REG_P0, p0);
  register_set (REG_R0, r0);

  free (p0);
  free (r0);
}

static void
core_wpu_init (int core)
{
  uint32_t p0, r0;
  uint32_t wpiactl, wpdactl;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, WPIACTL);

  wpiactl = WPIACTL_WPPWR;

  core_register_set (core, REG_R0, wpiactl);

  core_emuir_set (core, gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);

  wpiactl |= WPIACTL_EMUSW5 | WPIACTL_EMUSW4 | WPIACTL_EMUSW3;
  wpiactl |= WPIACTL_EMUSW2 | WPIACTL_EMUSW1 | WPIACTL_EMUSW0;

  core_register_set (core, REG_R0, wpiactl);
  core_emuir_set (core, gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);

  wpdactl = WPDACTL_WPDSRC1_A | WPDACTL_WPDSRC0_A;

  core_register_set (core, REG_R0, wpdactl);
  core_emuir_set (core, gen_store32_offset (REG_P0, WPDACTL - WPIACTL, REG_R0),
		  RUNTEST);

  core_register_set (core, REG_R0, 0);
  core_emuir_set (core, gen_store32_offset (REG_P0, WPSTAT - WPIACTL, REG_R0),
		  RUNTEST);

  cpu->cores[core].wpiactl = wpiactl;
  cpu->cores[core].wpdactl = wpdactl;

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
}

static void
wpu_enable (void)
{
  uint32_t *p0, *r0, *r0new;
  int i;

  p0 = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));
  r0 = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));
  r0new = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));

  if (!p0 || !r0 || !r0new)
    abort ();

  register_get (REG_P0, p0);
  register_get (REG_R0, r0);

  register_set_same (REG_P0, WPIACTL);
  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      cpu->cores[i].wpiactl |= WPIACTL_WPPWR;
      r0new[i] = cpu->cores[i].wpdactl;
    }
  register_set (REG_R0, r0new);
  emuir_set_same (gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);

  register_set (REG_P0, p0);
  register_set (REG_R0, r0);

  free (p0);
  free (r0);
  free (r0new);
}

static void
core_wpu_enable (int core)
{
  uint32_t p0, r0, r0new;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, WPIACTL);
  cpu->cores[core].wpiactl |= WPIACTL_WPPWR;
  r0new = cpu->cores[core].wpdactl;
  core_register_set (core, REG_R0, r0new);
  core_emuir_set (core, gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
}


static void
wpu_disable (void)
{
  uint32_t *p0, *r0, *r0new;
  int i;

  p0 = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));
  r0 = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));
  r0new = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));

  if (!p0 || !r0 || !r0new)
    abort ();

  register_get (REG_P0, p0);
  register_get (REG_R0, r0);

  register_set_same (REG_P0, WPIACTL);
  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      cpu->cores[i].wpiactl &= ~WPIACTL_WPPWR;
      r0new[i] = cpu->cores[i].wpdactl;
    }
  register_set (REG_R0, r0new);
  emuir_set_same (gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);

  register_set (REG_P0, p0);
  register_set (REG_R0, r0);

  free (p0);
  free (r0);
  free (r0new);
}

static void
core_wpu_disable (int core)
{
  uint32_t p0, r0, r0new;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, WPIACTL);
  cpu->cores[core].wpiactl &= ~WPIACTL_WPPWR;
  r0new = cpu->cores[core].wpdactl;
  core_register_set (core, REG_R0, r0new);
  core_emuir_set (core, gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
}

static uint32_t wpiaen[] = {
  WPIACTL_WPIAEN0,
  WPIACTL_WPIAEN1,
  WPIACTL_WPIAEN2,
  WPIACTL_WPIAEN3,
  WPIACTL_WPIAEN4,
  WPIACTL_WPIAEN5,
};

static void
wpu_set_wpia (int core, int n, uint32_t addr, int enable)
{
  uint32_t p0, r0;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, WPIACTL);
  if (enable)
    {
      cpu->cores[core].wpiactl |= wpiaen[n];
      core_register_set (core, REG_R0, addr);
      core_emuir_set (core,
		      gen_store32_offset (REG_P0, WPIA0 + 4 * n - WPIACTL, REG_R0),
		      RUNTEST);
    }
  else
    cpu->cores[core].wpiactl &= ~wpiaen[n];

  core_register_set (core, REG_R0, cpu->cores[core].wpiactl);
  core_emuir_set (core, gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
}

static void
wpu_set_wpda (int core, int n, uint32_t addr, uint32_t len, int range,
	      int mode)
{
  uint32_t p0, r0;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, WPDACTL);

  if (len > 4 || range)
    {
      assert (n == 0);

      switch (mode)
	{
	case WPDA_DISABLE:
	  cpu->cores[core].wpdactl &= ~WPDACTL_WPDREN01;
	  break;
	case WPDA_WRITE:
	  cpu->cores[core].wpdactl &= ~WPDACTL_WPDACC0_R;
	  cpu->cores[core].wpdactl |= WPDACTL_WPDREN01 | WPDACTL_WPDACC0_W;
	  break;
	case WPDA_READ:
	  cpu->cores[core].wpdactl &= ~WPDACTL_WPDACC0_W;
	  cpu->cores[core].wpdactl |= WPDACTL_WPDREN01 | WPDACTL_WPDACC0_R;
	  break;
	case WPDA_ALL:
	  cpu->cores[core].wpdactl |= WPDACTL_WPDREN01 | WPDACTL_WPDACC0_A;
	  break;
	default:
	  abort ();
	}

      if (mode != WPDA_DISABLE)
	{
	  core_register_set (core, REG_R0, addr - 1);
	  core_emuir_set (core,
			  gen_store32_offset (REG_P0, WPDA0 - WPDACTL, REG_R0),
			  RUNTEST);
	  core_register_set (core, REG_R0, addr + len - 1);
	  core_emuir_set (core,
			  gen_store32_offset (REG_P0, WPDA0 + 4 - WPDACTL, REG_R0),
			  RUNTEST);
	}
    }
  else
    {
      if (n == 0)
	switch (mode)
	  {
	  case WPDA_DISABLE:
	    cpu->cores[core].wpdactl &= ~WPDACTL_WPDAEN0;
	    break;
	  case WPDA_WRITE:
	    cpu->cores[core].wpdactl &= ~WPDACTL_WPDACC0_R;
	    cpu->cores[core].wpdactl |= WPDACTL_WPDAEN0 | WPDACTL_WPDACC0_W;
	    break;
	  case WPDA_READ:
	    cpu->cores[core].wpdactl &= ~WPDACTL_WPDACC0_W;
	    cpu->cores[core].wpdactl |= WPDACTL_WPDAEN0 | WPDACTL_WPDACC0_R;
	    break;
	  case WPDA_ALL:
	    cpu->cores[core].wpdactl |= WPDACTL_WPDAEN0 | WPDACTL_WPDACC0_A;
	    break;
	  default:
	    abort ();
	  }
      else
	switch (mode)
	  {
	  case WPDA_DISABLE:
	    cpu->cores[core].wpdactl &= ~WPDACTL_WPDAEN1;
	    break;
	  case WPDA_WRITE:
	    cpu->cores[core].wpdactl &= ~WPDACTL_WPDACC1_R;
	    cpu->cores[core].wpdactl |= WPDACTL_WPDAEN1 | WPDACTL_WPDACC1_W;
	    break;
	  case WPDA_READ:
	    cpu->cores[core].wpdactl &= ~WPDACTL_WPDACC1_W;
	    cpu->cores[core].wpdactl |= WPDACTL_WPDAEN1 | WPDACTL_WPDACC1_R;
	    break;
	  case WPDA_ALL:
	    cpu->cores[core].wpdactl |= WPDACTL_WPDAEN1 | WPDACTL_WPDACC1_A;
	    break;
	  default:
	    abort ();
	  }
      if (mode != WPDA_DISABLE)
	{
	  core_register_set (core, REG_R0, addr);
	  core_emuir_set (core,
			  gen_store32_offset (REG_P0, WPDA0 + 4 * n - WPDACTL,
					      REG_R0), RUNTEST);
	}
    }

  core_register_set (core, REG_R0, cpu->cores[core].wpdactl);
  core_emuir_set (core, gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
}

static void
wpstat_get (void)
{
  uint32_t *p0, *r0, *wpstat;
  int i;

  p0 = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));
  r0 = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));
  wpstat = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));

  if (!p0 || !r0 || !wpstat)
    abort ();

  register_get (REG_P0, p0);
  register_get (REG_R0, r0);

  register_set_same (REG_P0, WPSTAT);
  emuir_set_same (gen_load32_offset (REG_R0, REG_P0, 0), RUNTEST);
  register_get (REG_R0, wpstat);
  for (i = 0; i < cpu->chain->parts->len; i++)
    cpu->cores[i].wpstat = wpstat[i];

  register_set (REG_P0, p0);
  register_set (REG_R0, r0);

  free (p0);
  free (r0);
  free (wpstat);
}


static void
wpstat_clear (void)
{
  uint32_t *p0, *r0, *wpstat;
  int i;

  p0 = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));
  r0 = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));
  wpstat = (uint32_t *) malloc (cpu->chain->parts->len * sizeof (uint32_t));

  if (!p0 || !r0 || !wpstat)
    abort ();

  register_get (REG_P0, p0);
  register_get (REG_R0, r0);

  register_set_same (REG_P0, WPSTAT);

  for (i = 0; i < cpu->chain->parts->len; i++)
    wpstat[i] = cpu->cores[i].wpstat;
  register_set (REG_R0, wpstat);

  emuir_set_same (gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);

  for (i = 0; i < cpu->chain->parts->len; i++)
    cpu->cores[i].wpstat = 0;

  register_set (REG_P0, p0);
  register_set (REG_R0, r0);

  free (p0);
  free (r0);
  free (wpstat);
}

static uint32_t
core_wpstat_get (int core)
{
  uint32_t p0, r0;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, WPSTAT);
  core_emuir_set (core, gen_load32_offset (REG_R0, REG_P0, 0), RUNTEST);
  cpu->cores[core].wpstat = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);

  return cpu->cores[core].wpstat;
}

static uint32_t
core_wpstat_clear (int core)
{
  uint32_t p0, r0;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "[%d] clear wpstat", core);

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, WPSTAT);
  core_register_set (core, REG_R0, cpu->cores[core].wpstat);
  core_emuir_set (core, gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);
  cpu->cores[core].wpstat = 0;

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);

  core_wpstat_get (core);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "[%d]   WPSTAT [0x%08X]", core,
	    cpu->cores[core].wpstat);

  return cpu->cores[core].wpstat;
}

static void
emulation_enable (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: emulation_enable ()", bfin_target.name);

  dbgctl_set (DBGCTL_EMPWR, UPDATE);
  dbgstat_show ("EMPWR");
  dbgctl_set (DBGCTL_EMFEN, UPDATE);
  dbgstat_show ("EMFEN");
  dbgctl_set (DBGCTL_EMUIRSZ_32 | DBGCTL_EMUDATSZ_32, UPDATE);
  dbgstat_show ("EMUIRSZ|EMUDATSZ");
}

static void
core_emulation_enable (int core)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_emulation_enable ()", bfin_target.name, core);

  core_dbgctl_set (core, DBGCTL_EMPWR, UPDATE);
  core_dbgstat_show (core, "EMPWR");
  core_dbgctl_set (core, DBGCTL_EMFEN, UPDATE);
  core_dbgstat_show (core, "EMFEN");
  core_dbgctl_set (core, DBGCTL_EMUIRSZ_32 | DBGCTL_EMUDATSZ_32, UPDATE);
  core_dbgstat_show (core, "EMUIRSZ|EMUDATSZ");
}

static void
emulation_trigger (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: emulation_trigger ()", bfin_target.name);

  emuir_set_same (INSN_NOP, UPDATE);
  dbgstat_show ("before");
  dbgctl_set (DBGCTL_EMEEN | DBGCTL_EMPEN | DBGCTL_WAKEUP, RUNTEST);
  dbgstat_show ("after");
}

static void
core_emulation_trigger (int core)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_emulation_trigger ()", bfin_target.name, core);

  core_emuir_set (core, INSN_NOP, UPDATE);
  core_dbgstat_show (core, "before");
  core_dbgctl_set (core, DBGCTL_EMEEN | DBGCTL_EMPEN | DBGCTL_WAKEUP,
		   RUNTEST);
  core_dbgstat_show (core, "after");
}

static void
emulation_return (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: emulation_return ()", bfin_target.name);

  emuir_set_same (INSN_RTE, UPDATE);
  dbgstat_show ("before");
  dbgctl_clear (DBGCTL_EMEEN | DBGCTL_WAKEUP, RUNTEST);
  dbgstat_show ("after");
}

static void
core_emulation_return (int core)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_emulation_return ()", bfin_target.name, core);

  core_emuir_set (core, INSN_RTE, UPDATE);
  core_dbgstat_show (core, "before");
  core_dbgctl_clear (core, DBGCTL_EMEEN | DBGCTL_WAKEUP, RUNTEST);
  core_dbgstat_show (core, "after");
}

static void
emulation_disable (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: emulation_disable ()", bfin_target.name);

  dbgctl_clear (DBGCTL_EMPWR, UPDATE);
}

static void
core_emulation_disable (int core)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_emulation_disable ()", bfin_target.name, core);

  core_dbgctl_clear (core, DBGCTL_EMPWR, UPDATE);
}

static void
system_reset (void)
{
  uint32_t p0, r0;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "Reset system");

  p0 = core_register_get (cpu->core_a, REG_P0);
  r0 = core_register_get (cpu->core_a, REG_R0);

  core_register_set (cpu->core_a, REG_P0, SWRST);
  core_register_set (cpu->core_a, REG_R0, 0x7);
  core_emuir_set (cpu->core_a, gen_store16_offset (REG_P0, 0, REG_R0), RUNTEST);
  core_emuir_set (cpu->core_a, INSN_SSYNC, RUNTEST);
  core_register_set (cpu->core_a, REG_R0, 0);
  core_emuir_set (cpu->core_a, gen_store16_offset (REG_P0, 0, REG_R0), RUNTEST);
  core_emuir_set (cpu->core_a, INSN_SSYNC, RUNTEST);

  core_register_set (cpu->core_a, REG_P0, p0);
  core_register_set (cpu->core_a, REG_R0, r0);
}

static void
core_reset (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "Reset core(s)");

  emulation_disable ();

  core_emuir_set (cpu->core_a, INSN_NOP, UPDATE);
  dbgctl_set (DBGCTL_SRAM_INIT, UPDATE);
  core_dbgctl_set (cpu->core_a, DBGCTL_SYSRST, RUNTEST);
  wait_in_reset ();
  core_dbgctl_clear (cpu->core_a, DBGCTL_SYSRST, UPDATE);
  wait_reset ();

  emulation_enable ();
  emulation_trigger ();

  dbgctl_clear (DBGCTL_SRAM_INIT, UPDATE);
}

static uint32_t
mmr_read_clobber_p0r0 (int core, int32_t offset, size_t size)
{
  uint32_t value;

  assert (size == 2 || size == 4);

  if (offset == 0)
    {
      core_dbgctl_set (core, DBGCTL_EMUIRLPSZ_2, UPDATE);

      if (size == 2)
	core_emuir_set_2 (core,
			  gen_load16z (REG_R0, REG_P0),
			  gen_move (REG_EMUDAT, REG_R0), UPDATE);
      else
	core_emuir_set_2 (core,
			  gen_load32 (REG_R0, REG_P0),
			  gen_move (REG_EMUDAT, REG_R0), UPDATE);
    }
  else
    {
      if (size == 2)
	core_emuir_set (core, gen_load16z_offset (REG_R0, REG_P0, offset), RUNTEST);
      else
	core_emuir_set (core, gen_load32_offset (REG_R0, REG_P0, offset), RUNTEST);
      core_emuir_set (core, gen_move (REG_EMUDAT, REG_R0), UPDATE);
    }
  value = core_emudat_get (core, RUNTEST);

  if (offset == 0)
    core_dbgctl_clear (core, DBGCTL_EMUIRLPSZ_2, UPDATE);

  return value;
}

static uint32_t
mmr_read (int core, uint32_t addr, size_t size)
{
  uint32_t p0, r0;
  uint32_t value;

  if (addr == DMEM_CONTROL)
    {
      if (size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] DMEM_CONTROL can only be accessed as 32-bit word",
		    bfin_target.name,
		    core);
	  /* Return a weird value to notice people.  */
	  return 0xfffffff;
	}

      if (cpu->cores[core].dmem_control_valid_p)
	return cpu->cores[core].dmem_control;
    }
  else if (addr == IMEM_CONTROL)
    {
      if (size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] IMEM_CONTROL can only be accessed as 32-bit word",
		    bfin_target.name,
		    core);
	  /* Return a weird value to notice people.  */
	  return 0xfffffff;
	}

      if (cpu->cores[core].imem_control_valid_p)
	return cpu->cores[core].imem_control;
    }
  else if (addr >= DCPLB_ADDR0 && addr < DCPLB_ADDR0 + 4 * BFIN_DCPLB_NUM)
    {
      if ((addr & 0x3) != 0 || size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] misaligned or wrong size access to DCPLB_ADDRx",
		    bfin_target.name,
		    core);
	  /* Return a weird value to notice people.  */
	  return 0xfffffff;
	}

      if (cpu->cores[core].dcplbs_valid_p)
	return cpu->cores[core].dcplbs[(addr - DCPLB_ADDR0) / 4].addr;
    }
  else if (addr >= DCPLB_DATA0 && addr < DCPLB_DATA0 + 4 * BFIN_DCPLB_NUM)
    {
      if ((addr & 0x3) != 0 || size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] misaligned or wrong size access to DCPLB_DATAx",
		    bfin_target.name,
		    core);
	  /* Return a weird value to notice people.  */
	  return 0xfffffff;
	}

      if (cpu->cores[core].dcplbs_valid_p)
	return cpu->cores[core].dcplbs[(addr - DCPLB_DATA0) / 4].data;
    }
  else if (addr >= ICPLB_ADDR0 && addr < ICPLB_ADDR0 + 4 * BFIN_ICPLB_NUM)
    {
      if ((addr & 0x3) != 0 || size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] misaligned or wrong size access to ICPLB_ADDRx",
		    bfin_target.name,
		    core);
	  /* Return a weird value to notice people.  */
	  return 0xfffffff;
	}

      if (cpu->cores[core].icplbs_valid_p)
	return cpu->cores[core].icplbs[(addr - ICPLB_ADDR0) / 4].addr;
    }
  else if (addr >= ICPLB_DATA0 && addr < ICPLB_DATA0 + 4 * BFIN_ICPLB_NUM)
    {
      if ((addr & 0x3) != 0 || size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] misaligned or wrong size access to ICPLB_DATAx",
		    bfin_target.name,
		    core);
	  /* Return a weird value to notice people.  */
	  return 0xfffffff;
	}

      if (cpu->cores[core].icplbs_valid_p)
	return cpu->cores[core].icplbs[(addr - ICPLB_DATA0) / 4].data;
    }

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, addr);
  value = mmr_read_clobber_p0r0 (core, 0, size);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);

  if (addr == DMEM_CONTROL)
    {
      cpu->cores[core].dmem_control = value;
      cpu->cores[core].dmem_control_valid_p = 1;
    }
  else if (addr == IMEM_CONTROL)
    {
      cpu->cores[core].imem_control = value;
      cpu->cores[core].imem_control_valid_p = 1;
    }

  return value;
}

static void
mmr_write_clobber_p0r0 (int core, int32_t offset, uint32_t data, size_t size)
{
  assert (size == 2 || size == 4);

  core_emudat_set (core, data, UPDATE);

  if (offset == 0)
    {
      core_dbgctl_set (core, DBGCTL_EMUIRLPSZ_2, UPDATE);

      if (size == 2)
	core_emuir_set_2 (core,
			  gen_move (REG_R0, REG_EMUDAT),
			  gen_store16 (REG_P0, REG_R0), RUNTEST);
      else
	core_emuir_set_2 (core,
			  gen_move (REG_R0, REG_EMUDAT),
			  gen_store32 (REG_P0, REG_R0), RUNTEST);
    }
  else
    {
      core_emuir_set (core, gen_move (REG_R0, REG_EMUDAT), RUNTEST);
      if (size == 2)
	core_emuir_set (core, gen_store16_offset (REG_P0, offset, REG_R0), RUNTEST);
      else
	core_emuir_set (core, gen_store32_offset (REG_P0, offset, REG_R0), RUNTEST);
    }

  if (offset == 0)
    core_dbgctl_clear (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
}

static void
mmr_write (int core, uint32_t addr, uint32_t data, size_t size)
{
  uint32_t p0, r0;

  if (addr == DMEM_CONTROL)
    {
      if (size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] DMEM_CONTROL can only be accessed as 32-bit word",
		    bfin_target.name,
		    core);
	  return;
	}

      if (cpu->cores[core].dmem_control_valid_p
	  && cpu->cores[core].dmem_control == data)
	return;
      else
	{
	  cpu->cores[core].dmem_control = data;
	  cpu->cores[core].dmem_control_valid_p = 1;
	}
    }
  else if (addr == IMEM_CONTROL)
    {
      if (size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] IMEM_CONTROL can only be accessed as 32-bit word",
		    bfin_target.name,
		    core);
	  return;
	}

      if (cpu->cores[core].imem_control_valid_p
	  && cpu->cores[core].imem_control == data)
	return;
      else
	{
	  cpu->cores[core].imem_control = data;
	  cpu->cores[core].imem_control_valid_p = 1;
	}
    }
  else if (addr >= DCPLB_ADDR0 && addr < DCPLB_ADDR0 + 4 * BFIN_DCPLB_NUM)
    {
      if ((addr & 0x3) != 0 || size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] misaligned or wrong size access to DCPLB_ADDRx",
		    bfin_target.name,
		    core);
	  return;
	}

      if (cpu->cores[core].dcplbs_valid_p
	  && cpu->cores[core].dcplbs[(addr - DCPLB_ADDR0) / 4].addr == data)
	return;
      else
	cpu->cores[core].dcplbs[(addr - DCPLB_ADDR0) / 4].addr = data;
    }
  else if (addr >= DCPLB_DATA0 && addr < DCPLB_DATA0 + 4 * BFIN_DCPLB_NUM)
    {
      if ((addr & 0x3) != 0 || size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] misaligned or wrong size access to DCPLB_DATAx",
		    bfin_target.name,
		    core);
	  return;
	}

      if (cpu->cores[core].dcplbs_valid_p
	  && cpu->cores[core].dcplbs[(addr - DCPLB_DATA0) / 4].data == data)
	return;
      else
	cpu->cores[core].dcplbs[(addr - DCPLB_DATA0) / 4].data = data;
    }
  else if (addr >= ICPLB_ADDR0 && addr < ICPLB_ADDR0 + 4 * BFIN_ICPLB_NUM)
    {
      if ((addr & 0x3) != 0 || size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] misaligned or wrong size access to ICPLB_ADDRx",
		    bfin_target.name,
		    core);
	  return;
	}

      if (cpu->cores[core].icplbs_valid_p
	  && cpu->cores[core].icplbs[(addr - ICPLB_ADDR0) / 4].addr == data)
	return;
      else
	cpu->cores[core].icplbs[(addr - ICPLB_ADDR0) / 4].addr = data;
    }
  else if (addr >= ICPLB_DATA0 && addr < ICPLB_DATA0 + 4 * BFIN_ICPLB_NUM)
    {
      if ((addr & 0x3) != 0 || size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] misaligned or wrong size access to ICPLB_DATAx",
		    bfin_target.name,
		    core);
	  return;
	}

      if (cpu->cores[core].icplbs_valid_p
	  && cpu->cores[core].icplbs[(addr - ICPLB_DATA0) / 4].data == data)
	return;
      else
	cpu->cores[core].icplbs[(addr - ICPLB_DATA0) / 4].data = data;
    }

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, addr);
  mmr_write_clobber_p0r0 (core, 0, data, size);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
}

static int
sdram_init (void)
{
  uint32_t p0, r0;
  uint32_t value;
  int core;
  int i;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: sdram_init ()", bfin_target.name);

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (!cpu->cores[i].is_locked
	&& !cpu->cores[i].is_corefault
	&& !cpu->cores[i].is_running)
      break;

  if (i == cpu->chain->parts->len)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: no core available to init sdram",
		bfin_target.name);
      return -1;
    }
  else
    core = i;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, EBIU_SDGCTL);

  /* Check if SDRAM has been enabled already.
     If so, don't enable it again.  */
  value = mmr_read_clobber_p0r0 (core, EBIU_SDSTAT - EBIU_SDGCTL, 2);
  if ((value & 0x8) == 0)
    {
      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: sdram has already been enabled", bfin_target.name);
      return 0;
    }

  mmr_write_clobber_p0r0 (core, EBIU_SDRRC - EBIU_SDGCTL,
			  cpu->sdram_config->sdrrc, 2);
  mmr_write_clobber_p0r0 (core, EBIU_SDBCTL - EBIU_SDGCTL,
			  cpu->sdram_config->sdbctl, 2);
  mmr_write_clobber_p0r0 (core, 0, cpu->sdram_config->sdgctl, 4);
  core_emuir_set (core, INSN_SSYNC, RUNTEST);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
  return 0;
}

static int
ddr_init (void)
{
  uint32_t p0, r0;
  uint32_t value;
  int core;
  int i;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG2,
	    "%s: ddr_init ()", bfin_target.name);

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (!cpu->cores[i].is_locked
	&& !cpu->cores[i].is_corefault
	&& !cpu->cores[i].is_running)
      break;

  if (i == cpu->chain->parts->len)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: no core available to init ddr",
		bfin_target.name);
      return -1;
    }
  else
    core = i;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, EBIU_DDRCTL0);

  value = mmr_read_clobber_p0r0 (core, EBIU_RSTCTL - EBIU_DDRCTL0, 2);
  mmr_write_clobber_p0r0 (core, EBIU_RSTCTL - EBIU_DDRCTL0, value | 0x1, 2);
  core_emuir_set (core, INSN_SSYNC, RUNTEST);

  mmr_write_clobber_p0r0 (core, 0, cpu->ddr_config->ddrctl0, 4);
  mmr_write_clobber_p0r0 (core, EBIU_DDRCTL1 - EBIU_DDRCTL0,
			  cpu->ddr_config->ddrctl1, 4);
  mmr_write_clobber_p0r0 (core, EBIU_DDRCTL2 - EBIU_DDRCTL0,
			  cpu->ddr_config->ddrctl2, 4);
  core_emuir_set (core, INSN_SSYNC, RUNTEST);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
  return 0;
}

static void
core_dcplb_get_clobber_p0r0 (int core)
{
  int i;

  core_register_set (core, REG_P0, DCPLB_ADDR0);
  core_dbgctl_set (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
  core_emuir_set_2 (core,
		    gen_load32pi (REG_R0, REG_P0),
		    gen_move (REG_EMUDAT, REG_R0), UPDATE);
  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    cpu->cores[core].dcplbs[i].addr = core_emudat_get (core, RUNTEST);
  core_dbgctl_clear (core, DBGCTL_EMUIRLPSZ_2, UPDATE);

  core_register_set (core, REG_P0, DCPLB_DATA0);
  core_dbgctl_set (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
  core_emuir_set_2 (core,
		    gen_load32pi (REG_R0, REG_P0),
		    gen_move (REG_EMUDAT, REG_R0), UPDATE);
  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    cpu->cores[core].dcplbs[i].data = core_emudat_get (core, RUNTEST);
  core_dbgctl_clear (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
}

static void
core_dcplb_get (int core)
{
  uint32_t p0, r0;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_dcplb_get_clobber_p0r0 (core);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
}

static void
core_dcplb_set_clobber_p0r0 (int core)
{
  int i;

  core_register_set (core, REG_P0, DCPLB_ADDR0);
  core_dbgctl_set (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
  core_emuir_set_2 (core,
		    gen_move (REG_R0, REG_EMUDAT),
		    gen_store32pi (REG_P0, REG_R0), UPDATE);
  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    core_emudat_set (core, cpu->cores[core].dcplbs[i].addr, RUNTEST);
  core_dbgctl_clear (core, DBGCTL_EMUIRLPSZ_2, UPDATE);

  core_register_set (core, REG_P0, DCPLB_DATA0);
  core_dbgctl_set (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
  core_emuir_set_2 (core,
		    gen_move (REG_R0, REG_EMUDAT),
		    gen_store32pi (REG_P0, REG_R0), UPDATE);
  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    core_emudat_set (core, cpu->cores[core].dcplbs[i].data, RUNTEST);
  core_dbgctl_clear (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
}

static void
core_dcplb_set (int core)
{
  uint32_t p0, r0;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_dcplb_set_clobber_p0r0 (core);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
}

static void
core_icplb_set_clobber_p0r0 (int core)
{
  int i;

  core_register_set (core, REG_P0, ICPLB_ADDR0);
  core_dbgctl_set (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
  core_emuir_set_2 (core,
		    gen_move (REG_R0, REG_EMUDAT),
		    gen_store32pi (REG_P0, REG_R0), UPDATE);
  for (i = 0; i < BFIN_ICPLB_NUM; i++)
    core_emudat_set (core, cpu->cores[core].icplbs[i].addr, RUNTEST);
  core_dbgctl_clear (core, DBGCTL_EMUIRLPSZ_2, UPDATE);

  core_register_set (core, REG_P0, ICPLB_DATA0);
  core_dbgctl_set (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
  core_emuir_set_2 (core,
		    gen_move (REG_R0, REG_EMUDAT),
		    gen_store32pi (REG_P0, REG_R0), UPDATE);
  for (i = 0; i < BFIN_ICPLB_NUM; i++)
    core_emudat_set (core, cpu->cores[core].icplbs[i].data, RUNTEST);
  core_dbgctl_clear (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
}

static void
core_icplb_set (int core)
{
  uint32_t p0, r0;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_icplb_set_clobber_p0r0 (core);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
}

/* Make DCPLB valid for reading or writing SIZE bytes data at address
   ADDR. When called, array CLOBBERED should be initialized with all 0s.
   When return, CLOBBERED is set to non-zero for clobbered entries.
   If the bit 1 of element of CLOBBERED is not zero, the address part
   of the CPLB entry is clobbered.  The bit 2 is for data part.  */

/* The data structures.  */
struct cplb
{
  int entry;
  struct cplb *next;
};

struct mb
{
  uint32_t addr;
  uint32_t end;
  /* How many cplbs cover this memory block.  */
  int cplb_count;
  /* All cplbs that cover this memory block.  */
  struct cplb *cplbs;
  struct mb *next;
};

/* Split the memory block into two. The first one has NEW_SIZE bytes.  */
static void
split_mb (struct mb *p, size_t new_size, int *used)
{
  struct mb *p2;
  struct cplb *c, *c2;

  p2 = (struct mb *) malloc (sizeof (struct mb));
  if (p2 == NULL)
    abort ();

  p2->end = p->end;
  p2->addr = p->end = p->addr + new_size;
  p2->next = p->next;
  p->next = p2;

  p2->cplb_count = p->cplb_count;

  /* Copy all cplbs.  */
  p2->cplbs = NULL;

  for (c = p->cplbs; c != NULL; c = c->next)
    {
      c2 = (struct cplb *) malloc (sizeof (struct cplb));
      if (c2 == NULL)
	abort ();
      c2->entry = c->entry;
      c2->next = p2->cplbs;
      p2->cplbs = c2;
      used[c2->entry]++;
    }
}

static int
get_unused_dcplb (int *used, int *clobbered)
{
  int i;

  /* If there is any clobbered but unused cplb entries,
     use them first.  */
  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    if (!used[i] && clobbered[i])
      return i;

  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    if (!used[i])
      return i;

  return -1;
}

static struct mb *
dcplb_analyze (int core, uint32_t addr, size_t size, int *used)
{
  uint32_t p0, r0;
  struct mb *mbs, *p;
  int i;

  if (!cpu->cores[core].dmem_control_valid_p)
    {
      p0 = core_register_get (core, REG_P0);
      r0 = core_register_get (core, REG_R0);

      core_register_set (core, REG_P0, DMEM_CONTROL);
      cpu->cores[core].dmem_control
	= mmr_read_clobber_p0r0 (core, 0, 4);
      cpu->cores[core].dmem_control_valid_p = 1;

      if ((cpu->cores[core].dmem_control & ENDCPLB)
	  && !cpu->cores[core].dcplbs_valid_p)
	{
	  core_dcplb_get_clobber_p0r0 (core);
	  cpu->cores[core].dcplbs_valid_p = 1;
	}

      core_register_set (core, REG_P0, p0);
      core_register_set (core, REG_R0, r0);
    }
  else if ((cpu->cores[core].dmem_control & ENDCPLB)
	   && !cpu->cores[core].dcplbs_valid_p)
    {
      core_dcplb_get (core);
      cpu->cores[core].dcplbs_valid_p = 1;
    }

  if (!(cpu->cores[core].dmem_control & ENDCPLB))
    return NULL;

  mbs = (struct mb*) malloc (sizeof (struct mb));
  if (!mbs)
    abort ();

  mbs->addr = addr;
  mbs->end = addr + size;
  mbs->cplbs = NULL;
  mbs->cplb_count = 0;
  mbs->next = NULL;

  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    {
      uint32_t cplb_addr;
      uint32_t cplb_data;
      size_t page_size;
      struct cplb *c;

      cplb_data = cpu->cores[core].dcplbs[i].data;
      if (!(cplb_data & CPLB_VALID))
	continue;
      switch (cplb_data & PAGE_SIZE_MASK)
	{
	case PAGE_SIZE_4MB:
	  page_size = 4 * 1024 * 1024;
	  break;
	case PAGE_SIZE_1MB:
	  page_size = 1024 * 1024;
	  break;
	case PAGE_SIZE_4KB:
	  page_size = 4 * 1024;
	  break;
	case PAGE_SIZE_1KB:
	  page_size = 1024;
	  break;
	default:
	  abort ();
	}
      cplb_addr = cpu->cores[core].dcplbs[i].addr;
      cplb_addr &= ~(page_size - 1);

      /* Check if this dcplb entry cover any memory blocks.  */
      for (p = mbs; p != NULL; p = p->next)
	{
	  /* If the dcplb entry does not cover any part of the memory
	     block, do nothing.  */
	  if (p->addr >= cplb_addr + page_size
	      || p->end <= cplb_addr)
	    continue;

	  /* If the dcplb entry completely covers the memory block,
	     assign the dcplb entry to it.  */
	  else if (p->addr >= cplb_addr && p->end <= cplb_addr + page_size)
	    {
	      c = (struct cplb *) malloc (sizeof (struct cplb));
	      if (!c)
		abort ();
	      c->entry = i;
	      c->next = p->cplbs;
	      p->cplbs = c;
	      p->cplb_count++;
	      used[i]++;
	    }

	  /* If the dcplb entry only covers part of the memory block,
	     we have to split the memory block.  First the most
	     complicated one: cover the middle part.  */
	  else if (p->addr < cplb_addr && p->end > cplb_addr + page_size)
	    {
	      split_mb (p, cplb_addr - p->addr, used);
	      p = p->next;
	      split_mb (p, page_size, used);

	      c = (struct cplb *) malloc (sizeof (struct cplb));
	      if (!c)
		abort ();
	      c->entry = i;
	      c->next = p->cplbs;
	      p->cplbs = c;
	      p->cplb_count++;
	      used[i]++;

	      p = p->next;
	    }

	  /* Second one: cover the part near head.  */
	  else if (p->addr >= cplb_addr && p->addr < cplb_addr + page_size
		   && p->end > cplb_addr + page_size)
	    {
	      split_mb (p, cplb_addr - p->addr, used);

	      c = (struct cplb *) malloc (sizeof (struct cplb));
	      if (!c)
		abort ();
	      c->entry = i;
	      c->next = p->cplbs;
	      p->cplbs = c;
	      p->cplb_count++;
	      used[i]++;

	      p = p->next;
	    }

	  /* Third one: cover the part near end.  */
	  else if (p->addr < cplb_addr && p->end > cplb_addr
		   && p->end <= cplb_addr + page_size)
	    {
	      split_mb (p, cplb_addr - p->addr, used);
	      p = p->next;

	      c = (struct cplb *) malloc (sizeof (struct cplb));
	      if (!c)
		abort ();
	      c->entry = i;
	      c->next = p->cplbs;
	      p->cplbs = c;
	      p->cplb_count++;
	      used[i]++;
	    }

	  /* Should not reach here.  */
	  else
	    abort ();
	}
    }

  return mbs;
}

/* Free all allocated memory.  */
static void
mbs_free (struct mb *mbs)
{
  struct mb *m = mbs;

  while (m != NULL)
    {
      struct mb *p = m->next;
      struct cplb *c = m->cplbs;

      while (c != NULL)
	{
	  struct cplb *q = c->next;

	  free (c);
	  c = q;
	}

      free (m);
      m = p;
    }
}

static void
dcplb_validate_clobber_p0r0 (int core,
			     uint32_t addr,
			     size_t size,
			     int *clobbered)
{
  struct mb *mbs, *p;
  int used[BFIN_DCPLB_NUM], used2[BFIN_DCPLB_NUM];
  bfin_cplb_entry dcplbs_new[BFIN_DCPLB_NUM];
  int need_reorg;
  int i;
  int changed;

  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    used[i] = 0;

  mbs = dcplb_analyze (core, addr, size, used);

  /* If DCPLB is disabled, just return.  */
  if (mbs == NULL)
    return;

  /* Keep a copy of the used information, such that we can save one pass
     of analysis.  */
  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    used2[i] = used[i];

  /* Now we completed the analysis of the coverage.  Now we can try to
     do the validation.  */
  need_reorg = 0;

  for (p = mbs; p != NULL; p = p->next)
    {
      /* If there is exactly one cplb cover the memory block.  Nothing
	 we are required to do.  */
      if (p->cplb_count == 1)
	continue;

      /* If the memory block is not covered by any cplb. Try to cover
	 it.  */
      else if (p->cplb_count == 0)
	{
	  uint32_t cplb_addr;
	  uint32_t cplb_end;
	  size_t page_size;

	  cplb_addr = p->addr;
	  cplb_end = p->end;

	  /* If this is the only memory block.  */
	  if (p == mbs && p->next == NULL)
	    {
	      cplb_addr &= ~(4 * 1024 * 1024 - 1);
	      cplb_end += 4 * 1024 * 1024 - 1;
	      cplb_end &= ~(4 * 1024 * 1024 - 1);
	      page_size = 4 * 1024 * 1024;
	    }

	  /* If this is the first memory block, the largest cplb which
	     aligns with the end of the memory block is used.  */
	  else if (p == mbs)
	    {
	      if ((cplb_end & (4 * 1024 * 1024 - 1)) == 0)
		page_size = 4 * 1024 * 1024;
	      else if ((cplb_end & (1024 * 1024 - 1)) == 0)
		page_size = 1024 * 1024;
	      else if ((cplb_end & (4 * 1024 - 1)) == 0)
		page_size = 4 * 1024;
	      else if ((cplb_end & (1024 - 1)) == 0)
		page_size = 1024;
	      else
		abort ();

	      cplb_addr &= ~(page_size - 1);
	    }

	  /* If this is the last memory block, the largest cplb which
	     aligns with the start of the memory block is used.  */
	  else if (p->next == NULL)
	    {
	      if ((cplb_addr & (4 * 1024 * 1024 - 1)) == 0)
		page_size = 4 * 1024 * 1024;
	      else if ((cplb_addr & (1024 * 1024 - 1)) == 0)
		page_size = 1024 * 1024;
	      else if ((cplb_addr & (4 * 1024 - 1)) == 0)
		page_size = 4 * 1024;
	      else if ((cplb_addr & (1024 - 1)) == 0)
		page_size = 1024;
	      else
		abort ();

	      cplb_end += page_size - 1;
	      cplb_end &= ~(page_size - 1);
	    }

	  /* This is a memory block between two others, we have to use
	     the least cplb with aligns with the start and the end of
	     the memory block.  */
	  else
	    {
	      if ((cplb_addr & (4 * 1024 * 1024 - 1)) == 0
		  && (cplb_end & (4 * 1024 * 1024 - 1)) == 0)
		page_size = 4 * 1024 * 1024;
	      else if ((cplb_addr & (1024 * 1024 - 1)) == 0
		       && (cplb_end & (1024 * 1024 - 1)) == 0)
		page_size = 1024 * 1024;
	      else if ((cplb_addr & (4 * 1024 - 1)) == 0
		       && (cplb_end & (4 * 1024 - 1)) == 0)
		page_size = 4 * 1024;
	      else if ((cplb_addr & (1024 - 1)) == 0
		       && (cplb_end & (1024 - 1)) == 0)
		page_size = 1024;
	      else
		abort ();
	    }

	  /* Find unused cplbs to cover this memory block.  */
	  while (cplb_addr < cplb_end)
	    {
	      struct cplb *c;
	      int new_cplb;

	      new_cplb = get_unused_dcplb (used, clobbered);
	      if (new_cplb == -1)
		break;

	      clobbered[new_cplb] = 1;
	      dcplbs_new[new_cplb].addr = cplb_addr;
	      if (page_size == 4 * 1024 * 1024)
		dcplbs_new[new_cplb].data = DNON_CHBL_4MB;
	      else if (page_size == 1024 * 1024)
		dcplbs_new[new_cplb].data = DNON_CHBL_1MB;
	      else if (page_size == 4 * 1024)
		dcplbs_new[new_cplb].data = DNON_CHBL_4KB;
	      else if (page_size == 1024)
		dcplbs_new[new_cplb].data = DNON_CHBL_1KB;
	      else
		abort ();

	      split_mb (p, page_size, used);

	      c = (struct cplb *) malloc (sizeof (struct cplb));
	      if (!c)
		abort ();
	      c->entry = new_cplb;
	      c->next = p->cplbs;
	      p->cplbs = c;
	      p->cplb_count++;
	      used[new_cplb]++;

	      p = p->next;
	      cplb_addr += page_size;
	    }

	  /* If there are not enough unused dcplbs to cover it,
	     we have to reorg it.  */
	  if (cplb_addr < cplb_end)
	    {
	      need_reorg = 1;
	      break;
	    }
	}

      /* If there are more than one dcplbs covering the memory block.
	 we will see if we can remove the redundant ones.  */
      else if (p->cplb_count > 1)
	{
	  struct cplb **q = &(p->cplbs);

	  while (p->cplb_count > 1)
	    {
	      struct cplb *c = *q;

	      if (c == NULL)
		break;

	      /* Only remove the cplb which doesn't cover other memory
		 blocks to make our life easy.  */
	      if (used[c->entry] == 1)
		{
		  /* Disable the cplb.  */
		  clobbered[c->entry] = 1;
		  dcplbs_new[c->entry].addr
		    = cpu->cores[core].dcplbs[c->entry].addr;
		  dcplbs_new[c->entry].data
		    = cpu->cores[core].dcplbs[c->entry].addr & ~CPLB_VALID;

		  /* Unlink the cplb.  */
		  *q = c->next;
		  q = &(c->next);
		  used[c->entry]--;
		  free (c);

		  p->cplb_count--;
		}
	      else
		q = &(c->next);
	    }

	  if (p->cplb_count > 1)
	    {
	      need_reorg = 1;
	      break;
	    }
	}

      /* Cannot reach here.  */
      else
	abort ();
    }

  if (need_reorg)
    {
      uint32_t cplb_addr;
      uint32_t cplb_end;
      size_t page_size;

      for (i = 0; i < BFIN_DCPLB_NUM; i++)
	if (used2[i])
	  {
	    clobbered[i] = 1;
	    dcplbs_new[i].addr
	      = cpu->cores[core].dcplbs[i].addr;
	    dcplbs_new[i].data
	      = cpu->cores[core].dcplbs[i].addr & ~CPLB_VALID;
	    used2[i] = 0;
	  }
	else
	  clobbered[i] = 0;

      /* Reconstruct the cplbs for the memory region.
	 Use 4MB page size.  */
      cplb_addr = addr & ~(4 * 1024 * 1024 - 1);
      cplb_end = addr + size + 4 * 1024 * 1024 - 1;
      cplb_end &= ~(4 * 1024 * 1024 - 1);
      page_size = 4 * 1024 * 1024;

      while (cplb_addr < cplb_end)
	{
	  int new_cplb;

	  new_cplb = get_unused_dcplb (used2, clobbered);

	  /* This should never happen.  */
	  if (new_cplb == -1)
	    abort ();

	  clobbered[new_cplb] = 1;
	  dcplbs_new[new_cplb].addr = cplb_addr;
	  dcplbs_new[new_cplb].data = DNON_CHBL_4MB;
	  used2[new_cplb] = 1;
	  cplb_addr += page_size;
	}
    }

  mbs_free (mbs);

  /* Now it's the time to do the real changes.  */

  changed = 0;
  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    if (clobbered[i])
      {
	changed = 1;
	break;
      }

  /* If there is no cplb entries to be clobbered,
     Don't bother to set REG_P0.  */

  if (!changed)
    return;

  core_register_set (core, REG_P0, DCPLB_ADDR0);

  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    if (clobbered[i])
      {
	if (dcplbs_new[i].addr != cpu->cores[core].dcplbs[i].addr)
	  {
	    core_register_set (core, REG_R0, dcplbs_new[i].addr);
	    core_emuir_set (core,
			    gen_store32_offset (REG_P0, i * 4, REG_R0),
			    RUNTEST);
	    clobbered[i] |= 0x2 ;
	  }
	if (dcplbs_new[i].data != cpu->cores[core].dcplbs[i].data)
	  {
	    core_register_set (core, REG_R0, dcplbs_new[i].data);
	    core_emuir_set (core,
			    gen_store32_offset (REG_P0,
						DCPLB_DATA0
						- DCPLB_ADDR0 + i * 4,
						REG_R0),
			    RUNTEST);
	    clobbered[i] |= 0x4 ;
	  }
      }
}

static void
dcplb_restore_clobber_p0r0 (int core, int *clobbered)
{
  int changed, i;

  changed = 0;
  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    if (clobbered[i])
      {
	changed = 1;
	break;
      }

  /* If there is no cplb entries to be clobbered,
     Don't bother to set REG_P0.  */

  if (!changed)
    return;

  core_register_set (core, REG_P0, DCPLB_ADDR0);

  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    {
      if (clobbered[i] & 0x2)
	{
	  core_register_set (core, REG_R0, cpu->cores[core].dcplbs[i].addr);
	  core_emuir_set (core,
			  gen_store32_offset (REG_P0, i * 4, REG_R0),
			  RUNTEST);
	}
      if (clobbered[i] & 0x4)
	{
	  core_register_set (core, REG_R0, cpu->cores[core].dcplbs[i].data);
	  core_emuir_set (core,
			  gen_store32_offset (REG_P0,
					      DCPLB_DATA0 - DCPLB_ADDR0
					      + i * 4,
					      REG_R0),
			  RUNTEST);
	}
    }
}

static void
core_dcache_enable (int core, int method)
{
  uint32_t p0, r0;
  int i, j;

  i = 0;
  if (cpu->cores[core].l1_map.l1_data_a)
    {
      cpu->cores[core].dcplbs[i].addr = cpu->cores[core].l1_map.l1_data_a;
      cpu->cores[core].dcplbs[i].data = L1_DMEMORY;
      i++;
    }
  if (cpu->cores[core].l1_map.l1_data_b)
    {
      cpu->cores[core].dcplbs[i].addr = cpu->cores[core].l1_map.l1_data_b;
      cpu->cores[core].dcplbs[i].data = L1_DMEMORY;
      i++;
    }

  cpu->cores[core].dcplbs[i].addr = cpu->mem_map.flash;
  cpu->cores[core].dcplbs[i].data = SDRAM_DNON_CHBL;
  i++;
  
  j = i;
  for (; i < 16; i++)
    {
      if ((i - j) * 4 * 1024 * 1024 >= cpu->mem_map.sdram_end)
	break;
      cpu->cores[core].dcplbs[i].addr
	= cpu->mem_map.sdram + (i - j) * 4 * 1024 * 1024;
      if (method == WRITE_THROUGH)
	cpu->cores[core].dcplbs[i].data = SDRAM_DGEN_WT;
      else if (method == WRITE_BACK)
	cpu->cores[core].dcplbs[i].data = SDRAM_DGEN_WB;
      else
	abort ();
    }

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, DTEST_COMMAND);
  mmr_write_clobber_p0r0 (core, 0, 0, 4);
  core_emuir_set (core, INSN_CSYNC, RUNTEST);

  core_dcplb_set_clobber_p0r0 (core);
  cpu->cores[core].dcplbs_valid_p = 1;

  cpu->cores[core].dmem_control = ACACHE_BCACHE | ENDCPLB;
  mmr_write_clobber_p0r0 (core, DMEM_CONTROL - DCPLB_ADDR0,
			  cpu->cores[core].dmem_control, 4);
  cpu->cores[core].dmem_control_valid_p = 1;

  core_emuir_set (core, INSN_SSYNC, RUNTEST);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
}

static void
dcache_enable (int method)
{
  int i;

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      if (cpu->cores[i].is_locked
	  || cpu->cores[i].is_corefault
	  || cpu->cores[i].is_running)
	{
	  cpu->cores[i].l1_data_a_cache_enabled = 0;
	  cpu->cores[i].l1_data_b_cache_enabled = 0;
	}
      else
	{
	  core_dcache_enable (i, method);
	  cpu->cores[i].l1_data_a_cache_enabled = 1;
	  cpu->cores[i].l1_data_b_cache_enabled = 1;
	}
    }
}

static void
core_icache_enable (int core)
{
  uint32_t p0, r0;
  int i, j;

  i = 0;
  if (cpu->cores[core].l1_map.l1_code)
    {
      cpu->cores[core].icplbs[i].addr = cpu->cores[core].l1_map.l1_code;
      cpu->cores[core].icplbs[i].data = L1_IMEMORY;
      i++;
    }

  cpu->cores[core].icplbs[i].addr = cpu->mem_map.flash;
  cpu->cores[core].icplbs[i].data = SDRAM_INON_CHBL;
  i++;

  j = i;
  for (; i < 16; i++)
    {
      if ((i - j) * 4 * 1024 * 1024 >= cpu->mem_map.sdram_end)
	break;
      cpu->cores[core].icplbs[i].addr
	= cpu->mem_map.sdram + (i - j) * 4 * 1024 * 1024;
      cpu->cores[core].icplbs[i].data = SDRAM_IGENERIC;
    }

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, ITEST_COMMAND);
  mmr_write_clobber_p0r0 (core, 0, 0, 4);
  core_emuir_set (core, INSN_CSYNC, RUNTEST);

  core_icplb_set_clobber_p0r0 (core);
  cpu->cores[core].icplbs_valid_p = 1;

  cpu->cores[core].imem_control = IMC | ENICPLB;
  mmr_write_clobber_p0r0 (core, IMEM_CONTROL - ICPLB_ADDR0,
			  cpu->cores[core].imem_control, 4);
  cpu->cores[core].imem_control_valid_p = 1;

  core_emuir_set (core, INSN_SSYNC, RUNTEST);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
}

static void
icache_enable (void)
{
  int i;

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      if (cpu->cores[i].is_locked
	  || cpu->cores[i].is_corefault
	  || cpu->cores[i].is_running)
	cpu->cores[i].l1_code_cache_enabled = 0;
      else
	{
	  core_icache_enable (i);
	  cpu->cores[i].l1_code_cache_enabled = 1;
	}
    }
}

static void
icache_flush (int core, uint32_t addr, size_t size)
{
  uint32_t p0;
  int i;

  assert (size > 0);

  p0 = core_register_get (core, REG_P0);

  core_register_set (core, REG_P0, addr);
   for (i = (size + addr % CACHE_LINE_BYTES - 1) / CACHE_LINE_BYTES + 1;
	i > 0; i--)
     core_emuir_set (core, gen_iflush_pm (REG_P0), RUNTEST);

  core_register_set (core, REG_P0, p0);
}

static void
cache_flush (int core, uint32_t addr, size_t size)
{
  uint32_t p0;
  int i;

  assert (size > 0);

  p0 = core_register_get (core, REG_P0);

  core_register_set (core, REG_P0, addr);
  core_dbgctl_set (core, DBGCTL_EMUIRLPSZ_2, UPDATE);
  for (i = (size + addr % CACHE_LINE_BYTES - 1) / CACHE_LINE_BYTES + 1;
       i > 0; i--)
    core_emuir_set_2 (core, gen_flush (REG_P0),
		      gen_iflush_pm (REG_P0), RUNTEST);
  core_dbgctl_clear (core, DBGCTL_EMUIRLPSZ_2, UPDATE);

  core_register_set (core, REG_P0, p0);
}

static int
memory_read (int core, uint32_t addr, uint8_t *buf, size_t size)
{
  uint32_t p0, r0;
  int clobbered[BFIN_DCPLB_NUM];
  int i;

  assert (size > 0);

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    clobbered[i] = 0;

  dcplb_validate_clobber_p0r0 (core, addr, size, clobbered);

  core_register_set (core, REG_P0, addr);

  core_dbgctl_set (core, DBGCTL_EMUIRLPSZ_2, UPDATE);

  if ((addr & 0x3) != 0)
    core_emuir_set_2 (core,
		      gen_load8zpi (REG_R0, REG_P0),
		      gen_move (REG_EMUDAT, REG_R0), UPDATE);

  while ((addr & 0x3) != 0 && size != 0)
    {
      *buf++ = core_emudat_get (core, RUNTEST);
      addr++;
      size--;
    }
  if (size == 0)
    goto finish_read;

  if (size >= 4)
    core_emuir_set_2 (core,
		      gen_load32pi (REG_R0, REG_P0),
		      gen_move (REG_EMUDAT, REG_R0), UPDATE);

  for (; size >= 4; size -= 4)
    {
      uint32_t data;

      data = core_emudat_get (core, RUNTEST);
      *buf++ = data & 0xff;
      *buf++ = (data >> 8) & 0xff;
      *buf++ = (data >> 16) & 0xff;
      *buf++ = (data >> 24) & 0xff;
    }

  if (size == 0)
    goto finish_read;

  core_emuir_set_2 (core,
		    gen_load8zpi (REG_R0, REG_P0),
		    gen_move (REG_EMUDAT, REG_R0), UPDATE);

  for (; size > 0; size--)
    *buf++ = core_emudat_get (core, RUNTEST);

finish_read:

  core_dbgctl_clear (core, DBGCTL_EMUIRLPSZ_2, UPDATE);

  dcplb_restore_clobber_p0r0 (core, clobbered);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);

  return 0;
}


static void
dma_context_save_clobber_p0r0 (int core, uint32_t base, bfin_dma *dma)
{
  core_register_set (core, REG_P0, base);

  dma->start_addr = mmr_read_clobber_p0r0 (core, 0x04, 4);
  dma->config = mmr_read_clobber_p0r0 (core, 0x08, 2);
  dma->x_count = mmr_read_clobber_p0r0 (core, 0x10, 2);
  dma->x_modify = mmr_read_clobber_p0r0 (core, 0x14, 2);
  dma->irq_status = mmr_read_clobber_p0r0 (core, 0x28, 2);
}

static void
dma_context_restore_clobber_p0r0 (int core, uint32_t base, bfin_dma *dma)
{
  core_register_set (core, REG_P0, base);

  mmr_write_clobber_p0r0 (core, 0x04, dma->start_addr, 4);
  mmr_write_clobber_p0r0 (core, 0x10, dma->x_count, 2);
  mmr_write_clobber_p0r0 (core, 0x14, dma->x_modify, 2);
  mmr_write_clobber_p0r0 (core, 0x08, dma->config, 2);
}

static void
bfin_log_dma (uint32_t base, bfin_dma *dma)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG2,
	    "%s: DMA base [0x%08X]", bfin_target.name, base);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG2,
	    "%s: START_ADDR [0x%08X]", bfin_target.name, dma->start_addr);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG2,
	    "%s: CONFIG [0x%04X]", bfin_target.name, dma->config);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG2,
	    "%s: X_COUNT [0x%04X]", bfin_target.name, dma->x_count);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG2,
	    "%s: X_MODIFY [0x%04X]", bfin_target.name, dma->x_modify);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG2,
	    "%s: IRQ_STATUS [0x%04X]", bfin_target.name, dma->irq_status);
}

static int
dma_copy (int core, uint32_t dest, uint32_t src, size_t size)
{
  bfin_dma mdma_s0_save, mdma_d0_save;
  bfin_dma mdma_s0, mdma_d0;
  uint32_t p0, r0;
  uint16_t s0_irq_status, d0_irq_status;
  int ret;
  struct timespec dma_wait = {0, 50000000};

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  dma_context_save_clobber_p0r0 (core, cpu->mdma_s0, &mdma_s0_save);
  dma_context_save_clobber_p0r0 (core, cpu->mdma_d0, &mdma_d0_save);

  bfin_log_dma (cpu->mdma_s0, &mdma_s0_save);
  bfin_log_dma (cpu->mdma_d0, &mdma_d0_save);

  s0_irq_status = mdma_s0_save.irq_status;
  d0_irq_status = mdma_d0_save.irq_status;

  while ((s0_irq_status & DMA_IRQ_STATUS_DMA_RUN)
	 || (mdma_d0_save.irq_status & DMA_IRQ_STATUS_DMA_RUN))
    {
      bfin_log (RP_VAL_LOGLEVEL_NOTICE,
		"%s: wait DMA done: S0 [0x%04X] D0 [0x%04X]",
		bfin_target.name, s0_irq_status, d0_irq_status);

      nanosleep (&dma_wait, NULL);

      core_register_set (core, REG_P0, cpu->mdma_s0);
      s0_irq_status = mmr_read_clobber_p0r0 (core, 0x28, 2);
      core_register_set (core, REG_P0, cpu->mdma_d0);
      d0_irq_status = mmr_read_clobber_p0r0 (core, 0x28, 2);
    }

  if ((s0_irq_status & DMA_IRQ_STATUS_DMA_ERR))
    bfin_log (RP_VAL_LOGLEVEL_NOTICE,
	      "%s: clear MDMA0_S0 DMA ERR: IRQ_STATUS [0x%04X]",
	      bfin_target.name, s0_irq_status);
  if ((s0_irq_status & DMA_IRQ_STATUS_DMA_DONE))
    bfin_log (RP_VAL_LOGLEVEL_NOTICE,
	      "%s: clear MDMA_S0 DMA DONE: IRQ_STATUS [0x%04X]",
	      bfin_target.name, s0_irq_status);
  if ((d0_irq_status & DMA_IRQ_STATUS_DMA_ERR))
    bfin_log (RP_VAL_LOGLEVEL_NOTICE,
	      "%s: clear MDMA0_D0 DMA ERR: IRQ_STATUS [0x%04X]",
	      bfin_target.name, d0_irq_status);
  if ((d0_irq_status & DMA_IRQ_STATUS_DMA_DONE))
    bfin_log (RP_VAL_LOGLEVEL_NOTICE,
	      "%s: clear MDMA_D0 DMA DONE: IRQ_STATUS [0x%04X]",
	      bfin_target.name, d0_irq_status);

  if ((s0_irq_status & DMA_IRQ_STATUS_DMA_ERR)
      || (s0_irq_status & DMA_IRQ_STATUS_DMA_DONE))
    {
      core_register_set (core, REG_P0, cpu->mdma_s0);
      mmr_write_clobber_p0r0 (core,
			      0x28,
			      DMA_IRQ_STATUS_DMA_ERR | DMA_IRQ_STATUS_DMA_DONE,
			      2);
    }

  if ((d0_irq_status & DMA_IRQ_STATUS_DMA_ERR)
      || (d0_irq_status & DMA_IRQ_STATUS_DMA_DONE))
    {
      core_register_set (core, REG_P0, cpu->mdma_d0);
      mmr_write_clobber_p0r0 (core,
			      0x28,
			      DMA_IRQ_STATUS_DMA_ERR | DMA_IRQ_STATUS_DMA_DONE,
			      2);
    }

  core_register_set (core, REG_P0, cpu->mdma_s0);
  s0_irq_status = mmr_read_clobber_p0r0 (core, 0x28, 2);
  core_register_set (core, REG_P0, cpu->mdma_d0);
  d0_irq_status = mmr_read_clobber_p0r0 (core, 0x28, 2);

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: before dma copy MDMA0_S0 IRQ_STATUS [0x%04X]",
	    bfin_target.name, s0_irq_status);

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: before dma copy MDMA0_D0 IRQ_STATUS [0x%04X]",
	    bfin_target.name, d0_irq_status);

  mdma_s0.start_addr = src;
  mdma_s0.x_count = size;
  mdma_s0.x_modify = 1;
  mdma_s0.config = DMA_CONFIG_FLOW_STOP | DMA_CONFIG_NDSIZE_0;
  mdma_s0.config |= DMA_CONFIG_WDSIZE_8 | DMA_CONFIG_DMAEN | DMA_CONFIG_DI_EN;

  mdma_d0.start_addr = dest;
  mdma_d0.x_count = size;
  mdma_d0.x_modify = 1;
  mdma_d0.config =
    DMA_CONFIG_FLOW_STOP | DMA_CONFIG_NDSIZE_0 | DMA_CONFIG_WNR;
  mdma_d0.config |= DMA_CONFIG_WDSIZE_8 | DMA_CONFIG_DMAEN | DMA_CONFIG_DI_EN;

  dma_context_restore_clobber_p0r0 (core, cpu->mdma_s0, &mdma_s0);
  dma_context_restore_clobber_p0r0 (core, cpu->mdma_d0, &mdma_d0);

wait_dma:

  core_register_set (core, REG_P0, cpu->mdma_s0);
  s0_irq_status = mmr_read_clobber_p0r0 (core, 0x28, 2);
  core_register_set (core, REG_P0, cpu->mdma_d0);
  d0_irq_status = mmr_read_clobber_p0r0 (core, 0x28, 2);

  if ((s0_irq_status & DMA_IRQ_STATUS_DMA_ERR))
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: MDMA0_S0 DMA error: IRQ_STATUS [0x%04X]",
		bfin_target.name, s0_irq_status);
      ret = -1;
      goto finish_dma_copy;
    }
  if ((d0_irq_status & DMA_IRQ_STATUS_DMA_ERR))
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: MDMA0_D0 DMA error: IRQ_STATUS [0x%04X]",
		bfin_target.name, d0_irq_status);
      ret = -1;
      goto finish_dma_copy;
    }
  else if (!(s0_irq_status & DMA_IRQ_STATUS_DMA_DONE))
    {
      bfin_log (RP_VAL_LOGLEVEL_NOTICE,
		"%s: MDMA_S0 wait for done: IRQ_STATUS [0x%04X]",
		bfin_target.name, s0_irq_status);
      nanosleep (&dma_wait, NULL);
      goto wait_dma;
    }
  else if (!(d0_irq_status & DMA_IRQ_STATUS_DMA_DONE))
    {
      bfin_log (RP_VAL_LOGLEVEL_NOTICE,
		"%s: MDMA_D0 wait for done: IRQ_STATUS [0x%04X]",
		bfin_target.name, d0_irq_status);
      nanosleep (&dma_wait, NULL);
      goto wait_dma;
    }
  else
    ret = 0;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: done dma copy MDMA_S0 IRQ_STATUS [0x%04X]",
	    bfin_target.name, s0_irq_status);

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: done dma copy MDMA_D0 IRQ_STATUS [0x%04X]",
	    bfin_target.name, d0_irq_status);

  if ((s0_irq_status & DMA_IRQ_STATUS_DMA_ERR)
      || (s0_irq_status & DMA_IRQ_STATUS_DMA_DONE))
    {
      core_register_set (core, REG_P0, cpu->mdma_s0);
      mmr_write_clobber_p0r0 (core,
			      0x28,
			      DMA_IRQ_STATUS_DMA_ERR | DMA_IRQ_STATUS_DMA_DONE,
			      2);
    }

  if ((d0_irq_status & DMA_IRQ_STATUS_DMA_ERR)
      || (d0_irq_status & DMA_IRQ_STATUS_DMA_DONE))
    {
      core_register_set (core, REG_P0, cpu->mdma_d0);
      mmr_write_clobber_p0r0 (core,
			      0x28,
			      DMA_IRQ_STATUS_DMA_ERR | DMA_IRQ_STATUS_DMA_DONE,
			      2);
    }


  dma_context_restore_clobber_p0r0 (core, cpu->mdma_s0, &mdma_s0_save);
  dma_context_restore_clobber_p0r0 (core, cpu->mdma_d0, &mdma_d0_save);

finish_dma_copy:

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);

  return ret;
}


static int
memory_write (int core, uint32_t addr, uint8_t *buf, size_t size)
{
  uint32_t p0, r0;
  int clobbered[BFIN_DCPLB_NUM];
  int i;

  assert (size > 0);

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    clobbered[i] = 0;

  dcplb_validate_clobber_p0r0 (core, addr, size, clobbered);

  core_register_set (core, REG_P0, addr);

  core_dbgctl_set (core, DBGCTL_EMUIRLPSZ_2, UPDATE);

  if ((addr & 0x3) != 0)
    core_emuir_set_2 (core,
		      gen_move (REG_R0, REG_EMUDAT),
		      gen_store8pi (REG_P0, REG_R0), UPDATE);

  while ((addr & 0x3) != 0 && size != 0)
    {
      core_emudat_set (core, *buf++, RUNTEST);
      addr++;
      size--;
    }
  if (size == 0)
    goto finish_write;

  if (size >= 4)
    core_emuir_set_2 (core,
		      gen_move (REG_R0, REG_EMUDAT),
		      gen_store32pi (REG_P0, REG_R0), UPDATE);

  for (; size >= 4; size -= 4)
    {
      uint32_t data;

      data = *buf++;
      data |= (*buf++) << 8;
      data |= (*buf++) << 16;
      data |= (*buf++) << 24;
      core_emudat_set (core, data, RUNTEST);
    }

  if (size == 0)
    goto finish_write;

  core_emuir_set_2 (core,
		    gen_move (REG_R0, REG_EMUDAT),
		    gen_store8pi (REG_P0, REG_R0), UPDATE);

  for (; size > 0; size--)
    core_emudat_set (core, *buf++, RUNTEST);

finish_write:

  core_dbgctl_clear (core, DBGCTL_EMUIRLPSZ_2, UPDATE);

  dcplb_restore_clobber_p0r0 (core, clobbered);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);

  return 0;
}



static int
dma_sram_read (int core, uint32_t addr, uint8_t *buf, size_t size)
{
  uint8_t *tmp;
  int ret;

  assert (size > 0);
  assert (size < 0x4000);

  tmp = (uint8_t *) malloc (size);
  if (tmp == 0)
    abort ();

  memory_read (core, cpu->cores[core].l1_map.l1_data_a, tmp, size);

  ret = dma_copy (core, cpu->cores[core].l1_map.l1_data_a, addr, size);

  memory_read (core, cpu->cores[core].l1_map.l1_data_a, buf, size);

  memory_write (core, cpu->cores[core].l1_map.l1_data_a, tmp, size);

  free (tmp);

  return ret;
}

static int
dma_sram_write (int core, uint32_t addr, uint8_t *buf, size_t size)
{
  uint8_t *tmp;
  int ret;

  assert (size > 0);

  assert (cpu->cores[core].l1_map.l1_data_a);

  assert (size <= cpu->cores[core].l1_map.l1_data_a_end
	  - cpu->cores[core].l1_map.l1_data_a);

  tmp = (uint8_t *) malloc (size);
  if (tmp == 0)
    abort ();

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "dma_sram_write (core [%d], addr [0x%08X], size [0x%X])",
	    core, addr, size);

  memory_read (core, cpu->cores[core].l1_map.l1_data_a, tmp, size);

  memory_write (core, cpu->cores[core].l1_map.l1_data_a, buf, size);

  ret = dma_copy (core, addr, cpu->cores[core].l1_map.l1_data_a, size);

  memory_write (core, cpu->cores[core].l1_map.l1_data_a, tmp, size);

  free (tmp);

  return ret;
}


static uint8_t bfin_breakpoint_16[] = { 0x25, 0x0 };
static uint8_t bfin_breakpoint_32[] = { 0x25, 0x0, 0x0, 0x0 };

static bfin_swbp *
add_swbp_at (uint32_t addr)
{
  bfin_swbp *bp;
  size_t actual_size;
  int ret;

  bp = (bfin_swbp *) malloc (sizeof (bfin_swbp));
  if (!bp)
    abort ();

  ret = bfin_read_mem (addr, bp->old_data, 2, &actual_size);
  if (ret != RP_VAL_TARGETRET_OK || actual_size != 2)
    {
      free (bp);
      return NULL;
    }
  if ((bp->old_data[1] & 0xf0) >= 0xc0)
    {
      ret = bfin_read_mem (addr, bp->old_data, 4, &actual_size);
      if (ret != RP_VAL_TARGETRET_OK || actual_size != 4)
	{
	  free (bp);
	  return NULL;
	}
      ret = bfin_write_mem (addr, bfin_breakpoint_32, 4);
      if (ret != RP_VAL_TARGETRET_OK)
	{
	  free (bp);
	  return NULL;
	}
    }
  else
    {
      ret = bfin_write_mem (addr, bfin_breakpoint_16, 2);
      if (ret != RP_VAL_TARGETRET_OK)
	{
	  free (bp);
	  return NULL;
	}
    }

  bp->address = addr;
  bp->core = 0;
  bp->next = cpu->swbps;
  cpu->swbps = bp;

  return bp;
}

static bfin_swbp *
find_swbp_at (uint32_t addr)
{
  bfin_swbp *bp;

  bp = cpu->swbps;
  while (bp != NULL)
    {
      if (bp->address == addr)
	break;;
      bp = bp->next;
    }

  return bp;
}

static int
remove_swbp (bfin_swbp *bp)
{
  bfin_swbp *p;
  int ret;

  if (bp == NULL)
    return -1;

  if (cpu->swbps == bp)
    {
      ret = bfin_write_mem (bp->address,
			    bp->old_data,
			    (bp->old_data[1] & 0xf0) >= 0xc0 ? 4 : 2);
      if (ret != RP_VAL_TARGETRET_OK)
	return -1;

      cpu->swbps = bp->next;
      free (bp);
      return 0;
    }

  p = cpu->swbps;

  while (p->next)
    {
      if (p->next == bp)
	{
	  ret = bfin_write_mem (bp->address,
				bp->old_data,
				(bp->old_data[1] & 0xf0) >= 0xc0 ? 4 : 2);
	  if (ret != RP_VAL_TARGETRET_OK)
	    return -1;

	  p->next = bp->next;
	  free (bp);
	  return 0;
	}

      p = p->next;
    }

  return -1;
}

static void
core_single_step (int core)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_single_step ()", bfin_target.name, core);

  if (!cpu->cores[core].is_stepping)
    core_dbgctl_set (core, DBGCTL_ESSTEP, UPDATE);
  core_emuir_set (core, INSN_RTE, RUNTEST);
  core_wait_emuready (core);
  if (!cpu->cores[core].is_stepping)
    core_dbgctl_clear (core, DBGCTL_ESSTEP, UPDATE);
}

static void
core_cache_status_get (int core)
{
  uint32_t p0, r0;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_cache_status_get ()", bfin_target.name, core);

  /* If the core is locked, caches remain disabled by default.
     If the core has a core fault or is running, we cannot get
     its current cache status. Assume its cache status isn't changed
     since last get.  */
  if (cpu->cores[core].is_locked
      || cpu->cores[core].is_corefault
      || cpu->cores[core].is_running)
    return;

  if (!cpu->cores[core].dmem_control_valid_p
      && !cpu->cores[core].imem_control_valid_p)
    {
      p0 = core_register_get (core, REG_P0);
      r0 = core_register_get (core, REG_R0);

      core_register_set (core, REG_P0, DMEM_CONTROL);

      cpu->cores[core].dmem_control = mmr_read_clobber_p0r0 (core, 0, 4);
      cpu->cores[core].dmem_control_valid_p = 1;
      cpu->cores[core].imem_control = mmr_read_clobber_p0r0 (core, IMEM_CONTROL - DMEM_CONTROL, 4);
      cpu->cores[core].imem_control_valid_p = 1;

      core_register_set (core, REG_P0, p0);
      core_register_set (core, REG_R0, r0);
    }
  else if (!cpu->cores[core].dmem_control_valid_p)
    mmr_read (core, DMEM_CONTROL, 4);
  else if (!cpu->cores[core].imem_control_valid_p)
    mmr_read (core, IMEM_CONTROL, 4);
      
  if (cpu->cores[core].imem_control & IMC)
    cpu->cores[core].l1_code_cache_enabled = 1;
  else
    cpu->cores[core].l1_code_cache_enabled = 0;

  if ((cpu->cores[core].dmem_control & DMC) == ACACHE_BCACHE)
    {
      cpu->cores[core].l1_data_a_cache_enabled = 1;
      cpu->cores[core].l1_data_b_cache_enabled = 1;
    }
  else if ((cpu->cores[core].dmem_control & DMC) == ACACHE_BSRAM)
    {
      cpu->cores[core].l1_data_a_cache_enabled = 1;
      cpu->cores[core].l1_data_b_cache_enabled = 0;
    }
  else
    {
      cpu->cores[core].l1_data_a_cache_enabled = 0;
      cpu->cores[core].l1_data_b_cache_enabled = 0;
    }
}

/* Output registers in the format suitable
   for TAAn:r...;n:r...;  format */
static char *
bfin_out_treg_value (char *in, unsigned int reg_no, uint32_t value)
{
  static const char hex[] = "0123456789abcdef";

  if (in == NULL)
    return NULL;

  assert (reg_no < BFIN_NUM_REGS);
  assert (map_gdb_core[reg_no] != -1);

  *in++ = hex[(reg_no >> 4) & 0x0f];
  *in++ = hex[reg_no & 0x0f];
  *in++ = ':';

  /* The register goes into the buffer in little-endian order */
  *in++ = hex[(value >> 4) & 0x0f];
  *in++ = hex[value & 0x0f];
  *in++ = hex[(value >> 12) & 0x0f];
  *in++ = hex[(value >> 8) & 0x0f];
  *in++ = hex[(value >> 20) & 0x0f];
  *in++ = hex[(value >> 16) & 0x0f];
  *in++ = hex[(value >> 28) & 0x0f];
  *in++ = hex[(value >> 24) & 0x0f];

  *in++ = ';';
  *in = '\0';

  return in;
}

/* Output registers in the format suitable
   for TAAn:r...;n:r...;  format */
static char *
bfin_out_treg (char *in, unsigned int reg_no)
{
  int32_t value;

  assert (reg_no < BFIN_NUM_REGS);
  assert (map_gdb_core[reg_no] != -1);

  value = core_register_get (0, map_gdb_core[reg_no]);

  return bfin_out_treg_value (in, reg_no, value);
}


/* Target method */
static void
bfin_help (const char *prog_name)
{
  printf ("This is the Blackfin target for the GDB proxy server. Usage:\n\n");
  printf ("  %s [options] %s [blackfin-options]\n",
	  prog_name, bfin_target.name);
  printf ("\nOptions:\n\n");
  printf (" --debug                 run proxy in debug mode\n");
  printf (" --help                  `%s --help %s'  prints this message\n",
	  prog_name, bfin_target.name);
  printf (" --port=PORT             use specified port\n");

  printf ("\nBlackfin-options:\n\n");
  printf (" --connect=STRING        JTAG connection string\n");
  printf ("                         (default %s)\n", default_jtag_connect);
  printf (" --board=BOARD           specify the board\n");
  printf (" --enable-dcache=METHOD  enable all data SRAM caches\n");
  printf (" --enable-icache         enable all instruction SRAM caches\n");
  printf (" --flash-size=BYTES      specify the size of flash\n");
  printf (" --force-range-wp        always use range watchpoint\n");
  printf (" --init-sdram            initialize SDRAM or DDR memory\n");
  printf (" --no-auto-switch        Don't automatically switch to the core\n");
  printf ("                         which contains the address set to PC\n");
  printf (" --reset                 do a core and system reset when gdb connects\n");
  printf (" --sdram-size=BYTES      specify the size of SDRAM\n");
  printf (" --unlock-on-connect     unlock core when gdb connects\n");
  printf (" --unlock-on-load        unlock core when loading its L1 code\n");
  printf (" --loop-wait=USEC        wait USEC microseconds in wait loop (default 10000)\n");
  printf (" --emu-wait=USEC         wait USEC microseconds in emulator operations\n");
  printf ("                         (default 5000)\n");
  printf ("\n");

  return;
}


/* Target method */
static int
bfin_open (int argc,
	   char *const argv[], const char *prog_name, log_func log_fn)
{
  chain_t *chain;
  int i;
  bfin_board board;
  int sdram_size;
  int flash_size;
  int usec;

  char *connect_string = default_jtag_connect;
  char *cmd_detect[2] = {"detect", NULL};
  char *cmd_script[3] = {"include", NULL, NULL};
  const char *gdbproxy_datapath = gdbproxy_get_data_dir();

  /* Option descriptors */
  static struct option long_options[] = {
    {"board", required_argument, 0, 1},
    {"sdram-size", required_argument, 0, 2},
    {"flash-size", required_argument, 0, 3},
    {"force-range-wp", no_argument, 0, 4},
    {"unlock-on-connect", no_argument, 0, 5},
    {"unlock-on-load", no_argument, 0, 6},
    {"loop-wait", required_argument, 0, 7},
    {"init-sdram", no_argument, 0, 8},
    {"enable-dcache", required_argument, 0, 9},
    {"enable-icache", no_argument, 0, 10},
    {"reset", no_argument, 0, 11},
    {"emu-wait", required_argument, 0, 12},
    {"no-switch-on-load", no_argument, 0, 13},
    {"connect", required_argument, 0, 14},
    {NULL, 0, 0, 0}
  };

  assert (!cpu);
  assert (prog_name != NULL);
  assert (log_fn != NULL);

  /* Setup datapath */
  cmd_script[1] = malloc(strlen(gdbproxy_datapath) + 6);
  sprintf(cmd_script[1], "%s/bfin", gdbproxy_datapath);

  /* Set log */
  bfin_log = log_fn;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_open ()", bfin_target.name);

  sdram_size = -1;
  flash_size = -1;
  board = UNKNOWN_BOARD;

  /* Process options */
  optind = 1;
  for (;;)
    {
      int c;
      int option_index;

      c = getopt_long (argc, argv, "+", long_options, &option_index);
      if (c == -1)
	break;
      switch (c)
	{
	case 1:
	  if (strcmp (optarg, "bf527-ezkit") == 0)
	    board = BF527_EZKIT;
	  else if (strcmp (optarg, "bf533-stamp") == 0)
	    board = BF533_STAMP;
	  else if (strcmp (optarg, "bf533-ezkit") == 0)
	    board = BF533_EZKIT;
	  else if (strcmp (optarg, "bf537-stamp") == 0)
	    board = BF537_STAMP;
	  else if (strcmp (optarg, "bf537-ezkit") == 0)
	    board = BF537_EZKIT;
	  else if (strcmp (optarg, "bf538f-ezkit") == 0)
	    board = BF538F_EZKIT;
	  else if (strcmp (optarg, "bf548-ezkit") == 0)
	    board = BF548_EZKIT;
	  else if (strcmp (optarg, "bf561-ezkit") == 0)
	    board = BF561_EZKIT;
	  else
	    bfin_log (RP_VAL_LOGLEVEL_ERR,
		      "%s: unknown board  %s", bfin_target.name, optarg);
	  break;

	case 2:
	  sdram_size = atoi (optarg);
	  if (sdram_size < 0)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: bad SDRAM size %d",
			bfin_target.name, sdram_size);
	      exit (1);
	    }
	  break;

	case 3:
	  flash_size = atoi (optarg);
	  if (flash_size < 0)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: bad FLASH size %d",
			bfin_target.name, flash_size);
	      exit (1);
	    }
	  break;

	case 4:
	  bfin_force_range_wp = 1;
	  break;

	case 5:
	  bfin_unlock_on_connect = 1;
	  break;

	case 6:
	  bfin_unlock_on_load = 1;
	  break;

	case 7:
	  usec = atol (optarg);
	  if (usec < 0)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: bad wait time %d",
			bfin_target.name, usec);
	      exit (1);
	    }

	  if (usec > 5000000)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: wait time too large (> 5s) %d",
			bfin_target.name, usec);
	      exit (1);
	    }

	  bfin_loop_wait_ts.tv_nsec = usec * 1000 % 1000000000;
	  bfin_loop_wait_ts.tv_sec = usec * 1000 / 1000000000;
	  bfin_loop_wait_first_ts.tv_nsec = usec * 5000 % 1000000000;
	  bfin_loop_wait_first_ts.tv_sec = usec * 5000 / 1000000000;
	  break;

	case 8:
	  bfin_init_sdram = 1;
	  break;

	case 9:
	  if (strcmp (optarg, "write-through") == 0)
	    bfin_enable_dcache = WRITE_THROUGH;
	  else if (strcmp (optarg, "write-back") == 0)
	    bfin_enable_dcache = WRITE_BACK;
	  else
	    bfin_log (RP_VAL_LOGLEVEL_ERR,
		      "%s: back cache write method  %s",
		      bfin_target.name, optarg);
	  break;

	case 10:
	  bfin_enable_icache = 1;
	  break;

	case 11:
	  bfin_reset = 1;
	  break;

	case 12:
	  usec = atol (optarg);
	  if (usec < 0)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: bad wait time %d",
			bfin_target.name, usec);
	      exit (1);
	    }

	  if (usec > 5000000)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: wait time too large (> 5s) %d",
			bfin_target.name, usec);
	      exit (1);
	    }

	  bfin_emu_wait_ts.tv_nsec = usec * 1000 % 1000000000;
	  bfin_emu_wait_ts.tv_sec = usec * 1000 / 1000000000;
	  break;

	case 13:
	  bfin_auto_switch = 0;
	  break;

	case 14:
	  connect_string = optarg;
	  break;

	default:
	  bfin_log (RP_VAL_LOGLEVEL_NOTICE,
		    "%s: Use `%s --help %s' to see a complete list of options",
		    bfin_target.name, prog_name, bfin_target.name);
	  return RP_VAL_TARGETRET_ERR;
	}
    }

  if (board == UNKNOWN_BOARD && bfin_init_sdram)
    {
      bfin_log (RP_VAL_LOGLEVEL_WARNING,
		"%s: --init-sdram is ignored for unknown board",
		bfin_target.name);
      bfin_init_sdram = 0;
    }

  if (optind != argc)
    {
      /* Bad number of arguments.  */
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: bad number of arguments", bfin_target.name);
      bfin_target.help (prog_name);

      return RP_VAL_TARGETRET_ERR;
    }

  chain = chain_alloc ();

  if (!chain)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: chain allocation failed", bfin_target.name);
      return RP_VAL_TARGETRET_ERR;
    }

  jtag_parse_line (chain, connect_string);

  if (!chain->cable)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: cable initialization failed", bfin_target.name);
      return RP_VAL_TARGETRET_ERR;
    }

  cmd_run (chain, cmd_detect);
  if (!chain->parts || !chain->parts->len)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: detecting parts failed", bfin_target.name);
      return RP_VAL_TARGETRET_ERR;
    }

  /* Add Blackfin emulation instructions and registers.  */
  for (i = 0; i < chain->parts->len; i++)
    {
      chain->active_part = i;
      cmd_run (chain, cmd_script);
    }

  /* TODO initbus */

  /* TODO detectflash */

  cpu = (bfin_cpu *) malloc (sizeof (bfin_cpu)
			     + sizeof (bfin_core) * chain->parts->len);
  if (!cpu)
    {
      parts_free (chain->parts);
      chain->parts = 0;
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: cpu allocation failed", bfin_target.name);
      return RP_VAL_TARGETRET_ERR;
    }

  cpu->chain = chain;
  cpu->board = board;
  cpu->swbps = NULL;

  /* BF531/2/3

     #define MDMA_D0_NEXT_DESC_PTR      0xFFC00E00

     BF52x BF534/6/7 BF54x

     #define MDMA_D0_NEXT_DESC_PTR      0xFFC00F00

     BF538/9

     #define MDMA0_D0_NEXT_DESC_PTR     0xFFC00E00
     #define MDMA1_D0_NEXT_DESC_PTR     0xFFC01F00

     BF561

     #define MDMA1_D0_NEXT_DESC_PTR     0xFFC01F00
     #define MDMA2_D0_NEXT_DESC_PTR     0xFFC00F00
     #define IMDMA_D0_NEXT_DESC_PTR     0xFFC01800
   */

  if (!strcmp (chain->parts->parts[0]->part, "BF526") ||
      !strcmp (chain->parts->parts[0]->part, "BF527"))
    {
      assert (chain->parts->len == 1);

      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf52x_mem_map;
      cpu->cores[0].l1_map = bf52x_l1_map;
    }
  else if (!strcmp (chain->parts->parts[0]->part, "BF533"))
    {
      assert (chain->parts->len == 1);

      cpu->mdma_d0 = 0xffc00e00;
      cpu->mdma_s0 = 0xffc00e40;
      cpu->mem_map = bf533_mem_map;
      cpu->cores[0].l1_map = bf533_l1_map;
    }
  else if (!strcmp (chain->parts->parts[0]->part, "BF537"))
    {
      assert (chain->parts->len == 1);

      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf537_mem_map;
      cpu->cores[0].l1_map = bf537_l1_map;
    }
  else if (!strcmp (chain->parts->parts[0]->part, "BF538"))
    {
      assert (chain->parts->len == 1);

      cpu->mdma_d0 = 0xffc00e00;
      cpu->mdma_s0 = 0xffc00e40;
      cpu->mem_map = bf538_mem_map;
      cpu->cores[0].l1_map = bf538_l1_map;
    }
  else if (!strcmp (chain->parts->parts[0]->part, "BF549"))
    {
      assert (chain->parts->len == 1);

      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf54x_mem_map;
      cpu->cores[0].l1_map = bf54x_l1_map;
    }
  else if (!strcmp (chain->parts->parts[0]->part, "BF561"))
    {
      assert (chain->parts->len == 2);

      cpu->mdma_d0 = 0xffc01800;
      cpu->mdma_s0 = 0xffc01840;
      cpu->mem_map = bf561_mem_map;
      cpu->cores[1].l1_map = bf561_a_l1_map;
      cpu->cores[0].l1_map = bf561_b_l1_map;
    }
  else
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: unsupported processor '%s'",
		bfin_target.name, chain->parts->parts[0]->part);
      parts_free (chain->parts);
      chain->parts = 0;
      free (cpu);
      return RP_VAL_TARGETRET_ERR;
    }

  switch (board)
    {
    case BF527_EZKIT:
      if (strcmp (chain->parts->parts[0]->part, "BF527") != 0)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %s on BF527 EZKIT board",
		    bfin_target.name, chain->parts->parts[0]->part);
	  exit (1);
	}
      cpu->mem_map.sdram_end = 0x4000000;
      cpu->mem_map.flash_end = cpu->mem_map.flash + 0x400000;
      cpu->sdram_config = &bf527_ezkit_sdram_config;
      cpu->ddr_config = NULL;
      break;

    case BF533_EZKIT:
      if (strcmp (chain->parts->parts[0]->part, "BF533") != 0)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %s on BF533 EZKIT board",
		    bfin_target.name, chain->parts->parts[0]->part);
	  exit (1);
	}
      cpu->mem_map.sdram_end = 0x4000000;
      cpu->mem_map.flash_end = cpu->mem_map.flash + 0x200000;
      cpu->sdram_config = &bf533_ezkit_sdram_config;
      cpu->ddr_config = NULL;
      break;

    case BF533_STAMP:
      if (strcmp (chain->parts->parts[0]->part, "BF533") != 0)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %s on BF533 STAMP board",
		    bfin_target.name, chain->parts->parts[0]->part);
	  exit (1);
	}
      cpu->mem_map.sdram_end = 0x8000000;
      cpu->mem_map.flash_end = cpu->mem_map.flash + 0x400000;
      cpu->sdram_config = &bf533_stamp_sdram_config;
      cpu->ddr_config = NULL;
      break;

    case BF537_EZKIT:
    case BF537_STAMP:
      if (strcmp (chain->parts->parts[0]->part, "BF537") != 0)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %s on BF537 %s board",
		    bfin_target.name,
		    chain->parts->parts[0]->part,
		    board == BF537_EZKIT ? "EZKIT" : "STAMP");
	  exit (1);
	}
      cpu->mem_map.sdram_end = 0x4000000;
      cpu->mem_map.flash_end = cpu->mem_map.flash + 0x400000;
      cpu->sdram_config = &bf537_ezkit_sdram_config;
      cpu->ddr_config = NULL;
      break;

    case BF538F_EZKIT:
      if (strcmp (chain->parts->parts[0]->part, "BF538") != 0)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %s on BF538F EZKIT board",
		    bfin_target.name,
		    chain->parts->parts[0]->part);
	  exit (1);
	}
      cpu->mem_map.sdram_end = 0x4000000;
      cpu->mem_map.flash_end = cpu->mem_map.flash + 0x400000;
      cpu->sdram_config = &bf538f_ezkit_sdram_config;
      cpu->ddr_config = NULL;
      break;

    case BF548_EZKIT:
      if (strcmp (chain->parts->parts[0]->part, "BF549") != 0)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %s on BF548 EZKIT board",
		    bfin_target.name, chain->parts->parts[0]->part);
	  exit (1);
	}
      cpu->mem_map.sdram_end = 0x4000000;
      cpu->mem_map.flash_end = cpu->mem_map.flash + 0x1000000;
      cpu->sdram_config = NULL;
      cpu->ddr_config = &bf548_ezkit_ddr_config;
      break;

    case BF561_EZKIT:
      if (strcmp (chain->parts->parts[0]->part, "BF561") != 0)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %s on BF561 EZKIT board",
		    bfin_target.name, chain->parts->parts[0]->part);
	  exit (1);
	}
      cpu->mem_map.sdram_end = 0x4000000;
      cpu->mem_map.flash_end = cpu->mem_map.flash + 0x800000;
      cpu->sdram_config = &bf561_ezkit_sdram_config;
      cpu->ddr_config = NULL;
      break;

    case UNKNOWN_BOARD:
      bfin_log (RP_VAL_LOGLEVEL_WARNING,
		"%s: no board selected, %s is detected",
		bfin_target.name, chain->parts->parts[0]->part);
      cpu->sdram_config = NULL;
      cpu->ddr_config = NULL;
      break;

    default:
      abort ();
    }

  /* Let --sdram-size and --flash-size override the board settting.  */
  if (sdram_size != -1)
    cpu->mem_map.sdram_end = sdram_size;

  if (flash_size != -1)
    cpu->mem_map.flash_end = cpu->mem_map.flash + flash_size;

  if (!strcmp (chain->parts->parts[0]->part, "BF561"))
    {
      cpu->core_a = 1;
      cpu->cores[1].name = "Core A";
      cpu->cores[0].name = "Core B";
    }
  else
    {
      cpu->core_a = 0;
      cpu->cores[0].name = "Core";
    }

  return RP_VAL_TARGETRET_OK;
}


/* Target method */
static void
bfin_close (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_close()", bfin_target.name);

  assert (cpu);

  emulation_disable ();

  chain_free (cpu->chain);
  free (cpu);
  cpu = NULL;
}

/* Target method */
static int
bfin_connect (char *status_string, size_t status_string_len, int *can_restart)
{
  int i, j;
  char *cp;
  int need_reset;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_connect ()", bfin_target.name);

  assert (cpu);

  assert (status_string != NULL);
  assert (status_string_len >= 34);
  assert (can_restart != NULL);

  *can_restart = TRUE;

  /* Set up initial default values.  */
  cpu->general_core = INVALID_CORE;
  cpu->continue_core = INVALID_CORE;

  while (cpu->swbps)
    remove_swbp (cpu->swbps);

  assert (cpu->swbps == NULL);

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      /* We won't use IDCODE_SCAN in debugging. Set it as
         default, such that new scan will be selected.  */
      cpu->cores[i].scan = IDCODE_SCAN;
      cpu->cores[i].leave_stopped = 0;
      cpu->cores[i].is_running = 1;
      cpu->cores[i].is_interrupted = 0;
      cpu->cores[i].is_stepping = 0;
      cpu->cores[i].is_locked = 0;
      cpu->cores[i].is_corefault = 0;
      cpu->cores[i].status_pending_p = 0;
      cpu->cores[i].dmem_control_valid_p = 0;
      cpu->cores[i].imem_control_valid_p = 0;
      cpu->cores[i].dcplbs_valid_p = 0;
      cpu->cores[i].icplbs_valid_p = 0;
      cpu->cores[i].wpiactl = 0;
      cpu->cores[i].wpdactl = 0;
      cpu->cores[i].wpstat = 0;
      cpu->cores[i].dbgctl = 0;
      cpu->cores[i].dbgstat = 0;
      /* INSN_ILLEGAL is an insn we won't use in debugging.
         Set it as default, such that new instruction will
         be set.  */
      cpu->cores[i].emuir_a = INSN_ILLEGAL;
      cpu->cores[i].emuir_b = INSN_ILLEGAL;
      cpu->cores[i].emudat_out = 0;
      cpu->cores[i].emudat_in = 0;
      cpu->cores[i].emupc = -1;
      for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
	cpu->cores[i].hwbps[j] = -1;
      for (j = 0; j < RP_BFIN_MAX_HWWATCHPOINTS; j++)
	{
	  cpu->cores[i].hwwps[j].addr = -1;
	  cpu->cores[i].hwwps[j].len = 0;
	  cpu->cores[i].hwwps[j].mode = WPDA_DISABLE;
	}
    }

  /* Stop the processor.  */
  emulation_enable ();
  emulation_trigger ();

  dbgstat_get ();

  need_reset = 0;
  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      /* If there is core fault, we have to give it a reset.  */
      if (cpu->cores[i].dbgstat & DBGSTAT_CORE_FAULT)
	{
	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "[%d] core fault: DBGSTAT [0x%08X]",
		    i, cpu->cores[i].dbgstat);
	  need_reset = 1;
	}
      /* If emulator is not ready after emulation_trigger (),
	 we have to give it a reset, too.  */
      else if (!(cpu->cores[i].dbgstat & DBGSTAT_EMUREADY)
	       && (!(cpu->cores[i].dbgstat & DBGSTAT_EMUACK)
		   || (cpu->cores[i].dbgstat & DBGSTAT_IN_RESET)))
	{
	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "[%d] emulator not ready: DBGSTAT [0x%08X]",
		    i, cpu->cores[i].dbgstat);
	  need_reset = 1;
	}
    }

  if (need_reset || bfin_reset)
    {
      bfin_log (RP_VAL_LOGLEVEL_INFO, "Resetting ...");

      core_reset ();
      system_reset ();
    }

  for (i = 0; i < cpu->chain->parts->len; i++)
    if ((cpu->cores[i].dbgstat & DBGSTAT_EMUACK)
	&& !(cpu->cores[i].dbgstat & DBGSTAT_IN_RESET))
      {
	bfin_log (RP_VAL_LOGLEVEL_INFO,
		  "[%d] locked: DBGSTAT [0x%08X]", i, cpu->cores[i].dbgstat);
	cpu->cores[i].is_locked = 1;
	cpu->cores[i].is_running = 0;

	if (bfin_unlock_on_connect)
	  {
	    uint16_t sica_syscr;

	    bfin_log (RP_VAL_LOGLEVEL_INFO,
		      "%s: [%d] unlocking...", bfin_target.name, i);

	    sica_syscr = mmr_read (cpu->core_a, SICA_SYSCR, 2);
	    sica_syscr &= ~SICA_SYSCR_COREB_SRAM_INIT;
	    mmr_write (cpu->core_a, SICA_SYSCR, sica_syscr, 2);
	    core_wait_emuready (i);
	    cpu->cores[i].is_locked = 0;
	    core_wpu_init (i);

	    bfin_log (RP_VAL_LOGLEVEL_INFO,
		      "%s: [%d] done", bfin_target.name, i);
	  }
      }
    else
      {
	cpu->cores[i].is_running = 0;
	core_wpu_init (i);
      }

  if (bfin_init_sdram && cpu->sdram_config && sdram_init () != 0)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: SDRAM init failed", bfin_target.name);
      return RP_VAL_TARGETRET_ERR;
    }
	
  if (bfin_init_sdram && cpu->ddr_config && ddr_init () != 0)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: DDR init failed", bfin_target.name);
      return RP_VAL_TARGETRET_ERR;
    }

  if (bfin_enable_dcache)
    dcache_enable (bfin_enable_dcache);

  if (bfin_enable_icache)
    icache_enable ();

  if (rp_debug_level)
    {
      uint32_t rete;

      dbgstat_get ();

      for (i = 0; i < cpu->chain->parts->len; i++)
	{
	  if (!cpu->cores[i].is_locked)
	    {
	      rete = core_register_get (i, REG_RETE);
	      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
			"[%d] DBGSTAT [0x%08X] PC [0x%08X]",
			i, cpu->cores[i].dbgstat, rete);
	    }
	  else
	    bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		      "[%d] DBGSTAT [0x%08X] PC [0xXXXXXXXX]",
		      i, cpu->cores[i].dbgstat);
	}
    }

  /* Fill out the the status string.  */
  sprintf (status_string, "T%02d", RP_SIGNAL_TRAP);

  if (cpu->chain->parts->len == 1)
    {
      cp = bfin_out_treg (&status_string[3], BFIN_PC_REGNUM);
      cp = bfin_out_treg (cp, BFIN_FP_REGNUM);
    }
  else
    {
      cp = &status_string[3];
      for (i = cpu->chain->parts->len - 1; i >= 0; i--)
	{
	  sprintf (cp, "thread:%x;", THREAD_ID (i));
	  cp += strlen (cp);
	}
      sprintf (cp, "thread:%x;", THREAD_ID (cpu->core_a));
    }

  cpu->general_core = cpu->continue_core = cpu->core_a;

  return (cp != NULL) ? RP_VAL_TARGETRET_OK : RP_VAL_TARGETRET_ERR;
}

/* Target method */
static int
bfin_disconnect (void)
{
  int i;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_disconnect ()", bfin_target.name);

  wpu_disable ();

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (cpu->cores[i].is_stepping)
      {
	core_dbgctl_clear (i, DBGCTL_ESSTEP, UPDATE);
	cpu->cores[i].is_stepping = 0;
      }

  emulation_return ();

  for (i = 0; i < cpu->chain->parts->len; i++)
    cpu->cores[i].is_running = 1;

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static void
bfin_kill (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_kill ()", bfin_target.name);

  bfin_stop ();
}

static int
bfin_restart (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_restart ()", bfin_target.name);

  /* Just stop it. The actual restart will be done
     when connect is called again */
  bfin_stop ();

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static void
bfin_stop (void)
{
  int i;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_stop ()", bfin_target.name);

  assert (cpu);

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (cpu->cores[i].is_stepping)
      {
	core_dbgctl_clear (i, DBGCTL_ESSTEP, UPDATE);
	cpu->cores[i].is_stepping = 0;
      }

  emulation_trigger ();

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      cpu->cores[i].is_interrupted = 1;
      cpu->cores[i].is_running = 0;
    }
}

static int
bfin_set_gen_thread (rp_thread_ref *thread)
{
  int core;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_set_gen_thread (%lld)", bfin_target.name, thread->val);

  if (thread->val == ALL_THREADS)
    cpu->general_core = ALL_CORES;
  else if (thread->val == ANY_THREAD)
    {
      if (cpu->general_core == INVALID_CORE)
	cpu->general_core = cpu->core_a;
    }
  else
    {
      core = PART_NO (thread->val);

      if (core < 0 || core >= cpu->chain->parts->len)
	return RP_VAL_TARGETRET_ERR;

      cpu->general_core = core;
    }

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_set_ctrl_thread (rp_thread_ref *thread)
{
  int i, core;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_set_ctrl_thread (%lld)", bfin_target.name, thread->val);

  if (thread->val == ALL_THREADS)
    {
      cpu->continue_core = ALL_CORES;
      for (i = 0; i < cpu->chain->parts->len; i++)
	cpu->cores[i].leave_stopped = 0;
    }
  else if (thread->val == ANY_THREAD)
    {
      if (cpu->continue_core == INVALID_CORE)
	cpu->continue_core = cpu->core_a;
      for (i = 0; i < cpu->chain->parts->len; i++)
	cpu->cores[i].leave_stopped = 0;
    }
  else
    {
      core = PART_NO (thread->val);

      if (core < 0 || core >= cpu->chain->parts->len)
	return RP_VAL_TARGETRET_ERR;

      cpu->continue_core = core;
      for (i = 0; i < cpu->chain->parts->len; i++)
	if (i == core)
	  cpu->cores[i].leave_stopped = 0;
	else
	  cpu->cores[i].leave_stopped = 1;
    }

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_is_thread_alive (rp_thread_ref *thread, int *alive)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_is_thread_alive ()", bfin_target.name);

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_read_registers (uint8_t *data_buf,
		     uint8_t *avail_buf, size_t buf_size, size_t *read_size)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_read_registers ()", bfin_target.name);

  return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int
bfin_write_registers (uint8_t *buf, size_t write_size)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_write_registers ()", bfin_target.name);

  return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int
bfin_read_single_register (unsigned int reg_no,
			   uint8_t *data_buf,
			   uint8_t *avail_buf,
			   size_t buf_size, size_t *read_size)
{
  size_t reg_size;
  int core;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_read_single_register (%d)", bfin_target.name, reg_no);

  assert (cpu);

  core = cpu->general_core;
  reg_size = sizeof (cpu->cores[core].registers[reg_no]);

  assert (data_buf != NULL);
  assert (avail_buf != NULL);
  assert (reg_size == 4);
  assert (buf_size >= reg_size);
  assert (read_size != NULL);

  if (reg_no < 0 || reg_no >= BFIN_NUM_REGS)
    return RP_VAL_TARGETRET_ERR;

  if (cpu->cores[core].is_locked)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] locked core cannot read register [%d]",
		bfin_target.name, core, reg_no);
      memset (avail_buf, 0, reg_size);
    }
  else if (cpu->cores[core].is_corefault)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] CORE FAULT core cannot read register [%d]",
		bfin_target.name, core, reg_no);
      memset (avail_buf, 0, reg_size);
    }
  /* In GDB testsuite, we have to pretend these registers have value 0
     to get some tests PASS.  */
  else if (map_gdb_core[reg_no] == -1)
    {
      memset (data_buf, 0, reg_size);
      memset (avail_buf, 1, reg_size);
    }
  else
    {
      cpu->cores[core].registers[reg_no]
	= core_register_get (core, map_gdb_core[reg_no]);
      data_buf[0] = cpu->cores[core].registers[reg_no] & 0xff;
      data_buf[1] = (cpu->cores[core].registers[reg_no] >> 8) & 0xff;
      data_buf[2] = (cpu->cores[core].registers[reg_no] >> 16) & 0xff;
      data_buf[3] = (cpu->cores[core].registers[reg_no] >> 24) & 0xff;
      memset (avail_buf, 1, reg_size);
    }
  *read_size = reg_size;
  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_write_single_register (unsigned int reg_no,
			    uint8_t *buf, size_t write_size)
{
  size_t reg_size;
  uint32_t value;
  int core;
  int i;

  assert (cpu);

  core = cpu->general_core;
  reg_size = sizeof (cpu->cores[core].registers[reg_no]);

  assert (reg_size == 4);
  assert (buf != NULL);

  /* Read the value as little endian data.  */
  value = buf[0] + (buf[1] << 8) + (buf[2] << 16) + (buf[3] << 24);

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_write_single_register (%d, 0x%X)",
	    bfin_target.name, reg_no, value);

  assert (write_size == reg_size);

  if (reg_no < 0 || reg_no >= BFIN_NUM_REGS)
    return RP_VAL_TARGETRET_ERR;

  /* GDB don't like reporting error for these registers.  */
  if (map_gdb_core[reg_no] == -1)
    return RP_VAL_TARGETRET_OK;

  if (cpu->cores[core].is_locked)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] locked core cannot write register [%d]",
		bfin_target.name, core, reg_no);
      return RP_VAL_TARGETRET_ERR;
    }

  if (cpu->cores[core].is_corefault)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] CORE FAULT core cannot write register [%d]",
		bfin_target.name, core, reg_no);
      return RP_VAL_TARGETRET_ERR;
    }

  /* Setting RETE to an address that the general core cannot
     execute from is mostly a user error. Catch it.  */
  if (map_gdb_core[reg_no] == REG_RETE
      && value >= cpu->mem_map.l1
      && value < cpu->mem_map.l1_end
      && !((value >= cpu->cores[core].l1_map.l1_code
	    && value < cpu->cores[core].l1_map.l1_code_end)
	   || (value >= cpu->cores[core].l1_map.l1_code_cache
	       && value < cpu->cores[core].l1_map.l1_code_cache_end)
	   || (value >= cpu->cores[core].l1_map.l1_code_rom
	       && value < cpu->cores[core].l1_map.l1_code_rom_end)))
    {
      if (!bfin_auto_switch)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] set RETE an address the core cannot execute from",
		    bfin_target.name, core);
	  return RP_VAL_TARGETRET_ERR;
	}

      for (i = 0; i < cpu->chain->parts->len; i++)
	if ((value >= cpu->cores[i].l1_map.l1_code
	     && value < cpu->cores[i].l1_map.l1_code_end)
	    || (value >= cpu->cores[i].l1_map.l1_code_cache
		&& value < cpu->cores[i].l1_map.l1_code_cache_end)
	    || (value >= cpu->cores[i].l1_map.l1_code_rom
		&& value < cpu->cores[i].l1_map.l1_code_rom_end))
	  {
	    if (cpu->cores[i].is_locked)
	      {
		bfin_log (RP_VAL_LOGLEVEL_ERR,
			  "%s: [%d] locked core cannot write register [%d]",
			  bfin_target.name, i, reg_no);
		return RP_VAL_TARGETRET_ERR;
	      }

	    if (cpu->cores[i].is_corefault)
	      {
		bfin_log (RP_VAL_LOGLEVEL_ERR,
			  "%s: [%d] CORE FAULT core cannot write register [%d]",
			  bfin_target.name, i, reg_no);
		return RP_VAL_TARGETRET_ERR;
	      }

	    core = i;
	    /* Silently switch to the core to which the address belongs.  */
	    cpu->general_core = i;
	    cpu->continue_core = i;
	    break;
	  }

      if (i == cpu->chain->parts->len)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: No core can execute from [0x%08X]",
		    bfin_target.name, value);
	  return RP_VAL_TARGETRET_ERR;
	}
    }

  cpu->cores[core].registers[reg_no] = value;

  core_register_set (core, map_gdb_core[reg_no],
		     cpu->cores[core].registers[reg_no]);

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_read_mem (uint64_t addr,
	       uint8_t *buf, size_t req_size, size_t *actual_size)
{
  int i, ret;
  int avail_core, core;
  uint32_t value;
  uint32_t end;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_read_mem (0x%08llX, ptr, %d, ptr)",
	    bfin_target.name, addr, req_size);

  assert (cpu);
  assert (buf != NULL);
  assert (actual_size != NULL);

  if (req_size == 0)
    {
      *actual_size = req_size;
      return RP_VAL_TARGETRET_OK;
    }

  if (addr >= cpu->mem_map.coremmr
      && (cpu->cores[cpu->general_core].is_locked
	  || cpu->cores[cpu->general_core].is_corefault))
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] %s core cannot read its core MMR [0x%08llX]",
		bfin_target.name,
		cpu->general_core,
		cpu->cores[cpu->general_core].
		is_locked ? "locked" : "CORE FAULT", addr);
      return RP_VAL_TARGETRET_ERR;
    }

  /* If general_core is available, use it when needed.
     Otherwise choose any other available core.  */

  avail_core = cpu->general_core;

  if (cpu->cores[avail_core].is_locked
      || cpu->cores[avail_core].is_corefault)
    for (i = 0; i < cpu->chain->parts->len; i++)
      {
	if (!cpu->cores[i].is_locked
	    && !cpu->cores[i].is_corefault
	    && !cpu->cores[i].is_running)
	  {
	    avail_core = i;
	    break;
	  }

	if (i == cpu->chain->parts->len)
	  {
	    bfin_log (RP_VAL_LOGLEVEL_ERR,
		      "%s: no core available to read memory",
		      bfin_target.name);
	    return RP_VAL_TARGETRET_ERR;
	  }
      }

  if (addr < cpu->mem_map.l1 || addr >= cpu->mem_map.l1_end)
    goto skip_l1;

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      /* Use the core for its L1 memory if possible.  */
      if (!cpu->cores[i].is_locked
	  && !cpu->cores[i].is_corefault
	  && !cpu->cores[i].is_running)
	core = i;
      else
	core = avail_core;

      if (addr >= cpu->cores[i].l1_map.l1
	  && addr < cpu->cores[i].l1_map.l1_end)
	core_cache_status_get (i);
      else
	continue;

      if (addr >= cpu->cores[i].l1_map.l1_code
	  && addr < cpu->cores[i].l1_map.l1_code_end)
	{
	  if (!cpu->cores[i].l1_code_cache_enabled
	      && (cpu->cores[i].l1_map.l1_code_end
		  == cpu->cores[i].l1_map.l1_code_cache))
	    end = cpu->cores[i].l1_map.l1_code_cache_end;
	  else
	    end = cpu->cores[i].l1_map.l1_code_end;

	  if (addr + req_size > end)
	    req_size = end - addr;

	  ret = dma_sram_read (core, (uint32_t) addr, buf, req_size);
	  goto done;
	}
      else if (!cpu->cores[i].l1_code_cache_enabled
	       && addr >= cpu->cores[i].l1_map.l1_code_cache
	       && addr < cpu->cores[i].l1_map.l1_code_cache_end)
	{
	  if (addr + req_size > cpu->cores[i].l1_map.l1_code_cache_end)
	    req_size = cpu->cores[i].l1_map.l1_code_cache_end - addr;

	  ret = dma_sram_read (core, (uint32_t) addr, buf, req_size);
	  goto done;
	}
      else if (addr >= cpu->cores[i].l1_map.l1_code_rom
	       && addr < cpu->cores[i].l1_map.l1_code_rom_end)
	{
	  if (addr + req_size > cpu->cores[i].l1_map.l1_code_rom_end)
	    req_size = cpu->cores[i].l1_map.l1_code_rom_end - addr;

	  ret = dma_sram_read (core, (uint32_t) addr, buf, req_size);
	  goto done;
	}
      else if (addr >= cpu->cores[i].l1_map.l1_data_a
	       && addr < cpu->cores[i].l1_map.l1_data_a_end)
	{
	  if (!cpu->cores[i].l1_data_a_cache_enabled
	      && (cpu->cores[i].l1_map.l1_data_a_end
		  == cpu->cores[i].l1_map.l1_data_a_cache))
	    end = cpu->cores[i].l1_map.l1_data_a_cache_end;
	  else
	    end = cpu->cores[i].l1_map.l1_data_a_end;

	  if (addr + req_size > end)
	    req_size = end - addr;

	  if (i == core)
	    ret = memory_read (core, (uint32_t) addr, buf, req_size);
	  else
	    ret = dma_sram_read (core, (uint32_t) addr, buf, req_size);
	  goto done;
	}
      else if (!cpu->cores[i].l1_data_a_cache_enabled
	       && addr >= cpu->cores[i].l1_map.l1_data_a_cache
	       && addr < cpu->cores[i].l1_map.l1_data_a_cache_end)
	{
	  if (addr + req_size > cpu->cores[i].l1_map.l1_data_a_cache_end)
	    req_size = cpu->cores[i].l1_map.l1_data_a_cache_end - addr;

	  if (i == core)
	    ret = memory_read (core, (uint32_t) addr, buf, req_size);
	  else
	    ret = dma_sram_read (core, (uint32_t) addr, buf, req_size);
	  goto done;
	}
      else if (addr >= cpu->cores[i].l1_map.l1_data_b
	       && addr < cpu->cores[i].l1_map.l1_data_b_end)
	{
	  if (!cpu->cores[i].l1_data_b_cache_enabled
	      && (cpu->cores[i].l1_map.l1_data_b_end
		  == cpu->cores[i].l1_map.l1_data_b_cache))
	    end = cpu->cores[i].l1_map.l1_data_b_cache_end;
	  else
	    end = cpu->cores[i].l1_map.l1_data_b_end;

	  if (addr + req_size > end)
	    req_size = end - addr;

	  if (i == core)
	    ret = memory_read (core, (uint32_t) addr, buf, req_size);
	  else
	    ret = dma_sram_read (core, (uint32_t) addr, buf, req_size);
	  goto done;
	}
      else if (! cpu->cores[i].l1_data_b_cache_enabled
	       && addr >= cpu->cores[i].l1_map.l1_data_b_cache
	       && addr < cpu->cores[i].l1_map.l1_data_b_cache_end)
	{
	  if (addr + req_size > cpu->cores[i].l1_map.l1_data_b_cache_end)
	    req_size = cpu->cores[i].l1_map.l1_data_b_cache_end - addr;

	  if (i == core)
	    ret = memory_read (core, (uint32_t) addr, buf, req_size);
	  else
	    ret = dma_sram_read (core, (uint32_t) addr, buf, req_size);
	  goto done;
	}
      else if (addr >= cpu->cores[i].l1_map.l1_scratch
	       && addr < cpu->cores[i].l1_map.l1_scratch_end
	       && i == core)
	{
	  if (addr + req_size > cpu->cores[i].l1_map.l1_scratch_end)
	    req_size = cpu->cores[i].l1_map.l1_scratch_end - addr;
	  ret = memory_read (core, (uint32_t) addr, buf, req_size);
	  goto done;
	}
      else if (addr >= cpu->cores[i].l1_map.l1
	       && addr < cpu->cores[i].l1_map.l1_end)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] cannot read reserved L1 [0x%08llX]",
		    bfin_target.name, i, addr);
	  return RP_VAL_TARGETRET_ERR;
	}
    }

 skip_l1:

  /* TODO  Accurately check MMR validity.  */

  if (addr >= cpu->mem_map.coremmr)
    core = cpu->general_core;
  else
    core = avail_core;

  if ((addr >= cpu->mem_map.sysmmr
       && addr < cpu->mem_map.coremmr
       && addr + req_size > cpu->mem_map.coremmr)
      || (addr >= cpu->mem_map.coremmr
	  && addr + req_size < addr))
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] bad MMR addr or size [0x%08llX] size %d",
		bfin_target.name, core, addr, req_size);
      return RP_VAL_TARGETRET_ERR;
    }
  
  if (addr >= cpu->mem_map.sysmmr)
    {
      if (req_size != 2 && req_size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] bad MMR size [0x%08llX] size %d",
		    bfin_target.name, core, addr, req_size);
	  return RP_VAL_TARGETRET_ERR;
	}

      if ((addr & 0x1) == 1)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] odd MMR addr [0x%08llX]",
		    bfin_target.name, core, addr);
	  return RP_VAL_TARGETRET_ERR;
	}


      value = mmr_read (core, (uint32_t) addr, req_size);
      *buf++ = value & 0xff;
      *buf++ = (value >> 8) & 0xff;
      if (req_size == 4)
	{
	  *buf++ = (value >> 16) & 0xff;
	  *buf++ = (value >> 24) & 0xff;
	}
      ret = 0;
      goto done;
    }

  if (addr >= cpu->mem_map.sdram
      && addr < cpu->mem_map.sdram_end)
    {
      if (addr + req_size > cpu->mem_map.sdram_end)
	req_size = cpu->mem_map.sdram_end - addr;

      ret = memory_read (core, (uint32_t) addr, buf, req_size);
    }
  else if (addr >= cpu->mem_map.flash
	   && addr < cpu->mem_map.flash_end)
    {
      if (addr + req_size > cpu->mem_map.flash_end)
	req_size = cpu->mem_map.flash_end - addr;

      ret = memory_read (core, (uint32_t) addr, buf, req_size);
    }
  /* TODO  Allow access to devices mapped to async mem.  */
  else if (addr >= cpu->mem_map.boot_rom
	   && addr < cpu->mem_map.boot_rom_end)
    {
      if (addr + req_size > cpu->mem_map.boot_rom_end)
	req_size = cpu->mem_map.boot_rom_end - addr;
      ret = memory_read (core, (uint32_t) addr, buf, req_size);
    }
  else if (addr >= cpu->mem_map.l2_sram
	   && addr < cpu->mem_map.l2_sram_end)
    {
      if (addr + req_size > cpu->mem_map.l2_sram_end)
	req_size = cpu->mem_map.l2_sram_end - addr;

      ret = memory_read (core, (uint32_t) addr, buf, req_size);
    }
  else
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] cannot read reserved memory [0x%08llX]",
		bfin_target.name, core, addr);
      return RP_VAL_TARGETRET_ERR;
    }

done:

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_read_mem () through Core [%d]",
	    bfin_target.name, core);

  *actual_size = req_size;

  if (ret < 0)
    return RP_VAL_TARGETRET_ERR;
  else
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_write_mem (uint64_t addr, uint8_t *buf, size_t write_size)
{
  int i, ret;
  int avail_core, core;
  uint32_t value;
  uint32_t end;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_write_mem (0x%08llX, ptr, %d)",
	    bfin_target.name, addr, write_size);

  assert (cpu);
  assert (buf != NULL);

  /* GDB does zero length writes for some reason. Treat them harmlessly.  */
  if (write_size == 0)
    return RP_VAL_TARGETRET_OK;

  if (addr >= cpu->mem_map.coremmr
      && (cpu->cores[cpu->general_core].is_locked
	  || cpu->cores[cpu->general_core].is_corefault))
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] %s core cannot write its core MMR [0x%08llX]",
		bfin_target.name,
		cpu->general_core,
		cpu->cores[cpu->general_core].
		is_locked ? "locked" : "CORE FAULT", addr);
      return RP_VAL_TARGETRET_ERR;
    }

  /* If general_core is available, use it when needed.
     Otherwise choose any other available core.  */

  avail_core = cpu->general_core;

  if (cpu->cores[avail_core].is_locked
      || cpu->cores[avail_core].is_corefault)
    for (i = 0; i < cpu->chain->parts->len; i++)
      {
	if (!cpu->cores[i].is_locked
	    && !cpu->cores[i].is_corefault
	    && !cpu->cores[i].is_running)
	  {
	    avail_core = i;
	    break;
	  }

	if (i == cpu->chain->parts->len)
	  {
	    bfin_log (RP_VAL_LOGLEVEL_ERR,
		      "%s: no core available to write memory",
		      bfin_target.name);
	    return RP_VAL_TARGETRET_ERR;
	  }
      }

  if (addr < cpu->mem_map.l1 || addr >= cpu->mem_map.l1_end)
    goto skip_l1;

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      /* Use the core for its L1 memory if possible.  */
      if (!cpu->cores[i].is_locked
	  && !cpu->cores[i].is_corefault
	  && !cpu->cores[i].is_running)
	core = i;
      else
	core = avail_core;

      if (addr >= cpu->cores[i].l1_map.l1
	  && addr < cpu->cores[i].l1_map.l1_end)
	core_cache_status_get (i);
      else
	continue;

      if (addr >= cpu->cores[i].l1_map.l1_code
	  && addr < cpu->cores[i].l1_map.l1_code_end
	  && ((!cpu->cores[i].l1_code_cache_enabled
	       && (cpu->cores[i].l1_map.l1_code_end
		   == cpu->cores[i].l1_map.l1_code_cache)
	       && (end = cpu->cores[i].l1_map.l1_code_cache_end))
	      || (end = cpu->cores[i].l1_map.l1_code_end))
	  && addr + write_size <= end)
	{
	  ret = dma_sram_write (core, (uint32_t) addr, buf, write_size);

	  if (i == core)
	    icache_flush (i, addr, write_size);

	  if (addr == cpu->cores[i].l1_map.l1_code
	      && bfin_unlock_on_load && cpu->cores[i].is_locked)
	    {
	      uint16_t sica_syscr;

	      bfin_log (RP_VAL_LOGLEVEL_INFO,
			"%s: [%d] unlocking...", bfin_target.name, i);

	      sica_syscr = mmr_read (cpu->core_a, SICA_SYSCR, 2);
	      sica_syscr &= ~SICA_SYSCR_COREB_SRAM_INIT;
	      mmr_write (cpu->core_a, SICA_SYSCR, sica_syscr, 2);
	      core_wait_emuready (i);
	      cpu->cores[i].is_locked = 0;
	      core_wpu_init (i);
	      if (bfin_enable_dcache)
		{
		  core_dcache_enable (i, bfin_enable_dcache);
		  cpu->cores[i].l1_data_a_cache_enabled = 1;
		  cpu->cores[i].l1_data_b_cache_enabled = 1;
		}
	      if (bfin_enable_icache)
		{
		  core_icache_enable (i);
		  cpu->cores[i].l1_code_cache_enabled = 1;
		}

	      bfin_log (RP_VAL_LOGLEVEL_INFO,
			"%s: [%d] done", bfin_target.name, i);
	    }

	  goto done;
	}
      else if (!cpu->cores[i].l1_code_cache_enabled
	       && addr >= cpu->cores[i].l1_map.l1_code_cache
	       && addr < cpu->cores[i].l1_map.l1_code_cache_end
	       && addr + write_size <= cpu->cores[i].l1_map.l1_code_cache_end)
	{
	  ret = dma_sram_write (core, (uint32_t) addr, buf, write_size);
	  goto done;
	}
      else if (addr >= cpu->cores[i].l1_map.l1_data_a
	       && addr < cpu->cores[i].l1_map.l1_data_a_end
	       && ((!cpu->cores[i].l1_data_a_cache_enabled
		    && (cpu->cores[i].l1_map.l1_data_a_end
			== cpu->cores[i].l1_map.l1_data_a_cache)
		    && (end = cpu->cores[i].l1_map.l1_data_a_cache_end))
		   || (end = cpu->cores[i].l1_map.l1_data_a_end))
	       && addr + write_size <= end)
	{
	  if (i == core)
	    ret = memory_write (core, (uint32_t) addr, buf, write_size);
	  else
	    ret = dma_sram_write (core, (uint32_t) addr, buf, write_size);
	  goto done;
	}
      else if (!cpu->cores[i].l1_data_a_cache_enabled
	       && addr >= cpu->cores[i].l1_map.l1_data_a_cache
	       && addr < cpu->cores[i].l1_map.l1_data_a_cache_end
	       && addr + write_size <= cpu->cores[i].l1_map.l1_data_a_cache_end)
	{
	  if (i == core)
	    ret = memory_write (core, (uint32_t) addr, buf, write_size);
	  else
	    ret = dma_sram_write (core, (uint32_t) addr, buf, write_size);
	  goto done;
	}
      else if (addr >= cpu->cores[i].l1_map.l1_data_b
	       && addr < cpu->cores[i].l1_map.l1_data_b_end
	       && ((!cpu->cores[i].l1_data_b_cache_enabled
		    && (cpu->cores[i].l1_map.l1_data_b_end
			== cpu->cores[i].l1_map.l1_data_b_cache)
		    && (end = cpu->cores[i].l1_map.l1_data_b_cache_end))
		   || (end = cpu->cores[i].l1_map.l1_data_b_end))
	       && addr + write_size <= end)
	{
	  if (i == core)
	    ret = memory_write (core, (uint32_t) addr, buf, write_size);
	  else
	    ret = dma_sram_write (core, (uint32_t) addr, buf, write_size);
	  goto done;
	}
      else if (!cpu->cores[i].l1_data_b_cache_enabled
	       && addr >= cpu->cores[i].l1_map.l1_data_b_cache
	       && addr < cpu->cores[i].l1_map.l1_data_b_cache_end
	       && addr + write_size <= cpu->cores[i].l1_map.l1_data_b_cache_end)
	{
	  if (i == core)
	    ret = memory_write (core, (uint32_t) addr, buf, write_size);
	  else
	    ret = dma_sram_write (core, (uint32_t) addr, buf, write_size);
	  goto done;
	}
      else if (addr >= cpu->cores[i].l1_map.l1_scratch
	       && addr < cpu->cores[i].l1_map.l1_scratch_end
	       && i == core
	       && addr + write_size <= cpu->cores[i].l1_map.l1_scratch_end)
	{
	  ret = memory_write (core, (uint32_t) addr, buf, write_size);
	  goto done;
	}
      else if (addr >= cpu->cores[i].l1_map.l1
	       && addr < cpu->cores[i].l1_map.l1_end)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] cannot write reserved L1 [0x%08llX] size %d",
		    bfin_target.name, i, addr, write_size);
	  return RP_VAL_TARGETRET_ERR;
	}
    }

 skip_l1:

  /* TODO  Accurately check MMR validity.  */

  if (addr >= cpu->mem_map.coremmr)
    core = cpu->general_core;
  else
    core = avail_core;

  if ((addr >= cpu->mem_map.sysmmr
       && addr < cpu->mem_map.coremmr
       && addr + write_size > cpu->mem_map.coremmr)
      || (addr >= cpu->mem_map.coremmr
	  && addr + write_size < addr))
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] bad MMR addr or size [0x%08llX] size %d",
		bfin_target.name, core, addr, write_size);
      return RP_VAL_TARGETRET_ERR;
    }

  if (addr >= cpu->mem_map.sysmmr)
    {
      if (write_size != 2 && write_size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] bad MMR size [0x%08llX] size %d",
		    bfin_target.name, core, addr, write_size);
	  return RP_VAL_TARGETRET_ERR;
	}

      if ((addr & 0x1) == 1)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] odd MMR addr [0x%08llX]",
		    bfin_target.name, core, addr);
	  return RP_VAL_TARGETRET_ERR;
	}

      value = *buf++;
      value |= (*buf++) << 8;
      if (write_size == 4)
	{
	  value |= (*buf++) << 16;
	  value |= (*buf++) << 24;
	}
      mmr_write (core, (uint32_t) addr, value, write_size);
      ret = 0;
      goto done;
    }

  if ((addr >= cpu->mem_map.sdram
       && addr < cpu->mem_map.sdram_end
       && addr + write_size <= cpu->mem_map.sdram_end)
      || (addr >= cpu->mem_map.flash
	  && addr < cpu->mem_map.flash_end
	  && addr + write_size <= cpu->mem_map.flash_end)
      /* TODO  Allow access to devices mapped to async mem.  */
      || (addr >= cpu->mem_map.l2_sram
	  && addr < cpu->mem_map.l2_sram_end
	  && addr + write_size <= cpu->mem_map.l2_sram_end))
    {
      ret = memory_write (core, (uint32_t) addr, buf, write_size);
      cache_flush (core, addr, write_size);

      for (i = 0; i < cpu->chain->parts->len; i++)
	if (!cpu->cores[i].is_locked
	    && !cpu->cores[i].is_corefault
	    && !cpu->cores[i].is_running
	    && i != core)
	  icache_flush (i, addr, write_size);
    }
  else
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] cannot write memory [0x%08llX]",
		bfin_target.name, core, addr);
      return RP_VAL_TARGETRET_ERR;
    }

done:

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_write_mem () through Core [%d]",
	    bfin_target.name, core);

  if (ret < 0)
    return RP_VAL_TARGETRET_ERR;

  return RP_VAL_TARGETRET_OK;
}


/* Target method */
static int
bfin_resume_from_current (int step, int sig)
{
  int i, ret;
  uint8_t buf[2];
  size_t size;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_resume_from_current (%s, %d)",
	    bfin_target.name, (step) ? "step" : "run", sig);

  assert (cpu);

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      if (cpu->cores[i].is_locked)
	continue;

      if (cpu->cores[i].leave_stopped)
	continue;

      if (cpu->cores[i].is_running)
	continue;

      if (cpu->cores[i].status_pending_p
	  && cpu->cores[i].pending_is_breakpoint)
	{
	  ret = bfin_read_mem (cpu->cores[i].pending_stop_pc, buf, 2, &size);
	  assert (ret == RP_VAL_TARGETRET_OK && size == 2);

	  if (buf[0] != bfin_breakpoint_16[0]
	      || buf[1] != bfin_breakpoint_16[1])
	    {
	      /* The breakpoint is gone. Consume the pending status.  */
	      cpu->cores[i].pending_is_breakpoint = 0;
	      cpu->cores[i].status_pending_p = 0;
	    }
	}
      if (cpu->cores[i].status_pending_p)
	return RP_VAL_TARGETRET_OK;
    }

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      if (cpu->cores[i].is_locked)
	continue;

      if (cpu->cores[i].leave_stopped)
	continue;

      if (cpu->cores[i].is_running)
	continue;

      if (i == cpu->continue_core)
	{
	  if (step && !cpu->cores[i].is_stepping)
	    {
	      core_dbgctl_set (i, DBGCTL_ESSTEP, UPDATE);
	      cpu->cores[i].is_stepping = 1;
	    }
	  else if (!step && cpu->cores[i].is_stepping)
	    {
	      core_dbgctl_clear (i, DBGCTL_ESSTEP, UPDATE);
	      cpu->cores[i].is_stepping = 0;
	    }

	  cpu->cores[i].is_running = 1;
	  cpu->cores[i].is_interrupted = 0;
	  cpu->cores[i].dmem_control_valid_p = 0;
	  cpu->cores[i].imem_control_valid_p = 0;
	  cpu->cores[i].dcplbs_valid_p = 0;
	  cpu->cores[i].icplbs_valid_p = 0;
	  core_emulation_return (i);
	}
      else if (!cpu->cores[i].leave_stopped)
	{
	  if (cpu->cores[i].is_stepping)
	    {
	      core_dbgctl_clear (i, DBGCTL_ESSTEP, UPDATE);
	      cpu->cores[i].is_stepping = 0;
	    }

	  cpu->cores[i].is_running = 1;
	  cpu->cores[i].is_interrupted = 0;
	  cpu->cores[i].dmem_control_valid_p = 0;
	  cpu->cores[i].imem_control_valid_p = 0;
	  cpu->cores[i].dcplbs_valid_p = 0;
	  cpu->cores[i].icplbs_valid_p = 0;
	  core_emulation_return (i);
	}
    }

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_resume_from_addr (int step, int sig, uint64_t addr)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_resume_from_addr (%s, %d, 0x%08llX)",
	    bfin_target.name, (step) ? "step" : "run", sig, addr);

  assert (cpu);

  core_register_set (cpu->continue_core, REG_RETE, addr);

  return bfin_resume_from_current (step, sig);
}

/* Target method */
static int
bfin_go_waiting (int sig)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_go_waiting ()", bfin_target.name);
  return RP_VAL_TARGETRET_NOSUPP;
}

static void
bfin_log_emucause (int core, uint16_t cause, uint32_t rete, uint32_t fp)
{
  bfin_log (RP_VAL_LOGLEVEL_INFO,
	    "%s: [%d] %s: PC [0x%08X] FP [0x%08X]",
	    bfin_target.name, core, emucause_infos[cause], rete, fp);
}

/* Target method */
static int
bfin_wait_partial (int first,
		   char *status_string,
		   size_t status_string_len,
		   out_func of, int *implemented, int *more)
{
  uint16_t cause;
  char *cp;
  int sig;
  uint32_t pc, fp;
  int i, j;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_wait_partial ()", bfin_target.name);

  assert (cpu);
  assert (status_string != NULL);
  assert (status_string_len >= 34);
  assert (of != NULL);
  assert (implemented != NULL);
  assert (more != NULL);

  *implemented = TRUE;

  /* If we have any interesting pending event,
     report it instead of resume.  */
  for (i = 0; i < cpu->chain->parts->len; i++)
    if (!cpu->cores[i].is_locked
	&& !cpu->cores[i].leave_stopped && cpu->cores[i].status_pending_p)
      {
	sprintf (status_string, "T%02d", cpu->cores[i].pending_signal);
	cp = &status_string[3];
	if (cpu->cores[i].is_corefault)
	  {
	    pc = cpu->cores[i].emupc;
	    cp = bfin_out_treg_value (cp, BFIN_PC_REGNUM, pc);
	  }
	else
	  {
	    if (cpu->cores[i].wpstat & 0xc0)
	      core_single_step (i);
	    if (cpu->cores[i].wpstat & 0xff)
	      core_wpstat_clear (i);
	    pc = core_register_get (i, REG_RETE);
	    fp = core_register_get (i, REG_FP);
	    cp = bfin_out_treg_value (cp, BFIN_PC_REGNUM, pc);
	    cp = bfin_out_treg_value (cp, BFIN_FP_REGNUM, fp);
	  }
	if (cpu->chain->parts->len > 1)
	  sprintf (cp, "thread:%x;", THREAD_ID (i));

	cpu->cores[i].status_pending_p = 0;

	cpu->general_core = i;
	cpu->continue_core = i;

	*more = FALSE;

	return RP_VAL_TARGETRET_OK;
      }

  if (first)
    nanosleep (&bfin_loop_wait_first_ts, NULL);
  else
    nanosleep (&bfin_loop_wait_ts, NULL);

  dbgstat_get ();

  /* Quickly check if there is any core stopped.
     If so, stop others.  */

  *more = TRUE;

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      if (cpu->cores[i].leave_stopped)
	continue;

      if ((cpu->cores[i].dbgstat & DBGSTAT_EMUREADY)
	  && cpu->cores[i].is_locked)
	{
	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "%s: [%d] core unlocking...", bfin_target.name, i);

	  /* Set pending hardware breakpoints and watchpoints.  */

	  core_wpu_init (i);

	  for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
	    if (cpu->cores[i].hwbps[j] != -1)
	      wpu_set_wpia (i, j, cpu->cores[i].hwbps[j], 1);

	  if ((bfin_force_range_wp
	       || cpu->cores[i].hwwps[0].len > 4)
	      && cpu->cores[i].hwwps[j].mode != WPDA_DISABLE)
	    wpu_set_wpda (i,
			  0,
			  cpu->cores[i].hwwps[0].addr,
			  cpu->cores[i].hwwps[0].len,
			  1, cpu->cores[i].hwwps[0].mode);
	  else
	    for (j = 0; j < RP_BFIN_MAX_HWWATCHPOINTS; j++)
	      if (cpu->cores[i].hwwps[j].mode != WPDA_DISABLE)
		wpu_set_wpda (i,
			      j,
			      cpu->cores[i].hwwps[j].addr,
			      cpu->cores[i].hwwps[j].len,
			      0, cpu->cores[i].hwwps[j].mode);

	  if (bfin_enable_dcache)
	    {
	      core_dcache_enable (i, bfin_enable_dcache);
	      cpu->cores[i].l1_code_cache_enabled = 1;
	    }
	  if (bfin_enable_icache)
	    {
	      core_icache_enable (i);
	      cpu->cores[i].l1_data_a_cache_enabled = 1;
	      cpu->cores[i].l1_data_b_cache_enabled = 1;
	    }

	  core_emulation_return (i);
	  cpu->cores[i].is_locked = 0;
	  cpu->cores[i].is_running = 1;

	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "%s: [%d] done", bfin_target.name, i);
	}
      if ((cpu->cores[i].dbgstat & DBGSTAT_CORE_FAULT)
	  || (cpu->cores[i].dbgstat & DBGSTAT_EMUREADY))
	{
	  *more = FALSE;
	}
      else if (cpu->cores[i].dbgstat & DBGSTAT_IDLE)
	{
	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "%s: [%d] core is in idle mode", bfin_target.name, i);
	}
      else if (cpu->cores[i].dbgstat & DBGSTAT_IN_RESET)
	{
	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "%s: [%d] core is currently being reset",
		    bfin_target.name, i);
	}
    }

  /* No core stopped. keep waiting. Otherwise, stop all. */

  if (*more == TRUE)
    return RP_VAL_TARGETRET_OK;
  else
    {
      emulation_trigger ();
      for (i = 0; i < cpu->chain->parts->len; i++)
	cpu->cores[i].is_running = 0;
    }

  /* All cores are stopped. Check their status.  */

  dbgstat_get ();
  emupc_get ();

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      if (cpu->cores[i].leave_stopped)
	continue;

      if (cpu->cores[i].is_locked)
	continue;

      cause = (cpu->cores[i].dbgstat & DBGSTAT_EMUCAUSE_MASK) >> 6;

      if (cpu->cores[i].dbgstat & DBGSTAT_CORE_FAULT)
	{
	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "%s: [%d] a double fault has occured EMUPC [0x%08X]",
		    bfin_target.name, i, core_emupc_get (i));

	  sig = RP_SIGNAL_TRAP;
	  cpu->cores[i].is_corefault = 1;
	  cpu->cores[i].status_pending_p = 1;
	  cpu->cores[i].pending_is_breakpoint = 0;
	  cpu->cores[i].pending_signal = sig;
	  cpu->cores[i].pending_stop_pc = cpu->cores[i].emupc;
	}
      else if (cpu->cores[i].dbgstat & DBGSTAT_EMUREADY)
	{
	  pc = core_register_get (i, REG_RETE);
	  fp = core_register_get (i, REG_FP);

	  core_wpstat_get (i);

	  if (cpu->cores[i].wpstat & 0xff)
	    {
	      sig = RP_SIGNAL_TRAP;
	      cause = EMUCAUSE_WATCHPOINT;
	    }
	  else
	    switch (cause)
	      {
	      case EMUCAUSE_EMUEXCPT:
		pc -= 2;
		core_register_set (i, REG_RETE, pc);
		/* Fall through.  */

	      case EMUCAUSE_SINGLE_STEP:
		sig = RP_SIGNAL_TRAP;
		break;


	      case EMUCAUSE_WATCHPOINT:
		abort ();
		break;

	      case EMUCAUSE_EMUIN:
		/* If it isn't interrupted, it was stopped by
		   emulation_trigger () because there is some
		   other core stopped. Don't report it.  */
		if (cpu->cores[i].is_interrupted)
		  sig = RP_SIGNAL_INTERRUPT;
		else
		  sig = RP_SIGNAL_ABORTED;
		break;

	      case EMUCAUSE_PM0_OVERFLOW:
	      case EMUCAUSE_PM1_OVERFLOW:
	      default:
		/* We don't handle these causes now.  */
		sig = RP_SIGNAL_ABORTED;
		break;
	      }

	  bfin_log_emucause (i, cause, pc, fp);

	  if (cause != EMUCAUSE_EMUIN || cpu->cores[i].is_interrupted)
	    {
	      cpu->cores[i].status_pending_p = 1;

	      /* We only have to distinguish the breakpoint added by GDB.
	         GDB only use software breakpoint for its own purpose.  */

	      if (cause == EMUCAUSE_EMUEXCPT)
		cpu->cores[i].pending_is_breakpoint = 1;
	      else
		cpu->cores[i].pending_is_breakpoint = 0;

	      cpu->cores[i].pending_signal = sig;
	      cpu->cores[i].pending_stop_pc = pc;
	    }
	}
      else
	{
	  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		    "%s: [%d] unhandled debug status [0x%08X] EMUPC [0x%08X]",
		    bfin_target.name,
		    i, cpu->cores[i].dbgstat, cpu->cores[i].emupc);
	}
    }

  for (i = 0; i < cpu->chain->parts->len; i++)
    {
      if (cpu->cores[i].leave_stopped)
	continue;

      if (cpu->cores[i].status_pending_p)
	{
	  sprintf (status_string, "T%02d", cpu->cores[i].pending_signal);
	  cp = &status_string[3];
	  if (cpu->cores[i].is_corefault)
	    {
	      pc = cpu->cores[i].pending_stop_pc;
	      cp = bfin_out_treg_value (cp, BFIN_PC_REGNUM, pc);
	    }
	  else
	    {
	      if (cpu->cores[i].wpstat & 0xff)
		{
		  if (cpu->cores[i].wpstat & 0xc0)
		    core_single_step (i);
		  core_wpstat_clear (i);
		}
	      pc = core_register_get (i, REG_RETE);
	      fp = core_register_get (i, REG_FP);

	      cp = bfin_out_treg_value (cp, BFIN_PC_REGNUM, pc);
	      cp = bfin_out_treg_value (cp, BFIN_FP_REGNUM, fp);
	    }
	  if (cpu->chain->parts->len > 1)
	    sprintf (cp, "thread:%x;", THREAD_ID (i));

	  cpu->cores[i].status_pending_p = 0;

	  cpu->general_core = i;
	  cpu->continue_core = i;

	  /* Consume all interrupts.  */
	  for (i = 0; i < cpu->chain->parts->len; i++)
	    if (cpu->cores[i].status_pending_p
		&& cpu->cores[i].is_interrupted
		&& cpu->cores[i].pending_signal == RP_SIGNAL_INTERRUPT)
	      cpu->cores[i].status_pending_p = 0;

	  break;
	}
    }

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_wait (char *status_string,
	   size_t status_string_len, out_func of, int *implemented)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_wait()", bfin_target.name);

  assert (cpu);

  assert (status_string != NULL);
  assert (status_string_len >= 34);
  assert (of != NULL);
  assert (implemented != NULL);

  *implemented = FALSE;

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_process_query (unsigned int *mask,
		    rp_thread_ref *arg, rp_thread_info *info)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_process_query()", bfin_target.name);
  /* TODO: Does your target support threads? Is so, implement this function.
     Otherwise just return no support. */
  return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int
bfin_list_query (int first,
		 rp_thread_ref *arg,
		 rp_thread_ref *result,
		 size_t max_num, size_t *num, int *done)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_list_query()", bfin_target.name);
  /* TODO: Does your target support threads? Is so, implement this function.
     Otherwise just return no support. */
  return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int
bfin_current_thread_query (rp_thread_ref *thread)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_current_thread_query ()", bfin_target.name);

  thread->val = THREAD_ID (cpu->continue_core);
  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_offsets_query (uint64_t *text, uint64_t *data, uint64_t *bss)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_offsets_query ()", bfin_target.name);

  return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int
bfin_crc_query (uint64_t addr, size_t len, uint32_t *val)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_crc_query()", bfin_target.name);

  return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int
bfin_raw_query (char *in_buf, char *out_buf, size_t out_buf_size)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_raw_query ()", bfin_target.name);

  return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int
bfin_threadinfo_query (int first, char *out_buf, size_t out_buf_size)
{
  static int i;

  if (first)
    i = cpu->chain->parts->len - 1;
  else
    i--;

  if (i < 0)
    {
      out_buf[0] = 'l';
      out_buf[1] = '\0';
      return RP_VAL_TARGETRET_OK;
    }

  sprintf (out_buf, "m%x", THREAD_ID (i));
  if (strlen (out_buf) >= out_buf_size)
    return RP_VAL_TARGETRET_ERR;
  else
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_threadextrainfo_query (rp_thread_ref *thread, char *out_buf,
			    size_t out_buf_size)
{
  int core;
  char *cp;

  core = PART_NO (thread->val);

  if (core < 0 || core >= cpu->chain->parts->len)
    return RP_VAL_TARGETRET_ERR;

  cp = out_buf;
  sprintf (cp, "%s", cpu->cores[core].name);
  cp += strlen (cp);

  if (cpu->cores[core].is_locked)
    {
      sprintf (cp, " Locked");
      cp += strlen (cp);
    }

  sprintf (cp, " DBGSTAT [0x%08X]", cpu->cores[core].dbgstat);

  if (strlen (out_buf) >= out_buf_size)
    return RP_VAL_TARGETRET_ERR;
  else
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_packetsize_query (char *out_buf, size_t out_buf_size)
{
  int i;
  size_t size;

  size = RP_PARAM_INOUTBUF_SIZE - 1;
  for (i = 0; i < cpu->chain->parts->len; i++)
    if (size > cpu->cores[i].l1_map.l1_data_a_end
	- cpu->cores[i].l1_map.l1_data_a)
      size = cpu->cores[i].l1_map.l1_data_a_end
	- cpu->cores[i].l1_map.l1_data_a;

  /* 0x4000 is the largest packet size GDB would like.  */
  if (size > 0x4000)
    size = 0x4000;

  sprintf (out_buf, "PacketSize=%zx", size);

  if (strlen (out_buf) >= out_buf_size)
    return RP_VAL_TARGETRET_ERR;
  else
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_add_break (int type, uint64_t addr, unsigned int len)
{
  int mode;
  int i, j;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_add_break (%d, 0x%08llx, %d)",
	    bfin_target.name, type, addr, len);

  switch (type)
    {
    case 0:
      if (find_swbp_at (addr) != NULL)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: breakpoint already at 0x%08llx",
		    bfin_target.name, addr);
	  return RP_VAL_TARGETRET_ERR;
	}
      else if (add_swbp_at (addr) != NULL)
	return RP_VAL_TARGETRET_OK;
      else
	return RP_VAL_TARGETRET_ERR;

    case 1:
      if (addr >= cpu->mem_map.l1 && addr < cpu->mem_map.l1_end)
	{
	  for (i = 0; i < cpu->chain->parts->len; i++)
	    if (addr >= cpu->cores[i].l1_map.l1
		&& addr < cpu->cores[i].l1_map.l1_end)
	      {
		for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
		  if (cpu->cores[i].hwbps[j] == -1)
		    {
		      cpu->cores[i].hwbps[j] = addr;
		      if (!cpu->cores[i].is_locked)
			wpu_set_wpia (i, j, addr, 1);
		      return RP_VAL_TARGETRET_OK;
		    }

		bfin_log (RP_VAL_LOGLEVEL_ERR,
			  "%s: [%d] no more hardware breakpoint available",
			  bfin_target.name, i);
		return RP_VAL_TARGETRET_ERR;
	      }

	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: no core to set hardware breakpoint at 0x%08llX",
		    bfin_target.name, addr);
	  return RP_VAL_TARGETRET_ERR;
	}

      for (i = 0; i < cpu->chain->parts->len; i++)
	{
	  for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
	    if (cpu->cores[i].hwbps[j] == -1)
	      break;

	  if (j == RP_BFIN_MAX_HWBREAKPOINTS)
	    break;
	}

      if (i < cpu->chain->parts->len)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] no more hardware breakpoint available",
		    bfin_target.name, i);
	  return RP_VAL_TARGETRET_ERR;
	}

      for (i = 0; i < cpu->chain->parts->len; i++)
	for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
	  if (cpu->cores[i].hwbps[j] == -1)
	    {
	      cpu->cores[i].hwbps[j] = addr;
	      if (!cpu->cores[i].is_locked)
		wpu_set_wpia (i, j, addr, 1);
	      break;
	    }

      return RP_VAL_TARGETRET_OK;

    case 2:
      mode = WPDA_WRITE;
      break;

    case 3:
      mode = WPDA_READ;
      break;

    case 4:
      mode = WPDA_ALL;
      break;

    default:
      return RP_VAL_TARGETRET_NOSUPP;
    }

  if (addr >= cpu->mem_map.l1 && addr < cpu->mem_map.l1_end)
    {
      for (i = 0; i < cpu->chain->parts->len; i++)
	if (addr >= cpu->cores[i].l1_map.l1
	    && addr < cpu->cores[i].l1_map.l1_end)
	  {
	    if (!bfin_force_range_wp
		&& len <= 4
		&& (cpu->cores[i].hwwps[0].addr == -1
		    || cpu->cores[i].hwwps[1].addr == -1))
	      {
		j = cpu->cores[i].hwwps[0].addr == -1 ? 0 : 1;
		cpu->cores[i].hwwps[j].addr = addr;
		cpu->cores[i].hwwps[j].len = len;
		cpu->cores[i].hwwps[j].mode = mode;
		if (!cpu->cores[i].is_locked)
		  wpu_set_wpda (i, j, addr, len, 0, mode);
		return RP_VAL_TARGETRET_OK;
	      }
	    else if ((bfin_force_range_wp || len > 4)
		     && cpu->cores[i].hwwps[0].addr == -1
		     && cpu->cores[i].hwwps[1].addr == -1)
	      {
		cpu->cores[i].hwwps[0].addr = addr;
		cpu->cores[i].hwwps[0].len = len;
		cpu->cores[i].hwwps[0].mode = mode;
		cpu->cores[i].hwwps[1].addr = addr;
		cpu->cores[i].hwwps[1].len = len;
		cpu->cores[i].hwwps[1].mode = mode;
		if (!cpu->cores[i].is_locked)
		  wpu_set_wpda (i, 0, addr, len, 1, mode);
		return RP_VAL_TARGETRET_OK;
	      }

	    bfin_log (RP_VAL_LOGLEVEL_ERR,
		      "%s: [%d] no more hardware watchpoint available",
		      bfin_target.name, i);
	    return RP_VAL_TARGETRET_ERR;
	  }

      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: no core to set hardware watchpoint at 0x%08llX",
		bfin_target.name, addr);
      return RP_VAL_TARGETRET_ERR;
    }

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (!bfin_force_range_wp
	&& len <= 4
	&& cpu->cores[i].hwwps[0].addr != -1
	&& cpu->cores[i].hwwps[1].addr != -1)
      break;
    else if ((bfin_force_range_wp || len > 4)
	     && (cpu->cores[i].hwwps[0].addr != -1
		 || cpu->cores[i].hwwps[1].addr != -1))
      break;

  if (i < cpu->chain->parts->len)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] no more hardware breakpoint available",
		bfin_target.name, i);
      return RP_VAL_TARGETRET_ERR;
    }

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (!bfin_force_range_wp && len <= 4)
      {
	j = cpu->cores[i].hwwps[0].addr == -1 ? 0 : 1;

	cpu->cores[i].hwwps[j].addr = addr;
	cpu->cores[i].hwwps[j].len = len;
	cpu->cores[i].hwwps[j].mode = mode;
	if (!cpu->cores[i].is_locked)
	  wpu_set_wpda (i, j, addr, len, 0, mode);
      }
    else
      {
	cpu->cores[i].hwwps[0].addr = addr;
	cpu->cores[i].hwwps[0].len = len;
	cpu->cores[i].hwwps[0].mode = mode;
	cpu->cores[i].hwwps[1].addr = addr;
	cpu->cores[i].hwwps[1].len = len;
	cpu->cores[i].hwwps[1].mode = mode;
	if (!cpu->cores[i].is_locked)
	  wpu_set_wpda (i, 0, addr, len, 1, mode);
      }
  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_remove_break (int type, uint64_t addr, unsigned int len)
{
  int mode;
  int i, j;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_remove_break(%d, 0x%08llx, %d)",
	    bfin_target.name, type, addr, len);

  switch (type)
    {
    case 0:
      if (remove_swbp (find_swbp_at (addr)) == 0)
	return RP_VAL_TARGETRET_OK;
      else
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: no breakpoint at 0x%08llx", bfin_target.name, addr);
	  return RP_VAL_TARGETRET_ERR;
	}

    case 1:
      if (addr >= cpu->mem_map.l1 && addr < cpu->mem_map.l1_end)
	{
	  for (i = 0; i < cpu->chain->parts->len; i++)
	    if (addr >= cpu->cores[i].l1_map.l1
		&& addr < cpu->cores[i].l1_map.l1_end)
	      {
		for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
		  if (cpu->cores[i].hwbps[j] == addr)
		    {
		      cpu->cores[i].hwbps[j] = -1;
		      if (!cpu->cores[i].is_locked)
			wpu_set_wpia (i, j, addr, 0);
		      return RP_VAL_TARGETRET_OK;
		    }

		bfin_log (RP_VAL_LOGLEVEL_ERR,
			  "%s: [%d] no hardware breakpoint at 0x%08llX",
			  bfin_target.name, i, addr);
		return RP_VAL_TARGETRET_ERR;
	      }

	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: no core has hardware breakpoint at 0x%08llX",
		    bfin_target.name, addr);
	  return RP_VAL_TARGETRET_ERR;
	}

      for (i = 0; i < cpu->chain->parts->len; i++)
	{
	  for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
	    if (cpu->cores[i].hwbps[j] == addr)
	      break;

	  if (j == RP_BFIN_MAX_HWBREAKPOINTS)
	    break;
	}

      if (i < cpu->chain->parts->len)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] no hardware breakpoint at 0x%08llX",
		    bfin_target.name, i, addr);
	  return RP_VAL_TARGETRET_ERR;
	}

      for (i = 0; i < cpu->chain->parts->len; i++)
	for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
	  if (cpu->cores[i].hwbps[j] == addr)
	    {
	      cpu->cores[i].hwbps[j] = -1;
	      if (!cpu->cores[i].is_locked)
		wpu_set_wpia (i, j, addr, 0);
	      break;
	    }

      return RP_VAL_TARGETRET_OK;

    case 2:
      mode = WPDA_WRITE;
      break;

    case 3:
      mode = WPDA_READ;
      break;

    case 4:
      mode = WPDA_ALL;
      break;

    default:
      return RP_VAL_TARGETRET_NOSUPP;
    }

  if (addr >= cpu->mem_map.l1 && addr < cpu->mem_map.l1_end)
    {
      for (i = 0; i < cpu->chain->parts->len; i++)
	if (addr >= cpu->cores[i].l1_map.l1
	    && addr < cpu->cores[i].l1_map.l1_end)
	  {
	    if (!bfin_force_range_wp
		&& len <= 4
		&& ((cpu->cores[i].hwwps[0].addr == addr
		     && cpu->cores[i].hwwps[0].len == len
		     && cpu->cores[i].hwwps[0].mode == mode)
		    || (cpu->cores[i].hwwps[1].addr == addr
			&& cpu->cores[i].hwwps[1].len == len
			&& cpu->cores[i].hwwps[1].mode == mode)))
	      {
		j = cpu->cores[i].hwwps[0].addr == addr ? 0 : 1;
		cpu->cores[i].hwwps[j].addr = -1;
		cpu->cores[i].hwwps[j].len = 0;
		cpu->cores[i].hwwps[j].mode = WPDA_DISABLE;
		if (!cpu->cores[i].is_locked)
		  wpu_set_wpda (i, j, addr, len, 0, WPDA_DISABLE);
		return RP_VAL_TARGETRET_OK;
	      }
	    else if ((bfin_force_range_wp || len > 4)
		     && cpu->cores[i].hwwps[0].addr == addr
		     && cpu->cores[i].hwwps[0].len == len
		     && cpu->cores[i].hwwps[0].mode == mode
		     && cpu->cores[i].hwwps[1].addr == addr
		     && cpu->cores[i].hwwps[1].len == len
		     && cpu->cores[i].hwwps[1].mode == mode)
	      {
		cpu->cores[i].hwwps[0].addr = -1;
		cpu->cores[i].hwwps[0].len = 0;
		cpu->cores[i].hwwps[0].mode = WPDA_DISABLE;
		cpu->cores[i].hwwps[1].addr = -1;
		cpu->cores[i].hwwps[1].len = 0;
		cpu->cores[i].hwwps[1].mode = WPDA_DISABLE;
		if (!cpu->cores[i].is_locked)
		  wpu_set_wpda (i, 0, addr, len, 1, WPDA_DISABLE);
		return RP_VAL_TARGETRET_OK;
	      }

	    bfin_log (RP_VAL_LOGLEVEL_ERR,
		      "%s: [%d] no hardware watchpoint at 0x%08llX length %d",
		      bfin_target.name, i, addr, len);
	    return RP_VAL_TARGETRET_ERR;
	  }

      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: no core has hardware watchpoint at 0x%08llX length %d",
		bfin_target.name, addr, len);
      return RP_VAL_TARGETRET_ERR;
    }

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (!bfin_force_range_wp
	&& len <= 4
	&& (cpu->cores[i].hwwps[0].addr != addr
	    || cpu->cores[i].hwwps[0].len != len
	    || cpu->cores[i].hwwps[0].mode != mode)
	&& (cpu->cores[i].hwwps[1].addr != addr
	    || cpu->cores[i].hwwps[1].len != len
	    || cpu->cores[i].hwwps[1].mode != mode))
      break;
    else if ((bfin_force_range_wp || len > 4)
	     && (cpu->cores[i].hwwps[0].addr != addr
		 || cpu->cores[i].hwwps[0].len != len
		 || cpu->cores[i].hwwps[0].mode != mode
		 || cpu->cores[i].hwwps[1].addr != addr
		 || cpu->cores[i].hwwps[1].len != len
		 || cpu->cores[i].hwwps[1].mode != mode))
      break;

  if (i < cpu->chain->parts->len)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] no hardware breakpoint at 0x%08llX length %d",
		bfin_target.name, i, addr, len);
      return RP_VAL_TARGETRET_ERR;
    }

  for (i = 0; i < cpu->chain->parts->len; i++)
    if (!bfin_force_range_wp && len <= 4)
      {
	j = (cpu->cores[i].hwwps[0].addr == addr
	     && cpu->cores[i].hwwps[0].len == len
	     && cpu->cores[i].hwwps[0].mode == mode) ? 0 : 1;
	cpu->cores[i].hwwps[j].addr = -1;
	cpu->cores[i].hwwps[j].len = 0;
	cpu->cores[i].hwwps[j].mode = WPDA_DISABLE;
	if (!cpu->cores[i].is_locked)
	  wpu_set_wpda (i, j, addr, len, 0, WPDA_DISABLE);
      }
    else
      {
	cpu->cores[i].hwwps[0].addr = -1;
	cpu->cores[i].hwwps[0].len = 0;
	cpu->cores[i].hwwps[0].mode = WPDA_DISABLE;
	cpu->cores[i].hwwps[1].addr = -1;
	cpu->cores[i].hwwps[1].len = 0;
	cpu->cores[i].hwwps[1].mode = WPDA_DISABLE;
	if (!cpu->cores[i].is_locked)
	  wpu_set_wpda (i, 0, addr, len, 1, WPDA_DISABLE);
      }
  return RP_VAL_TARGETRET_OK;
}
