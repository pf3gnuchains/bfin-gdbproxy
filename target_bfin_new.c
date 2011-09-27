/* Copyright (C) 2008-2010 Analog Devices, Inc.

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

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <stdarg.h>
#include <getopt.h>
#include <sys/param.h>
#if defined(WIN32)
#include <winsock.h>
#endif

#include "gdbproxy.h"
#include "circ_buf.h"
#include "rpmisc.h"

#include <urjtag/cable.h>
#include <urjtag/part.h>
#include <urjtag/part_instruction.h>
#include <urjtag/chain.h>
#include <urjtag/tap.h>
#include <urjtag/tap_state.h>
#include <urjtag/tap_register.h>
#include <urjtag/data_register.h>
#include <urjtag/cmd.h>
#include <urjtag/jtag.h>
#include <urjtag/parse.h>
#include <urjtag/bfin.h>

#ifndef MIN
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif
#ifndef MAX
#define MAX(x,y) ((x) < (y) ? (y) : (x))
#endif

/* MMRs definitions */

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

#define UPDATE				0
#define RUNTEST				1

/* Convert part number to thread ID.
   Core A is assigned the thread ID 1.
   Core B is assigned the thread ID 2.  */
#define THREAD_ID(n) \
  (cpu->first_core + cpu->core_num - (n))
/* Convert thread id to part number.  */
#define PART_NO(n) \
  (cpu->first_core + cpu->core_num - (n))

#define ALL_THREADS	-1
#define ANY_THREAD	0

#define INVALID_CORE	-1
#define ALL_CORES	(cpu->core_num + 1)
#define ANY_CORE	(cpu->core_num)

#define CACHE_DISABLED 0
#define WRITE_THROUGH 1
#define WRITE_BACK 2

#define WPDA_DISABLE			0
#define WPDA_WRITE			1
#define WPDA_READ			2
#define WPDA_ALL			3

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
  REG_RETE,
  -1 /* REG_CC */,
  -1 /* REG_TEXT_ADDR */,
  -1 /* REG_TEXT_END_ADDR */,
  -1 /* REG_DATA_ADDR */,
  -1 /* REG_FDPIC_EXEC_REGNUM */,
  -1 /* REG_FDPIC_INTERP_REGNUM */,

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
			 int status_string_size, int *can_restart);
static int bfin_disconnect (void);
static void bfin_kill (void);
static int bfin_restart (void);
static void bfin_stop (void);
static int bfin_set_gen_thread (rp_thread_ref *thread);
static int bfin_set_ctrl_thread (rp_thread_ref *thread);
static int bfin_is_thread_alive (rp_thread_ref *thread, int *alive);
static int bfin_read_registers (uint8_t *data_buf,
				uint8_t *avail_buf,
				int buf_size, int *read_size);
static int bfin_write_registers (uint8_t *data_buf, int write_size);
static int bfin_read_single_register (unsigned int reg_no,
				      uint8_t *data_buf,
				      uint8_t *avail_buf,
				      int buf_size, int *read_size);
static int bfin_write_single_register (unsigned int reg_no,
				       uint8_t *data_buf, int write_size);
static int bfin_read_mem (uint64_t addr,
			  uint8_t *data_buf,
			  int req_size, int *actual_size);
static int bfin_write_mem (uint64_t addr,
			   uint8_t *data_buf, int req_sise);
static int bfin_resume_from_current (int step, int sig);
static int bfin_resume_from_addr (int step, int sig, uint64_t addr);
static int bfin_go_waiting (int sig);
static int bfin_wait_partial (int first,
			      char *status_string,
			      int status_string_len,
			      out_func out, int *implemented, int *more);
static int bfin_wait (char *status_string,
		      int status_string_len,
		      out_func out, int *implemented);
static int bfin_process_query (unsigned int *mask,
			       rp_thread_ref *arg, rp_thread_info *info);
static int bfin_list_query (int first,
			    rp_thread_ref *arg,
			    rp_thread_ref *result,
			    int max_num, int *num, int *done);
static int bfin_current_thread_query (rp_thread_ref *thread);
static int bfin_offsets_query (uint64_t *text,
			       uint64_t *data, uint64_t *bss);
static int bfin_crc_query (uint64_t addr, int len, uint32_t *val);
static int bfin_raw_query (char *in_buf, char *out_buf, int out_buf_size);
static int bfin_threadinfo_query (int first,
				  char *out_buf, int out_buf_size);
static int bfin_threadextrainfo_query (rp_thread_ref *thread,
				       char *out_buf, int out_buf_size);
static int bfin_supported_query (char *out_buf, int out_buf_size);
static int bfin_add_break (int type, uint64_t addr, int len);
static int bfin_remove_break (int type, uint64_t addr, int len);

static const RCMD_TABLE bfin_remote_commands[];

/* Global target descriptor */
rp_target bfin_target = {
  NULL,
  "bfin",
  "Blackfin JTAG target",
  bfin_remote_commands,
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
  bfin_supported_query,
};
static char default_jtag_connect[] = "cable probe";
static uint32_t bfin_frequency = 0;
static int bfin_processor = -1;
static struct timespec bfin_loop_wait_first_ts = {0, 50000000};
static struct timespec bfin_loop_wait_ts = {0, 10000000};
static int rti_limit = INT_MAX;

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

static const bfin_l1_map bf50x_l1_map = {
  .l1			= 0xff800000,
  .l1_data_a		= 0xff800000,
  .l1_data_a_end	= 0xff804000,
  .l1_data_a_cache	= 0xff804000,
  .l1_data_a_cache_end	= 0xff808000,
  .l1_code		= 0xffa00000,
  .l1_code_end		= 0xffa04000,
  .l1_code_cache	= 0xffa04000,
  .l1_code_cache_end	= 0xffa08000,
  .l1_scratch		= 0xffb00000,
  .l1_scratch_end	= 0xffb01000,
  .l1_end		= 0xffc00000,
};
static const bfin_l1_map bf51x_l1_map = {
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
  .l1_scratch		= 0xffb00000,
  .l1_scratch_end	= 0xffb01000,
  .l1_end		= 0xffc00000,
};
static const bfin_l1_map bf52x_l1_map = {
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
static const bfin_l1_map bf533_l1_map = {
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
static const bfin_l1_map bf537_l1_map = {
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
static const bfin_l1_map bf538_l1_map = {
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
static const bfin_l1_map bf54x_l1_map = {
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
  .l1_code_rom		= 0xffa14000,
  .l1_code_rom_end	= 0xffa24000,
  .l1_scratch		= 0xffb00000,
  .l1_scratch_end	= 0xffb01000,
  .l1_end		= 0xffc00000,
};
static const bfin_l1_map bf561_a_l1_map = {
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
  .l1_code_end		= 0xffa04000,
  .l1_code_cache	= 0xffa10000,
  .l1_code_cache_end	= 0xffa14000,
  .l1_scratch		= 0xffb00000,
  .l1_scratch_end	= 0xffb01000,
  .l1_end		= 0xffc00000,
};
static const bfin_l1_map bf561_b_l1_map = {
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
  .l1_code_end		= 0xff604000,
  .l1_code_cache	= 0xff610000,
  .l1_code_cache_end	= 0xff614000,
  .l1_scratch		= 0xff700000,
  .l1_scratch_end	= 0xff701000,
  .l1_end		= 0xff800000,
};
static const bfin_l1_map bf59x_l1_map = {
  .l1			= 0xff800000,
  .l1_data_a		= 0xff800000,
  .l1_data_a_end	= 0xff808000,
  .l1_code		= 0xffa00000,
  .l1_code_end		= 0xffa04000,
  .l1_code_cache	= 0xffa04000,
  .l1_code_cache_end	= 0xffa08000,
  .l1_scratch		= 0xffb00000,
  .l1_scratch_end	= 0xffb01000,
  .l1_code_rom		= 0xffa10000,
  .l1_code_rom_end	= 0xffa20000,
  .l1_end		= 0xffc00000,
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

static const bfin_mem_map bf50x_mem_map = {
  .sdram		= 0,
  .sdram_end		= 0,
  .async_mem		= 0x20000000,
  .flash		= 0x20000000,
  .flash_end		= 0x20400000,  /* Only valid for BF504F and BF506F.  */
  .async_mem_end	= 0x20400000,  /* Only valid for BF504F and BF506F.  */
  .boot_rom		= 0xef000000,
  .boot_rom_end		= 0xef001000,
  .l1			= 0xff800000,
  .l1_end		= 0xffc00000,
  .sysmmr		= 0xffc00000,
  .coremmr		= 0xffe00000,
};
static const bfin_mem_map bf52x_mem_map = {
  .sdram		= 0,
  .sdram_end		= 0x20000000,
  .async_mem		= 0x20000000,
  .flash		= 0x20000000,
  .flash_end		= 0x20100000,
  .async_mem_end	= 0x20400000,
  .boot_rom		= 0xef000000,
  .boot_rom_end		= 0xef008000,
  .l1			= 0xff800000,
  .l1_end		= 0xffc00000,
  .sysmmr		= 0xffc00000,
  .coremmr		= 0xffe00000,
};
static const bfin_mem_map bf533_mem_map = {
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
static const bfin_mem_map bf537_mem_map = {
  .sdram		= 0,
  .sdram_end		= 0x20000000,
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
static const bfin_mem_map bf538_mem_map = {
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
static const bfin_mem_map bf54x_mem_map = {
  .sdram		= 0,
  .sdram_end		= 0x20000000,
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
static const bfin_mem_map bf561_mem_map = {
  .sdram		= 0,
  .sdram_end		= 0x20000000,
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
static const bfin_mem_map bf59x_mem_map = {
  /* No SDRAM or async */
  .boot_rom		= 0xef000000,
  .boot_rom_end		= 0xef001000,
  .l1			= 0xff800000,
  .l1_end		= 0xffc00000,
  .sysmmr		= 0xffc00000,
  .coremmr		= 0xffe00000,
};

#define IN_RANGE(addr, lo, hi) ((addr) >= (lo) && (addr) < (hi))
#define IN_MAP(addr, map) IN_RANGE (addr, map, map##_end)
#define MAP_LEN(map) ((map##_end) - (map))

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

typedef struct _bfin_test_data
{
  uint32_t command_addr, data0_addr, data1_addr;
  uint32_t data0_off, data1_off;
  uint32_t data0;
  uint32_t data1;
} bfin_test_data;

typedef struct _bfin_hwwps
{
  uint32_t addr;
  int len;
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

  int pending_signal;
  uint32_t pending_stop_pc;

  const bfin_l1_map *l1_map;
  uint32_t registers[BFIN_NUM_REGS];
  uint32_t wpiactl;
  uint32_t wpdactl;
  uint32_t wpstat;

  uint32_t hwbps[RP_BFIN_MAX_HWBREAKPOINTS];
  bfin_hwwps hwwps[RP_BFIN_MAX_HWWATCHPOINTS];
  uint32_t dmem_control;
  uint32_t imem_control;
} bfin_core;

typedef struct _bfin_sdram_config
{
  uint16_t sdrrc;
  uint16_t sdbctl;
  uint32_t sdgctl;
} bfin_sdram_config;

static const bfin_sdram_config bf527_ezkit_sdram_config = {
  .sdrrc = 0x0407,
  .sdbctl = 0x0025,
  .sdgctl = 0x0091998d,
};

static const bfin_sdram_config bf533_ezkit_sdram_config = {
  .sdrrc = 0x01a0,
  .sdbctl = 0x0025,
  .sdgctl = 0x0091998d,
};

static const bfin_sdram_config bf533_stamp_sdram_config = {
  .sdrrc = 0x01a0,
  .sdbctl = 0x0025,
  .sdgctl = 0x0091998d,
};

static const bfin_sdram_config bf537_ezkit_sdram_config = {
  .sdrrc = 0x03a0,
  .sdbctl = 0x0025,
  .sdgctl = 0x0091998d,
};

static const bfin_sdram_config bf538f_ezkit_sdram_config = {
  .sdrrc = 0x03f6,
  .sdbctl = 0x0025,
  .sdgctl = 0x0091998d,
};

static const bfin_sdram_config bf561_ezkit_sdram_config = {
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

static const bfin_ddr_config bf548_ezkit_ddr_config = {
  .ddrctl0 = 0x218A8411,
  .ddrctl1 = 0x20022222,
  .ddrctl2 = 0x00000021,
};

typedef struct _bfin_cpu
{
  urj_chain_t *chain;
  bfin_swbp *swbps;
  bfin_board board;
  bfin_mem_map mem_map;
  const bfin_sdram_config *sdram_config;
  const bfin_ddr_config *ddr_config;
  uint32_t mdma_s0;
  uint32_t mdma_d0;

  /* The first core of this cpu in the chain.  */
  int first_core;
  /* The number of cores of this cpu.  */
  int core_num;

  /* The core will never be locked.  */
  int core_a;

  int general_core;
  int continue_core;

  bfin_core cores[0];
} bfin_cpu;

#define for_each_core(i, c) \
  for (c = &cpu->cores[i = 0]; i < (cpu)->core_num; c = &cpu->cores[++i])

static log_func bfin_log;
static bfin_cpu *cpu = NULL;
static int bfin_force_range_wp = 0;
static int bfin_unlock_on_connect = 0;
static int bfin_unlock_on_load = 0;
static int bfin_auto_switch = 1;
static int bfin_init_sdram = 0;
static int bfin_reset = 0;
static int bfin_enable_dcache = CACHE_DISABLED;
static int bfin_enable_icache = 0;
static int use_dma = 0;

#define INVALID_MEM_ACCESS_IGNORE     0
#define INVALID_MEM_ACCESS_REJECT     1
#define INVALID_MEM_ACCESS_ALLOW_DMA  2
#define INVALID_MEM_ACCESS_ALLOW_CORE 3
static int invalid_mem_access = INVALID_MEM_ACCESS_IGNORE;


/* Local functions */


static void
scan_select (int scan)
{
  chain_scan_select (cpu->chain, scan);
}

static void
core_scan_select (int core, int scan)
{
  part_scan_select (cpu->chain, cpu->first_core + core, scan);
}

#define DBGCTL_CLEAR_OR_SET_BIT(name)					\
  static void								\
  dbgctl_bit_clear_or_set_##name (int runtest, int set)			\
  {									\
    chain_scan_select (cpu->chain, DBGCTL_SCAN);			\
    if (set)								\
      chain_dbgctl_bit_set_##name (cpu->chain);				\
    else								\
      chain_dbgctl_bit_clear_##name (cpu->chain);			\
    urj_tap_chain_shift_data_registers_mode (cpu->chain, 0, 1, runtest ? \
				     URJ_CHAIN_EXITMODE_IDLE : URJ_CHAIN_EXITMODE_UPDATE);	\
  }

#define DBGCTL_SET_BIT(name)						\
  static void								\
  dbgctl_bit_set_##name (int runtest)					\
  {									\
    dbgctl_bit_clear_or_set_##name (runtest, 1);			\
  }

#define DBGCTL_CLEAR_BIT(name)						\
  static void								\
  dbgctl_bit_clear_##name (int runtest)					\
  {									\
    dbgctl_bit_clear_or_set_##name (runtest, 0);			\
  }

#define CORE_DBGCTL_CLEAR_OR_SET_BIT(name)				\
  static void								\
  core_dbgctl_bit_clear_or_set_##name (int core, int runtest, int set)	\
  {									\
    part_scan_select (cpu->chain, cpu->first_core + core, DBGCTL_SCAN);	\
    if (set)								\
      part_dbgctl_bit_set_##name (cpu->chain, cpu->first_core + core);	\
    else								\
      part_dbgctl_bit_clear_##name (cpu->chain, cpu->first_core + core); \
    urj_tap_chain_shift_data_registers_mode (cpu->chain, 0, 1, runtest ? \
				     URJ_CHAIN_EXITMODE_IDLE : URJ_CHAIN_EXITMODE_UPDATE);	\
  }

#define CORE_DBGCTL_SET_BIT(name)					\
  static void								\
  core_dbgctl_bit_set_##name (int core, int runtest)			\
  {									\
    core_dbgctl_bit_clear_or_set_##name (core, runtest, 1);		\
  }

#define CORE_DBGCTL_CLEAR_BIT(name)					\
  static void								\
  core_dbgctl_bit_clear_##name (int core, int runtest)			\
  {									\
    core_dbgctl_bit_clear_or_set_##name (core, runtest, 0);		\
  }

#define CORE_DBGCTL_IS(name)						\
  static int								\
  core_dbgctl_is_##name (int core)					\
  {									\
    return part_dbgctl_is_##name (cpu->chain, cpu->first_core + core);	\
  }

#define DBGCTL_BIT_OP(name)						\
  DBGCTL_CLEAR_OR_SET_BIT(name)						\
  DBGCTL_SET_BIT(name)							\
  DBGCTL_CLEAR_BIT(name)						\
  CORE_DBGCTL_CLEAR_OR_SET_BIT(name)					\
  CORE_DBGCTL_SET_BIT(name)						\
  CORE_DBGCTL_CLEAR_BIT(name)						\
  CORE_DBGCTL_IS(name)

DBGCTL_BIT_OP (sram_init)
DBGCTL_BIT_OP (wakeup)
DBGCTL_BIT_OP (sysrst)
DBGCTL_BIT_OP (esstep)
DBGCTL_BIT_OP (emudatsz_32)
DBGCTL_BIT_OP (emudatsz_40)
DBGCTL_BIT_OP (emudatsz_48)
DBGCTL_BIT_OP (emuirlpsz_2)
DBGCTL_BIT_OP (emuirsz_64)
DBGCTL_BIT_OP (emuirsz_48)
DBGCTL_BIT_OP (emuirsz_32)
DBGCTL_BIT_OP (empen)
DBGCTL_BIT_OP (emeen)
DBGCTL_BIT_OP (emfen)
DBGCTL_BIT_OP (empwr)

/* These functions check cached DBGSTAT. So before calling them,
   dbgstat_get or core_dbgstat_get has to be called to update cached
   DBGSTAT value.  */

#define CORE_DBGSTAT_BIT_IS(name)					\
  static int								\
  core_dbgstat_is_##name (int core)					\
  {									\
    return part_dbgstat_is_##name (cpu->chain, cpu->first_core + core);	\
  }

CORE_DBGSTAT_BIT_IS (lpdec1)
CORE_DBGSTAT_BIT_IS (in_powrgate)
CORE_DBGSTAT_BIT_IS (core_fault)
CORE_DBGSTAT_BIT_IS (idle)
CORE_DBGSTAT_BIT_IS (in_reset)
CORE_DBGSTAT_BIT_IS (lpdec0)
CORE_DBGSTAT_BIT_IS (bist_done)
CORE_DBGSTAT_BIT_IS (emuack)
CORE_DBGSTAT_BIT_IS (emuready)
CORE_DBGSTAT_BIT_IS (emudiovf)
CORE_DBGSTAT_BIT_IS (emudoovf)
CORE_DBGSTAT_BIT_IS (emudif)
CORE_DBGSTAT_BIT_IS (emudof)

static uint16_t
core_dbgstat_emucause (int core)
{
  return part_dbgstat_emucause (cpu->chain, cpu->first_core + core);
}

static void
dbgstat_get (void)
{
  chain_dbgstat_get (cpu->chain);
}

static void
core_dbgstat_get (int core)
{
  part_dbgstat_get (cpu->chain, cpu->first_core + core);
}

static void
emupc_get (int save)
{
  chain_emupc_get (cpu->chain, save);
}

static uint32_t
core_emupc_get (int core, int save)
{
  return part_emupc_get (cpu->chain, cpu->first_core + core, save);
}

static void
emupc_show (const char *id)
{
  urj_part_t *part;
  int i;

  chain_emupc_get (cpu->chain, 0);
  for (i = 0; i < cpu->core_num; i++)
    {
      part = cpu->chain->parts->parts[cpu->first_core + i];
      bfin_log (RP_VAL_LOGLEVEL_DEBUG, "[%d] EMUPC [0x%08x] <%s>",
		cpu->first_core + i, BFIN_PART_EMUPC (part), id);
    }
}

static void
core_emupc_show (int core, const char *id)
{
  urj_part_t *part;
  part_emupc_get (cpu->chain, cpu->first_core + core, 0);
  part = cpu->chain->parts->parts[cpu->first_core + core];
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "[%d] EMUPC [0x%08x] <%s>",
	    cpu->first_core + core, BFIN_PART_EMUPC (part), id);
}

static void
emupc_reset (void)
{
  uint32_t new_pc[cpu->chain->parts->len];
  bfin_core *c;
  int i;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "Reset EMUPC");

  for_each_core (i, c)
    new_pc[i + cpu->first_core] = c->l1_map->l1_code;

  emupc_show ("before");
  chain_emupc_reset (cpu->chain, new_pc);
  emupc_show ("after");
}

static void
dbgstat_clear_ovfs (void)
{
  chain_dbgstat_clear_ovfs (cpu->chain);
}

static void
core_dbgstat_clear_ovfs (int core)
{
  part_dbgstat_clear_ovfs (cpu->chain, cpu->first_core + core);
}

static void
dbgstat_show (const char *id)
{
  urj_part_t *part;
  char buf[1024];
  int i;

  assert (id != NULL);

  chain_dbgstat_get (cpu->chain);
  for (i = 0; i < cpu->core_num; i++) {
    part = cpu->chain->parts->parts[cpu->first_core + i];
    sprintf (buf, "[%d] DBGSTAT [0x%04X]:", cpu->first_core + i, BFIN_PART_DBGSTAT (part));
    if (core_dbgstat_is_lpdec1 (i))     strcat (buf, " lpdec1");
    if (core_dbgstat_is_core_fault (i)) strcat (buf, " core_fault");
    if (core_dbgstat_is_idle (i))       strcat (buf, " idle");
    if (core_dbgstat_is_in_reset (i))   strcat (buf, " in_reset");
    if (core_dbgstat_is_lpdec0 (i))     strcat (buf, " lpdec0");
    if (core_dbgstat_is_bist_done (i))  strcat (buf, " bist_done");
    if (core_dbgstat_is_emuack (i))     strcat (buf, " emuack");
    switch (core_dbgstat_emucause (i)) {
      case 0x0: strcat (buf, " cause:emuexcpt");   break;
      case 0x1: strcat (buf, " cause:emuin");      break;
      case 0x2: strcat (buf, " cause:watchpoint"); break;
      case 0x4: strcat (buf, " cause:perfmon0");   break;
      case 0x5: strcat (buf, " cause:perfmon1");   break;
      case 0x8: strcat (buf, " cause:emu-sstep");  break;
      default:  strcat (buf, " cause:unknown");    break;
    }
    if (core_dbgstat_is_emuready (i))   strcat (buf, " emuready");
    if (core_dbgstat_is_emudiovf (i))   strcat (buf, " emudiovf");
    if (core_dbgstat_is_emudoovf (i))   strcat (buf, " emudoovf");
    if (core_dbgstat_is_emudif (i))     strcat (buf, " emudif");
    if (core_dbgstat_is_emudof (i))     strcat (buf, " emudof");
    strcat (buf, " <");
    strcat (buf, id);
    strcat (buf, ">");
    bfin_log (RP_VAL_LOGLEVEL_DEBUG, buf);
  }
}

static void
core_dbgstat_show (int core, const char *id)
{
  urj_part_t *part;

  assert (id != NULL);

  part_dbgstat_get (cpu->chain, cpu->first_core + core);
  part = cpu->chain->parts->parts[cpu->first_core + core];
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "[%d] DBGSTAT [0x%04X] <%s>",
	    cpu->first_core + core, BFIN_PART_DBGSTAT (part), id);
}

static void
dbgctl_show (const char *id)
{
  urj_part_t *part;
  char buf[1024];
  int i;

  assert (id != NULL);

  for (i = 0; i < cpu->core_num; ++i) {
    part = cpu->chain->parts->parts[cpu->first_core + i];
    sprintf (buf, "[%i] DBGCTL [0x%04x]:", cpu->first_core + i,
	     BFIN_PART_DBGCTL (part));
    if (core_dbgctl_is_sram_init (i))   strcat (buf, " sram_init");
    if (core_dbgctl_is_wakeup (i))      strcat (buf, " wakeup");
    if (core_dbgctl_is_sysrst (i))      strcat (buf, " sysrst");
    if (core_dbgctl_is_esstep (i))      strcat (buf, " esstep");
    if (core_dbgctl_is_emudatsz_32 (i)) strcat (buf, " emudatsz_32");
    if (core_dbgctl_is_emudatsz_40 (i)) strcat (buf, " emudatsz_40");
    if (core_dbgctl_is_emudatsz_48 (i)) strcat (buf, " emudatsz_48");
    if (core_dbgctl_is_emuirlpsz_2 (i)) strcat (buf, " emuirlpsz_2");
    if (core_dbgctl_is_emuirsz_64 (i))  strcat (buf, " emuirsz_64");
    if (core_dbgctl_is_emuirsz_48 (i))  strcat (buf, " emuirsz_48");
    if (core_dbgctl_is_emuirsz_32 (i))  strcat (buf, " emuirsz_32");
    if (core_dbgctl_is_empen (i))       strcat (buf, " empen");
    if (core_dbgctl_is_emeen (i))       strcat (buf, " emeen");
    if (core_dbgctl_is_emfen (i))       strcat (buf, " emfen");
    if (core_dbgctl_is_empwr (i))       strcat (buf, " empwr");
    strcat (buf, " <");
    strcat (buf, id);
    strcat (buf, ">");
    bfin_log (RP_VAL_LOGLEVEL_DEBUG, buf);
  }
}

static void
check_emuready (void)
{
  chain_check_emuready (cpu->chain);
}

static void
core_check_emuready (int core)
{
  part_check_emuready (cpu->chain, cpu->first_core + core);
}

static int
core_sticky_in_reset (int core)
{
  return part_sticky_in_reset (cpu->chain, cpu->first_core + core);
}

static void
wait_in_reset (void)
{
  chain_wait_in_reset (cpu->chain);
}

static void
core_wait_in_reset (int core)
{
  part_wait_in_reset (cpu->chain, cpu->first_core + core);
}

static void
wait_reset (void)
{
  chain_wait_reset (cpu->chain);
}

static void
core_wait_reset (int core)
{
  part_wait_reset (cpu->chain, cpu->first_core + core);
}

static void
emuir_set_same (uint64_t insn, int runtest)
{
  chain_emuir_set_same (cpu->chain, insn,
			runtest ? URJ_CHAIN_EXITMODE_IDLE : URJ_CHAIN_EXITMODE_UPDATE);
}

static void
core_emuir_set (int core, uint64_t insn, int runtest)
{
  part_emuir_set (cpu->chain, cpu->first_core + core, insn,
		  runtest ? URJ_CHAIN_EXITMODE_IDLE : URJ_CHAIN_EXITMODE_UPDATE);
}

static void
emuir_set_same_2 (uint64_t insn1, uint64_t insn2, int runtest)
{
  chain_emuir_set_same_2 (cpu->chain, insn1, insn2,
			  runtest ? URJ_CHAIN_EXITMODE_IDLE : URJ_CHAIN_EXITMODE_UPDATE);
}

static void
core_emuir_set_2 (int core, uint64_t insn1, uint64_t insn2, int runtest)
{
  part_emuir_set_2 (cpu->chain, cpu->first_core + core, insn1, insn2,
		    runtest ? URJ_CHAIN_EXITMODE_IDLE : URJ_CHAIN_EXITMODE_UPDATE);
}

static void
emudat_clear_emudif (urj_tap_register_t *r)
{
  /* If the register size is larger than 32 bits, clear EMUDIF.  */
  if (r->len == 34 || r->len == 40 || r->len == 48)
    r->data[33] = 0;
}

/* These two emudat functions only care the payload data, which is the
   upper 32 bits.  Then follows EMUDOF and EMUDIF if the register size
   is larger than 32 bits.  The remaining is reserved or don't care
   bits.  */

static uint32_t
core_emudat_get (int core, int runtest)
{
  return part_emudat_get (cpu->chain, cpu->first_core + core,
			  runtest ? URJ_CHAIN_EXITMODE_IDLE : URJ_CHAIN_EXITMODE_UPDATE);
}

static void
core_emudat_defer_get (int core, int runtest)
{
  part_emudat_defer_get (cpu->chain, cpu->first_core + core,
			 runtest ? URJ_CHAIN_EXITMODE_IDLE : URJ_CHAIN_EXITMODE_UPDATE);
}

static uint32_t
core_emudat_get_done (int core, int runtest)
{
  return part_emudat_get_done (cpu->chain, cpu->first_core + core,
			       runtest ? URJ_CHAIN_EXITMODE_IDLE : URJ_CHAIN_EXITMODE_UPDATE);
}

static void
core_emudat_set (int core, uint32_t value, int runtest)
{
  part_emudat_set (cpu->chain, cpu->first_core + core, value,
		   runtest ? URJ_CHAIN_EXITMODE_IDLE : URJ_CHAIN_EXITMODE_UPDATE);
}

/* Forward declarations */
static void register_set (enum core_regnum reg, uint32_t *value);
static void core_register_set (int core, enum core_regnum reg,
			       uint32_t value);

static void
register_get (enum core_regnum reg, uint32_t *value)
{
  chain_register_get (cpu->chain, reg, value);
}

static uint32_t
core_register_get (int core, enum core_regnum reg)
{
  return part_register_get (cpu->chain, cpu->first_core + core, reg);
}

static void
register_set (enum core_regnum reg, uint32_t *value)
{
  chain_register_set (cpu->chain, reg, value);
}

static void
register_set_same (enum core_regnum reg, uint32_t value)
{
  chain_register_set_same (cpu->chain, reg, value);
}

static void
core_register_set (int core, enum core_regnum reg, uint32_t value)
{
  part_register_set (cpu->chain, cpu->first_core + core, reg, value);
}

static void
wpu_init (void)
{
  uint32_t *p0, *r0;
  uint32_t wpiactl, wpdactl;
  bfin_core *c;
  int i;

  p0 = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));
  r0 = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));

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

  for_each_core (i, c)
    {
      c->wpiactl = wpiactl;
      c->wpdactl = wpdactl;
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
  bfin_core *c;
  int i;

  p0 = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));
  r0 = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));
  r0new = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));

  if (!p0 || !r0 || !r0new)
    abort ();

  register_get (REG_P0, p0);
  register_get (REG_R0, r0);

  register_set_same (REG_P0, WPIACTL);
  for_each_core (i, c)
    {
      c->wpiactl |= WPIACTL_WPPWR;
      r0new[i] = c->wpdactl;
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
  bfin_core *c;
  int i;

  p0 = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));
  r0 = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));
  r0new = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));

  if (!p0 || !r0 || !r0new)
    abort ();

  register_get (REG_P0, p0);
  register_get (REG_R0, r0);

  register_set_same (REG_P0, WPIACTL);
  for_each_core (i, c)
    {
      c->wpiactl &= ~WPIACTL_WPPWR;
      r0new[i] = c->wpdactl;
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
  bfin_core *c;
  int i;

  p0 = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));
  r0 = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));
  wpstat = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));

  if (!p0 || !r0 || !wpstat)
    abort ();

  register_get (REG_P0, p0);
  register_get (REG_R0, r0);

  register_set_same (REG_P0, WPSTAT);
  emuir_set_same (gen_load32_offset (REG_R0, REG_P0, 0), RUNTEST);
  register_get (REG_R0, wpstat);
  for_each_core (i, c)
    c->wpstat = wpstat[i];

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
  bfin_core *c;
  int i;

  p0 = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));
  r0 = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));
  wpstat = (uint32_t *) malloc (cpu->core_num * sizeof (uint32_t));

  if (!p0 || !r0 || !wpstat)
    abort ();

  register_get (REG_P0, p0);
  register_get (REG_R0, r0);

  register_set_same (REG_P0, WPSTAT);

  for_each_core (i, c)
    wpstat[i] = c->wpstat;
  register_set (REG_R0, wpstat);

  emuir_set_same (gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);

  for_each_core (i, c)
    c->wpstat = 0;

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

  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "[%d] clear wpstat", cpu->first_core + core);

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, WPSTAT);
  core_register_set (core, REG_R0, cpu->cores[core].wpstat);
  core_emuir_set (core, gen_store32_offset (REG_P0, 0, REG_R0), RUNTEST);
  cpu->cores[core].wpstat = 0;

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);

  core_wpstat_get (core);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "[%d]   WPSTAT [0x%08X]",
	    cpu->first_core + core, cpu->cores[core].wpstat);

  return cpu->cores[core].wpstat;
}

static void
emulation_enable (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: emulation_enable ()", bfin_target.name);

  dbgstat_show ("before");
  chain_emulation_enable (cpu->chain);
  dbgstat_show ("after");

  dbgctl_show ("emulation_enable");
}

static void
core_emulation_enable (int core)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_emulation_enable ()",
	    bfin_target.name, cpu->first_core + core);

  core_dbgstat_show (core, "before");
  part_emulation_enable (cpu->chain, cpu->first_core + core);
  core_dbgstat_show (core, "after");
}

static void
emulation_trigger (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: emulation_trigger ()", bfin_target.name);

  dbgstat_show ("before");
  chain_emulation_trigger (cpu->chain);
  dbgstat_show ("after");
}

static void
core_emulation_trigger (int core)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_emulation_trigger ()",
	    bfin_target.name, cpu->first_core + core);

  core_dbgstat_show (core, "before");
  part_emulation_trigger (cpu->chain, cpu->first_core + core);
  core_dbgstat_show (core, "after");
}

static void
emulation_return (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: emulation_return ()", bfin_target.name);

  dbgstat_show ("before");
  chain_emulation_return (cpu->chain);
  dbgstat_show ("after");
}

static void
core_emulation_return (int core)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_emulation_return ()",
	    bfin_target.name, cpu->first_core + core);

  core_dbgstat_show (core, "before");
  part_emulation_return (cpu->chain, cpu->first_core + core);
  core_dbgstat_show (core, "after");
}

static void
emulation_disable (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: emulation_disable ()", bfin_target.name);

  chain_emulation_disable (cpu->chain);
}

static void
core_emulation_disable (int core)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_emulation_disable ()",
	    bfin_target.name, cpu->first_core + core);

  part_emulation_disable (cpu->chain, cpu->first_core + core);
}

static void
system_reset (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "Reset system");

  chain_system_reset (cpu->chain);
}

static void
core_reset (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "Reset core(s)");

  bfin_core_reset (cpu->chain);
}

static uint32_t
mmr_read_clobber_r0 (int core, int32_t offset, int size)
{
  return part_mmr_read_clobber_r0 (cpu->chain, cpu->first_core + core, offset, size);
}

static uint32_t
mmr_read (int core, uint32_t addr, int size)
{
  uint32_t value;

  if (addr == DMEM_CONTROL)
    {
      if (size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] DMEM_CONTROL can only be accessed as 32-bit word",
		    bfin_target.name, cpu->first_core + core);
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
		    bfin_target.name, cpu->first_core + core);
	  /* Return a weird value to notice people.  */
	  return 0xfffffff;
	}

      if (cpu->cores[core].imem_control_valid_p)
	return cpu->cores[core].imem_control;
    }

  value = part_mmr_read (cpu->chain, cpu->first_core + core, addr, size);

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
mmr_write_clobber_r0 (int core, int32_t offset, uint32_t data, int size)
{
  part_mmr_write_clobber_r0 (cpu->chain, cpu->first_core + core, offset, data, size);
}

static void
mmr_write (int core, uint32_t addr, uint32_t data, int size)
{
  if (addr == DMEM_CONTROL)
    {
      if (size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] DMEM_CONTROL can only be accessed as 32-bit word",
		    bfin_target.name, cpu->first_core + core);
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
		    bfin_target.name, cpu->first_core + core);
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

  part_mmr_write (cpu->chain, cpu->first_core + core, addr, data, size);
}

static int
bfin_sdram_init (int core)
{
  uint32_t p0, r0;
  uint32_t value;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_sdram_init ()", bfin_target.name);

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  core_register_set (core, REG_P0, EBIU_SDGCTL);

  /* Check if SDRAM has been enabled already.
     If so, don't enable it again.  */
  value = mmr_read_clobber_r0 (core, EBIU_SDSTAT - EBIU_SDGCTL, 2);
  if ((value & 0x8) == 0)
    {
      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: sdram has already been enabled", bfin_target.name);
      return 0;
    }

  mmr_write_clobber_r0 (core, EBIU_SDRRC - EBIU_SDGCTL,
			cpu->sdram_config->sdrrc, 2);
  mmr_write_clobber_r0 (core, EBIU_SDBCTL - EBIU_SDGCTL,
			cpu->sdram_config->sdbctl, 2);
  mmr_write_clobber_r0 (core, 0, cpu->sdram_config->sdgctl, 4);
  core_emuir_set (core, INSN_SSYNC, RUNTEST);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
  return 0;
}

static int
sdram_init (void)
{
  bfin_core *c;
  int i;

  for_each_core (i, c)
    if (!c->is_locked && !c->is_corefault && !c->is_running)
      break;

  if (i == cpu->core_num)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: no core available to init sdram",
		bfin_target.name);
      return -1;
    }
  else
    return bfin_sdram_init (i);
}

static int
ddr_init (void)
{
  uint32_t p0, r0;
  uint32_t value;
  bfin_core *c;
  int i;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG2,
	    "%s: ddr_init ()", bfin_target.name);

  for_each_core (i, c)
    if (!c->is_locked && !c->is_corefault && !c->is_running)
      break;

  if (i == cpu->core_num)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: no core available to init ddr",
		bfin_target.name);
      return -1;
    }

  p0 = core_register_get (i, REG_P0);
  r0 = core_register_get (i, REG_R0);

  core_register_set (i, REG_P0, EBIU_DDRCTL0);

  value = mmr_read_clobber_r0 (i, EBIU_RSTCTL - EBIU_DDRCTL0, 2);
  mmr_write_clobber_r0 (i, EBIU_RSTCTL - EBIU_DDRCTL0, value | 0x1, 2);
  core_emuir_set (i, INSN_SSYNC, RUNTEST);

  mmr_write_clobber_r0 (i, 0, cpu->ddr_config->ddrctl0, 4);
  mmr_write_clobber_r0 (i, EBIU_DDRCTL1 - EBIU_DDRCTL0,
			cpu->ddr_config->ddrctl1, 4);
  mmr_write_clobber_r0 (i, EBIU_DDRCTL2 - EBIU_DDRCTL0,
			cpu->ddr_config->ddrctl2, 4);
  core_emuir_set (i, INSN_SSYNC, RUNTEST);

  core_register_set (i, REG_P0, p0);
  core_register_set (i, REG_R0, r0);
  return 0;
}

static void
core_dcplb_enable_clobber_p0r0 (int core)
{
  core_register_set (core, REG_P0, DMEM_CONTROL);

  if (!cpu->cores[core].dmem_control_valid_p)
    {
      core_emuir_set (core, INSN_CSYNC, RUNTEST);
      cpu->cores[core].dmem_control = mmr_read_clobber_r0 (core, 0, 4);
      cpu->cores[core].dmem_control_valid_p = 1;
    }

  if (cpu->cores[core].dmem_control & ENDCPLB)
    return;

  cpu->cores[core].dmem_control |= ENDCPLB;
  mmr_write_clobber_r0 (core, 0, cpu->cores[core].dmem_control, 4);
  core_emuir_set (core, INSN_SSYNC, RUNTEST);
}

/* Disable DCPLB if it's enabled.  Return zero if DCPLB was not enabled originally.
   Othersize return non-zero.  */
static int
core_dcplb_disable_clobber_p0r0 (int core)
{
  int orig;

  core_register_set (core, REG_P0, DMEM_CONTROL);

  if (!cpu->cores[core].dmem_control_valid_p)
    {
      core_emuir_set (core, INSN_CSYNC, RUNTEST);
      cpu->cores[core].dmem_control = mmr_read_clobber_r0 (core, 0, 4);
      cpu->cores[core].dmem_control_valid_p = 1;
    }

  orig = cpu->cores[core].dmem_control & ENDCPLB;

  if (orig)
    {
      cpu->cores[core].dmem_control &= ~ENDCPLB;
      mmr_write_clobber_r0 (core, 0, cpu->cores[core].dmem_control, 4);
      core_emuir_set (core, INSN_SSYNC, RUNTEST);
    }

  return orig;
}

static void
core_dcplb_set_clobber_p0r0 (int core, bfin_cplb_entry *dcplbs)
{
  int i;

  core_register_set (core, REG_P0, DCPLB_ADDR0);
  core_dbgctl_bit_set_emuirlpsz_2 (core, UPDATE);
  core_emuir_set_2 (core,
		    gen_move (REG_R0, REG_EMUDAT),
		    gen_store32pi (REG_P0, REG_R0), UPDATE);
  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    core_emudat_set (core, dcplbs[i].addr, RUNTEST);
  core_dbgctl_bit_clear_emuirlpsz_2 (core, UPDATE);

  core_register_set (core, REG_P0, DCPLB_DATA0);
  core_dbgctl_bit_set_emuirlpsz_2 (core, UPDATE);
  core_emuir_set_2 (core,
		    gen_move (REG_R0, REG_EMUDAT),
		    gen_store32pi (REG_P0, REG_R0), UPDATE);
  for (i = 0; i < BFIN_DCPLB_NUM; i++)
    core_emudat_set (core, dcplbs[i].data, RUNTEST);
  core_dbgctl_bit_clear_emuirlpsz_2 (core, UPDATE);
}

static void
core_icplb_set_clobber_p0r0 (int core, bfin_cplb_entry *icplbs)
{
  int i;

  core_register_set (core, REG_P0, ICPLB_ADDR0);
  core_dbgctl_bit_set_emuirlpsz_2 (core, UPDATE);
  core_emuir_set_2 (core,
		    gen_move (REG_R0, REG_EMUDAT),
		    gen_store32pi (REG_P0, REG_R0), UPDATE);
  for (i = 0; i < BFIN_ICPLB_NUM; i++)
    core_emudat_set (core, icplbs[i].addr, RUNTEST);
  core_dbgctl_bit_clear_emuirlpsz_2 (core, UPDATE);

  core_register_set (core, REG_P0, ICPLB_DATA0);
  core_dbgctl_bit_set_emuirlpsz_2 (core, UPDATE);
  core_emuir_set_2 (core,
		    gen_move (REG_R0, REG_EMUDAT),
		    gen_store32pi (REG_P0, REG_R0), UPDATE);
  for (i = 0; i < BFIN_ICPLB_NUM; i++)
    core_emudat_set (core, icplbs[i].data, RUNTEST);
  core_dbgctl_bit_clear_emuirlpsz_2 (core, UPDATE);
}

static void
core_dcache_enable (int core, int method)
{
  bfin_cplb_entry dcplbs[BFIN_DCPLB_NUM];
  uint32_t p0, r0;
  int i, j;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  if (!cpu->cores[core].dmem_control_valid_p)
    {
      core_register_set (core, REG_P0, DMEM_CONTROL);
      core_emuir_set (core, INSN_CSYNC, RUNTEST);
      cpu->cores[core].dmem_control = mmr_read_clobber_r0 (core, 0, 4);
      cpu->cores[core].dmem_control_valid_p = 1;
    }
  if (cpu->cores[core].dmem_control & ENDCPLB)
    {
      core_register_set (core, REG_P0, p0);
      core_register_set (core, REG_R0, r0);
      return;
    }

  i = 0;
  if (cpu->cores[core].l1_map->l1_data_a)
    {
      dcplbs[i].addr = cpu->cores[core].l1_map->l1_data_a;
      dcplbs[i].data = L1_DMEMORY;
      i++;
    }
  if (cpu->cores[core].l1_map->l1_data_b)
    {
      dcplbs[i].addr = cpu->cores[core].l1_map->l1_data_b;
      dcplbs[i].data = L1_DMEMORY;
      i++;
    }

  dcplbs[i].addr = cpu->mem_map.flash;
  dcplbs[i].data = SDRAM_DNON_CHBL;
  i++;
  
  j = i;
  for (; i < 16; i++)
    {
      if ((i - j) * 4 * 1024 * 1024 >= cpu->mem_map.sdram_end)
	break;
      dcplbs[i].addr = cpu->mem_map.sdram + (i - j) * 4 * 1024 * 1024;
      if (method == WRITE_THROUGH)
	dcplbs[i].data = SDRAM_DGEN_WT;
      else if (method == WRITE_BACK)
	dcplbs[i].data = SDRAM_DGEN_WB;
      else
	abort ();
    }

  core_register_set (core, REG_P0, DTEST_COMMAND);
  mmr_write_clobber_r0 (core, 0, 0, 4);
  core_emuir_set (core, INSN_CSYNC, RUNTEST);

  core_dcplb_set_clobber_p0r0 (core, dcplbs);

  cpu->cores[core].dmem_control = ACACHE_BCACHE | ENDCPLB;
  core_register_set (core, REG_P0, DMEM_CONTROL);
  core_emuir_set (core, INSN_CSYNC, RUNTEST);
  mmr_write_clobber_r0 (core, 0, cpu->cores[core].dmem_control, 4);
  core_emuir_set (core, INSN_SSYNC, RUNTEST);
  cpu->cores[core].dmem_control_valid_p = 1;

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
}

static void
dcache_enable (int method)
{
  bfin_core *c;
  int i;

  for_each_core (i, c)
    if (c->is_locked || c->is_corefault || c->is_running)
      {
	c->l1_data_a_cache_enabled = 0;
	c->l1_data_b_cache_enabled = 0;
      }
    else
      {
	core_dcache_enable (i, method);
	c->l1_data_a_cache_enabled = 1;
	c->l1_data_b_cache_enabled = 1;
      }
}

static void
core_icache_enable (int core)
{
  bfin_cplb_entry icplbs[BFIN_ICPLB_NUM];
  uint32_t p0, r0;
  int i, j;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  if (!cpu->cores[core].imem_control_valid_p)
    {
      core_register_set (core, REG_P0, IMEM_CONTROL);
      core_emuir_set (core, INSN_CSYNC, RUNTEST);
      cpu->cores[core].imem_control = mmr_read_clobber_r0 (core, 0, 4);
      cpu->cores[core].imem_control_valid_p = 1;
    }
  if (cpu->cores[core].imem_control & ENICPLB)
    {
      core_register_set (core, REG_P0, p0);
      core_register_set (core, REG_R0, r0);
      return;
    }

  i = 0;
  if (cpu->cores[core].l1_map->l1_code)
    {
      icplbs[i].addr = cpu->cores[core].l1_map->l1_code;
      icplbs[i].data = L1_IMEMORY;
      i++;
    }

  icplbs[i].addr = cpu->mem_map.flash;
  icplbs[i].data = SDRAM_INON_CHBL;
  i++;

  j = i;
  for (; i < 16; i++)
    {
      if ((i - j) * 4 * 1024 * 1024 >= cpu->mem_map.sdram_end)
	break;
      icplbs[i].addr = cpu->mem_map.sdram + (i - j) * 4 * 1024 * 1024;
      icplbs[i].data = SDRAM_IGENERIC;
    }

  core_register_set (core, REG_P0, ITEST_COMMAND);
  mmr_write_clobber_r0 (core, 0, 0, 4);
  core_emuir_set (core, INSN_CSYNC, RUNTEST);

  core_icplb_set_clobber_p0r0 (core, icplbs);

  cpu->cores[core].imem_control = IMC | ENICPLB;
  core_register_set (core, REG_P0, IMEM_CONTROL);
  core_emuir_set (core, INSN_CSYNC, RUNTEST);
  mmr_write_clobber_r0 (core, 0, cpu->cores[core].imem_control, 4);
  core_emuir_set (core, INSN_SSYNC, RUNTEST);
  cpu->cores[core].imem_control_valid_p = 1;

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);
}

static void
icache_enable (void)
{
  bfin_core *c;
  int i;

  for_each_core (i, c)
    if (c->is_locked || c->is_corefault || c->is_running)
      {
	c->l1_code_cache_enabled = 0;
      }
    else
      {
	core_icache_enable (i);
	c->l1_code_cache_enabled = 1;
      }
}

static void
icache_flush (int core, uint32_t addr, int size)
{
  uint32_t p0, r0;
  int i;

  assert (size > 0);

  p0 = core_register_get (core, REG_P0);

  if (!cpu->cores[core].imem_control_valid_p)
    {
      r0 = core_register_get (core, REG_R0);
      core_register_set (core, REG_P0, IMEM_CONTROL);
      core_emuir_set (core, INSN_CSYNC, RUNTEST);
      cpu->cores[core].imem_control = mmr_read_clobber_r0 (core, 0, 4);
      cpu->cores[core].imem_control_valid_p = 1;
      core_register_set (core, REG_R0, r0);
    }

  if (cpu->cores[core].imem_control & IMC)
    {
      core_register_set (core, REG_P0, addr);
      for (i = (size + addr % CACHE_LINE_BYTES - 1) / CACHE_LINE_BYTES + 1;
	   i > 0; i--)
	core_emuir_set (core, gen_iflush_pm (REG_P0), RUNTEST);
    }

  core_register_set (core, REG_P0, p0);
}

static int
memory_read_1 (int core, uint32_t addr, uint8_t *buf, int size)
{
  uint32_t p0, r0;
  int count1 = 0, count2 = 0, count3 = 0;
  int dcplb_enabled;

  assert (size > 0);

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  dcplb_enabled = core_dcplb_disable_clobber_p0r0 (core);

  core_register_set (core, REG_P0, addr);

  core_dbgctl_bit_set_emuirlpsz_2 (core, UPDATE);

  if ((addr & 0x3) != 0)
    core_emuir_set_2 (core,
		      gen_load8zpi (REG_R0, REG_P0),
		      gen_move (REG_EMUDAT, REG_R0), UPDATE);

  while ((addr & 0x3) != 0 && size != 0)
    {
      core_emudat_defer_get (core, RUNTEST);
      addr++;
      size--;
      count1++;
    }
  if (size == 0)
    goto finish_read;

  if (size >= 4)
    core_emuir_set_2 (core,
		      gen_load32pi (REG_R0, REG_P0),
		      gen_move (REG_EMUDAT, REG_R0), UPDATE);

  for (; size >= 4; size -= 4)
    {
      core_emudat_defer_get (core, RUNTEST);
      count2++;
    }

  if (size == 0)
    goto finish_read;

  core_emuir_set_2 (core,
		    gen_load8zpi (REG_R0, REG_P0),
		    gen_move (REG_EMUDAT, REG_R0), UPDATE);

  for (; size > 0; size--)
    {
      core_emudat_defer_get (core, RUNTEST);
      count3++;
    }

finish_read:

  while (count1 > 0)
    {
      *buf++ = core_emudat_get_done (core, RUNTEST);
      count1--;
    }

  while (count2 > 0)
    {
      uint32_t data;

      data = core_emudat_get_done (core, RUNTEST);
      *buf++ = data & 0xff;
      *buf++ = (data >> 8) & 0xff;
      *buf++ = (data >> 16) & 0xff;
      *buf++ = (data >> 24) & 0xff;
      count2--;
    }

  while (count3 > 0)
    {
      *buf++ = core_emudat_get_done (core, RUNTEST);
      count3--;
    }

  core_dbgctl_bit_clear_emuirlpsz_2 (core, UPDATE);

  if (dcplb_enabled)
    core_dcplb_enable_clobber_p0r0 (core);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);

  return 0;
}

static int
memory_read (int core, uint32_t addr, uint8_t *buf, int size)
{
  int s;

  if ((addr & 0x3) != 0)
    {
      s = 4 - (addr & 0x3);
      s = size < s ? size : s;
      memory_read_1 (core, addr, buf, s);
      size -= s;
      addr += s;
      buf += s;
    }

  while (size > 0)
    {
      emupc_reset ();

      /* The overhead should be no larger than 0x20.  */
      s = (rti_limit - 0x20) * 4;
      s = size < s ? size : s;
      memory_read_1 (core, addr, buf, s);
      size -= s;
      addr += s;
      buf += s;
    }

  return 0;
}

static void
dma_context_save_clobber_p0r0 (int core, uint32_t base, bfin_dma *dma)
{
  core_register_set (core, REG_P0, base);

  dma->start_addr = mmr_read_clobber_r0 (core, 0x04, 4);
  dma->config = mmr_read_clobber_r0 (core, 0x08, 2);
  dma->x_count = mmr_read_clobber_r0 (core, 0x10, 2);
  dma->x_modify = mmr_read_clobber_r0 (core, 0x14, 2);
  dma->irq_status = mmr_read_clobber_r0 (core, 0x28, 2);
}

static void
dma_context_restore_clobber_p0r0 (int core, uint32_t base, bfin_dma *dma)
{
  core_register_set (core, REG_P0, base);

  mmr_write_clobber_r0 (core, 0x04, dma->start_addr, 4);
  mmr_write_clobber_r0 (core, 0x10, dma->x_count, 2);
  mmr_write_clobber_r0 (core, 0x14, dma->x_modify, 2);
  mmr_write_clobber_r0 (core, 0x08, dma->config, 2);
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
dma_copy (int core, uint32_t dest, uint32_t src, int size)
{
  bfin_dma mdma_s0_save, mdma_d0_save;
  bfin_dma mdma_s0, mdma_d0;
  uint32_t p0, r0;
  uint16_t s0_irq_status, d0_irq_status;
  int ret;
  struct timespec dma_wait = {0, 50000000};

  emupc_reset ();

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  dma_context_save_clobber_p0r0 (core, cpu->mdma_s0, &mdma_s0_save);
  dma_context_save_clobber_p0r0 (core, cpu->mdma_d0, &mdma_d0_save);

  bfin_log_dma (cpu->mdma_s0, &mdma_s0_save);
  bfin_log_dma (cpu->mdma_d0, &mdma_d0_save);

  s0_irq_status = mdma_s0_save.irq_status;
  d0_irq_status = mdma_d0_save.irq_status;

  while ((s0_irq_status & DMA_IRQ_STATUS_DMA_RUN) ||
	 (d0_irq_status & DMA_IRQ_STATUS_DMA_RUN))
    {
      if ((s0_irq_status & DMA_IRQ_STATUS_DMA_ERR) ||
	  (d0_irq_status & DMA_IRQ_STATUS_DMA_ERR))
	break;

      bfin_log (RP_VAL_LOGLEVEL_NOTICE,
		"%s: wait DMA done: S0:0x%08X [0x%04X] D0:0x%08X [0x%04X]",
		bfin_target.name, src, s0_irq_status, dest, d0_irq_status);

      nanosleep (&dma_wait, NULL);

      core_register_set (core, REG_P0, cpu->mdma_s0);
      s0_irq_status = mmr_read_clobber_r0 (core, 0x28, 2);
      core_register_set (core, REG_P0, cpu->mdma_d0);
      d0_irq_status = mmr_read_clobber_r0 (core, 0x28, 2);
    }

  if (s0_irq_status & DMA_IRQ_STATUS_DMA_ERR)
    bfin_log (RP_VAL_LOGLEVEL_NOTICE,
	      "%s: clear MDMA0_S0 DMA ERR: IRQ_STATUS [0x%04X]",
	      bfin_target.name, s0_irq_status);
  if (s0_irq_status & DMA_IRQ_STATUS_DMA_DONE)
    bfin_log (RP_VAL_LOGLEVEL_NOTICE,
	      "%s: clear MDMA_S0 DMA DONE: IRQ_STATUS [0x%04X]",
	      bfin_target.name, s0_irq_status);
  if (d0_irq_status & DMA_IRQ_STATUS_DMA_ERR)
    bfin_log (RP_VAL_LOGLEVEL_NOTICE,
	      "%s: clear MDMA0_D0 DMA ERR: IRQ_STATUS [0x%04X]",
	      bfin_target.name, d0_irq_status);
  if (d0_irq_status & DMA_IRQ_STATUS_DMA_DONE)
    bfin_log (RP_VAL_LOGLEVEL_NOTICE,
	      "%s: clear MDMA_D0 DMA DONE: IRQ_STATUS [0x%04X]",
	      bfin_target.name, d0_irq_status);

  if (s0_irq_status & (DMA_IRQ_STATUS_DMA_ERR | DMA_IRQ_STATUS_DMA_DONE))
    {
      core_register_set (core, REG_P0, cpu->mdma_s0);
      mmr_write_clobber_r0 (core,
			    0x28,
			    DMA_IRQ_STATUS_DMA_ERR | DMA_IRQ_STATUS_DMA_DONE,
			    2);
    }

  if (d0_irq_status & (DMA_IRQ_STATUS_DMA_ERR | DMA_IRQ_STATUS_DMA_DONE))
    {
      core_register_set (core, REG_P0, cpu->mdma_d0);
      mmr_write_clobber_r0 (core,
			    0x28,
			    DMA_IRQ_STATUS_DMA_ERR | DMA_IRQ_STATUS_DMA_DONE,
			    2);
    }

  core_register_set (core, REG_P0, cpu->mdma_s0);
  s0_irq_status = mmr_read_clobber_r0 (core, 0x28, 2);
  core_register_set (core, REG_P0, cpu->mdma_d0);
  d0_irq_status = mmr_read_clobber_r0 (core, 0x28, 2);

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
  s0_irq_status = mmr_read_clobber_r0 (core, 0x28, 2);
  core_register_set (core, REG_P0, cpu->mdma_d0);
  d0_irq_status = mmr_read_clobber_r0 (core, 0x28, 2);

  if (s0_irq_status & DMA_IRQ_STATUS_DMA_ERR)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: MDMA0_S0 DMA error: 0x%08X: IRQ_STATUS [0x%04X]",
		bfin_target.name, src, s0_irq_status);
      ret = -1;
      goto finish_dma_copy;
    }
  if (d0_irq_status & DMA_IRQ_STATUS_DMA_ERR)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: MDMA0_D0 DMA error: 0x%08X: IRQ_STATUS [0x%04X]",
		bfin_target.name, dest, d0_irq_status);
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
	    "%s: done dma copy MDMA_S0: 0x%08X: IRQ_STATUS [0x%04X]",
	    bfin_target.name, src, s0_irq_status);

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: done dma copy MDMA_D0: 0x%08X: IRQ_STATUS [0x%04X]",
	    bfin_target.name, dest, d0_irq_status);

  if ((s0_irq_status & DMA_IRQ_STATUS_DMA_ERR)
      || (s0_irq_status & DMA_IRQ_STATUS_DMA_DONE))
    {
      core_register_set (core, REG_P0, cpu->mdma_s0);
      mmr_write_clobber_r0 (core,
			    0x28,
			    DMA_IRQ_STATUS_DMA_ERR | DMA_IRQ_STATUS_DMA_DONE,
			    2);
    }

  if ((d0_irq_status & DMA_IRQ_STATUS_DMA_ERR)
      || (d0_irq_status & DMA_IRQ_STATUS_DMA_DONE))
    {
      core_register_set (core, REG_P0, cpu->mdma_d0);
      mmr_write_clobber_r0 (core,
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
memory_write_1 (int core, uint32_t addr, uint8_t *buf, int size)
{
  uint32_t p0, r0;
  int dcplb_enabled;

  assert (size > 0);

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  dcplb_enabled = core_dcplb_disable_clobber_p0r0 (core);

  core_register_set (core, REG_P0, addr);

  core_dbgctl_bit_set_emuirlpsz_2 (core, UPDATE);

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

  core_dbgctl_bit_clear_emuirlpsz_2 (core, UPDATE);

  if (dcplb_enabled)
    core_dcplb_enable_clobber_p0r0 (core);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);

  return 0;
}

static int
memory_write (int core, uint32_t addr, uint8_t *buf, int size)
{
  int s;

  if ((addr & 0x3) != 0)
    {
      s = 4 - (addr & 0x3);
      s = size < s ? size : s;
      memory_write_1 (core, addr, buf, s);
      size -= s;
      addr += s;
      buf += s;
    }

  while (size > 0)
    {
      emupc_reset ();

      /* The overhead should be no larger than 0x20.  */
      s = (rti_limit - 0x20) * 4;
      s = size < s ? size : s;
      memory_write_1 (core, addr, buf, s);
      size -= s;
      addr += s;
      buf += s;
    }

  return 0;
}

static int
dma_sram_read (int core, uint32_t addr, uint8_t *buf, int size)
{
  uint32_t l1_addr;
  uint8_t *tmp;
  int ret;

  l1_addr = cpu->cores[core].l1_map->l1_data_a;

  assert (size > 0);
  assert (size < 0x4000);

  tmp = (uint8_t *) malloc (size);
  if (tmp == 0)
    abort ();

  memory_read (core, l1_addr, tmp, size);

  ret = dma_copy (core, l1_addr, addr, size);

  memory_read (core, l1_addr, buf, size);

  memory_write (core, l1_addr, tmp, size);

  free (tmp);

  return ret;
}

static int
dma_sram_write (int core, uint32_t addr, uint8_t *buf, int size)
{
  uint32_t l1_addr;
  uint8_t *tmp;
  int ret;

  l1_addr = cpu->cores[core].l1_map->l1_data_a;

  assert (size > 0);

  assert (l1_addr);

  assert (size <= MAP_LEN (cpu->cores[core].l1_map->l1_data_a));

  tmp = (uint8_t *) malloc (size);
  if (tmp == 0)
    abort ();

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "dma_sram_write (core [%d], addr [0x%08X], size [0x%X])",
	    cpu->first_core + core, addr, size);

  memory_read (core, l1_addr, tmp, size);

  memory_write (core, l1_addr, buf, size);

  ret = dma_copy (core, addr, l1_addr, size);

  memory_write (core, l1_addr, tmp, size);

  free (tmp);

  return ret;
}

/* Read Instruction SRAM using Instruction Test Registers.  */

/* Do one ITEST read.  ADDR should be aligned to 8 bytes. BUF should
   be enough large to hold 64 bits.  */
static void
itest_read_clobber_r0 (int core, bfin_test_data *test_data,
                       uint32_t addr, uint8_t *buf)
{
  urj_part_t *part;
  uint32_t command, data1, data0;

  assert ((addr & 0x7) == 0);

  part = cpu->chain->parts->parts[cpu->first_core + core];

  EMU_OAB (part)->test_command (part, addr, 0, test_data->command_addr, &command);

  mmr_write_clobber_r0 (core, 0, command, 4);
  core_emuir_set (core, INSN_CSYNC, RUNTEST);
  data1 = mmr_read_clobber_r0 (core, test_data->data1_off, 4);
  data0 = mmr_read_clobber_r0 (core, test_data->data0_off, 4);

  *buf++ = data0 & 0xff;
  *buf++ = (data0 >> 8) & 0xff;
  *buf++ = (data0 >> 16) & 0xff;
  *buf++ = (data0 >> 24) & 0xff;
  *buf++ = data1 & 0xff;
  *buf++ = (data1 >> 8) & 0xff;
  *buf++ = (data1 >> 16) & 0xff;
  *buf++ = (data1 >> 24) & 0xff;
}

/* Do one ITEST write.  ADDR should be aligned to 8 bytes. BUF should
   be enough large to hold 64 bits.  */
static void
itest_write_clobber_r0 (int core, bfin_test_data *test_data,
                        uint32_t addr, uint8_t *buf)
{
  urj_part_t *part;
  uint32_t command, data1, data0;

  assert ((addr & 0x7) == 0);

  part = cpu->chain->parts->parts[cpu->first_core + core];

  EMU_OAB (part)->test_command (part, addr, 1, test_data->command_addr, &command);

  data0 = *buf++;
  data0 |= (*buf++) << 8;
  data0 |= (*buf++) << 16;
  data0 |= (*buf++) << 24;
  data1 = *buf++;
  data1 |= (*buf++) << 8;
  data1 |= (*buf++) << 16;
  data1 |= (*buf++) << 24;

  mmr_write_clobber_r0 (core, test_data->data1_off, data1, 4);
  mmr_write_clobber_r0 (core, test_data->data0_off, data0, 4);
  mmr_write_clobber_r0 (core, 0, command, 4);
  core_emuir_set (core, INSN_CSYNC, RUNTEST);
}

static void
test_context_save_clobber_r0 (int core, bfin_test_data *test_data)
{
  test_data->data1 = mmr_read_clobber_r0 (core, test_data->data1_off, 4);
  test_data->data0 = mmr_read_clobber_r0 (core, test_data->data0_off, 4);
}

static void
test_context_restore_clobber_r0 (int core, bfin_test_data *test_data)
{
  mmr_write_clobber_r0 (core, test_data->data1_off, test_data->data1, 4);
  mmr_write_clobber_r0 (core, test_data->data0_off, test_data->data0, 4);
  /* Yeah, TEST_COMMAND is reset to 0, i.e. clobbered!  But it should
     not do any harm to any reasonable user programs.  Codes trying to
     peek TEST_COMMAND might be affected.  But why do such codes
     exist?  */
  mmr_write_clobber_r0 (core, 0, 0, 4);
  core_emuir_set (core, INSN_CSYNC, RUNTEST);
}

static int
itest_sram_1 (int core, uint32_t addr, uint8_t *buf, int size, int w)
{
  bfin_test_data test_data;
  uint32_t p0, r0;
  uint8_t data[8];
  bfin_core *c;
  urj_part_t *part;

  p0 = core_register_get (core, REG_P0);
  r0 = core_register_get (core, REG_R0);

  c = &cpu->cores[core];
  part = cpu->chain->parts->parts[cpu->first_core + core];
  EMU_OAB (part)->test_command_mmrs (part, addr,
                                     IN_MAP (addr, c->l1_map->l1_code_cache),
                                     &test_data.command_addr,
                                     &test_data.data0_addr,
                                     &test_data.data1_addr);
  test_data.data0_off = test_data.data0_addr - test_data.command_addr;
  test_data.data1_off = test_data.data1_addr - test_data.command_addr;
  core_register_set (core, REG_P0, test_data.command_addr);

  test_context_save_clobber_r0 (core, &test_data);

  if ((addr & 0x7) != 0)
    {
      uint32_t aligned_addr = addr & 0xfffffff8;

      itest_read_clobber_r0 (core, &test_data, aligned_addr, data);

      if (w)
	{
	  while ((addr & 0x7) != 0 && size != 0)
	    {
	      data[addr & 0x7] = *buf++;
	      addr++;
	      size--;
	    }

	  itest_write_clobber_r0 (core, &test_data, aligned_addr, data);
	}
      else
	{
	  while ((addr & 0x7) != 0 && size != 0)
	    {
	      *buf++ = data[addr & 0x7];
	      addr++;
	      size--;
	    }
	}
    }

  for (; size >= 8; size -= 8)
    {
      if (w)
	itest_write_clobber_r0 (core, &test_data, addr, buf);
      else
	itest_read_clobber_r0 (core, &test_data, addr, buf);
      addr += 8;
      buf += 8;
    }

  if (size != 0)
    {
      itest_read_clobber_r0 (core, &test_data, addr, data);

      if (w)
	{
	  memcpy (data, buf, size);
	  itest_write_clobber_r0 (core, &test_data, addr, data);
	}
      else
	{
	  memcpy (buf, data, size);
	}
    }

  test_context_restore_clobber_r0 (core, &test_data);

  core_register_set (core, REG_P0, p0);
  core_register_set (core, REG_R0, r0);

  return 0;
}

static int
itest_sram (int core, uint32_t addr, uint8_t *buf, int size, int w)
{
  int s;

  if ((addr & 0x7) != 0)
    {
      s = 8 - (addr & 0x7);
      s = size < s ? size : s;
      itest_sram_1 (core, addr, buf, s, w);
      size -= s;
      addr += s;
      buf += s;
    }

  while (size > 0)
    {
      emupc_reset ();

      /* The overhead should be no larger than 0x20.  */
      s = (rti_limit - 0x20) / 25 * 32;
      s = size < s ? size : s;
      /* We also need to keep transfers from spanning SRAM banks.
	 The core itest logic looks up appropriate MMRs once per
	 call, and different SRAM banks might require a different
	 set of MMRs.  */
      if ((addr / 0x4000) != ((addr + s) / 0x4000))
	s -= ((addr + s) % 0x4000);
      itest_sram_1 (core, addr, buf, s, w);
      size -= s;
      addr += s;
      buf += s;
    }

  return 0;
}

/* Wrappers for sram_read and sram_write.  If DMA_P is not zero, DMA
   should be used.  Otherwise, ITEST may be used.  */

static int
sram_read_write (int core, uint32_t addr, uint8_t *buf, int size, int dma_p,
		 int w)
{
  bfin_core *c;
  urj_part_t *part;
  int use_test_mmrs;

  assert (!dma_p || cpu->mdma_d0 != 0);

  part = cpu->chain->parts->parts[cpu->first_core + core];
  c = &cpu->cores[core];

  use_test_mmrs = IN_MAP (addr, c->l1_map->l1_code_cache) ||
		  IN_MAP (addr, c->l1_map->l1_code);
  /* The BF592 has MMR access to its L1 ROM, but not DMA.  */
  if (!use_test_mmrs && !strcmp (part->part, "BF592"))
    use_test_mmrs = IN_MAP (addr, c->l1_map->l1_code_rom);

  if (dma_p || use_dma || !use_test_mmrs)
    {
      if (w)
	return dma_sram_write (core, addr, buf, size);
      else
	return dma_sram_read (core, addr, buf, size);
    }
  else
    return itest_sram (core, addr, buf, size, w);
}
#define sram_read(c, a, b, s, d) sram_read_write(c, a, b, s, d, 0)
#define sram_write(c, a, b, s, d) sram_read_write(c, a, b, s, d, 1)

static int
core_memory_read_write (int core, uint32_t addr, uint8_t *buf, int size,
			int local, int w)
{
  if (local)
    {
      if (w)
	return memory_write (core, addr, buf, size);
      else
	return memory_read (core, addr, buf, size);
    }
  else
    return sram_read_write (core, addr, buf, size, 1, w);
}
#define core_memory_read(c, a, b, s, l) core_memory_read_write(c, a, b, s, l, 0)
#define core_memory_write(c, a, b, s, l) core_memory_read_write(c, a, b, s, l, 1)

static uint8_t bfin_breakpoint_16[] = { 0x25, 0x0 };
static uint8_t bfin_breakpoint_32[] = { 0x25, 0x0, 0x0, 0x0 };

static bfin_swbp *
add_swbp_at (uint32_t addr)
{
  bfin_swbp *bp;
  int actual_size;
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
	    "%s: [%d] core_single_step ()",
	    bfin_target.name, cpu->first_core + core);

  if (!cpu->cores[core].is_stepping)
    core_dbgctl_bit_set_esstep (core, UPDATE);
  core_emuir_set (core, INSN_RTE, RUNTEST);
  core_check_emuready (core);
  if (!cpu->cores[core].is_stepping)
    core_dbgctl_bit_clear_esstep (core, UPDATE);
}

static void
core_cache_status_get (int core)
{
  uint32_t p0, r0;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_cache_status_get ()", bfin_target.name,
	    cpu->first_core + core);

  /* If the core is locked, caches remain disabled by default.
     If the core has a core fault or is running, we cannot get
     its current cache status. Assume its cache status isn't changed
     since last get.  */
  if (cpu->cores[core].is_locked
      || cpu->cores[core].is_corefault
      || cpu->cores[core].is_running)
    return;

  /* Instead of calling mmr_read twice, we save one of context save
     and restore.  */
  if (!cpu->cores[core].dmem_control_valid_p
      && !cpu->cores[core].imem_control_valid_p)
    {
      p0 = core_register_get (core, REG_P0);
      r0 = core_register_get (core, REG_R0);

      core_register_set (core, REG_P0, DMEM_CONTROL);

      cpu->cores[core].dmem_control = mmr_read_clobber_r0 (core, 0, 4);
      cpu->cores[core].dmem_control_valid_p = 1;
      cpu->cores[core].imem_control = mmr_read_clobber_r0 (core, IMEM_CONTROL - DMEM_CONTROL, 4);
      cpu->cores[core].imem_control_valid_p = 1;

      core_register_set (core, REG_P0, p0);
      core_register_set (core, REG_R0, r0);
    }
  else if (!cpu->cores[core].dmem_control_valid_p)
    /* No need to set dmem_control and dmem_control_valid_p here.
       mmr_read will handle them.  */
    mmr_read (core, DMEM_CONTROL, 4);
  else if (!cpu->cores[core].imem_control_valid_p)
    /* No need to set imem_control and imem_control_valid_p here.
       mmr_read will handle them.  */
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


static struct circ_buf jc_net_buf, jc_jtag_buf;
static uint32_t jc_net_len, jc_jtag_len;
static unsigned int jc_port = 2001;
static int jc_listen_sock = -1;

/* Helper function to make an fd non-blocking */
static void set_fd_nonblock (int fd)
{
#ifdef __MINGW32__
  int ret;
  u_long mode = 1;
  ret = ioctlsocket (fd, FIONBIO, &mode);
  assert (ret == 0);
#else
  int ret = fcntl (fd, F_GETFL);
  assert (ret != -1);
  ret = fcntl (fd, F_SETFL, ret | O_NONBLOCK);
  assert (ret == 0);
#endif
}

/* Helper function to decode/dump an EMUDAT 40bit register */
static void jc_emudat_show (urj_tap_register_t *r, const char *id)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: jc: %sjtag 0x%08"PRIx64"%s%s",
	    bfin_target.name, id, emudat_value (r),
	    r->data[32] ? " emudof" : "", r->data[33] ? " emudif" : "");
}

static void jc_state_reset (void)
{
  jc_net_len = jc_jtag_len = 0;
  circ_clear (&jc_net_buf);
  circ_clear (&jc_jtag_buf);
}

/* If EMUDAT_OUT is valid (the Blackfin sending data to us), read it from JTAG
 * chain and store it in the net circular buffer
 */
static int jc_maybe_queue (urj_tap_register_t *rif, urj_tap_register_t *rof)
{
  const char *fmt;
  uint64_t value;
  int ret = 0;

  if (!rof->data[32])
    return ret;

  /* First we scan in the length of the data, then we read the data */
  value = emudat_value (rof);
  if (jc_jtag_len)
    {
      uint32_t this_in;
      char data[4] = { /* shift manually to avoid endian issues */
	(value >>  0) & 0xff,
	(value >>  8) & 0xff,
	(value >> 16) & 0xff,
	(value >> 24) & 0xff,
      };
      this_in = MIN(jc_jtag_len, 4);
      circ_puts(&jc_jtag_buf, data, this_in);
      jc_jtag_len -= this_in;
      fmt = "<D";
      ret = 1;
    }
  else
    {
      jc_jtag_len = value;
      fmt = "<L";
    }

  jc_emudat_show (rof, fmt);
  rif->data[32] = 0;

  return ret;
}

/* Scan EMUDAT and see if there is any data from the Blackfin proc, or if
 * there is room for us to send data from the network.
 */
static int jc_process (int core)
{
  urj_part_t *part;
  urj_tap_register_t *rof, *rif;
  int ret;

  core_scan_select (cpu->core_a, EMUDAT_SCAN);

  part = cpu->chain->parts->parts[cpu->first_core + cpu->core_a];
  rof = part->active_instruction->data_register->out;
  rif = part->active_instruction->data_register->in;

  rif->data[33] = 0;
  urj_tap_chain_shift_data_registers (cpu->chain, 1);

  jc_emudat_show (rif, "I-");
  jc_emudat_show (rof, "O-");
  ret = jc_maybe_queue (rif, rof);

  /* EMUDAT_IN: data for Blackfin */
  if (!circ_empty(&jc_net_buf) && !rof->data[33])
    {
      size_t i, reg_size;
      uint32_t emudat, this_out;
      const char *fmt;

      reg_size = sizeof (emudat);
      if (jc_net_len)
	{
	  /* First we write the # of bytes we are going to send */
	  char data[4];
	  this_out = MIN(reg_size, jc_net_len);
	  circ_gets(&jc_net_buf, data, this_out);
	  /* shift manually to avoid endian issues */
	  emudat = data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24;
	  jc_net_len -= this_out;
	  fmt = "D>";
	}
      else
	{
	  /* Then we send the actual data */
	  jc_net_len = MIN(reg_size, circ_cnt(&jc_net_buf));
	  emudat = jc_net_len;
	  fmt = "L>";
	}

      /* Split our 32bit data into the data[] array */
      reg_size *= 8;
      for (i = 0; i < reg_size; ++i)
	rif->data[i] = (emudat >> (reg_size - 1 - i)) & 0x1;
      rif->data[33] = 1;
      jc_emudat_show (rif, fmt);

      /* Shift out the datum */
      urj_tap_chain_shift_data_registers (cpu->chain, 1);
      jc_emudat_show (rif, "I-");
      jc_emudat_show (rof, "O-");
      ret += jc_maybe_queue (rif, rof);
      rif->data[33] = 0;
    }

  return ret;
}

/* Check the network and jtag for pending data */
static int jc_loop (void)
{
  static int jc_sock = -1;
  char buf[CIRC_SIZE];
  ssize_t io_ret;

  if (jc_listen_sock == -1)
    return -1;

  /* We only handle one connection at a time */
  if (jc_sock == -1)
    {
      jc_sock = sock_accept (jc_listen_sock);
      if (jc_sock == -1)
	return -1;
      bfin_log (RP_VAL_LOGLEVEL_NOTICE, "%s: jc: connected", bfin_target.name);
      set_fd_nonblock (jc_sock);
    }

  /* Grab data from network into buffer for jtag transmission */
  if (!circ_full(&jc_net_buf))
    {
      io_ret = read (jc_sock, buf, circ_free(&jc_net_buf));
      if (io_ret > 0)
	{
	  buf[io_ret] = '\0';
	  circ_puts(&jc_net_buf, buf, io_ret);
	  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: jc: net[%i/%i]: %s",
		    bfin_target.name, io_ret, circ_cnt(&jc_net_buf), buf);
	}
      else if (io_ret == 0 || errno != EAGAIN)
	{
	  bfin_log (RP_VAL_LOGLEVEL_NOTICE, "%s: jc: disconnected",
		    bfin_target.name);
	  close (jc_sock);
	  jc_sock = -1;
	}
    }

  /* Send out data from jtag buffer to network */
  if (!circ_empty(&jc_jtag_buf))
    {
      io_ret = circ_cnt(&jc_jtag_buf);
      circ_gets(&jc_jtag_buf, buf, io_ret);
      buf[io_ret] = '\0';
      bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: jc: jtag[%i/%i]: %s",
		bfin_target.name, io_ret, circ_cnt(&jc_jtag_buf), buf);
      write (jc_sock, buf, io_ret);
    }

  /* See if there is anything in the jtag scan chain */
  return jc_process (0);
}

/* Set up the port for jtag communication */
static void jc_init (void)
{
  jc_listen_sock = listen_sock_open (&jc_port);
  if (jc_listen_sock == -1)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR, "%s: jc: TCP port not available",
		bfin_target.name);
      return;
    }
  set_fd_nonblock (jc_listen_sock);

  bfin_log (RP_VAL_LOGLEVEL_NOTICE, "%s: jc: waiting on TCP port %u",
	    bfin_target.name, jc_port);
  bfin_log (RP_VAL_LOGLEVEL_NOTICE, "%s: jc:  (you must connect GDB before using jtag console)",
	    bfin_target.name);
}


/* Urjtag logging hooks */
static out_func urjtag_of;
static int
bfin_urjtag_vprintf(const char *fmt, va_list ap)
{
  char raw[4096], cook[4096];

  vsnprintf (raw, sizeof (raw) - 1, fmt, ap);
  raw[sizeof (raw) - 1] = '\0';

  rp_encode_string (raw, cook, sizeof (cook));
  urjtag_of (cook);

  return 0;
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
  printf (" --board=BOARD           specify the board\n");
  printf (" --processor=Nth         specify the processor for debugging\n");
  printf (" --connect=STRING        JTAG connection string\n");
  printf ("                         (default %s)\n", default_jtag_connect);
  printf (" --frequency=FREQUENCY   set the cable frequency\n");
  printf (" --check-emuready        check for EMUREADY in emulator operations\n");
  printf (" --enable-dcache=METHOD  enable all data SRAM caches\n");
  printf (" --enable-icache         enable all instruction SRAM caches\n");
  printf (" --flash-size=BYTES      specify the size of flash\n");
  printf (" --force-range-wp        always use range watchpoint\n");
  printf (" --init-sdram            initialize SDRAM or DDR memory\n");
  printf (" --loop-wait=USEC        wait USEC microseconds in wait loop (default 10000)\n");
  printf (" --no-auto-switch        Don't automatically switch to the core\n");
  printf ("                         which contains the address set to PC\n");
  printf (" --invalid-mem-access={ignore,reject,allow-dma,allow-core}\n");
  printf ("                         Ignore/Reject/Allow unknown/invalid memory accesses\n");
  printf (" --reset                 do a core and system reset when gdb connects\n");
  printf (" --sdram-size=BYTES      specify the size of SDRAM\n");
  printf (" --unlock-on-connect     unlock core when gdb connects\n");
  printf (" --unlock-on-load        unlock core when loading its L1 code\n");
  printf (" --use-dma               Use DMA to access Instruction SRAM\n");
  printf ("                         Default ITEST or DTEST is used when possible\n");
  printf (" --jc-port=PORT          use specified port for JTAG communication\n");
  printf (" --wait-clocks=NUM       wait the specified number of clocks in Run-Test/Idle\n");
  printf ("                         so instruction in EMUIR could be completed\n");
  printf ("\n");

  return;
}

/* Target method */
static int
bfin_open (int argc,
	   char *const argv[], const char *prog_name, log_func log_fn)
{
  urj_chain_t *chain;
  bfin_core *c;
  int i;
  bfin_board board;
  int sdram_size;
  int flash_size;
  int usec;
  int cpu_num, first_core, core_num;

  char *connect_string = default_jtag_connect;
  char *cmd_detect[2] = {"detect", NULL};

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
    {"check-emuready", no_argument, 0, 12},
    {"no-switch-on-load", no_argument, 0, 13},
    {"connect", required_argument, 0, 14},
    {"invalid-mem-access", required_argument, 0, 15},
    {"use-dma", no_argument, 0, 16},
    {"jc-port", required_argument, 0, 17},
    {"frequency", required_argument, 0, 18},
    {"processor", required_argument, 0, 19},
    {"wait-clocks", required_argument, 0, 20},
    {NULL, 0, 0, 0}
  };

  assert (!cpu);
  assert (prog_name != NULL);
  assert (log_fn != NULL);

  /* Set log */
  bfin_log = log_fn;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_open ()", bfin_target.name);

  sdram_size = -1;
  flash_size = -1;
  board = UNKNOWN_BOARD;

  /* Process options */

  /* Default we don't check EMUREADY */
  bfin_check_emuready = 0;

  optind = 1;
  for (;;)
    {
      int opt;
      int option_index;

      opt = getopt_long (argc, argv, "+", long_options, &option_index);
      if (opt == -1)
	break;
      switch (opt)
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
		      "%s: bad cache write policy `%s', should be `write-through' or `write-back'",
		      bfin_target.name, optarg);
	  break;

	case 10:
	  bfin_enable_icache = 1;
	  break;

	case 11:
	  bfin_reset = 1;
	  break;

	case 12:
	  bfin_check_emuready = 1;
	  break;

	case 13:
	  bfin_auto_switch = 0;
	  break;

	case 14:
	  connect_string = optarg;
	  break;

	case 15:
	  if (!strcmp (optarg, "ignore"))
	    invalid_mem_access = INVALID_MEM_ACCESS_IGNORE;
	  else if (!strcmp (optarg, "reject"))
	    invalid_mem_access = INVALID_MEM_ACCESS_REJECT;
	  else if (!strcmp (optarg, "allow-dma"))
	    invalid_mem_access = INVALID_MEM_ACCESS_ALLOW_DMA;
	  else if (!strcmp (optarg, "allow-core"))
	    invalid_mem_access = INVALID_MEM_ACCESS_ALLOW_CORE;
	  else
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: bad invalid memory behavior %s",
			bfin_target.name, optarg);
	      exit (1);
	    }
	  break;

	case 16:
	  use_dma = 1;
	  break;

	case 17:
	  jc_port = atoi(optarg);
	  break;

	case 18:
	  bfin_frequency = atol (optarg);
	  if (bfin_frequency < 0)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: bad frequency %d",
			bfin_target.name, bfin_frequency);
	      exit (1);
	    }
	  break;

	case 19:
	  bfin_processor = atol (optarg);
	  if (bfin_processor < 0)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: bad processor number %d",
			bfin_target.name, bfin_processor);
	      exit (1);
	    }
	  break;

	case 20:
	  bfin_wait_clocks = atoi (optarg);
	  if (bfin_wait_clocks < 0)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: bad wait clocks number %d",
			bfin_target.name, bfin_wait_clocks);
	      exit (1);
	    }
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

  if (optind != argc && argc)
    {
      /* Bad number of arguments.  */
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: bad number of arguments", bfin_target.name);
      bfin_target.help (prog_name);

      return RP_VAL_TARGETRET_ERR;
    }

  chain = urj_tap_chain_alloc ();

  if (!chain)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: chain allocation failed", bfin_target.name);
      return RP_VAL_TARGETRET_ERR;
    }

  urj_parse_line (chain, connect_string);

  if (!chain->cable)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: cable initialization failed", bfin_target.name);
      return RP_VAL_TARGETRET_ERR;
    }

  if (bfin_frequency != 0)
    urj_tap_cable_set_frequency (chain->cable, bfin_frequency);

  urj_cmd_run (chain, cmd_detect);
  if (!chain->parts || !chain->parts->len)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: detecting parts failed", bfin_target.name);
      return RP_VAL_TARGETRET_ERR;
    }

  first_core = -1;
  cpu_num = 0;
  for (i = 0; i < chain->parts->len; i++)
    {
      /* If user does not specify a Blackfin processor for debugging,
	 use the first one.  */
      if ((bfin_processor == -1 && part_is_bfin (chain, i) && first_core == -1)
	  || cpu_num == bfin_processor)
	{
	  first_core = i;
	  if (!strcmp (chain->parts->parts[i]->part, "BF561"))
	    i++;
	}
      else
	{
	  part_bypass (chain, i);
	  if (!strcmp (chain->parts->parts[i]->part, "BF561"))
	    {
	      i++;
	      part_bypass (chain, i);
	    }
	}
      cpu_num++;
    }

  if (first_core == -1)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: no Blackfin processor is found", bfin_target.name);
      return RP_VAL_TARGETRET_ERR;
    }


  if (!strcmp (chain->parts->parts[first_core]->part, "BF561"))
    core_num = 2;
  else
    core_num = 1;

  /* TODO initbus */

  /* TODO detectflash */

  cpu = (bfin_cpu *) malloc (sizeof (bfin_cpu) + sizeof (bfin_core) * core_num);
  if (!cpu)
    {
      urj_part_parts_free (chain->parts);
      chain->parts = 0;
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: cpu allocation failed", bfin_target.name);
      return RP_VAL_TARGETRET_ERR;
    }

  cpu->chain = chain;
  cpu->board = board;
  cpu->swbps = NULL;
  cpu->first_core = first_core;
  cpu->core_num = core_num;

  /* BF531/2/3

     #define MDMA_D0_NEXT_DESC_PTR      0xFFC00E00

     BF52x BF534/6/7 BF54x BF51x BF50x BF59x

     #define MDMA_D0_NEXT_DESC_PTR      0xFFC00F00

     BF538/9

     #define MDMA0_D0_NEXT_DESC_PTR     0xFFC00E00
     #define MDMA1_D0_NEXT_DESC_PTR     0xFFC01F00

     BF561

     #define MDMA1_D0_NEXT_DESC_PTR     0xFFC01F00
     #define MDMA2_D0_NEXT_DESC_PTR     0xFFC00F00
     #define IMDMA_D0_NEXT_DESC_PTR     0xFFC01800
   */

  if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF506"))
    {
      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf50x_mem_map;
      cpu->cores[0].l1_map = &bf50x_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF518"))
    {
      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf52x_mem_map;
      cpu->cores[0].l1_map = &bf51x_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF526") ||
	   !strcmp (chain->parts->parts[cpu->first_core]->part, "BF527"))
    {
      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf52x_mem_map;
      cpu->cores[0].l1_map = &bf52x_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF533"))
    {
      cpu->mdma_d0 = 0xffc00e00;
      cpu->mdma_s0 = 0xffc00e40;
      cpu->mem_map = bf533_mem_map;
      cpu->cores[0].l1_map = &bf533_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF534") ||
           !strcmp (chain->parts->parts[cpu->first_core]->part, "BF537"))
    {
      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf537_mem_map;
      cpu->cores[0].l1_map = &bf537_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF538"))
    {
      cpu->mdma_d0 = 0xffc00e00;
      cpu->mdma_s0 = 0xffc00e40;
      cpu->mem_map = bf538_mem_map;
      cpu->cores[0].l1_map = &bf538_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF548") ||
           !strcmp (chain->parts->parts[cpu->first_core]->part, "BF548M"))
    {
      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf54x_mem_map;
      cpu->cores[0].l1_map = &bf54x_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF561"))
    {
      cpu->mdma_d0 = 0xffc01800;
      cpu->mdma_s0 = 0xffc01840;
      cpu->mem_map = bf561_mem_map;
      cpu->cores[1].l1_map = &bf561_a_l1_map;
      cpu->cores[0].l1_map = &bf561_b_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF592"))
    {
      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf59x_mem_map;
      cpu->cores[0].l1_map = &bf59x_l1_map;
    }
  else
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: unsupported processor '%s'",
		bfin_target.name, chain->parts->parts[0]->part);
      urj_part_parts_free (chain->parts);
      chain->parts = 0;
      free (cpu);
      return RP_VAL_TARGETRET_ERR;
    }

  if (use_dma && cpu->mdma_d0 == 0)
    {
      bfin_log (RP_VAL_LOGLEVEL_WARNING,
		"%s: --use-dma is ignored since there is no MDMA",
		bfin_target.name);
      use_dma = 0;
    }

  switch (board)
    {
    case BF527_EZKIT:
      if (chain->parts->len != 1)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %d cores on BF527 EZKIT board",
		    bfin_target.name, chain->parts->len);
	  exit (1);
	}
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
      if (chain->parts->len != 1)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %d cores on BF533 EZKIT board",
		    bfin_target.name, chain->parts->len);
	  exit (1);
	}
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
      if (chain->parts->len != 1)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %d cores on BF533 STAMP board",
		    bfin_target.name, chain->parts->len);
	  exit (1);
	}
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
      if (chain->parts->len != 1)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %d cores on BF537 %s board",
		    bfin_target.name,
		    chain->parts->len,
		    board == BF537_EZKIT ? "EZKIT" : "STAMP");
	  exit (1);
	}
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
      if (chain->parts->len != 1)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %d cores on BF538F EZKIT board",
		    bfin_target.name, chain->parts->len);
	  exit (1);
	}
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
      if (chain->parts->len != 1)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %d cores on BF548 EZKIT board",
		    bfin_target.name, chain->parts->len);
	  exit (1);
	}
      if (strcmp (chain->parts->parts[0]->part, "BF548") != 0)
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
      if (chain->parts->len != 2)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %d cores on BF561 EZKIT board",
		    bfin_target.name, chain->parts->len);
	  exit (1);
	}
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
      if (chain->parts->len == 1)
	bfin_log (RP_VAL_LOGLEVEL_WARNING,
		  "%s: no board selected, %s is detected",
		  bfin_target.name, chain->parts->parts[0]->part);
      else
	{
	  /* Assume maximal 64 cores.  */
	  char buf[6 * 64];
	  size_t len = 0;

	  bfin_log (RP_VAL_LOGLEVEL_WARNING,
		    "%s: no board selected, %d cores are detected",
		    bfin_target.name, chain->parts->len);
	  for (i = 0; i < chain->parts->len; i++)
	    {
	      if (i == cpu->first_core)
		len += sprintf (buf + len, "[");
	      len += sprintf (buf + len, "%s", chain->parts->parts[i]->part);
	      if (i == cpu->first_core + cpu->core_num - 1)
		len += sprintf (buf + len, "]");
	      len += sprintf (buf + len, " ");
	    }
	  bfin_log (RP_VAL_LOGLEVEL_WARNING, "%s:   cores: %s", bfin_target.name, buf);
	}
      cpu->sdram_config = NULL;
      cpu->ddr_config = NULL;
      break;

    default:
      abort ();
    }

  /* Assume only 32-bit instruction are used in gdbproxy.  */
  for_each_core (i, c)
    {
      int tmp;
      tmp = (c->l1_map->l1_code_end - c->l1_map->l1_code) / 8;
      if (rti_limit > tmp)
	rti_limit = tmp;
    }

  /* Let --sdram-size and --flash-size override the board settting.  */
  if (sdram_size != -1)
    cpu->mem_map.sdram_end = sdram_size;

  if (flash_size != -1)
    cpu->mem_map.flash_end = cpu->mem_map.flash + flash_size;

  if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF561"))
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

  chain->main_part = cpu->first_core + cpu->core_a;

  jc_init ();

  return RP_VAL_TARGETRET_OK;
}


/* Target method */
static void
bfin_close (void)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_close()", bfin_target.name);

  assert (cpu);

  emulation_disable ();

  urj_tap_chain_free (cpu->chain);
  free (cpu);
  cpu = NULL;
}

/* Target method */
static int
bfin_connect (char *status_string, int status_string_len, int *can_restart)
{
  bfin_core *c;
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

  for_each_core (i, c)
    {
      /* We won't use IDCODE_SCAN in debugging. Set it as
         default, such that new scan will be selected.  */
      c->scan = IDCODE_SCAN;
      c->leave_stopped = 0;
      c->is_running = 1;
      c->is_interrupted = 0;
      c->is_stepping = 0;
      c->is_locked = 0;
      c->is_corefault = 0;
      c->status_pending_p = 0;
      c->dmem_control_valid_p = 0;
      c->imem_control_valid_p = 0;
      c->wpiactl = 0;
      c->wpdactl = 0;
      c->wpstat = 0;
      for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
	c->hwbps[j] = -1;
      for (j = 0; j < RP_BFIN_MAX_HWWATCHPOINTS; j++)
	{
	  c->hwwps[j].addr = -1;
	  c->hwwps[j].len = 0;
	  c->hwwps[j].mode = WPDA_DISABLE;
	}
    }

  /* Stop the processor.  */
  emulation_enable ();
  emulation_trigger ();

  emupc_reset ();

  dbgstat_get ();

  need_reset = 0;
  for_each_core (i, c)
    {
      urj_part_t *part = cpu->chain->parts->parts[cpu->first_core + i];

      /* If there is core fault, we have to give it a reset.  */
      if (core_dbgstat_is_core_fault (i))
	{
	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "[%d] core fault: DBGSTAT [0x%04X]",
		    cpu->first_core + i, BFIN_PART_DBGSTAT (part));
	  need_reset = 1;
	}
      /* If emulator is not ready after emulation_trigger (),
	 we have to give it a reset, too.  */
      else if (!core_dbgstat_is_emuready (i)
	       && (!core_dbgstat_is_emuack (i)
		   || (core_dbgstat_is_in_reset (i)
		       && !core_sticky_in_reset (i))))
	{
	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "[%d] emulator not ready: DBGSTAT [0x%04X]",
		    cpu->first_core + i, BFIN_PART_DBGSTAT (part));
	  need_reset = 1;
	}
    }

  if (need_reset || bfin_reset)
    {
      bfin_log (RP_VAL_LOGLEVEL_INFO, "Resetting ...");

      core_reset ();

      /* FIXME  Find a better way to identify if the system exists.  */
      if (cpu->mdma_d0)
	system_reset ();
    }

  dbgstat_get ();

  for_each_core (i, c)
    if (core_dbgstat_is_emuack (i)
	&& (!core_dbgstat_is_in_reset (i)
	    || core_sticky_in_reset (i)))
      {
	urj_part_t *part = cpu->chain->parts->parts[cpu->first_core + i];

	bfin_log (RP_VAL_LOGLEVEL_INFO,
		  "[%d] locked: DBGSTAT [0x%04X]", cpu->first_core + i, BFIN_PART_DBGSTAT (part));
	c->is_locked = 1;
	c->is_running = 0;

	if (bfin_unlock_on_connect)
	  {
	    uint16_t sica_syscr;

	    bfin_log (RP_VAL_LOGLEVEL_INFO,
		      "%s: [%d] unlocking...", bfin_target.name, cpu->first_core + i);

	    sica_syscr = mmr_read (cpu->core_a, SICA_SYSCR, 2);
	    sica_syscr &= ~SICA_SYSCR_COREB_SRAM_INIT;
	    mmr_write (cpu->core_a, SICA_SYSCR, sica_syscr, 2);
	    core_check_emuready (i);
	    c->is_locked = 0;
	    core_wpu_init (i);

	    bfin_log (RP_VAL_LOGLEVEL_INFO,
		      "%s: [%d] done", bfin_target.name, cpu->first_core + i);
	  }
      }
    else
      {
	c->is_running = 0;
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

      for_each_core (i, c)
	{
	  urj_part_t *part = cpu->chain->parts->parts[cpu->first_core + i];

	  if (!c->is_locked)
	    {
	      rete = core_register_get (i, REG_RETE);
	      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
			"[%d] DBGSTAT [0x%04X] PC [0x%08X]",
			cpu->first_core + i, BFIN_PART_DBGSTAT (part), rete);
	    }
	  else
	    bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		      "[%d] DBGSTAT [0x%04X] PC [0xXXXXXXXX]",
		      cpu->first_core + i, BFIN_PART_DBGSTAT (part));
	}
    }

  /* Fill out the the status string.  */
  sprintf (status_string, "T%02d", RP_SIGNAL_TRAP);

  if (cpu->core_num == 1)
    {
      cp = bfin_out_treg (&status_string[3], BFIN_PC_REGNUM);
      cp = bfin_out_treg (cp, BFIN_FP_REGNUM);
    }
  else
    {
      cp = &status_string[3];
      for (i = cpu->core_num - 1; i >= 0; i--)
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
  bfin_core *c;
  int i;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_disconnect ()", bfin_target.name);

  wpu_disable ();

  for_each_core (i, c)
    if (c->is_stepping)
      {
	core_dbgctl_bit_clear_esstep (i, UPDATE);
	c->is_stepping = 0;
      }

  emulation_return ();

  for_each_core (i, c)
    c->is_running = 1;

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
  bfin_core *c;
  int i;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_stop ()", bfin_target.name);

  assert (cpu);

  for_each_core (i, c)
    if (c->is_stepping)
      {
	core_dbgctl_bit_clear_esstep (i, UPDATE);
	c->is_stepping = 0;
      }

  emulation_trigger ();

  for_each_core (i, c)
    {
      c->is_interrupted = 1;
      c->is_running = 0;
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

      if (core < 0 || core >= cpu->core_num)
	return RP_VAL_TARGETRET_ERR;

      cpu->general_core = core;
    }

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_set_ctrl_thread (rp_thread_ref *thread)
{
  bfin_core *c;
  int i, core;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_set_ctrl_thread (%lld)", bfin_target.name, thread->val);

  if (thread->val == ALL_THREADS)
    {
      cpu->continue_core = ALL_CORES;
      for_each_core (i, c)
	c->leave_stopped = 0;
    }
  else if (thread->val == ANY_THREAD)
    {
      if (cpu->continue_core == INVALID_CORE)
	cpu->continue_core = cpu->core_a;
      for_each_core (i, c)
	c->leave_stopped = 0;
    }
  else
    {
      core = PART_NO (thread->val);

      if (core < 0 || core >= cpu->core_num)
	return RP_VAL_TARGETRET_ERR;

      cpu->continue_core = core;
      for_each_core (i, c)
	if (i == core)
	  c->leave_stopped = 0;
	else
	  c->leave_stopped = 1;
    }

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_is_thread_alive (rp_thread_ref *thread, int *alive)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_is_thread_alive ()", bfin_target.name);

  *alive = 1;

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_read_registers (uint8_t *data_buf,
		     uint8_t *avail_buf, int buf_size, int *read_size)
{
  int ret, reg_no;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_read_registers ()", bfin_target.name);

  *read_size = 0;
  for (reg_no = 0; reg_no < BFIN_NUM_REGS; ++reg_no)
    {
      int s_read_size;

      ret = bfin_read_single_register (reg_no, data_buf, avail_buf, buf_size, &s_read_size);
      if (ret != RP_VAL_TARGETRET_OK)
	break;

      data_buf += s_read_size;
      avail_buf += s_read_size;
      buf_size -= s_read_size;
      *read_size += s_read_size;
    }

  return ret;
}

/* Target method */
static int
bfin_write_registers (uint8_t *buf, int write_size)
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
			   int buf_size, int *read_size)
{
  int reg_size;
  int core;

  assert (cpu);

  core = cpu->general_core;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] bfin_read_single_register (%d)",
	    bfin_target.name, core, reg_no);

  /* This is not the right way to get the size of register.  We should
     create a REGISTER type which has a SIZE field indicating the size
     of that register.  */
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
		bfin_target.name, cpu->first_core + core, reg_no);
      memset (avail_buf, 0, reg_size);
    }
  else if (cpu->cores[core].is_corefault)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] CORE FAULT core cannot read register [%d]",
		bfin_target.name, cpu->first_core + core, reg_no);
      memset (avail_buf, 0, reg_size);
    }
  /* The CC register is a pseudo register that is part of ASTAT (bit 5).  */
  else if (reg_no == BFIN_CC_REGNUM)
    {
      uint32_t val;
      reg_no = BFIN_ASTAT_REGNUM;
      emupc_reset ();
      val = (core_register_get (core, map_gdb_core[reg_no]) >> 5) & 1;
      data_buf[0] = val & 0xff;
      data_buf[1] = (val >> 8) & 0xff;
      data_buf[2] = (val >> 16) & 0xff;
      data_buf[3] = (val >> 24) & 0xff;
      memset (avail_buf, 1, reg_size);
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
      emupc_reset ();
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
			    uint8_t *buf, int write_size)
{
  int reg_size;
  uint32_t value;
  int core;
  bfin_core *c;
  int i;

  assert (cpu);

  core = cpu->general_core;

  /* This is not the right way to get the size of register.  We should
     create a REGISTER type which has a SIZE field indicating the size
     of that register.  */
  reg_size = sizeof (cpu->cores[core].registers[reg_no]);

  assert (reg_size == 4);
  assert (buf != NULL);

  /* Read the value as little endian data.  */
  value = buf[0] + (buf[1] << 8) + (buf[2] << 16) + (buf[3] << 24);

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] bfin_write_single_register (%d, 0x%X)",
	    bfin_target.name, core, reg_no, value);

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
		bfin_target.name, cpu->first_core + core, reg_no);
      return RP_VAL_TARGETRET_ERR;
    }

  if (cpu->cores[core].is_corefault)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] CORE FAULT core cannot write register [%d]",
		bfin_target.name, cpu->first_core + core, reg_no);
      return RP_VAL_TARGETRET_ERR;
    }

  /* Setting RETE to an address that the general core cannot
     execute from is mostly a user error. Catch it.  */
  if (map_gdb_core[reg_no] == REG_RETE && IN_MAP (value, cpu->mem_map.l1) &&
      !(IN_MAP (value, cpu->cores[core].l1_map->l1_code) ||
	IN_MAP (value, cpu->cores[core].l1_map->l1_code_cache) ||
	IN_MAP (value, cpu->cores[core].l1_map->l1_code_rom)))
    {
      if (!bfin_auto_switch)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] set RETE an address the core cannot execute from",
		    bfin_target.name, cpu->first_core + core);
	  return RP_VAL_TARGETRET_ERR;
	}

      for_each_core (i, c)
	if (IN_MAP (value, c->l1_map->l1_code) ||
	    IN_MAP (value, c->l1_map->l1_code_cache) ||
	    IN_MAP (value, c->l1_map->l1_code_rom))
	  {
	    if (c->is_locked)
	      {
		bfin_log (RP_VAL_LOGLEVEL_ERR,
			  "%s: [%d] locked core cannot write register [%d]",
			  bfin_target.name, cpu->first_core + i, reg_no);
		return RP_VAL_TARGETRET_ERR;
	      }

	    if (c->is_corefault)
	      {
		bfin_log (RP_VAL_LOGLEVEL_ERR,
			  "%s: [%d] CORE FAULT core cannot write register [%d]",
			  bfin_target.name, cpu->first_core + i, reg_no);
		return RP_VAL_TARGETRET_ERR;
	      }

	    core = i;
	    /* Silently switch to the core to which the address belongs.  */
	    cpu->general_core = i;
	    cpu->continue_core = i;
	    break;
	  }

      if (i == cpu->core_num)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: No core can execute from [0x%08X]",
		    bfin_target.name, value);
	  return RP_VAL_TARGETRET_ERR;
	}
    }

  cpu->cores[core].registers[reg_no] = value;

  emupc_reset ();
  core_register_set (core, map_gdb_core[reg_no],
		     cpu->cores[core].registers[reg_no]);

  core_check_emuready (core);

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_read_inv_mem (int core, uint64_t addr, uint8_t *buf, int *req_size)
{
  switch (invalid_mem_access)
    {
    case INVALID_MEM_ACCESS_IGNORE:
      /* fill with invalid bytes */
      buf[0] = 0xad;
      *req_size = 1;
      return 0;

    case INVALID_MEM_ACCESS_REJECT:
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] cannot read reserved memory [0x%08llX]",
		bfin_target.name, core, addr);
      return RP_VAL_TARGETRET_ERR;

    case INVALID_MEM_ACCESS_ALLOW_DMA:
      /* Use DMA for unknown memory to be safe as it shouldn't trigger
         crap like IVGHW, and it'll always work with L1 inst SRAM.  */
      return dma_sram_read (core, addr, buf, *req_size);

    case INVALID_MEM_ACCESS_ALLOW_CORE:
      return memory_read (core, addr, buf, *req_size);

    default:
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] unknown reserved memory mode %d [0x%08llX]",
		bfin_target.name, core, invalid_mem_access, addr);
      return RP_VAL_TARGETRET_ERR;
    }
}

/* Target method */
static int
bfin_read_mem (uint64_t addr,
	       uint8_t *buf, int req_size, int *actual_size)
{
  bfin_core *c;
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
		cpu->first_core + cpu->general_core,
		cpu->cores[cpu->general_core].
		is_locked ? "locked" : "CORE FAULT", addr);
      return RP_VAL_TARGETRET_ERR;
    }

  /* If general_core is available, use it when needed.
     Otherwise choose any other available core.  */

  avail_core = cpu->general_core;

  if (cpu->cores[avail_core].is_locked || cpu->cores[avail_core].is_corefault)
    for_each_core (i, c)
      {
	if (!c->is_locked && !c->is_corefault && !c->is_running)
	  {
	    avail_core = i;
	    break;
	  }

	if (i == cpu->core_num)
	  {
	    bfin_log (RP_VAL_LOGLEVEL_ERR,
		      "%s: no core available to read memory",
		      bfin_target.name);
	    return RP_VAL_TARGETRET_ERR;
	  }
      }

  emupc_reset ();

  if (!IN_MAP (addr, cpu->mem_map.l1))
    goto skip_l1;

  for_each_core (i, c)
    {
      /* Use the core for its L1 memory if possible.  */
      if (!c->is_locked && !c->is_corefault && !c->is_running)
	core = i;
      else
	core = avail_core;

      if (IN_MAP (addr, c->l1_map->l1))
	core_cache_status_get (i);
      else
	continue;

      if (IN_MAP (addr, c->l1_map->l1_code))
	{
	  if (!c->l1_code_cache_enabled &&
	      (c->l1_map->l1_code_end == c->l1_map->l1_code_cache))
	    end = c->l1_map->l1_code_cache_end;
	  else
	    end = c->l1_map->l1_code_end;

	  if (addr + req_size > end)
	    req_size = end - addr;

	  ret = sram_read (core, addr, buf, req_size, i != core);
	  goto done;
	}
      else if (!c->l1_code_cache_enabled &&
	       IN_MAP (addr, c->l1_map->l1_code_cache))
	{
	  if (addr + req_size > c->l1_map->l1_code_cache_end)
	    req_size = c->l1_map->l1_code_cache_end - addr;

	  ret = sram_read (core, addr, buf, req_size, i != core);
	  goto done;
	}
      else if (IN_MAP (addr, c->l1_map->l1_code_rom))
	{
	  if (addr + req_size > c->l1_map->l1_code_rom_end)
	    req_size = c->l1_map->l1_code_rom_end - addr;

	  ret = sram_read (core, addr, buf, req_size, i != core);
	  goto done;
	}
      else if (IN_MAP (addr, c->l1_map->l1_data_a))
	{
	  if (!c->l1_data_a_cache_enabled &&
	      (c->l1_map->l1_data_a_end == c->l1_map->l1_data_a_cache))
	    end = c->l1_map->l1_data_a_cache_end;
	  else
	    end = c->l1_map->l1_data_a_end;

	  if (addr + req_size > end)
	    req_size = end - addr;

	  ret = core_memory_read (core, addr, buf, req_size, i == core);
	  goto done;
	}
      else if (!c->l1_data_a_cache_enabled &&
	       IN_MAP (addr, c->l1_map->l1_data_a_cache))
	{
	  if (addr + req_size > c->l1_map->l1_data_a_cache_end)
	    req_size = c->l1_map->l1_data_a_cache_end - addr;

	  ret = core_memory_read (core, addr, buf, req_size, i == core);
	  goto done;
	}
      else if (IN_MAP (addr, c->l1_map->l1_data_b))
	{
	  if (!c->l1_data_b_cache_enabled
	      && (c->l1_map->l1_data_b_end
		  == c->l1_map->l1_data_b_cache))
	    end = c->l1_map->l1_data_b_cache_end;
	  else
	    end = c->l1_map->l1_data_b_end;

	  if (addr + req_size > end)
	    req_size = end - addr;

	  ret = core_memory_read (core, addr, buf, req_size, i == core);
	  goto done;
	}
      else if (!c->l1_data_b_cache_enabled &&
	       IN_MAP (addr, c->l1_map->l1_data_b_cache))
	{
	  if (addr + req_size > c->l1_map->l1_data_b_cache_end)
	    req_size = c->l1_map->l1_data_b_cache_end - addr;

	  ret = core_memory_read (core, addr, buf, req_size, i == core);
	  goto done;
	}
      else if (i == core && IN_MAP (addr, c->l1_map->l1_scratch))
	{
	  if (addr + req_size > c->l1_map->l1_scratch_end)
	    req_size = c->l1_map->l1_scratch_end - addr;

	  ret = memory_read (core, addr, buf, req_size);
	  goto done;
	}
      else if (IN_MAP (addr, c->l1_map->l1))
	{
	  ret = bfin_read_inv_mem (core, addr, buf, &req_size);
	  goto done;
	}
    }

 skip_l1:

  /* TODO  Accurately check MMR validity.  */

  if (addr >= cpu->mem_map.coremmr)
    core = cpu->general_core;
  else
    core = avail_core;

  if ((IN_RANGE (addr, cpu->mem_map.sysmmr, cpu->mem_map.coremmr)
       && addr + req_size > cpu->mem_map.coremmr)
      || (addr >= cpu->mem_map.coremmr && addr + req_size < addr))
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] bad MMR addr or size [0x%08llX] size %d",
		bfin_target.name, cpu->first_core + core, addr, req_size);
      return RP_VAL_TARGETRET_ERR;
    }
  
  if (addr >= cpu->mem_map.sysmmr)
    {
      if (req_size != 2 && req_size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] bad MMR size [0x%08llX] size %d",
		    bfin_target.name, cpu->first_core + core, addr, req_size);
	  return RP_VAL_TARGETRET_ERR;
	}

      if ((addr & 0x1) == 1)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] odd MMR addr [0x%08llX]",
		    bfin_target.name, cpu->first_core + core, addr);
	  return RP_VAL_TARGETRET_ERR;
	}


      value = mmr_read (core, addr, req_size);
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

  if (IN_MAP (addr, cpu->mem_map.sdram))
    {
      if (addr + req_size > cpu->mem_map.sdram_end)
	req_size = cpu->mem_map.sdram_end - addr;

      ret = memory_read (core, addr, buf, req_size);
    }
  else if (IN_MAP (addr, cpu->mem_map.async_mem))
    {
      if (addr + req_size > cpu->mem_map.async_mem_end)
	req_size = cpu->mem_map.async_mem_end - addr;

      ret = memory_read (core, addr, buf, req_size);
    }
  /* TODO  Allow access to devices mapped to async mem.  */
  else if (IN_MAP (addr, cpu->mem_map.boot_rom))
    {
      if (addr + req_size > cpu->mem_map.boot_rom_end)
	req_size = cpu->mem_map.boot_rom_end - addr;
      ret = memory_read (core, addr, buf, req_size);
    }
  else if (IN_MAP (addr, cpu->mem_map.l2_sram))
    {
      if (addr + req_size > cpu->mem_map.l2_sram_end)
	req_size = cpu->mem_map.l2_sram_end - addr;

      ret = memory_read (core, addr, buf, req_size);
    }
  else
    {
      ret = bfin_read_inv_mem (core, addr, buf, &req_size);
    }

done:

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_read_mem () through Core [%d]",
	    bfin_target.name, cpu->first_core + core);

  *actual_size = req_size;

  if (ret < 0)
    return RP_VAL_TARGETRET_ERR;
  else
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_write_mem (uint64_t addr, uint8_t *buf, int write_size)
{
  bfin_core *c;
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
		cpu->first_core + cpu->general_core,
		cpu->cores[cpu->general_core].
		is_locked ? "locked" : "CORE FAULT", addr);
      return RP_VAL_TARGETRET_ERR;
    }

  /* If general_core is available, use it when needed.
     Otherwise choose any other available core.  */

  avail_core = cpu->general_core;

  if (cpu->cores[avail_core].is_locked
      || cpu->cores[avail_core].is_corefault)
    for_each_core (i, c)
      {
	if (!c->is_locked && !c->is_corefault && !c->is_running)
	  {
	    avail_core = i;
	    break;
	  }

	if (i == cpu->core_num)
	  {
	    bfin_log (RP_VAL_LOGLEVEL_ERR,
		      "%s: no core available to write memory",
		      bfin_target.name);
	    return RP_VAL_TARGETRET_ERR;
	  }
      }

  emupc_reset ();

  if (!IN_MAP (addr, cpu->mem_map.l1))
    goto skip_l1;

  for_each_core (i, c)
    {
      /* Use the core for its L1 memory if possible.  */
      if (!c->is_locked && !c->is_corefault && !c->is_running)
	core = i;
      else
	core = avail_core;

      if (IN_MAP (addr, c->l1_map->l1))
	core_cache_status_get (i);
      else
	continue;

      if (IN_MAP (addr, c->l1_map->l1_code) &&
	  ((!c->l1_code_cache_enabled &&
	    (c->l1_map->l1_code_end == c->l1_map->l1_code_cache)
	    && (end = c->l1_map->l1_code_cache_end))
	   || (end = c->l1_map->l1_code_end))
	  && addr + write_size <= end)
	{
	  ret = sram_write (core, addr, buf, write_size, i != core);

	  if (i == core)
	    icache_flush (i, addr, write_size);

	  if (addr == c->l1_map->l1_code
	      && bfin_unlock_on_load && c->is_locked)
	    {
	      uint16_t sica_syscr;

	      bfin_log (RP_VAL_LOGLEVEL_INFO,
			"%s: [%d] unlocking...", bfin_target.name, cpu->first_core + i);

	      sica_syscr = mmr_read (cpu->core_a, SICA_SYSCR, 2);
	      sica_syscr &= ~SICA_SYSCR_COREB_SRAM_INIT;
	      mmr_write (cpu->core_a, SICA_SYSCR, sica_syscr, 2);
	      core_check_emuready (i);
	      c->is_locked = 0;
	      core_wpu_init (i);
	      if (bfin_enable_dcache)
		{
		  core_dcache_enable (i, bfin_enable_dcache);
		  c->l1_data_a_cache_enabled = 1;
		  c->l1_data_b_cache_enabled = 1;
		}
	      if (bfin_enable_icache)
		{
		  core_icache_enable (i);
		  c->l1_code_cache_enabled = 1;
		}

	      bfin_log (RP_VAL_LOGLEVEL_INFO,
			"%s: [%d] done", bfin_target.name, cpu->first_core + i);
	    }

	  goto done;
	}
      else if (IN_MAP (addr, c->l1_map->l1_code_cache) &&
	       addr + write_size <= c->l1_map->l1_code_cache_end)
	{
	  if (c->l1_code_cache_enabled)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: [%d] cannot write L1 icache when enabled [0x%08llX] size %d",
			bfin_target.name, cpu->first_core + i, addr, write_size);
	      return RP_VAL_TARGETRET_ERR;
	    }

	  ret = sram_write (core, addr, buf, write_size, i != core);
	  goto done;
	}
      else if (IN_MAP (addr, c->l1_map->l1_data_a) &&
	       ((!c->l1_data_a_cache_enabled &&
		 (c->l1_map->l1_data_a_end == c->l1_map->l1_data_a_cache)
		 && (end = c->l1_map->l1_data_a_cache_end))
		|| (end = c->l1_map->l1_data_a_end))
	       && addr + write_size <= end)
	{
	  ret = core_memory_write (core, addr, buf, write_size, i == core);
	  goto done;
	}
      else if (IN_MAP (addr, c->l1_map->l1_data_a_cache) &&
	       addr + write_size <= c->l1_map->l1_data_a_cache_end)
	{
	  if (c->l1_data_a_cache_enabled)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: [%d] cannot write L1 dcache when enabled [0x%08llX] size %d",
			bfin_target.name, cpu->first_core + i, addr, write_size);
	      return RP_VAL_TARGETRET_ERR;
	    }

	  ret = core_memory_write (core, addr, buf, write_size, i == core);
	  goto done;
	}
      else if (IN_MAP (addr, c->l1_map->l1_data_b) &&
	       ((!c->l1_data_b_cache_enabled &&
		 (c->l1_map->l1_data_b_end == c->l1_map->l1_data_b_cache)
		 && (end = c->l1_map->l1_data_b_cache_end))
		|| (end = c->l1_map->l1_data_b_end))
	       && addr + write_size <= end)
	{
	  ret = core_memory_write (core, addr, buf, write_size, i == core);
	  goto done;
	}
      else if (IN_MAP (addr, c->l1_map->l1_data_b_cache) &&
	       addr + write_size <= c->l1_map->l1_data_b_cache_end)
	{
	  if (c->l1_data_b_cache_enabled)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: [%d] cannot write L1 dcache when enabled [0x%08llX] size %d",
			bfin_target.name, cpu->first_core + i, addr, write_size);
	      return RP_VAL_TARGETRET_ERR;
	    }

	  ret = core_memory_write (core, addr, buf, write_size, i == core);
	  goto done;
	}
      else if (i == core && IN_MAP (addr, c->l1_map->l1_scratch) &&
	       addr + write_size <= c->l1_map->l1_scratch_end)
	{
	  ret = memory_write (core, addr, buf, write_size);
	  goto done;
	}
      else if (IN_MAP (addr, c->l1_map->l1))
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] cannot write reserved L1 [0x%08llX] size %d",
		    bfin_target.name, cpu->first_core + i, addr, write_size);
	  return RP_VAL_TARGETRET_ERR;
	}
    }

 skip_l1:

  /* TODO  Accurately check MMR validity.  */

  if (addr >= cpu->mem_map.coremmr)
    core = cpu->general_core;
  else
    core = avail_core;

  if ((IN_RANGE (addr, cpu->mem_map.sysmmr, cpu->mem_map.coremmr) &&
       addr + write_size > cpu->mem_map.coremmr)
      || (addr >= cpu->mem_map.coremmr && addr + write_size < addr))
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] bad MMR addr or size [0x%08llX] size %d",
		bfin_target.name, cpu->first_core + core, addr, write_size);
      return RP_VAL_TARGETRET_ERR;
    }

  if (addr >= cpu->mem_map.sysmmr)
    {
      if (write_size != 2 && write_size != 4)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] bad MMR size [0x%08llX] size %d",
		    bfin_target.name, cpu->first_core + core, addr, write_size);
	  return RP_VAL_TARGETRET_ERR;
	}

      if ((addr & 0x1) == 1)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] odd MMR addr [0x%08llX]",
		    bfin_target.name, cpu->first_core + core, addr);
	  return RP_VAL_TARGETRET_ERR;
	}

      value = *buf++;
      value |= (*buf++) << 8;
      if (write_size == 4)
	{
	  value |= (*buf++) << 16;
	  value |= (*buf++) << 24;
	}
      mmr_write (core, addr, value, write_size);
      ret = 0;
      goto done;
    }

  if ((IN_MAP (addr, cpu->mem_map.sdram) &&
       addr + write_size <= cpu->mem_map.sdram_end)
      || (IN_MAP (addr, cpu->mem_map.async_mem) &&
	  addr + write_size <= cpu->mem_map.async_mem_end)
      || (IN_MAP (addr, cpu->mem_map.l2_sram) &&
	  addr + write_size <= cpu->mem_map.l2_sram_end))
    {
      ret = memory_write (core, addr, buf, write_size);

      for_each_core (i, c)
	if (!c->is_locked && !c->is_corefault && !c->is_running)
	  icache_flush (i, addr, write_size);

    }
  else
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] cannot write memory [0x%08llX]",
		bfin_target.name, cpu->first_core + core, addr);
      return RP_VAL_TARGETRET_ERR;
    }

done:

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_write_mem () through Core [%d]",
	    bfin_target.name, cpu->first_core + core);

  if (ret < 0)
    return RP_VAL_TARGETRET_ERR;

  core_check_emuready (core);

  return RP_VAL_TARGETRET_OK;
}


/* Target method */
static int
bfin_resume_from_current (int step, int sig)
{
  bfin_core *c;
  int i, ret;
  uint8_t buf[2];
  int size;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_resume_from_current (%s, %d)",
	    bfin_target.name, (step) ? "step" : "run", sig);

  assert (cpu);

  emupc_reset ();

  for_each_core (i, c)
    {
      if (c->is_locked)
	continue;

      if (c->leave_stopped)
	continue;

      if (c->is_running)
	continue;

      if (c->status_pending_p
	  && c->pending_is_breakpoint)
	{
	  ret = bfin_read_mem (c->pending_stop_pc, buf, 2, &size);
	  assert (ret == RP_VAL_TARGETRET_OK && size == 2);

	  if (buf[0] != bfin_breakpoint_16[0]
	      || buf[1] != bfin_breakpoint_16[1])
	    {
	      /* The breakpoint is gone. Consume the pending status.  */
	      c->pending_is_breakpoint = 0;
	      c->status_pending_p = 0;
	    }
	}
      if (c->status_pending_p)
	return RP_VAL_TARGETRET_OK;
    }

  for_each_core (i, c)
    {
      if (c->is_locked)
	continue;

      if (c->leave_stopped)
	continue;

      if (c->is_running)
	continue;

      if (i == cpu->continue_core)
	{
	  if (step && !c->is_stepping)
	    {
	      core_dbgctl_bit_set_esstep (i, UPDATE);
	      c->is_stepping = 1;
	    }
	  else if (!step && c->is_stepping)
	    {
	      core_dbgctl_bit_clear_esstep (i, UPDATE);
	      c->is_stepping = 0;
	    }

	  c->is_running = 1;
	  c->is_interrupted = 0;
	  c->dmem_control_valid_p = 0;
	  c->imem_control_valid_p = 0;
	  core_emulation_return (i);
	}
      else if (!c->leave_stopped)
	{
	  if (c->is_stepping)
	    {
	      core_dbgctl_bit_clear_esstep (i, UPDATE);
	      c->is_stepping = 0;
	    }

	  c->is_running = 1;
	  c->is_interrupted = 0;
	  c->dmem_control_valid_p = 0;
	  c->imem_control_valid_p = 0;
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

  emupc_reset ();

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
	    bfin_target.name, cpu->first_core + core, emucause_infos[cause], rete, fp);
}

/* Target method */
static int
bfin_wait_partial (int first,
		   char *status_string,
		   int status_string_len,
		   out_func of, int *implemented, int *more)
{
  uint16_t cause;
  char *cp;
  int sig;
  uint32_t pc, fp;
  bfin_core *c;
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

  /* Check for pending jtag communications */
  for (i = 0; i < 100; ++i)
    if (jc_loop () <= 0)
      break;

  /* If we have any interesting pending event,
     report it instead of resume.  */
  for_each_core (i, c)
    if (!c->is_locked
	&& !c->leave_stopped && c->status_pending_p)
      {
	sprintf (status_string, "T%02d", c->pending_signal);
	cp = &status_string[3];
	if (c->is_corefault)
	  {
	    pc = c->pending_stop_pc;
	    cp = bfin_out_treg_value (cp, BFIN_PC_REGNUM, pc);
	  }
	else
	  {
	    if (c->wpstat & 0xc0)
	      core_single_step (i);
	    if (c->wpstat & 0xff)
	      core_wpstat_clear (i);
	    pc = core_register_get (i, REG_RETE);
	    fp = core_register_get (i, REG_FP);
	    cp = bfin_out_treg_value (cp, BFIN_PC_REGNUM, pc);
	    cp = bfin_out_treg_value (cp, BFIN_FP_REGNUM, fp);
	  }
	if (cpu->core_num > 1)
	  sprintf (cp, "thread:%x;", THREAD_ID (i));

	c->status_pending_p = 0;

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

  for_each_core (i, c)
    {
      if (c->leave_stopped)
	continue;

      if (core_dbgstat_is_emuready (i)
	  && c->is_locked)
	{
	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "%s: [%d] core unlocking...", bfin_target.name, cpu->first_core + i);

	  /* Set pending hardware breakpoints and watchpoints.  */

	  core_wpu_init (i);

	  for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
	    if (c->hwbps[j] != -1)
	      wpu_set_wpia (i, j, c->hwbps[j], 1);

	  if ((bfin_force_range_wp
	       || c->hwwps[0].len > 4)
	      && c->hwwps[0].mode != WPDA_DISABLE)
	    wpu_set_wpda (i,
			  0,
			  c->hwwps[0].addr,
			  c->hwwps[0].len,
			  1, c->hwwps[0].mode);
	  else
	    for (j = 0; j < RP_BFIN_MAX_HWWATCHPOINTS; j++)
	      if (c->hwwps[j].mode != WPDA_DISABLE)
		wpu_set_wpda (i,
			      j,
			      c->hwwps[j].addr,
			      c->hwwps[j].len,
			      0, c->hwwps[j].mode);

	  if (bfin_enable_dcache)
	    {
	      core_dcache_enable (i, bfin_enable_dcache);
	      c->l1_code_cache_enabled = 1;
	    }
	  if (bfin_enable_icache)
	    {
	      core_icache_enable (i);
	      c->l1_data_a_cache_enabled = 1;
	      c->l1_data_b_cache_enabled = 1;
	    }

	  core_emulation_return (i);
	  c->is_locked = 0;
	  c->is_running = 1;

	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "%s: [%d] done", bfin_target.name, cpu->first_core + i);
	}
      if (core_dbgstat_is_core_fault (i)
	  || core_dbgstat_is_emuready (i))
	{
	  *more = FALSE;
	}
      else if (core_dbgstat_is_idle (i))
	{
	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "%s: [%d] core is in idle mode", bfin_target.name, cpu->first_core + i);
	}
      else if (core_dbgstat_is_in_reset (i) && !core_sticky_in_reset (i))
	{
	  jc_state_reset ();
	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "%s: [%d] core is currently being reset",
		    bfin_target.name, cpu->first_core + i);
	}
    }

  /* No core stopped. keep waiting. Otherwise, stop all. */

  if (*more == TRUE)
    return RP_VAL_TARGETRET_OK;
  else
    {
      emulation_trigger ();
      for_each_core (i, c)
	c->is_running = 0;
    }

  /* All cores are stopped. Check their status.  */

  emupc_get (1);
  emupc_reset ();
  dbgstat_get ();

  for_each_core (i, c)
    {
      urj_part_t *part = cpu->chain->parts->parts[cpu->first_core + i];

      if (c->leave_stopped)
	continue;

      if (c->is_locked)
	continue;

      cause = core_dbgstat_emucause (i);

      if (core_dbgstat_is_core_fault (i))
	{
	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "%s: [%d] a double fault has occured EMUPC [0x%08X]",
		    bfin_target.name, cpu->first_core + i, BFIN_PART_EMUPC_ORIG (part));

	  sig = RP_SIGNAL_TRAP;
	  c->is_corefault = 1;
	  c->status_pending_p = 1;
	  c->pending_is_breakpoint = 0;
	  c->pending_signal = sig;
	  c->pending_stop_pc = BFIN_PART_EMUPC_ORIG (part);
	}
      else if (core_dbgstat_is_emuready (i))
	{
	  pc = core_register_get (i, REG_RETE);
	  fp = core_register_get (i, REG_FP);

	  core_wpstat_get (i);

	  if (c->wpstat & 0xff)
	    {
	      sig = RP_SIGNAL_TRAP;
	      cause = EMUCAUSE_WATCHPOINT;
	    }
	  else
	    switch (cause)
	      {
	      case EMUCAUSE_EMUEXCPT:
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
		if (c->is_interrupted)
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

	  if (cause != EMUCAUSE_EMUIN || c->is_interrupted)
	    {
	      c->status_pending_p = 1;

	      /* We only have to distinguish the breakpoint added by GDB.
	         GDB only use software breakpoint for its own purpose.  */

	      if (cause == EMUCAUSE_EMUEXCPT)
		c->pending_is_breakpoint = 1;
	      else
		c->pending_is_breakpoint = 0;

	      c->pending_signal = sig;
	      c->pending_stop_pc = pc;
	    }
	}
      else
	{
	  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		    "%s: [%d] unhandled debug status [0x%08X] EMUPC [0x%08X]",
		    bfin_target.name,
		    cpu->first_core + i, BFIN_PART_DBGSTAT (part), BFIN_PART_EMUPC_ORIG (part));
	}
    }

  for_each_core (i, c)
    {
      if (c->leave_stopped)
	continue;

      if (!c->status_pending_p)
	continue;

      sprintf (status_string, "T%02d", c->pending_signal);
      cp = &status_string[3];
      if (c->is_corefault)
	{
	  pc = c->pending_stop_pc;
	  cp = bfin_out_treg_value (cp, BFIN_PC_REGNUM, pc);
	}
      else
	{
	  if (c->wpstat & 0xff)
	    {
	      if (c->wpstat & 0xc0)
		core_single_step (i);
	      core_wpstat_clear (i);
	    }
	  pc = core_register_get (i, REG_RETE);
	  fp = core_register_get (i, REG_FP);

	  cp = bfin_out_treg_value (cp, BFIN_PC_REGNUM, pc);
	  cp = bfin_out_treg_value (cp, BFIN_FP_REGNUM, fp);
	}
      if (cpu->core_num > 1)
	sprintf (cp, "thread:%x;", THREAD_ID (i));

      c->status_pending_p = 0;

      cpu->general_core = i;
      cpu->continue_core = i;

      /* Consume all interrupts.  */
      for_each_core (i, c)
	if (c->status_pending_p && c->is_interrupted &&
	    c->pending_signal == RP_SIGNAL_INTERRUPT)
	  c->status_pending_p = 0;

      break;
    }

  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_wait (char *status_string,
	   int status_string_len, out_func of, int *implemented)
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
		 int max_num, int *num, int *done)
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
bfin_crc_query (uint64_t addr, int len, uint32_t *val)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_crc_query()", bfin_target.name);

  return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int
_bfin_raw_query_append (char **out_buf, int *out_buf_size, const char *fmt, ...)
{
  int ret;
  va_list args;

  va_start (args, fmt);
  ret = vsnprintf (*out_buf, *out_buf_size, fmt, args);
  va_end (args);

  if (ret < 0)
    return RP_VAL_TARGETRET_ERR;

  *out_buf += ret;
  *out_buf_size -= ret;
  return RP_VAL_TARGETRET_OK;
}
static int
__bfin_raw_query_append_mem (char **out_buf, int *out_buf_size, const char *type,
			     uint32_t start, uint32_t len)
{
  if (start == 0)
    return RP_VAL_TARGETRET_OK;
  return _bfin_raw_query_append (out_buf, out_buf_size,
	"<memory type=\"%s\" start=\"%#x\" length=\"%#x\"/>",
	type, start, len);
}
#define __bfin_raw_query_append_mem(b, s, t, m, l) \
  do { \
    int __ret = __bfin_raw_query_append_mem(b, s, t, m, l); \
    if (__ret != RP_VAL_TARGETRET_OK) \
      return __ret; \
  } while (0)
#define _bfin_raw_query_append_mem(b, s, t, m) \
  __bfin_raw_query_append_mem(b, s, t, m, MAP_LEN (m))

static int
bfin_raw_query (char *in_buf, char *out_buf, int out_buf_size)
{
  bfin_core *c;
  int i, ret;
  const char q_memory_map[] = "qXfer:memory-map:read::";

  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_raw_query ()", bfin_target.name);

  /* http://sourceware.org/gdb/current/onlinedocs/gdb/Memory-Map-Format.html#Memory-Map-Format */
  /* XXX: this doesn't handle the offset/len part of the packet, but
          gdb will usually (always?) use offset of 0 with a len of 4k,
          and that's plenty for our map below ...  */
  if (!strncmp (in_buf, q_memory_map, sizeof (q_memory_map) - 1))
    {
      const bfin_mem_map *mem = &cpu->mem_map;
      char **b = &out_buf;
      int *s = &out_buf_size;

      ret = _bfin_raw_query_append (b, s, "l<memory-map>");
      if (ret < 0)
	return RP_VAL_TARGETRET_ERR;

      for_each_core (i, c)
	{
	  const bfin_l1_map *l1 = c->l1_map;
	  /* XXX: Maybe label the caches as ROM ?  */
	  _bfin_raw_query_append_mem (b, s, "ram", l1->l1_data_a);
	  _bfin_raw_query_append_mem (b, s, "ram", l1->l1_data_a_cache);
	  _bfin_raw_query_append_mem (b, s, "ram", l1->l1_data_b);
	  _bfin_raw_query_append_mem (b, s, "ram", l1->l1_data_b_cache);
	  _bfin_raw_query_append_mem (b, s, "ram", l1->l1_code);
	  _bfin_raw_query_append_mem (b, s, "ram", l1->l1_code_cache);
	  _bfin_raw_query_append_mem (b, s, "rom", l1->l1_code_rom);
	  _bfin_raw_query_append_mem (b, s, "ram", l1->l1_scratch);
	}

      _bfin_raw_query_append_mem (b, s, "ram", mem->sdram);
      _bfin_raw_query_append_mem (b, s, "ram", mem->async_mem);
      _bfin_raw_query_append_mem (b, s, "rom", mem->boot_rom);
      _bfin_raw_query_append_mem (b, s, "ram", mem->l2_sram);
      __bfin_raw_query_append_mem (b, s, "ram", mem->sysmmr, 0x200000);
      __bfin_raw_query_append_mem (b, s, "ram", mem->coremmr, 0x200000);

      /* _bfin_raw_query_append_mem (b, s, "flash", mem->flash); */

      return _bfin_raw_query_append (b, s, "</memory-map>");
    }

  return RP_VAL_TARGETRET_NOSUPP;
}

/* Target method */
static int
bfin_threadinfo_query (int first, char *out_buf, int out_buf_size)
{
  static int i;

  if (first)
    i = cpu->core_num - 1;
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
			    int out_buf_size)
{
  int core;
  char *cp;
  urj_part_t *part;

  core = PART_NO (thread->val);

  if (core < 0 || core >= cpu->core_num)
    return RP_VAL_TARGETRET_ERR;

  cp = out_buf;
  sprintf (cp, "%s", cpu->cores[core].name);
  cp += strlen (cp);

  if (cpu->cores[core].is_locked)
    {
      sprintf (cp, " Locked");
      cp += strlen (cp);
    }

  part = cpu->chain->parts->parts[cpu->first_core + core];
  sprintf (cp, " DBGSTAT [0x%04X]", BFIN_PART_DBGSTAT (part));

  if (strlen (out_buf) >= out_buf_size)
    return RP_VAL_TARGETRET_ERR;
  else
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_supported_query (char *out_buf, int out_buf_size)
{
  bfin_core *c;
  int i;
  int size;

  /* 0x4000 is the largest packet size GDB would like.  */
  size = MIN (0x4000, RP_PARAM_INOUTBUF_SIZE - 1);
  for_each_core (i, c)
    size = MIN (size, MAP_LEN (c->l1_map->l1_data_a));

  i = snprintf (out_buf, out_buf_size,
		"PacketSize=%x"
		";qXfer:memory-map:read+",
		size);

  if (i < 0)
    return RP_VAL_TARGETRET_ERR;
  else
    return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_add_break (int type, uint64_t addr, int len)
{
  bfin_core *c;
  int mode;
  int i, j;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_add_break (%d, 0x%08llx, %d)",
	    bfin_target.name, type, addr, len);

  emupc_reset ();

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
      if (IN_MAP (addr, cpu->mem_map.l1))
	{
	  for_each_core (i, c)
	    if (IN_MAP (addr, c->l1_map->l1))
	      {
		for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
		  if (c->hwbps[j] == -1)
		    {
		      c->hwbps[j] = addr;
		      if (!c->is_locked)
			wpu_set_wpia (i, j, addr, 1);
		      return RP_VAL_TARGETRET_OK;
		    }

		bfin_log (RP_VAL_LOGLEVEL_ERR,
			  "%s: [%d] no more hardware breakpoint available",
			  bfin_target.name, cpu->first_core + i);
		return RP_VAL_TARGETRET_ERR;
	      }

	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: no core to set hardware breakpoint at 0x%08llX",
		    bfin_target.name, addr);
	  return RP_VAL_TARGETRET_ERR;
	}

      for_each_core (i, c)
	{
	  for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
	    if (c->hwbps[j] == -1)
	      break;

	  if (j == RP_BFIN_MAX_HWBREAKPOINTS)
	    break;
	}

      if (i < cpu->core_num)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] no more hardware breakpoint available",
		    bfin_target.name, cpu->first_core + i);
	  return RP_VAL_TARGETRET_ERR;
	}

      for_each_core (i, c)
	for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
	  if (c->hwbps[j] == -1)
	    {
	      c->hwbps[j] = addr;
	      if (!c->is_locked)
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

  if (IN_MAP (addr, cpu->mem_map.l1))
    {
      for_each_core (i, c)
	if (IN_MAP (addr, c->l1_map->l1))
	  {
	    if (!bfin_force_range_wp
		&& len <= 4
		&& (c->hwwps[0].addr == -1
		    || c->hwwps[1].addr == -1))
	      {
		j = c->hwwps[0].addr == -1 ? 0 : 1;
		c->hwwps[j].addr = addr;
		c->hwwps[j].len = len;
		c->hwwps[j].mode = mode;
		if (!c->is_locked)
		  wpu_set_wpda (i, j, addr, len, 0, mode);
		return RP_VAL_TARGETRET_OK;
	      }
	    else if ((bfin_force_range_wp || len > 4)
		     && c->hwwps[0].addr == -1
		     && c->hwwps[1].addr == -1)
	      {
		c->hwwps[0].addr = addr;
		c->hwwps[0].len = len;
		c->hwwps[0].mode = mode;
		c->hwwps[1].addr = addr;
		c->hwwps[1].len = len;
		c->hwwps[1].mode = mode;
		if (!c->is_locked)
		  wpu_set_wpda (i, 0, addr, len, 1, mode);
		return RP_VAL_TARGETRET_OK;
	      }

	    bfin_log (RP_VAL_LOGLEVEL_ERR,
		      "%s: [%d] no more hardware watchpoint available",
		      bfin_target.name, cpu->first_core + i);
	    return RP_VAL_TARGETRET_ERR;
	  }

      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: no core to set hardware watchpoint at 0x%08llX",
		bfin_target.name, addr);
      return RP_VAL_TARGETRET_ERR;
    }

  for_each_core (i, c)
    if (!bfin_force_range_wp
	&& len <= 4
	&& c->hwwps[0].addr != -1
	&& c->hwwps[1].addr != -1)
      break;
    else if ((bfin_force_range_wp || len > 4)
	     && (c->hwwps[0].addr != -1
		 || c->hwwps[1].addr != -1))
      break;

  if (i < cpu->core_num)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] no more hardware breakpoint available",
		bfin_target.name, cpu->first_core + i);
      return RP_VAL_TARGETRET_ERR;
    }

  for_each_core (i, c)
    if (!bfin_force_range_wp && len <= 4)
      {
	j = c->hwwps[0].addr == -1 ? 0 : 1;

	c->hwwps[j].addr = addr;
	c->hwwps[j].len = len;
	c->hwwps[j].mode = mode;
	if (!c->is_locked)
	  wpu_set_wpda (i, j, addr, len, 0, mode);
      }
    else
      {
	c->hwwps[0].addr = addr;
	c->hwwps[0].len = len;
	c->hwwps[0].mode = mode;
	c->hwwps[1].addr = addr;
	c->hwwps[1].len = len;
	c->hwwps[1].mode = mode;
	if (!c->is_locked)
	  wpu_set_wpda (i, 0, addr, len, 1, mode);
      }
  return RP_VAL_TARGETRET_OK;
}

/* Target method */
static int
bfin_remove_break (int type, uint64_t addr, int len)
{
  int mode;
  bfin_core *c;
  int i, j;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_remove_break(%d, 0x%08llx, %d)",
	    bfin_target.name, type, addr, len);

  emupc_reset ();

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
      if (IN_MAP (addr, cpu->mem_map.l1))
	{
	  for_each_core (i, c)
	    if (IN_MAP (addr, c->l1_map->l1))
	      {
		for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
		  if (c->hwbps[j] == addr)
		    {
		      c->hwbps[j] = -1;
		      if (!c->is_locked)
			wpu_set_wpia (i, j, addr, 0);
		      return RP_VAL_TARGETRET_OK;
		    }

		bfin_log (RP_VAL_LOGLEVEL_ERR,
			  "%s: [%d] no hardware breakpoint at 0x%08llX",
			  bfin_target.name, cpu->first_core + i, addr);
		return RP_VAL_TARGETRET_ERR;
	      }

	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: no core has hardware breakpoint at 0x%08llX",
		    bfin_target.name, addr);
	  return RP_VAL_TARGETRET_ERR;
	}

      for_each_core (i, c)
	{
	  for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
	    if (c->hwbps[j] == addr)
	      break;

	  if (j == RP_BFIN_MAX_HWBREAKPOINTS)
	    break;
	}

      if (i < cpu->core_num)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] no hardware breakpoint at 0x%08llX",
		    bfin_target.name, cpu->first_core + i, addr);
	  return RP_VAL_TARGETRET_ERR;
	}

      for_each_core (i, c)
	for (j = 0; j < RP_BFIN_MAX_HWBREAKPOINTS; j++)
	  if (c->hwbps[j] == addr)
	    {
	      c->hwbps[j] = -1;
	      if (!c->is_locked)
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

  if (IN_MAP (addr, cpu->mem_map.l1))
    {
      for_each_core (i, c)
	if (IN_MAP (addr, c->l1_map->l1))
	  {
	    if (!bfin_force_range_wp
		&& len <= 4
		&& ((c->hwwps[0].addr == addr
		     && c->hwwps[0].len == len
		     && c->hwwps[0].mode == mode)
		    || (c->hwwps[1].addr == addr
			&& c->hwwps[1].len == len
			&& c->hwwps[1].mode == mode)))
	      {
		j = c->hwwps[0].addr == addr ? 0 : 1;
		c->hwwps[j].addr = -1;
		c->hwwps[j].len = 0;
		c->hwwps[j].mode = WPDA_DISABLE;
		if (!c->is_locked)
		  wpu_set_wpda (i, j, addr, len, 0, WPDA_DISABLE);
		return RP_VAL_TARGETRET_OK;
	      }
	    else if ((bfin_force_range_wp || len > 4)
		     && c->hwwps[0].addr == addr
		     && c->hwwps[0].len == len
		     && c->hwwps[0].mode == mode
		     && c->hwwps[1].addr == addr
		     && c->hwwps[1].len == len
		     && c->hwwps[1].mode == mode)
	      {
		c->hwwps[0].addr = -1;
		c->hwwps[0].len = 0;
		c->hwwps[0].mode = WPDA_DISABLE;
		c->hwwps[1].addr = -1;
		c->hwwps[1].len = 0;
		c->hwwps[1].mode = WPDA_DISABLE;
		if (!c->is_locked)
		  wpu_set_wpda (i, 0, addr, len, 1, WPDA_DISABLE);
		return RP_VAL_TARGETRET_OK;
	      }

	    bfin_log (RP_VAL_LOGLEVEL_ERR,
		      "%s: [%d] no hardware watchpoint at 0x%08llX length %d",
		      bfin_target.name, cpu->first_core + i, addr, len);
	    return RP_VAL_TARGETRET_ERR;
	  }

      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: no core has hardware watchpoint at 0x%08llX length %d",
		bfin_target.name, addr, len);
      return RP_VAL_TARGETRET_ERR;
    }

  for_each_core (i, c)
    if (!bfin_force_range_wp
	&& len <= 4
	&& (c->hwwps[0].addr != addr
	    || c->hwwps[0].len != len
	    || c->hwwps[0].mode != mode)
	&& (c->hwwps[1].addr != addr
	    || c->hwwps[1].len != len
	    || c->hwwps[1].mode != mode))
      break;
    else if ((bfin_force_range_wp || len > 4)
	     && (c->hwwps[0].addr != addr
		 || c->hwwps[0].len != len
		 || c->hwwps[0].mode != mode
		 || c->hwwps[1].addr != addr
		 || c->hwwps[1].len != len
		 || c->hwwps[1].mode != mode))
      break;

  if (i < cpu->core_num)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] no hardware breakpoint at 0x%08llX length %d",
		bfin_target.name, cpu->first_core + i, addr, len);
      return RP_VAL_TARGETRET_ERR;
    }

  for_each_core (i, c)
    if (!bfin_force_range_wp && len <= 4)
      {
	j = (c->hwwps[0].addr == addr
	     && c->hwwps[0].len == len
	     && c->hwwps[0].mode == mode) ? 0 : 1;
	c->hwwps[j].addr = -1;
	c->hwwps[j].len = 0;
	c->hwwps[j].mode = WPDA_DISABLE;
	if (!c->is_locked)
	  wpu_set_wpda (i, j, addr, len, 0, WPDA_DISABLE);
      }
    else
      {
	c->hwwps[0].addr = -1;
	c->hwwps[0].len = 0;
	c->hwwps[0].mode = WPDA_DISABLE;
	c->hwwps[1].addr = -1;
	c->hwwps[1].len = 0;
	c->hwwps[1].mode = WPDA_DISABLE;
	if (!c->is_locked)
	  wpu_set_wpda (i, 0, addr, len, 1, WPDA_DISABLE);
      }
  return RP_VAL_TARGETRET_OK;
}

/* command: reset proc */
static int bfin_rcmd_reset(int argc, char *argv[], out_func of, data_func df)
{
  software_reset (cpu->chain);
  return RP_VAL_TARGETRET_OK;
}

static int bfin_rcmd_jc_reset(int argc, char *argv[], out_func of, data_func df)
{
  jc_state_reset ();
  return RP_VAL_TARGETRET_OK;
}

static int bfin_rcmd_urjtag(int argc, char *argv[], out_func of, data_func df)
{
  /* skip argv[0] which is "urjtag" */
  ++argv;

  /* abusing these hooks like this seems kind of hacky */
  urjtag_of = of;
  urj_log_state.out_vprintf = urj_log_state.err_vprintf = bfin_urjtag_vprintf;
  urj_cmd_run (cpu->chain, argv);
  urj_log_state.out_vprintf = urj_log_state.err_vprintf = vprintf;

  return RP_VAL_TARGETRET_OK;
}

#define RCMD(name, hlp) {#name, bfin_rcmd_##name, hlp}
static const RCMD_TABLE bfin_remote_commands[] =
{
  RCMD(reset, "Reset processor"),
  RCMD(jc_reset, "Reset JTAG console counters"),
  RCMD(urjtag, "Run urjtag commands"),
  {0,0,0},
};
