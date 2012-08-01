/* Copyright (C) 2008-2011 Analog Devices, Inc.

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
#include <math.h>
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
#include <urjtag/sdu.h>

#ifndef MIN
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif
#ifndef MAX
#define MAX(x,y) ((x) < (y) ? (y) : (x))
#endif

#define UNUSED __attribute__((unused))

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

/* BF60x CGU */
#define CGU0_CTL			0xffca8000
#define CGU0_STAT			0xffca8004
#define CGU0_DIV			0xffca8008

#define CGU0_CTL_DF_BITP		0
#define CGU0_CTL_DF_BITM		0x1
#define CGU0_CTL_MSEL_BITP		8
#define CGU0_CTL_MSEL_BITM		0x7f

#define CGU0_STAT_PLLEN			0
#define CGU0_STAT_PLLBP			1
#define CGU0_STAT_PLLLK			2
#define CGU0_STAT_CLKSALGN		3
#define CGU0_STAT_CCBF0EN		4
#define CGU0_STAT_CCBF1EN		5
#define CGU0_STAT_SCBF0EN		6
#define CGU0_STAT_SCBF1EN		7
#define CGU0_STAT_DCBFEN		8
#define CGU0_STAT_OCBFEN		9
#define CGU0_STAT_ADRERR		16
#define CGU0_STAT_LWERR			17
#define CGU0_STAT_DIVERR		18
#define CGU0_STAT_WDFMSERR		19
#define CGU0_STAT_WDIVERR		20
#define CGU0_STAT_PLLLKERR		21

#define CGU0_DIV_CSEL_BITP		0
#define CGU0_DIV_CSEL_BITM		0x1f
#define CGU0_DIV_S0SEL_BITP		5
#define CGU0_DIV_S0SEL_BITM		0x7
#define CGU0_DIV_SYSSEL_BITP		8
#define CGU0_DIV_SYSSEL_BITM		0x1f
#define CGU0_DIV_S1SEL_BITP		13
#define CGU0_DIV_S1SEL_BITM		0x7
#define CGU0_DIV_DSEL_BITP		16
#define CGU0_DIV_DSEL_BITM		0x1f
#define CGU0_DIV_OSEL_BITP		22
#define CGU0_DIV_OSEL_BITM		0x7f
#define CGU0_DIV_UPDT			30

/* BF60x DDR2 */
#define DDR0_ID				0xffc80000
#define DDR0_CTL			0xffc80004
#define DDR0_STAT			0xffc80008
#define DDR0_CFG			0xffc80040
#define DDR0_TR0			0xffc80044
#define DDR0_TR1			0xffc80048
#define DDR0_TR2			0xffc8004c
#define DDR0_MRWMR			0xffc8005c
#define DDR0_MR				0xffc80060
#define DDR0_EMR1			0xffc80064
#define DDR0_EMR2			0xffc80068
#define DDR0_EMR3			0xffc8006c
#define DDR0_DLLCTL			0xffc80080

#define DDR0_CTL_INIT			2
#define DDR0_CTL_RD_TO_WR_CYC_BITP	9
#define DDR0_CTL_RD_TO_WR_CYC_BITM	0x7

#define DDR0_STAT_DMC_IDLE		0
#define DDR0_STAT_INIT_DONE		2
#define DDR0_STAT_SRACK			3
#define DDR0_STAT_PDACK			4
#define DDR0_STAT_DPDACK		5
#define DDR0_STAT_DLL_CAL_DONE		13
#define DDR0_STAT_PEND_REF_BITP		16
#define DDR0_STAT_PEND_REF_BITM		0xf
#define DDR0_STAT_PHY_RD_PHASE_BITP	20
#define DDR0_STAT_PHY_RD_PHASE_BITM	0xf

#define DDR0_CFG_IF_WIDTH_BITP		0
#define DDR0_CFG_IF_WIDTH_BITM		0xf
#define DDR0_CFG_SDRAM_WIDTH_BITP	4
#define DDR0_CFG_SDRAM_WIDTH_BITM	0xf
#define DDR0_CFG_SDRAM_SIZE_BITP	8
#define DDR0_CFG_SDRAM_SIZE_BITM	0xf

#define DDR0_TR0_TRCD_BITP		0
#define DDR0_TR0_TRCD_BITM		0xf
#define DDR0_TR0_TWTR_BITP		4
#define DDR0_TR0_TWTR_BITM		0xf
#define DDR0_TR0_TRP_BITP		8
#define DDR0_TR0_TRP_BITM		0xf
#define DDR0_TR0_TRAS_BITP		12
#define DDR0_TR0_TRAS_BITM		0x1f
#define DDR0_TR0_TRC_BITP		20
#define DDR0_TR0_TRC_BITM		0x3f
#define DDR0_TR0_TMRD_BITP		28
#define DDR0_TR0_TMRD_BITM		0xf

#define DDR0_TR1_TREF_BITP		0
#define DDR0_TR1_TREF_BITM		0x3fff
#define DDR0_TR1_TRFC_BITP		16
#define DDR0_TR1_TRFC_BITM		0xff
#define DDR0_TR1_TRRD_BITP		28
#define DDR0_TR1_TRRD_BITM		0x7

#define DDR0_TR2_TFAW_BITP		0
#define DDR0_TR2_TFAW_BITM		0x1f
#define DDR0_TR2_TRTP_BITP		8
#define DDR0_TR2_TRTP_BITM		0xf
#define DDR0_TR2_TWR_BITP		12
#define DDR0_TR2_TWR_BITM		0xf
#define DDR0_TR2_TXP_BITP		16
#define DDR0_TR2_TXP_BITM		0xf
#define DDR0_TR2_TCKE_BITP		20
#define DDR0_TR2_TCKE_BITM		0xf

#define DDR0_MR_MASK			8
#define DDR0_EMR1_MASK			9
#define DDR0_EMR2_MASK			10
#define DDR0_EMR3_MASK			11

#define DDR0_MR_BL_BITP			0
#define DDR0_MR_BL_BITM			0x7
#define DDR0_MR_CL_BITP			4
#define DDR0_MR_CL_BITM			0x7
#define DDR0_MR_TWR_BITP		9
#define DDR0_MR_TWR_BITM		0x7

#define DDR0_DLLCTL_DATA_CYCLE_BITP	8
#define DDR0_DLLCTL_DATA_CYCLE_BITM	0xf

#define SDU0_MSG			0xffc1f080
#define SDU0_MSG_SET			0xffc1f084
#define SDU0_MSG_CLR			0xffc1f088

#define MSG_CALLERROR			0x80000000
#define MSG_CALLBACK			0x40000000
#define MSG_CALLINIT			0x20000000
#define MSG_CALLAPP			0x10000000
#define MSG_HALTONERROR			0x08000000
#define MSG_HALTONCALL			0x04000000
#define MSG_HALTONINIT			0x02000000
#define MSG_HALTONAPP			0x01000000
#define MSG_L2_INIT			0x00400000
#define MSG_C1L1_INIT			0x00020000
#define MSG_C0L1_INIT			0x00010000

/* BF60x RCU */
#define RCU0_CTL			0xffca6000
#define RCU0_STAT			0xffca6004
#define RCU0_CN_RES			0xffca6008
#define RCU0_CN_STAT			0xffca600c
#define RCU0_FUNIT_DIS			0xffca6010
#define RCU0_FUNIT_STAT			0xffca6014
#define RCU0_CN_CTL_LOCK		0xffca6018
#define RCU0_BCODE			0xffca601c
#define RCU0_CN_CTL0			0xffca6020
#define RCU0_CN_CTL1			0xffca6024
#define RCU0_STAT_BMODE_BITP		8
#define RCU0_STAT_BMODE_BITM		0xf


#define BIT_FIELD_VALUE(VALUE, MMR, BIT_FIELD) \
  (((VALUE) >> MMR##_##BIT_FIELD##_BITP) & MMR##_##BIT_FIELD##_BITM)

#define DDR0_STAT_BIT_FIELD_VALUE(BIT_FIELD) \
  BIT_FIELD_VALUE (ddr0_stat, DDR0_STAT, BIT_FIELD)

#define RCU0_STAT_BIT_FIELD_VALUE(BIT_FIELD) \
  BIT_FIELD_VALUE (rcu0_stat, RCU0_STAT, BIT_FIELD)


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
  BF609_EZKIT,
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
static const bfin_l1_map bf609_0_l1_map = {
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
static const bfin_l1_map bf609_1_l1_map = {
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
static const bfin_mem_map bf609_mem_map = {
  .sdram		= 0,
  .sdram_end		= 0x10000000,
  .async_mem		= 0xb0000000,
  .flash		= 0xb0000000,  /* ??? */
  .flash_end		= 0xb4000000,  /* ??? */
  .async_mem_end	= 0xc0000000,
  .boot_rom		= 0xc8000000,
  .boot_rom_end		= 0xc8008000,
  .l2_sram		= 0xc8080000,
  .l2_sram_end		= 0xc80c0000,
  .l1			= 0xff400000,
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

  /* If a core is dead.  You cannot unlocked it or access it.
     What you need to do is just set its scan to BYPASS.
     You can set it to other scans, but they will not take effect.  */
  unsigned int is_dead:1;
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
  int ddr2;
  uint32_t mdma_s0;
  uint32_t mdma_d0;

  /* The first core of this cpu in the chain.  */
  int first_core;
  /* The number of cores of this cpu.  */
  int core_num;

  /* The core will never be locked.  */
  /* TODO We need change this for BF609.  */
  int core_a;

  int general_core;
  int continue_core;

  /* The part number of SDU. -1 if not SDU.  */
  int sdu;

  bfin_core cores[0];
} bfin_cpu;

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
static int bfin_debug_boot_code = 0;
static int bfin_bf609_bmode1_workaround = 0;
static int bfin_bf609_sdu_mac_read = 0;
static int bfin_bf609_sdu_mac_write = 0;
static int bfin_bf609_check_macrdy = 0;
/* CCLK for BF609, default 500MHz */
static int bfin_bf609_cclk = 0;
/* SCLK for BF609, default to be CCLK/2 */
static int bfin_bf609_sclk = 0;
/* DCLK for BF609, default to be same as SCLK */
static int bfin_bf609_dclk = 0;
/* CAS Latency for BF609 DDR2 configuration */
static int bfin_bf609_ddrcl = 0;
/* The CLKIN for BF609 is 25MHz */
#define CLKIN 25
static int msel;
static int csel;
static int syssel;
static int s0sel;
static int s1sel;
static int dsel;
static int osel;

#define INVALID_MEM_ACCESS_IGNORE     0
#define INVALID_MEM_ACCESS_REJECT     1
#define INVALID_MEM_ACCESS_ALLOW_DMA  2
#define INVALID_MEM_ACCESS_ALLOW_CORE 3
static int invalid_mem_access = INVALID_MEM_ACCESS_IGNORE;


/* Local functions */

static int
bfin_iter_init_alive_core (void)
{
  int j;
  for (j = 0; j < cpu->core_num; j++)
    if (!cpu->cores[j].is_dead)
      break;
  return j;
}

static int
bfin_iter_next_alive_core (int i)
{
  int j;
  for (j = i + 1; j < cpu->core_num; j++)
    if (!cpu->cores[j].is_dead)
      break;
  return j;
}

#define FOR_EACH_ALIVE_CORE(I, C)					\
  for (I = bfin_iter_init_alive_core (),				\
	 C = I == cpu->core_num ? NULL : &cpu->cores[I];		\
       I < cpu->core_num;						\
       I = bfin_iter_next_alive_core (I),				\
	 C = I == cpu->core_num ? NULL : &cpu->cores[I])

#define FOR_EACH_CORE(I, C) \
  for (C = &cpu->cores[I = 0]; I < cpu->core_num; C = &cpu->cores[++I])

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
core_dbgstat_get (int core)
{
  part_dbgstat_get (cpu->chain, cpu->first_core + core);
}

static void
dbgstat_get (void)
{
  int i;
  for (i = 0; i < cpu->core_num; i++)
    if (!cpu->cores[i].is_dead)
      core_dbgstat_get (i);
}

static uint32_t
core_emupc_get (int core, int save)
{
  return part_emupc_get (cpu->chain, cpu->first_core + core, save);
}

static void
emupc_get (int save)
{
  int i;
  for (i = 0; i < cpu->core_num; i++)
    core_emupc_get (i, save);
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
core_emupc_reset (int core)
{
  bfin_core *c = &cpu->cores[core];

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] reset EMUPC to [0x%08x]",
	    bfin_target.name, cpu->first_core + core, c->l1_map->l1_code);

  core_emupc_show (core, "before");
  part_emupc_reset (cpu->chain, cpu->first_core + core, c->l1_map->l1_code);
  core_emupc_show (core, "after");
}

static void
emupc_reset (void)
{
  int i;

  for (i = 0; i < cpu->core_num; i++)
    if (!cpu->cores[i].is_dead)
      core_emupc_reset (i);
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

  for (i = 0; i < cpu->core_num; i++) {
    core_dbgstat_get (i);
    part = cpu->chain->parts->parts[cpu->first_core + i];
    sprintf (buf, "[%d] DBGSTAT [0x%04X]:", cpu->first_core + i, BFIN_PART_DBGSTAT (part));
    if (core_dbgstat_is_lpdec1 (i))     strcat (buf, " lpdec1");
    if (core_dbgstat_is_core_fault (i)) strcat (buf, " core_fault");
    if (core_dbgstat_is_idle (i))       strcat (buf, " idle");
    if (core_dbgstat_is_in_reset (i))   strcat (buf, " in_reset");
    if (core_dbgstat_is_lpdec0 (i))     strcat (buf, " lpdec0");
    if (core_dbgstat_is_bist_done (i))  strcat (buf, " bist_done");
    if (core_dbgstat_is_emuack (i))     strcat (buf, " emuack");
    if (core_dbgstat_is_emuready (i))
    {
    strcat (buf, " emuready");
    switch (core_dbgstat_emucause (i)) {
      case 0x0: strcat (buf, " cause:emuexcpt");   break;
      case 0x1: strcat (buf, " cause:emuin");      break;
      case 0x2: strcat (buf, " cause:watchpoint"); break;
      case 0x4: strcat (buf, " cause:perfmon0");   break;
      case 0x5: strcat (buf, " cause:perfmon1");   break;
      case 0x8: strcat (buf, " cause:emu-sstep");  break;
      default:  strcat (buf, " cause:unknown");    break;
    }
    }

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
sdu_stat_show (urj_chain_t *chain, int n, const char *id)
{
  urj_part_t *part;
  char buf[1024];

  assert (id != NULL);

  sdu_stat_get (chain, n);
  part = chain->parts->parts[n];
  sprintf (buf, "[%d] SDU_STAT [0x%08X]:", n, SDU_PART_STAT (part));
  if (sdu_stat_is_sysrst (chain, n))        strcat (buf, " sysrst");
  if (sdu_stat_is_err (chain, n))           strcat (buf, " err");
  if (sdu_stat_is_deepsleep (chain, n))     strcat (buf, " deepsleep");
  if (sdu_stat_is_secure (chain, n))        strcat (buf, " secure");
  if (sdu_stat_is_err (chain, n))
    switch (sdu_stat_errc (chain, n))
      {
	case 0x0: strcat (buf, " errc:[MAC Over/Underflow]"); break;
	case 0x1: strcat (buf, " errc:[DMARD Underflow]");    break;
	case 0x2: strcat (buf, " errc:[DMAWD Overflow]");     break;
	case 0x3: strcat (buf, " errc:[DMA Error]");          break;
	case 0x4: strcat (buf, " errc:[MAC AXI Error]");      break;
	default:  strcat (buf, " errc:[unknown]");            break;
      }
  if (sdu_stat_is_macrdy (chain, n))        strcat (buf, " macrdy");
  if (sdu_stat_is_dmardrdy (chain, n))      strcat (buf, " dmardrdy");
  if (sdu_stat_is_dmawdrdy (chain, n))      strcat (buf, " dmawdrdy");
  if (sdu_stat_is_addrerr (chain, n))       strcat (buf, " addrerr");
  switch (sdu_stat_dmafifo (chain, n))
    {
    case 0x0: strcat (buf, " dmafifo:empty");             break;
    case 0x1: strcat (buf, " dmafifo:(0,1/4]");           break;
    case 0x2: strcat (buf, " dmafifo:(1/4,1/2]");         break;
    case 0x3: strcat (buf, " dmafifo:(1/2,3/4]");         break;
    case 0x4: strcat (buf, " dmafifo:(3/4,1)");           break;
    case 0x7: strcat (buf, " dmafifo:full");              break;
    default:  strcat (buf, " dmafifo:unknown");           break;
    }
  if (sdu_stat_is_ghlt (chain, n))          strcat (buf, " ghlt");
  if (sdu_stat_is_ghlt (chain, n))
    switch (sdu_stat_ghltc (chain, n))
      {
	case 0x0: strcat (buf, " ghltc:system");              break;
	case 0x1: strcat (buf, " ghltc:core0");               break;
	case 0x2: strcat (buf, " ghltc:core1");               break;
	default:  strcat (buf, " ghltc:unknown");             break;
      }
  if (sdu_stat_is_eme (chain, n))           strcat (buf, " eme");
  if (sdu_stat_is_chlt (chain, n))          strcat (buf, " chlt");
  if (sdu_stat_is_crst (chain, n))          strcat (buf, " crst");
  strcat (buf, " <");
  strcat (buf, id);
  strcat (buf, ">");
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, buf);
}

static void
core_dbgctl_show (int core, const char *id)
{
  urj_part_t *part;
  char buf[1024];

  assert (id != NULL);

  part = cpu->chain->parts->parts[cpu->first_core + core];
  sprintf (buf, "[%i] DBGCTL [0x%04x]:", cpu->first_core + core,
	   BFIN_PART_DBGCTL (part));
  if (core_dbgctl_is_sram_init (core))   strcat (buf, " sram_init");
  if (core_dbgctl_is_wakeup (core))      strcat (buf, " wakeup");
  if (core_dbgctl_is_sysrst (core))      strcat (buf, " sysrst");
  if (core_dbgctl_is_esstep (core))      strcat (buf, " esstep");
  if (core_dbgctl_is_emudatsz_32 (core)) strcat (buf, " emudatsz_32");
  if (core_dbgctl_is_emudatsz_40 (core)) strcat (buf, " emudatsz_40");
  if (core_dbgctl_is_emudatsz_48 (core)) strcat (buf, " emudatsz_48");
  if (core_dbgctl_is_emuirlpsz_2 (core)) strcat (buf, " emuirlpsz_2");
  if (core_dbgctl_is_emuirsz_64 (core))  strcat (buf, " emuirsz_64");
  if (core_dbgctl_is_emuirsz_48 (core))  strcat (buf, " emuirsz_48");
  if (core_dbgctl_is_emuirsz_32 (core))  strcat (buf, " emuirsz_32");
  if (core_dbgctl_is_empen (core))       strcat (buf, " empen");
  if (core_dbgctl_is_emeen (core))       strcat (buf, " emeen");
  if (core_dbgctl_is_emfen (core))       strcat (buf, " emfen");
  if (core_dbgctl_is_empwr (core))       strcat (buf, " empwr");
  strcat (buf, " <");
  strcat (buf, id);
  strcat (buf, ">");
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, buf);
}

static void
core_check_emuready (int core)
{
  part_check_emuready (cpu->chain, cpu->first_core + core);
}

static void
core_wait_in_reset (int core)
{
  part_wait_in_reset (cpu->chain, cpu->first_core + core);
}

static void
core_wait_reset (int core)
{
  part_wait_reset (cpu->chain, cpu->first_core + core);
}

static void
core_emuir_set (int core, uint64_t insn, int runtest)
{
  part_emuir_set (cpu->chain, cpu->first_core + core, insn,
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
static void core_register_set (int core, enum core_regnum reg,
			       uint32_t value);

static uint32_t
core_register_get (int core, enum core_regnum reg)
{
  return part_register_get (cpu->chain, cpu->first_core + core, reg);
}

static void
core_register_set (int core, enum core_regnum reg, uint32_t value)
{
  part_register_set (cpu->chain, cpu->first_core + core, reg, value);
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

static void
wpu_disable (void)
{
  int i;
  for (i = 0; i < cpu->core_num; i++)
    if (!cpu->cores[i].is_dead)
      core_wpu_disable (i);
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
core_emulation_enable (int core)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_emulation_enable ()",
	    bfin_target.name, cpu->first_core + core);

  core_dbgstat_show (core, "before");
  part_emulation_enable (cpu->chain, cpu->first_core + core);
  core_dbgstat_show (core, "after");

  core_emupc_show (core, "");

  core_dbgctl_show (core, "emulation_enable");
}

static void
emulation_enable (void)
{
  int i;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: emulation_enable ()", bfin_target.name);

  for (i = 0; i < cpu->core_num; i++)
    core_emulation_enable (i);
}

static void
core_emulation_trigger (int core)
{
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_emulation_trigger ()",
	    bfin_target.name, cpu->first_core + core);

  core_dbgstat_show (core, "before");
  if (cpu->sdu == -1)
    part_emulation_trigger (cpu->chain, cpu->first_core + core);
  else
    {
      sdu_stat_show (cpu->chain, cpu->sdu, "before");
      sdu_halt (cpu->chain, cpu->sdu,
		SDU_CTL_EHLT_CORE (cpu->core_num - core - 1));
      sdu_stat_show (cpu->chain, cpu->sdu, "after");
    }    
  core_dbgstat_show (core, "after");
}

static void
emulation_trigger (void)
{
  int i;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: emulation_trigger ()", bfin_target.name);

  dbgstat_get ();
  for (i = 0; i < cpu->core_num; i++)
    {
      /* Triggerring emulation on a core which has already been in
	 emulation mode will cause unexpected effect.

	 For example, if the core is executing IDLE instruction.
	 The first trigger will bring the core out of the IDLE mode
	 and halt it at the next instruction.  But a consecutive
	 second trigger will cause the core to resume execution.  */
      if (!core_dbgstat_is_emuready (i))
	core_emulation_trigger (i);
    }
}

static void
core_emulation_return (int core)
{
  uint32_t rete;

  rete = core_register_get (core, REG_RETE);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: [%d] core_emulation_return () to [0x%08x]",
	    bfin_target.name, cpu->first_core + core, rete);

  core_dbgstat_show (core, "before");
  part_emulation_return (cpu->chain, cpu->first_core + core);
  core_dbgstat_show (core, "after");
}

static void
emulation_return (void)
{
  int i;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: emulation_return ()", bfin_target.name);

  for (i = 0; i < cpu->core_num; i++)
    core_emulation_return (i);
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
emulation_disable (void)
{
  int i;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: emulation_disable ()", bfin_target.name);

  for (i = 0; i < cpu->core_num; i++)
    core_emulation_disable (i);
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

  bfin_core_reset (cpu->chain, cpu->first_core + cpu->core_a);
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
rcu0_stat_bmode (uint32_t rcu0_stat)
{
  return RCU0_STAT_BIT_FIELD_VALUE (BMODE);
}

static int
bfin_bootmode (int core)
{
  uint32_t rcu0_stat;
  int bootmode;

  /* Only BF609 is supported now.  */
  if (cpu->sdu == -1)
    return -1;

  rcu0_stat = mmr_read (core, RCU0_STAT, 4);
  bootmode = rcu0_stat_bmode (rcu0_stat);
  return bootmode;
}

#define SDU_MSG_SET_HALTON_BIT(bitname, BITNAME)			\
  static void								\
  sdu_msg_set_halton##bitname (int core)				\
  {									\
    bfin_log (RP_VAL_LOGLEVEL_DEBUG,					\
	      "%s: [%d] set SDU_MSG bit MSG_HALTON" #BITNAME,		\
	      bfin_target.name, cpu->first_core + core);		\
    mmr_write (core, SDU0_MSG_SET, MSG_HALTON##BITNAME, 4);		\
  }

#define SDU_MSG_CLEAR_HALTON_BIT(bitname, BITNAME)			\
  static void								\
  sdu_msg_clear_halton##bitname (int core)				\
  {									\
    bfin_log (RP_VAL_LOGLEVEL_DEBUG,					\
	      "%s: [%d] set SDU_MSG bit MSG_HALTON" #BITNAME,		\
	      bfin_target.name, cpu->first_core + core);		\
    mmr_write (core, SDU0_MSG_CLR, MSG_HALTON##BITNAME, 4);		\
  }

#define SDU_MSG_IS_CALL_BIT(bitname, BITNAME)				\
  static int								\
  sdu_msg_is_call##bitname (int core)					\
  {									\
    uint32_t sdu_msg_set;						\
									\
    bfin_log (RP_VAL_LOGLEVEL_DEBUG,					\
	      "%s: [%d] test SDU_MSG bit MSG_CALL" #BITNAME,		\
	      bfin_target.name, cpu->first_core + core);		\
    									\
    sdu_msg_set = mmr_read (core, SDU0_MSG, 4);				\
    if (sdu_msg_set & MSG_CALL##BITNAME)				\
      return 1;								\
    else								\
      return 0;								\
  }

#define SDU_MSG_CLEAR_CALL_BIT(bitname, BITNAME)			\
  static void								\
  sdu_msg_clear_call##bitname (int core)				\
  {									\
    bfin_log (RP_VAL_LOGLEVEL_DEBUG,					\
	      "%s: [%d] clear SDU_MSG bit MSG_CALL" #BITNAME,		\
	      bfin_target.name, cpu->first_core + core);		\
    mmr_write (core, SDU0_MSG_CLR, MSG_CALL##BITNAME, 4);		\
  }

SDU_MSG_SET_HALTON_BIT (error, ERROR)
SDU_MSG_SET_HALTON_BIT (call, CALL)
SDU_MSG_SET_HALTON_BIT (init, INIT)
SDU_MSG_SET_HALTON_BIT (app, APP)

SDU_MSG_CLEAR_HALTON_BIT (error, ERROR)
SDU_MSG_CLEAR_HALTON_BIT (call, CALL)
SDU_MSG_CLEAR_HALTON_BIT (init, INIT)
SDU_MSG_CLEAR_HALTON_BIT (app, APP)

SDU_MSG_IS_CALL_BIT (error, ERROR)
SDU_MSG_IS_CALL_BIT (back, BACK)
SDU_MSG_IS_CALL_BIT (init, INIT)
SDU_MSG_IS_CALL_BIT (app, APP)

SDU_MSG_CLEAR_CALL_BIT (error, ERROR)
SDU_MSG_CLEAR_CALL_BIT (back, BACK)
SDU_MSG_CLEAR_CALL_BIT (init, INIT)
SDU_MSG_CLEAR_CALL_BIT (app, APP)

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
  bfin_core *c UNUSED;
  int i;

  FOR_EACH_ALIVE_CORE (i, c)
    {
      core_dbgstat_get (i);
      if (core_dbgstat_is_emuready (i))
	break;
    }

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
  bfin_core *c UNUSED;
  int i;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: ddr_init ()", bfin_target.name);

  FOR_EACH_ALIVE_CORE (i, c)
    {
      core_dbgstat_get (i);
      if (core_dbgstat_is_emuready (i))
	break;
    }

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

#define PRINT_BIT(VALUE, MMR, BIT)				\
  do								\
    {								\
      if (!(((VALUE) >> MMR##_##BIT) & 0x1))			\
	strcat (buf, "-");					\
      strcat (buf, #BIT " ");					\
    }								\
  while (0)

#define DDR0_PRINT_BIT(BIT) PRINT_BIT(ddr0_stat, DDR0_STAT, BIT)

static void
ddr0_stat_show (uint32_t ddr0_stat, const char *id)
{
  char buf[1024];
  char buf2[64];

  sprintf (buf, "DDR0_STAT [0x%08X]: ", ddr0_stat);

  sprintf (buf2, "PHY_RD_PHASE[%d] ", DDR0_STAT_BIT_FIELD_VALUE (PHY_RD_PHASE));
  strcat (buf, buf2);
  sprintf (buf2, "PEND_REF[%d] ", DDR0_STAT_BIT_FIELD_VALUE (PEND_REF));
  strcat (buf, buf2);
  DDR0_PRINT_BIT (DLL_CAL_DONE);
  strcat (buf, "\n             ");
  DDR0_PRINT_BIT (DPDACK);
  DDR0_PRINT_BIT (PDACK);
  DDR0_PRINT_BIT (SRACK);
  DDR0_PRINT_BIT (INIT_DONE);
  DDR0_PRINT_BIT (DMC_IDLE);

  strcat (buf, "<");
  strcat (buf, id);
  strcat (buf, ">");

  bfin_log (RP_VAL_LOGLEVEL_DEBUG, buf);
}

/* Timing parameters for MT47H64M16HR-3 */
#define tRC		55	/* min ns */
#define tRCD		15	/* min ns */
#define tRAS		40	/* min ns */
#define tRP		15	/* min ns */
#define tRRD		10	/* min ns */
#define tFAW		50	/* min ns */
#define tRTP		7.5	/* min ns */
#define tWR		15	/* min ns */
#define tWTR		7.5	/* min ns */
#define tMRD		2	/* min tCK */
#define tRFC		127.5	/* min ns */
#define tREFI		7812.5	/* max ns */
#define tXARD		2	/* min tCK */
#define tXP		2	/* min tCK */
#define tCKE		3	/* min tCK */

static int
ddr2_init (void)
{
  uint32_t p0, r0;
  float tck;
  /* parameters in ddrtr0 */
  uint32_t trcd, twtr, trp, tras, trc, tmrd;
  /* parameters in ddrtr1 */
  uint32_t tref, trfc, trrd;
  /* parameters in ddrtr2 */
  uint32_t tfaw, trtp, txp, tcke;
  /* parameters in ddrmr */
  uint32_t bl, cl, twr;
  uint32_t ddrctl, ddrcfg, ddrtr0, ddrtr1, ddrtr2;
  uint32_t ddrmr, ddremr1, ddremr2, ddremr3;
  uint32_t ddr0_mrwmr;
  uint32_t ddr0_stat, ddr0_dllctl, data_cycle;
  bfin_core *c UNUSED;
  int i, count;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: ddr2_init ()", bfin_target.name);

  FOR_EACH_ALIVE_CORE (i, c)
    {
      core_dbgstat_get (i);
      if (core_dbgstat_is_emuready (i))
	break;
    }

  if (i == cpu->core_num)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: no core available to init ddr2",
		bfin_target.name);
      return -1;
    }

  p0 = core_register_get (i, REG_P0);
  r0 = core_register_get (i, REG_R0);

  core_register_set (i, REG_P0, DDR0_ID);

  ddr0_stat = mmr_read_clobber_r0 (i, DDR0_STAT - DDR0_ID, 4);
  ddr0_stat_show (ddr0_stat, "before init");

  ddrctl = (1 << DDR0_CTL_INIT
	    | (2 & DDR0_CTL_RD_TO_WR_CYC_BITM) << DDR0_CTL_RD_TO_WR_CYC_BITP);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: ddrtcl 0x%08x", bfin_target.name, ddrctl);

  /* Interface width 16, SDRAM width 16, SDRAM size 1Gb */
  ddrcfg = ((2 & DDR0_CFG_IF_WIDTH_BITM) << DDR0_CFG_IF_WIDTH_BITP
	    | (2 & DDR0_CFG_SDRAM_WIDTH_BITM) << DDR0_CFG_SDRAM_WIDTH_BITP
	    | (4 & DDR0_CFG_SDRAM_SIZE_BITM) << DDR0_CFG_SDRAM_SIZE_BITP);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: ddrcfg 0x%08x", bfin_target.name, ddrcfg);

  /* tCK in ns.  bfin_bf609_dclk in MHz.  */
  tck = (float) 1000 / bfin_bf609_dclk;
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: tCK %0.1f", bfin_target.name, tck);

  trcd = ceilf (tRCD / tck);
  twtr = ceilf (tWTR / tck);
  trp  = ceilf (tRP / tck);
  tras = ceilf (tRAS / tck);
  trc  = ceilf (tRC / tck);
  tmrd = tMRD;

  ddrtr0 = ((trcd & DDR0_TR0_TRCD_BITM) << DDR0_TR0_TRCD_BITP
	    | (twtr & DDR0_TR0_TWTR_BITM) << DDR0_TR0_TWTR_BITP
	    | (trp & DDR0_TR0_TRP_BITM) << DDR0_TR0_TRP_BITP
	    | (tras & DDR0_TR0_TRAS_BITM) << DDR0_TR0_TRAS_BITP
	    | (trc & DDR0_TR0_TRC_BITM) << DDR0_TR0_TRC_BITP
	    | (tmrd & DDR0_TR0_TMRD_BITM) << DDR0_TR0_TMRD_BITP);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: ddrtr0 0x%08x", bfin_target.name, ddrtr0);

  tref = floorf (tREFI / tck);
  trfc = ceilf (tRFC / tck);
  trrd = ceilf (tRRD / tck);

  ddrtr1 = ((tref & DDR0_TR1_TREF_BITM) << DDR0_TR1_TREF_BITP
	    | (trfc & DDR0_TR1_TRFC_BITM) << DDR0_TR1_TRFC_BITP
	    | (trrd & DDR0_TR1_TRRD_BITM) << DDR0_TR1_TRRD_BITP);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: ddrtr1 0x%08x", bfin_target.name, ddrtr1);

  tfaw = ceil (tFAW / tck);
  trtp = ceil (tRTP / tck);
  txp  = tXP > tXARD ? tXP : tXARD;
  tcke = tCKE;

  ddrtr2 = ((tfaw & DDR0_TR2_TFAW_BITM) << DDR0_TR2_TFAW_BITP
	    | (trtp & DDR0_TR2_TRTP_BITM) << DDR0_TR2_TRTP_BITP
	    | (txp & DDR0_TR2_TXP_BITM) << DDR0_TR2_TXP_BITP
	    | (tcke & DDR0_TR2_TCKE_BITM) << DDR0_TR2_TCKE_BITP);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: ddrtr2 0x%08x", bfin_target.name, ddrtr2);

  /* Burst lenth
     "010" is a burst of 4, "011" is a burst of 8.  */
  bl = 2;
  /* CAS Latency
     calculate_clocks should have checked DCLK such that tck is valid.  */
  if (tck >= 5.0)
    cl = 3;
  else if (tck >= 3.75)
    cl = 4;
  else
    cl = 5;
  if (bfin_bf609_ddrcl != 0)
    {
      if (bfin_bf609_ddrcl < cl)
	bfin_log (RP_VAL_LOGLEVEL_WARNING,
		  "%s: DDR CL %d is too small, should be at least %d",
		  bfin_target.name, bfin_bf609_ddrcl, cl);
      cl = bfin_bf609_ddrcl;
    }
  bfin_log (RP_VAL_LOGLEVEL_INFO,
	    "%s: DDR CL set to %d", bfin_target.name, cl);

  /* Write Recovery */
  twr = ceil (tWR / tck);

  ddrmr = ((bl & DDR0_MR_BL_BITM) << DDR0_MR_BL_BITP
	   | (cl & DDR0_MR_CL_BITM) << DDR0_MR_CL_BITP
	   | (twr & DDR0_MR_TWR_BITM) << DDR0_MR_TWR_BITP);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: ddrmr 0x%08x", bfin_target.name, ddrmr);

  ddremr1 = 0;
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: ddremr1 0x%08x", bfin_target.name, ddremr1);

  ddremr2 = 0;
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: ddremr2 0x%08x", bfin_target.name, ddremr2);

  ddremr3 = 0;
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: ddremr3 0x%08x", bfin_target.name, ddremr3);

  mmr_write_clobber_r0 (i, DDR0_CFG - DDR0_ID, ddrcfg, 4);
  mmr_write_clobber_r0 (i, DDR0_TR0 - DDR0_ID, ddrtr0, 4);
  mmr_write_clobber_r0 (i, DDR0_TR1 - DDR0_ID, ddrtr1, 4);
  mmr_write_clobber_r0 (i, DDR0_TR2 - DDR0_ID, ddrtr2, 4);

  ddr0_mrwmr = mmr_read_clobber_r0 (i, DDR0_MRWMR - DDR0_ID, 4);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: DDR0 MR WMR 0x%08x", bfin_target.name, ddr0_mrwmr);

  ddr0_mrwmr = DDR0_MR_MASK | DDR0_EMR1_MASK | DDR0_EMR2_MASK | DDR0_EMR3_MASK;
  mmr_write_clobber_r0 (i, DDR0_MRWMR - DDR0_ID, ddr0_mrwmr, 4);

  mmr_write_clobber_r0 (i, DDR0_MR - DDR0_ID, ddrmr, 4);
  mmr_write_clobber_r0 (i, DDR0_EMR1 - DDR0_ID, ddremr1, 4);
  mmr_write_clobber_r0 (i, DDR0_EMR2 - DDR0_ID, ddremr2, 4);
  mmr_write_clobber_r0 (i, DDR0_EMR3 - DDR0_ID, ddremr3, 4);

  mmr_write_clobber_r0 (i, DDR0_CTL - DDR0_ID, ddrctl, 4);

  count = 0;
  do
    {
      ddr0_stat = mmr_read_clobber_r0 (i, DDR0_STAT - DDR0_ID, 4);
      ddr0_stat_show (ddr0_stat, "after init");
      count++;
      if (count == 10)
	break;
    }
  while (((ddr0_stat >> DDR0_STAT_INIT_DONE) & 0x1) == 0);

  if (count == 10)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: DDR2 initialization failed",
		bfin_target.name);
      core_register_set (i, REG_P0, p0);
      core_register_set (i, REG_R0, r0);
      return -1;
    }

  ddr0_mrwmr = mmr_read_clobber_r0 (i, DDR0_MRWMR - DDR0_ID, 4);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: DDR0 MR WMR 0x%08x <after init>",
	    bfin_target.name, ddr0_mrwmr);

  /* Update DATA_CYCLE in DDR0_DLLCTL with PHY_RD_PHASE from DDR0_STAT */
  data_cycle = DDR0_STAT_BIT_FIELD_VALUE (PHY_RD_PHASE);
  ddr0_dllctl = mmr_read_clobber_r0 (i, DDR0_DLLCTL - DDR0_ID, 4);
  ddr0_dllctl &= ~(DDR0_DLLCTL_DATA_CYCLE_BITM << DDR0_DLLCTL_DATA_CYCLE_BITP);
  ddr0_dllctl |= data_cycle << DDR0_DLLCTL_DATA_CYCLE_BITP;
  mmr_write_clobber_r0 (i, DDR0_DLLCTL - DDR0_ID, ddr0_dllctl, 4);

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: ddr2_init done", bfin_target.name);

  core_register_set (i, REG_P0, p0);
  core_register_set (i, REG_R0, r0);
  return 0;
}

#define CGU0_PRINT_BIT(BIT) PRINT_BIT(cgu0_stat, CGU0_STAT, BIT)

static void
cgu0_stat_show (uint32_t cgu0_stat, const char *id)
{
  char buf[1024];

  sprintf (buf, "CGU0_STAT [0x%08X]: ", cgu0_stat);
  CGU0_PRINT_BIT (PLLLKERR);
  CGU0_PRINT_BIT (WDIVERR);
  CGU0_PRINT_BIT (WDFMSERR);
  CGU0_PRINT_BIT (DIVERR);
  CGU0_PRINT_BIT (LWERR);
  strcat (buf, "\n             ");
  CGU0_PRINT_BIT (ADRERR);
  CGU0_PRINT_BIT (OCBFEN);
  CGU0_PRINT_BIT (DCBFEN);
  CGU0_PRINT_BIT (SCBF1EN);
  CGU0_PRINT_BIT (SCBF0EN);
  CGU0_PRINT_BIT (CCBF1EN);
  CGU0_PRINT_BIT (CCBF0EN);
  strcat (buf, "\n             ");
  CGU0_PRINT_BIT (CLKSALGN);
  CGU0_PRINT_BIT (PLLLK);
  CGU0_PRINT_BIT (PLLBP);
  CGU0_PRINT_BIT (PLLEN);

  strcat (buf, "<");
  strcat (buf, id);
  strcat (buf, ">");

  bfin_log (RP_VAL_LOGLEVEL_DEBUG, buf);
}

/* CLKIN 25MHz */
static int
calculate_clocks (void)
{
  /* The default CCLK is 500 MHz.  */
  if (bfin_bf609_cclk == 0)
    bfin_bf609_cclk = 500;

  /* The default SCLK is CCLK/2.  */
  if (bfin_bf609_sclk == 0)
    bfin_bf609_sclk = bfin_bf609_cclk / 2;

  /* The default DCLK is same as SCLK.  */
  if (bfin_bf609_dclk == 0)
    bfin_bf609_dclk = bfin_bf609_sclk;

  if (bfin_init_sdram && cpu->ddr2)
    {
      if (bfin_bf609_dclk < 125)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: DCLK %d is less than 125MHz",
		    bfin_target.name, bfin_bf609_cclk);
	  return -1;
	}

      if (bfin_bf609_dclk > 333)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: DCLK %d is larger than 333MHz",
		    bfin_target.name, bfin_bf609_cclk);
	  return -1;
	}
    }

  for (msel = 1; msel <= 128; msel++)
    {
      for (csel = 1; csel <= 32; csel++)
	if (CLKIN * msel == bfin_bf609_cclk * csel)
	  break;

      if (csel == 33)
	continue;

      for (syssel = 1; syssel <= 32; syssel++)
	if (CLKIN * msel == bfin_bf609_sclk * syssel)
	  break;

      if (syssel != 33)
	break;;
    }

  if (msel > 40)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: bad CCLK %d or SCLK %d, can't be generated",
		bfin_target.name, bfin_bf609_cclk, bfin_bf609_sclk);
      return -1;
    }

  for (s0sel = 2; s0sel <= 4; s0sel++)
    if (bfin_bf609_sclk / s0sel * s0sel == bfin_bf609_sclk)
      break;

  if (s0sel == 5)
    s0sel = 1;

  s1sel = s0sel;

  for (dsel = 1; dsel <= 32; dsel++)
    if (CLKIN * msel == bfin_bf609_dclk * dsel)
      break;

  if (dsel == 33)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: bad DCLK %d, can't be generated",
		bfin_target.name, bfin_bf609_dclk);
      return -1;
    }

  osel = msel;

  bfin_log (RP_VAL_LOGLEVEL_INFO,
	    "%s: CCLK %dMHz SCLK %dMHz S0CLK %dMHz S1CLK %dMHz DCLK %dMHz",
	    bfin_target.name,
	    bfin_bf609_cclk, bfin_bf609_sclk,
	    bfin_bf609_sclk / s0sel, bfin_bf609_sclk/ s1sel, bfin_bf609_dclk);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: MSEL %d CSEL %d SYSSEL %d S0SEL %d S1SEL %d DSEL %d OSEL %d",
	    bfin_target.name, msel, csel, syssel, s0sel, s1sel, dsel, osel);

  return 0;

}

static int
pll_init (void)
{
  uint32_t p0, r0;
  bfin_core *c UNUSED;
  int i, count;
  uint32_t cgu_stat;
  uint32_t cgu_ctl, cgu_ctl_new;
  uint32_t cgu_div, cgu_div_new;

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: pll_init ()", bfin_target.name);

  FOR_EACH_ALIVE_CORE (i, c)
    {
      core_dbgstat_get (i);
      if (core_dbgstat_is_emuready (i))
	break;
    }

  if (i == cpu->core_num)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: no core available to init PLL",
		bfin_target.name);
      return -1;
    }

  cgu_ctl_new = (msel & CGU0_CTL_MSEL_BITM) << CGU0_CTL_MSEL_BITP;
  cgu_div_new = ((osel & CGU0_DIV_OSEL_BITM) << CGU0_DIV_OSEL_BITP
		 | (dsel & CGU0_DIV_DSEL_BITM) << CGU0_DIV_DSEL_BITP
		 | (s1sel & CGU0_DIV_S1SEL_BITM) << CGU0_DIV_S1SEL_BITP
		 | (syssel & CGU0_DIV_SYSSEL_BITM) << CGU0_DIV_SYSSEL_BITP
		 | (s0sel & CGU0_DIV_S0SEL_BITM) << CGU0_DIV_S0SEL_BITP
		 | (csel & CGU0_DIV_CSEL_BITM) << CGU0_DIV_CSEL_BITP);

  p0 = core_register_get (i, REG_P0);
  r0 = core_register_get (i, REG_R0);

  core_register_set (i, REG_P0, CGU0_CTL);

  cgu_stat = mmr_read_clobber_r0 (i, CGU0_STAT - CGU0_CTL, 4);
  cgu0_stat_show(cgu_stat, "before init");

  if (!(cgu_stat & (1 << CGU0_STAT_PLLEN)))
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: PLL is disabled",
		bfin_target.name);
      core_register_set (i, REG_P0, p0);
      core_register_set (i, REG_R0, r0);
      return -1;
    }

  if (!(cgu_stat & (1 << CGU0_STAT_PLLLK))
      && !(cgu_stat & (1 << CGU0_STAT_PLLLKERR)))
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: PLL is locking",
		bfin_target.name);
      core_register_set (i, REG_P0, p0);
      core_register_set (i, REG_R0, r0);
      return -1;
    }

  if (cgu_stat & (1 << CGU0_STAT_CLKSALGN))
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: clocks is aligning",
		bfin_target.name);
      core_register_set (i, REG_P0, p0);
      core_register_set (i, REG_R0, r0);
      return -1;
    }

  cgu_ctl = mmr_read_clobber_r0 (i, 0, 4);
  cgu_div = mmr_read_clobber_r0 (i, CGU0_DIV - CGU0_CTL, 4);

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: current CGU_CTL 0x%08x CGU_DIV 0x%08x",
	    bfin_target.name, cgu_ctl, cgu_div);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: new CGU_CTL 0x%08x CGU_DIV 0x%08x",
	    bfin_target.name, cgu_ctl_new, cgu_div_new);

  if (cgu_ctl == cgu_ctl_new && cgu_div == cgu_div_new)
    {
      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: no need to set them",
		bfin_target.name);
      core_register_set (i, REG_P0, p0);
      core_register_set (i, REG_R0, r0);
      return 0;
    }

  if (cgu_ctl == cgu_ctl_new && cgu_div != cgu_div_new)
    cgu_div_new |= (1 << CGU0_DIV_UPDT);

  if (cgu_div != cgu_div_new)
    {
      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: changing CGU0_DIV to %08x",
		bfin_target.name, cgu_div_new);

      mmr_write_clobber_r0 (i, CGU0_DIV - CGU0_CTL, cgu_div_new, 4);
    }

  /* TODO WIDLE bit */
  if (cgu_ctl != cgu_ctl_new)
    {
      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: changing CGU0_CTL to %08x",
		bfin_target.name, cgu_ctl_new);

      mmr_write_clobber_r0 (i, 0, cgu_ctl_new, 4);
    }

  count = 0;
  do
    {
      cgu_stat = mmr_read_clobber_r0 (i, CGU0_STAT - CGU0_CTL, 4);
      cgu0_stat_show(cgu_stat, "after init");
      count++;
      if (count == 10)
	break;
    }
  while (((cgu_stat >> CGU0_STAT_PLLLK) & 0x1) == 0
	 || ((cgu_stat >> CGU0_STAT_PLLBP) & 0x1) != 0
	 || ((cgu_stat >> CGU0_STAT_CLKSALGN) & 0x1) != 0);

  if (count == 10)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: PLL initialization failed",
		bfin_target.name);

      core_register_set (i, REG_P0, p0);
      core_register_set (i, REG_R0, r0);
      return -1;
    }
  else
    {
      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: PLL initialization is done",
		bfin_target.name);

      core_register_set (i, REG_P0, p0);
      core_register_set (i, REG_R0, r0);
      return 0;
    }
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

  FOR_EACH_ALIVE_CORE (i, c)
    {
      core_dbgstat_get (i);
      if (!core_dbgstat_is_emuready (i))
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

  FOR_EACH_ALIVE_CORE (i, c)
    {
      core_dbgstat_get (i);
      if (!core_dbgstat_is_emuready (i))
	{
	  c->l1_code_cache_enabled = 0;
	}
      else
	{
	  core_icache_enable (i);
	  c->l1_code_cache_enabled = 1;
	}
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

  if (cpu->sdu != -1 && bfin_bf609_sdu_mac_read)
    return sdu_mac_memory_read (cpu->chain, cpu->sdu, addr, buf, size,
				bfin_bf609_check_macrdy);

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

  if (cpu->sdu != -1 && bfin_bf609_sdu_mac_write)
    return sdu_mac_memory_write (cpu->chain, cpu->sdu, addr, buf, size,
				 bfin_bf609_check_macrdy);

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

static void
bfin_test_command_mmrs (urj_part_t *part, uint32_t addr, int icache,
                        uint32_t *command_addr,
                        uint32_t *data0_addr, uint32_t *data1_addr)
{
  if (icache)
    {
      *command_addr = ITEST_COMMAND;
      *data0_addr = ITEST_DATA0;
      *data1_addr = ITEST_DATA1;
    }
  else
    {
      *command_addr = DTEST_COMMAND;
      *data0_addr = DTEST_DATA0;
      *data1_addr = DTEST_DATA1;
    }
}

static void
bfin_test_command (urj_part_t *part, uint32_t addr, int w,
                   uint32_t command_addr, uint32_t *command_value)
{
  /* The shifting here is a bit funky, but should be straight forward and
     easier to maintain than hand coded masks.  So, start with the break
     down of the [DI]TEST_COMMAND MMR in the HRM and follow along:

     We need to put bit 11 of the address into bit 26 of the MMR.  So first
     we mask off ADDR[11] with:
     (addr & (1 << 11))

     Then we shift it from its current position (11) to its new one (26):
     ((...) << (26 - 11))
  */

  /* Start with the bits ITEST/DTEST share.  */
  *command_value =
    ((addr & (0x03 << 12)) << (16 - 12)) | /* ADDR[13:12] -> MMR[17:16] */
    ((addr & (0x01 << 14)) << (14 - 14)) | /* ADDR[14]    -> MMR[14]    */
    ((addr & (0xff <<  3)) << ( 3 -  3)) | /* ADDR[10:3]  -> MMR[10:3]  */
    (1 << 2)                             | /* 1 (data)    -> MMR[2]     */
    (w << 1);                              /* read/write  -> MMR[1]     */

  /* Now for the bits that aren't the same.  */
  if (command_addr == ITEST_COMMAND)
    *command_value |=
      ((addr & (0x03 << 10)) << (26 - 10));  /* ADDR[11:10] -> MMR[27:26] */
  else
    *command_value |=
      ((addr & (0x01 << 11)) << (26 - 11)) | /* ADDR[11] -> MMR[26] */
      ((addr & (0x01 << 21)) << (24 - 21));  /* ADDR[21] -> MMR[24] */

  /* Now, just for fun, some parts are slightly different.  */
  if (command_addr == DTEST_COMMAND)
    {
      /* BF50x has no additional needs.  */
      if (!strcmp (part->part, "BF518"))
        {
	  /* MMR[23]:
	     0 - Data Bank A (0xff800000) / Inst Bank A (0xffa00000)
	     1 - Data Bank B (0xff900000) / Inst Bank B (0xffa04000)
	  */
	  if ((addr & 0xfff04000) == 0xffa04000 ||
	      (addr & 0xfff00000) == 0xff900000)
	    *command_value |= (1 << 23);
        }
      else if (!strcmp (part->part, "BF526") ||
	       !strcmp (part->part, "BF527") ||
	       !strcmp (part->part, "BF533") ||
	       !strcmp (part->part, "BF534") ||
	       !strcmp (part->part, "BF537") ||
	       !strcmp (part->part, "BF538") ||
	       !strcmp (part->part, "BF548") ||
	       !strcmp (part->part, "BF548M") ||
	       !strcmp (part->part, "BF609"))
        {
	  /* MMR[23]:
	     0 - Data Bank A (0xff800000) / Inst Bank A (0xffa00000)
	     1 - Data Bank B (0xff900000) / Inst Bank B (0xffa08000)
	  */
	  if ((addr & 0xfff08000) == 0xffa08000 ||
	      (addr & 0xfff00000) == 0xff900000)
	    *command_value |= (1 << 23);
        }
      else if (!strcmp (part->part, "BF561"))
        {
	  /* MMR[23]:
	     0 - Data Bank A (Core A: 0xff800000 Core B: 0xff400000)
	     Inst Bank A (Core A: 0xffa00000 Core B: 0xff600000)
	     1 - Data Bank B (Core A: 0xff900000 Core B: 0xff500000)
	     N/A for Inst (no Bank B)
	  */
	  uint32_t hi = (addr >> 20);
	  if (hi == 0xff9 || hi == 0xff5)
	    *command_value |= (1 << 23);
        }
      else if (!strcmp (part->part, "BF592"))
        {
	  /* ADDR[15] -> MMR[15]
	     MMR[22]:
	     0 - L1 Inst (0xffa00000)
	     1 - L1 ROM  (0xffa10000)
	  */
	  *command_value |= (addr & (1 << 15));
	  if ((addr >> 16) == 0xffa1)
	    *command_value |= (1 << 22);
        }
    }
}

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

  bfin_test_command (part, addr, 0, test_data->command_addr, &command);

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

  bfin_test_command (part, addr, 1, test_data->command_addr, &command);

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
  bfin_test_command_mmrs (part, addr,
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
sram_read_write (int core, uint32_t addr, uint8_t *buf, int size, int local,
		 int w)
{
  bfin_core *c = &cpu->cores[core];
  urj_part_t *part = cpu->chain->parts->parts[cpu->first_core + core];
  int use_test_mmrs;

  if (w && cpu->sdu != -1 && (!local || bfin_bf609_sdu_mac_write))
    return sdu_mac_memory_write (cpu->chain, cpu->sdu, addr, buf, size,
				 bfin_bf609_check_macrdy);

  if (!w && cpu->sdu != -1 && (!local || bfin_bf609_sdu_mac_read))
    return sdu_mac_memory_read (cpu->chain, cpu->sdu, addr, buf, size,
				bfin_bf609_check_macrdy);

  assert (local || cpu->mdma_d0 != 0);

  use_test_mmrs = IN_MAP (addr, c->l1_map->l1_code_cache) ||
		  IN_MAP (addr, c->l1_map->l1_code);
  /* The BF592 has MMR access to its L1 ROM, but not DMA.  */
  if (!use_test_mmrs && !strcmp (part->part, "BF592"))
    use_test_mmrs = IN_MAP (addr, c->l1_map->l1_code_rom);

  if (!local || use_dma || !use_test_mmrs)
    {
      if (w)
	return dma_sram_write (core, addr, buf, size);
      else
	return dma_sram_read (core, addr, buf, size);
    }
  else
    return itest_sram (core, addr, buf, size, w);
}
#define sram_read(c, a, b, s, l) sram_read_write(c, a, b, s, l, 0)
#define sram_write(c, a, b, s, l) sram_read_write(c, a, b, s, l, 1)

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
    return sram_read_write (core, addr, buf, size, local, w);
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
  /* Get the RTE out of EMUIR so we don't execute it more than once.
     This is for working around an issue of ICE-100B.  */
  core_emuir_set (core, INSN_NOP, UPDATE);
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
  core_dbgstat_get (core);
  if (!core_dbgstat_is_emuready (core))
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


static int
core_is_locked (int core)
{
  if (cpu->sdu != -1)
    return core_dbgstat_is_in_reset (core);
  else
    return (core_dbgstat_is_emuack (core)
	    && !core_dbgstat_is_in_reset (core));
}

static void
core_unlock (int core)
{
  bfin_log (RP_VAL_LOGLEVEL_INFO,
	    "%s: [%d] unlocking...", bfin_target.name, cpu->first_core + core);

  core_dbgstat_show (core, "before");

  if (cpu->sdu == -1)
    {
      uint16_t sica_syscr;

      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: [%d] unlock by clearing COREB_SRAM_INIT in SICA_SYSCR",
		bfin_target.name, cpu->first_core + core);

      sica_syscr = mmr_read (cpu->core_a, SICA_SYSCR, 2);
      sica_syscr &= ~SICA_SYSCR_COREB_SRAM_INIT;
      mmr_write (cpu->core_a, SICA_SYSCR, sica_syscr, 2);
    }
  else
    {
      uint32_t rcu0_cn_res;
      uint32_t rcu0_cn_stat;
      uint32_t rcu0_stat;

      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: [%d] unlock by clear bit %d of RCU0_CN_RES",
		bfin_target.name, core, cpu->core_num - core - 1);

      rcu0_stat = mmr_read (cpu->core_a, RCU0_STAT, 4);
      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: RCU0_STAT  0x%x <before>", bfin_target.name, rcu0_stat);

      rcu0_cn_res = mmr_read (cpu->core_a, RCU0_CN_RES, 4);
      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: RCU0_CN_RES  0x%x <before>", bfin_target.name, rcu0_cn_res);

      rcu0_cn_stat = mmr_read (cpu->core_a, RCU0_CN_STAT, 4);
      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: RCU0_CN_STAT 0x%x <before>", bfin_target.name, rcu0_cn_stat);

      /* Only unlocking Core 1 is supported */
      /* This does not work.  It seems we have to set RETE directly.
      if (core == 0)
	{
	  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		    "%s: set RCU0_CN_CTL1 0xff60", bfin_target.name);
	  mmr_write (cpu->core_a, RCU0_CN_CTL1, 0xff60, 4);
	}  */

      /* Trigger halt first. So when it's released from reset,
	 it will be just halted at the first address.  */
      sdu_halt_trigger (cpu->chain, cpu->sdu,
			SDU_CTL_EHLT_CORE (cpu->core_num - core - 1));

      /* Release it from reset.  */
      rcu0_cn_res &= ~(1 << (cpu->core_num - core - 1));
      mmr_write (cpu->core_a, RCU0_CN_RES, rcu0_cn_res, 4);

      /* Wait for halt to happen.  */
      sdu_halt_wait (cpu->chain, cpu->sdu);

      /* Only unlocking Core 1 is supported */
      /* set RETE directly since setting RCU0_CN_CTL1 does not work.  */
      if (core == 0)
	{
	  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		    "%s: [%d] set PC to 0xff600000", bfin_target.name, core);
	  core_register_set (core, REG_RETE, 0xff600000);
	}

      rcu0_stat = mmr_read (cpu->core_a, RCU0_STAT, 4);
      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: RCU0_STAT  0x%x", bfin_target.name, rcu0_stat);

      rcu0_cn_stat = mmr_read (cpu->core_a, RCU0_CN_STAT, 4);
      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: RCU0_CN_STAT 0x%x <after>", bfin_target.name, rcu0_cn_stat);

      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: clear all bit in RCU0_CN_STAT", bfin_target.name);
      mmr_write (cpu->core_a, RCU0_CN_STAT, rcu0_cn_stat, 4);

      rcu0_cn_stat = mmr_read (cpu->core_a, RCU0_CN_STAT, 4);
      bfin_log (RP_VAL_LOGLEVEL_DEBUG,
		"%s: RCU0_CN_STAT 0x%x <after clear>", bfin_target.name, rcu0_cn_stat);
    }

  core_dbgstat_show (core, "after");

  core_check_emuready (core);
  cpu->cores[core].is_locked = 0;
  core_wpu_init (core);

  bfin_log (RP_VAL_LOGLEVEL_INFO,
	    "%s: [%d] done", bfin_target.name, cpu->first_core + core);
}

static void
core_lock (int core)
{
  uint32_t rcu0_cn_res;

  bfin_log (RP_VAL_LOGLEVEL_INFO,
	    "%s: [%d] locking...", bfin_target.name, cpu->first_core + core);

  if (cpu->sdu == -1)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] does not support being relocked",
		bfin_target.name, cpu->first_core + core);
      return;
    }

  core_dbgstat_show (core, "before");

  rcu0_cn_res = mmr_read (cpu->core_a, RCU0_CN_RES, 4);
  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: RCU0_CN_RES  0x%x <before>", bfin_target.name, rcu0_cn_res);

  /* Put it in reset.  */
  rcu0_cn_res |= 1 << (cpu->core_num - core - 1);
  mmr_write (cpu->core_a, RCU0_CN_RES, rcu0_cn_res, 4);

  core_dbgstat_show (core, "after");

  cpu->cores[core].is_locked = 1;

  bfin_log (RP_VAL_LOGLEVEL_INFO,
	    "%s: [%d] done", bfin_target.name, cpu->first_core + core);
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
  printf (" --debug-boot-code       stop processor at the start address of boot ROM\n");
  printf ("                         after reset.  Only for BF609\n");
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
  int cpu_num, first_core, core_num, sdu;

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
    {"debug-boot-code", no_argument, 0, 21},
    {"bf609-bmode1-workaround", no_argument, 0, 22},
    {"bf609-sdu-mac-read", no_argument, 0, 23},
    {"bf609-sdu-mac-write", no_argument, 0, 24},
    {"bf609-check-macrdy", no_argument, 0, 25},
    {"bf609-cclk", required_argument, 0, 26},
    {"bf609-sclk", required_argument, 0, 27},
    {"bf609-dclk", required_argument, 0, 28},
    {"bf609-ddrcl", required_argument, 0, 29},
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
	  else if (strcmp (optarg, "bf609-ezkit") == 0)
	    board = BF609_EZKIT;
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

	case 21:
	  bfin_debug_boot_code = 1;
	  break;

	case 22:
	  bfin_bf609_bmode1_workaround = 1;
	  break;

	case 23:
	  bfin_bf609_sdu_mac_read = 1;
	  break;

	case 24:
	  bfin_bf609_sdu_mac_write = 1;
	  break;

	case 25:
	  bfin_bf609_check_macrdy = 1;
	  break;

	case 26:
	  bfin_bf609_cclk = atol (optarg);
	  if (bfin_bf609_cclk < 0)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: bad CCLK %d",
			bfin_target.name, bfin_bf609_cclk);
	      exit (1);
	    }
	  break;

	case 27:
	  bfin_bf609_sclk = atol (optarg);
	  if (bfin_bf609_sclk < 0)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: bad SCLK %d",
			bfin_target.name, bfin_bf609_sclk);
	      exit (1);
	    }
	  break;

	case 28:
	  bfin_bf609_dclk = atol (optarg);
	  if (bfin_bf609_dclk < 0)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: bad DCLK %d",
			bfin_target.name, bfin_bf609_dclk);
	      exit (1);
	    }
	  break;

	case 29:
	  bfin_bf609_ddrcl = atol (optarg);
	  if (bfin_bf609_ddrcl < 2 || bfin_bf609_ddrcl > 6)
	    {
	      bfin_log (RP_VAL_LOGLEVEL_ERR,
			"%s: bad DDR CL %d, should be 2, 3, 4, 5, or 6.",
			bfin_target.name, bfin_bf609_ddrcl);
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

  /* Enable core scan path if an SDU is found.  */
  /* TODO We blindly enable core scan path for all SDUs for now.
	  Invent some options to control this behavior.  */
  for (i = 0; i < chain->parts->len; i++)
    if (!strcmp (chain->parts->parts[i]->part, "SDU")
	/* TODO && !sdu_core_scan_path_is_enabled (chain, i) */)
      {
	sdu_enable_core_scan_path (chain, i);

	/* Since the cores enabled by SDU don't have IDCODE register,
	   we need to set up configurations for them.

	   We assume SDU controls two cores and they come before it
	   in the scan chain.  */

	urj_tap_manual_add_at (chain, i++, "BF609", 5);
	urj_tap_manual_init (chain, "analog/bf609/bf609");

	urj_tap_manual_add_at (chain, i++, "BF609", 5);
	urj_tap_manual_init (chain, "analog/bf609/bf609");
      }

  first_core = -1;
  cpu_num = 0;
  sdu = -1;
  for (i = 0; i < chain->parts->len; i++)
    {
      /* If user does not specify a Blackfin processor for debugging,
	 use the first one.  */
      if ((bfin_processor == -1 && part_is_bfin (chain, i) && first_core == -1)
	  || cpu_num == bfin_processor)
	{
	  first_core = i;
	  if (!strcmp (chain->parts->parts[i]->part, "BF561") ||
	      !strcmp (chain->parts->parts[i]->part, "BF609"))
	    i++;
	  if (!strcmp (chain->parts->parts[i]->part, "BF609"))
	    sdu = i + 1;
	}
      else
	{
	  if (!strcmp (chain->parts->parts[i]->part, "BF561") ||
	      !strcmp (chain->parts->parts[i]->part, "BF609"))
	    i++;
	}
      cpu_num++;
    }

  if (first_core == -1)
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: no Blackfin processor is found", bfin_target.name);
      return RP_VAL_TARGETRET_ERR;
    }


  if (!strcmp (chain->parts->parts[first_core]->part, "BF561") ||
      !strcmp (chain->parts->parts[first_core]->part, "BF609"))
    core_num = 2;
  else
    core_num = 1;

  if (strcmp (chain->parts->parts[first_core]->part, "BF609"))
    {
      if (bfin_debug_boot_code)
	bfin_log (RP_VAL_LOGLEVEL_WARNING,
		  "%s: --debug-boot-code only valid for BF609, will be ignored",
		  bfin_target.name);

      if (bfin_bf609_bmode1_workaround)
	bfin_log (RP_VAL_LOGLEVEL_WARNING,
		  "%s: --bf609-bmode1-workaround only valid for BF609, will be ignored",
		  bfin_target.name);

      if (bfin_bf609_sdu_mac_read)
	bfin_log (RP_VAL_LOGLEVEL_WARNING,
		  "%s: --bf609-sdu-mac-read only valid for BF609, will be ignored",
		  bfin_target.name);

      if (bfin_bf609_sdu_mac_write)
	bfin_log (RP_VAL_LOGLEVEL_WARNING,
		  "%s: --bf609-sdu-mac-write only valid for BF609, will be ignored",
		  bfin_target.name);

      if (bfin_bf609_check_macrdy)
	bfin_log (RP_VAL_LOGLEVEL_WARNING,
		  "%s: --bf609-check-macrdy only valid for BF609, will be ignored",
		  bfin_target.name);
    }

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
  cpu->sdu = sdu;

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

  /* We need to set cores[n].is_dead here since later FOR_EACH_ALIVE_CORE
     will use is_dead.  We can use it to turn off a specific core.  */

  if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF506"))
    {
      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf50x_mem_map;
      cpu->cores[0].is_dead = 0;
      cpu->cores[0].l1_map = &bf50x_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF518"))
    {
      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf52x_mem_map;
      cpu->cores[0].is_dead = 0;
      cpu->cores[0].l1_map = &bf51x_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF526") ||
	   !strcmp (chain->parts->parts[cpu->first_core]->part, "BF527"))
    {
      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf52x_mem_map;
      cpu->cores[0].is_dead = 0;
      cpu->cores[0].l1_map = &bf52x_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF533"))
    {
      cpu->mdma_d0 = 0xffc00e00;
      cpu->mdma_s0 = 0xffc00e40;
      cpu->mem_map = bf533_mem_map;
      cpu->cores[0].is_dead = 0;
      cpu->cores[0].l1_map = &bf533_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF534") ||
           !strcmp (chain->parts->parts[cpu->first_core]->part, "BF537"))
    {
      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf537_mem_map;
      cpu->cores[0].is_dead = 0;
      cpu->cores[0].l1_map = &bf537_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF538"))
    {
      cpu->mdma_d0 = 0xffc00e00;
      cpu->mdma_s0 = 0xffc00e40;
      cpu->mem_map = bf538_mem_map;
      cpu->cores[0].is_dead = 0;
      cpu->cores[0].l1_map = &bf538_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF548") ||
           !strcmp (chain->parts->parts[cpu->first_core]->part, "BF548M"))
    {
      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf54x_mem_map;
      cpu->cores[0].is_dead = 0;
      cpu->cores[0].l1_map = &bf54x_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF561"))
    {
      cpu->mdma_d0 = 0xffc01800;
      cpu->mdma_s0 = 0xffc01840;
      cpu->mem_map = bf561_mem_map;
      cpu->cores[1].is_dead = 0;
      cpu->cores[0].is_dead = 0;
      cpu->cores[1].l1_map = &bf561_a_l1_map;
      cpu->cores[0].l1_map = &bf561_b_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF592"))
    {
      cpu->mdma_d0 = 0xffc00f00;
      cpu->mdma_s0 = 0xffc00f40;
      cpu->mem_map = bf59x_mem_map;
      cpu->cores[0].is_dead = 0;
      cpu->cores[0].l1_map = &bf59x_l1_map;
    }
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF609"))
    {
      cpu->mdma_d0 = 0;
      cpu->mdma_s0 = 0;
      cpu->mem_map = bf609_mem_map;
      cpu->cores[1].is_dead = 0;
      cpu->cores[0].is_dead = 0;
      cpu->cores[1].l1_map = &bf609_0_l1_map;
      cpu->cores[0].l1_map = &bf609_1_l1_map;
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

  cpu->sdram_config = NULL;
  cpu->ddr_config = NULL;
  cpu->ddr2 = 0;

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
      break;

    case BF609_EZKIT:
      if (chain->parts->len != 3)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %d cores on BF609 EZKIT board",
		    bfin_target.name, chain->parts->len);
	  exit (1);
	}
      if (strcmp (chain->parts->parts[0]->part, "BF609") != 0)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: found %s on BF609 EZKIT board",
		    bfin_target.name, chain->parts->parts[0]->part);
	  exit (1);
	}
      cpu->mem_map.sdram_end = 0x8000000;
      cpu->mem_map.flash_end = cpu->mem_map.flash + 0x2000000;
      cpu->ddr2 = 1;
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
		    "%s: no board selected, %d parts are detected",
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
	  bfin_log (RP_VAL_LOGLEVEL_WARNING, "%s:   parts: %s", bfin_target.name, buf);
	}
      break;

    default:
      abort ();
    }

  /* Assume only 32-bit instruction are used in gdbproxy.  */
  FOR_EACH_ALIVE_CORE (i, c)
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
  else if (!strcmp (chain->parts->parts[cpu->first_core]->part, "BF609"))
    {
      cpu->core_a = 1;
      cpu->cores[1].name = "Core 0";
      cpu->cores[0].name = "Core 1";
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
  int i;
  
  bfin_log (RP_VAL_LOGLEVEL_DEBUG, "%s: bfin_close()", bfin_target.name);

  assert (cpu);

  for (i = 0; i < cpu->core_num; i++)
    if (!cpu->cores[i].is_dead)
      core_emulation_disable (i);

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

  FOR_EACH_ALIVE_CORE (i, c)
    {
      /* We won't use IDCODE_SCAN in debugging. Set it as
         default, such that new scan will be selected.  */
      c->scan = IDCODE_SCAN;
      c->leave_stopped = 0;
      c->is_running = 1;
      c->is_interrupted = 0;
      c->is_stepping = 0;
      /* c->is_dead should be initialized in bfin_open.  */
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
  FOR_EACH_ALIVE_CORE (i, c)
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
	       && !core_is_locked (i)
	       && (!core_dbgstat_is_emuack (i)
		   || core_dbgstat_is_in_reset (i)))
	{
	  bfin_log (RP_VAL_LOGLEVEL_INFO,
		    "[%d] emulator not ready: DBGSTAT [0x%04X]",
		    cpu->first_core + i, BFIN_PART_DBGSTAT (part));
	  if (!(cpu->sdu != -1 && core_dbgstat_is_in_reset (i)))
	    need_reset = 1;
	}
    }

  if (need_reset || bfin_reset)
    {
      bfin_log (RP_VAL_LOGLEVEL_INFO, "Resetting ...");
      if (cpu->sdu != -1)
	{
	  emulation_disable ();

	  sdu_reset_assert (cpu->chain, cpu->sdu);

	  emulation_enable ();
	  sdu_halt_trigger (cpu->chain, cpu->sdu, SDU_CTL_EHLT_CORE_0);
	  sdu_reset_deassert (cpu->chain, cpu->sdu);
	  sdu_halt_wait (cpu->chain, cpu->sdu);

	  if (!bfin_debug_boot_code)
	    {
	      sdu_msg_set_haltonerror (cpu->core_a);
	      sdu_msg_set_haltoncall (cpu->core_a);
	      sdu_msg_set_haltoninit (cpu->core_a);
	      sdu_msg_set_haltonapp (cpu->core_a);

	      /* Work around a bug in boot rom code to boot from parallel
		 flash.
		 - set a hardware breakpoint at 0xc8000a32
		 - resume
		 - when halted, check R0 is 0x1003001
		 - change R0 to 0x1001001  */

#define BF609_BMODE1_WA_ADDR	0xc8000a32
#define BF609_BMODE1_WA_R0_BAD	0x01003001
#define BF609_BMODE1_WA_R0_GOOD	0x01001001

	      if (bfin_bf609_bmode1_workaround)
		{
		  int bootmode;

		  /* Check if it's Mode 1 */
		  bootmode = bfin_bootmode (cpu->core_a);
		  if (bootmode != 1)
		    {
		      bfin_log (RP_VAL_LOGLEVEL_WARNING,
				"%s: --bf609-bmode1-workaround will be ignored since boot mode is %d",
				bfin_target.name, bootmode);
		      bfin_bf609_bmode1_workaround = 0;
		    }
		}

	      if (bfin_bf609_bmode1_workaround)
		{
		  int ret;
		  uint32_t pc, r0;

		  core_wpu_init (cpu->core_a);
		  ret = bfin_add_break (1, BF609_BMODE1_WA_ADDR, 2);
		  if (ret != RP_VAL_TARGETRET_OK)
		    {
		      bfin_log (RP_VAL_LOGLEVEL_ERR,
				"%s: boot mode 1 workaround failed",
				bfin_target.name);
		      return ret;
		    }

		  core_emulation_return (cpu->core_a);
		  do
		    {
		      core_dbgstat_get (cpu->core_a);
		    }
		  while (!core_dbgstat_is_emuready (cpu->core_a));

		  pc = core_register_get (cpu->core_a, REG_RETE);
		  if (pc != BF609_BMODE1_WA_ADDR)
		    {
		      bfin_log (RP_VAL_LOGLEVEL_ERR,
				"%s: [%d] didn't halt at 0x%08x",
				bfin_target.name, cpu->core_a,
				BF609_BMODE1_WA_ADDR);
		      bfin_remove_break (1, BF609_BMODE1_WA_ADDR, 2);
		      return ret;
		    }
		  r0 = core_register_get (cpu->core_a, REG_R0);
		  if (r0 != BF609_BMODE1_WA_R0_BAD)
		    {
		      bfin_log (RP_VAL_LOGLEVEL_ERR,
				"%s: [%d] R0 is 0x%08x, not expected 0x%08x",
				bfin_target.name, cpu->core_a, r0,
				BF609_BMODE1_WA_R0_BAD);
		      bfin_remove_break (1, BF609_BMODE1_WA_ADDR, 2);
		      return ret;
		    }

		  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
			    "%s: [%d] set R0 to 0x%08x",
			    bfin_target.name, cpu->core_a,
			    BF609_BMODE1_WA_R0_GOOD);
		  core_register_set (cpu->core_a, REG_R0, BF609_BMODE1_WA_R0_GOOD);

		  bfin_remove_break (1, BF609_BMODE1_WA_ADDR, 2);

#undef BF609_BMODE1_WA_ADDR
#undef BF609_BMODE1_WA_R0_BAD
#undef BF609_BMODE1_WA_R0_GOOD
		}

		{
		  int count = 0;

		  core_emulation_return (cpu->core_a);
		  do
		    {
		      core_dbgstat_get (cpu->core_a);
		      count++;
		      /* A rather random limit */
		      if (count == 100)
			break;
		    }
		  while (!core_dbgstat_is_emuready (cpu->core_a));
		}

	      if (core_dbgstat_is_emuready (cpu->core_a))
		{
		  if (sdu_msg_is_callapp (cpu->core_a))
		    {
		      sdu_msg_clear_haltonapp (cpu->core_a);
		      sdu_msg_clear_callapp (cpu->core_a);

		      core_single_step (cpu->core_a);
		      bfin_log (RP_VAL_LOGLEVEL_INFO,
				"[%d] halted on application",
				cpu->first_core + cpu->core_a);
		    }
		  else if (sdu_msg_is_callerror (cpu->core_a))
		    {
		      sdu_msg_clear_haltonerror (cpu->core_a);
		      sdu_msg_clear_callerror (cpu->core_a);

		      bfin_log (RP_VAL_LOGLEVEL_INFO,
				"[%d] halted on callerror",
				cpu->first_core + cpu->core_a);
		    }
		  else if (sdu_msg_is_callback (cpu->core_a))
		    {
		      sdu_msg_clear_haltoncall (cpu->core_a);
		      sdu_msg_clear_callback (cpu->core_a);

		      bfin_log (RP_VAL_LOGLEVEL_INFO,
				"[%d] halted on callback",
				cpu->first_core + cpu->core_a);
		    }
		  else if (sdu_msg_is_callinit (cpu->core_a))
		    {
		      sdu_msg_clear_haltoninit (cpu->core_a);
		      sdu_msg_clear_callinit (cpu->core_a);

		      bfin_log (RP_VAL_LOGLEVEL_INFO,
				"[%d] halted on callinit",
				cpu->first_core + cpu->core_a);
		    }
		  else
		    {
		      bfin_log (RP_VAL_LOGLEVEL_INFO,
				"[%d] halted on unknown reason",
				cpu->first_core + cpu->core_a);
		    }
		}
	      else
		{
		  core_emulation_trigger (cpu->core_a);
		  bfin_log (RP_VAL_LOGLEVEL_INFO,
			    "[%d] forced to halt",
			    cpu->first_core + cpu->core_a);
		}
	    }
	}
      else
	{
	  core_reset ();

	  /* FIXME  Find a better way to identify if the system exists.  */
	  if (cpu->mdma_d0)
	    system_reset ();
	}
    }

  dbgstat_get ();

  FOR_EACH_ALIVE_CORE (i, c)
    if (core_is_locked (i)
	&& (cpu->sdu != -1
	    || (core_dbgstat_is_emuack (i)
		&& !core_dbgstat_is_in_reset (i))))
      {
	urj_part_t *part = cpu->chain->parts->parts[cpu->first_core + i];

	bfin_log (RP_VAL_LOGLEVEL_INFO,
		  "[%d] locked: DBGSTAT [0x%04X]", cpu->first_core + i, BFIN_PART_DBGSTAT (part));
	c->is_locked = 1;
	c->is_running = 0;

	if (bfin_unlock_on_connect)
	  core_unlock (i);
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

  /* If any bf609 clock option is specified, initialize PLL.  If
     SDRAM initialization option is specified, we also need initialize
     PLL since the default DCLK is too low on BF609 EZKIT.  */

  if (cpu->sdu
      && (bfin_bf609_cclk || bfin_bf609_sclk || bfin_bf609_dclk
          || bfin_init_sdram))
    {
      if (calculate_clocks () != 0)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: Calculate clocks failed", bfin_target.name);
	  return RP_VAL_TARGETRET_ERR;
	}

      if (pll_init () != 0)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: PLL init failed", bfin_target.name);
	  return RP_VAL_TARGETRET_ERR;
	}
    }

  if (bfin_init_sdram && cpu->ddr2)
    {
      if (ddr2_init () != 0)
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: DDR2 init failed", bfin_target.name);
	  return RP_VAL_TARGETRET_ERR;
	}
    }

  if (bfin_enable_dcache)
    dcache_enable (bfin_enable_dcache);

  if (bfin_enable_icache)
    icache_enable ();

  if (rp_debug_level)
    {
      uint32_t rete;

      dbgstat_get ();

      FOR_EACH_ALIVE_CORE (i, c)
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
	if (!cpu->cores[i].is_dead)
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

  FOR_EACH_ALIVE_CORE (i, c)
    if (c->is_stepping)
      {
	core_dbgctl_bit_clear_esstep (i, UPDATE);
	c->is_stepping = 0;
      }

  emulation_return ();

  FOR_EACH_ALIVE_CORE (i, c)
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

  FOR_EACH_ALIVE_CORE (i, c)
    if (c->is_stepping)
      {
	core_dbgctl_bit_clear_esstep (i, UPDATE);
	c->is_stepping = 0;
      }

  emulation_trigger ();

  FOR_EACH_ALIVE_CORE (i, c)
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
      FOR_EACH_ALIVE_CORE (i, c)
	c->leave_stopped = 0;
    }
  else if (thread->val == ANY_THREAD)
    {
      if (cpu->continue_core == INVALID_CORE)
	cpu->continue_core = cpu->core_a;
      FOR_EACH_ALIVE_CORE (i, c)
	c->leave_stopped = 0;
    }
  else
    {
      core = PART_NO (thread->val);

      if (core < 0 || core >= cpu->core_num)
	return RP_VAL_TARGETRET_ERR;

      cpu->continue_core = core;
      FOR_EACH_ALIVE_CORE (i, c)
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

      FOR_EACH_ALIVE_CORE (i, c)
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
    FOR_EACH_ALIVE_CORE (i, c)
      {
	core_dbgstat_get (i);
	if (core_dbgstat_is_emuready (i))
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

  FOR_EACH_ALIVE_CORE (i, c)
    {
      /* Use the core for its L1 memory if possible.  */
      core_dbgstat_get (i);
      if (core_dbgstat_is_emuready (i))
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

	  ret = sram_read (core, addr, buf, req_size, i == core);
	}
      else if (!c->l1_code_cache_enabled &&
	       IN_MAP (addr, c->l1_map->l1_code_cache))
	{
	  if (addr + req_size > c->l1_map->l1_code_cache_end)
	    req_size = c->l1_map->l1_code_cache_end - addr;

	  ret = sram_read (core, addr, buf, req_size, i == core);
	}
      else if (IN_MAP (addr, c->l1_map->l1_code_rom))
	{
	  if (addr + req_size > c->l1_map->l1_code_rom_end)
	    req_size = c->l1_map->l1_code_rom_end - addr;

	  ret = sram_read (core, addr, buf, req_size, i == core);
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
	}
      else if (!c->l1_data_a_cache_enabled &&
	       IN_MAP (addr, c->l1_map->l1_data_a_cache))
	{
	  if (addr + req_size > c->l1_map->l1_data_a_cache_end)
	    req_size = c->l1_map->l1_data_a_cache_end - addr;

	  ret = core_memory_read (core, addr, buf, req_size, i == core);
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
	}
      else if (!c->l1_data_b_cache_enabled &&
	       IN_MAP (addr, c->l1_map->l1_data_b_cache))
	{
	  if (addr + req_size > c->l1_map->l1_data_b_cache_end)
	    req_size = c->l1_map->l1_data_b_cache_end - addr;

	  ret = core_memory_read (core, addr, buf, req_size, i == core);
	}
      else if (i == core && IN_MAP (addr, c->l1_map->l1_scratch))
	{
	  if (addr + req_size > c->l1_map->l1_scratch_end)
	    req_size = c->l1_map->l1_scratch_end - addr;

	  ret = memory_read (core, addr, buf, req_size);
	}
      else
	{
	  ret = bfin_read_inv_mem (core, addr, buf, &req_size);
	}

      goto done;
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
  int was_locked;

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
    FOR_EACH_ALIVE_CORE (i, c)
      {
	core_dbgstat_get (i);
	if (core_dbgstat_is_emuready (i))
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

  was_locked = 0;

  if (!IN_MAP (addr, cpu->mem_map.l1))
    goto skip_l1;

  FOR_EACH_ALIVE_CORE (i, c)
    {
      if (!IN_MAP (addr, c->l1_map->l1))
	continue;

      if (c->is_locked
	  && (cpu->sdu != -1
	      || bfin_unlock_on_load))
	{
	  was_locked = c->is_locked;

	  core_unlock (i);

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
	}
      else
	core_cache_status_get (i);

      /* Use the core for its L1 memory if possible.  */
      core_dbgstat_get (i);
      if (core_dbgstat_is_emuready (i))
	core = i;
      else
	core = avail_core;

      if (IN_MAP (addr, c->l1_map->l1_code) &&
	  ((!c->l1_code_cache_enabled &&
	    (c->l1_map->l1_code_end == c->l1_map->l1_code_cache)
	    && (end = c->l1_map->l1_code_cache_end))
	   || (end = c->l1_map->l1_code_end))
	  && addr + write_size <= end)
	{
	  ret = sram_write (core, addr, buf, write_size, i == core);

	  if (i == core)
	    icache_flush (i, addr, write_size);
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

	  ret = sram_write (core, addr, buf, write_size, i == core);
	}
      else if (IN_MAP (addr, c->l1_map->l1_data_a) &&
	       ((!c->l1_data_a_cache_enabled &&
		 (c->l1_map->l1_data_a_end == c->l1_map->l1_data_a_cache)
		 && (end = c->l1_map->l1_data_a_cache_end))
		|| (end = c->l1_map->l1_data_a_end))
	       && addr + write_size <= end)
	{
	  ret = core_memory_write (core, addr, buf, write_size, i == core);
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
	}
      else if (IN_MAP (addr, c->l1_map->l1_data_b) &&
	       ((!c->l1_data_b_cache_enabled &&
		 (c->l1_map->l1_data_b_end == c->l1_map->l1_data_b_cache)
		 && (end = c->l1_map->l1_data_b_cache_end))
		|| (end = c->l1_map->l1_data_b_end))
	       && addr + write_size <= end)
	{
	  ret = core_memory_write (core, addr, buf, write_size, i == core);
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
	}
      else if (i == core && IN_MAP (addr, c->l1_map->l1_scratch) &&
	       addr + write_size <= c->l1_map->l1_scratch_end)
	{
	  ret = memory_write (core, addr, buf, write_size);
	}
      else
	{
	  bfin_log (RP_VAL_LOGLEVEL_ERR,
		    "%s: [%d] cannot write reserved L1 [0x%08llX] size %d",
		    bfin_target.name, cpu->first_core + i, addr, write_size);
	  return RP_VAL_TARGETRET_ERR;
	}

      core_check_emuready (core);

      if (cpu->sdu != -1 && was_locked && !bfin_unlock_on_load)
	core_lock (i);

      goto done;
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
    }
  else if ((IN_MAP (addr, cpu->mem_map.sdram) &&
	    addr + write_size <= cpu->mem_map.sdram_end)
	   || (IN_MAP (addr, cpu->mem_map.async_mem) &&
	       addr + write_size <= cpu->mem_map.async_mem_end)
	   || (IN_MAP (addr, cpu->mem_map.l2_sram) &&
	       addr + write_size <= cpu->mem_map.l2_sram_end))
    {
      ret = memory_write (core, addr, buf, write_size);

      FOR_EACH_ALIVE_CORE (i, c)
	{
	  core_dbgstat_get (i);
	  if (core_dbgstat_is_emuready (i))
	    icache_flush (i, addr, write_size);
	}
    }
  else
    {
      bfin_log (RP_VAL_LOGLEVEL_ERR,
		"%s: [%d] cannot write memory [0x%08llX]",
		bfin_target.name, cpu->first_core + core, addr);
      return RP_VAL_TARGETRET_ERR;
    }

  core_check_emuready (core);

done:

  bfin_log (RP_VAL_LOGLEVEL_DEBUG,
	    "%s: bfin_write_mem () through Core [%d]",
	    bfin_target.name, cpu->first_core + core);

  if (ret < 0)
    return RP_VAL_TARGETRET_ERR;

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

  FOR_EACH_ALIVE_CORE (i, c)
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

  FOR_EACH_ALIVE_CORE (i, c)
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
  FOR_EACH_ALIVE_CORE (i, c)
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

  FOR_EACH_ALIVE_CORE (i, c)
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
      else if (core_dbgstat_is_in_reset (i))
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
      FOR_EACH_ALIVE_CORE (i, c)
	c->is_running = 0;
    }

  /* All cores are stopped. Check their status.  */

  emupc_get (1);
  emupc_reset ();
  dbgstat_get ();

  FOR_EACH_ALIVE_CORE (i, c)
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

  FOR_EACH_ALIVE_CORE (i, c)
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
      FOR_EACH_ALIVE_CORE (i, c)
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

      FOR_EACH_ALIVE_CORE (i, c)
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
  FOR_EACH_ALIVE_CORE (i, c)
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
	  FOR_EACH_ALIVE_CORE (i, c)
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

      FOR_EACH_ALIVE_CORE (i, c)
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

      FOR_EACH_ALIVE_CORE (i, c)
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
      FOR_EACH_ALIVE_CORE (i, c)
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

  FOR_EACH_ALIVE_CORE (i, c)
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

  FOR_EACH_ALIVE_CORE (i, c)
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
	  FOR_EACH_ALIVE_CORE (i, c)
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

      FOR_EACH_ALIVE_CORE (i, c)
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

      FOR_EACH_ALIVE_CORE (i, c)
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
      FOR_EACH_ALIVE_CORE (i, c)
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

  FOR_EACH_ALIVE_CORE (i, c)
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

  FOR_EACH_ALIVE_CORE (i, c)
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
  software_reset (cpu->chain, cpu->first_core + cpu->core_a);
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
