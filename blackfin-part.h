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


#ifndef _BLACKFIN_PART_H_
#define _BLACKFIN_PART_H_

#include "part.h"

/* TODO  Turn these macros into EMU_OAB method calls.  */

#define DTEST_COMMAND			0xffe00300
#define DTEST_DATA0			0xffe00400
#define DTEST_DATA1			0xffe00404

#define ITEST_COMMAND			0xffe01300
#define ITEST_DATA0			0xffe01400
#define ITEST_DATA1			0xffe01404


/* OAB stands for Operations and Bits.  */

struct emu_oab
{
  /* Operations */

  void (*dbgctl_init) (part_t *part, uint16_t value);
  uint16_t (*dbgstat_value) (part_t *part);

  /* Generate TEST_COMMAND from ADDR and W(rite).  */
  uint32_t (*test_command) (uint32_t addr, int w);

  /* For BF579 it's ITEST_COMMAND address. But for other Blackfin
     processors, it's actually DTEST_COMMAND address.  */
  uint32_t test_command_addr;

  /* For BF579 they are addresses for ITEST_DATA0 and ITEST_DATA1.
     But for other Blackfin processors, they are actually DTEST_DATA
     addresses.  */
  uint32_t test_data0_addr;
  uint32_t test_data1_addr;

  /* For BF579 DBGCTL and DBGSTAT is in one chain.  */
  int dbgctl_dbgstat_in_one_chain;

  /* The IN_RESET bit in DBGSTAT for BF579 are sticky, but not for
     others.  */
  int sticky_in_reset;

  /* Bits */
  uint16_t dbgctl_sram_init;
  uint16_t dbgctl_wakeup;
  uint16_t dbgctl_sysrst;
  uint16_t dbgctl_esstep;
  uint16_t dbgctl_emudatsz_32;
  uint16_t dbgctl_emudatsz_40;
  uint16_t dbgctl_emudatsz_48;
  uint16_t dbgctl_emudatsz_mask;
  uint16_t dbgctl_emuirlpsz_2;
  uint16_t dbgctl_emuirsz_64;
  uint16_t dbgctl_emuirsz_48;
  uint16_t dbgctl_emuirsz_32;
  uint16_t dbgctl_emuirsz_mask;
  uint16_t dbgctl_empen;
  uint16_t dbgctl_emeen;
  uint16_t dbgctl_emfen;
  uint16_t dbgctl_empwr;

  uint16_t dbgstat_lpdec1;
  uint16_t dbgstat_in_powrgate;
  uint16_t dbgstat_core_fault;
  uint16_t dbgstat_idle;
  uint16_t dbgstat_in_reset;
  uint16_t dbgstat_lpdec0;
  uint16_t dbgstat_bist_done;
  uint16_t dbgstat_emucause_mask;
  uint16_t dbgstat_emuack;
  uint16_t dbgstat_emuready;
  uint16_t dbgstat_emudiovf;
  uint16_t dbgstat_emudoovf;
  uint16_t dbgstat_emudif;
  uint16_t dbgstat_emudof;
};

#define EMU_OAB(part)			((struct emu_oab *)(part->data))

extern struct emu_oab bfin_emu_oab;
extern struct emu_oab bf579_emu_oab;

extern void part_dbgctl_init (part_t *part, uint16_t value);
extern uint16_t part_dbgstat_value (part_t *part);
extern uint16_t part_dbgstat_emucause_mask (part_t *part);

#define DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT(name)				\
  extern uint16_t part_dbgctl_bit_clear_or_set_##name (part_t *part, uint16_t dbgctl, int set);

#define DECLARE_PART_DBGSTAT_BIT_IS(name)					\
  extern int part_dbgstat_is_##name (part_t *part, uint16_t dbgstat);

#define DECLARE_PART_DBGSTAT_CLEAR_BIT(name)					\
  extern uint16_t part_dbgstat_bit_clear_##name (part_t *part, uint16_t gdbstat);

#define DECLARE_PART_DBGSTAT_SET_BIT(name)					\
  extern uint16_t part_dbgstat_bit_set_##name (part_t *part, uint16_t gdbstat);


DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (sram_init)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (wakeup)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (sysrst)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (esstep)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (emudatsz_32)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (emudatsz_40)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (emudatsz_48)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (emuirlpsz_2)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (emuirsz_64)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (emuirsz_48)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (emuirsz_32)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (empen)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (emeen)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (emfen)
DECLARE_PART_DBGCTL_CLEAR_OR_SET_BIT (empwr)

DECLARE_PART_DBGSTAT_BIT_IS (lpdec1)
DECLARE_PART_DBGSTAT_BIT_IS (in_powrgate)
DECLARE_PART_DBGSTAT_BIT_IS (core_fault)
DECLARE_PART_DBGSTAT_BIT_IS (idle)
DECLARE_PART_DBGSTAT_BIT_IS (in_reset)
DECLARE_PART_DBGSTAT_BIT_IS (lpdec0)
DECLARE_PART_DBGSTAT_BIT_IS (bist_done)
DECLARE_PART_DBGSTAT_BIT_IS (emuack)
DECLARE_PART_DBGSTAT_BIT_IS (emuready)
DECLARE_PART_DBGSTAT_BIT_IS (emudiovf)
DECLARE_PART_DBGSTAT_BIT_IS (emudoovf)
DECLARE_PART_DBGSTAT_BIT_IS (emudif)
DECLARE_PART_DBGSTAT_BIT_IS (emudof)

DECLARE_PART_DBGSTAT_CLEAR_BIT (emudiovf)
DECLARE_PART_DBGSTAT_CLEAR_BIT (emudoovf)

DECLARE_PART_DBGSTAT_SET_BIT (emudiovf)
DECLARE_PART_DBGSTAT_SET_BIT (emudoovf)

extern int part_sticky_in_reset (part_t *part);
extern int part_dbgctl_dbgstat_in_one_chain (part_t *part);

extern tap_register *register_init_value (tap_register *tr, uint64_t value);
extern uint64_t register_value (tap_register *tr);

#endif /* _BLACKFIN_PART_H_ */
