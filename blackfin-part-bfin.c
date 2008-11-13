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


#include <stdint.h>
#include <assert.h>
 
#include "part.h"
#include "blackfin-part.h"

/* The helper functions for Blackfin DBGCTL and DBGSTAT operations.  */

static void
bfin_dbgctl_set (part_t *part, uint16_t v)
{
  register_init_value (part->active_instruction->data_register->in, v);
}

static void
bfin_dbgstat_pre_get (part_t *part, uint16_t value)
{
  return;
}

static uint16_t
bfin_dbgstat_get (part_t *part)
{
  return register_value (part->active_instruction->data_register->out);
}

static uint32_t
bfin_test_command (uint32_t addr, int w)
{
  uint32_t test_command;

  /* We can only access [15:0] range.  */
  if ((addr & 0xf0000) != 0)
    return 0;

  test_command =
    (addr & 0x0800) << 15	/* Address bit 11 */
    | (addr & 0x8000) << 8	/* Address bit 15 */
    | (addr & 0x3000) << 4	/* Address bits [13:12] */
    | (addr & 0x47f8)		/* Address bits 14 and [10:3] */
    | 0x1000000			/* Access instruction */
    | 0x4;			/* Access data array */

  if (w)
    test_command |= 0x2;	/* Write */

  return test_command;
}

struct emu_oab bfin_emu_oab =
{
  bfin_dbgctl_set,
  bfin_dbgstat_get,

  bfin_test_command,

  DTEST_COMMAND,
  DTEST_DATA0,
  DTEST_DATA1,

  0, /* dbgctl_dbgstat_in_one_chain */
  0, /* sticky_in_reset */

  0x1000, /* DBGCTL_SRAM_INIT */
  0x0800, /* DBGCTL_WAKEUP */
  0x0400, /* DBGCTL_SYSRST */
  0x0200, /* DBGCTL_ESSTEP */
  0x0000, /* DBGCTL_EMUDATSZ_32 */
  0x0080, /* DBGCTL_EMUDATSZ_40 */
  0x0100, /* DBGCTL_EMUDATSZ_48 */
  0x0180, /* DBGCTL_EMUDATSZ_MASK */
  0x0040, /* DBGCTL_EMUIRLPSZ_2 */
  0x0000, /* DBGCTL_EMUIRSZ_64 */
  0x0010, /* DBGCTL_EMUIRSZ_48 */
  0x0020, /* DBGCTL_EMUIRSZ_32 */
  0x0030, /* DBGCTL_EMUIRSZ_MASK */
  0x0008, /* DBGCTL_EMPEN */
  0x0004, /* DBGCTL_EMEEN */
  0x0002, /* DBGCTL_EMFEN */
  0x0001, /* DBGCTL_EMPWR */

  0x8000, /* DBGSTAT_LPDEC1 */
  0x0000, /* No DBGSTAT_IN_POWRGATE for bfin */
  0x4000, /* DBGSTAT_CORE_FAULT */
  0x2000, /* DBGSTAT_IDLE */
  0x1000, /* DBGSTAT_IN_RESET */
  0x0800, /* DBGSTAT_LPDEC0 */
  0x0400, /* DBGSTAT_BIST_DONE */
  0x03c0, /* DBGSTAT_EMUCAUSE_MASK */
  0x0020, /* DBGSTAT_EMUACK */
  0x0010, /* DBGSTAT_EMUREADY */
  0x0008, /* DBGSTAT_EMUDIOVF */
  0x0004, /* DBGSTAT_EMUDOOVF */
  0x0002, /* DBGSTAT_EMUDIF */
  0x0001, /* DBGSTAT_EMUDOF */
};
