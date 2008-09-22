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

/* The helper functions for BF579 DBGCTL and DBGSTAT operations.  */

static void
bf579_dbgctl_init (part_t *part, uint16_t v)
{
  register_init_value (part->active_instruction->data_register->in, v << 16);
}

static uint16_t
bf579_dbgstat_value (part_t *part)
{
  return register_value (part->active_instruction->data_register->out);
}

static uint32_t
bf579_test_command (uint32_t addr, int w)
{
  uint32_t test_command;

  test_command =
    (addr & 0xfff8)		/* Address bits [15:3] */
    | 0x1;			/* Access to ISRAM */

  if (w)			/* Write */
    test_command |= 0x2;

  return test_command;
}

struct emu_oab bf579_emu_oab =
{
  bf579_dbgctl_init,
  bf579_dbgstat_value,

  bf579_test_command,

  ITEST_COMMAND,
  ITEST_DATA0,
  ITEST_DATA1,

  1, /* dbgctl_dbgstat_in_one_chain */
  1, /* sticky_in_reset */

  0x0800, /* DBGCTL_SRAM_INIT */
  0x0400, /* DBGCTL_WAKEUP */
  0x0200, /* DBGCTL_SYSRST */
  0x0100, /* DBGCTL_ESSTEP */
  0x0000, /* DBGCTL_EMUDATSZ_32 */
  0x0,    /* No DBGCTL_EMUDATSZ_40 for bf579 */
  0x0080, /* DBGCTL_EMUDATSZ_48 */
  0x0080, /* DBGCTL_EMUDATSZ_MASK */
  0x0040, /* DBGCTL_EMUIRLPSZ_2 */
  0x0000, /* DBGCTL_EMUIRSZ_64 */
  0x0010, /* DBGCTL_EMUIRSZ_48 */
  0x0020, /* DBGCTL_EMUIRSZ_32 */
  0x0030, /* DBGCTL_EMUIRSZ_MASK */
  0x0008, /* DBGCTL_EMPEN */
  0x0004, /* DBGCTL_EMEEN */
  0x0002, /* DBGCTL_EMFEN */
  0x0001, /* DBGCTL_EMPWR */

  0x0000, /* No DBGSTAT_LPDEC1 for bf579 */
  0x8000, /* DBGSTAT_IN_POWRGATE */
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
