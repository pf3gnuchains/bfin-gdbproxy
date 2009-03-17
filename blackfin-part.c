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
#include <string.h>

#include "part.h"
#include "blackfin-part.h"

/* Wrappers of the helper functions.  */

void
part_dbgctl_init (part_t *part, uint16_t value)
{
  EMU_OAB (part)->dbgctl_init (part, value);
}

uint16_t
part_dbgstat_value (part_t *part)
{
  return EMU_OAB (part)->dbgstat_value (part);
}

/* Routines to access DBGCTL and DBGSTAT bits.  */

#define PART_DBGCTL_CLEAR_OR_SET_BIT(name)				\
  uint16_t								\
  part_dbgctl_bit_clear_or_set_##name (part_t *part, uint16_t dbgctl, int set) \
  {									\
    if (set)								\
      return dbgctl | EMU_OAB (part)->dbgctl_##name;			\
    else								\
      return dbgctl & ~EMU_OAB (part)->dbgctl_##name;			\
  }

#define PART_DBGCTL_BIT_IS(name)					\
  int									\
  part_dbgctl_is_##name (part_t *part, uint16_t dbgctl)			\
  {									\
    if (dbgctl & EMU_OAB (part)->dbgctl_##name)				\
      return 1;								\
    else								\
      return 0;								\
  }

#define PART_DBGCTL(name)						\
  PART_DBGCTL_CLEAR_OR_SET_BIT(name)					\
  PART_DBGCTL_BIT_IS(name)

#define PART_DBGCTL_BIT_IS_MASK(base, sfx)				\
  int									\
  part_dbgctl_is_##base##_##sfx (part_t *part, uint16_t dbgctl)		\
  {									\
    if ((dbgctl & EMU_OAB (part)->dbgctl_##base##_mask) ==		\
	EMU_OAB (part)->dbgctl_##base##_##sfx)				\
      return 1;								\
    else								\
      return 0;								\
  }

#define PART_DBGCTL_MASK(base, sfx)					\
  PART_DBGCTL_CLEAR_OR_SET_BIT(base##_##sfx)				\
  PART_DBGCTL_BIT_IS_MASK(base, sfx)

PART_DBGCTL (sram_init)
PART_DBGCTL (wakeup)
PART_DBGCTL (sysrst)
PART_DBGCTL (esstep)
PART_DBGCTL_MASK (emudatsz, 32)
PART_DBGCTL_MASK (emudatsz, 40)
PART_DBGCTL_MASK (emudatsz, 48)
PART_DBGCTL (emuirlpsz_2)
PART_DBGCTL_MASK (emuirsz, 64)
PART_DBGCTL_MASK (emuirsz, 48)
PART_DBGCTL_MASK (emuirsz, 32)
PART_DBGCTL (empen)
PART_DBGCTL (emeen)
PART_DBGCTL (emfen)
PART_DBGCTL (empwr)

#define PART_DBGSTAT_BIT_IS(name)					\
  int									\
  part_dbgstat_is_##name (part_t *part, uint16_t dbgstat)		\
  {									\
    if (dbgstat & EMU_OAB (part)->dbgstat_##name)			\
      return 1;								\
    else								\
      return 0;								\
  }

#define PART_DBGSTAT_CLEAR_BIT(name)					\
  uint16_t								\
  part_dbgstat_bit_clear_##name (part_t *part, uint16_t dbgstat)	\
  {									\
    return dbgstat & ~EMU_OAB (part)->dbgstat_##name;			\
  }

#define PART_DBGSTAT_SET_BIT(name)					\
  uint16_t								\
  part_dbgstat_bit_set_##name (part_t *part, uint16_t dbgstat)		\
  {									\
    return dbgstat | EMU_OAB (part)->dbgstat_##name;			\
  }

PART_DBGSTAT_BIT_IS (lpdec1)
PART_DBGSTAT_BIT_IS (in_powrgate)
PART_DBGSTAT_BIT_IS (core_fault)
PART_DBGSTAT_BIT_IS (idle)
PART_DBGSTAT_BIT_IS (in_reset)
PART_DBGSTAT_BIT_IS (lpdec0)
PART_DBGSTAT_BIT_IS (bist_done)
PART_DBGSTAT_BIT_IS (emuack)
PART_DBGSTAT_BIT_IS (emuready)
PART_DBGSTAT_BIT_IS (emudiovf)
PART_DBGSTAT_BIT_IS (emudoovf)
PART_DBGSTAT_BIT_IS (emudif)
PART_DBGSTAT_BIT_IS (emudof)

PART_DBGSTAT_CLEAR_BIT (emudiovf)
PART_DBGSTAT_CLEAR_BIT (emudoovf)

PART_DBGSTAT_SET_BIT (emudiovf)
PART_DBGSTAT_SET_BIT (emudoovf)

int
part_dbgctl_dbgstat_in_one_chain (part_t *part)
{
  return EMU_OAB (part)->dbgctl_dbgstat_in_one_chain;
}

int
part_sticky_in_reset (part_t *part)
{
  return EMU_OAB (part)->sticky_in_reset;
}

uint16_t
part_dbgstat_emucause_mask (part_t *part)
{
  return EMU_OAB (part)->dbgstat_emucause_mask;
}

tap_register *
register_init_value (tap_register *tr, uint64_t value)
{
  int i;

  //  assert (tr->len <= 64);

  for (i = 0; i < tr->len; i++)
    tr->data[i] = (value >> (tr->len - i - 1)) & 1;

  return tr;
}

uint64_t
register_value (tap_register *tr)
{
  uint64_t v = 0;
  int i;

  //  assert (tr->len <= 64);

  for (i = 0; i < tr->len; i++)
    v = (v << 1) | tr->data[i];

  return v;
}
