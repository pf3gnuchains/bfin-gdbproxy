/* Copyright (C) 2009 Analog Devices, Inc.

   This file is subject to the terms and conditions of the GNU
   General Public License as published by the Free Software
   Foundation; either version 2, or (at your option) any later
   version.  See the file COPYING for more details.

   This file is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   Non-GPL License is also available.  Please contact
   <david.babicz@analog.com> for more information.  */

#ifndef _CIRC_BUF_
#define _CIRC_BUF_

#include <stdint.h>

#define CIRC_SIZE 8192
#define CIRC_MASK (CIRC_SIZE - 1)
struct circ_buf {
	char _buf[CIRC_SIZE];
	int head, tail;
};
#define circ_clear(c) (c)->head = (c)->tail
#define circ_empty(c) ((c)->head == (c)->tail)
#define circ_full(c)  (circ_cnt(c) == CIRC_MASK)
#define circ_cnt(c)   (((c)->head - (c)->tail) & (CIRC_MASK))
#define circ_free(c)  (CIRC_MASK - circ_cnt(c))
#define circ_puts(c, buf, cnt) do { ssize_t _i, _cnt = (cnt); for (_i = 0; _i < _cnt; ++_i) (c)->_buf[(c)->head++ & CIRC_MASK] = (buf)[_i]; } while (0)
#define circ_gets(c, buf, cnt) do { ssize_t _i, _cnt = (cnt); for (_i = 0; _i < _cnt; ++_i) (buf)[_i] = (c)->_buf[(c)->tail++ & CIRC_MASK]; } while (0)

#endif
