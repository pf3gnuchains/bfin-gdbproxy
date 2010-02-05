#
#
# Copyright (C) 1999-2001 Quality Quorum, Inc.
# Copyright (C) 2002 Chris Liechti and Steve Underwood
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. The name of the author may not be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
# EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# QQI can be contacted as qqi@world.std.com
# 
#
# This is a part of gdbproxy - a remote proxy server for gdb 
#

SUBDIRS = gnulib/lib

ACLOCAL_AMFLAGS = -I gnulib/m4

AUTOMAKE_OPTIONS = gnu

bin_PROGRAMS = gdbproxy

if USE_RC_SOURCES
RC_SOURCES = gdbproxy-icon.rc
else
RC_SOURCES =
endif
gdbproxy_SOURCES = copying.c init.c remote.c gdbproxy.c rpmisc.c serial.c target_bfin_new.c gdbproxy.h rpmisc.h serial.h $(RC_SOURCES)

AM_CPPFLAGS = \
	-I$(top_builddir)/gnulib/lib -I$(top_srcdir)/gnulib/lib \
	-I$(jtag_srcdir)/include \
	-DGDBPROXY_BIN_DIR=\"$(bindir)\" -DGDBPROXY_DATA_DIR=\"$(pkgdatadir)\"
gdbproxy_LDADD = \
	$(jtag_builddir)/src/.libs/liburjtag.a \
	-lm \
	$(top_builddir)/gnulib/lib/libgnu.a \
	$(LIB_CLOCK_GETTIME) $(LIB_NANOSLEEP) $(LIBINTL)

dist_pkgdata_DATA = bfin

info_TEXINFOS = gdbproxy.texi

man_MANS = gdbproxy.1

EXTRA_DIST = gdbproxy.1 Makefile.am bootstrap gnulib/m4/gnulib-cache.m4

CLEANFILES = configure.scan res tmp.tmp

MAINTAINERCLEANFILES = aclocal.m4 configure config.h.in stamp-h.in Makefile.in

.rc.o:
	$(WINDRES) -I $(srcdir) $< -o $@
%.o : %.rc
	$(WINDRES) -I $(srcdir) $^ -o $@
