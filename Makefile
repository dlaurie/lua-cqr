# Based on LHF's Makefile in lcomplex-100.tar.gz obtainebale from:
# http://webserver2.tecgraf.puc-rio.br/~lhf/ftp/lua/index.html#lcomplex 

LUA_TOPDIR= /usr/local
LUA_INCDIR= $(LUA_TOPDIR)/include
LUA_BINDIR= $(LUA_TOPDIR)/bin
    LIBDIR= $(LUA_TOPDIR)/lib/lua/5.3
       LUA= $(LUA_BINDIR)/lua

CC= gcc -std=c99
CFLAGS= -Wall -Wextra -Wfatal-errors -O2
MYCFLAGS= $(CFLAGS) -I$(LUA_INCDIR)

MYNAME= cqr
MYLIBS= -lCQRlib
MYFILE= l$(MYNAME).c
MYMOD= $(MYNAME).$(LIBEXT)

all:	so test

so:
	@$(MAKE) `uname`

test:
	$(LUA) test.lua

install:
	cp $(MYMOD) $(LIBDIR)

clean:
	rm -f *.o *.so

doc:
	@echo "$(MYNAME) library:"
	@fgrep '/**' $(MYFILE) | cut -f2 -d/ | tr -d '* ' | sort | column

Linux:
	$(CC) $(MYCFLAGS) -o $(MYMOD) -shared -fPIC $(MYFILE) $(MYLIBS)

Darwin:
	$(CC) $(MYCFLAGS) -o $(MYMOD) -bundle -undefined dynamic_lookup $(MYFILE) $(MYLIBS)

build:
	$(CC) $(MYCFLAGS) -o $(MYMOD) $(LIBFLAG) $(MYFILE) $(MYLIBS)

LIBFLAG= -shared -fPIC
LIBEXT= so

.PHONY: all so test install clean doc Linux Darwin build
