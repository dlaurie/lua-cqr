# makefile for quaternion library for Lua
# adapted from LHF's complex library

# change these to reflect your Lua installation
#LUA= 
#LUAINC= 
#LUALIB= 
#LUABIN= 

# these will probably work if Lua has been installed globally
LUA= /usr/local
LUAINC= $(LUA)/include
LUALIB= $(LUA)/lib
LUABIN= $(LUA)/bin

# probably no need to change anything below here
CC= gcc
CFLAGS= -std=c99 -fPIC $(INCS) $(WARN) -O2 $G 
WARN= -pedantic -Wall -Wextra
INCS= -I$(LUAINC)
MAKESO= $(CC) -shared
#MAKESO= $(CC) -bundle -undefined dynamic_lookup

MYNAME= cqr
BINDSTO= -lCQRlib
MYLIB= l$(MYNAME)
T= $(MYNAME).so 
OBJS= $(MYLIB).o
TEST= test.lua

all:	test

test:	$T
	$(LUABIN)/lua $(TEST)

o:	$(MYLIB).o

so:	$T

$T:	$(OBJS)
	$(MAKESO) -o $@ $(OBJS) $(BINDSTO)

clean:
	rm -f $(OBJS) $T core core.*

doc:
	@echo "$(MYNAME) library:"
	@fgrep '/**' $(MYLIB).c | cut -f2 -d/ | tr -d '*' | sort | column

# eof
