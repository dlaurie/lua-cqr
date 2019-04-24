# lua-cqr
Extended idiomatic Lua interface to the CQRlib quaternion library.

This module provides an extended idiomatic Lua interface to the [`CQRlib` 
library](https://sourceforge.net/projects/cqrlib) written by 
Herbert J. Bernstein. It is not a "binding" and access to the original
documentation is only required for those who wish to understand the 
module's C code in `cqr.c`.

A quaternion is implemented as an immutable userdata consisting of four 
double precision numbers. The metatable of that module, which is its own 
`__index`, is returned by `require "cqr"`.

Full documentation is provided in `lua-cqr.html`.

