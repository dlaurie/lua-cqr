-- test Lua cqr library

------------------------------------------------------------------------------
package.path = ''
ok, complex = pcall(require,"complex")
package.cpath = "./?.so"
H=require"cqr"   -- class of quaternions is named 'H' for Hamilton
print(H._VERSION,'\n')

-- generic constructor
local construct = function(class,...) return class.new(...) end
setmetatable(H,{__call = construct})

-- math functions globally visible
setmetatable(_ENV,{__index=math})

-- extend H by adding an 'eval' function if the complex library is standard
if complex and complex.new and complex.real and complex.imag then
  H.eval = function(q,f)
    local fz = complex[f]
    if type(fz) ~= "function"  then
      error("Attempt to evaluate quaternion function '"..f..
            "'\n  but no function 'complex."..f.."' is available")
    end
    local fq = fz(complex.new(q:scalar(),#q:vector()))
    return fq:real() + fq:imag()*q:axis()
  end
end

failed, passed = {},{new=0, _VERSION=H._VERSION and 0}
tnum = 0
if H.__name == "quaternion" then passed.__name=tnum else failed.__name=tnum end
if H.__index == H then passed.__index=tnum else failed.__index=tnum end

local is = function(object,class)
  return getmetatable(object)==class
end

-- Display multiplication table

local centerformat = function(s,n)
  s = tostring(s)
  local m = (n-#s)//2
  local k = n-#s-m
  return (' '):rep(k) ..s.. (' '):rep(m)
end

local multab = {}
for m=1,4 do 
  local mult = {}
  for n=1,4 do
    local a, b = {0,0,0,0}, {0,0,0,0}
    a[m], b[n] = 1, 1
    a,b = H(table.unpack(a)),  H(table.unpack(b))
    mult[n] = centerformat(a*b,6)
  end
  multab[m] = table.concat(mult)
end

multab = table.concat(multab,"\n")
print"  Multiplication table\n"
print(multab); print()

correct_multab = [[
  1.0    i     j     k  
   i   -1.0    k    -j  
   j    -k   -1.0    i  
   k     j    -i   -1.0 ]] 
if multab == correct_multab then
  passed.__mul, passed.unpack, passed.__tostring = 0,0
else
  print"Multiplication table prints wrong, should be:\n"
  print(correct_multab)
end

local TOL = 1e-12
local function eq(x,y,tol)
  return abs(x-y)<=(tol or TOL)
end

local function test(q,msg,...)
  local w,x,y,z = q:unpack()
  local W,X,Y,Z = ...
  if is(W,H) then W,X,Y,Z = q:unpack() end
  local result
  if not (eq(w,W) and eq(x,X) and eq(y,Y) and eq(z,Z)) then
    print(msg..(": wrong result; expected %s, got %s"):format(H(...),q))
    result = failed
  else
    result = passed
  end
  tnum = tnum+1
  for name in msg:gmatch"[_%a]+" do result[name] = tnum end
end 

local function testreal(x,msg,y)
  local result
  if not eq(x,y) then
    print(msg..(": wrong result; expected %s, got %s"):format(y,x))
    result = failed
  else
    result = passed
  end
  for name in msg:gmatch"[_%a]+" do result[name] = tnum end
end 

p = H(3,5,7,11)

if tostring(p)=="3.0+5.0i+7.0j+11.0k" then 
  passed.__tostring=0 else 
  failed.__tostring=0 
end

v = p:vector(); vv = H(0,5,7,11)
if v==vv then t=passed else t=failed end
  tnum = tnum+1; t.__eq, t.vector = tnum,tnum
testreal(#p,"__len",sqrt(204))
test(p*~p,"__mul,__bnot,normsq",H(204));
testreal(p..p,"__concat",204)

-- test specific functions using random arguments
p = H(random(),random(),random(),random())
q = H(random(),random(),random(),random())
print"    Specific tests"
a = (q+5):log()
testreal(cos(p:arg())*#p,"arg,scalar,__add",p:scalar())
if a.eval then
  test(a:exp(),"exp,eval",a:eval"exp")
  test(a:log(),"log",a:eval"log")
  test(a:sqrt(),"sqrt",a:eval"sqrt")
else
  test(a:exp():log(),"exp,log",a)
  test(a:log():exp(),"log,exp",a)
  test(a:sqrt()^2,"sqrt",a)
end
test(q-p,"__sub,__unm",-(p-q))
w = v >> pi/6
test(w,"__shr,matrix",H(w:matrix()))
a,b = p:axis(), a:axis()
testreal(#a,'axis',1)
test(a,"angles",H(a:angles()))
testreal(H.slerpdist(a,b),"slerpdist,arg,__div",(a/b):arg())  
test(p:root(5,3)^5,"root,__pow",p)
testreal(H.hlerpdist(a,-b),"hlerpdist",H.slerpdist(a,b))
test(H.hlerp(-p,q,8,5), "hlerp",H.slerp(p,q,8,5))
-- rotation, slerp tests are not random, though
w = H(0,0,0,1) >> pi/6
test(w*H(0,1,0,0)*~w,"__idiv",H(0,cos(pi/6),sin(pi/6),0))
test(H.slerp(H(0,1,0,0),H(0,0,1,0),0.7,0.3), "slerp",
   H(0,cos(pi*0.3),sin(pi*0,3),0))
u,v = p:vector(), q:vector()
test(u//(v|u), "__bor", (#u/#v)*v)
print""

any = false
for k,v in pairs(H) do if failed[k] then
  print(k.." FAILED in test #"..failed[k])
  any=true
end end
if any then print"" else print"No test failed.\n"
end

any = false
for k,v in pairs(H) do if passed[k] and not failed[k] then
  print(k.." passed in test #"..passed[k])
  any=true
end end
if any then print"" end

any = false
for k,v in pairs(H) do if not (passed[k] or failed[k]) then
  print(k.." was not tested")
  any=true
end end
if any then print"" end

------------------------------------------------------------------------------
