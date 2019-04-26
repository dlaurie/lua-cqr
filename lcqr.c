#include <cqrlib.h>
#include <stdio.h>
#include "lua.h"
#include "lauxlib.h"
#include "assert.h"
#include <math.h>

#define Quat		CQRQuaternion
#define QH		CQRQuaternionHandle
#define MYNAME		"quaternion"
#define MYTYPE		MYNAME 
#define MYVERSION	MYTYPE " library for " LUA_VERSION ": Apr 2019"

#define qcheck(L,i) ((Quat*)luaL_checkudata(L,i,MYTYPE))
#define nget lua_tonumber
#define ncheck luaL_checknumber

#define NaN  nan("")

// copy/default constructor
Quat * pushquaternion(lua_State *L, Quat *z)
{ Quat *p=lua_newuserdata(L,sizeof(Quat));
  if (z) *p=*z;
  luaL_setmetatable(L,MYTYPE);
  return p;
}

// constructor from components
Quat * pushnewquaternion(lua_State *L, LUA_NUMBER w, LUA_NUMBER x, 
  LUA_NUMBER y, LUA_NUMBER z )
{ Quat *p = pushquaternion(L, NULL);
  p->w=w; p->x=x; p->y=y; p->z=z; 
  return p;
}

void tomatrix(lua_State *L, int t, LUA_NUMBER mat[3][3]) { 
  lua_rawgeti(L,t,1); luaL_checktype(L,-1,LUA_TTABLE);
  lua_rawgeti(L,t,2); luaL_checktype(L,-1,LUA_TTABLE);
  lua_rawgeti(L,t,3); luaL_checktype(L,-1,LUA_TTABLE);
  for (int i=0;i<3;i++) for (int j=0;j<3;j++) {
    lua_rawgeti(L,i-3,j+1); luaL_checktype(L,-1,LUA_TNUMBER);
    mat[i][j] = luaL_checknumber(L,-1); lua_pop(L,1); }
  return;  
}

void pushmatrix(lua_State *L, LUA_NUMBER mat[3][3]) { 
  lua_createtable(L,3,0);
  int t = lua_gettop(L);
  for (int i=0;i<3;i++) {
    lua_createtable(L,3,0); 
    for (int j=0;j<3;j++) {
      lua_pushnumber(L,mat[i][j]); 
      lua_rawseti(L,t+1,j+1); 
    }
    lua_rawseti(L,t,i+1); 
  }
  return;  
}

// Simple bindings

#define QUAT2QUAT(MODNAME,LIBNAME) \
static int MODNAME (lua_State *L) { \
  CQR##LIBNAME(pushquaternion(L,NULL),qcheck(L,1)); \
  return 1; \
}

#define QUAT2NUM(MODNAME,LIBNAME) \
static int MODNAME (lua_State *L) { \
  LUA_NUMBER v=NaN; \
  CQR##LIBNAME(&v,qcheck(L,1)); \
  lua_pushnumber(L,v); \
  return 1; \
}

QUAT2QUAT(axis,GetQuaternionAxis)
QUAT2QUAT(Llog,Log)
QUAT2QUAT(Lexp,Exp)
QUAT2QUAT(Lbnot,Conjugate)              // ~Q means conjugate of Q
QUAT2QUAT(vector,GetQuaternionIm) 
QUAT2NUM(normsq,Normsq)
QUAT2NUM(scalar,GetQuaternionW)
QUAT2NUM(Larg,GetQuaternionAngle)
QUAT2NUM(Llen,Norm)                     // #Q means norm of Q

// Constructor

static int new (lua_State *L) {  
  switch (lua_gettop(L)) {
case 4:                                 // quaternion from components
  pushnewquaternion(L,luaL_checknumber(L,1), 
                      luaL_checknumber(L,2),
                      luaL_checknumber(L,3),
                      luaL_checknumber(L,4) ); 
  break;
case 3:                                 // rotator from Euler angles
   CQRAngles2Quaternion(pushquaternion(L,NULL),
     ncheck(L,1),ncheck(L,2),ncheck(L,3));
   break;
case 1:   
   if (lua_type(L,1)==LUA_TNUMBER)      // scalar quaternion
     pushnewquaternion(L,nget(L,1),0,0,0);
   else if (lua_type(L,1)==LUA_TTABLE) { // rotator from 3x3 matrix
     LUA_NUMBER mat[3][3];
     tomatrix(L,1,mat);
     CQRMatrix2Quaternion(pushquaternion(L,NULL),mat);
     break;
   }
   else                                // copy constructor
     pushquaternion(L,qcheck(L,1));
   break;
default:                                // zero quaternion
   pushnewquaternion(L,0,0,0,0);
  }
  return 1;
}

// One-off bindings
// add, sub, mul, div allow one argument to be a number

static int Leq (lua_State *L) {
  lua_pushboolean(L,!CQREqual(qcheck(L,1),qcheck(L,2)));
  return 1;
}

static int Ladd (lua_State *L) { 
  int m=1, n=2; 
  if (lua_type(L,1)==LUA_TNUMBER) m=2, n=1; 
  if (lua_type(L,n)==LUA_TNUMBER) { 
    Quat *r = pushquaternion(L,qcheck(L,m)); 
    r->w = r->w + nget(L,n); } 
  else 
  CQRAdd(pushquaternion(L,NULL),qcheck(L,1),qcheck(L,2)); 
  return 1; 
}

static int Lsub (lua_State *L) { 
  if (lua_type(L,2)==LUA_TNUMBER) {
    Quat *r = pushquaternion(L,qcheck(L,1)); 
    r->w = r->w - nget(L,2); } 
  else if (lua_type(L,1)==LUA_TNUMBER) { 
    Quat *r = pushquaternion(L,NULL);
    CQRConjugate(r,qcheck(L,2));
    r->w = nget(L,1) - r->w; } 
  else 
  CQRSubtract(pushquaternion(L,NULL),qcheck(L,1),qcheck(L,2)); 
  return 1; 
}

static int Lunm (lua_State *L) {
  Quat *r = pushquaternion(L,NULL);
  CQRConjugate(r,qcheck(L,2));
  r->w = - r->w; 
  return 1;
}

static int Lmul (lua_State *L) { 
  int m=1, n=2; 
  if (lua_type(L,1)==LUA_TNUMBER) m=2, n=1; 
  if (lua_type(L,n)==LUA_TNUMBER) 
    CQRScalarMultiply(pushquaternion(L,NULL),qcheck(L,m),nget(L,n));
  else
    CQRMultiply(pushquaternion(L,NULL),qcheck(L,1),qcheck(L,2));
  return 1;
}

static int Ldiv (lua_State *L) { 
  Quat *r = pushquaternion(L,NULL);
  if (lua_type(L,2)==LUA_TNUMBER) 
    CQRScalarMultiply(r,qcheck(L,1),1/nget(L,2));
  else if (lua_type(L,1)==LUA_TNUMBER) {
    CQRInverse(r,qcheck(L,2));
    CQRScalarMultiply(r,r,nget(L,1));}
  else
    CQRDivide(r,qcheck(L,1),qcheck(L,2));
  return 1;
}

//  Q//U means rotate Q by the rotator U. It is not tested whether U is in 
// fact a unit quaternion.
static int Lidiv (lua_State *L) { 
  Quat * v = qcheck(L,1);
  CQRRotateByQuaternion(&(pushquaternion(L,v)->x),qcheck(L,2),&(v->x));
  return 1;
}

// V..W means the L⁴ dot product of V and W
static int Lconcat (lua_State *L) {
  LUA_NUMBER a=NaN;
  CQRDot(&a,qcheck(L,1),qcheck(L,2));
  lua_pushnumber(L,a);
  return 1;
}

static int root (lua_State *L) {
  CQRIntegerRoot(pushquaternion(L,NULL),qcheck(L,1),ncheck(L,2),ncheck(L,3));
  return 1;
}

// V>>theta means the rotator that rotates around the axis V by angle theta.
static int Lshr (lua_State *L) { 
  Quat * v = qcheck(L,1);
  CQRAxis2Quaternion(pushquaternion(L,NULL),&(v->x),ncheck(L,2));
  return 1;
}
 
static int Lpow (lua_State *L) { 
  Quat *r = pushquaternion(L,NULL);  
  if (lua_isinteger(L,2)) CQRIntegerPower(r,qcheck(L,1),lua_tointeger(L,2));
  else if (lua_isnumber(L,2)) CQRDoublePower(r,qcheck(L,1),lua_tonumber(L,2));
  else CQRQuaternionPower(r,qcheck(L,1),qcheck(L,2));
  return 1;
}    
  
static int angles(lua_State *L) {
  LUA_NUMBER rotx=NaN,roty=NaN,rotz=NaN;
  CQRQuaternion2Angles(&rotx,&roty,&rotz, qcheck(L,1));
  lua_pushnumber(L,rotx); lua_pushnumber(L,roty); lua_pushnumber(L,rotz);
  return 3;
}

static int slerp(lua_State *L) {
  CQRSLERP (pushquaternion(L,NULL),qcheck(L,1),qcheck(L,2),ncheck(L,3),
    ncheck(L,4));
  return 1;
}

static int hlerp(lua_State *L) {
  CQRHLERP (pushquaternion(L,NULL),qcheck(L,1),qcheck(L,2),ncheck(L,3),
    ncheck(L,4));
  return 1;
}

static int slerpdist(lua_State *L) {
  LUA_NUMBER d=NaN;
  CQRSLERPDist (&d,qcheck(L,1),qcheck(L,2));
  lua_pushnumber(L,d);
  return 1;
}

static int hlerpdist(lua_State *L) {
  LUA_NUMBER d=NaN;
  CQRHLERPDist (&d,qcheck(L,1),qcheck(L,2));
  lua_pushnumber(L,d);
  return 1;
}

static int matrix(lua_State *L) {
  LUA_NUMBER R[3][3] = {{NaN,NaN,NaN},{NaN,NaN,NaN},{NaN,NaN,NaN}};
  CQRQuaternion2Matrix(R,qcheck(L,1));  
  pushmatrix(L,R);
  return 1;
}

// Additional functionality

static int unpack (lua_State *L) {
  Quat *q = qcheck(L,1);
  lua_pushnumber(L,q->w);
  lua_pushnumber(L,q->x);
  lua_pushnumber(L,q->y);
  lua_pushnumber(L,q->z);
  return 4;
}

static int Lsqrt (lua_State *L) {
  LUA_NUMBER a=NaN, b=NaN;
  Quat *p = qcheck(L,1);
  CQRNorm(&a,p);                //  a = #p
  a = sqrt(a);
  Quat *w = pushquaternion(L,NULL);        
  CQRScalarMultiply(w,p,1/a);
  w->w = w->w + a;              //  w = a+p/a
  CQRNorm(&b,w);
  if (b==0) return 0;
  CQRScalarMultiply(w,w,a/b);   //  w*(a/#w) 
  return 1;
}

#define IMPART(X,I) \
if ((X!=X || X>0) && lua_gettop(L)>0) lua_pushliteral(L,"+"); \
if (X!=X) { \
  lua_pushliteral(L,"NaN"); \
  lua_pushliteral(L,#I); \
} \
else { \
  if (X<0) { \
    lua_pushliteral(L,"-"); \
    X = -X; \
  } \
  if (X>0) { \
    if (X!=1) lua_pushnumber(L,X); \
    lua_pushliteral(L,#I); \
  } \
} 
  
static int Ltostring(lua_State *L) 
{ Quat *q = qcheck(L,1);
  LUA_NUMBER w = q->w, x = q->x, y = q->y, z = q->z;
  lua_settop(L,0);
  if (w!=w) lua_pushliteral(L,"NaN");
  else if (w!=0) lua_pushnumber(L,w);
  IMPART(x,i) 
  IMPART(y,j)
  IMPART(z,k)
  if (!lua_gettop(L)) lua_pushnumber(L,0.0);
  lua_concat(L,lua_gettop(L));
 return 1;
}

static int Lbor(lua_State *L) 
{ Quat *p = qcheck(L,1), *q = qcheck(L,2);
  if (p->w != 0) luaL_error(L,
"bad argument #1 to __lbor, expected scalar part 0, got %s",p->w);
  if (q->w != 0) luaL_error(L,
"bad argument #2 to __lbor, expected scalar part 0, got %s",q->w);
  Quat *r = pushquaternion(L,NULL); 
  LUA_NUMBER t;
  CQRDivide(r,p,q);            // a = p/q
  CQRGetQuaternionAngle(&t,r);
  CQRGetQuaternionIm(r,r);
  CQRAxis2Quaternion(r,&r->x,t);   // r = a:vector() >> a:arg()
  return 1; 
}


// Metatable 
#define METHOD(NAME) {#NAME, NAME},
#define LMETHOD(NAME) {#NAME, L##NAME},
#define META(NAME) {"__"#NAME, L##NAME},
static const luaL_Reg R[] =
{ METHOD(new)
  METHOD(unpack)
  METHOD(vector)
  METHOD(scalar)
  METHOD(normsq)
  METHOD(axis)
  METHOD(angles)
  METHOD(matrix)
  METHOD(root)
  METHOD(slerp)
  METHOD(hlerp)
  METHOD(slerpdist)
  METHOD(hlerpdist)
  LMETHOD(exp)
  LMETHOD(log)
  LMETHOD(arg)
  LMETHOD(sqrt)
  META(eq)
  META(len)
  META(sub)
  META(add)
  META(unm)
  META(mul)
  META(div)
  META(pow)
  META(idiv)
  META(bnot)
  META(bor)
  META(shr)
  META(concat)
  META(tostring)
  {NULL, NULL} 
};

#define Lbor \
    local p, q = ... \
    local a = (p/q):vector()      -- axis of rotation \
    return a >> acos((p..q)/#p/#q)

LUALIB_API int luaopen_cqr(lua_State *L)
{
 luaL_newmetatable(L,MYTYPE);
 luaL_setfuncs(L,R,0);
 lua_pushliteral(L,"_VERSION");			/** version */
 lua_pushliteral(L,MYVERSION);
 lua_settable(L,-3);
 lua_pushliteral(L,"__index");
 lua_pushvalue(L,-2);
 lua_settable(L,-3);
 return 1;
}

/*    Description

   The cqrlib.h header file defines the CQRQuaternionHandle type as a pointer
   to a struct of the CQRQuaternion type:

     typedef struct {
         double w;
         double x;
         double y;
         double z; } CQRQuaternion;

   representing w + xi +yj + zk. 
   
   CQRCreateQuaternion          Dynamic memory is handled by Lua
   CQRCreateEmptyQuaternion             "
   CQRFreeQuaternion                    "
   CQRPoint2Quaternion          Points are represented as quaternions
   CQRInverse                   Use 1/Q         
   CQRGetQuaternionX            Use Q:unpack()
   CQRGetQuaternionY                    "
   CQRGetQuaternionZ                    "
   CQRQGetQuaternion            Does not exist in CGRlib
   CQRDistsq                    Use (P-Q):normsq()
   CQRDist                      Use #(P-Q)

   CQRMatrix2Quaterion forms the quaternion equivalent a 3x3
   rotation matrix R. CQRQuaternion2Matrix forms a 3x3 rotation matrix from a
   quaternion. 

   The SLERP and HLERP functions combine quaternions by speherical linear
   interpolation. SLERP take two quaternions and two weights and combine them
   following a great circle on the unit quaternion 4-D sphere and linear
   interpolation between the radii. SLERP keeps a quaternion separate from
   the negative of the same quaternion and is not appropriate for quaternions
   representing rotations. Use HLERP to apply SLERP to quaternions
   representing rotations.

    Returns

   The CQRlib functions return 0 for normal completion, or the sum of one or
   more of the following non-zero error codes:

     Error Return     Numeric Value    Meaning                                
     CQR_BAD_ARGUMENT    1              An argument is not valid          
     CQR_NO_MEMORY       2              A call to allocate memory failed 
     CQR_FAILED          4              Operation failed    
*/

