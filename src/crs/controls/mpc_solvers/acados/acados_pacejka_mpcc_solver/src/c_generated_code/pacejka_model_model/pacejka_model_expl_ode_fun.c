/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) pacejka_model_expl_ode_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[13] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[24] = {20, 1, 0, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

/* pacejka_model_expl_ode_fun:(i0[9],i1[3],i2[20])->(o0[9]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11;
  /* #0: @0 = input[0][3] */
  w0 = arg[0] ? arg[0][3] : 0;
  /* #1: @1 = input[0][2] */
  w1 = arg[0] ? arg[0][2] : 0;
  /* #2: @2 = cos(@1) */
  w2 = cos( w1 );
  /* #3: @2 = (@0*@2) */
  w2  = (w0*w2);
  /* #4: @3 = input[0][4] */
  w3 = arg[0] ? arg[0][4] : 0;
  /* #5: @4 = sin(@1) */
  w4 = sin( w1 );
  /* #6: @4 = (@3*@4) */
  w4  = (w3*w4);
  /* #7: @2 = (@2-@4) */
  w2 -= w4;
  /* #8: output[0][0] = @2 */
  if (res[0]) res[0][0] = w2;
  /* #9: @2 = sin(@1) */
  w2 = sin( w1 );
  /* #10: @2 = (@0*@2) */
  w2  = (w0*w2);
  /* #11: @1 = cos(@1) */
  w1 = cos( w1 );
  /* #12: @1 = (@3*@1) */
  w1  = (w3*w1);
  /* #13: @2 = (@2+@1) */
  w2 += w1;
  /* #14: output[0][1] = @2 */
  if (res[0]) res[0][1] = w2;
  /* #15: @2 = input[0][5] */
  w2 = arg[0] ? arg[0][5] : 0;
  /* #16: output[0][2] = @2 */
  if (res[0]) res[0][2] = w2;
  /* #17: @1 = input[2][16] */
  w1 = arg[2] ? arg[2][16] : 0;
  /* #18: @4 = input[2][17] */
  w4 = arg[2] ? arg[2][17] : 0;
  /* #19: @4 = (@4*@0) */
  w4 *= w0;
  /* #20: @1 = (@1-@4) */
  w1 -= w4;
  /* #21: @4 = input[0][6] */
  w4 = arg[0] ? arg[0][6] : 0;
  /* #22: @1 = (@1*@4) */
  w1 *= w4;
  /* #23: @4 = input[2][18] */
  w4 = arg[2] ? arg[2][18] : 0;
  /* #24: @4 = (@4*@0) */
  w4 *= w0;
  /* #25: @4 = (@4*@0) */
  w4 *= w0;
  /* #26: @1 = (@1-@4) */
  w1 -= w4;
  /* #27: @4 = input[2][19] */
  w4 = arg[2] ? arg[2][19] : 0;
  /* #28: @1 = (@1-@4) */
  w1 -= w4;
  /* #29: @4 = input[2][10] */
  w4 = arg[2] ? arg[2][10] : 0;
  /* #30: @5 = input[2][11] */
  w5 = arg[2] ? arg[2][11] : 0;
  /* #31: @6 = input[2][12] */
  w6 = arg[2] ? arg[2][12] : 0;
  /* #32: @7 = input[0][7] */
  w7 = arg[0] ? arg[0][7] : 0;
  /* #33: @8 = (@3/@0) */
  w8  = (w3/w0);
  /* #34: @8 = atan(@8) */
  w8 = atan( w8 );
  /* #35: @9 = (@7-@8) */
  w9  = (w7-w8);
  /* #36: @10 = input[2][7] */
  w10 = arg[2] ? arg[2][7] : 0;
  /* #37: @11 = (@10*@2) */
  w11  = (w10*w2);
  /* #38: @11 = (@11/@0) */
  w11 /= w0;
  /* #39: @9 = (@9-@11) */
  w9 -= w11;
  /* #40: @6 = (@6*@9) */
  w6 *= w9;
  /* #41: @6 = atan(@6) */
  w6 = atan( w6 );
  /* #42: @5 = (@5*@6) */
  w5 *= w6;
  /* #43: @5 = sin(@5) */
  w5 = sin( w5 );
  /* #44: @4 = (@4*@5) */
  w4 *= w5;
  /* #45: @5 = sin(@7) */
  w5 = sin( w7 );
  /* #46: @5 = (@4*@5) */
  w5  = (w4*w5);
  /* #47: @1 = (@1-@5) */
  w1 -= w5;
  /* #48: @5 = input[2][8] */
  w5 = arg[2] ? arg[2][8] : 0;
  /* #49: @3 = (@5*@3) */
  w3  = (w5*w3);
  /* #50: @3 = (@3*@2) */
  w3 *= w2;
  /* #51: @1 = (@1+@3) */
  w1 += w3;
  /* #52: @1 = (@1/@5) */
  w1 /= w5;
  /* #53: output[0][3] = @1 */
  if (res[0]) res[0][3] = w1;
  /* #54: @1 = input[2][13] */
  w1 = arg[2] ? arg[2][13] : 0;
  /* #55: @3 = input[2][14] */
  w3 = arg[2] ? arg[2][14] : 0;
  /* #56: @6 = input[2][15] */
  w6 = arg[2] ? arg[2][15] : 0;
  /* #57: @9 = input[2][6] */
  w9 = arg[2] ? arg[2][6] : 0;
  /* #58: @11 = (@9*@2) */
  w11  = (w9*w2);
  /* #59: @11 = (@11/@0) */
  w11 /= w0;
  /* #60: @11 = (@11-@8) */
  w11 -= w8;
  /* #61: @6 = (@6*@11) */
  w6 *= w11;
  /* #62: @6 = atan(@6) */
  w6 = atan( w6 );
  /* #63: @3 = (@3*@6) */
  w3 *= w6;
  /* #64: @3 = sin(@3) */
  w3 = sin( w3 );
  /* #65: @1 = (@1*@3) */
  w1 *= w3;
  /* #66: @3 = cos(@7) */
  w3 = cos( w7 );
  /* #67: @3 = (@4*@3) */
  w3  = (w4*w3);
  /* #68: @3 = (@1+@3) */
  w3  = (w1+w3);
  /* #69: @0 = (@5*@0) */
  w0  = (w5*w0);
  /* #70: @0 = (@0*@2) */
  w0 *= w2;
  /* #71: @3 = (@3-@0) */
  w3 -= w0;
  /* #72: @3 = (@3/@5) */
  w3 /= w5;
  /* #73: output[0][4] = @3 */
  if (res[0]) res[0][4] = w3;
  /* #74: @4 = (@4*@10) */
  w4 *= w10;
  /* #75: @7 = cos(@7) */
  w7 = cos( w7 );
  /* #76: @4 = (@4*@7) */
  w4 *= w7;
  /* #77: @1 = (@1*@9) */
  w1 *= w9;
  /* #78: @4 = (@4-@1) */
  w4 -= w1;
  /* #79: @1 = input[2][9] */
  w1 = arg[2] ? arg[2][9] : 0;
  /* #80: @4 = (@4/@1) */
  w4 /= w1;
  /* #81: output[0][5] = @4 */
  if (res[0]) res[0][5] = w4;
  /* #82: @4 = input[1][0] */
  w4 = arg[1] ? arg[1][0] : 0;
  /* #83: output[0][6] = @4 */
  if (res[0]) res[0][6] = w4;
  /* #84: @4 = input[1][1] */
  w4 = arg[1] ? arg[1][1] : 0;
  /* #85: output[0][7] = @4 */
  if (res[0]) res[0][7] = w4;
  /* #86: @4 = input[1][2] */
  w4 = arg[1] ? arg[1][2] : 0;
  /* #87: output[0][8] = @4 */
  if (res[0]) res[0][8] = w4;
  return 0;
}

CASADI_SYMBOL_EXPORT int pacejka_model_expl_ode_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int pacejka_model_expl_ode_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int pacejka_model_expl_ode_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void pacejka_model_expl_ode_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int pacejka_model_expl_ode_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void pacejka_model_expl_ode_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void pacejka_model_expl_ode_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void pacejka_model_expl_ode_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int pacejka_model_expl_ode_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int pacejka_model_expl_ode_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real pacejka_model_expl_ode_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* pacejka_model_expl_ode_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* pacejka_model_expl_ode_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* pacejka_model_expl_ode_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* pacejka_model_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int pacejka_model_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 12;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
