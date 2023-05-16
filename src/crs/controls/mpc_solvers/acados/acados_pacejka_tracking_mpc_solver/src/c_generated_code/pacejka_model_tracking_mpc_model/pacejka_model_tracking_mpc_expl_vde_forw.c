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
  #define CASADI_PREFIX(ID) pacejka_model_tracking_mpc_expl_vde_forw_ ## ID
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
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_sq CASADI_PREFIX(sq)

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

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s1[75] = {8, 8, 0, 8, 16, 24, 32, 40, 48, 56, 64, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s2[21] = {8, 2, 0, 8, 16, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s3[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s4[24] = {20, 1, 0, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

/* pacejka_model_tracking_mpc_expl_vde_forw:(i0[8],i1[8x8],i2[8x2],i3[2],i4[20])->(o0[8],o1[8x8],o2[8x2]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a7, a8, a9;
  a0=arg[0]? arg[0][3] : 0;
  a1=arg[0]? arg[0][2] : 0;
  a2=cos(a1);
  a3=(a0*a2);
  a4=arg[0]? arg[0][4] : 0;
  a5=sin(a1);
  a6=(a4*a5);
  a3=(a3-a6);
  if (res[0]!=0) res[0][0]=a3;
  a3=sin(a1);
  a6=(a0*a3);
  a7=cos(a1);
  a8=(a4*a7);
  a6=(a6+a8);
  if (res[0]!=0) res[0][1]=a6;
  a6=arg[0]? arg[0][5] : 0;
  if (res[0]!=0) res[0][2]=a6;
  a8=arg[4]? arg[4][16] : 0;
  a9=arg[4]? arg[4][17] : 0;
  a10=(a9*a0);
  a8=(a8-a10);
  a10=arg[0]? arg[0][6] : 0;
  a11=(a8*a10);
  a12=arg[4]? arg[4][18] : 0;
  a13=(a12*a0);
  a14=(a13*a0);
  a11=(a11-a14);
  a14=arg[4]? arg[4][19] : 0;
  a11=(a11-a14);
  a14=arg[4]? arg[4][10] : 0;
  a15=arg[4]? arg[4][11] : 0;
  a16=arg[4]? arg[4][12] : 0;
  a17=arg[0]? arg[0][7] : 0;
  a18=(a4/a0);
  a19=atan(a18);
  a20=(a17-a19);
  a21=arg[4]? arg[4][7] : 0;
  a22=(a21*a6);
  a22=(a22/a0);
  a20=(a20-a22);
  a20=(a16*a20);
  a23=atan(a20);
  a23=(a15*a23);
  a24=sin(a23);
  a24=(a14*a24);
  a25=sin(a17);
  a26=(a24*a25);
  a11=(a11-a26);
  a26=arg[4]? arg[4][8] : 0;
  a27=(a26*a4);
  a28=(a27*a6);
  a11=(a11+a28);
  a11=(a11/a26);
  if (res[0]!=0) res[0][3]=a11;
  a11=arg[4]? arg[4][13] : 0;
  a28=arg[4]? arg[4][14] : 0;
  a29=arg[4]? arg[4][15] : 0;
  a30=arg[4]? arg[4][6] : 0;
  a31=(a30*a6);
  a31=(a31/a0);
  a19=(a31-a19);
  a19=(a29*a19);
  a32=atan(a19);
  a32=(a28*a32);
  a33=sin(a32);
  a33=(a11*a33);
  a34=cos(a17);
  a35=(a24*a34);
  a35=(a33+a35);
  a36=(a26*a0);
  a37=(a36*a6);
  a35=(a35-a37);
  a35=(a35/a26);
  if (res[0]!=0) res[0][4]=a35;
  a35=(a24*a21);
  a37=cos(a17);
  a38=(a35*a37);
  a33=(a33*a30);
  a38=(a38-a33);
  a33=arg[4]? arg[4][9] : 0;
  a38=(a38/a33);
  if (res[0]!=0) res[0][5]=a38;
  a38=arg[3]? arg[3][0] : 0;
  if (res[0]!=0) res[0][6]=a38;
  a38=arg[3]? arg[3][1] : 0;
  if (res[0]!=0) res[0][7]=a38;
  a38=arg[1]? arg[1][3] : 0;
  a39=(a2*a38);
  a40=sin(a1);
  a41=arg[1]? arg[1][2] : 0;
  a42=(a40*a41);
  a42=(a0*a42);
  a39=(a39-a42);
  a42=arg[1]? arg[1][4] : 0;
  a43=(a5*a42);
  a44=cos(a1);
  a45=(a44*a41);
  a45=(a4*a45);
  a43=(a43+a45);
  a39=(a39-a43);
  if (res[1]!=0) res[1][0]=a39;
  a39=(a3*a38);
  a43=cos(a1);
  a45=(a43*a41);
  a45=(a0*a45);
  a39=(a39+a45);
  a45=(a7*a42);
  a46=sin(a1);
  a41=(a46*a41);
  a41=(a4*a41);
  a45=(a45-a41);
  a39=(a39+a45);
  if (res[1]!=0) res[1][1]=a39;
  a39=arg[1]? arg[1][5] : 0;
  if (res[1]!=0) res[1][2]=a39;
  a45=arg[1]? arg[1][6] : 0;
  a45=(a8*a45);
  a41=(a9*a38);
  a41=(a10*a41);
  a45=(a45-a41);
  a41=(a12*a38);
  a41=(a0*a41);
  a47=(a13*a38);
  a41=(a41+a47);
  a45=(a45-a41);
  a41=cos(a23);
  a47=arg[1]? arg[1][7] : 0;
  a48=(a42/a0);
  a49=(a18/a0);
  a50=(a49*a38);
  a48=(a48-a50);
  a50=1.;
  a51=casadi_sq(a18);
  a51=(a50+a51);
  a48=(a48/a51);
  a52=(a47-a48);
  a53=(a21*a39);
  a53=(a53/a0);
  a54=(a22/a0);
  a55=(a54*a38);
  a53=(a53-a55);
  a52=(a52-a53);
  a52=(a16*a52);
  a53=casadi_sq(a20);
  a53=(a50+a53);
  a52=(a52/a53);
  a52=(a15*a52);
  a52=(a41*a52);
  a52=(a14*a52);
  a55=(a25*a52);
  a56=cos(a17);
  a57=(a56*a47);
  a57=(a24*a57);
  a55=(a55+a57);
  a45=(a45-a55);
  a42=(a26*a42);
  a42=(a6*a42);
  a55=(a27*a39);
  a42=(a42+a55);
  a45=(a45+a42);
  a45=(a45/a26);
  if (res[1]!=0) res[1][3]=a45;
  a45=cos(a32);
  a42=(a30*a39);
  a42=(a42/a0);
  a55=(a31/a0);
  a57=(a55*a38);
  a42=(a42-a57);
  a42=(a42-a48);
  a42=(a29*a42);
  a48=casadi_sq(a19);
  a48=(a50+a48);
  a42=(a42/a48);
  a42=(a28*a42);
  a42=(a45*a42);
  a42=(a11*a42);
  a57=(a34*a52);
  a58=sin(a17);
  a59=(a58*a47);
  a59=(a24*a59);
  a57=(a57-a59);
  a57=(a42+a57);
  a38=(a26*a38);
  a38=(a6*a38);
  a39=(a36*a39);
  a38=(a38+a39);
  a57=(a57-a38);
  a57=(a57/a26);
  if (res[1]!=0) res[1][4]=a57;
  a52=(a21*a52);
  a52=(a37*a52);
  a57=sin(a17);
  a47=(a57*a47);
  a47=(a35*a47);
  a52=(a52-a47);
  a42=(a30*a42);
  a52=(a52-a42);
  a52=(a52/a33);
  if (res[1]!=0) res[1][5]=a52;
  a52=0.;
  if (res[1]!=0) res[1][6]=a52;
  if (res[1]!=0) res[1][7]=a52;
  a42=arg[1]? arg[1][11] : 0;
  a47=(a2*a42);
  a38=arg[1]? arg[1][10] : 0;
  a39=(a40*a38);
  a39=(a0*a39);
  a47=(a47-a39);
  a39=arg[1]? arg[1][12] : 0;
  a59=(a5*a39);
  a60=(a44*a38);
  a60=(a4*a60);
  a59=(a59+a60);
  a47=(a47-a59);
  if (res[1]!=0) res[1][8]=a47;
  a47=(a3*a42);
  a59=(a43*a38);
  a59=(a0*a59);
  a47=(a47+a59);
  a59=(a7*a39);
  a38=(a46*a38);
  a38=(a4*a38);
  a59=(a59-a38);
  a47=(a47+a59);
  if (res[1]!=0) res[1][9]=a47;
  a47=arg[1]? arg[1][13] : 0;
  if (res[1]!=0) res[1][10]=a47;
  a59=arg[1]? arg[1][14] : 0;
  a59=(a8*a59);
  a38=(a9*a42);
  a38=(a10*a38);
  a59=(a59-a38);
  a38=(a12*a42);
  a38=(a0*a38);
  a60=(a13*a42);
  a38=(a38+a60);
  a59=(a59-a38);
  a38=arg[1]? arg[1][15] : 0;
  a60=(a39/a0);
  a61=(a49*a42);
  a60=(a60-a61);
  a60=(a60/a51);
  a61=(a38-a60);
  a62=(a21*a47);
  a62=(a62/a0);
  a63=(a54*a42);
  a62=(a62-a63);
  a61=(a61-a62);
  a61=(a16*a61);
  a61=(a61/a53);
  a61=(a15*a61);
  a61=(a41*a61);
  a61=(a14*a61);
  a62=(a25*a61);
  a63=(a56*a38);
  a63=(a24*a63);
  a62=(a62+a63);
  a59=(a59-a62);
  a39=(a26*a39);
  a39=(a6*a39);
  a62=(a27*a47);
  a39=(a39+a62);
  a59=(a59+a39);
  a59=(a59/a26);
  if (res[1]!=0) res[1][11]=a59;
  a59=(a30*a47);
  a59=(a59/a0);
  a39=(a55*a42);
  a59=(a59-a39);
  a59=(a59-a60);
  a59=(a29*a59);
  a59=(a59/a48);
  a59=(a28*a59);
  a59=(a45*a59);
  a59=(a11*a59);
  a60=(a34*a61);
  a39=(a58*a38);
  a39=(a24*a39);
  a60=(a60-a39);
  a60=(a59+a60);
  a42=(a26*a42);
  a42=(a6*a42);
  a47=(a36*a47);
  a42=(a42+a47);
  a60=(a60-a42);
  a60=(a60/a26);
  if (res[1]!=0) res[1][12]=a60;
  a61=(a21*a61);
  a61=(a37*a61);
  a38=(a57*a38);
  a38=(a35*a38);
  a61=(a61-a38);
  a59=(a30*a59);
  a61=(a61-a59);
  a61=(a61/a33);
  if (res[1]!=0) res[1][13]=a61;
  if (res[1]!=0) res[1][14]=a52;
  if (res[1]!=0) res[1][15]=a52;
  a61=arg[1]? arg[1][19] : 0;
  a59=(a2*a61);
  a38=arg[1]? arg[1][18] : 0;
  a60=(a40*a38);
  a60=(a0*a60);
  a59=(a59-a60);
  a60=arg[1]? arg[1][20] : 0;
  a42=(a5*a60);
  a47=(a44*a38);
  a47=(a4*a47);
  a42=(a42+a47);
  a59=(a59-a42);
  if (res[1]!=0) res[1][16]=a59;
  a59=(a3*a61);
  a42=(a43*a38);
  a42=(a0*a42);
  a59=(a59+a42);
  a42=(a7*a60);
  a38=(a46*a38);
  a38=(a4*a38);
  a42=(a42-a38);
  a59=(a59+a42);
  if (res[1]!=0) res[1][17]=a59;
  a59=arg[1]? arg[1][21] : 0;
  if (res[1]!=0) res[1][18]=a59;
  a42=arg[1]? arg[1][22] : 0;
  a42=(a8*a42);
  a38=(a9*a61);
  a38=(a10*a38);
  a42=(a42-a38);
  a38=(a12*a61);
  a38=(a0*a38);
  a47=(a13*a61);
  a38=(a38+a47);
  a42=(a42-a38);
  a38=arg[1]? arg[1][23] : 0;
  a47=(a60/a0);
  a39=(a49*a61);
  a47=(a47-a39);
  a47=(a47/a51);
  a39=(a38-a47);
  a62=(a21*a59);
  a62=(a62/a0);
  a63=(a54*a61);
  a62=(a62-a63);
  a39=(a39-a62);
  a39=(a16*a39);
  a39=(a39/a53);
  a39=(a15*a39);
  a39=(a41*a39);
  a39=(a14*a39);
  a62=(a25*a39);
  a63=(a56*a38);
  a63=(a24*a63);
  a62=(a62+a63);
  a42=(a42-a62);
  a60=(a26*a60);
  a60=(a6*a60);
  a62=(a27*a59);
  a60=(a60+a62);
  a42=(a42+a60);
  a42=(a42/a26);
  if (res[1]!=0) res[1][19]=a42;
  a42=(a30*a59);
  a42=(a42/a0);
  a60=(a55*a61);
  a42=(a42-a60);
  a42=(a42-a47);
  a42=(a29*a42);
  a42=(a42/a48);
  a42=(a28*a42);
  a42=(a45*a42);
  a42=(a11*a42);
  a47=(a34*a39);
  a60=(a58*a38);
  a60=(a24*a60);
  a47=(a47-a60);
  a47=(a42+a47);
  a61=(a26*a61);
  a61=(a6*a61);
  a59=(a36*a59);
  a61=(a61+a59);
  a47=(a47-a61);
  a47=(a47/a26);
  if (res[1]!=0) res[1][20]=a47;
  a39=(a21*a39);
  a39=(a37*a39);
  a38=(a57*a38);
  a38=(a35*a38);
  a39=(a39-a38);
  a42=(a30*a42);
  a39=(a39-a42);
  a39=(a39/a33);
  if (res[1]!=0) res[1][21]=a39;
  if (res[1]!=0) res[1][22]=a52;
  if (res[1]!=0) res[1][23]=a52;
  a39=arg[1]? arg[1][27] : 0;
  a42=(a2*a39);
  a38=arg[1]? arg[1][26] : 0;
  a47=(a40*a38);
  a47=(a0*a47);
  a42=(a42-a47);
  a47=arg[1]? arg[1][28] : 0;
  a61=(a5*a47);
  a59=(a44*a38);
  a59=(a4*a59);
  a61=(a61+a59);
  a42=(a42-a61);
  if (res[1]!=0) res[1][24]=a42;
  a42=(a3*a39);
  a61=(a43*a38);
  a61=(a0*a61);
  a42=(a42+a61);
  a61=(a7*a47);
  a38=(a46*a38);
  a38=(a4*a38);
  a61=(a61-a38);
  a42=(a42+a61);
  if (res[1]!=0) res[1][25]=a42;
  a42=arg[1]? arg[1][29] : 0;
  if (res[1]!=0) res[1][26]=a42;
  a61=arg[1]? arg[1][30] : 0;
  a61=(a8*a61);
  a38=(a9*a39);
  a38=(a10*a38);
  a61=(a61-a38);
  a38=(a12*a39);
  a38=(a0*a38);
  a59=(a13*a39);
  a38=(a38+a59);
  a61=(a61-a38);
  a38=arg[1]? arg[1][31] : 0;
  a59=(a47/a0);
  a60=(a49*a39);
  a59=(a59-a60);
  a59=(a59/a51);
  a60=(a38-a59);
  a62=(a21*a42);
  a62=(a62/a0);
  a63=(a54*a39);
  a62=(a62-a63);
  a60=(a60-a62);
  a60=(a16*a60);
  a60=(a60/a53);
  a60=(a15*a60);
  a60=(a41*a60);
  a60=(a14*a60);
  a62=(a25*a60);
  a63=(a56*a38);
  a63=(a24*a63);
  a62=(a62+a63);
  a61=(a61-a62);
  a47=(a26*a47);
  a47=(a6*a47);
  a62=(a27*a42);
  a47=(a47+a62);
  a61=(a61+a47);
  a61=(a61/a26);
  if (res[1]!=0) res[1][27]=a61;
  a61=(a30*a42);
  a61=(a61/a0);
  a47=(a55*a39);
  a61=(a61-a47);
  a61=(a61-a59);
  a61=(a29*a61);
  a61=(a61/a48);
  a61=(a28*a61);
  a61=(a45*a61);
  a61=(a11*a61);
  a59=(a34*a60);
  a47=(a58*a38);
  a47=(a24*a47);
  a59=(a59-a47);
  a59=(a61+a59);
  a39=(a26*a39);
  a39=(a6*a39);
  a42=(a36*a42);
  a39=(a39+a42);
  a59=(a59-a39);
  a59=(a59/a26);
  if (res[1]!=0) res[1][28]=a59;
  a60=(a21*a60);
  a60=(a37*a60);
  a38=(a57*a38);
  a38=(a35*a38);
  a60=(a60-a38);
  a61=(a30*a61);
  a60=(a60-a61);
  a60=(a60/a33);
  if (res[1]!=0) res[1][29]=a60;
  if (res[1]!=0) res[1][30]=a52;
  if (res[1]!=0) res[1][31]=a52;
  a60=arg[1]? arg[1][35] : 0;
  a61=(a2*a60);
  a38=arg[1]? arg[1][34] : 0;
  a59=(a40*a38);
  a59=(a0*a59);
  a61=(a61-a59);
  a59=arg[1]? arg[1][36] : 0;
  a39=(a5*a59);
  a42=(a44*a38);
  a42=(a4*a42);
  a39=(a39+a42);
  a61=(a61-a39);
  if (res[1]!=0) res[1][32]=a61;
  a61=(a3*a60);
  a39=(a43*a38);
  a39=(a0*a39);
  a61=(a61+a39);
  a39=(a7*a59);
  a38=(a46*a38);
  a38=(a4*a38);
  a39=(a39-a38);
  a61=(a61+a39);
  if (res[1]!=0) res[1][33]=a61;
  a61=arg[1]? arg[1][37] : 0;
  if (res[1]!=0) res[1][34]=a61;
  a39=arg[1]? arg[1][38] : 0;
  a39=(a8*a39);
  a38=(a9*a60);
  a38=(a10*a38);
  a39=(a39-a38);
  a38=(a12*a60);
  a38=(a0*a38);
  a42=(a13*a60);
  a38=(a38+a42);
  a39=(a39-a38);
  a38=arg[1]? arg[1][39] : 0;
  a42=(a59/a0);
  a47=(a49*a60);
  a42=(a42-a47);
  a42=(a42/a51);
  a47=(a38-a42);
  a62=(a21*a61);
  a62=(a62/a0);
  a63=(a54*a60);
  a62=(a62-a63);
  a47=(a47-a62);
  a47=(a16*a47);
  a47=(a47/a53);
  a47=(a15*a47);
  a47=(a41*a47);
  a47=(a14*a47);
  a62=(a25*a47);
  a63=(a56*a38);
  a63=(a24*a63);
  a62=(a62+a63);
  a39=(a39-a62);
  a59=(a26*a59);
  a59=(a6*a59);
  a62=(a27*a61);
  a59=(a59+a62);
  a39=(a39+a59);
  a39=(a39/a26);
  if (res[1]!=0) res[1][35]=a39;
  a39=(a30*a61);
  a39=(a39/a0);
  a59=(a55*a60);
  a39=(a39-a59);
  a39=(a39-a42);
  a39=(a29*a39);
  a39=(a39/a48);
  a39=(a28*a39);
  a39=(a45*a39);
  a39=(a11*a39);
  a42=(a34*a47);
  a59=(a58*a38);
  a59=(a24*a59);
  a42=(a42-a59);
  a42=(a39+a42);
  a60=(a26*a60);
  a60=(a6*a60);
  a61=(a36*a61);
  a60=(a60+a61);
  a42=(a42-a60);
  a42=(a42/a26);
  if (res[1]!=0) res[1][36]=a42;
  a47=(a21*a47);
  a47=(a37*a47);
  a38=(a57*a38);
  a38=(a35*a38);
  a47=(a47-a38);
  a39=(a30*a39);
  a47=(a47-a39);
  a47=(a47/a33);
  if (res[1]!=0) res[1][37]=a47;
  if (res[1]!=0) res[1][38]=a52;
  if (res[1]!=0) res[1][39]=a52;
  a47=arg[1]? arg[1][43] : 0;
  a39=(a2*a47);
  a38=arg[1]? arg[1][42] : 0;
  a42=(a40*a38);
  a42=(a0*a42);
  a39=(a39-a42);
  a42=arg[1]? arg[1][44] : 0;
  a60=(a5*a42);
  a61=(a44*a38);
  a61=(a4*a61);
  a60=(a60+a61);
  a39=(a39-a60);
  if (res[1]!=0) res[1][40]=a39;
  a39=(a3*a47);
  a60=(a43*a38);
  a60=(a0*a60);
  a39=(a39+a60);
  a60=(a7*a42);
  a38=(a46*a38);
  a38=(a4*a38);
  a60=(a60-a38);
  a39=(a39+a60);
  if (res[1]!=0) res[1][41]=a39;
  a39=arg[1]? arg[1][45] : 0;
  if (res[1]!=0) res[1][42]=a39;
  a60=arg[1]? arg[1][46] : 0;
  a60=(a8*a60);
  a38=(a9*a47);
  a38=(a10*a38);
  a60=(a60-a38);
  a38=(a12*a47);
  a38=(a0*a38);
  a61=(a13*a47);
  a38=(a38+a61);
  a60=(a60-a38);
  a38=arg[1]? arg[1][47] : 0;
  a61=(a42/a0);
  a59=(a49*a47);
  a61=(a61-a59);
  a61=(a61/a51);
  a59=(a38-a61);
  a62=(a21*a39);
  a62=(a62/a0);
  a63=(a54*a47);
  a62=(a62-a63);
  a59=(a59-a62);
  a59=(a16*a59);
  a59=(a59/a53);
  a59=(a15*a59);
  a59=(a41*a59);
  a59=(a14*a59);
  a62=(a25*a59);
  a63=(a56*a38);
  a63=(a24*a63);
  a62=(a62+a63);
  a60=(a60-a62);
  a42=(a26*a42);
  a42=(a6*a42);
  a62=(a27*a39);
  a42=(a42+a62);
  a60=(a60+a42);
  a60=(a60/a26);
  if (res[1]!=0) res[1][43]=a60;
  a60=(a30*a39);
  a60=(a60/a0);
  a42=(a55*a47);
  a60=(a60-a42);
  a60=(a60-a61);
  a60=(a29*a60);
  a60=(a60/a48);
  a60=(a28*a60);
  a60=(a45*a60);
  a60=(a11*a60);
  a61=(a34*a59);
  a42=(a58*a38);
  a42=(a24*a42);
  a61=(a61-a42);
  a61=(a60+a61);
  a47=(a26*a47);
  a47=(a6*a47);
  a39=(a36*a39);
  a47=(a47+a39);
  a61=(a61-a47);
  a61=(a61/a26);
  if (res[1]!=0) res[1][44]=a61;
  a59=(a21*a59);
  a59=(a37*a59);
  a38=(a57*a38);
  a38=(a35*a38);
  a59=(a59-a38);
  a60=(a30*a60);
  a59=(a59-a60);
  a59=(a59/a33);
  if (res[1]!=0) res[1][45]=a59;
  if (res[1]!=0) res[1][46]=a52;
  if (res[1]!=0) res[1][47]=a52;
  a59=arg[1]? arg[1][51] : 0;
  a60=(a2*a59);
  a38=arg[1]? arg[1][50] : 0;
  a61=(a40*a38);
  a61=(a0*a61);
  a60=(a60-a61);
  a61=arg[1]? arg[1][52] : 0;
  a47=(a5*a61);
  a39=(a44*a38);
  a39=(a4*a39);
  a47=(a47+a39);
  a60=(a60-a47);
  if (res[1]!=0) res[1][48]=a60;
  a60=(a3*a59);
  a47=(a43*a38);
  a47=(a0*a47);
  a60=(a60+a47);
  a47=(a7*a61);
  a38=(a46*a38);
  a38=(a4*a38);
  a47=(a47-a38);
  a60=(a60+a47);
  if (res[1]!=0) res[1][49]=a60;
  a60=arg[1]? arg[1][53] : 0;
  if (res[1]!=0) res[1][50]=a60;
  a47=arg[1]? arg[1][54] : 0;
  a47=(a8*a47);
  a38=(a9*a59);
  a38=(a10*a38);
  a47=(a47-a38);
  a38=(a12*a59);
  a38=(a0*a38);
  a39=(a13*a59);
  a38=(a38+a39);
  a47=(a47-a38);
  a38=arg[1]? arg[1][55] : 0;
  a39=(a61/a0);
  a42=(a49*a59);
  a39=(a39-a42);
  a39=(a39/a51);
  a42=(a38-a39);
  a62=(a21*a60);
  a62=(a62/a0);
  a63=(a54*a59);
  a62=(a62-a63);
  a42=(a42-a62);
  a42=(a16*a42);
  a42=(a42/a53);
  a42=(a15*a42);
  a42=(a41*a42);
  a42=(a14*a42);
  a62=(a25*a42);
  a63=(a56*a38);
  a63=(a24*a63);
  a62=(a62+a63);
  a47=(a47-a62);
  a61=(a26*a61);
  a61=(a6*a61);
  a62=(a27*a60);
  a61=(a61+a62);
  a47=(a47+a61);
  a47=(a47/a26);
  if (res[1]!=0) res[1][51]=a47;
  a47=(a30*a60);
  a47=(a47/a0);
  a61=(a55*a59);
  a47=(a47-a61);
  a47=(a47-a39);
  a47=(a29*a47);
  a47=(a47/a48);
  a47=(a28*a47);
  a47=(a45*a47);
  a47=(a11*a47);
  a39=(a34*a42);
  a61=(a58*a38);
  a61=(a24*a61);
  a39=(a39-a61);
  a39=(a47+a39);
  a59=(a26*a59);
  a59=(a6*a59);
  a60=(a36*a60);
  a59=(a59+a60);
  a39=(a39-a59);
  a39=(a39/a26);
  if (res[1]!=0) res[1][52]=a39;
  a42=(a21*a42);
  a42=(a37*a42);
  a38=(a57*a38);
  a38=(a35*a38);
  a42=(a42-a38);
  a47=(a30*a47);
  a42=(a42-a47);
  a42=(a42/a33);
  if (res[1]!=0) res[1][53]=a42;
  if (res[1]!=0) res[1][54]=a52;
  if (res[1]!=0) res[1][55]=a52;
  a42=arg[1]? arg[1][59] : 0;
  a47=(a2*a42);
  a38=arg[1]? arg[1][58] : 0;
  a40=(a40*a38);
  a40=(a0*a40);
  a47=(a47-a40);
  a40=arg[1]? arg[1][60] : 0;
  a39=(a5*a40);
  a44=(a44*a38);
  a44=(a4*a44);
  a39=(a39+a44);
  a47=(a47-a39);
  if (res[1]!=0) res[1][56]=a47;
  a47=(a3*a42);
  a43=(a43*a38);
  a43=(a0*a43);
  a47=(a47+a43);
  a43=(a7*a40);
  a46=(a46*a38);
  a46=(a4*a46);
  a43=(a43-a46);
  a47=(a47+a43);
  if (res[1]!=0) res[1][57]=a47;
  a47=arg[1]? arg[1][61] : 0;
  if (res[1]!=0) res[1][58]=a47;
  a43=arg[1]? arg[1][62] : 0;
  a43=(a8*a43);
  a46=(a9*a42);
  a46=(a10*a46);
  a43=(a43-a46);
  a46=(a12*a42);
  a46=(a0*a46);
  a38=(a13*a42);
  a46=(a46+a38);
  a43=(a43-a46);
  a46=arg[1]? arg[1][63] : 0;
  a38=(a40/a0);
  a49=(a49*a42);
  a38=(a38-a49);
  a38=(a38/a51);
  a51=(a46-a38);
  a49=(a21*a47);
  a49=(a49/a0);
  a54=(a54*a42);
  a49=(a49-a54);
  a51=(a51-a49);
  a51=(a16*a51);
  a51=(a51/a53);
  a51=(a15*a51);
  a41=(a41*a51);
  a41=(a14*a41);
  a51=(a25*a41);
  a56=(a56*a46);
  a56=(a24*a56);
  a51=(a51+a56);
  a43=(a43-a51);
  a40=(a26*a40);
  a40=(a6*a40);
  a51=(a27*a47);
  a40=(a40+a51);
  a43=(a43+a40);
  a43=(a43/a26);
  if (res[1]!=0) res[1][59]=a43;
  a43=(a30*a47);
  a43=(a43/a0);
  a55=(a55*a42);
  a43=(a43-a55);
  a43=(a43-a38);
  a43=(a29*a43);
  a43=(a43/a48);
  a43=(a28*a43);
  a45=(a45*a43);
  a45=(a11*a45);
  a43=(a34*a41);
  a58=(a58*a46);
  a58=(a24*a58);
  a43=(a43-a58);
  a43=(a45+a43);
  a42=(a26*a42);
  a42=(a6*a42);
  a47=(a36*a47);
  a42=(a42+a47);
  a43=(a43-a42);
  a43=(a43/a26);
  if (res[1]!=0) res[1][60]=a43;
  a41=(a21*a41);
  a41=(a37*a41);
  a57=(a57*a46);
  a57=(a35*a57);
  a41=(a41-a57);
  a45=(a30*a45);
  a41=(a41-a45);
  a41=(a41/a33);
  if (res[1]!=0) res[1][61]=a41;
  if (res[1]!=0) res[1][62]=a52;
  if (res[1]!=0) res[1][63]=a52;
  a41=arg[2]? arg[2][3] : 0;
  a45=(a2*a41);
  a57=sin(a1);
  a46=arg[2]? arg[2][2] : 0;
  a43=(a57*a46);
  a43=(a0*a43);
  a45=(a45-a43);
  a43=arg[2]? arg[2][4] : 0;
  a42=(a5*a43);
  a47=cos(a1);
  a58=(a47*a46);
  a58=(a4*a58);
  a42=(a42+a58);
  a45=(a45-a42);
  if (res[2]!=0) res[2][0]=a45;
  a45=(a3*a41);
  a42=cos(a1);
  a58=(a42*a46);
  a58=(a0*a58);
  a45=(a45+a58);
  a58=(a7*a43);
  a1=sin(a1);
  a46=(a1*a46);
  a46=(a4*a46);
  a58=(a58-a46);
  a45=(a45+a58);
  if (res[2]!=0) res[2][1]=a45;
  a45=arg[2]? arg[2][5] : 0;
  if (res[2]!=0) res[2][2]=a45;
  a58=arg[2]? arg[2][6] : 0;
  a58=(a8*a58);
  a46=(a9*a41);
  a46=(a10*a46);
  a58=(a58-a46);
  a46=(a12*a41);
  a46=(a0*a46);
  a48=(a13*a41);
  a46=(a46+a48);
  a58=(a58-a46);
  a23=cos(a23);
  a46=arg[2]? arg[2][7] : 0;
  a48=(a43/a0);
  a38=(a18/a0);
  a55=(a38*a41);
  a48=(a48-a55);
  a18=casadi_sq(a18);
  a18=(a50+a18);
  a48=(a48/a18);
  a55=(a46-a48);
  a40=(a21*a45);
  a40=(a40/a0);
  a22=(a22/a0);
  a51=(a22*a41);
  a40=(a40-a51);
  a55=(a55-a40);
  a55=(a16*a55);
  a20=casadi_sq(a20);
  a20=(a50+a20);
  a55=(a55/a20);
  a55=(a15*a55);
  a55=(a23*a55);
  a55=(a14*a55);
  a40=(a25*a55);
  a51=cos(a17);
  a56=(a51*a46);
  a56=(a24*a56);
  a40=(a40+a56);
  a58=(a58-a40);
  a43=(a26*a43);
  a43=(a6*a43);
  a40=(a27*a45);
  a43=(a43+a40);
  a58=(a58+a43);
  a58=(a58/a26);
  if (res[2]!=0) res[2][3]=a58;
  a32=cos(a32);
  a58=(a30*a45);
  a58=(a58/a0);
  a31=(a31/a0);
  a43=(a31*a41);
  a58=(a58-a43);
  a58=(a58-a48);
  a58=(a29*a58);
  a19=casadi_sq(a19);
  a19=(a50+a19);
  a58=(a58/a19);
  a58=(a28*a58);
  a58=(a32*a58);
  a58=(a11*a58);
  a48=(a34*a55);
  a43=sin(a17);
  a40=(a43*a46);
  a40=(a24*a40);
  a48=(a48-a40);
  a48=(a58+a48);
  a41=(a26*a41);
  a41=(a6*a41);
  a45=(a36*a45);
  a41=(a41+a45);
  a48=(a48-a41);
  a48=(a48/a26);
  if (res[2]!=0) res[2][4]=a48;
  a55=(a21*a55);
  a55=(a37*a55);
  a17=sin(a17);
  a46=(a17*a46);
  a46=(a35*a46);
  a55=(a55-a46);
  a58=(a30*a58);
  a55=(a55-a58);
  a55=(a55/a33);
  if (res[2]!=0) res[2][5]=a55;
  if (res[2]!=0) res[2][6]=a50;
  if (res[2]!=0) res[2][7]=a52;
  a55=arg[2]? arg[2][11] : 0;
  a2=(a2*a55);
  a58=arg[2]? arg[2][10] : 0;
  a57=(a57*a58);
  a57=(a0*a57);
  a2=(a2-a57);
  a57=arg[2]? arg[2][12] : 0;
  a5=(a5*a57);
  a47=(a47*a58);
  a47=(a4*a47);
  a5=(a5+a47);
  a2=(a2-a5);
  if (res[2]!=0) res[2][8]=a2;
  a3=(a3*a55);
  a42=(a42*a58);
  a42=(a0*a42);
  a3=(a3+a42);
  a7=(a7*a57);
  a1=(a1*a58);
  a4=(a4*a1);
  a7=(a7-a4);
  a3=(a3+a7);
  if (res[2]!=0) res[2][9]=a3;
  a3=arg[2]? arg[2][13] : 0;
  if (res[2]!=0) res[2][10]=a3;
  a7=arg[2]? arg[2][14] : 0;
  a8=(a8*a7);
  a9=(a9*a55);
  a10=(a10*a9);
  a8=(a8-a10);
  a12=(a12*a55);
  a12=(a0*a12);
  a13=(a13*a55);
  a12=(a12+a13);
  a8=(a8-a12);
  a12=arg[2]? arg[2][15] : 0;
  a13=(a57/a0);
  a38=(a38*a55);
  a13=(a13-a38);
  a13=(a13/a18);
  a18=(a12-a13);
  a38=(a21*a3);
  a38=(a38/a0);
  a22=(a22*a55);
  a38=(a38-a22);
  a18=(a18-a38);
  a16=(a16*a18);
  a16=(a16/a20);
  a15=(a15*a16);
  a23=(a23*a15);
  a14=(a14*a23);
  a25=(a25*a14);
  a51=(a51*a12);
  a51=(a24*a51);
  a25=(a25+a51);
  a8=(a8-a25);
  a57=(a26*a57);
  a57=(a6*a57);
  a27=(a27*a3);
  a57=(a57+a27);
  a8=(a8+a57);
  a8=(a8/a26);
  if (res[2]!=0) res[2][11]=a8;
  a8=(a30*a3);
  a8=(a8/a0);
  a31=(a31*a55);
  a8=(a8-a31);
  a8=(a8-a13);
  a29=(a29*a8);
  a29=(a29/a19);
  a28=(a28*a29);
  a32=(a32*a28);
  a11=(a11*a32);
  a34=(a34*a14);
  a43=(a43*a12);
  a24=(a24*a43);
  a34=(a34-a24);
  a34=(a11+a34);
  a55=(a26*a55);
  a6=(a6*a55);
  a36=(a36*a3);
  a6=(a6+a36);
  a34=(a34-a6);
  a34=(a34/a26);
  if (res[2]!=0) res[2][12]=a34;
  a21=(a21*a14);
  a37=(a37*a21);
  a17=(a17*a12);
  a35=(a35*a17);
  a37=(a37-a35);
  a30=(a30*a11);
  a37=(a37-a30);
  a37=(a37/a33);
  if (res[2]!=0) res[2][13]=a37;
  if (res[2]!=0) res[2][14]=a52;
  if (res[2]!=0) res[2][15]=a50;
  return 0;
}

CASADI_SYMBOL_EXPORT int pacejka_model_tracking_mpc_expl_vde_forw(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int pacejka_model_tracking_mpc_expl_vde_forw_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int pacejka_model_tracking_mpc_expl_vde_forw_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void pacejka_model_tracking_mpc_expl_vde_forw_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int pacejka_model_tracking_mpc_expl_vde_forw_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void pacejka_model_tracking_mpc_expl_vde_forw_release(int mem) {
}

CASADI_SYMBOL_EXPORT void pacejka_model_tracking_mpc_expl_vde_forw_incref(void) {
}

CASADI_SYMBOL_EXPORT void pacejka_model_tracking_mpc_expl_vde_forw_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int pacejka_model_tracking_mpc_expl_vde_forw_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int pacejka_model_tracking_mpc_expl_vde_forw_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real pacejka_model_tracking_mpc_expl_vde_forw_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* pacejka_model_tracking_mpc_expl_vde_forw_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* pacejka_model_tracking_mpc_expl_vde_forw_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* pacejka_model_tracking_mpc_expl_vde_forw_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* pacejka_model_tracking_mpc_expl_vde_forw_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int pacejka_model_tracking_mpc_expl_vde_forw_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif