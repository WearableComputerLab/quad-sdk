/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) eval_jac_g_leg_simple_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
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

static const casadi_int casadi_s0[40] = {36, 1, 0, 36, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35};
static const casadi_int casadi_s1[58] = {54, 1, 0, 54, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53};
static const casadi_int casadi_s2[180] = {28, 36, 0, 1, 2, 3, 8, 13, 16, 18, 20, 22, 26, 32, 38, 45, 52, 60, 67, 74, 82, 89, 96, 104, 111, 118, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 137, 139, 141, 0, 1, 2, 3, 4, 5, 10, 11, 3, 4, 5, 9, 11, 5, 9, 10, 0, 6, 1, 7, 2, 8, 3, 9, 10, 11, 3, 4, 5, 9, 10, 11, 3, 4, 5, 9, 10, 11, 0, 6, 9, 10, 11, 12, 13, 1, 7, 9, 10, 11, 14, 15, 2, 8, 9, 10, 12, 13, 14, 15, 0, 6, 9, 10, 11, 16, 17, 1, 7, 9, 10, 11, 18, 19, 2, 8, 9, 10, 16, 17, 18, 19, 0, 6, 9, 10, 11, 20, 21, 1, 7, 9, 10, 11, 22, 23, 2, 8, 9, 10, 20, 21, 22, 23, 0, 6, 9, 10, 11, 24, 25, 1, 7, 9, 10, 11, 26, 27, 2, 8, 9, 10, 24, 25, 26, 27, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 10, 11, 10, 11};

/* eval_jac_g_leg_simple:(w[36],p[54])->(jac_g[28x36,141nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a8, a9;
  a0=-1.;
  if (res[0]!=0) res[0][0]=a0;
  if (res[0]!=0) res[0][1]=a0;
  if (res[0]!=0) res[0][2]=a0;
  a1=arg[0]? arg[0][3] : 0;
  a2=sin(a1);
  a3=arg[0]? arg[0][11] : 0;
  a4=arg[0]? arg[0][4] : 0;
  a5=sin(a4);
  a6=arg[1]? arg[1][0] : 0;
  a7=cos(a4);
  a8=(a6/a7);
  a9=(a5*a8);
  a10=(a3*a9);
  a10=(a2*a10);
  a11=cos(a1);
  a12=arg[0]? arg[0][10] : 0;
  a13=(a5*a8);
  a14=(a12*a13);
  a14=(a11*a14);
  a10=(a10-a14);
  a10=(a10+a0);
  if (res[0]!=0) res[0][3]=a10;
  a10=(a3*a6);
  a10=(a11*a10);
  a14=(a12*a6);
  a14=(a2*a14);
  a10=(a10+a14);
  if (res[0]!=0) res[0][4]=a10;
  a10=(a6/a7);
  a14=(a3*a10);
  a14=(a2*a14);
  a15=(a12*a10);
  a15=(a11*a15);
  a14=(a14-a15);
  if (res[0]!=0) res[0][5]=a14;
  a14=24358218631252112.;
  a15=arg[0]? arg[0][9] : 0;
  a16=-1.3877787807814460e-17;
  a16=(a16*a6);
  a17=(a15*a16);
  a18=(a14*a17);
  a19=(a12*a18);
  a20=-4.6021926780173911e-01;
  a21=arg[0]? arg[0][35] : 0;
  a21=(a21-a3);
  a22=(a20*a21);
  a19=(a19+a22);
  a19=(a11*a19);
  a22=26445640661418040.;
  a17=(a22*a17);
  a23=(a3*a17);
  a24=4.3125046780173909e-01;
  a25=arg[0]? arg[0][34] : 0;
  a25=(a25-a12);
  a26=(a24*a25);
  a23=(a23+a26);
  a23=(a2*a23);
  a19=(a19-a23);
  if (res[0]!=0) res[0][6]=a19;
  a25=(a24*a25);
  a19=(a7*a25);
  a23=(a15*a3);
  a26=(a23*a7);
  a27=3.6700698954086952e-01;
  a27=(a27*a6);
  a26=(a26*a27);
  a19=(a19-a26);
  a11=(a11*a19);
  a19=(a15*a12);
  a26=3.3803818954086950e-01;
  a26=(a26*a6);
  a28=(a7*a26);
  a29=(a19*a28);
  a30=4.6021926780173911e-01;
  a21=(a30*a21);
  a31=(a7*a21);
  a29=(a29+a31);
  a2=(a2*a29);
  a11=(a11-a2);
  if (res[0]!=0) res[0][7]=a11;
  a11=cos(a4);
  a2=sin(a1);
  a29=(a12*a2);
  a31=(a29*a8);
  a1=cos(a1);
  a32=(a3*a1);
  a33=(a32*a8);
  a31=(a31+a33);
  a31=(a11*a31);
  a4=sin(a4);
  a33=(a15*a7);
  a32=(a32*a5);
  a33=(a33+a32);
  a29=(a29*a5);
  a33=(a33+a29);
  a33=(a33/a7);
  a33=(a33/a7);
  a33=(a33*a6);
  a8=(a15*a8);
  a33=(a33-a8);
  a33=(a4*a33);
  a31=(a31+a33);
  a31=(-a31);
  if (res[0]!=0) res[0][8]=a31;
  if (res[0]!=0) res[0][9]=a0;
  a31=(a3*a1);
  a33=(a12*a2);
  a31=(a31+a33);
  a31=(a31/a7);
  a31=(a31/a7);
  a31=(a31*a6);
  a31=(a4*a31);
  a31=(-a31);
  if (res[0]!=0) res[0][10]=a31;
  a31=arg[1]? arg[1][12] : 0;
  a33=arg[0]? arg[0][21] : 0;
  a8=(a33*a6);
  a29=(a31*a8);
  a32=arg[1]? arg[1][11] : 0;
  a34=arg[0]? arg[0][22] : 0;
  a35=(a34*a6);
  a36=(a32*a35);
  a29=(a29-a36);
  a36=arg[1]? arg[1][8] : 0;
  a37=arg[0]? arg[0][19] : 0;
  a38=(a37*a6);
  a39=(a36*a38);
  a29=(a29-a39);
  a39=arg[1]? arg[1][9] : 0;
  a40=arg[0]? arg[0][18] : 0;
  a41=(a40*a6);
  a42=(a39*a41);
  a29=(a29+a42);
  a42=arg[1]? arg[1][5] : 0;
  a43=arg[0]? arg[0][16] : 0;
  a44=(a43*a6);
  a45=(a42*a44);
  a29=(a29-a45);
  a45=arg[1]? arg[1][6] : 0;
  a46=arg[0]? arg[0][15] : 0;
  a47=(a46*a6);
  a48=(a45*a47);
  a29=(a29+a48);
  a48=arg[1]? arg[1][3] : 0;
  a49=arg[0]? arg[0][12] : 0;
  a50=(a49*a6);
  a51=(a48*a50);
  a29=(a29+a51);
  a51=arg[1]? arg[1][2] : 0;
  a52=arg[0]? arg[0][13] : 0;
  a53=(a52*a6);
  a54=(a51*a53);
  a29=(a29-a54);
  a29=(a11*a29);
  a54=arg[0]? arg[0][5] : 0;
  a55=cos(a54);
  a56=(a31*a55);
  a57=sin(a54);
  a58=(a32*a57);
  a56=(a56-a58);
  a58=arg[0]? arg[0][23] : 0;
  a59=(a58*a6);
  a60=(a56*a59);
  a61=arg[1]? arg[1][13] : 0;
  a62=(a55*a35);
  a62=(a61*a62);
  a60=(a60-a62);
  a62=(a57*a8);
  a62=(a61*a62);
  a60=(a60+a62);
  a62=(a39*a55);
  a63=(a36*a57);
  a62=(a62-a63);
  a63=arg[0]? arg[0][20] : 0;
  a64=(a63*a6);
  a65=(a62*a64);
  a60=(a60+a65);
  a65=arg[1]? arg[1][10] : 0;
  a66=(a55*a38);
  a66=(a65*a66);
  a60=(a60-a66);
  a66=(a57*a41);
  a66=(a65*a66);
  a60=(a60+a66);
  a66=(a45*a55);
  a67=(a42*a57);
  a66=(a66-a67);
  a67=arg[0]? arg[0][17] : 0;
  a68=(a67*a6);
  a69=(a66*a68);
  a60=(a60+a69);
  a69=arg[1]? arg[1][7] : 0;
  a70=(a55*a44);
  a70=(a69*a70);
  a60=(a60-a70);
  a70=(a57*a47);
  a70=(a69*a70);
  a60=(a60+a70);
  a70=(a48*a55);
  a71=(a51*a57);
  a70=(a70-a71);
  a71=arg[0]? arg[0][14] : 0;
  a72=(a71*a6);
  a73=(a70*a72);
  a60=(a60+a73);
  a73=arg[1]? arg[1][4] : 0;
  a74=(a57*a50);
  a74=(a73*a74);
  a60=(a60+a74);
  a74=(a55*a53);
  a74=(a73*a74);
  a60=(a60-a74);
  a60=(a4*a60);
  a29=(a29-a60);
  if (res[0]!=0) res[0][11]=a29;
  a29=(a12*a3);
  a60=-2.8968800000000020e-02;
  a60=(a60*a6);
  a29=(a29*a60);
  a74=-9.3212278260869572e-02;
  a75=arg[0]? arg[0][33] : 0;
  a75=(a75-a15);
  a75=(a74*a75);
  a29=(a29+a75);
  a11=(a11*a29);
  a19=(a19*a1);
  a19=(a19*a26);
  a27=(a2*a27);
  a23=(a23*a27);
  a19=(a19-a23);
  a21=(a1*a21);
  a19=(a19+a21);
  a25=(a2*a25);
  a19=(a19+a25);
  a4=(a4*a19);
  a11=(a11-a4);
  if (res[0]!=0) res[0][12]=a11;
  if (res[0]!=0) res[0][13]=a0;
  a11=cos(a54);
  a4=(a61*a7);
  a8=(a4*a8);
  a59=(a7*a59);
  a19=(a32*a59);
  a8=(a8-a19);
  a64=(a7*a64);
  a19=(a36*a64);
  a8=(a8-a19);
  a19=(a65*a7);
  a41=(a19*a41);
  a8=(a8+a41);
  a68=(a7*a68);
  a41=(a42*a68);
  a8=(a8-a41);
  a41=(a69*a7);
  a47=(a41*a47);
  a8=(a8+a47);
  a72=(a7*a72);
  a47=(a51*a72);
  a8=(a8-a47);
  a47=(a73*a7);
  a50=(a47*a50);
  a8=(a8+a50);
  a8=(a11*a8);
  a54=sin(a54);
  a59=(a31*a59);
  a50=(a61*a7);
  a35=(a50*a35);
  a59=(a59-a35);
  a64=(a39*a64);
  a59=(a59+a64);
  a64=(a65*a7);
  a38=(a64*a38);
  a59=(a59-a38);
  a68=(a45*a68);
  a59=(a59+a68);
  a68=(a69*a7);
  a44=(a68*a44);
  a59=(a59-a44);
  a72=(a48*a72);
  a59=(a59+a72);
  a72=(a73*a7);
  a53=(a72*a53);
  a59=(a59-a53);
  a59=(a54*a59);
  a8=(a8-a59);
  if (res[0]!=0) res[0][14]=a8;
  a34=(a34*a6);
  a34=(a61*a34);
  a58=(a58*a6);
  a8=(a31*a58);
  a34=(a34-a8);
  a63=(a63*a6);
  a8=(a39*a63);
  a34=(a34-a8);
  a37=(a37*a6);
  a37=(a65*a37);
  a34=(a34+a37);
  a67=(a67*a6);
  a37=(a45*a67);
  a34=(a34-a37);
  a43=(a43*a6);
  a43=(a69*a43);
  a34=(a34+a43);
  a52=(a52*a6);
  a52=(a73*a52);
  a34=(a34+a52);
  a71=(a71*a6);
  a52=(a48*a71);
  a34=(a34-a52);
  a11=(a11*a34);
  a33=(a33*a6);
  a33=(a61*a33);
  a58=(a32*a58);
  a33=(a33-a58);
  a63=(a36*a63);
  a33=(a33-a63);
  a40=(a40*a6);
  a40=(a65*a40);
  a33=(a33+a40);
  a67=(a42*a67);
  a33=(a33-a67);
  a46=(a46*a6);
  a46=(a69*a46);
  a33=(a33+a46);
  a49=(a49*a6);
  a49=(a73*a49);
  a33=(a33+a49);
  a71=(a51*a71);
  a33=(a33-a71);
  a54=(a54*a33);
  a11=(a11-a54);
  if (res[0]!=0) res[0][15]=a11;
  a11=(-a6);
  if (res[0]!=0) res[0][16]=a11;
  if (res[0]!=0) res[0][17]=a0;
  a11=(-a6);
  if (res[0]!=0) res[0][18]=a11;
  if (res[0]!=0) res[0][19]=a0;
  a11=(-a6);
  if (res[0]!=0) res[0][20]=a11;
  if (res[0]!=0) res[0][21]=a0;
  a11=(-a6);
  if (res[0]!=0) res[0][22]=a11;
  if (res[0]!=0) res[0][23]=a74;
  a11=(a3*a1);
  a22=(a22*a11);
  a11=(a12*a2);
  a14=(a14*a11);
  a22=(a22+a14);
  a22=(a22*a16);
  if (res[0]!=0) res[0][24]=a22;
  a28=(a1*a28);
  a22=(a12*a28);
  a27=(a7*a27);
  a16=(a3*a27);
  a22=(a22-a16);
  a74=(a74*a5);
  a22=(a22-a74);
  if (res[0]!=0) res[0][25]=a22;
  a13=(a2*a13);
  a13=(-a13);
  if (res[0]!=0) res[0][26]=a13;
  a13=(a1*a6);
  a13=(-a13);
  if (res[0]!=0) res[0][27]=a13;
  a13=(a2*a10);
  a13=(-a13);
  if (res[0]!=0) res[0][28]=a13;
  a13=2.8968800000000020e-02;
  a13=(a13*a6);
  a22=(a3*a13);
  if (res[0]!=0) res[0][29]=a22;
  a18=(a2*a18);
  a22=(a24*a1);
  a18=(a18-a22);
  if (res[0]!=0) res[0][30]=a18;
  a28=(a15*a28);
  a60=(a5*a60);
  a3=(a3*a60);
  a28=(a28+a3);
  a3=(a7*a2);
  a24=(a24*a3);
  a28=(a28-a24);
  if (res[0]!=0) res[0][31]=a28;
  a9=(a1*a9);
  a9=(-a9);
  if (res[0]!=0) res[0][32]=a9;
  a9=(a2*a6);
  if (res[0]!=0) res[0][33]=a9;
  a10=(a1*a10);
  a10=(-a10);
  if (res[0]!=0) res[0][34]=a10;
  a13=(a12*a13);
  if (res[0]!=0) res[0][35]=a13;
  a17=(a1*a17);
  a20=(a20*a2);
  a17=(a17-a20);
  if (res[0]!=0) res[0][36]=a17;
  a12=(a12*a60);
  a15=(a15*a27);
  a12=(a12-a15);
  a1=(a1*a7);
  a30=(a30*a1);
  a12=(a12-a30);
  if (res[0]!=0) res[0][37]=a12;
  a12=7.5187969924812026e-02;
  a1=5.0000000000000000e-01;
  a15=casadi_sq(a6);
  a27=(a1*a15);
  a27=(a12*a27);
  a27=(-a27);
  if (res[0]!=0) res[0][38]=a27;
  a60=(a12*a6);
  a60=(-a60);
  if (res[0]!=0) res[0][39]=a60;
  a17=(a48*a5);
  a47=(a47*a57);
  a17=(a17+a47);
  a17=(a17*a6);
  if (res[0]!=0) res[0][40]=a17;
  a17=(a73*a55);
  a17=(a17*a6);
  if (res[0]!=0) res[0][41]=a17;
  a17=(a48*a6);
  a17=(-a17);
  if (res[0]!=0) res[0][42]=a17;
  a17=1.;
  if (res[0]!=0) res[0][43]=a17;
  if (res[0]!=0) res[0][44]=a0;
  a47=(a1*a15);
  a47=(a12*a47);
  a47=(-a47);
  if (res[0]!=0) res[0][45]=a47;
  a2=(a12*a6);
  a2=(-a2);
  if (res[0]!=0) res[0][46]=a2;
  a13=(a51*a5);
  a72=(a72*a55);
  a13=(a13+a72);
  a13=(a13*a6);
  a13=(-a13);
  if (res[0]!=0) res[0][47]=a13;
  a73=(a73*a57);
  a73=(a73*a6);
  if (res[0]!=0) res[0][48]=a73;
  a73=(a51*a6);
  if (res[0]!=0) res[0][49]=a73;
  if (res[0]!=0) res[0][50]=a17;
  if (res[0]!=0) res[0][51]=a0;
  a1=(a1*a15);
  a1=(a12*a1);
  a1=(-a1);
  if (res[0]!=0) res[0][52]=a1;
  a12=(a12*a6);
  a12=(-a12);
  if (res[0]!=0) res[0][53]=a12;
  a70=(a7*a70);
  a70=(a70*a6);
  if (res[0]!=0) res[0][54]=a70;
  a51=(a51*a55);
  a48=(a48*a57);
  a51=(a51+a48);
  a51=(a51*a6);
  a51=(-a51);
  if (res[0]!=0) res[0][55]=a51;
  a51=arg[1]? arg[1][1] : 0;
  a48=(-a51);
  if (res[0]!=0) res[0][56]=a48;
  a48=(-a51);
  if (res[0]!=0) res[0][57]=a48;
  a48=(-a51);
  if (res[0]!=0) res[0][58]=a48;
  a48=(-a51);
  if (res[0]!=0) res[0][59]=a48;
  if (res[0]!=0) res[0][60]=a27;
  if (res[0]!=0) res[0][61]=a60;
  a48=(a45*a5);
  a41=(a41*a57);
  a48=(a48+a41);
  a48=(a48*a6);
  if (res[0]!=0) res[0][62]=a48;
  a48=(a69*a55);
  a48=(a48*a6);
  if (res[0]!=0) res[0][63]=a48;
  a48=(a45*a6);
  a48=(-a48);
  if (res[0]!=0) res[0][64]=a48;
  if (res[0]!=0) res[0][65]=a17;
  if (res[0]!=0) res[0][66]=a0;
  if (res[0]!=0) res[0][67]=a47;
  if (res[0]!=0) res[0][68]=a2;
  a48=(a42*a5);
  a68=(a68*a55);
  a48=(a48+a68);
  a48=(a48*a6);
  a48=(-a48);
  if (res[0]!=0) res[0][69]=a48;
  a69=(a69*a57);
  a69=(a69*a6);
  if (res[0]!=0) res[0][70]=a69;
  a69=(a42*a6);
  if (res[0]!=0) res[0][71]=a69;
  if (res[0]!=0) res[0][72]=a17;
  if (res[0]!=0) res[0][73]=a0;
  if (res[0]!=0) res[0][74]=a1;
  if (res[0]!=0) res[0][75]=a12;
  a66=(a7*a66);
  a66=(a66*a6);
  if (res[0]!=0) res[0][76]=a66;
  a42=(a42*a55);
  a45=(a45*a57);
  a42=(a42+a45);
  a42=(a42*a6);
  a42=(-a42);
  if (res[0]!=0) res[0][77]=a42;
  a42=(-a51);
  if (res[0]!=0) res[0][78]=a42;
  a42=(-a51);
  if (res[0]!=0) res[0][79]=a42;
  a42=(-a51);
  if (res[0]!=0) res[0][80]=a42;
  a42=(-a51);
  if (res[0]!=0) res[0][81]=a42;
  if (res[0]!=0) res[0][82]=a27;
  if (res[0]!=0) res[0][83]=a60;
  a42=(a39*a5);
  a19=(a19*a57);
  a42=(a42+a19);
  a42=(a42*a6);
  if (res[0]!=0) res[0][84]=a42;
  a42=(a65*a55);
  a42=(a42*a6);
  if (res[0]!=0) res[0][85]=a42;
  a42=(a39*a6);
  a42=(-a42);
  if (res[0]!=0) res[0][86]=a42;
  if (res[0]!=0) res[0][87]=a17;
  if (res[0]!=0) res[0][88]=a0;
  if (res[0]!=0) res[0][89]=a47;
  if (res[0]!=0) res[0][90]=a2;
  a42=(a36*a5);
  a64=(a64*a55);
  a42=(a42+a64);
  a42=(a42*a6);
  a42=(-a42);
  if (res[0]!=0) res[0][91]=a42;
  a65=(a65*a57);
  a65=(a65*a6);
  if (res[0]!=0) res[0][92]=a65;
  a65=(a36*a6);
  if (res[0]!=0) res[0][93]=a65;
  if (res[0]!=0) res[0][94]=a17;
  if (res[0]!=0) res[0][95]=a0;
  if (res[0]!=0) res[0][96]=a1;
  if (res[0]!=0) res[0][97]=a12;
  a62=(a7*a62);
  a62=(a62*a6);
  if (res[0]!=0) res[0][98]=a62;
  a36=(a36*a55);
  a39=(a39*a57);
  a36=(a36+a39);
  a36=(a36*a6);
  a36=(-a36);
  if (res[0]!=0) res[0][99]=a36;
  a36=(-a51);
  if (res[0]!=0) res[0][100]=a36;
  a36=(-a51);
  if (res[0]!=0) res[0][101]=a36;
  a36=(-a51);
  if (res[0]!=0) res[0][102]=a36;
  a36=(-a51);
  if (res[0]!=0) res[0][103]=a36;
  if (res[0]!=0) res[0][104]=a27;
  if (res[0]!=0) res[0][105]=a60;
  a60=(a31*a5);
  a4=(a4*a57);
  a60=(a60+a4);
  a60=(a60*a6);
  if (res[0]!=0) res[0][106]=a60;
  a60=(a61*a55);
  a60=(a60*a6);
  if (res[0]!=0) res[0][107]=a60;
  a60=(a31*a6);
  a60=(-a60);
  if (res[0]!=0) res[0][108]=a60;
  if (res[0]!=0) res[0][109]=a17;
  if (res[0]!=0) res[0][110]=a0;
  if (res[0]!=0) res[0][111]=a47;
  if (res[0]!=0) res[0][112]=a2;
  a5=(a32*a5);
  a50=(a50*a55);
  a5=(a5+a50);
  a5=(a5*a6);
  a5=(-a5);
  if (res[0]!=0) res[0][113]=a5;
  a61=(a61*a57);
  a61=(a61*a6);
  if (res[0]!=0) res[0][114]=a61;
  a61=(a32*a6);
  if (res[0]!=0) res[0][115]=a61;
  if (res[0]!=0) res[0][116]=a17;
  if (res[0]!=0) res[0][117]=a0;
  if (res[0]!=0) res[0][118]=a1;
  if (res[0]!=0) res[0][119]=a12;
  a7=(a7*a56);
  a7=(a7*a6);
  if (res[0]!=0) res[0][120]=a7;
  a32=(a32*a55);
  a31=(a31*a57);
  a32=(a32+a31);
  a32=(a32*a6);
  a32=(-a32);
  if (res[0]!=0) res[0][121]=a32;
  a32=(-a51);
  if (res[0]!=0) res[0][122]=a32;
  a32=(-a51);
  if (res[0]!=0) res[0][123]=a32;
  a32=(-a51);
  if (res[0]!=0) res[0][124]=a32;
  a51=(-a51);
  if (res[0]!=0) res[0][125]=a51;
  if (res[0]!=0) res[0][126]=a17;
  if (res[0]!=0) res[0][127]=a17;
  if (res[0]!=0) res[0][128]=a17;
  if (res[0]!=0) res[0][129]=a17;
  if (res[0]!=0) res[0][130]=a17;
  if (res[0]!=0) res[0][131]=a17;
  if (res[0]!=0) res[0][132]=a17;
  if (res[0]!=0) res[0][133]=a17;
  if (res[0]!=0) res[0][134]=a17;
  a17=9.3212278260869572e-02;
  if (res[0]!=0) res[0][135]=a17;
  if (res[0]!=0) res[0][136]=a74;
  if (res[0]!=0) res[0][137]=a22;
  if (res[0]!=0) res[0][138]=a24;
  if (res[0]!=0) res[0][139]=a20;
  if (res[0]!=0) res[0][140]=a30;
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_leg_simple(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_leg_simple_alloc_mem(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_leg_simple_init_mem(int mem) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void eval_jac_g_leg_simple_free_mem(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_leg_simple_checkout(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void eval_jac_g_leg_simple_release(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT void eval_jac_g_leg_simple_incref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT void eval_jac_g_leg_simple_decref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT casadi_int eval_jac_g_leg_simple_n_in(void) { return 2;}

extern "C" CASADI_SYMBOL_EXPORT casadi_int eval_jac_g_leg_simple_n_out(void) { return 1;}

extern "C" CASADI_SYMBOL_EXPORT casadi_real eval_jac_g_leg_simple_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* eval_jac_g_leg_simple_name_in(casadi_int i){
  switch (i) {
    case 0: return "w";
    case 1: return "p";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* eval_jac_g_leg_simple_name_out(casadi_int i){
  switch (i) {
    case 0: return "jac_g";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* eval_jac_g_leg_simple_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* eval_jac_g_leg_simple_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT int eval_jac_g_leg_simple_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


