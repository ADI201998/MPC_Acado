/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 5 + 4];

acadoWorkspace.state[40] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[41] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[42] = acadoVariables.od[lRun1 * 40];
acadoWorkspace.state[43] = acadoVariables.od[lRun1 * 40 + 1];
acadoWorkspace.state[44] = acadoVariables.od[lRun1 * 40 + 2];
acadoWorkspace.state[45] = acadoVariables.od[lRun1 * 40 + 3];
acadoWorkspace.state[46] = acadoVariables.od[lRun1 * 40 + 4];
acadoWorkspace.state[47] = acadoVariables.od[lRun1 * 40 + 5];
acadoWorkspace.state[48] = acadoVariables.od[lRun1 * 40 + 6];
acadoWorkspace.state[49] = acadoVariables.od[lRun1 * 40 + 7];
acadoWorkspace.state[50] = acadoVariables.od[lRun1 * 40 + 8];
acadoWorkspace.state[51] = acadoVariables.od[lRun1 * 40 + 9];
acadoWorkspace.state[52] = acadoVariables.od[lRun1 * 40 + 10];
acadoWorkspace.state[53] = acadoVariables.od[lRun1 * 40 + 11];
acadoWorkspace.state[54] = acadoVariables.od[lRun1 * 40 + 12];
acadoWorkspace.state[55] = acadoVariables.od[lRun1 * 40 + 13];
acadoWorkspace.state[56] = acadoVariables.od[lRun1 * 40 + 14];
acadoWorkspace.state[57] = acadoVariables.od[lRun1 * 40 + 15];
acadoWorkspace.state[58] = acadoVariables.od[lRun1 * 40 + 16];
acadoWorkspace.state[59] = acadoVariables.od[lRun1 * 40 + 17];
acadoWorkspace.state[60] = acadoVariables.od[lRun1 * 40 + 18];
acadoWorkspace.state[61] = acadoVariables.od[lRun1 * 40 + 19];
acadoWorkspace.state[62] = acadoVariables.od[lRun1 * 40 + 20];
acadoWorkspace.state[63] = acadoVariables.od[lRun1 * 40 + 21];
acadoWorkspace.state[64] = acadoVariables.od[lRun1 * 40 + 22];
acadoWorkspace.state[65] = acadoVariables.od[lRun1 * 40 + 23];
acadoWorkspace.state[66] = acadoVariables.od[lRun1 * 40 + 24];
acadoWorkspace.state[67] = acadoVariables.od[lRun1 * 40 + 25];
acadoWorkspace.state[68] = acadoVariables.od[lRun1 * 40 + 26];
acadoWorkspace.state[69] = acadoVariables.od[lRun1 * 40 + 27];
acadoWorkspace.state[70] = acadoVariables.od[lRun1 * 40 + 28];
acadoWorkspace.state[71] = acadoVariables.od[lRun1 * 40 + 29];
acadoWorkspace.state[72] = acadoVariables.od[lRun1 * 40 + 30];
acadoWorkspace.state[73] = acadoVariables.od[lRun1 * 40 + 31];
acadoWorkspace.state[74] = acadoVariables.od[lRun1 * 40 + 32];
acadoWorkspace.state[75] = acadoVariables.od[lRun1 * 40 + 33];
acadoWorkspace.state[76] = acadoVariables.od[lRun1 * 40 + 34];
acadoWorkspace.state[77] = acadoVariables.od[lRun1 * 40 + 35];
acadoWorkspace.state[78] = acadoVariables.od[lRun1 * 40 + 36];
acadoWorkspace.state[79] = acadoVariables.od[lRun1 * 40 + 37];
acadoWorkspace.state[80] = acadoVariables.od[lRun1 * 40 + 38];
acadoWorkspace.state[81] = acadoVariables.od[lRun1 * 40 + 39];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 5] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 5 + 5];
acadoWorkspace.d[lRun1 * 5 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 5 + 6];
acadoWorkspace.d[lRun1 * 5 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 5 + 7];
acadoWorkspace.d[lRun1 * 5 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 5 + 8];
acadoWorkspace.d[lRun1 * 5 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 5 + 9];

acadoWorkspace.evGx[lRun1 * 25] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 25 + 1] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 25 + 2] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 25 + 3] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 25 + 4] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 25 + 5] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 25 + 6] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 25 + 7] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 25 + 8] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 25 + 9] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 25 + 10] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 25 + 11] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 25 + 12] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 25 + 13] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 25 + 14] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 25 + 15] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 25 + 16] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 25 + 17] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 25 + 18] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 25 + 19] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 25 + 20] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 25 + 21] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 25 + 22] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 25 + 23] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 25 + 24] = acadoWorkspace.state[29];

acadoWorkspace.evGu[lRun1 * 10] = acadoWorkspace.state[30];
acadoWorkspace.evGu[lRun1 * 10 + 1] = acadoWorkspace.state[31];
acadoWorkspace.evGu[lRun1 * 10 + 2] = acadoWorkspace.state[32];
acadoWorkspace.evGu[lRun1 * 10 + 3] = acadoWorkspace.state[33];
acadoWorkspace.evGu[lRun1 * 10 + 4] = acadoWorkspace.state[34];
acadoWorkspace.evGu[lRun1 * 10 + 5] = acadoWorkspace.state[35];
acadoWorkspace.evGu[lRun1 * 10 + 6] = acadoWorkspace.state[36];
acadoWorkspace.evGu[lRun1 * 10 + 7] = acadoWorkspace.state[37];
acadoWorkspace.evGu[lRun1 * 10 + 8] = acadoWorkspace.state[38];
acadoWorkspace.evGu[lRun1 * 10 + 9] = acadoWorkspace.state[39];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 5;
const real_t* od = in + 7;
/* Vector of auxiliary variables; number of elements: 146. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (pow((xd[0]-od[0]),2));
a[1] = (pow((xd[1]-od[1]),2));
a[2] = (sqrt((a[0]+a[1])));
a[3] = (pow((xd[0]-od[2]),2));
a[4] = (pow((xd[1]-od[3]),2));
a[5] = (sqrt((a[3]+a[4])));
a[6] = (pow((xd[0]-od[4]),2));
a[7] = (pow((xd[1]-od[5]),2));
a[8] = (sqrt((a[6]+a[7])));
a[9] = (pow((xd[0]-od[6]),2));
a[10] = (pow((xd[1]-od[7]),2));
a[11] = (sqrt((a[9]+a[10])));
a[12] = (pow((xd[0]-od[8]),2));
a[13] = (pow((xd[1]-od[9]),2));
a[14] = (sqrt((a[12]+a[13])));
a[15] = (pow((xd[0]-od[10]),2));
a[16] = (pow((xd[1]-od[11]),2));
a[17] = (sqrt((a[15]+a[16])));
a[18] = (pow((xd[0]-od[12]),2));
a[19] = (pow((xd[1]-od[13]),2));
a[20] = (sqrt((a[18]+a[19])));
a[21] = (pow((xd[0]-od[14]),2));
a[22] = (pow((xd[1]-od[15]),2));
a[23] = (sqrt((a[21]+a[22])));
a[24] = (pow((xd[0]-od[16]),2));
a[25] = (pow((xd[1]-od[17]),2));
a[26] = (sqrt((a[24]+a[25])));
a[27] = (pow((xd[0]-od[18]),2));
a[28] = (pow((xd[1]-od[19]),2));
a[29] = (sqrt((a[27]+a[28])));
a[30] = (pow((((od[20]*xd[0])+(od[21]*xd[1]))+od[22]),2));
a[31] = (sqrt(a[30]));
a[32] = ((od[20])*(od[20]));
a[33] = ((od[21])*(od[21]));
a[34] = (sqrt((a[32]+a[33])));
a[35] = (pow((xd[0]-od[23]),2));
a[36] = (pow((xd[1]-od[24]),2));
a[37] = (sqrt((a[35]+a[36])));
a[38] = (pow((a[37]-od[25]),2));
a[39] = (sqrt(a[38]));
a[40] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[0]));
a[41] = (1.0/sqrt((a[0]+a[1])));
a[42] = (a[41]*(real_t)(5.0000000000000000e-01));
a[43] = (a[40]*a[42]);
a[44] = ((real_t)(1.0000000000000000e+00)/(a[2]-(real_t)(2.3999999999999999e+00)));
a[45] = (a[44]*a[44]);
a[46] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[1]));
a[47] = (a[46]*a[42]);
a[48] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[2]));
a[49] = (1.0/sqrt((a[3]+a[4])));
a[50] = (a[49]*(real_t)(5.0000000000000000e-01));
a[51] = (a[48]*a[50]);
a[52] = ((real_t)(1.0000000000000000e+00)/(a[5]-(real_t)(2.3999999999999999e+00)));
a[53] = (a[52]*a[52]);
a[54] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[3]));
a[55] = (a[54]*a[50]);
a[56] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[4]));
a[57] = (1.0/sqrt((a[6]+a[7])));
a[58] = (a[57]*(real_t)(5.0000000000000000e-01));
a[59] = (a[56]*a[58]);
a[60] = ((real_t)(1.0000000000000000e+00)/(a[8]-(real_t)(2.3999999999999999e+00)));
a[61] = (a[60]*a[60]);
a[62] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[5]));
a[63] = (a[62]*a[58]);
a[64] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[6]));
a[65] = (1.0/sqrt((a[9]+a[10])));
a[66] = (a[65]*(real_t)(5.0000000000000000e-01));
a[67] = (a[64]*a[66]);
a[68] = ((real_t)(1.0000000000000000e+00)/(a[11]-(real_t)(2.3999999999999999e+00)));
a[69] = (a[68]*a[68]);
a[70] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[7]));
a[71] = (a[70]*a[66]);
a[72] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[8]));
a[73] = (1.0/sqrt((a[12]+a[13])));
a[74] = (a[73]*(real_t)(5.0000000000000000e-01));
a[75] = (a[72]*a[74]);
a[76] = ((real_t)(1.0000000000000000e+00)/(a[14]-(real_t)(2.3999999999999999e+00)));
a[77] = (a[76]*a[76]);
a[78] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[9]));
a[79] = (a[78]*a[74]);
a[80] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[10]));
a[81] = (1.0/sqrt((a[15]+a[16])));
a[82] = (a[81]*(real_t)(5.0000000000000000e-01));
a[83] = (a[80]*a[82]);
a[84] = ((real_t)(1.0000000000000000e+00)/(a[17]-(real_t)(2.3999999999999999e+00)));
a[85] = (a[84]*a[84]);
a[86] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[11]));
a[87] = (a[86]*a[82]);
a[88] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[12]));
a[89] = (1.0/sqrt((a[18]+a[19])));
a[90] = (a[89]*(real_t)(5.0000000000000000e-01));
a[91] = (a[88]*a[90]);
a[92] = ((real_t)(1.0000000000000000e+00)/(a[20]-(real_t)(2.3999999999999999e+00)));
a[93] = (a[92]*a[92]);
a[94] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[13]));
a[95] = (a[94]*a[90]);
a[96] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[14]));
a[97] = (1.0/sqrt((a[21]+a[22])));
a[98] = (a[97]*(real_t)(5.0000000000000000e-01));
a[99] = (a[96]*a[98]);
a[100] = ((real_t)(1.0000000000000000e+00)/(a[23]-(real_t)(2.3999999999999999e+00)));
a[101] = (a[100]*a[100]);
a[102] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[15]));
a[103] = (a[102]*a[98]);
a[104] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[16]));
a[105] = (1.0/sqrt((a[24]+a[25])));
a[106] = (a[105]*(real_t)(5.0000000000000000e-01));
a[107] = (a[104]*a[106]);
a[108] = ((real_t)(1.0000000000000000e+00)/(a[26]-(real_t)(2.3999999999999999e+00)));
a[109] = (a[108]*a[108]);
a[110] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[17]));
a[111] = (a[110]*a[106]);
a[112] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[18]));
a[113] = (1.0/sqrt((a[27]+a[28])));
a[114] = (a[113]*(real_t)(5.0000000000000000e-01));
a[115] = (a[112]*a[114]);
a[116] = ((real_t)(1.0000000000000000e+00)/(a[29]-(real_t)(2.3999999999999999e+00)));
a[117] = (a[116]*a[116]);
a[118] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[19]));
a[119] = (a[118]*a[114]);
a[120] = ((real_t)(2.0000000000000000e+00)*(((od[20]*xd[0])+(od[21]*xd[1]))+od[22]));
a[121] = (a[120]*od[20]);
a[122] = (1.0/sqrt(a[30]));
a[123] = (a[122]*(real_t)(5.0000000000000000e-01));
a[124] = (a[121]*a[123]);
a[125] = ((real_t)(1.0000000000000000e+00)/a[34]);
a[126] = (a[120]*od[21]);
a[127] = (a[126]*a[123]);
a[128] = ((real_t)(2.0000000000000000e+00)*(a[37]-od[25]));
a[129] = ((real_t)(2.0000000000000000e+00)*(xd[0]-od[23]));
a[130] = (1.0/sqrt((a[35]+a[36])));
a[131] = (a[130]*(real_t)(5.0000000000000000e-01));
a[132] = (a[129]*a[131]);
a[133] = (a[128]*a[132]);
a[134] = (1.0/sqrt(a[38]));
a[135] = (a[134]*(real_t)(5.0000000000000000e-01));
a[136] = (a[133]*a[135]);
a[137] = ((real_t)(2.0000000000000000e+00)*(xd[1]-od[24]));
a[138] = (a[137]*a[131]);
a[139] = (a[128]*a[138]);
a[140] = (a[139]*a[135]);
a[141] = (real_t)(0.0000000000000000e+00);
a[142] = (real_t)(0.0000000000000000e+00);
a[143] = (real_t)(0.0000000000000000e+00);
a[144] = (real_t)(0.0000000000000000e+00);
a[145] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[3];
out[3] = u[1];
out[4] = u[0];
out[5] = ((real_t)(1.0000000000000000e+00)/(a[2]-(real_t)(2.3999999999999999e+00)));
out[6] = ((real_t)(1.0000000000000000e+00)/(a[5]-(real_t)(2.3999999999999999e+00)));
out[7] = ((real_t)(1.0000000000000000e+00)/(a[8]-(real_t)(2.3999999999999999e+00)));
out[8] = ((real_t)(1.0000000000000000e+00)/(a[11]-(real_t)(2.3999999999999999e+00)));
out[9] = ((real_t)(1.0000000000000000e+00)/(a[14]-(real_t)(2.3999999999999999e+00)));
out[10] = ((real_t)(1.0000000000000000e+00)/(a[17]-(real_t)(2.3999999999999999e+00)));
out[11] = ((real_t)(1.0000000000000000e+00)/(a[20]-(real_t)(2.3999999999999999e+00)));
out[12] = ((real_t)(1.0000000000000000e+00)/(a[23]-(real_t)(2.3999999999999999e+00)));
out[13] = ((real_t)(1.0000000000000000e+00)/(a[26]-(real_t)(2.3999999999999999e+00)));
out[14] = ((real_t)(1.0000000000000000e+00)/(a[29]-(real_t)(2.3999999999999999e+00)));
out[15] = (a[31]/a[34]);
out[16] = a[39];
out[17] = (real_t)(1.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(1.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(1.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = ((real_t)(0.0000000000000000e+00)-(a[43]*a[45]));
out[43] = ((real_t)(0.0000000000000000e+00)-(a[47]*a[45]));
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = ((real_t)(0.0000000000000000e+00)-(a[51]*a[53]));
out[48] = ((real_t)(0.0000000000000000e+00)-(a[55]*a[53]));
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = ((real_t)(0.0000000000000000e+00)-(a[59]*a[61]));
out[53] = ((real_t)(0.0000000000000000e+00)-(a[63]*a[61]));
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = ((real_t)(0.0000000000000000e+00)-(a[67]*a[69]));
out[58] = ((real_t)(0.0000000000000000e+00)-(a[71]*a[69]));
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = ((real_t)(0.0000000000000000e+00)-(a[75]*a[77]));
out[63] = ((real_t)(0.0000000000000000e+00)-(a[79]*a[77]));
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = ((real_t)(0.0000000000000000e+00)-(a[83]*a[85]));
out[68] = ((real_t)(0.0000000000000000e+00)-(a[87]*a[85]));
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = ((real_t)(0.0000000000000000e+00)-(a[91]*a[93]));
out[73] = ((real_t)(0.0000000000000000e+00)-(a[95]*a[93]));
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = ((real_t)(0.0000000000000000e+00)-(a[99]*a[101]));
out[78] = ((real_t)(0.0000000000000000e+00)-(a[103]*a[101]));
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = ((real_t)(0.0000000000000000e+00)-(a[107]*a[109]));
out[83] = ((real_t)(0.0000000000000000e+00)-(a[111]*a[109]));
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = ((real_t)(0.0000000000000000e+00)-(a[115]*a[117]));
out[88] = ((real_t)(0.0000000000000000e+00)-(a[119]*a[117]));
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (a[124]*a[125]);
out[93] = (a[127]*a[125]);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = a[136];
out[98] = a[140];
out[99] = a[141];
out[100] = a[142];
out[101] = a[143];
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(1.0000000000000000e+00);
out[110] = (real_t)(1.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = (real_t)(0.0000000000000000e+00);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = a[144];
out[135] = a[145];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
}

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0]*tmpObjS[0] + tmpFx[5]*tmpObjS[17] + tmpFx[10]*tmpObjS[34] + tmpFx[15]*tmpObjS[51] + tmpFx[20]*tmpObjS[68] + tmpFx[25]*tmpObjS[85] + tmpFx[30]*tmpObjS[102] + tmpFx[35]*tmpObjS[119] + tmpFx[40]*tmpObjS[136] + tmpFx[45]*tmpObjS[153] + tmpFx[50]*tmpObjS[170] + tmpFx[55]*tmpObjS[187] + tmpFx[60]*tmpObjS[204] + tmpFx[65]*tmpObjS[221] + tmpFx[70]*tmpObjS[238] + tmpFx[75]*tmpObjS[255] + tmpFx[80]*tmpObjS[272];
tmpQ2[1] = + tmpFx[0]*tmpObjS[1] + tmpFx[5]*tmpObjS[18] + tmpFx[10]*tmpObjS[35] + tmpFx[15]*tmpObjS[52] + tmpFx[20]*tmpObjS[69] + tmpFx[25]*tmpObjS[86] + tmpFx[30]*tmpObjS[103] + tmpFx[35]*tmpObjS[120] + tmpFx[40]*tmpObjS[137] + tmpFx[45]*tmpObjS[154] + tmpFx[50]*tmpObjS[171] + tmpFx[55]*tmpObjS[188] + tmpFx[60]*tmpObjS[205] + tmpFx[65]*tmpObjS[222] + tmpFx[70]*tmpObjS[239] + tmpFx[75]*tmpObjS[256] + tmpFx[80]*tmpObjS[273];
tmpQ2[2] = + tmpFx[0]*tmpObjS[2] + tmpFx[5]*tmpObjS[19] + tmpFx[10]*tmpObjS[36] + tmpFx[15]*tmpObjS[53] + tmpFx[20]*tmpObjS[70] + tmpFx[25]*tmpObjS[87] + tmpFx[30]*tmpObjS[104] + tmpFx[35]*tmpObjS[121] + tmpFx[40]*tmpObjS[138] + tmpFx[45]*tmpObjS[155] + tmpFx[50]*tmpObjS[172] + tmpFx[55]*tmpObjS[189] + tmpFx[60]*tmpObjS[206] + tmpFx[65]*tmpObjS[223] + tmpFx[70]*tmpObjS[240] + tmpFx[75]*tmpObjS[257] + tmpFx[80]*tmpObjS[274];
tmpQ2[3] = + tmpFx[0]*tmpObjS[3] + tmpFx[5]*tmpObjS[20] + tmpFx[10]*tmpObjS[37] + tmpFx[15]*tmpObjS[54] + tmpFx[20]*tmpObjS[71] + tmpFx[25]*tmpObjS[88] + tmpFx[30]*tmpObjS[105] + tmpFx[35]*tmpObjS[122] + tmpFx[40]*tmpObjS[139] + tmpFx[45]*tmpObjS[156] + tmpFx[50]*tmpObjS[173] + tmpFx[55]*tmpObjS[190] + tmpFx[60]*tmpObjS[207] + tmpFx[65]*tmpObjS[224] + tmpFx[70]*tmpObjS[241] + tmpFx[75]*tmpObjS[258] + tmpFx[80]*tmpObjS[275];
tmpQ2[4] = + tmpFx[0]*tmpObjS[4] + tmpFx[5]*tmpObjS[21] + tmpFx[10]*tmpObjS[38] + tmpFx[15]*tmpObjS[55] + tmpFx[20]*tmpObjS[72] + tmpFx[25]*tmpObjS[89] + tmpFx[30]*tmpObjS[106] + tmpFx[35]*tmpObjS[123] + tmpFx[40]*tmpObjS[140] + tmpFx[45]*tmpObjS[157] + tmpFx[50]*tmpObjS[174] + tmpFx[55]*tmpObjS[191] + tmpFx[60]*tmpObjS[208] + tmpFx[65]*tmpObjS[225] + tmpFx[70]*tmpObjS[242] + tmpFx[75]*tmpObjS[259] + tmpFx[80]*tmpObjS[276];
tmpQ2[5] = + tmpFx[0]*tmpObjS[5] + tmpFx[5]*tmpObjS[22] + tmpFx[10]*tmpObjS[39] + tmpFx[15]*tmpObjS[56] + tmpFx[20]*tmpObjS[73] + tmpFx[25]*tmpObjS[90] + tmpFx[30]*tmpObjS[107] + tmpFx[35]*tmpObjS[124] + tmpFx[40]*tmpObjS[141] + tmpFx[45]*tmpObjS[158] + tmpFx[50]*tmpObjS[175] + tmpFx[55]*tmpObjS[192] + tmpFx[60]*tmpObjS[209] + tmpFx[65]*tmpObjS[226] + tmpFx[70]*tmpObjS[243] + tmpFx[75]*tmpObjS[260] + tmpFx[80]*tmpObjS[277];
tmpQ2[6] = + tmpFx[0]*tmpObjS[6] + tmpFx[5]*tmpObjS[23] + tmpFx[10]*tmpObjS[40] + tmpFx[15]*tmpObjS[57] + tmpFx[20]*tmpObjS[74] + tmpFx[25]*tmpObjS[91] + tmpFx[30]*tmpObjS[108] + tmpFx[35]*tmpObjS[125] + tmpFx[40]*tmpObjS[142] + tmpFx[45]*tmpObjS[159] + tmpFx[50]*tmpObjS[176] + tmpFx[55]*tmpObjS[193] + tmpFx[60]*tmpObjS[210] + tmpFx[65]*tmpObjS[227] + tmpFx[70]*tmpObjS[244] + tmpFx[75]*tmpObjS[261] + tmpFx[80]*tmpObjS[278];
tmpQ2[7] = + tmpFx[0]*tmpObjS[7] + tmpFx[5]*tmpObjS[24] + tmpFx[10]*tmpObjS[41] + tmpFx[15]*tmpObjS[58] + tmpFx[20]*tmpObjS[75] + tmpFx[25]*tmpObjS[92] + tmpFx[30]*tmpObjS[109] + tmpFx[35]*tmpObjS[126] + tmpFx[40]*tmpObjS[143] + tmpFx[45]*tmpObjS[160] + tmpFx[50]*tmpObjS[177] + tmpFx[55]*tmpObjS[194] + tmpFx[60]*tmpObjS[211] + tmpFx[65]*tmpObjS[228] + tmpFx[70]*tmpObjS[245] + tmpFx[75]*tmpObjS[262] + tmpFx[80]*tmpObjS[279];
tmpQ2[8] = + tmpFx[0]*tmpObjS[8] + tmpFx[5]*tmpObjS[25] + tmpFx[10]*tmpObjS[42] + tmpFx[15]*tmpObjS[59] + tmpFx[20]*tmpObjS[76] + tmpFx[25]*tmpObjS[93] + tmpFx[30]*tmpObjS[110] + tmpFx[35]*tmpObjS[127] + tmpFx[40]*tmpObjS[144] + tmpFx[45]*tmpObjS[161] + tmpFx[50]*tmpObjS[178] + tmpFx[55]*tmpObjS[195] + tmpFx[60]*tmpObjS[212] + tmpFx[65]*tmpObjS[229] + tmpFx[70]*tmpObjS[246] + tmpFx[75]*tmpObjS[263] + tmpFx[80]*tmpObjS[280];
tmpQ2[9] = + tmpFx[0]*tmpObjS[9] + tmpFx[5]*tmpObjS[26] + tmpFx[10]*tmpObjS[43] + tmpFx[15]*tmpObjS[60] + tmpFx[20]*tmpObjS[77] + tmpFx[25]*tmpObjS[94] + tmpFx[30]*tmpObjS[111] + tmpFx[35]*tmpObjS[128] + tmpFx[40]*tmpObjS[145] + tmpFx[45]*tmpObjS[162] + tmpFx[50]*tmpObjS[179] + tmpFx[55]*tmpObjS[196] + tmpFx[60]*tmpObjS[213] + tmpFx[65]*tmpObjS[230] + tmpFx[70]*tmpObjS[247] + tmpFx[75]*tmpObjS[264] + tmpFx[80]*tmpObjS[281];
tmpQ2[10] = + tmpFx[0]*tmpObjS[10] + tmpFx[5]*tmpObjS[27] + tmpFx[10]*tmpObjS[44] + tmpFx[15]*tmpObjS[61] + tmpFx[20]*tmpObjS[78] + tmpFx[25]*tmpObjS[95] + tmpFx[30]*tmpObjS[112] + tmpFx[35]*tmpObjS[129] + tmpFx[40]*tmpObjS[146] + tmpFx[45]*tmpObjS[163] + tmpFx[50]*tmpObjS[180] + tmpFx[55]*tmpObjS[197] + tmpFx[60]*tmpObjS[214] + tmpFx[65]*tmpObjS[231] + tmpFx[70]*tmpObjS[248] + tmpFx[75]*tmpObjS[265] + tmpFx[80]*tmpObjS[282];
tmpQ2[11] = + tmpFx[0]*tmpObjS[11] + tmpFx[5]*tmpObjS[28] + tmpFx[10]*tmpObjS[45] + tmpFx[15]*tmpObjS[62] + tmpFx[20]*tmpObjS[79] + tmpFx[25]*tmpObjS[96] + tmpFx[30]*tmpObjS[113] + tmpFx[35]*tmpObjS[130] + tmpFx[40]*tmpObjS[147] + tmpFx[45]*tmpObjS[164] + tmpFx[50]*tmpObjS[181] + tmpFx[55]*tmpObjS[198] + tmpFx[60]*tmpObjS[215] + tmpFx[65]*tmpObjS[232] + tmpFx[70]*tmpObjS[249] + tmpFx[75]*tmpObjS[266] + tmpFx[80]*tmpObjS[283];
tmpQ2[12] = + tmpFx[0]*tmpObjS[12] + tmpFx[5]*tmpObjS[29] + tmpFx[10]*tmpObjS[46] + tmpFx[15]*tmpObjS[63] + tmpFx[20]*tmpObjS[80] + tmpFx[25]*tmpObjS[97] + tmpFx[30]*tmpObjS[114] + tmpFx[35]*tmpObjS[131] + tmpFx[40]*tmpObjS[148] + tmpFx[45]*tmpObjS[165] + tmpFx[50]*tmpObjS[182] + tmpFx[55]*tmpObjS[199] + tmpFx[60]*tmpObjS[216] + tmpFx[65]*tmpObjS[233] + tmpFx[70]*tmpObjS[250] + tmpFx[75]*tmpObjS[267] + tmpFx[80]*tmpObjS[284];
tmpQ2[13] = + tmpFx[0]*tmpObjS[13] + tmpFx[5]*tmpObjS[30] + tmpFx[10]*tmpObjS[47] + tmpFx[15]*tmpObjS[64] + tmpFx[20]*tmpObjS[81] + tmpFx[25]*tmpObjS[98] + tmpFx[30]*tmpObjS[115] + tmpFx[35]*tmpObjS[132] + tmpFx[40]*tmpObjS[149] + tmpFx[45]*tmpObjS[166] + tmpFx[50]*tmpObjS[183] + tmpFx[55]*tmpObjS[200] + tmpFx[60]*tmpObjS[217] + tmpFx[65]*tmpObjS[234] + tmpFx[70]*tmpObjS[251] + tmpFx[75]*tmpObjS[268] + tmpFx[80]*tmpObjS[285];
tmpQ2[14] = + tmpFx[0]*tmpObjS[14] + tmpFx[5]*tmpObjS[31] + tmpFx[10]*tmpObjS[48] + tmpFx[15]*tmpObjS[65] + tmpFx[20]*tmpObjS[82] + tmpFx[25]*tmpObjS[99] + tmpFx[30]*tmpObjS[116] + tmpFx[35]*tmpObjS[133] + tmpFx[40]*tmpObjS[150] + tmpFx[45]*tmpObjS[167] + tmpFx[50]*tmpObjS[184] + tmpFx[55]*tmpObjS[201] + tmpFx[60]*tmpObjS[218] + tmpFx[65]*tmpObjS[235] + tmpFx[70]*tmpObjS[252] + tmpFx[75]*tmpObjS[269] + tmpFx[80]*tmpObjS[286];
tmpQ2[15] = + tmpFx[0]*tmpObjS[15] + tmpFx[5]*tmpObjS[32] + tmpFx[10]*tmpObjS[49] + tmpFx[15]*tmpObjS[66] + tmpFx[20]*tmpObjS[83] + tmpFx[25]*tmpObjS[100] + tmpFx[30]*tmpObjS[117] + tmpFx[35]*tmpObjS[134] + tmpFx[40]*tmpObjS[151] + tmpFx[45]*tmpObjS[168] + tmpFx[50]*tmpObjS[185] + tmpFx[55]*tmpObjS[202] + tmpFx[60]*tmpObjS[219] + tmpFx[65]*tmpObjS[236] + tmpFx[70]*tmpObjS[253] + tmpFx[75]*tmpObjS[270] + tmpFx[80]*tmpObjS[287];
tmpQ2[16] = + tmpFx[0]*tmpObjS[16] + tmpFx[5]*tmpObjS[33] + tmpFx[10]*tmpObjS[50] + tmpFx[15]*tmpObjS[67] + tmpFx[20]*tmpObjS[84] + tmpFx[25]*tmpObjS[101] + tmpFx[30]*tmpObjS[118] + tmpFx[35]*tmpObjS[135] + tmpFx[40]*tmpObjS[152] + tmpFx[45]*tmpObjS[169] + tmpFx[50]*tmpObjS[186] + tmpFx[55]*tmpObjS[203] + tmpFx[60]*tmpObjS[220] + tmpFx[65]*tmpObjS[237] + tmpFx[70]*tmpObjS[254] + tmpFx[75]*tmpObjS[271] + tmpFx[80]*tmpObjS[288];
tmpQ2[17] = + tmpFx[1]*tmpObjS[0] + tmpFx[6]*tmpObjS[17] + tmpFx[11]*tmpObjS[34] + tmpFx[16]*tmpObjS[51] + tmpFx[21]*tmpObjS[68] + tmpFx[26]*tmpObjS[85] + tmpFx[31]*tmpObjS[102] + tmpFx[36]*tmpObjS[119] + tmpFx[41]*tmpObjS[136] + tmpFx[46]*tmpObjS[153] + tmpFx[51]*tmpObjS[170] + tmpFx[56]*tmpObjS[187] + tmpFx[61]*tmpObjS[204] + tmpFx[66]*tmpObjS[221] + tmpFx[71]*tmpObjS[238] + tmpFx[76]*tmpObjS[255] + tmpFx[81]*tmpObjS[272];
tmpQ2[18] = + tmpFx[1]*tmpObjS[1] + tmpFx[6]*tmpObjS[18] + tmpFx[11]*tmpObjS[35] + tmpFx[16]*tmpObjS[52] + tmpFx[21]*tmpObjS[69] + tmpFx[26]*tmpObjS[86] + tmpFx[31]*tmpObjS[103] + tmpFx[36]*tmpObjS[120] + tmpFx[41]*tmpObjS[137] + tmpFx[46]*tmpObjS[154] + tmpFx[51]*tmpObjS[171] + tmpFx[56]*tmpObjS[188] + tmpFx[61]*tmpObjS[205] + tmpFx[66]*tmpObjS[222] + tmpFx[71]*tmpObjS[239] + tmpFx[76]*tmpObjS[256] + tmpFx[81]*tmpObjS[273];
tmpQ2[19] = + tmpFx[1]*tmpObjS[2] + tmpFx[6]*tmpObjS[19] + tmpFx[11]*tmpObjS[36] + tmpFx[16]*tmpObjS[53] + tmpFx[21]*tmpObjS[70] + tmpFx[26]*tmpObjS[87] + tmpFx[31]*tmpObjS[104] + tmpFx[36]*tmpObjS[121] + tmpFx[41]*tmpObjS[138] + tmpFx[46]*tmpObjS[155] + tmpFx[51]*tmpObjS[172] + tmpFx[56]*tmpObjS[189] + tmpFx[61]*tmpObjS[206] + tmpFx[66]*tmpObjS[223] + tmpFx[71]*tmpObjS[240] + tmpFx[76]*tmpObjS[257] + tmpFx[81]*tmpObjS[274];
tmpQ2[20] = + tmpFx[1]*tmpObjS[3] + tmpFx[6]*tmpObjS[20] + tmpFx[11]*tmpObjS[37] + tmpFx[16]*tmpObjS[54] + tmpFx[21]*tmpObjS[71] + tmpFx[26]*tmpObjS[88] + tmpFx[31]*tmpObjS[105] + tmpFx[36]*tmpObjS[122] + tmpFx[41]*tmpObjS[139] + tmpFx[46]*tmpObjS[156] + tmpFx[51]*tmpObjS[173] + tmpFx[56]*tmpObjS[190] + tmpFx[61]*tmpObjS[207] + tmpFx[66]*tmpObjS[224] + tmpFx[71]*tmpObjS[241] + tmpFx[76]*tmpObjS[258] + tmpFx[81]*tmpObjS[275];
tmpQ2[21] = + tmpFx[1]*tmpObjS[4] + tmpFx[6]*tmpObjS[21] + tmpFx[11]*tmpObjS[38] + tmpFx[16]*tmpObjS[55] + tmpFx[21]*tmpObjS[72] + tmpFx[26]*tmpObjS[89] + tmpFx[31]*tmpObjS[106] + tmpFx[36]*tmpObjS[123] + tmpFx[41]*tmpObjS[140] + tmpFx[46]*tmpObjS[157] + tmpFx[51]*tmpObjS[174] + tmpFx[56]*tmpObjS[191] + tmpFx[61]*tmpObjS[208] + tmpFx[66]*tmpObjS[225] + tmpFx[71]*tmpObjS[242] + tmpFx[76]*tmpObjS[259] + tmpFx[81]*tmpObjS[276];
tmpQ2[22] = + tmpFx[1]*tmpObjS[5] + tmpFx[6]*tmpObjS[22] + tmpFx[11]*tmpObjS[39] + tmpFx[16]*tmpObjS[56] + tmpFx[21]*tmpObjS[73] + tmpFx[26]*tmpObjS[90] + tmpFx[31]*tmpObjS[107] + tmpFx[36]*tmpObjS[124] + tmpFx[41]*tmpObjS[141] + tmpFx[46]*tmpObjS[158] + tmpFx[51]*tmpObjS[175] + tmpFx[56]*tmpObjS[192] + tmpFx[61]*tmpObjS[209] + tmpFx[66]*tmpObjS[226] + tmpFx[71]*tmpObjS[243] + tmpFx[76]*tmpObjS[260] + tmpFx[81]*tmpObjS[277];
tmpQ2[23] = + tmpFx[1]*tmpObjS[6] + tmpFx[6]*tmpObjS[23] + tmpFx[11]*tmpObjS[40] + tmpFx[16]*tmpObjS[57] + tmpFx[21]*tmpObjS[74] + tmpFx[26]*tmpObjS[91] + tmpFx[31]*tmpObjS[108] + tmpFx[36]*tmpObjS[125] + tmpFx[41]*tmpObjS[142] + tmpFx[46]*tmpObjS[159] + tmpFx[51]*tmpObjS[176] + tmpFx[56]*tmpObjS[193] + tmpFx[61]*tmpObjS[210] + tmpFx[66]*tmpObjS[227] + tmpFx[71]*tmpObjS[244] + tmpFx[76]*tmpObjS[261] + tmpFx[81]*tmpObjS[278];
tmpQ2[24] = + tmpFx[1]*tmpObjS[7] + tmpFx[6]*tmpObjS[24] + tmpFx[11]*tmpObjS[41] + tmpFx[16]*tmpObjS[58] + tmpFx[21]*tmpObjS[75] + tmpFx[26]*tmpObjS[92] + tmpFx[31]*tmpObjS[109] + tmpFx[36]*tmpObjS[126] + tmpFx[41]*tmpObjS[143] + tmpFx[46]*tmpObjS[160] + tmpFx[51]*tmpObjS[177] + tmpFx[56]*tmpObjS[194] + tmpFx[61]*tmpObjS[211] + tmpFx[66]*tmpObjS[228] + tmpFx[71]*tmpObjS[245] + tmpFx[76]*tmpObjS[262] + tmpFx[81]*tmpObjS[279];
tmpQ2[25] = + tmpFx[1]*tmpObjS[8] + tmpFx[6]*tmpObjS[25] + tmpFx[11]*tmpObjS[42] + tmpFx[16]*tmpObjS[59] + tmpFx[21]*tmpObjS[76] + tmpFx[26]*tmpObjS[93] + tmpFx[31]*tmpObjS[110] + tmpFx[36]*tmpObjS[127] + tmpFx[41]*tmpObjS[144] + tmpFx[46]*tmpObjS[161] + tmpFx[51]*tmpObjS[178] + tmpFx[56]*tmpObjS[195] + tmpFx[61]*tmpObjS[212] + tmpFx[66]*tmpObjS[229] + tmpFx[71]*tmpObjS[246] + tmpFx[76]*tmpObjS[263] + tmpFx[81]*tmpObjS[280];
tmpQ2[26] = + tmpFx[1]*tmpObjS[9] + tmpFx[6]*tmpObjS[26] + tmpFx[11]*tmpObjS[43] + tmpFx[16]*tmpObjS[60] + tmpFx[21]*tmpObjS[77] + tmpFx[26]*tmpObjS[94] + tmpFx[31]*tmpObjS[111] + tmpFx[36]*tmpObjS[128] + tmpFx[41]*tmpObjS[145] + tmpFx[46]*tmpObjS[162] + tmpFx[51]*tmpObjS[179] + tmpFx[56]*tmpObjS[196] + tmpFx[61]*tmpObjS[213] + tmpFx[66]*tmpObjS[230] + tmpFx[71]*tmpObjS[247] + tmpFx[76]*tmpObjS[264] + tmpFx[81]*tmpObjS[281];
tmpQ2[27] = + tmpFx[1]*tmpObjS[10] + tmpFx[6]*tmpObjS[27] + tmpFx[11]*tmpObjS[44] + tmpFx[16]*tmpObjS[61] + tmpFx[21]*tmpObjS[78] + tmpFx[26]*tmpObjS[95] + tmpFx[31]*tmpObjS[112] + tmpFx[36]*tmpObjS[129] + tmpFx[41]*tmpObjS[146] + tmpFx[46]*tmpObjS[163] + tmpFx[51]*tmpObjS[180] + tmpFx[56]*tmpObjS[197] + tmpFx[61]*tmpObjS[214] + tmpFx[66]*tmpObjS[231] + tmpFx[71]*tmpObjS[248] + tmpFx[76]*tmpObjS[265] + tmpFx[81]*tmpObjS[282];
tmpQ2[28] = + tmpFx[1]*tmpObjS[11] + tmpFx[6]*tmpObjS[28] + tmpFx[11]*tmpObjS[45] + tmpFx[16]*tmpObjS[62] + tmpFx[21]*tmpObjS[79] + tmpFx[26]*tmpObjS[96] + tmpFx[31]*tmpObjS[113] + tmpFx[36]*tmpObjS[130] + tmpFx[41]*tmpObjS[147] + tmpFx[46]*tmpObjS[164] + tmpFx[51]*tmpObjS[181] + tmpFx[56]*tmpObjS[198] + tmpFx[61]*tmpObjS[215] + tmpFx[66]*tmpObjS[232] + tmpFx[71]*tmpObjS[249] + tmpFx[76]*tmpObjS[266] + tmpFx[81]*tmpObjS[283];
tmpQ2[29] = + tmpFx[1]*tmpObjS[12] + tmpFx[6]*tmpObjS[29] + tmpFx[11]*tmpObjS[46] + tmpFx[16]*tmpObjS[63] + tmpFx[21]*tmpObjS[80] + tmpFx[26]*tmpObjS[97] + tmpFx[31]*tmpObjS[114] + tmpFx[36]*tmpObjS[131] + tmpFx[41]*tmpObjS[148] + tmpFx[46]*tmpObjS[165] + tmpFx[51]*tmpObjS[182] + tmpFx[56]*tmpObjS[199] + tmpFx[61]*tmpObjS[216] + tmpFx[66]*tmpObjS[233] + tmpFx[71]*tmpObjS[250] + tmpFx[76]*tmpObjS[267] + tmpFx[81]*tmpObjS[284];
tmpQ2[30] = + tmpFx[1]*tmpObjS[13] + tmpFx[6]*tmpObjS[30] + tmpFx[11]*tmpObjS[47] + tmpFx[16]*tmpObjS[64] + tmpFx[21]*tmpObjS[81] + tmpFx[26]*tmpObjS[98] + tmpFx[31]*tmpObjS[115] + tmpFx[36]*tmpObjS[132] + tmpFx[41]*tmpObjS[149] + tmpFx[46]*tmpObjS[166] + tmpFx[51]*tmpObjS[183] + tmpFx[56]*tmpObjS[200] + tmpFx[61]*tmpObjS[217] + tmpFx[66]*tmpObjS[234] + tmpFx[71]*tmpObjS[251] + tmpFx[76]*tmpObjS[268] + tmpFx[81]*tmpObjS[285];
tmpQ2[31] = + tmpFx[1]*tmpObjS[14] + tmpFx[6]*tmpObjS[31] + tmpFx[11]*tmpObjS[48] + tmpFx[16]*tmpObjS[65] + tmpFx[21]*tmpObjS[82] + tmpFx[26]*tmpObjS[99] + tmpFx[31]*tmpObjS[116] + tmpFx[36]*tmpObjS[133] + tmpFx[41]*tmpObjS[150] + tmpFx[46]*tmpObjS[167] + tmpFx[51]*tmpObjS[184] + tmpFx[56]*tmpObjS[201] + tmpFx[61]*tmpObjS[218] + tmpFx[66]*tmpObjS[235] + tmpFx[71]*tmpObjS[252] + tmpFx[76]*tmpObjS[269] + tmpFx[81]*tmpObjS[286];
tmpQ2[32] = + tmpFx[1]*tmpObjS[15] + tmpFx[6]*tmpObjS[32] + tmpFx[11]*tmpObjS[49] + tmpFx[16]*tmpObjS[66] + tmpFx[21]*tmpObjS[83] + tmpFx[26]*tmpObjS[100] + tmpFx[31]*tmpObjS[117] + tmpFx[36]*tmpObjS[134] + tmpFx[41]*tmpObjS[151] + tmpFx[46]*tmpObjS[168] + tmpFx[51]*tmpObjS[185] + tmpFx[56]*tmpObjS[202] + tmpFx[61]*tmpObjS[219] + tmpFx[66]*tmpObjS[236] + tmpFx[71]*tmpObjS[253] + tmpFx[76]*tmpObjS[270] + tmpFx[81]*tmpObjS[287];
tmpQ2[33] = + tmpFx[1]*tmpObjS[16] + tmpFx[6]*tmpObjS[33] + tmpFx[11]*tmpObjS[50] + tmpFx[16]*tmpObjS[67] + tmpFx[21]*tmpObjS[84] + tmpFx[26]*tmpObjS[101] + tmpFx[31]*tmpObjS[118] + tmpFx[36]*tmpObjS[135] + tmpFx[41]*tmpObjS[152] + tmpFx[46]*tmpObjS[169] + tmpFx[51]*tmpObjS[186] + tmpFx[56]*tmpObjS[203] + tmpFx[61]*tmpObjS[220] + tmpFx[66]*tmpObjS[237] + tmpFx[71]*tmpObjS[254] + tmpFx[76]*tmpObjS[271] + tmpFx[81]*tmpObjS[288];
tmpQ2[34] = + tmpFx[2]*tmpObjS[0] + tmpFx[7]*tmpObjS[17] + tmpFx[12]*tmpObjS[34] + tmpFx[17]*tmpObjS[51] + tmpFx[22]*tmpObjS[68] + tmpFx[27]*tmpObjS[85] + tmpFx[32]*tmpObjS[102] + tmpFx[37]*tmpObjS[119] + tmpFx[42]*tmpObjS[136] + tmpFx[47]*tmpObjS[153] + tmpFx[52]*tmpObjS[170] + tmpFx[57]*tmpObjS[187] + tmpFx[62]*tmpObjS[204] + tmpFx[67]*tmpObjS[221] + tmpFx[72]*tmpObjS[238] + tmpFx[77]*tmpObjS[255] + tmpFx[82]*tmpObjS[272];
tmpQ2[35] = + tmpFx[2]*tmpObjS[1] + tmpFx[7]*tmpObjS[18] + tmpFx[12]*tmpObjS[35] + tmpFx[17]*tmpObjS[52] + tmpFx[22]*tmpObjS[69] + tmpFx[27]*tmpObjS[86] + tmpFx[32]*tmpObjS[103] + tmpFx[37]*tmpObjS[120] + tmpFx[42]*tmpObjS[137] + tmpFx[47]*tmpObjS[154] + tmpFx[52]*tmpObjS[171] + tmpFx[57]*tmpObjS[188] + tmpFx[62]*tmpObjS[205] + tmpFx[67]*tmpObjS[222] + tmpFx[72]*tmpObjS[239] + tmpFx[77]*tmpObjS[256] + tmpFx[82]*tmpObjS[273];
tmpQ2[36] = + tmpFx[2]*tmpObjS[2] + tmpFx[7]*tmpObjS[19] + tmpFx[12]*tmpObjS[36] + tmpFx[17]*tmpObjS[53] + tmpFx[22]*tmpObjS[70] + tmpFx[27]*tmpObjS[87] + tmpFx[32]*tmpObjS[104] + tmpFx[37]*tmpObjS[121] + tmpFx[42]*tmpObjS[138] + tmpFx[47]*tmpObjS[155] + tmpFx[52]*tmpObjS[172] + tmpFx[57]*tmpObjS[189] + tmpFx[62]*tmpObjS[206] + tmpFx[67]*tmpObjS[223] + tmpFx[72]*tmpObjS[240] + tmpFx[77]*tmpObjS[257] + tmpFx[82]*tmpObjS[274];
tmpQ2[37] = + tmpFx[2]*tmpObjS[3] + tmpFx[7]*tmpObjS[20] + tmpFx[12]*tmpObjS[37] + tmpFx[17]*tmpObjS[54] + tmpFx[22]*tmpObjS[71] + tmpFx[27]*tmpObjS[88] + tmpFx[32]*tmpObjS[105] + tmpFx[37]*tmpObjS[122] + tmpFx[42]*tmpObjS[139] + tmpFx[47]*tmpObjS[156] + tmpFx[52]*tmpObjS[173] + tmpFx[57]*tmpObjS[190] + tmpFx[62]*tmpObjS[207] + tmpFx[67]*tmpObjS[224] + tmpFx[72]*tmpObjS[241] + tmpFx[77]*tmpObjS[258] + tmpFx[82]*tmpObjS[275];
tmpQ2[38] = + tmpFx[2]*tmpObjS[4] + tmpFx[7]*tmpObjS[21] + tmpFx[12]*tmpObjS[38] + tmpFx[17]*tmpObjS[55] + tmpFx[22]*tmpObjS[72] + tmpFx[27]*tmpObjS[89] + tmpFx[32]*tmpObjS[106] + tmpFx[37]*tmpObjS[123] + tmpFx[42]*tmpObjS[140] + tmpFx[47]*tmpObjS[157] + tmpFx[52]*tmpObjS[174] + tmpFx[57]*tmpObjS[191] + tmpFx[62]*tmpObjS[208] + tmpFx[67]*tmpObjS[225] + tmpFx[72]*tmpObjS[242] + tmpFx[77]*tmpObjS[259] + tmpFx[82]*tmpObjS[276];
tmpQ2[39] = + tmpFx[2]*tmpObjS[5] + tmpFx[7]*tmpObjS[22] + tmpFx[12]*tmpObjS[39] + tmpFx[17]*tmpObjS[56] + tmpFx[22]*tmpObjS[73] + tmpFx[27]*tmpObjS[90] + tmpFx[32]*tmpObjS[107] + tmpFx[37]*tmpObjS[124] + tmpFx[42]*tmpObjS[141] + tmpFx[47]*tmpObjS[158] + tmpFx[52]*tmpObjS[175] + tmpFx[57]*tmpObjS[192] + tmpFx[62]*tmpObjS[209] + tmpFx[67]*tmpObjS[226] + tmpFx[72]*tmpObjS[243] + tmpFx[77]*tmpObjS[260] + tmpFx[82]*tmpObjS[277];
tmpQ2[40] = + tmpFx[2]*tmpObjS[6] + tmpFx[7]*tmpObjS[23] + tmpFx[12]*tmpObjS[40] + tmpFx[17]*tmpObjS[57] + tmpFx[22]*tmpObjS[74] + tmpFx[27]*tmpObjS[91] + tmpFx[32]*tmpObjS[108] + tmpFx[37]*tmpObjS[125] + tmpFx[42]*tmpObjS[142] + tmpFx[47]*tmpObjS[159] + tmpFx[52]*tmpObjS[176] + tmpFx[57]*tmpObjS[193] + tmpFx[62]*tmpObjS[210] + tmpFx[67]*tmpObjS[227] + tmpFx[72]*tmpObjS[244] + tmpFx[77]*tmpObjS[261] + tmpFx[82]*tmpObjS[278];
tmpQ2[41] = + tmpFx[2]*tmpObjS[7] + tmpFx[7]*tmpObjS[24] + tmpFx[12]*tmpObjS[41] + tmpFx[17]*tmpObjS[58] + tmpFx[22]*tmpObjS[75] + tmpFx[27]*tmpObjS[92] + tmpFx[32]*tmpObjS[109] + tmpFx[37]*tmpObjS[126] + tmpFx[42]*tmpObjS[143] + tmpFx[47]*tmpObjS[160] + tmpFx[52]*tmpObjS[177] + tmpFx[57]*tmpObjS[194] + tmpFx[62]*tmpObjS[211] + tmpFx[67]*tmpObjS[228] + tmpFx[72]*tmpObjS[245] + tmpFx[77]*tmpObjS[262] + tmpFx[82]*tmpObjS[279];
tmpQ2[42] = + tmpFx[2]*tmpObjS[8] + tmpFx[7]*tmpObjS[25] + tmpFx[12]*tmpObjS[42] + tmpFx[17]*tmpObjS[59] + tmpFx[22]*tmpObjS[76] + tmpFx[27]*tmpObjS[93] + tmpFx[32]*tmpObjS[110] + tmpFx[37]*tmpObjS[127] + tmpFx[42]*tmpObjS[144] + tmpFx[47]*tmpObjS[161] + tmpFx[52]*tmpObjS[178] + tmpFx[57]*tmpObjS[195] + tmpFx[62]*tmpObjS[212] + tmpFx[67]*tmpObjS[229] + tmpFx[72]*tmpObjS[246] + tmpFx[77]*tmpObjS[263] + tmpFx[82]*tmpObjS[280];
tmpQ2[43] = + tmpFx[2]*tmpObjS[9] + tmpFx[7]*tmpObjS[26] + tmpFx[12]*tmpObjS[43] + tmpFx[17]*tmpObjS[60] + tmpFx[22]*tmpObjS[77] + tmpFx[27]*tmpObjS[94] + tmpFx[32]*tmpObjS[111] + tmpFx[37]*tmpObjS[128] + tmpFx[42]*tmpObjS[145] + tmpFx[47]*tmpObjS[162] + tmpFx[52]*tmpObjS[179] + tmpFx[57]*tmpObjS[196] + tmpFx[62]*tmpObjS[213] + tmpFx[67]*tmpObjS[230] + tmpFx[72]*tmpObjS[247] + tmpFx[77]*tmpObjS[264] + tmpFx[82]*tmpObjS[281];
tmpQ2[44] = + tmpFx[2]*tmpObjS[10] + tmpFx[7]*tmpObjS[27] + tmpFx[12]*tmpObjS[44] + tmpFx[17]*tmpObjS[61] + tmpFx[22]*tmpObjS[78] + tmpFx[27]*tmpObjS[95] + tmpFx[32]*tmpObjS[112] + tmpFx[37]*tmpObjS[129] + tmpFx[42]*tmpObjS[146] + tmpFx[47]*tmpObjS[163] + tmpFx[52]*tmpObjS[180] + tmpFx[57]*tmpObjS[197] + tmpFx[62]*tmpObjS[214] + tmpFx[67]*tmpObjS[231] + tmpFx[72]*tmpObjS[248] + tmpFx[77]*tmpObjS[265] + tmpFx[82]*tmpObjS[282];
tmpQ2[45] = + tmpFx[2]*tmpObjS[11] + tmpFx[7]*tmpObjS[28] + tmpFx[12]*tmpObjS[45] + tmpFx[17]*tmpObjS[62] + tmpFx[22]*tmpObjS[79] + tmpFx[27]*tmpObjS[96] + tmpFx[32]*tmpObjS[113] + tmpFx[37]*tmpObjS[130] + tmpFx[42]*tmpObjS[147] + tmpFx[47]*tmpObjS[164] + tmpFx[52]*tmpObjS[181] + tmpFx[57]*tmpObjS[198] + tmpFx[62]*tmpObjS[215] + tmpFx[67]*tmpObjS[232] + tmpFx[72]*tmpObjS[249] + tmpFx[77]*tmpObjS[266] + tmpFx[82]*tmpObjS[283];
tmpQ2[46] = + tmpFx[2]*tmpObjS[12] + tmpFx[7]*tmpObjS[29] + tmpFx[12]*tmpObjS[46] + tmpFx[17]*tmpObjS[63] + tmpFx[22]*tmpObjS[80] + tmpFx[27]*tmpObjS[97] + tmpFx[32]*tmpObjS[114] + tmpFx[37]*tmpObjS[131] + tmpFx[42]*tmpObjS[148] + tmpFx[47]*tmpObjS[165] + tmpFx[52]*tmpObjS[182] + tmpFx[57]*tmpObjS[199] + tmpFx[62]*tmpObjS[216] + tmpFx[67]*tmpObjS[233] + tmpFx[72]*tmpObjS[250] + tmpFx[77]*tmpObjS[267] + tmpFx[82]*tmpObjS[284];
tmpQ2[47] = + tmpFx[2]*tmpObjS[13] + tmpFx[7]*tmpObjS[30] + tmpFx[12]*tmpObjS[47] + tmpFx[17]*tmpObjS[64] + tmpFx[22]*tmpObjS[81] + tmpFx[27]*tmpObjS[98] + tmpFx[32]*tmpObjS[115] + tmpFx[37]*tmpObjS[132] + tmpFx[42]*tmpObjS[149] + tmpFx[47]*tmpObjS[166] + tmpFx[52]*tmpObjS[183] + tmpFx[57]*tmpObjS[200] + tmpFx[62]*tmpObjS[217] + tmpFx[67]*tmpObjS[234] + tmpFx[72]*tmpObjS[251] + tmpFx[77]*tmpObjS[268] + tmpFx[82]*tmpObjS[285];
tmpQ2[48] = + tmpFx[2]*tmpObjS[14] + tmpFx[7]*tmpObjS[31] + tmpFx[12]*tmpObjS[48] + tmpFx[17]*tmpObjS[65] + tmpFx[22]*tmpObjS[82] + tmpFx[27]*tmpObjS[99] + tmpFx[32]*tmpObjS[116] + tmpFx[37]*tmpObjS[133] + tmpFx[42]*tmpObjS[150] + tmpFx[47]*tmpObjS[167] + tmpFx[52]*tmpObjS[184] + tmpFx[57]*tmpObjS[201] + tmpFx[62]*tmpObjS[218] + tmpFx[67]*tmpObjS[235] + tmpFx[72]*tmpObjS[252] + tmpFx[77]*tmpObjS[269] + tmpFx[82]*tmpObjS[286];
tmpQ2[49] = + tmpFx[2]*tmpObjS[15] + tmpFx[7]*tmpObjS[32] + tmpFx[12]*tmpObjS[49] + tmpFx[17]*tmpObjS[66] + tmpFx[22]*tmpObjS[83] + tmpFx[27]*tmpObjS[100] + tmpFx[32]*tmpObjS[117] + tmpFx[37]*tmpObjS[134] + tmpFx[42]*tmpObjS[151] + tmpFx[47]*tmpObjS[168] + tmpFx[52]*tmpObjS[185] + tmpFx[57]*tmpObjS[202] + tmpFx[62]*tmpObjS[219] + tmpFx[67]*tmpObjS[236] + tmpFx[72]*tmpObjS[253] + tmpFx[77]*tmpObjS[270] + tmpFx[82]*tmpObjS[287];
tmpQ2[50] = + tmpFx[2]*tmpObjS[16] + tmpFx[7]*tmpObjS[33] + tmpFx[12]*tmpObjS[50] + tmpFx[17]*tmpObjS[67] + tmpFx[22]*tmpObjS[84] + tmpFx[27]*tmpObjS[101] + tmpFx[32]*tmpObjS[118] + tmpFx[37]*tmpObjS[135] + tmpFx[42]*tmpObjS[152] + tmpFx[47]*tmpObjS[169] + tmpFx[52]*tmpObjS[186] + tmpFx[57]*tmpObjS[203] + tmpFx[62]*tmpObjS[220] + tmpFx[67]*tmpObjS[237] + tmpFx[72]*tmpObjS[254] + tmpFx[77]*tmpObjS[271] + tmpFx[82]*tmpObjS[288];
tmpQ2[51] = + tmpFx[3]*tmpObjS[0] + tmpFx[8]*tmpObjS[17] + tmpFx[13]*tmpObjS[34] + tmpFx[18]*tmpObjS[51] + tmpFx[23]*tmpObjS[68] + tmpFx[28]*tmpObjS[85] + tmpFx[33]*tmpObjS[102] + tmpFx[38]*tmpObjS[119] + tmpFx[43]*tmpObjS[136] + tmpFx[48]*tmpObjS[153] + tmpFx[53]*tmpObjS[170] + tmpFx[58]*tmpObjS[187] + tmpFx[63]*tmpObjS[204] + tmpFx[68]*tmpObjS[221] + tmpFx[73]*tmpObjS[238] + tmpFx[78]*tmpObjS[255] + tmpFx[83]*tmpObjS[272];
tmpQ2[52] = + tmpFx[3]*tmpObjS[1] + tmpFx[8]*tmpObjS[18] + tmpFx[13]*tmpObjS[35] + tmpFx[18]*tmpObjS[52] + tmpFx[23]*tmpObjS[69] + tmpFx[28]*tmpObjS[86] + tmpFx[33]*tmpObjS[103] + tmpFx[38]*tmpObjS[120] + tmpFx[43]*tmpObjS[137] + tmpFx[48]*tmpObjS[154] + tmpFx[53]*tmpObjS[171] + tmpFx[58]*tmpObjS[188] + tmpFx[63]*tmpObjS[205] + tmpFx[68]*tmpObjS[222] + tmpFx[73]*tmpObjS[239] + tmpFx[78]*tmpObjS[256] + tmpFx[83]*tmpObjS[273];
tmpQ2[53] = + tmpFx[3]*tmpObjS[2] + tmpFx[8]*tmpObjS[19] + tmpFx[13]*tmpObjS[36] + tmpFx[18]*tmpObjS[53] + tmpFx[23]*tmpObjS[70] + tmpFx[28]*tmpObjS[87] + tmpFx[33]*tmpObjS[104] + tmpFx[38]*tmpObjS[121] + tmpFx[43]*tmpObjS[138] + tmpFx[48]*tmpObjS[155] + tmpFx[53]*tmpObjS[172] + tmpFx[58]*tmpObjS[189] + tmpFx[63]*tmpObjS[206] + tmpFx[68]*tmpObjS[223] + tmpFx[73]*tmpObjS[240] + tmpFx[78]*tmpObjS[257] + tmpFx[83]*tmpObjS[274];
tmpQ2[54] = + tmpFx[3]*tmpObjS[3] + tmpFx[8]*tmpObjS[20] + tmpFx[13]*tmpObjS[37] + tmpFx[18]*tmpObjS[54] + tmpFx[23]*tmpObjS[71] + tmpFx[28]*tmpObjS[88] + tmpFx[33]*tmpObjS[105] + tmpFx[38]*tmpObjS[122] + tmpFx[43]*tmpObjS[139] + tmpFx[48]*tmpObjS[156] + tmpFx[53]*tmpObjS[173] + tmpFx[58]*tmpObjS[190] + tmpFx[63]*tmpObjS[207] + tmpFx[68]*tmpObjS[224] + tmpFx[73]*tmpObjS[241] + tmpFx[78]*tmpObjS[258] + tmpFx[83]*tmpObjS[275];
tmpQ2[55] = + tmpFx[3]*tmpObjS[4] + tmpFx[8]*tmpObjS[21] + tmpFx[13]*tmpObjS[38] + tmpFx[18]*tmpObjS[55] + tmpFx[23]*tmpObjS[72] + tmpFx[28]*tmpObjS[89] + tmpFx[33]*tmpObjS[106] + tmpFx[38]*tmpObjS[123] + tmpFx[43]*tmpObjS[140] + tmpFx[48]*tmpObjS[157] + tmpFx[53]*tmpObjS[174] + tmpFx[58]*tmpObjS[191] + tmpFx[63]*tmpObjS[208] + tmpFx[68]*tmpObjS[225] + tmpFx[73]*tmpObjS[242] + tmpFx[78]*tmpObjS[259] + tmpFx[83]*tmpObjS[276];
tmpQ2[56] = + tmpFx[3]*tmpObjS[5] + tmpFx[8]*tmpObjS[22] + tmpFx[13]*tmpObjS[39] + tmpFx[18]*tmpObjS[56] + tmpFx[23]*tmpObjS[73] + tmpFx[28]*tmpObjS[90] + tmpFx[33]*tmpObjS[107] + tmpFx[38]*tmpObjS[124] + tmpFx[43]*tmpObjS[141] + tmpFx[48]*tmpObjS[158] + tmpFx[53]*tmpObjS[175] + tmpFx[58]*tmpObjS[192] + tmpFx[63]*tmpObjS[209] + tmpFx[68]*tmpObjS[226] + tmpFx[73]*tmpObjS[243] + tmpFx[78]*tmpObjS[260] + tmpFx[83]*tmpObjS[277];
tmpQ2[57] = + tmpFx[3]*tmpObjS[6] + tmpFx[8]*tmpObjS[23] + tmpFx[13]*tmpObjS[40] + tmpFx[18]*tmpObjS[57] + tmpFx[23]*tmpObjS[74] + tmpFx[28]*tmpObjS[91] + tmpFx[33]*tmpObjS[108] + tmpFx[38]*tmpObjS[125] + tmpFx[43]*tmpObjS[142] + tmpFx[48]*tmpObjS[159] + tmpFx[53]*tmpObjS[176] + tmpFx[58]*tmpObjS[193] + tmpFx[63]*tmpObjS[210] + tmpFx[68]*tmpObjS[227] + tmpFx[73]*tmpObjS[244] + tmpFx[78]*tmpObjS[261] + tmpFx[83]*tmpObjS[278];
tmpQ2[58] = + tmpFx[3]*tmpObjS[7] + tmpFx[8]*tmpObjS[24] + tmpFx[13]*tmpObjS[41] + tmpFx[18]*tmpObjS[58] + tmpFx[23]*tmpObjS[75] + tmpFx[28]*tmpObjS[92] + tmpFx[33]*tmpObjS[109] + tmpFx[38]*tmpObjS[126] + tmpFx[43]*tmpObjS[143] + tmpFx[48]*tmpObjS[160] + tmpFx[53]*tmpObjS[177] + tmpFx[58]*tmpObjS[194] + tmpFx[63]*tmpObjS[211] + tmpFx[68]*tmpObjS[228] + tmpFx[73]*tmpObjS[245] + tmpFx[78]*tmpObjS[262] + tmpFx[83]*tmpObjS[279];
tmpQ2[59] = + tmpFx[3]*tmpObjS[8] + tmpFx[8]*tmpObjS[25] + tmpFx[13]*tmpObjS[42] + tmpFx[18]*tmpObjS[59] + tmpFx[23]*tmpObjS[76] + tmpFx[28]*tmpObjS[93] + tmpFx[33]*tmpObjS[110] + tmpFx[38]*tmpObjS[127] + tmpFx[43]*tmpObjS[144] + tmpFx[48]*tmpObjS[161] + tmpFx[53]*tmpObjS[178] + tmpFx[58]*tmpObjS[195] + tmpFx[63]*tmpObjS[212] + tmpFx[68]*tmpObjS[229] + tmpFx[73]*tmpObjS[246] + tmpFx[78]*tmpObjS[263] + tmpFx[83]*tmpObjS[280];
tmpQ2[60] = + tmpFx[3]*tmpObjS[9] + tmpFx[8]*tmpObjS[26] + tmpFx[13]*tmpObjS[43] + tmpFx[18]*tmpObjS[60] + tmpFx[23]*tmpObjS[77] + tmpFx[28]*tmpObjS[94] + tmpFx[33]*tmpObjS[111] + tmpFx[38]*tmpObjS[128] + tmpFx[43]*tmpObjS[145] + tmpFx[48]*tmpObjS[162] + tmpFx[53]*tmpObjS[179] + tmpFx[58]*tmpObjS[196] + tmpFx[63]*tmpObjS[213] + tmpFx[68]*tmpObjS[230] + tmpFx[73]*tmpObjS[247] + tmpFx[78]*tmpObjS[264] + tmpFx[83]*tmpObjS[281];
tmpQ2[61] = + tmpFx[3]*tmpObjS[10] + tmpFx[8]*tmpObjS[27] + tmpFx[13]*tmpObjS[44] + tmpFx[18]*tmpObjS[61] + tmpFx[23]*tmpObjS[78] + tmpFx[28]*tmpObjS[95] + tmpFx[33]*tmpObjS[112] + tmpFx[38]*tmpObjS[129] + tmpFx[43]*tmpObjS[146] + tmpFx[48]*tmpObjS[163] + tmpFx[53]*tmpObjS[180] + tmpFx[58]*tmpObjS[197] + tmpFx[63]*tmpObjS[214] + tmpFx[68]*tmpObjS[231] + tmpFx[73]*tmpObjS[248] + tmpFx[78]*tmpObjS[265] + tmpFx[83]*tmpObjS[282];
tmpQ2[62] = + tmpFx[3]*tmpObjS[11] + tmpFx[8]*tmpObjS[28] + tmpFx[13]*tmpObjS[45] + tmpFx[18]*tmpObjS[62] + tmpFx[23]*tmpObjS[79] + tmpFx[28]*tmpObjS[96] + tmpFx[33]*tmpObjS[113] + tmpFx[38]*tmpObjS[130] + tmpFx[43]*tmpObjS[147] + tmpFx[48]*tmpObjS[164] + tmpFx[53]*tmpObjS[181] + tmpFx[58]*tmpObjS[198] + tmpFx[63]*tmpObjS[215] + tmpFx[68]*tmpObjS[232] + tmpFx[73]*tmpObjS[249] + tmpFx[78]*tmpObjS[266] + tmpFx[83]*tmpObjS[283];
tmpQ2[63] = + tmpFx[3]*tmpObjS[12] + tmpFx[8]*tmpObjS[29] + tmpFx[13]*tmpObjS[46] + tmpFx[18]*tmpObjS[63] + tmpFx[23]*tmpObjS[80] + tmpFx[28]*tmpObjS[97] + tmpFx[33]*tmpObjS[114] + tmpFx[38]*tmpObjS[131] + tmpFx[43]*tmpObjS[148] + tmpFx[48]*tmpObjS[165] + tmpFx[53]*tmpObjS[182] + tmpFx[58]*tmpObjS[199] + tmpFx[63]*tmpObjS[216] + tmpFx[68]*tmpObjS[233] + tmpFx[73]*tmpObjS[250] + tmpFx[78]*tmpObjS[267] + tmpFx[83]*tmpObjS[284];
tmpQ2[64] = + tmpFx[3]*tmpObjS[13] + tmpFx[8]*tmpObjS[30] + tmpFx[13]*tmpObjS[47] + tmpFx[18]*tmpObjS[64] + tmpFx[23]*tmpObjS[81] + tmpFx[28]*tmpObjS[98] + tmpFx[33]*tmpObjS[115] + tmpFx[38]*tmpObjS[132] + tmpFx[43]*tmpObjS[149] + tmpFx[48]*tmpObjS[166] + tmpFx[53]*tmpObjS[183] + tmpFx[58]*tmpObjS[200] + tmpFx[63]*tmpObjS[217] + tmpFx[68]*tmpObjS[234] + tmpFx[73]*tmpObjS[251] + tmpFx[78]*tmpObjS[268] + tmpFx[83]*tmpObjS[285];
tmpQ2[65] = + tmpFx[3]*tmpObjS[14] + tmpFx[8]*tmpObjS[31] + tmpFx[13]*tmpObjS[48] + tmpFx[18]*tmpObjS[65] + tmpFx[23]*tmpObjS[82] + tmpFx[28]*tmpObjS[99] + tmpFx[33]*tmpObjS[116] + tmpFx[38]*tmpObjS[133] + tmpFx[43]*tmpObjS[150] + tmpFx[48]*tmpObjS[167] + tmpFx[53]*tmpObjS[184] + tmpFx[58]*tmpObjS[201] + tmpFx[63]*tmpObjS[218] + tmpFx[68]*tmpObjS[235] + tmpFx[73]*tmpObjS[252] + tmpFx[78]*tmpObjS[269] + tmpFx[83]*tmpObjS[286];
tmpQ2[66] = + tmpFx[3]*tmpObjS[15] + tmpFx[8]*tmpObjS[32] + tmpFx[13]*tmpObjS[49] + tmpFx[18]*tmpObjS[66] + tmpFx[23]*tmpObjS[83] + tmpFx[28]*tmpObjS[100] + tmpFx[33]*tmpObjS[117] + tmpFx[38]*tmpObjS[134] + tmpFx[43]*tmpObjS[151] + tmpFx[48]*tmpObjS[168] + tmpFx[53]*tmpObjS[185] + tmpFx[58]*tmpObjS[202] + tmpFx[63]*tmpObjS[219] + tmpFx[68]*tmpObjS[236] + tmpFx[73]*tmpObjS[253] + tmpFx[78]*tmpObjS[270] + tmpFx[83]*tmpObjS[287];
tmpQ2[67] = + tmpFx[3]*tmpObjS[16] + tmpFx[8]*tmpObjS[33] + tmpFx[13]*tmpObjS[50] + tmpFx[18]*tmpObjS[67] + tmpFx[23]*tmpObjS[84] + tmpFx[28]*tmpObjS[101] + tmpFx[33]*tmpObjS[118] + tmpFx[38]*tmpObjS[135] + tmpFx[43]*tmpObjS[152] + tmpFx[48]*tmpObjS[169] + tmpFx[53]*tmpObjS[186] + tmpFx[58]*tmpObjS[203] + tmpFx[63]*tmpObjS[220] + tmpFx[68]*tmpObjS[237] + tmpFx[73]*tmpObjS[254] + tmpFx[78]*tmpObjS[271] + tmpFx[83]*tmpObjS[288];
tmpQ2[68] = + tmpFx[4]*tmpObjS[0] + tmpFx[9]*tmpObjS[17] + tmpFx[14]*tmpObjS[34] + tmpFx[19]*tmpObjS[51] + tmpFx[24]*tmpObjS[68] + tmpFx[29]*tmpObjS[85] + tmpFx[34]*tmpObjS[102] + tmpFx[39]*tmpObjS[119] + tmpFx[44]*tmpObjS[136] + tmpFx[49]*tmpObjS[153] + tmpFx[54]*tmpObjS[170] + tmpFx[59]*tmpObjS[187] + tmpFx[64]*tmpObjS[204] + tmpFx[69]*tmpObjS[221] + tmpFx[74]*tmpObjS[238] + tmpFx[79]*tmpObjS[255] + tmpFx[84]*tmpObjS[272];
tmpQ2[69] = + tmpFx[4]*tmpObjS[1] + tmpFx[9]*tmpObjS[18] + tmpFx[14]*tmpObjS[35] + tmpFx[19]*tmpObjS[52] + tmpFx[24]*tmpObjS[69] + tmpFx[29]*tmpObjS[86] + tmpFx[34]*tmpObjS[103] + tmpFx[39]*tmpObjS[120] + tmpFx[44]*tmpObjS[137] + tmpFx[49]*tmpObjS[154] + tmpFx[54]*tmpObjS[171] + tmpFx[59]*tmpObjS[188] + tmpFx[64]*tmpObjS[205] + tmpFx[69]*tmpObjS[222] + tmpFx[74]*tmpObjS[239] + tmpFx[79]*tmpObjS[256] + tmpFx[84]*tmpObjS[273];
tmpQ2[70] = + tmpFx[4]*tmpObjS[2] + tmpFx[9]*tmpObjS[19] + tmpFx[14]*tmpObjS[36] + tmpFx[19]*tmpObjS[53] + tmpFx[24]*tmpObjS[70] + tmpFx[29]*tmpObjS[87] + tmpFx[34]*tmpObjS[104] + tmpFx[39]*tmpObjS[121] + tmpFx[44]*tmpObjS[138] + tmpFx[49]*tmpObjS[155] + tmpFx[54]*tmpObjS[172] + tmpFx[59]*tmpObjS[189] + tmpFx[64]*tmpObjS[206] + tmpFx[69]*tmpObjS[223] + tmpFx[74]*tmpObjS[240] + tmpFx[79]*tmpObjS[257] + tmpFx[84]*tmpObjS[274];
tmpQ2[71] = + tmpFx[4]*tmpObjS[3] + tmpFx[9]*tmpObjS[20] + tmpFx[14]*tmpObjS[37] + tmpFx[19]*tmpObjS[54] + tmpFx[24]*tmpObjS[71] + tmpFx[29]*tmpObjS[88] + tmpFx[34]*tmpObjS[105] + tmpFx[39]*tmpObjS[122] + tmpFx[44]*tmpObjS[139] + tmpFx[49]*tmpObjS[156] + tmpFx[54]*tmpObjS[173] + tmpFx[59]*tmpObjS[190] + tmpFx[64]*tmpObjS[207] + tmpFx[69]*tmpObjS[224] + tmpFx[74]*tmpObjS[241] + tmpFx[79]*tmpObjS[258] + tmpFx[84]*tmpObjS[275];
tmpQ2[72] = + tmpFx[4]*tmpObjS[4] + tmpFx[9]*tmpObjS[21] + tmpFx[14]*tmpObjS[38] + tmpFx[19]*tmpObjS[55] + tmpFx[24]*tmpObjS[72] + tmpFx[29]*tmpObjS[89] + tmpFx[34]*tmpObjS[106] + tmpFx[39]*tmpObjS[123] + tmpFx[44]*tmpObjS[140] + tmpFx[49]*tmpObjS[157] + tmpFx[54]*tmpObjS[174] + tmpFx[59]*tmpObjS[191] + tmpFx[64]*tmpObjS[208] + tmpFx[69]*tmpObjS[225] + tmpFx[74]*tmpObjS[242] + tmpFx[79]*tmpObjS[259] + tmpFx[84]*tmpObjS[276];
tmpQ2[73] = + tmpFx[4]*tmpObjS[5] + tmpFx[9]*tmpObjS[22] + tmpFx[14]*tmpObjS[39] + tmpFx[19]*tmpObjS[56] + tmpFx[24]*tmpObjS[73] + tmpFx[29]*tmpObjS[90] + tmpFx[34]*tmpObjS[107] + tmpFx[39]*tmpObjS[124] + tmpFx[44]*tmpObjS[141] + tmpFx[49]*tmpObjS[158] + tmpFx[54]*tmpObjS[175] + tmpFx[59]*tmpObjS[192] + tmpFx[64]*tmpObjS[209] + tmpFx[69]*tmpObjS[226] + tmpFx[74]*tmpObjS[243] + tmpFx[79]*tmpObjS[260] + tmpFx[84]*tmpObjS[277];
tmpQ2[74] = + tmpFx[4]*tmpObjS[6] + tmpFx[9]*tmpObjS[23] + tmpFx[14]*tmpObjS[40] + tmpFx[19]*tmpObjS[57] + tmpFx[24]*tmpObjS[74] + tmpFx[29]*tmpObjS[91] + tmpFx[34]*tmpObjS[108] + tmpFx[39]*tmpObjS[125] + tmpFx[44]*tmpObjS[142] + tmpFx[49]*tmpObjS[159] + tmpFx[54]*tmpObjS[176] + tmpFx[59]*tmpObjS[193] + tmpFx[64]*tmpObjS[210] + tmpFx[69]*tmpObjS[227] + tmpFx[74]*tmpObjS[244] + tmpFx[79]*tmpObjS[261] + tmpFx[84]*tmpObjS[278];
tmpQ2[75] = + tmpFx[4]*tmpObjS[7] + tmpFx[9]*tmpObjS[24] + tmpFx[14]*tmpObjS[41] + tmpFx[19]*tmpObjS[58] + tmpFx[24]*tmpObjS[75] + tmpFx[29]*tmpObjS[92] + tmpFx[34]*tmpObjS[109] + tmpFx[39]*tmpObjS[126] + tmpFx[44]*tmpObjS[143] + tmpFx[49]*tmpObjS[160] + tmpFx[54]*tmpObjS[177] + tmpFx[59]*tmpObjS[194] + tmpFx[64]*tmpObjS[211] + tmpFx[69]*tmpObjS[228] + tmpFx[74]*tmpObjS[245] + tmpFx[79]*tmpObjS[262] + tmpFx[84]*tmpObjS[279];
tmpQ2[76] = + tmpFx[4]*tmpObjS[8] + tmpFx[9]*tmpObjS[25] + tmpFx[14]*tmpObjS[42] + tmpFx[19]*tmpObjS[59] + tmpFx[24]*tmpObjS[76] + tmpFx[29]*tmpObjS[93] + tmpFx[34]*tmpObjS[110] + tmpFx[39]*tmpObjS[127] + tmpFx[44]*tmpObjS[144] + tmpFx[49]*tmpObjS[161] + tmpFx[54]*tmpObjS[178] + tmpFx[59]*tmpObjS[195] + tmpFx[64]*tmpObjS[212] + tmpFx[69]*tmpObjS[229] + tmpFx[74]*tmpObjS[246] + tmpFx[79]*tmpObjS[263] + tmpFx[84]*tmpObjS[280];
tmpQ2[77] = + tmpFx[4]*tmpObjS[9] + tmpFx[9]*tmpObjS[26] + tmpFx[14]*tmpObjS[43] + tmpFx[19]*tmpObjS[60] + tmpFx[24]*tmpObjS[77] + tmpFx[29]*tmpObjS[94] + tmpFx[34]*tmpObjS[111] + tmpFx[39]*tmpObjS[128] + tmpFx[44]*tmpObjS[145] + tmpFx[49]*tmpObjS[162] + tmpFx[54]*tmpObjS[179] + tmpFx[59]*tmpObjS[196] + tmpFx[64]*tmpObjS[213] + tmpFx[69]*tmpObjS[230] + tmpFx[74]*tmpObjS[247] + tmpFx[79]*tmpObjS[264] + tmpFx[84]*tmpObjS[281];
tmpQ2[78] = + tmpFx[4]*tmpObjS[10] + tmpFx[9]*tmpObjS[27] + tmpFx[14]*tmpObjS[44] + tmpFx[19]*tmpObjS[61] + tmpFx[24]*tmpObjS[78] + tmpFx[29]*tmpObjS[95] + tmpFx[34]*tmpObjS[112] + tmpFx[39]*tmpObjS[129] + tmpFx[44]*tmpObjS[146] + tmpFx[49]*tmpObjS[163] + tmpFx[54]*tmpObjS[180] + tmpFx[59]*tmpObjS[197] + tmpFx[64]*tmpObjS[214] + tmpFx[69]*tmpObjS[231] + tmpFx[74]*tmpObjS[248] + tmpFx[79]*tmpObjS[265] + tmpFx[84]*tmpObjS[282];
tmpQ2[79] = + tmpFx[4]*tmpObjS[11] + tmpFx[9]*tmpObjS[28] + tmpFx[14]*tmpObjS[45] + tmpFx[19]*tmpObjS[62] + tmpFx[24]*tmpObjS[79] + tmpFx[29]*tmpObjS[96] + tmpFx[34]*tmpObjS[113] + tmpFx[39]*tmpObjS[130] + tmpFx[44]*tmpObjS[147] + tmpFx[49]*tmpObjS[164] + tmpFx[54]*tmpObjS[181] + tmpFx[59]*tmpObjS[198] + tmpFx[64]*tmpObjS[215] + tmpFx[69]*tmpObjS[232] + tmpFx[74]*tmpObjS[249] + tmpFx[79]*tmpObjS[266] + tmpFx[84]*tmpObjS[283];
tmpQ2[80] = + tmpFx[4]*tmpObjS[12] + tmpFx[9]*tmpObjS[29] + tmpFx[14]*tmpObjS[46] + tmpFx[19]*tmpObjS[63] + tmpFx[24]*tmpObjS[80] + tmpFx[29]*tmpObjS[97] + tmpFx[34]*tmpObjS[114] + tmpFx[39]*tmpObjS[131] + tmpFx[44]*tmpObjS[148] + tmpFx[49]*tmpObjS[165] + tmpFx[54]*tmpObjS[182] + tmpFx[59]*tmpObjS[199] + tmpFx[64]*tmpObjS[216] + tmpFx[69]*tmpObjS[233] + tmpFx[74]*tmpObjS[250] + tmpFx[79]*tmpObjS[267] + tmpFx[84]*tmpObjS[284];
tmpQ2[81] = + tmpFx[4]*tmpObjS[13] + tmpFx[9]*tmpObjS[30] + tmpFx[14]*tmpObjS[47] + tmpFx[19]*tmpObjS[64] + tmpFx[24]*tmpObjS[81] + tmpFx[29]*tmpObjS[98] + tmpFx[34]*tmpObjS[115] + tmpFx[39]*tmpObjS[132] + tmpFx[44]*tmpObjS[149] + tmpFx[49]*tmpObjS[166] + tmpFx[54]*tmpObjS[183] + tmpFx[59]*tmpObjS[200] + tmpFx[64]*tmpObjS[217] + tmpFx[69]*tmpObjS[234] + tmpFx[74]*tmpObjS[251] + tmpFx[79]*tmpObjS[268] + tmpFx[84]*tmpObjS[285];
tmpQ2[82] = + tmpFx[4]*tmpObjS[14] + tmpFx[9]*tmpObjS[31] + tmpFx[14]*tmpObjS[48] + tmpFx[19]*tmpObjS[65] + tmpFx[24]*tmpObjS[82] + tmpFx[29]*tmpObjS[99] + tmpFx[34]*tmpObjS[116] + tmpFx[39]*tmpObjS[133] + tmpFx[44]*tmpObjS[150] + tmpFx[49]*tmpObjS[167] + tmpFx[54]*tmpObjS[184] + tmpFx[59]*tmpObjS[201] + tmpFx[64]*tmpObjS[218] + tmpFx[69]*tmpObjS[235] + tmpFx[74]*tmpObjS[252] + tmpFx[79]*tmpObjS[269] + tmpFx[84]*tmpObjS[286];
tmpQ2[83] = + tmpFx[4]*tmpObjS[15] + tmpFx[9]*tmpObjS[32] + tmpFx[14]*tmpObjS[49] + tmpFx[19]*tmpObjS[66] + tmpFx[24]*tmpObjS[83] + tmpFx[29]*tmpObjS[100] + tmpFx[34]*tmpObjS[117] + tmpFx[39]*tmpObjS[134] + tmpFx[44]*tmpObjS[151] + tmpFx[49]*tmpObjS[168] + tmpFx[54]*tmpObjS[185] + tmpFx[59]*tmpObjS[202] + tmpFx[64]*tmpObjS[219] + tmpFx[69]*tmpObjS[236] + tmpFx[74]*tmpObjS[253] + tmpFx[79]*tmpObjS[270] + tmpFx[84]*tmpObjS[287];
tmpQ2[84] = + tmpFx[4]*tmpObjS[16] + tmpFx[9]*tmpObjS[33] + tmpFx[14]*tmpObjS[50] + tmpFx[19]*tmpObjS[67] + tmpFx[24]*tmpObjS[84] + tmpFx[29]*tmpObjS[101] + tmpFx[34]*tmpObjS[118] + tmpFx[39]*tmpObjS[135] + tmpFx[44]*tmpObjS[152] + tmpFx[49]*tmpObjS[169] + tmpFx[54]*tmpObjS[186] + tmpFx[59]*tmpObjS[203] + tmpFx[64]*tmpObjS[220] + tmpFx[69]*tmpObjS[237] + tmpFx[74]*tmpObjS[254] + tmpFx[79]*tmpObjS[271] + tmpFx[84]*tmpObjS[288];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[5] + tmpQ2[2]*tmpFx[10] + tmpQ2[3]*tmpFx[15] + tmpQ2[4]*tmpFx[20] + tmpQ2[5]*tmpFx[25] + tmpQ2[6]*tmpFx[30] + tmpQ2[7]*tmpFx[35] + tmpQ2[8]*tmpFx[40] + tmpQ2[9]*tmpFx[45] + tmpQ2[10]*tmpFx[50] + tmpQ2[11]*tmpFx[55] + tmpQ2[12]*tmpFx[60] + tmpQ2[13]*tmpFx[65] + tmpQ2[14]*tmpFx[70] + tmpQ2[15]*tmpFx[75] + tmpQ2[16]*tmpFx[80];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[6] + tmpQ2[2]*tmpFx[11] + tmpQ2[3]*tmpFx[16] + tmpQ2[4]*tmpFx[21] + tmpQ2[5]*tmpFx[26] + tmpQ2[6]*tmpFx[31] + tmpQ2[7]*tmpFx[36] + tmpQ2[8]*tmpFx[41] + tmpQ2[9]*tmpFx[46] + tmpQ2[10]*tmpFx[51] + tmpQ2[11]*tmpFx[56] + tmpQ2[12]*tmpFx[61] + tmpQ2[13]*tmpFx[66] + tmpQ2[14]*tmpFx[71] + tmpQ2[15]*tmpFx[76] + tmpQ2[16]*tmpFx[81];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[7] + tmpQ2[2]*tmpFx[12] + tmpQ2[3]*tmpFx[17] + tmpQ2[4]*tmpFx[22] + tmpQ2[5]*tmpFx[27] + tmpQ2[6]*tmpFx[32] + tmpQ2[7]*tmpFx[37] + tmpQ2[8]*tmpFx[42] + tmpQ2[9]*tmpFx[47] + tmpQ2[10]*tmpFx[52] + tmpQ2[11]*tmpFx[57] + tmpQ2[12]*tmpFx[62] + tmpQ2[13]*tmpFx[67] + tmpQ2[14]*tmpFx[72] + tmpQ2[15]*tmpFx[77] + tmpQ2[16]*tmpFx[82];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[8] + tmpQ2[2]*tmpFx[13] + tmpQ2[3]*tmpFx[18] + tmpQ2[4]*tmpFx[23] + tmpQ2[5]*tmpFx[28] + tmpQ2[6]*tmpFx[33] + tmpQ2[7]*tmpFx[38] + tmpQ2[8]*tmpFx[43] + tmpQ2[9]*tmpFx[48] + tmpQ2[10]*tmpFx[53] + tmpQ2[11]*tmpFx[58] + tmpQ2[12]*tmpFx[63] + tmpQ2[13]*tmpFx[68] + tmpQ2[14]*tmpFx[73] + tmpQ2[15]*tmpFx[78] + tmpQ2[16]*tmpFx[83];
tmpQ1[4] = + tmpQ2[0]*tmpFx[4] + tmpQ2[1]*tmpFx[9] + tmpQ2[2]*tmpFx[14] + tmpQ2[3]*tmpFx[19] + tmpQ2[4]*tmpFx[24] + tmpQ2[5]*tmpFx[29] + tmpQ2[6]*tmpFx[34] + tmpQ2[7]*tmpFx[39] + tmpQ2[8]*tmpFx[44] + tmpQ2[9]*tmpFx[49] + tmpQ2[10]*tmpFx[54] + tmpQ2[11]*tmpFx[59] + tmpQ2[12]*tmpFx[64] + tmpQ2[13]*tmpFx[69] + tmpQ2[14]*tmpFx[74] + tmpQ2[15]*tmpFx[79] + tmpQ2[16]*tmpFx[84];
tmpQ1[5] = + tmpQ2[17]*tmpFx[0] + tmpQ2[18]*tmpFx[5] + tmpQ2[19]*tmpFx[10] + tmpQ2[20]*tmpFx[15] + tmpQ2[21]*tmpFx[20] + tmpQ2[22]*tmpFx[25] + tmpQ2[23]*tmpFx[30] + tmpQ2[24]*tmpFx[35] + tmpQ2[25]*tmpFx[40] + tmpQ2[26]*tmpFx[45] + tmpQ2[27]*tmpFx[50] + tmpQ2[28]*tmpFx[55] + tmpQ2[29]*tmpFx[60] + tmpQ2[30]*tmpFx[65] + tmpQ2[31]*tmpFx[70] + tmpQ2[32]*tmpFx[75] + tmpQ2[33]*tmpFx[80];
tmpQ1[6] = + tmpQ2[17]*tmpFx[1] + tmpQ2[18]*tmpFx[6] + tmpQ2[19]*tmpFx[11] + tmpQ2[20]*tmpFx[16] + tmpQ2[21]*tmpFx[21] + tmpQ2[22]*tmpFx[26] + tmpQ2[23]*tmpFx[31] + tmpQ2[24]*tmpFx[36] + tmpQ2[25]*tmpFx[41] + tmpQ2[26]*tmpFx[46] + tmpQ2[27]*tmpFx[51] + tmpQ2[28]*tmpFx[56] + tmpQ2[29]*tmpFx[61] + tmpQ2[30]*tmpFx[66] + tmpQ2[31]*tmpFx[71] + tmpQ2[32]*tmpFx[76] + tmpQ2[33]*tmpFx[81];
tmpQ1[7] = + tmpQ2[17]*tmpFx[2] + tmpQ2[18]*tmpFx[7] + tmpQ2[19]*tmpFx[12] + tmpQ2[20]*tmpFx[17] + tmpQ2[21]*tmpFx[22] + tmpQ2[22]*tmpFx[27] + tmpQ2[23]*tmpFx[32] + tmpQ2[24]*tmpFx[37] + tmpQ2[25]*tmpFx[42] + tmpQ2[26]*tmpFx[47] + tmpQ2[27]*tmpFx[52] + tmpQ2[28]*tmpFx[57] + tmpQ2[29]*tmpFx[62] + tmpQ2[30]*tmpFx[67] + tmpQ2[31]*tmpFx[72] + tmpQ2[32]*tmpFx[77] + tmpQ2[33]*tmpFx[82];
tmpQ1[8] = + tmpQ2[17]*tmpFx[3] + tmpQ2[18]*tmpFx[8] + tmpQ2[19]*tmpFx[13] + tmpQ2[20]*tmpFx[18] + tmpQ2[21]*tmpFx[23] + tmpQ2[22]*tmpFx[28] + tmpQ2[23]*tmpFx[33] + tmpQ2[24]*tmpFx[38] + tmpQ2[25]*tmpFx[43] + tmpQ2[26]*tmpFx[48] + tmpQ2[27]*tmpFx[53] + tmpQ2[28]*tmpFx[58] + tmpQ2[29]*tmpFx[63] + tmpQ2[30]*tmpFx[68] + tmpQ2[31]*tmpFx[73] + tmpQ2[32]*tmpFx[78] + tmpQ2[33]*tmpFx[83];
tmpQ1[9] = + tmpQ2[17]*tmpFx[4] + tmpQ2[18]*tmpFx[9] + tmpQ2[19]*tmpFx[14] + tmpQ2[20]*tmpFx[19] + tmpQ2[21]*tmpFx[24] + tmpQ2[22]*tmpFx[29] + tmpQ2[23]*tmpFx[34] + tmpQ2[24]*tmpFx[39] + tmpQ2[25]*tmpFx[44] + tmpQ2[26]*tmpFx[49] + tmpQ2[27]*tmpFx[54] + tmpQ2[28]*tmpFx[59] + tmpQ2[29]*tmpFx[64] + tmpQ2[30]*tmpFx[69] + tmpQ2[31]*tmpFx[74] + tmpQ2[32]*tmpFx[79] + tmpQ2[33]*tmpFx[84];
tmpQ1[10] = + tmpQ2[34]*tmpFx[0] + tmpQ2[35]*tmpFx[5] + tmpQ2[36]*tmpFx[10] + tmpQ2[37]*tmpFx[15] + tmpQ2[38]*tmpFx[20] + tmpQ2[39]*tmpFx[25] + tmpQ2[40]*tmpFx[30] + tmpQ2[41]*tmpFx[35] + tmpQ2[42]*tmpFx[40] + tmpQ2[43]*tmpFx[45] + tmpQ2[44]*tmpFx[50] + tmpQ2[45]*tmpFx[55] + tmpQ2[46]*tmpFx[60] + tmpQ2[47]*tmpFx[65] + tmpQ2[48]*tmpFx[70] + tmpQ2[49]*tmpFx[75] + tmpQ2[50]*tmpFx[80];
tmpQ1[11] = + tmpQ2[34]*tmpFx[1] + tmpQ2[35]*tmpFx[6] + tmpQ2[36]*tmpFx[11] + tmpQ2[37]*tmpFx[16] + tmpQ2[38]*tmpFx[21] + tmpQ2[39]*tmpFx[26] + tmpQ2[40]*tmpFx[31] + tmpQ2[41]*tmpFx[36] + tmpQ2[42]*tmpFx[41] + tmpQ2[43]*tmpFx[46] + tmpQ2[44]*tmpFx[51] + tmpQ2[45]*tmpFx[56] + tmpQ2[46]*tmpFx[61] + tmpQ2[47]*tmpFx[66] + tmpQ2[48]*tmpFx[71] + tmpQ2[49]*tmpFx[76] + tmpQ2[50]*tmpFx[81];
tmpQ1[12] = + tmpQ2[34]*tmpFx[2] + tmpQ2[35]*tmpFx[7] + tmpQ2[36]*tmpFx[12] + tmpQ2[37]*tmpFx[17] + tmpQ2[38]*tmpFx[22] + tmpQ2[39]*tmpFx[27] + tmpQ2[40]*tmpFx[32] + tmpQ2[41]*tmpFx[37] + tmpQ2[42]*tmpFx[42] + tmpQ2[43]*tmpFx[47] + tmpQ2[44]*tmpFx[52] + tmpQ2[45]*tmpFx[57] + tmpQ2[46]*tmpFx[62] + tmpQ2[47]*tmpFx[67] + tmpQ2[48]*tmpFx[72] + tmpQ2[49]*tmpFx[77] + tmpQ2[50]*tmpFx[82];
tmpQ1[13] = + tmpQ2[34]*tmpFx[3] + tmpQ2[35]*tmpFx[8] + tmpQ2[36]*tmpFx[13] + tmpQ2[37]*tmpFx[18] + tmpQ2[38]*tmpFx[23] + tmpQ2[39]*tmpFx[28] + tmpQ2[40]*tmpFx[33] + tmpQ2[41]*tmpFx[38] + tmpQ2[42]*tmpFx[43] + tmpQ2[43]*tmpFx[48] + tmpQ2[44]*tmpFx[53] + tmpQ2[45]*tmpFx[58] + tmpQ2[46]*tmpFx[63] + tmpQ2[47]*tmpFx[68] + tmpQ2[48]*tmpFx[73] + tmpQ2[49]*tmpFx[78] + tmpQ2[50]*tmpFx[83];
tmpQ1[14] = + tmpQ2[34]*tmpFx[4] + tmpQ2[35]*tmpFx[9] + tmpQ2[36]*tmpFx[14] + tmpQ2[37]*tmpFx[19] + tmpQ2[38]*tmpFx[24] + tmpQ2[39]*tmpFx[29] + tmpQ2[40]*tmpFx[34] + tmpQ2[41]*tmpFx[39] + tmpQ2[42]*tmpFx[44] + tmpQ2[43]*tmpFx[49] + tmpQ2[44]*tmpFx[54] + tmpQ2[45]*tmpFx[59] + tmpQ2[46]*tmpFx[64] + tmpQ2[47]*tmpFx[69] + tmpQ2[48]*tmpFx[74] + tmpQ2[49]*tmpFx[79] + tmpQ2[50]*tmpFx[84];
tmpQ1[15] = + tmpQ2[51]*tmpFx[0] + tmpQ2[52]*tmpFx[5] + tmpQ2[53]*tmpFx[10] + tmpQ2[54]*tmpFx[15] + tmpQ2[55]*tmpFx[20] + tmpQ2[56]*tmpFx[25] + tmpQ2[57]*tmpFx[30] + tmpQ2[58]*tmpFx[35] + tmpQ2[59]*tmpFx[40] + tmpQ2[60]*tmpFx[45] + tmpQ2[61]*tmpFx[50] + tmpQ2[62]*tmpFx[55] + tmpQ2[63]*tmpFx[60] + tmpQ2[64]*tmpFx[65] + tmpQ2[65]*tmpFx[70] + tmpQ2[66]*tmpFx[75] + tmpQ2[67]*tmpFx[80];
tmpQ1[16] = + tmpQ2[51]*tmpFx[1] + tmpQ2[52]*tmpFx[6] + tmpQ2[53]*tmpFx[11] + tmpQ2[54]*tmpFx[16] + tmpQ2[55]*tmpFx[21] + tmpQ2[56]*tmpFx[26] + tmpQ2[57]*tmpFx[31] + tmpQ2[58]*tmpFx[36] + tmpQ2[59]*tmpFx[41] + tmpQ2[60]*tmpFx[46] + tmpQ2[61]*tmpFx[51] + tmpQ2[62]*tmpFx[56] + tmpQ2[63]*tmpFx[61] + tmpQ2[64]*tmpFx[66] + tmpQ2[65]*tmpFx[71] + tmpQ2[66]*tmpFx[76] + tmpQ2[67]*tmpFx[81];
tmpQ1[17] = + tmpQ2[51]*tmpFx[2] + tmpQ2[52]*tmpFx[7] + tmpQ2[53]*tmpFx[12] + tmpQ2[54]*tmpFx[17] + tmpQ2[55]*tmpFx[22] + tmpQ2[56]*tmpFx[27] + tmpQ2[57]*tmpFx[32] + tmpQ2[58]*tmpFx[37] + tmpQ2[59]*tmpFx[42] + tmpQ2[60]*tmpFx[47] + tmpQ2[61]*tmpFx[52] + tmpQ2[62]*tmpFx[57] + tmpQ2[63]*tmpFx[62] + tmpQ2[64]*tmpFx[67] + tmpQ2[65]*tmpFx[72] + tmpQ2[66]*tmpFx[77] + tmpQ2[67]*tmpFx[82];
tmpQ1[18] = + tmpQ2[51]*tmpFx[3] + tmpQ2[52]*tmpFx[8] + tmpQ2[53]*tmpFx[13] + tmpQ2[54]*tmpFx[18] + tmpQ2[55]*tmpFx[23] + tmpQ2[56]*tmpFx[28] + tmpQ2[57]*tmpFx[33] + tmpQ2[58]*tmpFx[38] + tmpQ2[59]*tmpFx[43] + tmpQ2[60]*tmpFx[48] + tmpQ2[61]*tmpFx[53] + tmpQ2[62]*tmpFx[58] + tmpQ2[63]*tmpFx[63] + tmpQ2[64]*tmpFx[68] + tmpQ2[65]*tmpFx[73] + tmpQ2[66]*tmpFx[78] + tmpQ2[67]*tmpFx[83];
tmpQ1[19] = + tmpQ2[51]*tmpFx[4] + tmpQ2[52]*tmpFx[9] + tmpQ2[53]*tmpFx[14] + tmpQ2[54]*tmpFx[19] + tmpQ2[55]*tmpFx[24] + tmpQ2[56]*tmpFx[29] + tmpQ2[57]*tmpFx[34] + tmpQ2[58]*tmpFx[39] + tmpQ2[59]*tmpFx[44] + tmpQ2[60]*tmpFx[49] + tmpQ2[61]*tmpFx[54] + tmpQ2[62]*tmpFx[59] + tmpQ2[63]*tmpFx[64] + tmpQ2[64]*tmpFx[69] + tmpQ2[65]*tmpFx[74] + tmpQ2[66]*tmpFx[79] + tmpQ2[67]*tmpFx[84];
tmpQ1[20] = + tmpQ2[68]*tmpFx[0] + tmpQ2[69]*tmpFx[5] + tmpQ2[70]*tmpFx[10] + tmpQ2[71]*tmpFx[15] + tmpQ2[72]*tmpFx[20] + tmpQ2[73]*tmpFx[25] + tmpQ2[74]*tmpFx[30] + tmpQ2[75]*tmpFx[35] + tmpQ2[76]*tmpFx[40] + tmpQ2[77]*tmpFx[45] + tmpQ2[78]*tmpFx[50] + tmpQ2[79]*tmpFx[55] + tmpQ2[80]*tmpFx[60] + tmpQ2[81]*tmpFx[65] + tmpQ2[82]*tmpFx[70] + tmpQ2[83]*tmpFx[75] + tmpQ2[84]*tmpFx[80];
tmpQ1[21] = + tmpQ2[68]*tmpFx[1] + tmpQ2[69]*tmpFx[6] + tmpQ2[70]*tmpFx[11] + tmpQ2[71]*tmpFx[16] + tmpQ2[72]*tmpFx[21] + tmpQ2[73]*tmpFx[26] + tmpQ2[74]*tmpFx[31] + tmpQ2[75]*tmpFx[36] + tmpQ2[76]*tmpFx[41] + tmpQ2[77]*tmpFx[46] + tmpQ2[78]*tmpFx[51] + tmpQ2[79]*tmpFx[56] + tmpQ2[80]*tmpFx[61] + tmpQ2[81]*tmpFx[66] + tmpQ2[82]*tmpFx[71] + tmpQ2[83]*tmpFx[76] + tmpQ2[84]*tmpFx[81];
tmpQ1[22] = + tmpQ2[68]*tmpFx[2] + tmpQ2[69]*tmpFx[7] + tmpQ2[70]*tmpFx[12] + tmpQ2[71]*tmpFx[17] + tmpQ2[72]*tmpFx[22] + tmpQ2[73]*tmpFx[27] + tmpQ2[74]*tmpFx[32] + tmpQ2[75]*tmpFx[37] + tmpQ2[76]*tmpFx[42] + tmpQ2[77]*tmpFx[47] + tmpQ2[78]*tmpFx[52] + tmpQ2[79]*tmpFx[57] + tmpQ2[80]*tmpFx[62] + tmpQ2[81]*tmpFx[67] + tmpQ2[82]*tmpFx[72] + tmpQ2[83]*tmpFx[77] + tmpQ2[84]*tmpFx[82];
tmpQ1[23] = + tmpQ2[68]*tmpFx[3] + tmpQ2[69]*tmpFx[8] + tmpQ2[70]*tmpFx[13] + tmpQ2[71]*tmpFx[18] + tmpQ2[72]*tmpFx[23] + tmpQ2[73]*tmpFx[28] + tmpQ2[74]*tmpFx[33] + tmpQ2[75]*tmpFx[38] + tmpQ2[76]*tmpFx[43] + tmpQ2[77]*tmpFx[48] + tmpQ2[78]*tmpFx[53] + tmpQ2[79]*tmpFx[58] + tmpQ2[80]*tmpFx[63] + tmpQ2[81]*tmpFx[68] + tmpQ2[82]*tmpFx[73] + tmpQ2[83]*tmpFx[78] + tmpQ2[84]*tmpFx[83];
tmpQ1[24] = + tmpQ2[68]*tmpFx[4] + tmpQ2[69]*tmpFx[9] + tmpQ2[70]*tmpFx[14] + tmpQ2[71]*tmpFx[19] + tmpQ2[72]*tmpFx[24] + tmpQ2[73]*tmpFx[29] + tmpQ2[74]*tmpFx[34] + tmpQ2[75]*tmpFx[39] + tmpQ2[76]*tmpFx[44] + tmpQ2[77]*tmpFx[49] + tmpQ2[78]*tmpFx[54] + tmpQ2[79]*tmpFx[59] + tmpQ2[80]*tmpFx[64] + tmpQ2[81]*tmpFx[69] + tmpQ2[82]*tmpFx[74] + tmpQ2[83]*tmpFx[79] + tmpQ2[84]*tmpFx[84];
}

void acado_setObjR1R2( real_t* const tmpFu, real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = + tmpFu[0]*tmpObjS[0] + tmpFu[2]*tmpObjS[17] + tmpFu[4]*tmpObjS[34] + tmpFu[6]*tmpObjS[51] + tmpFu[8]*tmpObjS[68] + tmpFu[10]*tmpObjS[85] + tmpFu[12]*tmpObjS[102] + tmpFu[14]*tmpObjS[119] + tmpFu[16]*tmpObjS[136] + tmpFu[18]*tmpObjS[153] + tmpFu[20]*tmpObjS[170] + tmpFu[22]*tmpObjS[187] + tmpFu[24]*tmpObjS[204] + tmpFu[26]*tmpObjS[221] + tmpFu[28]*tmpObjS[238] + tmpFu[30]*tmpObjS[255] + tmpFu[32]*tmpObjS[272];
tmpR2[1] = + tmpFu[0]*tmpObjS[1] + tmpFu[2]*tmpObjS[18] + tmpFu[4]*tmpObjS[35] + tmpFu[6]*tmpObjS[52] + tmpFu[8]*tmpObjS[69] + tmpFu[10]*tmpObjS[86] + tmpFu[12]*tmpObjS[103] + tmpFu[14]*tmpObjS[120] + tmpFu[16]*tmpObjS[137] + tmpFu[18]*tmpObjS[154] + tmpFu[20]*tmpObjS[171] + tmpFu[22]*tmpObjS[188] + tmpFu[24]*tmpObjS[205] + tmpFu[26]*tmpObjS[222] + tmpFu[28]*tmpObjS[239] + tmpFu[30]*tmpObjS[256] + tmpFu[32]*tmpObjS[273];
tmpR2[2] = + tmpFu[0]*tmpObjS[2] + tmpFu[2]*tmpObjS[19] + tmpFu[4]*tmpObjS[36] + tmpFu[6]*tmpObjS[53] + tmpFu[8]*tmpObjS[70] + tmpFu[10]*tmpObjS[87] + tmpFu[12]*tmpObjS[104] + tmpFu[14]*tmpObjS[121] + tmpFu[16]*tmpObjS[138] + tmpFu[18]*tmpObjS[155] + tmpFu[20]*tmpObjS[172] + tmpFu[22]*tmpObjS[189] + tmpFu[24]*tmpObjS[206] + tmpFu[26]*tmpObjS[223] + tmpFu[28]*tmpObjS[240] + tmpFu[30]*tmpObjS[257] + tmpFu[32]*tmpObjS[274];
tmpR2[3] = + tmpFu[0]*tmpObjS[3] + tmpFu[2]*tmpObjS[20] + tmpFu[4]*tmpObjS[37] + tmpFu[6]*tmpObjS[54] + tmpFu[8]*tmpObjS[71] + tmpFu[10]*tmpObjS[88] + tmpFu[12]*tmpObjS[105] + tmpFu[14]*tmpObjS[122] + tmpFu[16]*tmpObjS[139] + tmpFu[18]*tmpObjS[156] + tmpFu[20]*tmpObjS[173] + tmpFu[22]*tmpObjS[190] + tmpFu[24]*tmpObjS[207] + tmpFu[26]*tmpObjS[224] + tmpFu[28]*tmpObjS[241] + tmpFu[30]*tmpObjS[258] + tmpFu[32]*tmpObjS[275];
tmpR2[4] = + tmpFu[0]*tmpObjS[4] + tmpFu[2]*tmpObjS[21] + tmpFu[4]*tmpObjS[38] + tmpFu[6]*tmpObjS[55] + tmpFu[8]*tmpObjS[72] + tmpFu[10]*tmpObjS[89] + tmpFu[12]*tmpObjS[106] + tmpFu[14]*tmpObjS[123] + tmpFu[16]*tmpObjS[140] + tmpFu[18]*tmpObjS[157] + tmpFu[20]*tmpObjS[174] + tmpFu[22]*tmpObjS[191] + tmpFu[24]*tmpObjS[208] + tmpFu[26]*tmpObjS[225] + tmpFu[28]*tmpObjS[242] + tmpFu[30]*tmpObjS[259] + tmpFu[32]*tmpObjS[276];
tmpR2[5] = + tmpFu[0]*tmpObjS[5] + tmpFu[2]*tmpObjS[22] + tmpFu[4]*tmpObjS[39] + tmpFu[6]*tmpObjS[56] + tmpFu[8]*tmpObjS[73] + tmpFu[10]*tmpObjS[90] + tmpFu[12]*tmpObjS[107] + tmpFu[14]*tmpObjS[124] + tmpFu[16]*tmpObjS[141] + tmpFu[18]*tmpObjS[158] + tmpFu[20]*tmpObjS[175] + tmpFu[22]*tmpObjS[192] + tmpFu[24]*tmpObjS[209] + tmpFu[26]*tmpObjS[226] + tmpFu[28]*tmpObjS[243] + tmpFu[30]*tmpObjS[260] + tmpFu[32]*tmpObjS[277];
tmpR2[6] = + tmpFu[0]*tmpObjS[6] + tmpFu[2]*tmpObjS[23] + tmpFu[4]*tmpObjS[40] + tmpFu[6]*tmpObjS[57] + tmpFu[8]*tmpObjS[74] + tmpFu[10]*tmpObjS[91] + tmpFu[12]*tmpObjS[108] + tmpFu[14]*tmpObjS[125] + tmpFu[16]*tmpObjS[142] + tmpFu[18]*tmpObjS[159] + tmpFu[20]*tmpObjS[176] + tmpFu[22]*tmpObjS[193] + tmpFu[24]*tmpObjS[210] + tmpFu[26]*tmpObjS[227] + tmpFu[28]*tmpObjS[244] + tmpFu[30]*tmpObjS[261] + tmpFu[32]*tmpObjS[278];
tmpR2[7] = + tmpFu[0]*tmpObjS[7] + tmpFu[2]*tmpObjS[24] + tmpFu[4]*tmpObjS[41] + tmpFu[6]*tmpObjS[58] + tmpFu[8]*tmpObjS[75] + tmpFu[10]*tmpObjS[92] + tmpFu[12]*tmpObjS[109] + tmpFu[14]*tmpObjS[126] + tmpFu[16]*tmpObjS[143] + tmpFu[18]*tmpObjS[160] + tmpFu[20]*tmpObjS[177] + tmpFu[22]*tmpObjS[194] + tmpFu[24]*tmpObjS[211] + tmpFu[26]*tmpObjS[228] + tmpFu[28]*tmpObjS[245] + tmpFu[30]*tmpObjS[262] + tmpFu[32]*tmpObjS[279];
tmpR2[8] = + tmpFu[0]*tmpObjS[8] + tmpFu[2]*tmpObjS[25] + tmpFu[4]*tmpObjS[42] + tmpFu[6]*tmpObjS[59] + tmpFu[8]*tmpObjS[76] + tmpFu[10]*tmpObjS[93] + tmpFu[12]*tmpObjS[110] + tmpFu[14]*tmpObjS[127] + tmpFu[16]*tmpObjS[144] + tmpFu[18]*tmpObjS[161] + tmpFu[20]*tmpObjS[178] + tmpFu[22]*tmpObjS[195] + tmpFu[24]*tmpObjS[212] + tmpFu[26]*tmpObjS[229] + tmpFu[28]*tmpObjS[246] + tmpFu[30]*tmpObjS[263] + tmpFu[32]*tmpObjS[280];
tmpR2[9] = + tmpFu[0]*tmpObjS[9] + tmpFu[2]*tmpObjS[26] + tmpFu[4]*tmpObjS[43] + tmpFu[6]*tmpObjS[60] + tmpFu[8]*tmpObjS[77] + tmpFu[10]*tmpObjS[94] + tmpFu[12]*tmpObjS[111] + tmpFu[14]*tmpObjS[128] + tmpFu[16]*tmpObjS[145] + tmpFu[18]*tmpObjS[162] + tmpFu[20]*tmpObjS[179] + tmpFu[22]*tmpObjS[196] + tmpFu[24]*tmpObjS[213] + tmpFu[26]*tmpObjS[230] + tmpFu[28]*tmpObjS[247] + tmpFu[30]*tmpObjS[264] + tmpFu[32]*tmpObjS[281];
tmpR2[10] = + tmpFu[0]*tmpObjS[10] + tmpFu[2]*tmpObjS[27] + tmpFu[4]*tmpObjS[44] + tmpFu[6]*tmpObjS[61] + tmpFu[8]*tmpObjS[78] + tmpFu[10]*tmpObjS[95] + tmpFu[12]*tmpObjS[112] + tmpFu[14]*tmpObjS[129] + tmpFu[16]*tmpObjS[146] + tmpFu[18]*tmpObjS[163] + tmpFu[20]*tmpObjS[180] + tmpFu[22]*tmpObjS[197] + tmpFu[24]*tmpObjS[214] + tmpFu[26]*tmpObjS[231] + tmpFu[28]*tmpObjS[248] + tmpFu[30]*tmpObjS[265] + tmpFu[32]*tmpObjS[282];
tmpR2[11] = + tmpFu[0]*tmpObjS[11] + tmpFu[2]*tmpObjS[28] + tmpFu[4]*tmpObjS[45] + tmpFu[6]*tmpObjS[62] + tmpFu[8]*tmpObjS[79] + tmpFu[10]*tmpObjS[96] + tmpFu[12]*tmpObjS[113] + tmpFu[14]*tmpObjS[130] + tmpFu[16]*tmpObjS[147] + tmpFu[18]*tmpObjS[164] + tmpFu[20]*tmpObjS[181] + tmpFu[22]*tmpObjS[198] + tmpFu[24]*tmpObjS[215] + tmpFu[26]*tmpObjS[232] + tmpFu[28]*tmpObjS[249] + tmpFu[30]*tmpObjS[266] + tmpFu[32]*tmpObjS[283];
tmpR2[12] = + tmpFu[0]*tmpObjS[12] + tmpFu[2]*tmpObjS[29] + tmpFu[4]*tmpObjS[46] + tmpFu[6]*tmpObjS[63] + tmpFu[8]*tmpObjS[80] + tmpFu[10]*tmpObjS[97] + tmpFu[12]*tmpObjS[114] + tmpFu[14]*tmpObjS[131] + tmpFu[16]*tmpObjS[148] + tmpFu[18]*tmpObjS[165] + tmpFu[20]*tmpObjS[182] + tmpFu[22]*tmpObjS[199] + tmpFu[24]*tmpObjS[216] + tmpFu[26]*tmpObjS[233] + tmpFu[28]*tmpObjS[250] + tmpFu[30]*tmpObjS[267] + tmpFu[32]*tmpObjS[284];
tmpR2[13] = + tmpFu[0]*tmpObjS[13] + tmpFu[2]*tmpObjS[30] + tmpFu[4]*tmpObjS[47] + tmpFu[6]*tmpObjS[64] + tmpFu[8]*tmpObjS[81] + tmpFu[10]*tmpObjS[98] + tmpFu[12]*tmpObjS[115] + tmpFu[14]*tmpObjS[132] + tmpFu[16]*tmpObjS[149] + tmpFu[18]*tmpObjS[166] + tmpFu[20]*tmpObjS[183] + tmpFu[22]*tmpObjS[200] + tmpFu[24]*tmpObjS[217] + tmpFu[26]*tmpObjS[234] + tmpFu[28]*tmpObjS[251] + tmpFu[30]*tmpObjS[268] + tmpFu[32]*tmpObjS[285];
tmpR2[14] = + tmpFu[0]*tmpObjS[14] + tmpFu[2]*tmpObjS[31] + tmpFu[4]*tmpObjS[48] + tmpFu[6]*tmpObjS[65] + tmpFu[8]*tmpObjS[82] + tmpFu[10]*tmpObjS[99] + tmpFu[12]*tmpObjS[116] + tmpFu[14]*tmpObjS[133] + tmpFu[16]*tmpObjS[150] + tmpFu[18]*tmpObjS[167] + tmpFu[20]*tmpObjS[184] + tmpFu[22]*tmpObjS[201] + tmpFu[24]*tmpObjS[218] + tmpFu[26]*tmpObjS[235] + tmpFu[28]*tmpObjS[252] + tmpFu[30]*tmpObjS[269] + tmpFu[32]*tmpObjS[286];
tmpR2[15] = + tmpFu[0]*tmpObjS[15] + tmpFu[2]*tmpObjS[32] + tmpFu[4]*tmpObjS[49] + tmpFu[6]*tmpObjS[66] + tmpFu[8]*tmpObjS[83] + tmpFu[10]*tmpObjS[100] + tmpFu[12]*tmpObjS[117] + tmpFu[14]*tmpObjS[134] + tmpFu[16]*tmpObjS[151] + tmpFu[18]*tmpObjS[168] + tmpFu[20]*tmpObjS[185] + tmpFu[22]*tmpObjS[202] + tmpFu[24]*tmpObjS[219] + tmpFu[26]*tmpObjS[236] + tmpFu[28]*tmpObjS[253] + tmpFu[30]*tmpObjS[270] + tmpFu[32]*tmpObjS[287];
tmpR2[16] = + tmpFu[0]*tmpObjS[16] + tmpFu[2]*tmpObjS[33] + tmpFu[4]*tmpObjS[50] + tmpFu[6]*tmpObjS[67] + tmpFu[8]*tmpObjS[84] + tmpFu[10]*tmpObjS[101] + tmpFu[12]*tmpObjS[118] + tmpFu[14]*tmpObjS[135] + tmpFu[16]*tmpObjS[152] + tmpFu[18]*tmpObjS[169] + tmpFu[20]*tmpObjS[186] + tmpFu[22]*tmpObjS[203] + tmpFu[24]*tmpObjS[220] + tmpFu[26]*tmpObjS[237] + tmpFu[28]*tmpObjS[254] + tmpFu[30]*tmpObjS[271] + tmpFu[32]*tmpObjS[288];
tmpR2[17] = + tmpFu[1]*tmpObjS[0] + tmpFu[3]*tmpObjS[17] + tmpFu[5]*tmpObjS[34] + tmpFu[7]*tmpObjS[51] + tmpFu[9]*tmpObjS[68] + tmpFu[11]*tmpObjS[85] + tmpFu[13]*tmpObjS[102] + tmpFu[15]*tmpObjS[119] + tmpFu[17]*tmpObjS[136] + tmpFu[19]*tmpObjS[153] + tmpFu[21]*tmpObjS[170] + tmpFu[23]*tmpObjS[187] + tmpFu[25]*tmpObjS[204] + tmpFu[27]*tmpObjS[221] + tmpFu[29]*tmpObjS[238] + tmpFu[31]*tmpObjS[255] + tmpFu[33]*tmpObjS[272];
tmpR2[18] = + tmpFu[1]*tmpObjS[1] + tmpFu[3]*tmpObjS[18] + tmpFu[5]*tmpObjS[35] + tmpFu[7]*tmpObjS[52] + tmpFu[9]*tmpObjS[69] + tmpFu[11]*tmpObjS[86] + tmpFu[13]*tmpObjS[103] + tmpFu[15]*tmpObjS[120] + tmpFu[17]*tmpObjS[137] + tmpFu[19]*tmpObjS[154] + tmpFu[21]*tmpObjS[171] + tmpFu[23]*tmpObjS[188] + tmpFu[25]*tmpObjS[205] + tmpFu[27]*tmpObjS[222] + tmpFu[29]*tmpObjS[239] + tmpFu[31]*tmpObjS[256] + tmpFu[33]*tmpObjS[273];
tmpR2[19] = + tmpFu[1]*tmpObjS[2] + tmpFu[3]*tmpObjS[19] + tmpFu[5]*tmpObjS[36] + tmpFu[7]*tmpObjS[53] + tmpFu[9]*tmpObjS[70] + tmpFu[11]*tmpObjS[87] + tmpFu[13]*tmpObjS[104] + tmpFu[15]*tmpObjS[121] + tmpFu[17]*tmpObjS[138] + tmpFu[19]*tmpObjS[155] + tmpFu[21]*tmpObjS[172] + tmpFu[23]*tmpObjS[189] + tmpFu[25]*tmpObjS[206] + tmpFu[27]*tmpObjS[223] + tmpFu[29]*tmpObjS[240] + tmpFu[31]*tmpObjS[257] + tmpFu[33]*tmpObjS[274];
tmpR2[20] = + tmpFu[1]*tmpObjS[3] + tmpFu[3]*tmpObjS[20] + tmpFu[5]*tmpObjS[37] + tmpFu[7]*tmpObjS[54] + tmpFu[9]*tmpObjS[71] + tmpFu[11]*tmpObjS[88] + tmpFu[13]*tmpObjS[105] + tmpFu[15]*tmpObjS[122] + tmpFu[17]*tmpObjS[139] + tmpFu[19]*tmpObjS[156] + tmpFu[21]*tmpObjS[173] + tmpFu[23]*tmpObjS[190] + tmpFu[25]*tmpObjS[207] + tmpFu[27]*tmpObjS[224] + tmpFu[29]*tmpObjS[241] + tmpFu[31]*tmpObjS[258] + tmpFu[33]*tmpObjS[275];
tmpR2[21] = + tmpFu[1]*tmpObjS[4] + tmpFu[3]*tmpObjS[21] + tmpFu[5]*tmpObjS[38] + tmpFu[7]*tmpObjS[55] + tmpFu[9]*tmpObjS[72] + tmpFu[11]*tmpObjS[89] + tmpFu[13]*tmpObjS[106] + tmpFu[15]*tmpObjS[123] + tmpFu[17]*tmpObjS[140] + tmpFu[19]*tmpObjS[157] + tmpFu[21]*tmpObjS[174] + tmpFu[23]*tmpObjS[191] + tmpFu[25]*tmpObjS[208] + tmpFu[27]*tmpObjS[225] + tmpFu[29]*tmpObjS[242] + tmpFu[31]*tmpObjS[259] + tmpFu[33]*tmpObjS[276];
tmpR2[22] = + tmpFu[1]*tmpObjS[5] + tmpFu[3]*tmpObjS[22] + tmpFu[5]*tmpObjS[39] + tmpFu[7]*tmpObjS[56] + tmpFu[9]*tmpObjS[73] + tmpFu[11]*tmpObjS[90] + tmpFu[13]*tmpObjS[107] + tmpFu[15]*tmpObjS[124] + tmpFu[17]*tmpObjS[141] + tmpFu[19]*tmpObjS[158] + tmpFu[21]*tmpObjS[175] + tmpFu[23]*tmpObjS[192] + tmpFu[25]*tmpObjS[209] + tmpFu[27]*tmpObjS[226] + tmpFu[29]*tmpObjS[243] + tmpFu[31]*tmpObjS[260] + tmpFu[33]*tmpObjS[277];
tmpR2[23] = + tmpFu[1]*tmpObjS[6] + tmpFu[3]*tmpObjS[23] + tmpFu[5]*tmpObjS[40] + tmpFu[7]*tmpObjS[57] + tmpFu[9]*tmpObjS[74] + tmpFu[11]*tmpObjS[91] + tmpFu[13]*tmpObjS[108] + tmpFu[15]*tmpObjS[125] + tmpFu[17]*tmpObjS[142] + tmpFu[19]*tmpObjS[159] + tmpFu[21]*tmpObjS[176] + tmpFu[23]*tmpObjS[193] + tmpFu[25]*tmpObjS[210] + tmpFu[27]*tmpObjS[227] + tmpFu[29]*tmpObjS[244] + tmpFu[31]*tmpObjS[261] + tmpFu[33]*tmpObjS[278];
tmpR2[24] = + tmpFu[1]*tmpObjS[7] + tmpFu[3]*tmpObjS[24] + tmpFu[5]*tmpObjS[41] + tmpFu[7]*tmpObjS[58] + tmpFu[9]*tmpObjS[75] + tmpFu[11]*tmpObjS[92] + tmpFu[13]*tmpObjS[109] + tmpFu[15]*tmpObjS[126] + tmpFu[17]*tmpObjS[143] + tmpFu[19]*tmpObjS[160] + tmpFu[21]*tmpObjS[177] + tmpFu[23]*tmpObjS[194] + tmpFu[25]*tmpObjS[211] + tmpFu[27]*tmpObjS[228] + tmpFu[29]*tmpObjS[245] + tmpFu[31]*tmpObjS[262] + tmpFu[33]*tmpObjS[279];
tmpR2[25] = + tmpFu[1]*tmpObjS[8] + tmpFu[3]*tmpObjS[25] + tmpFu[5]*tmpObjS[42] + tmpFu[7]*tmpObjS[59] + tmpFu[9]*tmpObjS[76] + tmpFu[11]*tmpObjS[93] + tmpFu[13]*tmpObjS[110] + tmpFu[15]*tmpObjS[127] + tmpFu[17]*tmpObjS[144] + tmpFu[19]*tmpObjS[161] + tmpFu[21]*tmpObjS[178] + tmpFu[23]*tmpObjS[195] + tmpFu[25]*tmpObjS[212] + tmpFu[27]*tmpObjS[229] + tmpFu[29]*tmpObjS[246] + tmpFu[31]*tmpObjS[263] + tmpFu[33]*tmpObjS[280];
tmpR2[26] = + tmpFu[1]*tmpObjS[9] + tmpFu[3]*tmpObjS[26] + tmpFu[5]*tmpObjS[43] + tmpFu[7]*tmpObjS[60] + tmpFu[9]*tmpObjS[77] + tmpFu[11]*tmpObjS[94] + tmpFu[13]*tmpObjS[111] + tmpFu[15]*tmpObjS[128] + tmpFu[17]*tmpObjS[145] + tmpFu[19]*tmpObjS[162] + tmpFu[21]*tmpObjS[179] + tmpFu[23]*tmpObjS[196] + tmpFu[25]*tmpObjS[213] + tmpFu[27]*tmpObjS[230] + tmpFu[29]*tmpObjS[247] + tmpFu[31]*tmpObjS[264] + tmpFu[33]*tmpObjS[281];
tmpR2[27] = + tmpFu[1]*tmpObjS[10] + tmpFu[3]*tmpObjS[27] + tmpFu[5]*tmpObjS[44] + tmpFu[7]*tmpObjS[61] + tmpFu[9]*tmpObjS[78] + tmpFu[11]*tmpObjS[95] + tmpFu[13]*tmpObjS[112] + tmpFu[15]*tmpObjS[129] + tmpFu[17]*tmpObjS[146] + tmpFu[19]*tmpObjS[163] + tmpFu[21]*tmpObjS[180] + tmpFu[23]*tmpObjS[197] + tmpFu[25]*tmpObjS[214] + tmpFu[27]*tmpObjS[231] + tmpFu[29]*tmpObjS[248] + tmpFu[31]*tmpObjS[265] + tmpFu[33]*tmpObjS[282];
tmpR2[28] = + tmpFu[1]*tmpObjS[11] + tmpFu[3]*tmpObjS[28] + tmpFu[5]*tmpObjS[45] + tmpFu[7]*tmpObjS[62] + tmpFu[9]*tmpObjS[79] + tmpFu[11]*tmpObjS[96] + tmpFu[13]*tmpObjS[113] + tmpFu[15]*tmpObjS[130] + tmpFu[17]*tmpObjS[147] + tmpFu[19]*tmpObjS[164] + tmpFu[21]*tmpObjS[181] + tmpFu[23]*tmpObjS[198] + tmpFu[25]*tmpObjS[215] + tmpFu[27]*tmpObjS[232] + tmpFu[29]*tmpObjS[249] + tmpFu[31]*tmpObjS[266] + tmpFu[33]*tmpObjS[283];
tmpR2[29] = + tmpFu[1]*tmpObjS[12] + tmpFu[3]*tmpObjS[29] + tmpFu[5]*tmpObjS[46] + tmpFu[7]*tmpObjS[63] + tmpFu[9]*tmpObjS[80] + tmpFu[11]*tmpObjS[97] + tmpFu[13]*tmpObjS[114] + tmpFu[15]*tmpObjS[131] + tmpFu[17]*tmpObjS[148] + tmpFu[19]*tmpObjS[165] + tmpFu[21]*tmpObjS[182] + tmpFu[23]*tmpObjS[199] + tmpFu[25]*tmpObjS[216] + tmpFu[27]*tmpObjS[233] + tmpFu[29]*tmpObjS[250] + tmpFu[31]*tmpObjS[267] + tmpFu[33]*tmpObjS[284];
tmpR2[30] = + tmpFu[1]*tmpObjS[13] + tmpFu[3]*tmpObjS[30] + tmpFu[5]*tmpObjS[47] + tmpFu[7]*tmpObjS[64] + tmpFu[9]*tmpObjS[81] + tmpFu[11]*tmpObjS[98] + tmpFu[13]*tmpObjS[115] + tmpFu[15]*tmpObjS[132] + tmpFu[17]*tmpObjS[149] + tmpFu[19]*tmpObjS[166] + tmpFu[21]*tmpObjS[183] + tmpFu[23]*tmpObjS[200] + tmpFu[25]*tmpObjS[217] + tmpFu[27]*tmpObjS[234] + tmpFu[29]*tmpObjS[251] + tmpFu[31]*tmpObjS[268] + tmpFu[33]*tmpObjS[285];
tmpR2[31] = + tmpFu[1]*tmpObjS[14] + tmpFu[3]*tmpObjS[31] + tmpFu[5]*tmpObjS[48] + tmpFu[7]*tmpObjS[65] + tmpFu[9]*tmpObjS[82] + tmpFu[11]*tmpObjS[99] + tmpFu[13]*tmpObjS[116] + tmpFu[15]*tmpObjS[133] + tmpFu[17]*tmpObjS[150] + tmpFu[19]*tmpObjS[167] + tmpFu[21]*tmpObjS[184] + tmpFu[23]*tmpObjS[201] + tmpFu[25]*tmpObjS[218] + tmpFu[27]*tmpObjS[235] + tmpFu[29]*tmpObjS[252] + tmpFu[31]*tmpObjS[269] + tmpFu[33]*tmpObjS[286];
tmpR2[32] = + tmpFu[1]*tmpObjS[15] + tmpFu[3]*tmpObjS[32] + tmpFu[5]*tmpObjS[49] + tmpFu[7]*tmpObjS[66] + tmpFu[9]*tmpObjS[83] + tmpFu[11]*tmpObjS[100] + tmpFu[13]*tmpObjS[117] + tmpFu[15]*tmpObjS[134] + tmpFu[17]*tmpObjS[151] + tmpFu[19]*tmpObjS[168] + tmpFu[21]*tmpObjS[185] + tmpFu[23]*tmpObjS[202] + tmpFu[25]*tmpObjS[219] + tmpFu[27]*tmpObjS[236] + tmpFu[29]*tmpObjS[253] + tmpFu[31]*tmpObjS[270] + tmpFu[33]*tmpObjS[287];
tmpR2[33] = + tmpFu[1]*tmpObjS[16] + tmpFu[3]*tmpObjS[33] + tmpFu[5]*tmpObjS[50] + tmpFu[7]*tmpObjS[67] + tmpFu[9]*tmpObjS[84] + tmpFu[11]*tmpObjS[101] + tmpFu[13]*tmpObjS[118] + tmpFu[15]*tmpObjS[135] + tmpFu[17]*tmpObjS[152] + tmpFu[19]*tmpObjS[169] + tmpFu[21]*tmpObjS[186] + tmpFu[23]*tmpObjS[203] + tmpFu[25]*tmpObjS[220] + tmpFu[27]*tmpObjS[237] + tmpFu[29]*tmpObjS[254] + tmpFu[31]*tmpObjS[271] + tmpFu[33]*tmpObjS[288];
tmpR1[0] = + tmpR2[0]*tmpFu[0] + tmpR2[1]*tmpFu[2] + tmpR2[2]*tmpFu[4] + tmpR2[3]*tmpFu[6] + tmpR2[4]*tmpFu[8] + tmpR2[5]*tmpFu[10] + tmpR2[6]*tmpFu[12] + tmpR2[7]*tmpFu[14] + tmpR2[8]*tmpFu[16] + tmpR2[9]*tmpFu[18] + tmpR2[10]*tmpFu[20] + tmpR2[11]*tmpFu[22] + tmpR2[12]*tmpFu[24] + tmpR2[13]*tmpFu[26] + tmpR2[14]*tmpFu[28] + tmpR2[15]*tmpFu[30] + tmpR2[16]*tmpFu[32];
tmpR1[1] = + tmpR2[0]*tmpFu[1] + tmpR2[1]*tmpFu[3] + tmpR2[2]*tmpFu[5] + tmpR2[3]*tmpFu[7] + tmpR2[4]*tmpFu[9] + tmpR2[5]*tmpFu[11] + tmpR2[6]*tmpFu[13] + tmpR2[7]*tmpFu[15] + tmpR2[8]*tmpFu[17] + tmpR2[9]*tmpFu[19] + tmpR2[10]*tmpFu[21] + tmpR2[11]*tmpFu[23] + tmpR2[12]*tmpFu[25] + tmpR2[13]*tmpFu[27] + tmpR2[14]*tmpFu[29] + tmpR2[15]*tmpFu[31] + tmpR2[16]*tmpFu[33];
tmpR1[2] = + tmpR2[17]*tmpFu[0] + tmpR2[18]*tmpFu[2] + tmpR2[19]*tmpFu[4] + tmpR2[20]*tmpFu[6] + tmpR2[21]*tmpFu[8] + tmpR2[22]*tmpFu[10] + tmpR2[23]*tmpFu[12] + tmpR2[24]*tmpFu[14] + tmpR2[25]*tmpFu[16] + tmpR2[26]*tmpFu[18] + tmpR2[27]*tmpFu[20] + tmpR2[28]*tmpFu[22] + tmpR2[29]*tmpFu[24] + tmpR2[30]*tmpFu[26] + tmpR2[31]*tmpFu[28] + tmpR2[32]*tmpFu[30] + tmpR2[33]*tmpFu[32];
tmpR1[3] = + tmpR2[17]*tmpFu[1] + tmpR2[18]*tmpFu[3] + tmpR2[19]*tmpFu[5] + tmpR2[20]*tmpFu[7] + tmpR2[21]*tmpFu[9] + tmpR2[22]*tmpFu[11] + tmpR2[23]*tmpFu[13] + tmpR2[24]*tmpFu[15] + tmpR2[25]*tmpFu[17] + tmpR2[26]*tmpFu[19] + tmpR2[27]*tmpFu[21] + tmpR2[28]*tmpFu[23] + tmpR2[29]*tmpFu[25] + tmpR2[30]*tmpFu[27] + tmpR2[31]*tmpFu[29] + tmpR2[32]*tmpFu[31] + tmpR2[33]*tmpFu[33];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = 0.0;
;
tmpQN2[10] = 0.0;
;
tmpQN2[11] = 0.0;
;
tmpQN2[12] = 0.0;
;
tmpQN2[13] = 0.0;
;
tmpQN2[14] = 0.0;
;
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = 0.0;
;
tmpQN1[4] = 0.0;
;
tmpQN1[5] = + tmpQN2[3];
tmpQN1[6] = + tmpQN2[4];
tmpQN1[7] = + tmpQN2[5];
tmpQN1[8] = 0.0;
;
tmpQN1[9] = 0.0;
;
tmpQN1[10] = + tmpQN2[6];
tmpQN1[11] = + tmpQN2[7];
tmpQN1[12] = + tmpQN2[8];
tmpQN1[13] = 0.0;
;
tmpQN1[14] = 0.0;
;
tmpQN1[15] = + tmpQN2[9];
tmpQN1[16] = + tmpQN2[10];
tmpQN1[17] = + tmpQN2[11];
tmpQN1[18] = 0.0;
;
tmpQN1[19] = 0.0;
;
tmpQN1[20] = + tmpQN2[12];
tmpQN1[21] = + tmpQN2[13];
tmpQN1[22] = + tmpQN2[14];
tmpQN1[23] = 0.0;
;
tmpQN1[24] = 0.0;
;
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 40];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 40 + 1];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 40 + 2];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj * 40 + 3];
acadoWorkspace.objValueIn[11] = acadoVariables.od[runObj * 40 + 4];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 40 + 5];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 40 + 6];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 40 + 7];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 40 + 8];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 40 + 9];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 40 + 10];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 40 + 11];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 40 + 12];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 40 + 13];
acadoWorkspace.objValueIn[21] = acadoVariables.od[runObj * 40 + 14];
acadoWorkspace.objValueIn[22] = acadoVariables.od[runObj * 40 + 15];
acadoWorkspace.objValueIn[23] = acadoVariables.od[runObj * 40 + 16];
acadoWorkspace.objValueIn[24] = acadoVariables.od[runObj * 40 + 17];
acadoWorkspace.objValueIn[25] = acadoVariables.od[runObj * 40 + 18];
acadoWorkspace.objValueIn[26] = acadoVariables.od[runObj * 40 + 19];
acadoWorkspace.objValueIn[27] = acadoVariables.od[runObj * 40 + 20];
acadoWorkspace.objValueIn[28] = acadoVariables.od[runObj * 40 + 21];
acadoWorkspace.objValueIn[29] = acadoVariables.od[runObj * 40 + 22];
acadoWorkspace.objValueIn[30] = acadoVariables.od[runObj * 40 + 23];
acadoWorkspace.objValueIn[31] = acadoVariables.od[runObj * 40 + 24];
acadoWorkspace.objValueIn[32] = acadoVariables.od[runObj * 40 + 25];
acadoWorkspace.objValueIn[33] = acadoVariables.od[runObj * 40 + 26];
acadoWorkspace.objValueIn[34] = acadoVariables.od[runObj * 40 + 27];
acadoWorkspace.objValueIn[35] = acadoVariables.od[runObj * 40 + 28];
acadoWorkspace.objValueIn[36] = acadoVariables.od[runObj * 40 + 29];
acadoWorkspace.objValueIn[37] = acadoVariables.od[runObj * 40 + 30];
acadoWorkspace.objValueIn[38] = acadoVariables.od[runObj * 40 + 31];
acadoWorkspace.objValueIn[39] = acadoVariables.od[runObj * 40 + 32];
acadoWorkspace.objValueIn[40] = acadoVariables.od[runObj * 40 + 33];
acadoWorkspace.objValueIn[41] = acadoVariables.od[runObj * 40 + 34];
acadoWorkspace.objValueIn[42] = acadoVariables.od[runObj * 40 + 35];
acadoWorkspace.objValueIn[43] = acadoVariables.od[runObj * 40 + 36];
acadoWorkspace.objValueIn[44] = acadoVariables.od[runObj * 40 + 37];
acadoWorkspace.objValueIn[45] = acadoVariables.od[runObj * 40 + 38];
acadoWorkspace.objValueIn[46] = acadoVariables.od[runObj * 40 + 39];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 17] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 17 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 17 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 17 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 17 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 17 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 17 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 17 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 17 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 17 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 17 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 17 + 11] = acadoWorkspace.objValueOut[11];
acadoWorkspace.Dy[runObj * 17 + 12] = acadoWorkspace.objValueOut[12];
acadoWorkspace.Dy[runObj * 17 + 13] = acadoWorkspace.objValueOut[13];
acadoWorkspace.Dy[runObj * 17 + 14] = acadoWorkspace.objValueOut[14];
acadoWorkspace.Dy[runObj * 17 + 15] = acadoWorkspace.objValueOut[15];
acadoWorkspace.Dy[runObj * 17 + 16] = acadoWorkspace.objValueOut[16];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 17 ]), &(acadoVariables.W[ runObj * 289 ]), &(acadoWorkspace.Q1[ runObj * 25 ]), &(acadoWorkspace.Q2[ runObj * 85 ]) );

acado_setObjR1R2( &(acadoWorkspace.objValueOut[ 102 ]), &(acadoVariables.W[ runObj * 289 ]), &(acadoWorkspace.R1[ runObj * 4 ]), &(acadoWorkspace.R2[ runObj * 34 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.x[153];
acadoWorkspace.objValueIn[4] = acadoVariables.x[154];
acadoWorkspace.objValueIn[5] = acadoVariables.od[1200];
acadoWorkspace.objValueIn[6] = acadoVariables.od[1201];
acadoWorkspace.objValueIn[7] = acadoVariables.od[1202];
acadoWorkspace.objValueIn[8] = acadoVariables.od[1203];
acadoWorkspace.objValueIn[9] = acadoVariables.od[1204];
acadoWorkspace.objValueIn[10] = acadoVariables.od[1205];
acadoWorkspace.objValueIn[11] = acadoVariables.od[1206];
acadoWorkspace.objValueIn[12] = acadoVariables.od[1207];
acadoWorkspace.objValueIn[13] = acadoVariables.od[1208];
acadoWorkspace.objValueIn[14] = acadoVariables.od[1209];
acadoWorkspace.objValueIn[15] = acadoVariables.od[1210];
acadoWorkspace.objValueIn[16] = acadoVariables.od[1211];
acadoWorkspace.objValueIn[17] = acadoVariables.od[1212];
acadoWorkspace.objValueIn[18] = acadoVariables.od[1213];
acadoWorkspace.objValueIn[19] = acadoVariables.od[1214];
acadoWorkspace.objValueIn[20] = acadoVariables.od[1215];
acadoWorkspace.objValueIn[21] = acadoVariables.od[1216];
acadoWorkspace.objValueIn[22] = acadoVariables.od[1217];
acadoWorkspace.objValueIn[23] = acadoVariables.od[1218];
acadoWorkspace.objValueIn[24] = acadoVariables.od[1219];
acadoWorkspace.objValueIn[25] = acadoVariables.od[1220];
acadoWorkspace.objValueIn[26] = acadoVariables.od[1221];
acadoWorkspace.objValueIn[27] = acadoVariables.od[1222];
acadoWorkspace.objValueIn[28] = acadoVariables.od[1223];
acadoWorkspace.objValueIn[29] = acadoVariables.od[1224];
acadoWorkspace.objValueIn[30] = acadoVariables.od[1225];
acadoWorkspace.objValueIn[31] = acadoVariables.od[1226];
acadoWorkspace.objValueIn[32] = acadoVariables.od[1227];
acadoWorkspace.objValueIn[33] = acadoVariables.od[1228];
acadoWorkspace.objValueIn[34] = acadoVariables.od[1229];
acadoWorkspace.objValueIn[35] = acadoVariables.od[1230];
acadoWorkspace.objValueIn[36] = acadoVariables.od[1231];
acadoWorkspace.objValueIn[37] = acadoVariables.od[1232];
acadoWorkspace.objValueIn[38] = acadoVariables.od[1233];
acadoWorkspace.objValueIn[39] = acadoVariables.od[1234];
acadoWorkspace.objValueIn[40] = acadoVariables.od[1235];
acadoWorkspace.objValueIn[41] = acadoVariables.od[1236];
acadoWorkspace.objValueIn[42] = acadoVariables.od[1237];
acadoWorkspace.objValueIn[43] = acadoVariables.od[1238];
acadoWorkspace.objValueIn[44] = acadoVariables.od[1239];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4];
dNew[1] += + Gx1[5]*dOld[0] + Gx1[6]*dOld[1] + Gx1[7]*dOld[2] + Gx1[8]*dOld[3] + Gx1[9]*dOld[4];
dNew[2] += + Gx1[10]*dOld[0] + Gx1[11]*dOld[1] + Gx1[12]*dOld[2] + Gx1[13]*dOld[3] + Gx1[14]*dOld[4];
dNew[3] += + Gx1[15]*dOld[0] + Gx1[16]*dOld[1] + Gx1[17]*dOld[2] + Gx1[18]*dOld[3] + Gx1[19]*dOld[4];
dNew[4] += + Gx1[20]*dOld[0] + Gx1[21]*dOld[1] + Gx1[22]*dOld[2] + Gx1[23]*dOld[3] + Gx1[24]*dOld[4];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[15] + Gx1[4]*Gx2[20];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[16] + Gx1[4]*Gx2[21];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[12] + Gx1[3]*Gx2[17] + Gx1[4]*Gx2[22];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[13] + Gx1[3]*Gx2[18] + Gx1[4]*Gx2[23];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[19] + Gx1[4]*Gx2[24];
Gx3[5] = + Gx1[5]*Gx2[0] + Gx1[6]*Gx2[5] + Gx1[7]*Gx2[10] + Gx1[8]*Gx2[15] + Gx1[9]*Gx2[20];
Gx3[6] = + Gx1[5]*Gx2[1] + Gx1[6]*Gx2[6] + Gx1[7]*Gx2[11] + Gx1[8]*Gx2[16] + Gx1[9]*Gx2[21];
Gx3[7] = + Gx1[5]*Gx2[2] + Gx1[6]*Gx2[7] + Gx1[7]*Gx2[12] + Gx1[8]*Gx2[17] + Gx1[9]*Gx2[22];
Gx3[8] = + Gx1[5]*Gx2[3] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[13] + Gx1[8]*Gx2[18] + Gx1[9]*Gx2[23];
Gx3[9] = + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[14] + Gx1[8]*Gx2[19] + Gx1[9]*Gx2[24];
Gx3[10] = + Gx1[10]*Gx2[0] + Gx1[11]*Gx2[5] + Gx1[12]*Gx2[10] + Gx1[13]*Gx2[15] + Gx1[14]*Gx2[20];
Gx3[11] = + Gx1[10]*Gx2[1] + Gx1[11]*Gx2[6] + Gx1[12]*Gx2[11] + Gx1[13]*Gx2[16] + Gx1[14]*Gx2[21];
Gx3[12] = + Gx1[10]*Gx2[2] + Gx1[11]*Gx2[7] + Gx1[12]*Gx2[12] + Gx1[13]*Gx2[17] + Gx1[14]*Gx2[22];
Gx3[13] = + Gx1[10]*Gx2[3] + Gx1[11]*Gx2[8] + Gx1[12]*Gx2[13] + Gx1[13]*Gx2[18] + Gx1[14]*Gx2[23];
Gx3[14] = + Gx1[10]*Gx2[4] + Gx1[11]*Gx2[9] + Gx1[12]*Gx2[14] + Gx1[13]*Gx2[19] + Gx1[14]*Gx2[24];
Gx3[15] = + Gx1[15]*Gx2[0] + Gx1[16]*Gx2[5] + Gx1[17]*Gx2[10] + Gx1[18]*Gx2[15] + Gx1[19]*Gx2[20];
Gx3[16] = + Gx1[15]*Gx2[1] + Gx1[16]*Gx2[6] + Gx1[17]*Gx2[11] + Gx1[18]*Gx2[16] + Gx1[19]*Gx2[21];
Gx3[17] = + Gx1[15]*Gx2[2] + Gx1[16]*Gx2[7] + Gx1[17]*Gx2[12] + Gx1[18]*Gx2[17] + Gx1[19]*Gx2[22];
Gx3[18] = + Gx1[15]*Gx2[3] + Gx1[16]*Gx2[8] + Gx1[17]*Gx2[13] + Gx1[18]*Gx2[18] + Gx1[19]*Gx2[23];
Gx3[19] = + Gx1[15]*Gx2[4] + Gx1[16]*Gx2[9] + Gx1[17]*Gx2[14] + Gx1[18]*Gx2[19] + Gx1[19]*Gx2[24];
Gx3[20] = + Gx1[20]*Gx2[0] + Gx1[21]*Gx2[5] + Gx1[22]*Gx2[10] + Gx1[23]*Gx2[15] + Gx1[24]*Gx2[20];
Gx3[21] = + Gx1[20]*Gx2[1] + Gx1[21]*Gx2[6] + Gx1[22]*Gx2[11] + Gx1[23]*Gx2[16] + Gx1[24]*Gx2[21];
Gx3[22] = + Gx1[20]*Gx2[2] + Gx1[21]*Gx2[7] + Gx1[22]*Gx2[12] + Gx1[23]*Gx2[17] + Gx1[24]*Gx2[22];
Gx3[23] = + Gx1[20]*Gx2[3] + Gx1[21]*Gx2[8] + Gx1[22]*Gx2[13] + Gx1[23]*Gx2[18] + Gx1[24]*Gx2[23];
Gx3[24] = + Gx1[20]*Gx2[4] + Gx1[21]*Gx2[9] + Gx1[22]*Gx2[14] + Gx1[23]*Gx2[19] + Gx1[24]*Gx2[24];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9];
Gu2[2] = + Gx1[5]*Gu1[0] + Gx1[6]*Gu1[2] + Gx1[7]*Gu1[4] + Gx1[8]*Gu1[6] + Gx1[9]*Gu1[8];
Gu2[3] = + Gx1[5]*Gu1[1] + Gx1[6]*Gu1[3] + Gx1[7]*Gu1[5] + Gx1[8]*Gu1[7] + Gx1[9]*Gu1[9];
Gu2[4] = + Gx1[10]*Gu1[0] + Gx1[11]*Gu1[2] + Gx1[12]*Gu1[4] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[8];
Gu2[5] = + Gx1[10]*Gu1[1] + Gx1[11]*Gu1[3] + Gx1[12]*Gu1[5] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[9];
Gu2[6] = + Gx1[15]*Gu1[0] + Gx1[16]*Gu1[2] + Gx1[17]*Gu1[4] + Gx1[18]*Gu1[6] + Gx1[19]*Gu1[8];
Gu2[7] = + Gx1[15]*Gu1[1] + Gx1[16]*Gu1[3] + Gx1[17]*Gu1[5] + Gx1[18]*Gu1[7] + Gx1[19]*Gu1[9];
Gu2[8] = + Gx1[20]*Gu1[0] + Gx1[21]*Gu1[2] + Gx1[22]*Gu1[4] + Gx1[23]*Gu1[6] + Gx1[24]*Gu1[8];
Gu2[9] = + Gx1[20]*Gu1[1] + Gx1[21]*Gu1[3] + Gx1[22]*Gu1[5] + Gx1[23]*Gu1[7] + Gx1[24]*Gu1[9];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8];
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = R11[0];
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = R11[2];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = R11[3];
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 120) + (iCol * 2)] = acadoWorkspace.H[(iCol * 120) + (iRow * 2)];
acadoWorkspace.H[(iRow * 120) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 120 + 60) + (iRow * 2)];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2)] = acadoWorkspace.H[(iCol * 120) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 120 + 60) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 120 + 60) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4];
dNew[1] = + Gx1[5]*dOld[0] + Gx1[6]*dOld[1] + Gx1[7]*dOld[2] + Gx1[8]*dOld[3] + Gx1[9]*dOld[4];
dNew[2] = + Gx1[10]*dOld[0] + Gx1[11]*dOld[1] + Gx1[12]*dOld[2] + Gx1[13]*dOld[3] + Gx1[14]*dOld[4];
dNew[3] = + Gx1[15]*dOld[0] + Gx1[16]*dOld[1] + Gx1[17]*dOld[2] + Gx1[18]*dOld[3] + Gx1[19]*dOld[4];
dNew[4] = + Gx1[20]*dOld[0] + Gx1[21]*dOld[1] + Gx1[22]*dOld[2] + Gx1[23]*dOld[3] + Gx1[24]*dOld[4];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2] + acadoWorkspace.QN1[3]*dOld[3] + acadoWorkspace.QN1[4]*dOld[4];
dNew[1] = + acadoWorkspace.QN1[5]*dOld[0] + acadoWorkspace.QN1[6]*dOld[1] + acadoWorkspace.QN1[7]*dOld[2] + acadoWorkspace.QN1[8]*dOld[3] + acadoWorkspace.QN1[9]*dOld[4];
dNew[2] = + acadoWorkspace.QN1[10]*dOld[0] + acadoWorkspace.QN1[11]*dOld[1] + acadoWorkspace.QN1[12]*dOld[2] + acadoWorkspace.QN1[13]*dOld[3] + acadoWorkspace.QN1[14]*dOld[4];
dNew[3] = + acadoWorkspace.QN1[15]*dOld[0] + acadoWorkspace.QN1[16]*dOld[1] + acadoWorkspace.QN1[17]*dOld[2] + acadoWorkspace.QN1[18]*dOld[3] + acadoWorkspace.QN1[19]*dOld[4];
dNew[4] = + acadoWorkspace.QN1[20]*dOld[0] + acadoWorkspace.QN1[21]*dOld[1] + acadoWorkspace.QN1[22]*dOld[2] + acadoWorkspace.QN1[23]*dOld[3] + acadoWorkspace.QN1[24]*dOld[4];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11] + R2[12]*Dy1[12] + R2[13]*Dy1[13] + R2[14]*Dy1[14] + R2[15]*Dy1[15] + R2[16]*Dy1[16];
RDy1[1] = + R2[17]*Dy1[0] + R2[18]*Dy1[1] + R2[19]*Dy1[2] + R2[20]*Dy1[3] + R2[21]*Dy1[4] + R2[22]*Dy1[5] + R2[23]*Dy1[6] + R2[24]*Dy1[7] + R2[25]*Dy1[8] + R2[26]*Dy1[9] + R2[27]*Dy1[10] + R2[28]*Dy1[11] + R2[29]*Dy1[12] + R2[30]*Dy1[13] + R2[31]*Dy1[14] + R2[32]*Dy1[15] + R2[33]*Dy1[16];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11] + Q2[12]*Dy1[12] + Q2[13]*Dy1[13] + Q2[14]*Dy1[14] + Q2[15]*Dy1[15] + Q2[16]*Dy1[16];
QDy1[1] = + Q2[17]*Dy1[0] + Q2[18]*Dy1[1] + Q2[19]*Dy1[2] + Q2[20]*Dy1[3] + Q2[21]*Dy1[4] + Q2[22]*Dy1[5] + Q2[23]*Dy1[6] + Q2[24]*Dy1[7] + Q2[25]*Dy1[8] + Q2[26]*Dy1[9] + Q2[27]*Dy1[10] + Q2[28]*Dy1[11] + Q2[29]*Dy1[12] + Q2[30]*Dy1[13] + Q2[31]*Dy1[14] + Q2[32]*Dy1[15] + Q2[33]*Dy1[16];
QDy1[2] = + Q2[34]*Dy1[0] + Q2[35]*Dy1[1] + Q2[36]*Dy1[2] + Q2[37]*Dy1[3] + Q2[38]*Dy1[4] + Q2[39]*Dy1[5] + Q2[40]*Dy1[6] + Q2[41]*Dy1[7] + Q2[42]*Dy1[8] + Q2[43]*Dy1[9] + Q2[44]*Dy1[10] + Q2[45]*Dy1[11] + Q2[46]*Dy1[12] + Q2[47]*Dy1[13] + Q2[48]*Dy1[14] + Q2[49]*Dy1[15] + Q2[50]*Dy1[16];
QDy1[3] = + Q2[51]*Dy1[0] + Q2[52]*Dy1[1] + Q2[53]*Dy1[2] + Q2[54]*Dy1[3] + Q2[55]*Dy1[4] + Q2[56]*Dy1[5] + Q2[57]*Dy1[6] + Q2[58]*Dy1[7] + Q2[59]*Dy1[8] + Q2[60]*Dy1[9] + Q2[61]*Dy1[10] + Q2[62]*Dy1[11] + Q2[63]*Dy1[12] + Q2[64]*Dy1[13] + Q2[65]*Dy1[14] + Q2[66]*Dy1[15] + Q2[67]*Dy1[16];
QDy1[4] = + Q2[68]*Dy1[0] + Q2[69]*Dy1[1] + Q2[70]*Dy1[2] + Q2[71]*Dy1[3] + Q2[72]*Dy1[4] + Q2[73]*Dy1[5] + Q2[74]*Dy1[6] + Q2[75]*Dy1[7] + Q2[76]*Dy1[8] + Q2[77]*Dy1[9] + Q2[78]*Dy1[10] + Q2[79]*Dy1[11] + Q2[80]*Dy1[12] + Q2[81]*Dy1[13] + Q2[82]*Dy1[14] + Q2[83]*Dy1[15] + Q2[84]*Dy1[16];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3] + E1[8]*QDy1[4];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3] + E1[9]*QDy1[4];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[5] + E1[4]*Gx1[10] + E1[6]*Gx1[15] + E1[8]*Gx1[20];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[6] + E1[4]*Gx1[11] + E1[6]*Gx1[16] + E1[8]*Gx1[21];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[7] + E1[4]*Gx1[12] + E1[6]*Gx1[17] + E1[8]*Gx1[22];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[8] + E1[4]*Gx1[13] + E1[6]*Gx1[18] + E1[8]*Gx1[23];
H101[4] += + E1[0]*Gx1[4] + E1[2]*Gx1[9] + E1[4]*Gx1[14] + E1[6]*Gx1[19] + E1[8]*Gx1[24];
H101[5] += + E1[1]*Gx1[0] + E1[3]*Gx1[5] + E1[5]*Gx1[10] + E1[7]*Gx1[15] + E1[9]*Gx1[20];
H101[6] += + E1[1]*Gx1[1] + E1[3]*Gx1[6] + E1[5]*Gx1[11] + E1[7]*Gx1[16] + E1[9]*Gx1[21];
H101[7] += + E1[1]*Gx1[2] + E1[3]*Gx1[7] + E1[5]*Gx1[12] + E1[7]*Gx1[17] + E1[9]*Gx1[22];
H101[8] += + E1[1]*Gx1[3] + E1[3]*Gx1[8] + E1[5]*Gx1[13] + E1[7]*Gx1[18] + E1[9]*Gx1[23];
H101[9] += + E1[1]*Gx1[4] + E1[3]*Gx1[9] + E1[5]*Gx1[14] + E1[7]*Gx1[19] + E1[9]*Gx1[24];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 10; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
dNew[4] += + E1[8]*U1[0] + E1[9]*U1[1];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[5] + Hx[2]*Gx[10] + Hx[3]*Gx[15] + Hx[4]*Gx[20];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[6] + Hx[2]*Gx[11] + Hx[3]*Gx[16] + Hx[4]*Gx[21];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[7] + Hx[2]*Gx[12] + Hx[3]*Gx[17] + Hx[4]*Gx[22];
A01[3] = + Hx[0]*Gx[3] + Hx[1]*Gx[8] + Hx[2]*Gx[13] + Hx[3]*Gx[18] + Hx[4]*Gx[23];
A01[4] = + Hx[0]*Gx[4] + Hx[1]*Gx[9] + Hx[2]*Gx[14] + Hx[3]*Gx[19] + Hx[4]*Gx[24];
A01[5] = + Hx[5]*Gx[0] + Hx[6]*Gx[5] + Hx[7]*Gx[10] + Hx[8]*Gx[15] + Hx[9]*Gx[20];
A01[6] = + Hx[5]*Gx[1] + Hx[6]*Gx[6] + Hx[7]*Gx[11] + Hx[8]*Gx[16] + Hx[9]*Gx[21];
A01[7] = + Hx[5]*Gx[2] + Hx[6]*Gx[7] + Hx[7]*Gx[12] + Hx[8]*Gx[17] + Hx[9]*Gx[22];
A01[8] = + Hx[5]*Gx[3] + Hx[6]*Gx[8] + Hx[7]*Gx[13] + Hx[8]*Gx[18] + Hx[9]*Gx[23];
A01[9] = + Hx[5]*Gx[4] + Hx[6]*Gx[9] + Hx[7]*Gx[14] + Hx[8]*Gx[19] + Hx[9]*Gx[24];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 120 + 3600) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4] + Hx[3]*E[6] + Hx[4]*E[8];
acadoWorkspace.A[(row * 120 + 3600) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5] + Hx[3]*E[7] + Hx[4]*E[9];
acadoWorkspace.A[(row * 120 + 3660) + (col * 2)] = + Hx[5]*E[0] + Hx[6]*E[2] + Hx[7]*E[4] + Hx[8]*E[6] + Hx[9]*E[8];
acadoWorkspace.A[(row * 120 + 3660) + (col * 2 + 1)] = + Hx[5]*E[1] + Hx[6]*E[3] + Hx[7]*E[5] + Hx[8]*E[7] + Hx[9]*E[9];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4];
acadoWorkspace.evHxd[1] = + Hx[5]*tmpd[0] + Hx[6]*tmpd[1] + Hx[7]*tmpd[2] + Hx[8]*tmpd[3] + Hx[9]*tmpd[4];
lbA[0] -= acadoWorkspace.evHxd[0];
lbA[1] -= acadoWorkspace.evHxd[1];
ubA[0] -= acadoWorkspace.evHxd[0];
ubA[1] -= acadoWorkspace.evHxd[1];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 7;
/* Vector of auxiliary variables; number of elements: 26. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = ((xd[0])*(xd[0]));
a[1] = ((xd[1])*(xd[1]));
a[2] = ((xd[0])*(xd[0]));
a[3] = ((xd[1])*(xd[1]));
a[4] = ((real_t)(2.0000000000000000e+00)*xd[0]);
a[5] = (a[4]*od[26]);
a[6] = (a[5]+od[27]);
a[7] = ((real_t)(2.0000000000000000e+00)*xd[1]);
a[8] = (a[7]*od[29]);
a[9] = (a[8]+od[30]);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(0.0000000000000000e+00);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = ((real_t)(2.0000000000000000e+00)*xd[0]);
a[14] = (a[13]*od[33]);
a[15] = (a[14]+od[34]);
a[16] = ((real_t)(2.0000000000000000e+00)*xd[1]);
a[17] = (a[16]*od[36]);
a[18] = (a[17]+od[37]);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = (((((((od[26]*a[0])+(od[27]*xd[0]))+od[28])+(od[29]*a[1]))+(od[30]*xd[1]))+od[31])+od[32]);
out[1] = (((((((od[33]*a[2])+(od[34]*xd[0]))+od[35])+(od[36]*a[3]))+(od[37]*xd[1]))+od[38])+od[39]);
out[2] = a[6];
out[3] = a[9];
out[4] = a[10];
out[5] = a[11];
out[6] = a[12];
out[7] = a[15];
out[8] = a[18];
out[9] = a[19];
out[10] = a[20];
out[11] = a[21];
out[12] = a[22];
out[13] = a[23];
out[14] = a[24];
out[15] = a[25];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 60 */
static const int xBoundIndices[ 60 ] = 
{ 8, 9, 13, 14, 18, 19, 23, 24, 28, 29, 33, 34, 38, 39, 43, 44, 48, 49, 53, 54, 58, 59, 63, 64, 68, 69, 73, 74, 78, 79, 83, 84, 88, 89, 93, 94, 98, 99, 103, 104, 108, 109, 113, 114, 118, 119, 123, 124, 128, 129, 133, 134, 138, 139, 143, 144, 148, 149, 153, 154 };
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
for (lRun1 = 1; lRun1 < 30; ++lRun1)
{
acado_moveGxT( &(acadoWorkspace.evGx[ lRun1 * 25 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ lRun1 * 5-5 ]), &(acadoWorkspace.evGx[ lRun1 * 25 ]), &(acadoWorkspace.d[ lRun1 * 5 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ lRun1 * 25-25 ]), &(acadoWorkspace.evGx[ lRun1 * 25 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ lRun4 * 10 ]), &(acadoWorkspace.E[ lRun3 * 10 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun1 * 10 ]), &(acadoWorkspace.E[ lRun3 * 10 ]) );
}

for (lRun1 = 0; lRun1 < 29; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( &(acadoWorkspace.Q1[ lRun1 * 25 + 25 ]), &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.QE[ lRun3 * 10 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.QE[ lRun3 * 10 ]) );
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 10 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 10 ]), &(acadoWorkspace.evGx[ lRun2 * 25 ]), &(acadoWorkspace.H10[ lRun1 * 10 ]) );
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1, &(acadoWorkspace.R1[ lRun1 * 4 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 10 ]), &(acadoWorkspace.QE[ lRun5 * 10 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 10 ]), &(acadoWorkspace.QE[ lRun5 * 10 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun1, lRun2 );
}
}

acado_multQ1d( &(acadoWorkspace.Q1[ 25 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 50 ]), &(acadoWorkspace.d[ 5 ]), &(acadoWorkspace.Qd[ 5 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 75 ]), &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.Qd[ 10 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.d[ 15 ]), &(acadoWorkspace.Qd[ 15 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.Qd[ 20 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.d[ 25 ]), &(acadoWorkspace.Qd[ 25 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.Qd[ 30 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.d[ 35 ]), &(acadoWorkspace.Qd[ 35 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.Qd[ 40 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 250 ]), &(acadoWorkspace.d[ 45 ]), &(acadoWorkspace.Qd[ 45 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 275 ]), &(acadoWorkspace.d[ 50 ]), &(acadoWorkspace.Qd[ 50 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 300 ]), &(acadoWorkspace.d[ 55 ]), &(acadoWorkspace.Qd[ 55 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 325 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 350 ]), &(acadoWorkspace.d[ 65 ]), &(acadoWorkspace.Qd[ 65 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 375 ]), &(acadoWorkspace.d[ 70 ]), &(acadoWorkspace.Qd[ 70 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.d[ 75 ]), &(acadoWorkspace.Qd[ 75 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 425 ]), &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.Qd[ 80 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 450 ]), &(acadoWorkspace.d[ 85 ]), &(acadoWorkspace.Qd[ 85 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 475 ]), &(acadoWorkspace.d[ 90 ]), &(acadoWorkspace.Qd[ 90 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 500 ]), &(acadoWorkspace.d[ 95 ]), &(acadoWorkspace.Qd[ 95 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 525 ]), &(acadoWorkspace.d[ 100 ]), &(acadoWorkspace.Qd[ 100 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 550 ]), &(acadoWorkspace.d[ 105 ]), &(acadoWorkspace.Qd[ 105 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 575 ]), &(acadoWorkspace.d[ 110 ]), &(acadoWorkspace.Qd[ 110 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 600 ]), &(acadoWorkspace.d[ 115 ]), &(acadoWorkspace.Qd[ 115 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 625 ]), &(acadoWorkspace.d[ 120 ]), &(acadoWorkspace.Qd[ 120 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 650 ]), &(acadoWorkspace.d[ 125 ]), &(acadoWorkspace.Qd[ 125 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 675 ]), &(acadoWorkspace.d[ 130 ]), &(acadoWorkspace.Qd[ 130 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 700 ]), &(acadoWorkspace.d[ 135 ]), &(acadoWorkspace.Qd[ 135 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 725 ]), &(acadoWorkspace.d[ 140 ]), &(acadoWorkspace.Qd[ 140 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 145 ]), &(acadoWorkspace.Qd[ 145 ]) );

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_macETSlu( &(acadoWorkspace.QE[ lRun3 * 10 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}
for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 5;
lRun4 = ((lRun3) / (5)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = (((((lRun4) * (lRun4-1)) / (2)) + (lRun2)) * (5)) + ((lRun3) % (5));
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 2)] = acadoWorkspace.E[lRun5 * 2];
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 2 + 1)] = acadoWorkspace.E[lRun5 * 2 + 1];
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun1 * 5 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.conValueIn[6] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.conValueIn[7] = acadoVariables.od[lRun1 * 40];
acadoWorkspace.conValueIn[8] = acadoVariables.od[lRun1 * 40 + 1];
acadoWorkspace.conValueIn[9] = acadoVariables.od[lRun1 * 40 + 2];
acadoWorkspace.conValueIn[10] = acadoVariables.od[lRun1 * 40 + 3];
acadoWorkspace.conValueIn[11] = acadoVariables.od[lRun1 * 40 + 4];
acadoWorkspace.conValueIn[12] = acadoVariables.od[lRun1 * 40 + 5];
acadoWorkspace.conValueIn[13] = acadoVariables.od[lRun1 * 40 + 6];
acadoWorkspace.conValueIn[14] = acadoVariables.od[lRun1 * 40 + 7];
acadoWorkspace.conValueIn[15] = acadoVariables.od[lRun1 * 40 + 8];
acadoWorkspace.conValueIn[16] = acadoVariables.od[lRun1 * 40 + 9];
acadoWorkspace.conValueIn[17] = acadoVariables.od[lRun1 * 40 + 10];
acadoWorkspace.conValueIn[18] = acadoVariables.od[lRun1 * 40 + 11];
acadoWorkspace.conValueIn[19] = acadoVariables.od[lRun1 * 40 + 12];
acadoWorkspace.conValueIn[20] = acadoVariables.od[lRun1 * 40 + 13];
acadoWorkspace.conValueIn[21] = acadoVariables.od[lRun1 * 40 + 14];
acadoWorkspace.conValueIn[22] = acadoVariables.od[lRun1 * 40 + 15];
acadoWorkspace.conValueIn[23] = acadoVariables.od[lRun1 * 40 + 16];
acadoWorkspace.conValueIn[24] = acadoVariables.od[lRun1 * 40 + 17];
acadoWorkspace.conValueIn[25] = acadoVariables.od[lRun1 * 40 + 18];
acadoWorkspace.conValueIn[26] = acadoVariables.od[lRun1 * 40 + 19];
acadoWorkspace.conValueIn[27] = acadoVariables.od[lRun1 * 40 + 20];
acadoWorkspace.conValueIn[28] = acadoVariables.od[lRun1 * 40 + 21];
acadoWorkspace.conValueIn[29] = acadoVariables.od[lRun1 * 40 + 22];
acadoWorkspace.conValueIn[30] = acadoVariables.od[lRun1 * 40 + 23];
acadoWorkspace.conValueIn[31] = acadoVariables.od[lRun1 * 40 + 24];
acadoWorkspace.conValueIn[32] = acadoVariables.od[lRun1 * 40 + 25];
acadoWorkspace.conValueIn[33] = acadoVariables.od[lRun1 * 40 + 26];
acadoWorkspace.conValueIn[34] = acadoVariables.od[lRun1 * 40 + 27];
acadoWorkspace.conValueIn[35] = acadoVariables.od[lRun1 * 40 + 28];
acadoWorkspace.conValueIn[36] = acadoVariables.od[lRun1 * 40 + 29];
acadoWorkspace.conValueIn[37] = acadoVariables.od[lRun1 * 40 + 30];
acadoWorkspace.conValueIn[38] = acadoVariables.od[lRun1 * 40 + 31];
acadoWorkspace.conValueIn[39] = acadoVariables.od[lRun1 * 40 + 32];
acadoWorkspace.conValueIn[40] = acadoVariables.od[lRun1 * 40 + 33];
acadoWorkspace.conValueIn[41] = acadoVariables.od[lRun1 * 40 + 34];
acadoWorkspace.conValueIn[42] = acadoVariables.od[lRun1 * 40 + 35];
acadoWorkspace.conValueIn[43] = acadoVariables.od[lRun1 * 40 + 36];
acadoWorkspace.conValueIn[44] = acadoVariables.od[lRun1 * 40 + 37];
acadoWorkspace.conValueIn[45] = acadoVariables.od[lRun1 * 40 + 38];
acadoWorkspace.conValueIn[46] = acadoVariables.od[lRun1 * 40 + 39];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 2] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 2 + 1] = acadoWorkspace.conValueOut[1];

acadoWorkspace.evHx[lRun1 * 10] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun1 * 10 + 1] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 10 + 2] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 10 + 3] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 10 + 4] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 10 + 5] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 10 + 6] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 10 + 7] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 10 + 8] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 10 + 9] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHu[lRun1 * 4] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHu[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHu[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHu[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[15];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];
acadoWorkspace.A01[4] = acadoWorkspace.evHx[4];
acadoWorkspace.A01[5] = acadoWorkspace.evHx[5];
acadoWorkspace.A01[6] = acadoWorkspace.evHx[6];
acadoWorkspace.A01[7] = acadoWorkspace.evHx[7];
acadoWorkspace.A01[8] = acadoWorkspace.evHx[8];
acadoWorkspace.A01[9] = acadoWorkspace.evHx[9];

acado_multHxC( &(acadoWorkspace.evHx[ 10 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 10 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 20 ]), &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.A01[ 20 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.A01[ 30 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.A01[ 40 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 50 ]), &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.A01[ 50 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.A01[ 60 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.A01[ 70 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.A01[ 80 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.A01[ 90 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.A01[ 100 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.evGx[ 250 ]), &(acadoWorkspace.A01[ 110 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.evGx[ 275 ]), &(acadoWorkspace.A01[ 120 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.evGx[ 300 ]), &(acadoWorkspace.A01[ 130 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.evGx[ 325 ]), &(acadoWorkspace.A01[ 140 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.evGx[ 350 ]), &(acadoWorkspace.A01[ 150 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.evGx[ 375 ]), &(acadoWorkspace.A01[ 160 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.A01[ 170 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.evGx[ 425 ]), &(acadoWorkspace.A01[ 180 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.evGx[ 450 ]), &(acadoWorkspace.A01[ 190 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 200 ]), &(acadoWorkspace.evGx[ 475 ]), &(acadoWorkspace.A01[ 200 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.evGx[ 500 ]), &(acadoWorkspace.A01[ 210 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 220 ]), &(acadoWorkspace.evGx[ 525 ]), &(acadoWorkspace.A01[ 220 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 230 ]), &(acadoWorkspace.evGx[ 550 ]), &(acadoWorkspace.A01[ 230 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.evGx[ 575 ]), &(acadoWorkspace.A01[ 240 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 250 ]), &(acadoWorkspace.evGx[ 600 ]), &(acadoWorkspace.A01[ 250 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 260 ]), &(acadoWorkspace.evGx[ 625 ]), &(acadoWorkspace.A01[ 260 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.evGx[ 650 ]), &(acadoWorkspace.A01[ 270 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 280 ]), &(acadoWorkspace.evGx[ 675 ]), &(acadoWorkspace.A01[ 280 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 290 ]), &(acadoWorkspace.evGx[ 700 ]), &(acadoWorkspace.A01[ 290 ]) );

for (lRun2 = 0; lRun2 < 29; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun3);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 10 + 10 ]), &(acadoWorkspace.E[ lRun4 * 10 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[3600] = acadoWorkspace.evHu[0];
acadoWorkspace.A[3601] = acadoWorkspace.evHu[1];
acadoWorkspace.A[3660] = acadoWorkspace.evHu[2];
acadoWorkspace.A[3661] = acadoWorkspace.evHu[3];
acadoWorkspace.A[3722] = acadoWorkspace.evHu[4];
acadoWorkspace.A[3723] = acadoWorkspace.evHu[5];
acadoWorkspace.A[3782] = acadoWorkspace.evHu[6];
acadoWorkspace.A[3783] = acadoWorkspace.evHu[7];
acadoWorkspace.A[3844] = acadoWorkspace.evHu[8];
acadoWorkspace.A[3845] = acadoWorkspace.evHu[9];
acadoWorkspace.A[3904] = acadoWorkspace.evHu[10];
acadoWorkspace.A[3905] = acadoWorkspace.evHu[11];
acadoWorkspace.A[3966] = acadoWorkspace.evHu[12];
acadoWorkspace.A[3967] = acadoWorkspace.evHu[13];
acadoWorkspace.A[4026] = acadoWorkspace.evHu[14];
acadoWorkspace.A[4027] = acadoWorkspace.evHu[15];
acadoWorkspace.A[4088] = acadoWorkspace.evHu[16];
acadoWorkspace.A[4089] = acadoWorkspace.evHu[17];
acadoWorkspace.A[4148] = acadoWorkspace.evHu[18];
acadoWorkspace.A[4149] = acadoWorkspace.evHu[19];
acadoWorkspace.A[4210] = acadoWorkspace.evHu[20];
acadoWorkspace.A[4211] = acadoWorkspace.evHu[21];
acadoWorkspace.A[4270] = acadoWorkspace.evHu[22];
acadoWorkspace.A[4271] = acadoWorkspace.evHu[23];
acadoWorkspace.A[4332] = acadoWorkspace.evHu[24];
acadoWorkspace.A[4333] = acadoWorkspace.evHu[25];
acadoWorkspace.A[4392] = acadoWorkspace.evHu[26];
acadoWorkspace.A[4393] = acadoWorkspace.evHu[27];
acadoWorkspace.A[4454] = acadoWorkspace.evHu[28];
acadoWorkspace.A[4455] = acadoWorkspace.evHu[29];
acadoWorkspace.A[4514] = acadoWorkspace.evHu[30];
acadoWorkspace.A[4515] = acadoWorkspace.evHu[31];
acadoWorkspace.A[4576] = acadoWorkspace.evHu[32];
acadoWorkspace.A[4577] = acadoWorkspace.evHu[33];
acadoWorkspace.A[4636] = acadoWorkspace.evHu[34];
acadoWorkspace.A[4637] = acadoWorkspace.evHu[35];
acadoWorkspace.A[4698] = acadoWorkspace.evHu[36];
acadoWorkspace.A[4699] = acadoWorkspace.evHu[37];
acadoWorkspace.A[4758] = acadoWorkspace.evHu[38];
acadoWorkspace.A[4759] = acadoWorkspace.evHu[39];
acadoWorkspace.A[4820] = acadoWorkspace.evHu[40];
acadoWorkspace.A[4821] = acadoWorkspace.evHu[41];
acadoWorkspace.A[4880] = acadoWorkspace.evHu[42];
acadoWorkspace.A[4881] = acadoWorkspace.evHu[43];
acadoWorkspace.A[4942] = acadoWorkspace.evHu[44];
acadoWorkspace.A[4943] = acadoWorkspace.evHu[45];
acadoWorkspace.A[5002] = acadoWorkspace.evHu[46];
acadoWorkspace.A[5003] = acadoWorkspace.evHu[47];
acadoWorkspace.A[5064] = acadoWorkspace.evHu[48];
acadoWorkspace.A[5065] = acadoWorkspace.evHu[49];
acadoWorkspace.A[5124] = acadoWorkspace.evHu[50];
acadoWorkspace.A[5125] = acadoWorkspace.evHu[51];
acadoWorkspace.A[5186] = acadoWorkspace.evHu[52];
acadoWorkspace.A[5187] = acadoWorkspace.evHu[53];
acadoWorkspace.A[5246] = acadoWorkspace.evHu[54];
acadoWorkspace.A[5247] = acadoWorkspace.evHu[55];
acadoWorkspace.A[5308] = acadoWorkspace.evHu[56];
acadoWorkspace.A[5309] = acadoWorkspace.evHu[57];
acadoWorkspace.A[5368] = acadoWorkspace.evHu[58];
acadoWorkspace.A[5369] = acadoWorkspace.evHu[59];
acadoWorkspace.A[5430] = acadoWorkspace.evHu[60];
acadoWorkspace.A[5431] = acadoWorkspace.evHu[61];
acadoWorkspace.A[5490] = acadoWorkspace.evHu[62];
acadoWorkspace.A[5491] = acadoWorkspace.evHu[63];
acadoWorkspace.A[5552] = acadoWorkspace.evHu[64];
acadoWorkspace.A[5553] = acadoWorkspace.evHu[65];
acadoWorkspace.A[5612] = acadoWorkspace.evHu[66];
acadoWorkspace.A[5613] = acadoWorkspace.evHu[67];
acadoWorkspace.A[5674] = acadoWorkspace.evHu[68];
acadoWorkspace.A[5675] = acadoWorkspace.evHu[69];
acadoWorkspace.A[5734] = acadoWorkspace.evHu[70];
acadoWorkspace.A[5735] = acadoWorkspace.evHu[71];
acadoWorkspace.A[5796] = acadoWorkspace.evHu[72];
acadoWorkspace.A[5797] = acadoWorkspace.evHu[73];
acadoWorkspace.A[5856] = acadoWorkspace.evHu[74];
acadoWorkspace.A[5857] = acadoWorkspace.evHu[75];
acadoWorkspace.A[5918] = acadoWorkspace.evHu[76];
acadoWorkspace.A[5919] = acadoWorkspace.evHu[77];
acadoWorkspace.A[5978] = acadoWorkspace.evHu[78];
acadoWorkspace.A[5979] = acadoWorkspace.evHu[79];
acadoWorkspace.A[6040] = acadoWorkspace.evHu[80];
acadoWorkspace.A[6041] = acadoWorkspace.evHu[81];
acadoWorkspace.A[6100] = acadoWorkspace.evHu[82];
acadoWorkspace.A[6101] = acadoWorkspace.evHu[83];
acadoWorkspace.A[6162] = acadoWorkspace.evHu[84];
acadoWorkspace.A[6163] = acadoWorkspace.evHu[85];
acadoWorkspace.A[6222] = acadoWorkspace.evHu[86];
acadoWorkspace.A[6223] = acadoWorkspace.evHu[87];
acadoWorkspace.A[6284] = acadoWorkspace.evHu[88];
acadoWorkspace.A[6285] = acadoWorkspace.evHu[89];
acadoWorkspace.A[6344] = acadoWorkspace.evHu[90];
acadoWorkspace.A[6345] = acadoWorkspace.evHu[91];
acadoWorkspace.A[6406] = acadoWorkspace.evHu[92];
acadoWorkspace.A[6407] = acadoWorkspace.evHu[93];
acadoWorkspace.A[6466] = acadoWorkspace.evHu[94];
acadoWorkspace.A[6467] = acadoWorkspace.evHu[95];
acadoWorkspace.A[6528] = acadoWorkspace.evHu[96];
acadoWorkspace.A[6529] = acadoWorkspace.evHu[97];
acadoWorkspace.A[6588] = acadoWorkspace.evHu[98];
acadoWorkspace.A[6589] = acadoWorkspace.evHu[99];
acadoWorkspace.A[6650] = acadoWorkspace.evHu[100];
acadoWorkspace.A[6651] = acadoWorkspace.evHu[101];
acadoWorkspace.A[6710] = acadoWorkspace.evHu[102];
acadoWorkspace.A[6711] = acadoWorkspace.evHu[103];
acadoWorkspace.A[6772] = acadoWorkspace.evHu[104];
acadoWorkspace.A[6773] = acadoWorkspace.evHu[105];
acadoWorkspace.A[6832] = acadoWorkspace.evHu[106];
acadoWorkspace.A[6833] = acadoWorkspace.evHu[107];
acadoWorkspace.A[6894] = acadoWorkspace.evHu[108];
acadoWorkspace.A[6895] = acadoWorkspace.evHu[109];
acadoWorkspace.A[6954] = acadoWorkspace.evHu[110];
acadoWorkspace.A[6955] = acadoWorkspace.evHu[111];
acadoWorkspace.A[7016] = acadoWorkspace.evHu[112];
acadoWorkspace.A[7017] = acadoWorkspace.evHu[113];
acadoWorkspace.A[7076] = acadoWorkspace.evHu[114];
acadoWorkspace.A[7077] = acadoWorkspace.evHu[115];
acadoWorkspace.A[7138] = acadoWorkspace.evHu[116];
acadoWorkspace.A[7139] = acadoWorkspace.evHu[117];
acadoWorkspace.A[7198] = acadoWorkspace.evHu[118];
acadoWorkspace.A[7199] = acadoWorkspace.evHu[119];
acadoWorkspace.lbA[60] = acadoVariables.lbAValues[60] - acadoWorkspace.evH[0];
acadoWorkspace.lbA[61] = acadoVariables.lbAValues[61] - acadoWorkspace.evH[1];
acadoWorkspace.lbA[62] = acadoVariables.lbAValues[62] - acadoWorkspace.evH[2];
acadoWorkspace.lbA[63] = acadoVariables.lbAValues[63] - acadoWorkspace.evH[3];
acadoWorkspace.lbA[64] = acadoVariables.lbAValues[64] - acadoWorkspace.evH[4];
acadoWorkspace.lbA[65] = acadoVariables.lbAValues[65] - acadoWorkspace.evH[5];
acadoWorkspace.lbA[66] = acadoVariables.lbAValues[66] - acadoWorkspace.evH[6];
acadoWorkspace.lbA[67] = acadoVariables.lbAValues[67] - acadoWorkspace.evH[7];
acadoWorkspace.lbA[68] = acadoVariables.lbAValues[68] - acadoWorkspace.evH[8];
acadoWorkspace.lbA[69] = acadoVariables.lbAValues[69] - acadoWorkspace.evH[9];
acadoWorkspace.lbA[70] = acadoVariables.lbAValues[70] - acadoWorkspace.evH[10];
acadoWorkspace.lbA[71] = acadoVariables.lbAValues[71] - acadoWorkspace.evH[11];
acadoWorkspace.lbA[72] = acadoVariables.lbAValues[72] - acadoWorkspace.evH[12];
acadoWorkspace.lbA[73] = acadoVariables.lbAValues[73] - acadoWorkspace.evH[13];
acadoWorkspace.lbA[74] = acadoVariables.lbAValues[74] - acadoWorkspace.evH[14];
acadoWorkspace.lbA[75] = acadoVariables.lbAValues[75] - acadoWorkspace.evH[15];
acadoWorkspace.lbA[76] = acadoVariables.lbAValues[76] - acadoWorkspace.evH[16];
acadoWorkspace.lbA[77] = acadoVariables.lbAValues[77] - acadoWorkspace.evH[17];
acadoWorkspace.lbA[78] = acadoVariables.lbAValues[78] - acadoWorkspace.evH[18];
acadoWorkspace.lbA[79] = acadoVariables.lbAValues[79] - acadoWorkspace.evH[19];
acadoWorkspace.lbA[80] = acadoVariables.lbAValues[80] - acadoWorkspace.evH[20];
acadoWorkspace.lbA[81] = acadoVariables.lbAValues[81] - acadoWorkspace.evH[21];
acadoWorkspace.lbA[82] = acadoVariables.lbAValues[82] - acadoWorkspace.evH[22];
acadoWorkspace.lbA[83] = acadoVariables.lbAValues[83] - acadoWorkspace.evH[23];
acadoWorkspace.lbA[84] = acadoVariables.lbAValues[84] - acadoWorkspace.evH[24];
acadoWorkspace.lbA[85] = acadoVariables.lbAValues[85] - acadoWorkspace.evH[25];
acadoWorkspace.lbA[86] = acadoVariables.lbAValues[86] - acadoWorkspace.evH[26];
acadoWorkspace.lbA[87] = acadoVariables.lbAValues[87] - acadoWorkspace.evH[27];
acadoWorkspace.lbA[88] = acadoVariables.lbAValues[88] - acadoWorkspace.evH[28];
acadoWorkspace.lbA[89] = acadoVariables.lbAValues[89] - acadoWorkspace.evH[29];
acadoWorkspace.lbA[90] = acadoVariables.lbAValues[90] - acadoWorkspace.evH[30];
acadoWorkspace.lbA[91] = acadoVariables.lbAValues[91] - acadoWorkspace.evH[31];
acadoWorkspace.lbA[92] = acadoVariables.lbAValues[92] - acadoWorkspace.evH[32];
acadoWorkspace.lbA[93] = acadoVariables.lbAValues[93] - acadoWorkspace.evH[33];
acadoWorkspace.lbA[94] = acadoVariables.lbAValues[94] - acadoWorkspace.evH[34];
acadoWorkspace.lbA[95] = acadoVariables.lbAValues[95] - acadoWorkspace.evH[35];
acadoWorkspace.lbA[96] = acadoVariables.lbAValues[96] - acadoWorkspace.evH[36];
acadoWorkspace.lbA[97] = acadoVariables.lbAValues[97] - acadoWorkspace.evH[37];
acadoWorkspace.lbA[98] = acadoVariables.lbAValues[98] - acadoWorkspace.evH[38];
acadoWorkspace.lbA[99] = acadoVariables.lbAValues[99] - acadoWorkspace.evH[39];
acadoWorkspace.lbA[100] = acadoVariables.lbAValues[100] - acadoWorkspace.evH[40];
acadoWorkspace.lbA[101] = acadoVariables.lbAValues[101] - acadoWorkspace.evH[41];
acadoWorkspace.lbA[102] = acadoVariables.lbAValues[102] - acadoWorkspace.evH[42];
acadoWorkspace.lbA[103] = acadoVariables.lbAValues[103] - acadoWorkspace.evH[43];
acadoWorkspace.lbA[104] = acadoVariables.lbAValues[104] - acadoWorkspace.evH[44];
acadoWorkspace.lbA[105] = acadoVariables.lbAValues[105] - acadoWorkspace.evH[45];
acadoWorkspace.lbA[106] = acadoVariables.lbAValues[106] - acadoWorkspace.evH[46];
acadoWorkspace.lbA[107] = acadoVariables.lbAValues[107] - acadoWorkspace.evH[47];
acadoWorkspace.lbA[108] = acadoVariables.lbAValues[108] - acadoWorkspace.evH[48];
acadoWorkspace.lbA[109] = acadoVariables.lbAValues[109] - acadoWorkspace.evH[49];
acadoWorkspace.lbA[110] = acadoVariables.lbAValues[110] - acadoWorkspace.evH[50];
acadoWorkspace.lbA[111] = acadoVariables.lbAValues[111] - acadoWorkspace.evH[51];
acadoWorkspace.lbA[112] = acadoVariables.lbAValues[112] - acadoWorkspace.evH[52];
acadoWorkspace.lbA[113] = acadoVariables.lbAValues[113] - acadoWorkspace.evH[53];
acadoWorkspace.lbA[114] = acadoVariables.lbAValues[114] - acadoWorkspace.evH[54];
acadoWorkspace.lbA[115] = acadoVariables.lbAValues[115] - acadoWorkspace.evH[55];
acadoWorkspace.lbA[116] = acadoVariables.lbAValues[116] - acadoWorkspace.evH[56];
acadoWorkspace.lbA[117] = acadoVariables.lbAValues[117] - acadoWorkspace.evH[57];
acadoWorkspace.lbA[118] = acadoVariables.lbAValues[118] - acadoWorkspace.evH[58];
acadoWorkspace.lbA[119] = acadoVariables.lbAValues[119] - acadoWorkspace.evH[59];

acadoWorkspace.ubA[60] = acadoVariables.ubAValues[60] - acadoWorkspace.evH[0];
acadoWorkspace.ubA[61] = acadoVariables.ubAValues[61] - acadoWorkspace.evH[1];
acadoWorkspace.ubA[62] = acadoVariables.ubAValues[62] - acadoWorkspace.evH[2];
acadoWorkspace.ubA[63] = acadoVariables.ubAValues[63] - acadoWorkspace.evH[3];
acadoWorkspace.ubA[64] = acadoVariables.ubAValues[64] - acadoWorkspace.evH[4];
acadoWorkspace.ubA[65] = acadoVariables.ubAValues[65] - acadoWorkspace.evH[5];
acadoWorkspace.ubA[66] = acadoVariables.ubAValues[66] - acadoWorkspace.evH[6];
acadoWorkspace.ubA[67] = acadoVariables.ubAValues[67] - acadoWorkspace.evH[7];
acadoWorkspace.ubA[68] = acadoVariables.ubAValues[68] - acadoWorkspace.evH[8];
acadoWorkspace.ubA[69] = acadoVariables.ubAValues[69] - acadoWorkspace.evH[9];
acadoWorkspace.ubA[70] = acadoVariables.ubAValues[70] - acadoWorkspace.evH[10];
acadoWorkspace.ubA[71] = acadoVariables.ubAValues[71] - acadoWorkspace.evH[11];
acadoWorkspace.ubA[72] = acadoVariables.ubAValues[72] - acadoWorkspace.evH[12];
acadoWorkspace.ubA[73] = acadoVariables.ubAValues[73] - acadoWorkspace.evH[13];
acadoWorkspace.ubA[74] = acadoVariables.ubAValues[74] - acadoWorkspace.evH[14];
acadoWorkspace.ubA[75] = acadoVariables.ubAValues[75] - acadoWorkspace.evH[15];
acadoWorkspace.ubA[76] = acadoVariables.ubAValues[76] - acadoWorkspace.evH[16];
acadoWorkspace.ubA[77] = acadoVariables.ubAValues[77] - acadoWorkspace.evH[17];
acadoWorkspace.ubA[78] = acadoVariables.ubAValues[78] - acadoWorkspace.evH[18];
acadoWorkspace.ubA[79] = acadoVariables.ubAValues[79] - acadoWorkspace.evH[19];
acadoWorkspace.ubA[80] = acadoVariables.ubAValues[80] - acadoWorkspace.evH[20];
acadoWorkspace.ubA[81] = acadoVariables.ubAValues[81] - acadoWorkspace.evH[21];
acadoWorkspace.ubA[82] = acadoVariables.ubAValues[82] - acadoWorkspace.evH[22];
acadoWorkspace.ubA[83] = acadoVariables.ubAValues[83] - acadoWorkspace.evH[23];
acadoWorkspace.ubA[84] = acadoVariables.ubAValues[84] - acadoWorkspace.evH[24];
acadoWorkspace.ubA[85] = acadoVariables.ubAValues[85] - acadoWorkspace.evH[25];
acadoWorkspace.ubA[86] = acadoVariables.ubAValues[86] - acadoWorkspace.evH[26];
acadoWorkspace.ubA[87] = acadoVariables.ubAValues[87] - acadoWorkspace.evH[27];
acadoWorkspace.ubA[88] = acadoVariables.ubAValues[88] - acadoWorkspace.evH[28];
acadoWorkspace.ubA[89] = acadoVariables.ubAValues[89] - acadoWorkspace.evH[29];
acadoWorkspace.ubA[90] = acadoVariables.ubAValues[90] - acadoWorkspace.evH[30];
acadoWorkspace.ubA[91] = acadoVariables.ubAValues[91] - acadoWorkspace.evH[31];
acadoWorkspace.ubA[92] = acadoVariables.ubAValues[92] - acadoWorkspace.evH[32];
acadoWorkspace.ubA[93] = acadoVariables.ubAValues[93] - acadoWorkspace.evH[33];
acadoWorkspace.ubA[94] = acadoVariables.ubAValues[94] - acadoWorkspace.evH[34];
acadoWorkspace.ubA[95] = acadoVariables.ubAValues[95] - acadoWorkspace.evH[35];
acadoWorkspace.ubA[96] = acadoVariables.ubAValues[96] - acadoWorkspace.evH[36];
acadoWorkspace.ubA[97] = acadoVariables.ubAValues[97] - acadoWorkspace.evH[37];
acadoWorkspace.ubA[98] = acadoVariables.ubAValues[98] - acadoWorkspace.evH[38];
acadoWorkspace.ubA[99] = acadoVariables.ubAValues[99] - acadoWorkspace.evH[39];
acadoWorkspace.ubA[100] = acadoVariables.ubAValues[100] - acadoWorkspace.evH[40];
acadoWorkspace.ubA[101] = acadoVariables.ubAValues[101] - acadoWorkspace.evH[41];
acadoWorkspace.ubA[102] = acadoVariables.ubAValues[102] - acadoWorkspace.evH[42];
acadoWorkspace.ubA[103] = acadoVariables.ubAValues[103] - acadoWorkspace.evH[43];
acadoWorkspace.ubA[104] = acadoVariables.ubAValues[104] - acadoWorkspace.evH[44];
acadoWorkspace.ubA[105] = acadoVariables.ubAValues[105] - acadoWorkspace.evH[45];
acadoWorkspace.ubA[106] = acadoVariables.ubAValues[106] - acadoWorkspace.evH[46];
acadoWorkspace.ubA[107] = acadoVariables.ubAValues[107] - acadoWorkspace.evH[47];
acadoWorkspace.ubA[108] = acadoVariables.ubAValues[108] - acadoWorkspace.evH[48];
acadoWorkspace.ubA[109] = acadoVariables.ubAValues[109] - acadoWorkspace.evH[49];
acadoWorkspace.ubA[110] = acadoVariables.ubAValues[110] - acadoWorkspace.evH[50];
acadoWorkspace.ubA[111] = acadoVariables.ubAValues[111] - acadoWorkspace.evH[51];
acadoWorkspace.ubA[112] = acadoVariables.ubAValues[112] - acadoWorkspace.evH[52];
acadoWorkspace.ubA[113] = acadoVariables.ubAValues[113] - acadoWorkspace.evH[53];
acadoWorkspace.ubA[114] = acadoVariables.ubAValues[114] - acadoWorkspace.evH[54];
acadoWorkspace.ubA[115] = acadoVariables.ubAValues[115] - acadoWorkspace.evH[55];
acadoWorkspace.ubA[116] = acadoVariables.ubAValues[116] - acadoWorkspace.evH[56];
acadoWorkspace.ubA[117] = acadoVariables.ubAValues[117] - acadoWorkspace.evH[57];
acadoWorkspace.ubA[118] = acadoVariables.ubAValues[118] - acadoWorkspace.evH[58];
acadoWorkspace.ubA[119] = acadoVariables.ubAValues[119] - acadoWorkspace.evH[59];

acado_macHxd( &(acadoWorkspace.evHx[ 10 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 62 ]), &(acadoWorkspace.ubA[ 62 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 20 ]), &(acadoWorkspace.d[ 5 ]), &(acadoWorkspace.lbA[ 64 ]), &(acadoWorkspace.ubA[ 64 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.d[ 10 ]), &(acadoWorkspace.lbA[ 66 ]), &(acadoWorkspace.ubA[ 66 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 40 ]), &(acadoWorkspace.d[ 15 ]), &(acadoWorkspace.lbA[ 68 ]), &(acadoWorkspace.ubA[ 68 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 50 ]), &(acadoWorkspace.d[ 20 ]), &(acadoWorkspace.lbA[ 70 ]), &(acadoWorkspace.ubA[ 70 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.d[ 25 ]), &(acadoWorkspace.lbA[ 72 ]), &(acadoWorkspace.ubA[ 72 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 70 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.lbA[ 74 ]), &(acadoWorkspace.ubA[ 74 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 80 ]), &(acadoWorkspace.d[ 35 ]), &(acadoWorkspace.lbA[ 76 ]), &(acadoWorkspace.ubA[ 76 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.lbA[ 78 ]), &(acadoWorkspace.ubA[ 78 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 100 ]), &(acadoWorkspace.d[ 45 ]), &(acadoWorkspace.lbA[ 80 ]), &(acadoWorkspace.ubA[ 80 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 110 ]), &(acadoWorkspace.d[ 50 ]), &(acadoWorkspace.lbA[ 82 ]), &(acadoWorkspace.ubA[ 82 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.d[ 55 ]), &(acadoWorkspace.lbA[ 84 ]), &(acadoWorkspace.ubA[ 84 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 130 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.lbA[ 86 ]), &(acadoWorkspace.ubA[ 86 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 140 ]), &(acadoWorkspace.d[ 65 ]), &(acadoWorkspace.lbA[ 88 ]), &(acadoWorkspace.ubA[ 88 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.d[ 70 ]), &(acadoWorkspace.lbA[ 90 ]), &(acadoWorkspace.ubA[ 90 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 160 ]), &(acadoWorkspace.d[ 75 ]), &(acadoWorkspace.lbA[ 92 ]), &(acadoWorkspace.ubA[ 92 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 170 ]), &(acadoWorkspace.d[ 80 ]), &(acadoWorkspace.lbA[ 94 ]), &(acadoWorkspace.ubA[ 94 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.d[ 85 ]), &(acadoWorkspace.lbA[ 96 ]), &(acadoWorkspace.ubA[ 96 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 190 ]), &(acadoWorkspace.d[ 90 ]), &(acadoWorkspace.lbA[ 98 ]), &(acadoWorkspace.ubA[ 98 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 200 ]), &(acadoWorkspace.d[ 95 ]), &(acadoWorkspace.lbA[ 100 ]), &(acadoWorkspace.ubA[ 100 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.d[ 100 ]), &(acadoWorkspace.lbA[ 102 ]), &(acadoWorkspace.ubA[ 102 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 220 ]), &(acadoWorkspace.d[ 105 ]), &(acadoWorkspace.lbA[ 104 ]), &(acadoWorkspace.ubA[ 104 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 230 ]), &(acadoWorkspace.d[ 110 ]), &(acadoWorkspace.lbA[ 106 ]), &(acadoWorkspace.ubA[ 106 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.d[ 115 ]), &(acadoWorkspace.lbA[ 108 ]), &(acadoWorkspace.ubA[ 108 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 250 ]), &(acadoWorkspace.d[ 120 ]), &(acadoWorkspace.lbA[ 110 ]), &(acadoWorkspace.ubA[ 110 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 260 ]), &(acadoWorkspace.d[ 125 ]), &(acadoWorkspace.lbA[ 112 ]), &(acadoWorkspace.ubA[ 112 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.d[ 130 ]), &(acadoWorkspace.lbA[ 114 ]), &(acadoWorkspace.ubA[ 114 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 280 ]), &(acadoWorkspace.d[ 135 ]), &(acadoWorkspace.lbA[ 116 ]), &(acadoWorkspace.ubA[ 116 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 290 ]), &(acadoWorkspace.d[ 140 ]), &(acadoWorkspace.lbA[ 118 ]), &(acadoWorkspace.ubA[ 118 ]) );

}

void acado_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];

for (lRun2 = 0; lRun2 < 510; ++lRun2)
acadoWorkspace.Dy[lRun2] -= acadoVariables.y[lRun2];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 34 ]), &(acadoWorkspace.Dy[ 17 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 68 ]), &(acadoWorkspace.Dy[ 34 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 102 ]), &(acadoWorkspace.Dy[ 51 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 136 ]), &(acadoWorkspace.Dy[ 68 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 170 ]), &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 204 ]), &(acadoWorkspace.Dy[ 102 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 238 ]), &(acadoWorkspace.Dy[ 119 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 272 ]), &(acadoWorkspace.Dy[ 136 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 306 ]), &(acadoWorkspace.Dy[ 153 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 340 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 374 ]), &(acadoWorkspace.Dy[ 187 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 408 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 442 ]), &(acadoWorkspace.Dy[ 221 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 476 ]), &(acadoWorkspace.Dy[ 238 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 510 ]), &(acadoWorkspace.Dy[ 255 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 544 ]), &(acadoWorkspace.Dy[ 272 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 578 ]), &(acadoWorkspace.Dy[ 289 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 612 ]), &(acadoWorkspace.Dy[ 306 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 646 ]), &(acadoWorkspace.Dy[ 323 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 680 ]), &(acadoWorkspace.Dy[ 340 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 714 ]), &(acadoWorkspace.Dy[ 357 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 748 ]), &(acadoWorkspace.Dy[ 374 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 782 ]), &(acadoWorkspace.Dy[ 391 ]), &(acadoWorkspace.g[ 46 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 816 ]), &(acadoWorkspace.Dy[ 408 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 850 ]), &(acadoWorkspace.Dy[ 425 ]), &(acadoWorkspace.g[ 50 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 884 ]), &(acadoWorkspace.Dy[ 442 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 918 ]), &(acadoWorkspace.Dy[ 459 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 952 ]), &(acadoWorkspace.Dy[ 476 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 986 ]), &(acadoWorkspace.Dy[ 493 ]), &(acadoWorkspace.g[ 58 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 85 ]), &(acadoWorkspace.Dy[ 17 ]), &(acadoWorkspace.QDy[ 5 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 170 ]), &(acadoWorkspace.Dy[ 34 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 255 ]), &(acadoWorkspace.Dy[ 51 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 340 ]), &(acadoWorkspace.Dy[ 68 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 425 ]), &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.QDy[ 25 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 510 ]), &(acadoWorkspace.Dy[ 102 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 595 ]), &(acadoWorkspace.Dy[ 119 ]), &(acadoWorkspace.QDy[ 35 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 680 ]), &(acadoWorkspace.Dy[ 136 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 765 ]), &(acadoWorkspace.Dy[ 153 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 850 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.QDy[ 50 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 935 ]), &(acadoWorkspace.Dy[ 187 ]), &(acadoWorkspace.QDy[ 55 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1020 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1105 ]), &(acadoWorkspace.Dy[ 221 ]), &(acadoWorkspace.QDy[ 65 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1190 ]), &(acadoWorkspace.Dy[ 238 ]), &(acadoWorkspace.QDy[ 70 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1275 ]), &(acadoWorkspace.Dy[ 255 ]), &(acadoWorkspace.QDy[ 75 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1360 ]), &(acadoWorkspace.Dy[ 272 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1445 ]), &(acadoWorkspace.Dy[ 289 ]), &(acadoWorkspace.QDy[ 85 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1530 ]), &(acadoWorkspace.Dy[ 306 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1615 ]), &(acadoWorkspace.Dy[ 323 ]), &(acadoWorkspace.QDy[ 95 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1700 ]), &(acadoWorkspace.Dy[ 340 ]), &(acadoWorkspace.QDy[ 100 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1785 ]), &(acadoWorkspace.Dy[ 357 ]), &(acadoWorkspace.QDy[ 105 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1870 ]), &(acadoWorkspace.Dy[ 374 ]), &(acadoWorkspace.QDy[ 110 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1955 ]), &(acadoWorkspace.Dy[ 391 ]), &(acadoWorkspace.QDy[ 115 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2040 ]), &(acadoWorkspace.Dy[ 408 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2125 ]), &(acadoWorkspace.Dy[ 425 ]), &(acadoWorkspace.QDy[ 125 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2210 ]), &(acadoWorkspace.Dy[ 442 ]), &(acadoWorkspace.QDy[ 130 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2295 ]), &(acadoWorkspace.Dy[ 459 ]), &(acadoWorkspace.QDy[ 135 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2380 ]), &(acadoWorkspace.Dy[ 476 ]), &(acadoWorkspace.QDy[ 140 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2465 ]), &(acadoWorkspace.Dy[ 493 ]), &(acadoWorkspace.QDy[ 145 ]) );

acadoWorkspace.QDy[150] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[151] = + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[152] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[153] = + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[154] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2];

for (lRun2 = 0; lRun2 < 150; ++lRun2)
acadoWorkspace.QDy[lRun2 + 5] += acadoWorkspace.Qd[lRun2];


for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multEQDy( &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.QDy[ lRun2 * 5 + 5 ]), &(acadoWorkspace.g[ lRun1 * 2 ]) );
}
}

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[1] += + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[2] += + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[3] += + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[4] += + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[5] += + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[6] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[7] += + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[8] += + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[9] += + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[10] += + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[11] += + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[12] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[13] += + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[14] += + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[15] += + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[16] += + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[17] += + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[18] += + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[19] += + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[20] += + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[21] += + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[22] += + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[23] += + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[24] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[25] += + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[26] += + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[27] += + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[28] += + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[29] += + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[30] += + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[31] += + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[32] += + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[33] += + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[34] += + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[35] += + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[36] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[37] += + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[38] += + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[39] += + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[40] += + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[41] += + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[42] += + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[43] += + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[44] += + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[45] += + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[46] += + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[47] += + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[48] += + acadoWorkspace.H10[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[244]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[49] += + acadoWorkspace.H10[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[249]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[50] += + acadoWorkspace.H10[250]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[251]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[252]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[253]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[254]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[51] += + acadoWorkspace.H10[255]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[256]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[257]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[258]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[259]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[52] += + acadoWorkspace.H10[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[264]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[53] += + acadoWorkspace.H10[265]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[266]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[267]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[268]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[269]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[54] += + acadoWorkspace.H10[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[274]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[55] += + acadoWorkspace.H10[275]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[276]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[277]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[278]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[279]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[56] += + acadoWorkspace.H10[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[284]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[57] += + acadoWorkspace.H10[285]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[286]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[287]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[288]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[289]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[58] += + acadoWorkspace.H10[290]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[291]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[292]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[293]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[294]*acadoWorkspace.Dx0[4];
acadoWorkspace.g[59] += + acadoWorkspace.H10[295]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[296]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[297]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[298]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[299]*acadoWorkspace.Dx0[4];

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];

tmp = + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[4] + acadoVariables.x[8];
tmp += acadoWorkspace.d[3];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - tmp;
acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - tmp;
tmp = + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[4] + acadoVariables.x[9];
tmp += acadoWorkspace.d[4];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - tmp;
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - tmp;
tmp = + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[4] + acadoVariables.x[13];
tmp += acadoWorkspace.d[8];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - tmp;
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - tmp;
tmp = + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[4] + acadoVariables.x[14];
tmp += acadoWorkspace.d[9];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - tmp;
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - tmp;
tmp = + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[4] + acadoVariables.x[18];
tmp += acadoWorkspace.d[13];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - tmp;
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - tmp;
tmp = + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[4] + acadoVariables.x[19];
tmp += acadoWorkspace.d[14];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - tmp;
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - tmp;
tmp = + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[4] + acadoVariables.x[23];
tmp += acadoWorkspace.d[18];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - tmp;
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - tmp;
tmp = + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[4] + acadoVariables.x[24];
tmp += acadoWorkspace.d[19];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - tmp;
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - tmp;
tmp = + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[4] + acadoVariables.x[28];
tmp += acadoWorkspace.d[23];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - tmp;
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - tmp;
tmp = + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4] + acadoVariables.x[29];
tmp += acadoWorkspace.d[24];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - tmp;
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - tmp;
tmp = + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[4] + acadoVariables.x[33];
tmp += acadoWorkspace.d[28];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - tmp;
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - tmp;
tmp = + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[4] + acadoVariables.x[34];
tmp += acadoWorkspace.d[29];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - tmp;
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - tmp;
tmp = + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[4] + acadoVariables.x[38];
tmp += acadoWorkspace.d[33];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - tmp;
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - tmp;
tmp = + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[4] + acadoVariables.x[39];
tmp += acadoWorkspace.d[34];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - tmp;
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - tmp;
tmp = + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[4] + acadoVariables.x[43];
tmp += acadoWorkspace.d[38];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - tmp;
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - tmp;
tmp = + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[4] + acadoVariables.x[44];
tmp += acadoWorkspace.d[39];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - tmp;
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - tmp;
tmp = + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[4] + acadoVariables.x[48];
tmp += acadoWorkspace.d[43];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - tmp;
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - tmp;
tmp = + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[4] + acadoVariables.x[49];
tmp += acadoWorkspace.d[44];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - tmp;
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - tmp;
tmp = + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4] + acadoVariables.x[53];
tmp += acadoWorkspace.d[48];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - tmp;
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - tmp;
tmp = + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[4] + acadoVariables.x[54];
tmp += acadoWorkspace.d[49];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - tmp;
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - tmp;
tmp = + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[4] + acadoVariables.x[58];
tmp += acadoWorkspace.d[53];
acadoWorkspace.lbA[20] = acadoVariables.lbAValues[20] - tmp;
acadoWorkspace.ubA[20] = acadoVariables.ubAValues[20] - tmp;
tmp = + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[4] + acadoVariables.x[59];
tmp += acadoWorkspace.d[54];
acadoWorkspace.lbA[21] = acadoVariables.lbAValues[21] - tmp;
acadoWorkspace.ubA[21] = acadoVariables.ubAValues[21] - tmp;
tmp = + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[4] + acadoVariables.x[63];
tmp += acadoWorkspace.d[58];
acadoWorkspace.lbA[22] = acadoVariables.lbAValues[22] - tmp;
acadoWorkspace.ubA[22] = acadoVariables.ubAValues[22] - tmp;
tmp = + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[4] + acadoVariables.x[64];
tmp += acadoWorkspace.d[59];
acadoWorkspace.lbA[23] = acadoVariables.lbAValues[23] - tmp;
acadoWorkspace.ubA[23] = acadoVariables.ubAValues[23] - tmp;
tmp = + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[4] + acadoVariables.x[68];
tmp += acadoWorkspace.d[63];
acadoWorkspace.lbA[24] = acadoVariables.lbAValues[24] - tmp;
acadoWorkspace.ubA[24] = acadoVariables.ubAValues[24] - tmp;
tmp = + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[4] + acadoVariables.x[69];
tmp += acadoWorkspace.d[64];
acadoWorkspace.lbA[25] = acadoVariables.lbAValues[25] - tmp;
acadoWorkspace.ubA[25] = acadoVariables.ubAValues[25] - tmp;
tmp = + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[4] + acadoVariables.x[73];
tmp += acadoWorkspace.d[68];
acadoWorkspace.lbA[26] = acadoVariables.lbAValues[26] - tmp;
acadoWorkspace.ubA[26] = acadoVariables.ubAValues[26] - tmp;
tmp = + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[4] + acadoVariables.x[74];
tmp += acadoWorkspace.d[69];
acadoWorkspace.lbA[27] = acadoVariables.lbAValues[27] - tmp;
acadoWorkspace.ubA[27] = acadoVariables.ubAValues[27] - tmp;
tmp = + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[4] + acadoVariables.x[78];
tmp += acadoWorkspace.d[73];
acadoWorkspace.lbA[28] = acadoVariables.lbAValues[28] - tmp;
acadoWorkspace.ubA[28] = acadoVariables.ubAValues[28] - tmp;
tmp = + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[4] + acadoVariables.x[79];
tmp += acadoWorkspace.d[74];
acadoWorkspace.lbA[29] = acadoVariables.lbAValues[29] - tmp;
acadoWorkspace.ubA[29] = acadoVariables.ubAValues[29] - tmp;
tmp = + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[4] + acadoVariables.x[83];
tmp += acadoWorkspace.d[78];
acadoWorkspace.lbA[30] = acadoVariables.lbAValues[30] - tmp;
acadoWorkspace.ubA[30] = acadoVariables.ubAValues[30] - tmp;
tmp = + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[4] + acadoVariables.x[84];
tmp += acadoWorkspace.d[79];
acadoWorkspace.lbA[31] = acadoVariables.lbAValues[31] - tmp;
acadoWorkspace.ubA[31] = acadoVariables.ubAValues[31] - tmp;
tmp = + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[4] + acadoVariables.x[88];
tmp += acadoWorkspace.d[83];
acadoWorkspace.lbA[32] = acadoVariables.lbAValues[32] - tmp;
acadoWorkspace.ubA[32] = acadoVariables.ubAValues[32] - tmp;
tmp = + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[4] + acadoVariables.x[89];
tmp += acadoWorkspace.d[84];
acadoWorkspace.lbA[33] = acadoVariables.lbAValues[33] - tmp;
acadoWorkspace.ubA[33] = acadoVariables.ubAValues[33] - tmp;
tmp = + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[4] + acadoVariables.x[93];
tmp += acadoWorkspace.d[88];
acadoWorkspace.lbA[34] = acadoVariables.lbAValues[34] - tmp;
acadoWorkspace.ubA[34] = acadoVariables.ubAValues[34] - tmp;
tmp = + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[4] + acadoVariables.x[94];
tmp += acadoWorkspace.d[89];
acadoWorkspace.lbA[35] = acadoVariables.lbAValues[35] - tmp;
acadoWorkspace.ubA[35] = acadoVariables.ubAValues[35] - tmp;
tmp = + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[4] + acadoVariables.x[98];
tmp += acadoWorkspace.d[93];
acadoWorkspace.lbA[36] = acadoVariables.lbAValues[36] - tmp;
acadoWorkspace.ubA[36] = acadoVariables.ubAValues[36] - tmp;
tmp = + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[4] + acadoVariables.x[99];
tmp += acadoWorkspace.d[94];
acadoWorkspace.lbA[37] = acadoVariables.lbAValues[37] - tmp;
acadoWorkspace.ubA[37] = acadoVariables.ubAValues[37] - tmp;
tmp = + acadoWorkspace.evGx[490]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[491]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[492]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[493]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[494]*acadoWorkspace.Dx0[4] + acadoVariables.x[103];
tmp += acadoWorkspace.d[98];
acadoWorkspace.lbA[38] = acadoVariables.lbAValues[38] - tmp;
acadoWorkspace.ubA[38] = acadoVariables.ubAValues[38] - tmp;
tmp = + acadoWorkspace.evGx[495]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[496]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[497]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[498]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[499]*acadoWorkspace.Dx0[4] + acadoVariables.x[104];
tmp += acadoWorkspace.d[99];
acadoWorkspace.lbA[39] = acadoVariables.lbAValues[39] - tmp;
acadoWorkspace.ubA[39] = acadoVariables.ubAValues[39] - tmp;
tmp = + acadoWorkspace.evGx[515]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[516]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[517]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[518]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[519]*acadoWorkspace.Dx0[4] + acadoVariables.x[108];
tmp += acadoWorkspace.d[103];
acadoWorkspace.lbA[40] = acadoVariables.lbAValues[40] - tmp;
acadoWorkspace.ubA[40] = acadoVariables.ubAValues[40] - tmp;
tmp = + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[4] + acadoVariables.x[109];
tmp += acadoWorkspace.d[104];
acadoWorkspace.lbA[41] = acadoVariables.lbAValues[41] - tmp;
acadoWorkspace.ubA[41] = acadoVariables.ubAValues[41] - tmp;
tmp = + acadoWorkspace.evGx[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[543]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[544]*acadoWorkspace.Dx0[4] + acadoVariables.x[113];
tmp += acadoWorkspace.d[108];
acadoWorkspace.lbA[42] = acadoVariables.lbAValues[42] - tmp;
acadoWorkspace.ubA[42] = acadoVariables.ubAValues[42] - tmp;
tmp = + acadoWorkspace.evGx[545]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[546]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[547]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[548]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[549]*acadoWorkspace.Dx0[4] + acadoVariables.x[114];
tmp += acadoWorkspace.d[109];
acadoWorkspace.lbA[43] = acadoVariables.lbAValues[43] - tmp;
acadoWorkspace.ubA[43] = acadoVariables.ubAValues[43] - tmp;
tmp = + acadoWorkspace.evGx[565]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[566]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[567]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[568]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[569]*acadoWorkspace.Dx0[4] + acadoVariables.x[118];
tmp += acadoWorkspace.d[113];
acadoWorkspace.lbA[44] = acadoVariables.lbAValues[44] - tmp;
acadoWorkspace.ubA[44] = acadoVariables.ubAValues[44] - tmp;
tmp = + acadoWorkspace.evGx[570]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[571]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[572]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[573]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[574]*acadoWorkspace.Dx0[4] + acadoVariables.x[119];
tmp += acadoWorkspace.d[114];
acadoWorkspace.lbA[45] = acadoVariables.lbAValues[45] - tmp;
acadoWorkspace.ubA[45] = acadoVariables.ubAValues[45] - tmp;
tmp = + acadoWorkspace.evGx[590]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[591]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[592]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[593]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[594]*acadoWorkspace.Dx0[4] + acadoVariables.x[123];
tmp += acadoWorkspace.d[118];
acadoWorkspace.lbA[46] = acadoVariables.lbAValues[46] - tmp;
acadoWorkspace.ubA[46] = acadoVariables.ubAValues[46] - tmp;
tmp = + acadoWorkspace.evGx[595]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[596]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[597]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[598]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[599]*acadoWorkspace.Dx0[4] + acadoVariables.x[124];
tmp += acadoWorkspace.d[119];
acadoWorkspace.lbA[47] = acadoVariables.lbAValues[47] - tmp;
acadoWorkspace.ubA[47] = acadoVariables.ubAValues[47] - tmp;
tmp = + acadoWorkspace.evGx[615]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[616]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[617]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[618]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[619]*acadoWorkspace.Dx0[4] + acadoVariables.x[128];
tmp += acadoWorkspace.d[123];
acadoWorkspace.lbA[48] = acadoVariables.lbAValues[48] - tmp;
acadoWorkspace.ubA[48] = acadoVariables.ubAValues[48] - tmp;
tmp = + acadoWorkspace.evGx[620]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[621]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[622]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[623]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[624]*acadoWorkspace.Dx0[4] + acadoVariables.x[129];
tmp += acadoWorkspace.d[124];
acadoWorkspace.lbA[49] = acadoVariables.lbAValues[49] - tmp;
acadoWorkspace.ubA[49] = acadoVariables.ubAValues[49] - tmp;
tmp = + acadoWorkspace.evGx[640]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[641]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[642]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[643]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[644]*acadoWorkspace.Dx0[4] + acadoVariables.x[133];
tmp += acadoWorkspace.d[128];
acadoWorkspace.lbA[50] = acadoVariables.lbAValues[50] - tmp;
acadoWorkspace.ubA[50] = acadoVariables.ubAValues[50] - tmp;
tmp = + acadoWorkspace.evGx[645]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[646]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[647]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[648]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[649]*acadoWorkspace.Dx0[4] + acadoVariables.x[134];
tmp += acadoWorkspace.d[129];
acadoWorkspace.lbA[51] = acadoVariables.lbAValues[51] - tmp;
acadoWorkspace.ubA[51] = acadoVariables.ubAValues[51] - tmp;
tmp = + acadoWorkspace.evGx[665]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[666]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[667]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[668]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[669]*acadoWorkspace.Dx0[4] + acadoVariables.x[138];
tmp += acadoWorkspace.d[133];
acadoWorkspace.lbA[52] = acadoVariables.lbAValues[52] - tmp;
acadoWorkspace.ubA[52] = acadoVariables.ubAValues[52] - tmp;
tmp = + acadoWorkspace.evGx[670]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[671]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[672]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[673]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[674]*acadoWorkspace.Dx0[4] + acadoVariables.x[139];
tmp += acadoWorkspace.d[134];
acadoWorkspace.lbA[53] = acadoVariables.lbAValues[53] - tmp;
acadoWorkspace.ubA[53] = acadoVariables.ubAValues[53] - tmp;
tmp = + acadoWorkspace.evGx[690]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[691]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[692]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[693]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[694]*acadoWorkspace.Dx0[4] + acadoVariables.x[143];
tmp += acadoWorkspace.d[138];
acadoWorkspace.lbA[54] = acadoVariables.lbAValues[54] - tmp;
acadoWorkspace.ubA[54] = acadoVariables.ubAValues[54] - tmp;
tmp = + acadoWorkspace.evGx[695]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[696]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[697]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[698]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[699]*acadoWorkspace.Dx0[4] + acadoVariables.x[144];
tmp += acadoWorkspace.d[139];
acadoWorkspace.lbA[55] = acadoVariables.lbAValues[55] - tmp;
acadoWorkspace.ubA[55] = acadoVariables.ubAValues[55] - tmp;
tmp = + acadoWorkspace.evGx[715]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[716]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[717]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[718]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[719]*acadoWorkspace.Dx0[4] + acadoVariables.x[148];
tmp += acadoWorkspace.d[143];
acadoWorkspace.lbA[56] = acadoVariables.lbAValues[56] - tmp;
acadoWorkspace.ubA[56] = acadoVariables.ubAValues[56] - tmp;
tmp = + acadoWorkspace.evGx[720]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[721]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[722]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[723]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[724]*acadoWorkspace.Dx0[4] + acadoVariables.x[149];
tmp += acadoWorkspace.d[144];
acadoWorkspace.lbA[57] = acadoVariables.lbAValues[57] - tmp;
acadoWorkspace.ubA[57] = acadoVariables.ubAValues[57] - tmp;
tmp = + acadoWorkspace.evGx[740]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[741]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[742]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[743]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[744]*acadoWorkspace.Dx0[4] + acadoVariables.x[153];
tmp += acadoWorkspace.d[148];
acadoWorkspace.lbA[58] = acadoVariables.lbAValues[58] - tmp;
acadoWorkspace.ubA[58] = acadoVariables.ubAValues[58] - tmp;
tmp = + acadoWorkspace.evGx[745]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[746]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[747]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[748]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[749]*acadoWorkspace.Dx0[4] + acadoVariables.x[154];
tmp += acadoWorkspace.d[149];
acadoWorkspace.lbA[59] = acadoVariables.lbAValues[59] - tmp;
acadoWorkspace.ubA[59] = acadoVariables.ubAValues[59] - tmp;

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[124]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[125]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[126]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[127]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[128]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[129]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[134]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[139]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[144]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[149]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[30] = + acadoWorkspace.A01[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[154]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[31] = + acadoWorkspace.A01[155]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[156]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[157]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[158]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[159]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[32] = + acadoWorkspace.A01[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[164]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[33] = + acadoWorkspace.A01[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[169]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[34] = + acadoWorkspace.A01[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[174]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[35] = + acadoWorkspace.A01[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[179]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[36] = + acadoWorkspace.A01[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[184]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[37] = + acadoWorkspace.A01[185]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[186]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[187]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[188]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[189]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[38] = + acadoWorkspace.A01[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[194]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[39] = + acadoWorkspace.A01[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[199]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[40] = + acadoWorkspace.A01[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[204]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[41] = + acadoWorkspace.A01[205]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[206]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[207]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[208]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[209]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[42] = + acadoWorkspace.A01[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[214]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[43] = + acadoWorkspace.A01[215]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[216]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[217]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[218]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[219]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[44] = + acadoWorkspace.A01[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[224]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[45] = + acadoWorkspace.A01[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[227]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[228]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[229]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[46] = + acadoWorkspace.A01[230]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[231]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[232]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[233]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[234]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[47] = + acadoWorkspace.A01[235]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[236]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[237]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[238]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[239]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[48] = + acadoWorkspace.A01[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[244]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[49] = + acadoWorkspace.A01[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[249]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[50] = + acadoWorkspace.A01[250]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[251]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[252]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[253]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[254]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[51] = + acadoWorkspace.A01[255]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[256]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[257]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[258]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[259]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[52] = + acadoWorkspace.A01[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[264]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[53] = + acadoWorkspace.A01[265]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[266]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[267]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[268]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[269]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[54] = + acadoWorkspace.A01[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[274]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[55] = + acadoWorkspace.A01[275]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[276]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[277]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[278]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[279]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[56] = + acadoWorkspace.A01[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[284]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[57] = + acadoWorkspace.A01[285]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[286]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[287]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[288]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[289]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[58] = + acadoWorkspace.A01[290]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[291]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[292]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[293]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[294]*acadoWorkspace.Dx0[4];
acadoWorkspace.pacA01Dx0[59] = + acadoWorkspace.A01[295]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[296]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[297]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[298]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[299]*acadoWorkspace.Dx0[4];
acadoWorkspace.lbA[60] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.lbA[61] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.lbA[62] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.lbA[63] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.lbA[64] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.lbA[65] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.lbA[66] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.lbA[67] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.lbA[68] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.lbA[69] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.lbA[70] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.lbA[71] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.lbA[72] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.lbA[73] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.lbA[74] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.lbA[75] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.lbA[76] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.lbA[77] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.lbA[78] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.lbA[79] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.lbA[80] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.lbA[81] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.lbA[82] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.lbA[83] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.lbA[84] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.lbA[85] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.lbA[86] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.lbA[87] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.lbA[88] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.lbA[89] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.lbA[90] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.lbA[91] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.lbA[92] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.lbA[93] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.lbA[94] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.lbA[95] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.lbA[96] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.lbA[97] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.lbA[98] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.lbA[99] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.lbA[100] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.lbA[101] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.lbA[102] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.lbA[103] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.lbA[104] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.lbA[105] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.lbA[106] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.lbA[107] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.lbA[108] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.lbA[109] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.lbA[110] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.lbA[111] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.lbA[112] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.lbA[113] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.lbA[114] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.lbA[115] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.lbA[116] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.lbA[117] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.lbA[118] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.lbA[119] -= acadoWorkspace.pacA01Dx0[59];

acadoWorkspace.ubA[60] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.ubA[61] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.ubA[62] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.ubA[63] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.ubA[64] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.ubA[65] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.ubA[66] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.ubA[67] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.ubA[68] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.ubA[69] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.ubA[70] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.ubA[71] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.ubA[72] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.ubA[73] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.ubA[74] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.ubA[75] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.ubA[76] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.ubA[77] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.ubA[78] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.ubA[79] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.ubA[80] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.ubA[81] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.ubA[82] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.ubA[83] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.ubA[84] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.ubA[85] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.ubA[86] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.ubA[87] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.ubA[88] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.ubA[89] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.ubA[90] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.ubA[91] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.ubA[92] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.ubA[93] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.ubA[94] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.ubA[95] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.ubA[96] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.ubA[97] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.ubA[98] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.ubA[99] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.ubA[100] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.ubA[101] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.ubA[102] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.ubA[103] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.ubA[104] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.ubA[105] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.ubA[106] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.ubA[107] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.ubA[108] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.ubA[109] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.ubA[110] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.ubA[111] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.ubA[112] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.ubA[113] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.ubA[114] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.ubA[115] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.ubA[116] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.ubA[117] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.ubA[118] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.ubA[119] -= acadoWorkspace.pacA01Dx0[59];

}

void acado_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];

acadoVariables.x[5] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[0];
acadoVariables.x[6] += + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[1];
acadoVariables.x[7] += + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[2];
acadoVariables.x[8] += + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[3];
acadoVariables.x[9] += + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[4];
acadoVariables.x[10] += + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[5];
acadoVariables.x[11] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[6];
acadoVariables.x[12] += + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[7];
acadoVariables.x[13] += + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[8];
acadoVariables.x[14] += + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[9];
acadoVariables.x[15] += + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[10];
acadoVariables.x[16] += + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[11];
acadoVariables.x[17] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[12];
acadoVariables.x[18] += + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[13];
acadoVariables.x[19] += + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[14];
acadoVariables.x[20] += + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[15];
acadoVariables.x[21] += + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[16];
acadoVariables.x[22] += + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[17];
acadoVariables.x[23] += + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[18];
acadoVariables.x[24] += + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[19];
acadoVariables.x[25] += + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[20];
acadoVariables.x[26] += + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[21];
acadoVariables.x[27] += + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[22];
acadoVariables.x[28] += + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[23];
acadoVariables.x[29] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[24];
acadoVariables.x[30] += + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[25];
acadoVariables.x[31] += + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[26];
acadoVariables.x[32] += + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[27];
acadoVariables.x[33] += + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[28];
acadoVariables.x[34] += + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[29];
acadoVariables.x[35] += + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[30];
acadoVariables.x[36] += + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[31];
acadoVariables.x[37] += + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[32];
acadoVariables.x[38] += + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[33];
acadoVariables.x[39] += + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[34];
acadoVariables.x[40] += + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[35];
acadoVariables.x[41] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[36];
acadoVariables.x[42] += + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[37];
acadoVariables.x[43] += + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[38];
acadoVariables.x[44] += + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[39];
acadoVariables.x[45] += + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[40];
acadoVariables.x[46] += + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[41];
acadoVariables.x[47] += + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[42];
acadoVariables.x[48] += + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[43];
acadoVariables.x[49] += + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[44];
acadoVariables.x[50] += + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[45];
acadoVariables.x[51] += + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[46];
acadoVariables.x[52] += + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[47];
acadoVariables.x[53] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[48];
acadoVariables.x[54] += + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[49];
acadoVariables.x[55] += + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[50];
acadoVariables.x[56] += + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[51];
acadoVariables.x[57] += + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[52];
acadoVariables.x[58] += + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[53];
acadoVariables.x[59] += + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[54];
acadoVariables.x[60] += + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[55];
acadoVariables.x[61] += + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[56];
acadoVariables.x[62] += + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[57];
acadoVariables.x[63] += + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[58];
acadoVariables.x[64] += + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[59];
acadoVariables.x[65] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[60];
acadoVariables.x[66] += + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[61];
acadoVariables.x[67] += + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[62];
acadoVariables.x[68] += + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[63];
acadoVariables.x[69] += + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[64];
acadoVariables.x[70] += + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[65];
acadoVariables.x[71] += + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[66];
acadoVariables.x[72] += + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[67];
acadoVariables.x[73] += + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[68];
acadoVariables.x[74] += + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[69];
acadoVariables.x[75] += + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[70];
acadoVariables.x[76] += + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[71];
acadoVariables.x[77] += + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[72];
acadoVariables.x[78] += + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[73];
acadoVariables.x[79] += + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[74];
acadoVariables.x[80] += + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[75];
acadoVariables.x[81] += + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[76];
acadoVariables.x[82] += + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[77];
acadoVariables.x[83] += + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[78];
acadoVariables.x[84] += + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[79];
acadoVariables.x[85] += + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[80];
acadoVariables.x[86] += + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[408]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[409]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[81];
acadoVariables.x[87] += + acadoWorkspace.evGx[410]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[411]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[412]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[413]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[414]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[82];
acadoVariables.x[88] += + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[83];
acadoVariables.x[89] += + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[84];
acadoVariables.x[90] += + acadoWorkspace.evGx[425]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[426]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[427]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[428]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[429]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[85];
acadoVariables.x[91] += + acadoWorkspace.evGx[430]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[431]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[86];
acadoVariables.x[92] += + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[87];
acadoVariables.x[93] += + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[88];
acadoVariables.x[94] += + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[89];
acadoVariables.x[95] += + acadoWorkspace.evGx[450]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[451]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[452]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[453]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[454]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[90];
acadoVariables.x[96] += + acadoWorkspace.evGx[455]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[456]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[457]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[458]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[459]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[91];
acadoVariables.x[97] += + acadoWorkspace.evGx[460]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[461]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[462]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[463]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[464]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[92];
acadoVariables.x[98] += + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[93];
acadoVariables.x[99] += + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[94];
acadoVariables.x[100] += + acadoWorkspace.evGx[475]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[476]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[477]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[478]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[479]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[95];
acadoVariables.x[101] += + acadoWorkspace.evGx[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[483]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[484]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[96];
acadoVariables.x[102] += + acadoWorkspace.evGx[485]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[486]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[487]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[488]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[489]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[97];
acadoVariables.x[103] += + acadoWorkspace.evGx[490]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[491]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[492]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[493]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[494]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[98];
acadoVariables.x[104] += + acadoWorkspace.evGx[495]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[496]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[497]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[498]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[499]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[99];
acadoVariables.x[105] += + acadoWorkspace.evGx[500]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[501]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[502]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[503]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[504]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[100];
acadoVariables.x[106] += + acadoWorkspace.evGx[505]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[506]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[507]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[508]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[509]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[101];
acadoVariables.x[107] += + acadoWorkspace.evGx[510]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[511]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[512]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[513]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[514]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[102];
acadoVariables.x[108] += + acadoWorkspace.evGx[515]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[516]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[517]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[518]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[519]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[103];
acadoVariables.x[109] += + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[104];
acadoVariables.x[110] += + acadoWorkspace.evGx[525]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[526]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[527]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[528]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[529]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[105];
acadoVariables.x[111] += + acadoWorkspace.evGx[530]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[531]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[532]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[533]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[534]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[106];
acadoVariables.x[112] += + acadoWorkspace.evGx[535]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[536]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[537]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[538]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[539]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[107];
acadoVariables.x[113] += + acadoWorkspace.evGx[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[543]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[544]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[108];
acadoVariables.x[114] += + acadoWorkspace.evGx[545]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[546]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[547]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[548]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[549]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[109];
acadoVariables.x[115] += + acadoWorkspace.evGx[550]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[551]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[552]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[553]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[554]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[110];
acadoVariables.x[116] += + acadoWorkspace.evGx[555]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[556]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[557]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[558]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[559]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[111];
acadoVariables.x[117] += + acadoWorkspace.evGx[560]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[561]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[562]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[563]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[564]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[112];
acadoVariables.x[118] += + acadoWorkspace.evGx[565]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[566]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[567]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[568]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[569]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[113];
acadoVariables.x[119] += + acadoWorkspace.evGx[570]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[571]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[572]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[573]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[574]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[114];
acadoVariables.x[120] += + acadoWorkspace.evGx[575]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[576]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[577]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[578]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[579]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[115];
acadoVariables.x[121] += + acadoWorkspace.evGx[580]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[581]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[582]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[583]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[584]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[116];
acadoVariables.x[122] += + acadoWorkspace.evGx[585]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[586]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[587]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[588]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[589]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[117];
acadoVariables.x[123] += + acadoWorkspace.evGx[590]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[591]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[592]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[593]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[594]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[118];
acadoVariables.x[124] += + acadoWorkspace.evGx[595]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[596]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[597]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[598]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[599]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[119];
acadoVariables.x[125] += + acadoWorkspace.evGx[600]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[601]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[602]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[603]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[604]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[120];
acadoVariables.x[126] += + acadoWorkspace.evGx[605]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[606]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[607]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[608]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[609]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[121];
acadoVariables.x[127] += + acadoWorkspace.evGx[610]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[611]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[612]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[613]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[614]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[122];
acadoVariables.x[128] += + acadoWorkspace.evGx[615]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[616]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[617]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[618]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[619]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[123];
acadoVariables.x[129] += + acadoWorkspace.evGx[620]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[621]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[622]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[623]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[624]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[124];
acadoVariables.x[130] += + acadoWorkspace.evGx[625]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[626]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[627]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[628]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[629]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[125];
acadoVariables.x[131] += + acadoWorkspace.evGx[630]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[631]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[632]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[633]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[634]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[126];
acadoVariables.x[132] += + acadoWorkspace.evGx[635]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[636]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[637]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[638]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[639]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[127];
acadoVariables.x[133] += + acadoWorkspace.evGx[640]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[641]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[642]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[643]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[644]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[128];
acadoVariables.x[134] += + acadoWorkspace.evGx[645]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[646]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[647]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[648]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[649]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[129];
acadoVariables.x[135] += + acadoWorkspace.evGx[650]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[651]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[652]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[653]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[654]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[130];
acadoVariables.x[136] += + acadoWorkspace.evGx[655]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[656]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[657]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[658]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[659]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[131];
acadoVariables.x[137] += + acadoWorkspace.evGx[660]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[661]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[662]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[663]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[664]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[132];
acadoVariables.x[138] += + acadoWorkspace.evGx[665]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[666]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[667]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[668]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[669]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[133];
acadoVariables.x[139] += + acadoWorkspace.evGx[670]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[671]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[672]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[673]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[674]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[134];
acadoVariables.x[140] += + acadoWorkspace.evGx[675]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[676]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[677]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[678]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[679]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[135];
acadoVariables.x[141] += + acadoWorkspace.evGx[680]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[681]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[682]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[683]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[684]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[136];
acadoVariables.x[142] += + acadoWorkspace.evGx[685]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[686]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[687]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[688]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[689]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[137];
acadoVariables.x[143] += + acadoWorkspace.evGx[690]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[691]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[692]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[693]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[694]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[138];
acadoVariables.x[144] += + acadoWorkspace.evGx[695]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[696]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[697]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[698]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[699]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[139];
acadoVariables.x[145] += + acadoWorkspace.evGx[700]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[701]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[702]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[703]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[704]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[140];
acadoVariables.x[146] += + acadoWorkspace.evGx[705]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[706]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[707]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[708]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[709]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[141];
acadoVariables.x[147] += + acadoWorkspace.evGx[710]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[711]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[712]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[713]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[714]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[142];
acadoVariables.x[148] += + acadoWorkspace.evGx[715]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[716]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[717]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[718]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[719]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[143];
acadoVariables.x[149] += + acadoWorkspace.evGx[720]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[721]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[722]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[723]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[724]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[144];
acadoVariables.x[150] += + acadoWorkspace.evGx[725]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[726]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[727]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[728]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[729]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[145];
acadoVariables.x[151] += + acadoWorkspace.evGx[730]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[731]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[732]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[733]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[734]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[146];
acadoVariables.x[152] += + acadoWorkspace.evGx[735]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[736]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[737]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[738]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[739]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[147];
acadoVariables.x[153] += + acadoWorkspace.evGx[740]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[741]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[742]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[743]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[744]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[148];
acadoVariables.x[154] += + acadoWorkspace.evGx[745]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[746]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[747]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[748]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[749]*acadoWorkspace.Dx0[4] + acadoWorkspace.d[149];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multEDu( &(acadoWorkspace.E[ lRun3 * 10 ]), &(acadoWorkspace.x[ lRun2 * 2 ]), &(acadoVariables.x[ lRun1 * 5 + 5 ]) );
}
}
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -2.0000000000000000e+00;
acadoVariables.lbValues[1] = -5.0000000000000000e+00;
acadoVariables.lbValues[2] = -2.0000000000000000e+00;
acadoVariables.lbValues[3] = -5.0000000000000000e+00;
acadoVariables.lbValues[4] = -2.0000000000000000e+00;
acadoVariables.lbValues[5] = -5.0000000000000000e+00;
acadoVariables.lbValues[6] = -2.0000000000000000e+00;
acadoVariables.lbValues[7] = -5.0000000000000000e+00;
acadoVariables.lbValues[8] = -2.0000000000000000e+00;
acadoVariables.lbValues[9] = -5.0000000000000000e+00;
acadoVariables.lbValues[10] = -2.0000000000000000e+00;
acadoVariables.lbValues[11] = -5.0000000000000000e+00;
acadoVariables.lbValues[12] = -2.0000000000000000e+00;
acadoVariables.lbValues[13] = -5.0000000000000000e+00;
acadoVariables.lbValues[14] = -2.0000000000000000e+00;
acadoVariables.lbValues[15] = -5.0000000000000000e+00;
acadoVariables.lbValues[16] = -2.0000000000000000e+00;
acadoVariables.lbValues[17] = -5.0000000000000000e+00;
acadoVariables.lbValues[18] = -2.0000000000000000e+00;
acadoVariables.lbValues[19] = -5.0000000000000000e+00;
acadoVariables.lbValues[20] = -2.0000000000000000e+00;
acadoVariables.lbValues[21] = -5.0000000000000000e+00;
acadoVariables.lbValues[22] = -2.0000000000000000e+00;
acadoVariables.lbValues[23] = -5.0000000000000000e+00;
acadoVariables.lbValues[24] = -2.0000000000000000e+00;
acadoVariables.lbValues[25] = -5.0000000000000000e+00;
acadoVariables.lbValues[26] = -2.0000000000000000e+00;
acadoVariables.lbValues[27] = -5.0000000000000000e+00;
acadoVariables.lbValues[28] = -2.0000000000000000e+00;
acadoVariables.lbValues[29] = -5.0000000000000000e+00;
acadoVariables.lbValues[30] = -2.0000000000000000e+00;
acadoVariables.lbValues[31] = -5.0000000000000000e+00;
acadoVariables.lbValues[32] = -2.0000000000000000e+00;
acadoVariables.lbValues[33] = -5.0000000000000000e+00;
acadoVariables.lbValues[34] = -2.0000000000000000e+00;
acadoVariables.lbValues[35] = -5.0000000000000000e+00;
acadoVariables.lbValues[36] = -2.0000000000000000e+00;
acadoVariables.lbValues[37] = -5.0000000000000000e+00;
acadoVariables.lbValues[38] = -2.0000000000000000e+00;
acadoVariables.lbValues[39] = -5.0000000000000000e+00;
acadoVariables.lbValues[40] = -2.0000000000000000e+00;
acadoVariables.lbValues[41] = -5.0000000000000000e+00;
acadoVariables.lbValues[42] = -2.0000000000000000e+00;
acadoVariables.lbValues[43] = -5.0000000000000000e+00;
acadoVariables.lbValues[44] = -2.0000000000000000e+00;
acadoVariables.lbValues[45] = -5.0000000000000000e+00;
acadoVariables.lbValues[46] = -2.0000000000000000e+00;
acadoVariables.lbValues[47] = -5.0000000000000000e+00;
acadoVariables.lbValues[48] = -2.0000000000000000e+00;
acadoVariables.lbValues[49] = -5.0000000000000000e+00;
acadoVariables.lbValues[50] = -2.0000000000000000e+00;
acadoVariables.lbValues[51] = -5.0000000000000000e+00;
acadoVariables.lbValues[52] = -2.0000000000000000e+00;
acadoVariables.lbValues[53] = -5.0000000000000000e+00;
acadoVariables.lbValues[54] = -2.0000000000000000e+00;
acadoVariables.lbValues[55] = -5.0000000000000000e+00;
acadoVariables.lbValues[56] = -2.0000000000000000e+00;
acadoVariables.lbValues[57] = -5.0000000000000000e+00;
acadoVariables.lbValues[58] = -2.0000000000000000e+00;
acadoVariables.lbValues[59] = -5.0000000000000000e+00;
acadoVariables.ubValues[0] = 2.0000000000000000e+00;
acadoVariables.ubValues[1] = 5.0000000000000000e+00;
acadoVariables.ubValues[2] = 2.0000000000000000e+00;
acadoVariables.ubValues[3] = 5.0000000000000000e+00;
acadoVariables.ubValues[4] = 2.0000000000000000e+00;
acadoVariables.ubValues[5] = 5.0000000000000000e+00;
acadoVariables.ubValues[6] = 2.0000000000000000e+00;
acadoVariables.ubValues[7] = 5.0000000000000000e+00;
acadoVariables.ubValues[8] = 2.0000000000000000e+00;
acadoVariables.ubValues[9] = 5.0000000000000000e+00;
acadoVariables.ubValues[10] = 2.0000000000000000e+00;
acadoVariables.ubValues[11] = 5.0000000000000000e+00;
acadoVariables.ubValues[12] = 2.0000000000000000e+00;
acadoVariables.ubValues[13] = 5.0000000000000000e+00;
acadoVariables.ubValues[14] = 2.0000000000000000e+00;
acadoVariables.ubValues[15] = 5.0000000000000000e+00;
acadoVariables.ubValues[16] = 2.0000000000000000e+00;
acadoVariables.ubValues[17] = 5.0000000000000000e+00;
acadoVariables.ubValues[18] = 2.0000000000000000e+00;
acadoVariables.ubValues[19] = 5.0000000000000000e+00;
acadoVariables.ubValues[20] = 2.0000000000000000e+00;
acadoVariables.ubValues[21] = 5.0000000000000000e+00;
acadoVariables.ubValues[22] = 2.0000000000000000e+00;
acadoVariables.ubValues[23] = 5.0000000000000000e+00;
acadoVariables.ubValues[24] = 2.0000000000000000e+00;
acadoVariables.ubValues[25] = 5.0000000000000000e+00;
acadoVariables.ubValues[26] = 2.0000000000000000e+00;
acadoVariables.ubValues[27] = 5.0000000000000000e+00;
acadoVariables.ubValues[28] = 2.0000000000000000e+00;
acadoVariables.ubValues[29] = 5.0000000000000000e+00;
acadoVariables.ubValues[30] = 2.0000000000000000e+00;
acadoVariables.ubValues[31] = 5.0000000000000000e+00;
acadoVariables.ubValues[32] = 2.0000000000000000e+00;
acadoVariables.ubValues[33] = 5.0000000000000000e+00;
acadoVariables.ubValues[34] = 2.0000000000000000e+00;
acadoVariables.ubValues[35] = 5.0000000000000000e+00;
acadoVariables.ubValues[36] = 2.0000000000000000e+00;
acadoVariables.ubValues[37] = 5.0000000000000000e+00;
acadoVariables.ubValues[38] = 2.0000000000000000e+00;
acadoVariables.ubValues[39] = 5.0000000000000000e+00;
acadoVariables.ubValues[40] = 2.0000000000000000e+00;
acadoVariables.ubValues[41] = 5.0000000000000000e+00;
acadoVariables.ubValues[42] = 2.0000000000000000e+00;
acadoVariables.ubValues[43] = 5.0000000000000000e+00;
acadoVariables.ubValues[44] = 2.0000000000000000e+00;
acadoVariables.ubValues[45] = 5.0000000000000000e+00;
acadoVariables.ubValues[46] = 2.0000000000000000e+00;
acadoVariables.ubValues[47] = 5.0000000000000000e+00;
acadoVariables.ubValues[48] = 2.0000000000000000e+00;
acadoVariables.ubValues[49] = 5.0000000000000000e+00;
acadoVariables.ubValues[50] = 2.0000000000000000e+00;
acadoVariables.ubValues[51] = 5.0000000000000000e+00;
acadoVariables.ubValues[52] = 2.0000000000000000e+00;
acadoVariables.ubValues[53] = 5.0000000000000000e+00;
acadoVariables.ubValues[54] = 2.0000000000000000e+00;
acadoVariables.ubValues[55] = 5.0000000000000000e+00;
acadoVariables.ubValues[56] = 2.0000000000000000e+00;
acadoVariables.ubValues[57] = 5.0000000000000000e+00;
acadoVariables.ubValues[58] = 2.0000000000000000e+00;
acadoVariables.ubValues[59] = 5.0000000000000000e+00;
acadoVariables.lbAValues[0] = -2.0000000000000000e+00;
acadoVariables.lbAValues[1] = -2.0000000000000000e+00;
acadoVariables.lbAValues[2] = -2.0000000000000000e+00;
acadoVariables.lbAValues[3] = -2.0000000000000000e+00;
acadoVariables.lbAValues[4] = -2.0000000000000000e+00;
acadoVariables.lbAValues[5] = -2.0000000000000000e+00;
acadoVariables.lbAValues[6] = -2.0000000000000000e+00;
acadoVariables.lbAValues[7] = -2.0000000000000000e+00;
acadoVariables.lbAValues[8] = -2.0000000000000000e+00;
acadoVariables.lbAValues[9] = -2.0000000000000000e+00;
acadoVariables.lbAValues[10] = -2.0000000000000000e+00;
acadoVariables.lbAValues[11] = -2.0000000000000000e+00;
acadoVariables.lbAValues[12] = -2.0000000000000000e+00;
acadoVariables.lbAValues[13] = -2.0000000000000000e+00;
acadoVariables.lbAValues[14] = -2.0000000000000000e+00;
acadoVariables.lbAValues[15] = -2.0000000000000000e+00;
acadoVariables.lbAValues[16] = -2.0000000000000000e+00;
acadoVariables.lbAValues[17] = -2.0000000000000000e+00;
acadoVariables.lbAValues[18] = -2.0000000000000000e+00;
acadoVariables.lbAValues[19] = -2.0000000000000000e+00;
acadoVariables.lbAValues[20] = -2.0000000000000000e+00;
acadoVariables.lbAValues[21] = -2.0000000000000000e+00;
acadoVariables.lbAValues[22] = -2.0000000000000000e+00;
acadoVariables.lbAValues[23] = -2.0000000000000000e+00;
acadoVariables.lbAValues[24] = -2.0000000000000000e+00;
acadoVariables.lbAValues[25] = -2.0000000000000000e+00;
acadoVariables.lbAValues[26] = -2.0000000000000000e+00;
acadoVariables.lbAValues[27] = -2.0000000000000000e+00;
acadoVariables.lbAValues[28] = -2.0000000000000000e+00;
acadoVariables.lbAValues[29] = -2.0000000000000000e+00;
acadoVariables.lbAValues[30] = -2.0000000000000000e+00;
acadoVariables.lbAValues[31] = -2.0000000000000000e+00;
acadoVariables.lbAValues[32] = -2.0000000000000000e+00;
acadoVariables.lbAValues[33] = -2.0000000000000000e+00;
acadoVariables.lbAValues[34] = -2.0000000000000000e+00;
acadoVariables.lbAValues[35] = -2.0000000000000000e+00;
acadoVariables.lbAValues[36] = -2.0000000000000000e+00;
acadoVariables.lbAValues[37] = -2.0000000000000000e+00;
acadoVariables.lbAValues[38] = -2.0000000000000000e+00;
acadoVariables.lbAValues[39] = -2.0000000000000000e+00;
acadoVariables.lbAValues[40] = -2.0000000000000000e+00;
acadoVariables.lbAValues[41] = -2.0000000000000000e+00;
acadoVariables.lbAValues[42] = -2.0000000000000000e+00;
acadoVariables.lbAValues[43] = -2.0000000000000000e+00;
acadoVariables.lbAValues[44] = -2.0000000000000000e+00;
acadoVariables.lbAValues[45] = -2.0000000000000000e+00;
acadoVariables.lbAValues[46] = -2.0000000000000000e+00;
acadoVariables.lbAValues[47] = -2.0000000000000000e+00;
acadoVariables.lbAValues[48] = -2.0000000000000000e+00;
acadoVariables.lbAValues[49] = -2.0000000000000000e+00;
acadoVariables.lbAValues[50] = -2.0000000000000000e+00;
acadoVariables.lbAValues[51] = -2.0000000000000000e+00;
acadoVariables.lbAValues[52] = -2.0000000000000000e+00;
acadoVariables.lbAValues[53] = -2.0000000000000000e+00;
acadoVariables.lbAValues[54] = -2.0000000000000000e+00;
acadoVariables.lbAValues[55] = -2.0000000000000000e+00;
acadoVariables.lbAValues[56] = -2.0000000000000000e+00;
acadoVariables.lbAValues[57] = -2.0000000000000000e+00;
acadoVariables.lbAValues[58] = -2.0000000000000000e+00;
acadoVariables.lbAValues[59] = -2.0000000000000000e+00;
acadoVariables.lbAValues[60] = -1.0000000000000000e+12;
acadoVariables.lbAValues[61] = -1.0000000000000000e+12;
acadoVariables.lbAValues[62] = -1.0000000000000000e+12;
acadoVariables.lbAValues[63] = -1.0000000000000000e+12;
acadoVariables.lbAValues[64] = -1.0000000000000000e+12;
acadoVariables.lbAValues[65] = -1.0000000000000000e+12;
acadoVariables.lbAValues[66] = -1.0000000000000000e+12;
acadoVariables.lbAValues[67] = -1.0000000000000000e+12;
acadoVariables.lbAValues[68] = -1.0000000000000000e+12;
acadoVariables.lbAValues[69] = -1.0000000000000000e+12;
acadoVariables.lbAValues[70] = -1.0000000000000000e+12;
acadoVariables.lbAValues[71] = -1.0000000000000000e+12;
acadoVariables.lbAValues[72] = -1.0000000000000000e+12;
acadoVariables.lbAValues[73] = -1.0000000000000000e+12;
acadoVariables.lbAValues[74] = -1.0000000000000000e+12;
acadoVariables.lbAValues[75] = -1.0000000000000000e+12;
acadoVariables.lbAValues[76] = -1.0000000000000000e+12;
acadoVariables.lbAValues[77] = -1.0000000000000000e+12;
acadoVariables.lbAValues[78] = -1.0000000000000000e+12;
acadoVariables.lbAValues[79] = -1.0000000000000000e+12;
acadoVariables.lbAValues[80] = -1.0000000000000000e+12;
acadoVariables.lbAValues[81] = -1.0000000000000000e+12;
acadoVariables.lbAValues[82] = -1.0000000000000000e+12;
acadoVariables.lbAValues[83] = -1.0000000000000000e+12;
acadoVariables.lbAValues[84] = -1.0000000000000000e+12;
acadoVariables.lbAValues[85] = -1.0000000000000000e+12;
acadoVariables.lbAValues[86] = -1.0000000000000000e+12;
acadoVariables.lbAValues[87] = -1.0000000000000000e+12;
acadoVariables.lbAValues[88] = -1.0000000000000000e+12;
acadoVariables.lbAValues[89] = -1.0000000000000000e+12;
acadoVariables.lbAValues[90] = -1.0000000000000000e+12;
acadoVariables.lbAValues[91] = -1.0000000000000000e+12;
acadoVariables.lbAValues[92] = -1.0000000000000000e+12;
acadoVariables.lbAValues[93] = -1.0000000000000000e+12;
acadoVariables.lbAValues[94] = -1.0000000000000000e+12;
acadoVariables.lbAValues[95] = -1.0000000000000000e+12;
acadoVariables.lbAValues[96] = -1.0000000000000000e+12;
acadoVariables.lbAValues[97] = -1.0000000000000000e+12;
acadoVariables.lbAValues[98] = -1.0000000000000000e+12;
acadoVariables.lbAValues[99] = -1.0000000000000000e+12;
acadoVariables.lbAValues[100] = -1.0000000000000000e+12;
acadoVariables.lbAValues[101] = -1.0000000000000000e+12;
acadoVariables.lbAValues[102] = -1.0000000000000000e+12;
acadoVariables.lbAValues[103] = -1.0000000000000000e+12;
acadoVariables.lbAValues[104] = -1.0000000000000000e+12;
acadoVariables.lbAValues[105] = -1.0000000000000000e+12;
acadoVariables.lbAValues[106] = -1.0000000000000000e+12;
acadoVariables.lbAValues[107] = -1.0000000000000000e+12;
acadoVariables.lbAValues[108] = -1.0000000000000000e+12;
acadoVariables.lbAValues[109] = -1.0000000000000000e+12;
acadoVariables.lbAValues[110] = -1.0000000000000000e+12;
acadoVariables.lbAValues[111] = -1.0000000000000000e+12;
acadoVariables.lbAValues[112] = -1.0000000000000000e+12;
acadoVariables.lbAValues[113] = -1.0000000000000000e+12;
acadoVariables.lbAValues[114] = -1.0000000000000000e+12;
acadoVariables.lbAValues[115] = -1.0000000000000000e+12;
acadoVariables.lbAValues[116] = -1.0000000000000000e+12;
acadoVariables.lbAValues[117] = -1.0000000000000000e+12;
acadoVariables.lbAValues[118] = -1.0000000000000000e+12;
acadoVariables.lbAValues[119] = -1.0000000000000000e+12;
acadoVariables.ubAValues[0] = 9.0000000000000000e+00;
acadoVariables.ubAValues[1] = 2.0000000000000000e+00;
acadoVariables.ubAValues[2] = 9.0000000000000000e+00;
acadoVariables.ubAValues[3] = 2.0000000000000000e+00;
acadoVariables.ubAValues[4] = 9.0000000000000000e+00;
acadoVariables.ubAValues[5] = 2.0000000000000000e+00;
acadoVariables.ubAValues[6] = 9.0000000000000000e+00;
acadoVariables.ubAValues[7] = 2.0000000000000000e+00;
acadoVariables.ubAValues[8] = 9.0000000000000000e+00;
acadoVariables.ubAValues[9] = 2.0000000000000000e+00;
acadoVariables.ubAValues[10] = 9.0000000000000000e+00;
acadoVariables.ubAValues[11] = 2.0000000000000000e+00;
acadoVariables.ubAValues[12] = 9.0000000000000000e+00;
acadoVariables.ubAValues[13] = 2.0000000000000000e+00;
acadoVariables.ubAValues[14] = 9.0000000000000000e+00;
acadoVariables.ubAValues[15] = 2.0000000000000000e+00;
acadoVariables.ubAValues[16] = 9.0000000000000000e+00;
acadoVariables.ubAValues[17] = 2.0000000000000000e+00;
acadoVariables.ubAValues[18] = 9.0000000000000000e+00;
acadoVariables.ubAValues[19] = 2.0000000000000000e+00;
acadoVariables.ubAValues[20] = 9.0000000000000000e+00;
acadoVariables.ubAValues[21] = 2.0000000000000000e+00;
acadoVariables.ubAValues[22] = 9.0000000000000000e+00;
acadoVariables.ubAValues[23] = 2.0000000000000000e+00;
acadoVariables.ubAValues[24] = 9.0000000000000000e+00;
acadoVariables.ubAValues[25] = 2.0000000000000000e+00;
acadoVariables.ubAValues[26] = 9.0000000000000000e+00;
acadoVariables.ubAValues[27] = 2.0000000000000000e+00;
acadoVariables.ubAValues[28] = 9.0000000000000000e+00;
acadoVariables.ubAValues[29] = 2.0000000000000000e+00;
acadoVariables.ubAValues[30] = 9.0000000000000000e+00;
acadoVariables.ubAValues[31] = 2.0000000000000000e+00;
acadoVariables.ubAValues[32] = 9.0000000000000000e+00;
acadoVariables.ubAValues[33] = 2.0000000000000000e+00;
acadoVariables.ubAValues[34] = 9.0000000000000000e+00;
acadoVariables.ubAValues[35] = 2.0000000000000000e+00;
acadoVariables.ubAValues[36] = 9.0000000000000000e+00;
acadoVariables.ubAValues[37] = 2.0000000000000000e+00;
acadoVariables.ubAValues[38] = 9.0000000000000000e+00;
acadoVariables.ubAValues[39] = 2.0000000000000000e+00;
acadoVariables.ubAValues[40] = 9.0000000000000000e+00;
acadoVariables.ubAValues[41] = 2.0000000000000000e+00;
acadoVariables.ubAValues[42] = 9.0000000000000000e+00;
acadoVariables.ubAValues[43] = 2.0000000000000000e+00;
acadoVariables.ubAValues[44] = 9.0000000000000000e+00;
acadoVariables.ubAValues[45] = 2.0000000000000000e+00;
acadoVariables.ubAValues[46] = 9.0000000000000000e+00;
acadoVariables.ubAValues[47] = 2.0000000000000000e+00;
acadoVariables.ubAValues[48] = 9.0000000000000000e+00;
acadoVariables.ubAValues[49] = 2.0000000000000000e+00;
acadoVariables.ubAValues[50] = 9.0000000000000000e+00;
acadoVariables.ubAValues[51] = 2.0000000000000000e+00;
acadoVariables.ubAValues[52] = 9.0000000000000000e+00;
acadoVariables.ubAValues[53] = 2.0000000000000000e+00;
acadoVariables.ubAValues[54] = 9.0000000000000000e+00;
acadoVariables.ubAValues[55] = 2.0000000000000000e+00;
acadoVariables.ubAValues[56] = 9.0000000000000000e+00;
acadoVariables.ubAValues[57] = 2.0000000000000000e+00;
acadoVariables.ubAValues[58] = 9.0000000000000000e+00;
acadoVariables.ubAValues[59] = 2.0000000000000000e+00;
acadoVariables.ubAValues[60] = 0.0000000000000000e+00;
acadoVariables.ubAValues[61] = 0.0000000000000000e+00;
acadoVariables.ubAValues[62] = 0.0000000000000000e+00;
acadoVariables.ubAValues[63] = 0.0000000000000000e+00;
acadoVariables.ubAValues[64] = 0.0000000000000000e+00;
acadoVariables.ubAValues[65] = 0.0000000000000000e+00;
acadoVariables.ubAValues[66] = 0.0000000000000000e+00;
acadoVariables.ubAValues[67] = 0.0000000000000000e+00;
acadoVariables.ubAValues[68] = 0.0000000000000000e+00;
acadoVariables.ubAValues[69] = 0.0000000000000000e+00;
acadoVariables.ubAValues[70] = 0.0000000000000000e+00;
acadoVariables.ubAValues[71] = 0.0000000000000000e+00;
acadoVariables.ubAValues[72] = 0.0000000000000000e+00;
acadoVariables.ubAValues[73] = 0.0000000000000000e+00;
acadoVariables.ubAValues[74] = 0.0000000000000000e+00;
acadoVariables.ubAValues[75] = 0.0000000000000000e+00;
acadoVariables.ubAValues[76] = 0.0000000000000000e+00;
acadoVariables.ubAValues[77] = 0.0000000000000000e+00;
acadoVariables.ubAValues[78] = 0.0000000000000000e+00;
acadoVariables.ubAValues[79] = 0.0000000000000000e+00;
acadoVariables.ubAValues[80] = 0.0000000000000000e+00;
acadoVariables.ubAValues[81] = 0.0000000000000000e+00;
acadoVariables.ubAValues[82] = 0.0000000000000000e+00;
acadoVariables.ubAValues[83] = 0.0000000000000000e+00;
acadoVariables.ubAValues[84] = 0.0000000000000000e+00;
acadoVariables.ubAValues[85] = 0.0000000000000000e+00;
acadoVariables.ubAValues[86] = 0.0000000000000000e+00;
acadoVariables.ubAValues[87] = 0.0000000000000000e+00;
acadoVariables.ubAValues[88] = 0.0000000000000000e+00;
acadoVariables.ubAValues[89] = 0.0000000000000000e+00;
acadoVariables.ubAValues[90] = 0.0000000000000000e+00;
acadoVariables.ubAValues[91] = 0.0000000000000000e+00;
acadoVariables.ubAValues[92] = 0.0000000000000000e+00;
acadoVariables.ubAValues[93] = 0.0000000000000000e+00;
acadoVariables.ubAValues[94] = 0.0000000000000000e+00;
acadoVariables.ubAValues[95] = 0.0000000000000000e+00;
acadoVariables.ubAValues[96] = 0.0000000000000000e+00;
acadoVariables.ubAValues[97] = 0.0000000000000000e+00;
acadoVariables.ubAValues[98] = 0.0000000000000000e+00;
acadoVariables.ubAValues[99] = 0.0000000000000000e+00;
acadoVariables.ubAValues[100] = 0.0000000000000000e+00;
acadoVariables.ubAValues[101] = 0.0000000000000000e+00;
acadoVariables.ubAValues[102] = 0.0000000000000000e+00;
acadoVariables.ubAValues[103] = 0.0000000000000000e+00;
acadoVariables.ubAValues[104] = 0.0000000000000000e+00;
acadoVariables.ubAValues[105] = 0.0000000000000000e+00;
acadoVariables.ubAValues[106] = 0.0000000000000000e+00;
acadoVariables.ubAValues[107] = 0.0000000000000000e+00;
acadoVariables.ubAValues[108] = 0.0000000000000000e+00;
acadoVariables.ubAValues[109] = 0.0000000000000000e+00;
acadoVariables.ubAValues[110] = 0.0000000000000000e+00;
acadoVariables.ubAValues[111] = 0.0000000000000000e+00;
acadoVariables.ubAValues[112] = 0.0000000000000000e+00;
acadoVariables.ubAValues[113] = 0.0000000000000000e+00;
acadoVariables.ubAValues[114] = 0.0000000000000000e+00;
acadoVariables.ubAValues[115] = 0.0000000000000000e+00;
acadoVariables.ubAValues[116] = 0.0000000000000000e+00;
acadoVariables.ubAValues[117] = 0.0000000000000000e+00;
acadoVariables.ubAValues[118] = 0.0000000000000000e+00;
acadoVariables.ubAValues[119] = 0.0000000000000000e+00;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 5];
acadoWorkspace.state[1] = acadoVariables.x[index * 5 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 5 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 5 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 5 + 4];
acadoWorkspace.state[40] = acadoVariables.u[index * 2];
acadoWorkspace.state[41] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[42] = acadoVariables.od[index * 40];
acadoWorkspace.state[43] = acadoVariables.od[index * 40 + 1];
acadoWorkspace.state[44] = acadoVariables.od[index * 40 + 2];
acadoWorkspace.state[45] = acadoVariables.od[index * 40 + 3];
acadoWorkspace.state[46] = acadoVariables.od[index * 40 + 4];
acadoWorkspace.state[47] = acadoVariables.od[index * 40 + 5];
acadoWorkspace.state[48] = acadoVariables.od[index * 40 + 6];
acadoWorkspace.state[49] = acadoVariables.od[index * 40 + 7];
acadoWorkspace.state[50] = acadoVariables.od[index * 40 + 8];
acadoWorkspace.state[51] = acadoVariables.od[index * 40 + 9];
acadoWorkspace.state[52] = acadoVariables.od[index * 40 + 10];
acadoWorkspace.state[53] = acadoVariables.od[index * 40 + 11];
acadoWorkspace.state[54] = acadoVariables.od[index * 40 + 12];
acadoWorkspace.state[55] = acadoVariables.od[index * 40 + 13];
acadoWorkspace.state[56] = acadoVariables.od[index * 40 + 14];
acadoWorkspace.state[57] = acadoVariables.od[index * 40 + 15];
acadoWorkspace.state[58] = acadoVariables.od[index * 40 + 16];
acadoWorkspace.state[59] = acadoVariables.od[index * 40 + 17];
acadoWorkspace.state[60] = acadoVariables.od[index * 40 + 18];
acadoWorkspace.state[61] = acadoVariables.od[index * 40 + 19];
acadoWorkspace.state[62] = acadoVariables.od[index * 40 + 20];
acadoWorkspace.state[63] = acadoVariables.od[index * 40 + 21];
acadoWorkspace.state[64] = acadoVariables.od[index * 40 + 22];
acadoWorkspace.state[65] = acadoVariables.od[index * 40 + 23];
acadoWorkspace.state[66] = acadoVariables.od[index * 40 + 24];
acadoWorkspace.state[67] = acadoVariables.od[index * 40 + 25];
acadoWorkspace.state[68] = acadoVariables.od[index * 40 + 26];
acadoWorkspace.state[69] = acadoVariables.od[index * 40 + 27];
acadoWorkspace.state[70] = acadoVariables.od[index * 40 + 28];
acadoWorkspace.state[71] = acadoVariables.od[index * 40 + 29];
acadoWorkspace.state[72] = acadoVariables.od[index * 40 + 30];
acadoWorkspace.state[73] = acadoVariables.od[index * 40 + 31];
acadoWorkspace.state[74] = acadoVariables.od[index * 40 + 32];
acadoWorkspace.state[75] = acadoVariables.od[index * 40 + 33];
acadoWorkspace.state[76] = acadoVariables.od[index * 40 + 34];
acadoWorkspace.state[77] = acadoVariables.od[index * 40 + 35];
acadoWorkspace.state[78] = acadoVariables.od[index * 40 + 36];
acadoWorkspace.state[79] = acadoVariables.od[index * 40 + 37];
acadoWorkspace.state[80] = acadoVariables.od[index * 40 + 38];
acadoWorkspace.state[81] = acadoVariables.od[index * 40 + 39];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 5 + 5] = acadoWorkspace.state[0];
acadoVariables.x[index * 5 + 6] = acadoWorkspace.state[1];
acadoVariables.x[index * 5 + 7] = acadoWorkspace.state[2];
acadoVariables.x[index * 5 + 8] = acadoWorkspace.state[3];
acadoVariables.x[index * 5 + 9] = acadoWorkspace.state[4];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoVariables.x[index * 5] = acadoVariables.x[index * 5 + 5];
acadoVariables.x[index * 5 + 1] = acadoVariables.x[index * 5 + 6];
acadoVariables.x[index * 5 + 2] = acadoVariables.x[index * 5 + 7];
acadoVariables.x[index * 5 + 3] = acadoVariables.x[index * 5 + 8];
acadoVariables.x[index * 5 + 4] = acadoVariables.x[index * 5 + 9];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[150] = xEnd[0];
acadoVariables.x[151] = xEnd[1];
acadoVariables.x[152] = xEnd[2];
acadoVariables.x[153] = xEnd[3];
acadoVariables.x[154] = xEnd[4];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[150];
acadoWorkspace.state[1] = acadoVariables.x[151];
acadoWorkspace.state[2] = acadoVariables.x[152];
acadoWorkspace.state[3] = acadoVariables.x[153];
acadoWorkspace.state[4] = acadoVariables.x[154];
if (uEnd != 0)
{
acadoWorkspace.state[40] = uEnd[0];
acadoWorkspace.state[41] = uEnd[1];
}
else
{
acadoWorkspace.state[40] = acadoVariables.u[58];
acadoWorkspace.state[41] = acadoVariables.u[59];
}
acadoWorkspace.state[42] = acadoVariables.od[1200];
acadoWorkspace.state[43] = acadoVariables.od[1201];
acadoWorkspace.state[44] = acadoVariables.od[1202];
acadoWorkspace.state[45] = acadoVariables.od[1203];
acadoWorkspace.state[46] = acadoVariables.od[1204];
acadoWorkspace.state[47] = acadoVariables.od[1205];
acadoWorkspace.state[48] = acadoVariables.od[1206];
acadoWorkspace.state[49] = acadoVariables.od[1207];
acadoWorkspace.state[50] = acadoVariables.od[1208];
acadoWorkspace.state[51] = acadoVariables.od[1209];
acadoWorkspace.state[52] = acadoVariables.od[1210];
acadoWorkspace.state[53] = acadoVariables.od[1211];
acadoWorkspace.state[54] = acadoVariables.od[1212];
acadoWorkspace.state[55] = acadoVariables.od[1213];
acadoWorkspace.state[56] = acadoVariables.od[1214];
acadoWorkspace.state[57] = acadoVariables.od[1215];
acadoWorkspace.state[58] = acadoVariables.od[1216];
acadoWorkspace.state[59] = acadoVariables.od[1217];
acadoWorkspace.state[60] = acadoVariables.od[1218];
acadoWorkspace.state[61] = acadoVariables.od[1219];
acadoWorkspace.state[62] = acadoVariables.od[1220];
acadoWorkspace.state[63] = acadoVariables.od[1221];
acadoWorkspace.state[64] = acadoVariables.od[1222];
acadoWorkspace.state[65] = acadoVariables.od[1223];
acadoWorkspace.state[66] = acadoVariables.od[1224];
acadoWorkspace.state[67] = acadoVariables.od[1225];
acadoWorkspace.state[68] = acadoVariables.od[1226];
acadoWorkspace.state[69] = acadoVariables.od[1227];
acadoWorkspace.state[70] = acadoVariables.od[1228];
acadoWorkspace.state[71] = acadoVariables.od[1229];
acadoWorkspace.state[72] = acadoVariables.od[1230];
acadoWorkspace.state[73] = acadoVariables.od[1231];
acadoWorkspace.state[74] = acadoVariables.od[1232];
acadoWorkspace.state[75] = acadoVariables.od[1233];
acadoWorkspace.state[76] = acadoVariables.od[1234];
acadoWorkspace.state[77] = acadoVariables.od[1235];
acadoWorkspace.state[78] = acadoVariables.od[1236];
acadoWorkspace.state[79] = acadoVariables.od[1237];
acadoWorkspace.state[80] = acadoVariables.od[1238];
acadoWorkspace.state[81] = acadoVariables.od[1239];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[150] = acadoWorkspace.state[0];
acadoVariables.x[151] = acadoWorkspace.state[1];
acadoVariables.x[152] = acadoWorkspace.state[2];
acadoVariables.x[153] = acadoWorkspace.state[3];
acadoVariables.x[154] = acadoWorkspace.state[4];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 29; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[58] = uEnd[0];
acadoVariables.u[59] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59];
kkt = fabs( kkt );
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 120; ++index)
{
prd = acadoWorkspace.y[index + 60];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 17 */
real_t tmpDy[ 17 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 40];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 40 + 1];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 40 + 2];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1 * 40 + 3];
acadoWorkspace.objValueIn[11] = acadoVariables.od[lRun1 * 40 + 4];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 40 + 5];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 40 + 6];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 40 + 7];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 40 + 8];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 40 + 9];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 40 + 10];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 40 + 11];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 40 + 12];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 40 + 13];
acadoWorkspace.objValueIn[21] = acadoVariables.od[lRun1 * 40 + 14];
acadoWorkspace.objValueIn[22] = acadoVariables.od[lRun1 * 40 + 15];
acadoWorkspace.objValueIn[23] = acadoVariables.od[lRun1 * 40 + 16];
acadoWorkspace.objValueIn[24] = acadoVariables.od[lRun1 * 40 + 17];
acadoWorkspace.objValueIn[25] = acadoVariables.od[lRun1 * 40 + 18];
acadoWorkspace.objValueIn[26] = acadoVariables.od[lRun1 * 40 + 19];
acadoWorkspace.objValueIn[27] = acadoVariables.od[lRun1 * 40 + 20];
acadoWorkspace.objValueIn[28] = acadoVariables.od[lRun1 * 40 + 21];
acadoWorkspace.objValueIn[29] = acadoVariables.od[lRun1 * 40 + 22];
acadoWorkspace.objValueIn[30] = acadoVariables.od[lRun1 * 40 + 23];
acadoWorkspace.objValueIn[31] = acadoVariables.od[lRun1 * 40 + 24];
acadoWorkspace.objValueIn[32] = acadoVariables.od[lRun1 * 40 + 25];
acadoWorkspace.objValueIn[33] = acadoVariables.od[lRun1 * 40 + 26];
acadoWorkspace.objValueIn[34] = acadoVariables.od[lRun1 * 40 + 27];
acadoWorkspace.objValueIn[35] = acadoVariables.od[lRun1 * 40 + 28];
acadoWorkspace.objValueIn[36] = acadoVariables.od[lRun1 * 40 + 29];
acadoWorkspace.objValueIn[37] = acadoVariables.od[lRun1 * 40 + 30];
acadoWorkspace.objValueIn[38] = acadoVariables.od[lRun1 * 40 + 31];
acadoWorkspace.objValueIn[39] = acadoVariables.od[lRun1 * 40 + 32];
acadoWorkspace.objValueIn[40] = acadoVariables.od[lRun1 * 40 + 33];
acadoWorkspace.objValueIn[41] = acadoVariables.od[lRun1 * 40 + 34];
acadoWorkspace.objValueIn[42] = acadoVariables.od[lRun1 * 40 + 35];
acadoWorkspace.objValueIn[43] = acadoVariables.od[lRun1 * 40 + 36];
acadoWorkspace.objValueIn[44] = acadoVariables.od[lRun1 * 40 + 37];
acadoWorkspace.objValueIn[45] = acadoVariables.od[lRun1 * 40 + 38];
acadoWorkspace.objValueIn[46] = acadoVariables.od[lRun1 * 40 + 39];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 17] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 17];
acadoWorkspace.Dy[lRun1 * 17 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 17 + 1];
acadoWorkspace.Dy[lRun1 * 17 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 17 + 2];
acadoWorkspace.Dy[lRun1 * 17 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 17 + 3];
acadoWorkspace.Dy[lRun1 * 17 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 17 + 4];
acadoWorkspace.Dy[lRun1 * 17 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 17 + 5];
acadoWorkspace.Dy[lRun1 * 17 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 17 + 6];
acadoWorkspace.Dy[lRun1 * 17 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 17 + 7];
acadoWorkspace.Dy[lRun1 * 17 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 17 + 8];
acadoWorkspace.Dy[lRun1 * 17 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 17 + 9];
acadoWorkspace.Dy[lRun1 * 17 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 17 + 10];
acadoWorkspace.Dy[lRun1 * 17 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 17 + 11];
acadoWorkspace.Dy[lRun1 * 17 + 12] = acadoWorkspace.objValueOut[12] - acadoVariables.y[lRun1 * 17 + 12];
acadoWorkspace.Dy[lRun1 * 17 + 13] = acadoWorkspace.objValueOut[13] - acadoVariables.y[lRun1 * 17 + 13];
acadoWorkspace.Dy[lRun1 * 17 + 14] = acadoWorkspace.objValueOut[14] - acadoVariables.y[lRun1 * 17 + 14];
acadoWorkspace.Dy[lRun1 * 17 + 15] = acadoWorkspace.objValueOut[15] - acadoVariables.y[lRun1 * 17 + 15];
acadoWorkspace.Dy[lRun1 * 17 + 16] = acadoWorkspace.objValueOut[16] - acadoVariables.y[lRun1 * 17 + 16];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[150];
acadoWorkspace.objValueIn[1] = acadoVariables.x[151];
acadoWorkspace.objValueIn[2] = acadoVariables.x[152];
acadoWorkspace.objValueIn[3] = acadoVariables.x[153];
acadoWorkspace.objValueIn[4] = acadoVariables.x[154];
acadoWorkspace.objValueIn[5] = acadoVariables.od[1200];
acadoWorkspace.objValueIn[6] = acadoVariables.od[1201];
acadoWorkspace.objValueIn[7] = acadoVariables.od[1202];
acadoWorkspace.objValueIn[8] = acadoVariables.od[1203];
acadoWorkspace.objValueIn[9] = acadoVariables.od[1204];
acadoWorkspace.objValueIn[10] = acadoVariables.od[1205];
acadoWorkspace.objValueIn[11] = acadoVariables.od[1206];
acadoWorkspace.objValueIn[12] = acadoVariables.od[1207];
acadoWorkspace.objValueIn[13] = acadoVariables.od[1208];
acadoWorkspace.objValueIn[14] = acadoVariables.od[1209];
acadoWorkspace.objValueIn[15] = acadoVariables.od[1210];
acadoWorkspace.objValueIn[16] = acadoVariables.od[1211];
acadoWorkspace.objValueIn[17] = acadoVariables.od[1212];
acadoWorkspace.objValueIn[18] = acadoVariables.od[1213];
acadoWorkspace.objValueIn[19] = acadoVariables.od[1214];
acadoWorkspace.objValueIn[20] = acadoVariables.od[1215];
acadoWorkspace.objValueIn[21] = acadoVariables.od[1216];
acadoWorkspace.objValueIn[22] = acadoVariables.od[1217];
acadoWorkspace.objValueIn[23] = acadoVariables.od[1218];
acadoWorkspace.objValueIn[24] = acadoVariables.od[1219];
acadoWorkspace.objValueIn[25] = acadoVariables.od[1220];
acadoWorkspace.objValueIn[26] = acadoVariables.od[1221];
acadoWorkspace.objValueIn[27] = acadoVariables.od[1222];
acadoWorkspace.objValueIn[28] = acadoVariables.od[1223];
acadoWorkspace.objValueIn[29] = acadoVariables.od[1224];
acadoWorkspace.objValueIn[30] = acadoVariables.od[1225];
acadoWorkspace.objValueIn[31] = acadoVariables.od[1226];
acadoWorkspace.objValueIn[32] = acadoVariables.od[1227];
acadoWorkspace.objValueIn[33] = acadoVariables.od[1228];
acadoWorkspace.objValueIn[34] = acadoVariables.od[1229];
acadoWorkspace.objValueIn[35] = acadoVariables.od[1230];
acadoWorkspace.objValueIn[36] = acadoVariables.od[1231];
acadoWorkspace.objValueIn[37] = acadoVariables.od[1232];
acadoWorkspace.objValueIn[38] = acadoVariables.od[1233];
acadoWorkspace.objValueIn[39] = acadoVariables.od[1234];
acadoWorkspace.objValueIn[40] = acadoVariables.od[1235];
acadoWorkspace.objValueIn[41] = acadoVariables.od[1236];
acadoWorkspace.objValueIn[42] = acadoVariables.od[1237];
acadoWorkspace.objValueIn[43] = acadoVariables.od[1238];
acadoWorkspace.objValueIn[44] = acadoVariables.od[1239];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 17] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 34] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 51] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 68] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 85] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 102] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 119] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 136] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 153] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 170] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 187] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 204] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 221] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 238] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 255] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 272];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 1] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 18] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 35] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 52] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 69] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 86] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 103] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 120] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 137] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 154] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 171] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 188] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 205] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 222] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 239] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 256] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 273];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 2] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 19] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 36] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 53] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 70] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 87] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 104] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 121] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 138] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 155] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 172] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 189] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 206] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 223] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 240] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 257] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 274];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 3] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 20] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 37] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 54] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 71] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 88] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 105] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 122] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 139] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 156] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 173] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 190] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 207] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 224] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 241] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 258] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 275];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 4] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 21] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 38] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 55] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 72] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 89] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 106] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 123] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 140] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 157] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 174] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 191] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 208] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 225] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 242] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 259] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 276];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 5] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 22] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 39] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 56] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 73] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 90] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 107] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 124] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 141] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 158] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 175] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 192] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 209] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 226] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 243] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 260] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 277];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 6] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 23] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 40] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 57] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 74] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 91] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 108] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 125] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 142] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 159] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 176] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 193] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 210] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 227] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 244] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 261] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 278];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 7] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 24] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 41] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 58] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 75] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 92] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 109] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 126] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 143] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 160] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 177] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 194] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 211] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 228] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 245] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 262] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 279];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 8] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 25] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 42] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 59] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 76] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 93] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 110] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 127] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 144] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 161] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 178] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 195] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 212] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 229] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 246] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 263] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 280];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 9] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 26] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 43] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 60] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 77] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 94] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 111] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 128] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 145] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 162] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 179] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 196] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 213] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 230] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 247] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 264] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 281];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 10] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 27] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 44] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 61] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 78] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 95] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 112] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 129] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 146] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 163] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 180] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 197] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 214] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 231] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 248] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 265] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 282];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 11] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 28] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 45] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 62] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 79] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 96] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 113] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 130] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 147] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 164] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 181] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 198] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 215] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 232] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 249] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 266] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 283];
tmpDy[12] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 12] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 29] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 46] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 63] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 80] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 97] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 114] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 131] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 148] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 165] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 182] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 199] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 216] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 233] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 250] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 267] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 284];
tmpDy[13] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 13] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 30] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 47] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 64] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 81] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 98] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 115] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 132] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 149] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 166] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 183] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 200] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 217] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 234] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 251] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 268] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 285];
tmpDy[14] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 14] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 31] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 48] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 65] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 82] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 99] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 116] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 133] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 150] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 167] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 184] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 201] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 218] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 235] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 252] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 269] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 286];
tmpDy[15] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 15] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 32] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 49] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 66] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 83] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 100] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 117] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 134] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 151] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 168] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 185] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 202] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 219] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 236] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 253] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 270] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 287];
tmpDy[16] = + acadoWorkspace.Dy[lRun1 * 17]*acadoVariables.W[lRun1 * 289 + 16] + acadoWorkspace.Dy[lRun1 * 17 + 1]*acadoVariables.W[lRun1 * 289 + 33] + acadoWorkspace.Dy[lRun1 * 17 + 2]*acadoVariables.W[lRun1 * 289 + 50] + acadoWorkspace.Dy[lRun1 * 17 + 3]*acadoVariables.W[lRun1 * 289 + 67] + acadoWorkspace.Dy[lRun1 * 17 + 4]*acadoVariables.W[lRun1 * 289 + 84] + acadoWorkspace.Dy[lRun1 * 17 + 5]*acadoVariables.W[lRun1 * 289 + 101] + acadoWorkspace.Dy[lRun1 * 17 + 6]*acadoVariables.W[lRun1 * 289 + 118] + acadoWorkspace.Dy[lRun1 * 17 + 7]*acadoVariables.W[lRun1 * 289 + 135] + acadoWorkspace.Dy[lRun1 * 17 + 8]*acadoVariables.W[lRun1 * 289 + 152] + acadoWorkspace.Dy[lRun1 * 17 + 9]*acadoVariables.W[lRun1 * 289 + 169] + acadoWorkspace.Dy[lRun1 * 17 + 10]*acadoVariables.W[lRun1 * 289 + 186] + acadoWorkspace.Dy[lRun1 * 17 + 11]*acadoVariables.W[lRun1 * 289 + 203] + acadoWorkspace.Dy[lRun1 * 17 + 12]*acadoVariables.W[lRun1 * 289 + 220] + acadoWorkspace.Dy[lRun1 * 17 + 13]*acadoVariables.W[lRun1 * 289 + 237] + acadoWorkspace.Dy[lRun1 * 17 + 14]*acadoVariables.W[lRun1 * 289 + 254] + acadoWorkspace.Dy[lRun1 * 17 + 15]*acadoVariables.W[lRun1 * 289 + 271] + acadoWorkspace.Dy[lRun1 * 17 + 16]*acadoVariables.W[lRun1 * 289 + 288];
objVal += + acadoWorkspace.Dy[lRun1 * 17]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 17 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 17 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 17 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 17 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 17 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 17 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 17 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 17 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 17 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 17 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 17 + 11]*tmpDy[11] + acadoWorkspace.Dy[lRun1 * 17 + 12]*tmpDy[12] + acadoWorkspace.Dy[lRun1 * 17 + 13]*tmpDy[13] + acadoWorkspace.Dy[lRun1 * 17 + 14]*tmpDy[14] + acadoWorkspace.Dy[lRun1 * 17 + 15]*tmpDy[15] + acadoWorkspace.Dy[lRun1 * 17 + 16]*tmpDy[16];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[4];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[8];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

