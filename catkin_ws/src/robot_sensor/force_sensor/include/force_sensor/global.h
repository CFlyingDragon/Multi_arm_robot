#ifndef GLOBAL_H
#define GLOBAL_H


#define PI 3.14159265f
#define JNT_NUM 7

extern float g_fSampleTime_OBC;									/*-- ���ݲ�ֵʱ���� --*/
extern float g_afJntRateMaxDeg_OBC;								/*-- �ؽڽ��ٶ����� -- */
extern float gOut_fVisEndErr_OBC[6];									/*-- ��ǰ�����Ӿ��ŷ����ѿ����ռ䣩 --*/
extern float g_afEndRotSpdMax_OBC;					/*-- ĩ���˶����ٶ����� --*/
extern float g_afEndVelMax_OBC;									/*--  ĩ������˶��ٶ�40mm/s*/
extern float g_afEndZPosErrDisLimit_OBC;								/*�Ӿ��ŷ�Z�������������*/
extern float g_afVisKp_OBC[6];					/*-- �Ӿ��ŷ����Ʊ���ϵ�� --*/
extern float g_fCapRunTime_OBC;									/*-- �����Ӿ��ŷ����Ƶ�ִ��ʱ�� --*/
extern float g_afStartTime_OBC;
extern float g_afVisThreshold_OBC[6];
extern float g_afDesiredBethingPose_OBC[6];
extern float g_EndHisStepPara[3];
/*---------����ʹ�õ�ȫ�ֱ���-----------*/
extern float g_EndHisStep1[6];									 /*��һ�ε�ĩ���˶�����*/
extern float g_EndHisStep2 [6];									 /* �϶��ε�ĩ���˶�����*/
extern float g_EndHisStep3 [6];									 /* �����ε�ĩ���˶�����*/
extern float g_afZInnerVisThreshold_OBC;								/*�Ӿ��ŷ�Z���ڲ�ץ����Χ*/
extern float D_H[JNT_NUM][4];
extern float JointAngmax[JNT_NUM];
extern float JointAngmin[JNT_NUM];

int Rbt_InvMtrx( float *C, float *IC, int n );
void Rbt_PInvMtrx67( float AA[6][7], float AA_pinv[7][6]);
void Rbt_Cross(float u[], float v[], float n[]);
void Rbt_CalJcb(float DH[JNT_NUM][4], float JointAngle[JNT_NUM],float dRbtJcb[6][JNT_NUM],float T0n_c[4][4]);
void Rbt_MulMtrx(int m, int n, int p, float *A, float *B, float *C);
int Rbt_ikineItera(float DH[JNT_NUM][4], float T0n[4][4], float JntCurrent[JNT_NUM], float JntCalcted[JNT_NUM]);
void Rbt_fkine(float JointAngle[JNT_NUM], float DH[JNT_NUM][4], float T0n[4][4]);
void nfZyxEuler(float Euler_zyx[6],float TransMtrx[4][4]);
int nfVisualServoPlanPBVS(float fTempMatrix4[][4], float CurrentJntAngle[], float CurrentJntRate[],
						  float fJntAngleDesired[], float fJntRateDesired[], float fJntAccDesired[]);
int nfremotecontrol(float fPosedeltaX[6], float CurrentJntAngle[JNT_NUM],float fJntAngleDesired[JNT_NUM]);
int My_ikine_IteraFcn(float T0n[4][4], float JntCurrent[JNT_NUM], float JntCalcted[JNT_NUM]);
int Judge_joint_outof_angle(float theta0[7],float theta1[7]);
void My_forward_kinematics(float CurrentJntAngle[7],float T0n[4][4]);
void Interp5rdPoly_Param_fcn(float q0, float qv0, float qa0, float qf, float qvf, float qaf, float tf, float aa[6]);
void PathPlanning(float tt,float Toa_ea[4][4],float Tob_eb[4][4]);
void PathPlanningSim(float tt,float Toa_ea[4][4], float TB_L_tt[4][4], float Tob_eb[4][4]);
float EnvironmentForce(float Tss[4][4], float Tt[4][4], float t);
#endif
