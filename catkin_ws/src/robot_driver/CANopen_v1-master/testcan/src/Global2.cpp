
// #include"stdafx.h"
#include "math.h"
#include "Global2.h"

		float g_CurrentJntAngleDeg[JNT_NUM], g_CurrentJntRateDeg[JNT_NUM], g_CurrentJntAccDeg[JNT_NUM];
		float g_CurrentJntAngleDeg_L[JNT_NUM_L], g_CurrentJntRateDeg_L[JNT_NUM_L], g_CurrentJntAccDeg_L[JNT_NUM_L];
        float g_CurrentJntAngleDeg_R[JNT_NUM_R], g_CurrentJntRateDeg_R[JNT_NUM_R], g_CurrentJntAccDeg_R[JNT_NUM_R];
        /*---------��"deg"�ĵ�λΪ��----------*/

		float g_CurrentJntAngle[JNT_NUM], g_CurrentJntRate[JNT_NUM], g_CurrentJntAcc[JNT_NUM];
		float g_CurrentJntAngle_L[JNT_NUM_L], g_CurrentJntRate_L[JNT_NUM_L], g_CurrentJntAcc_L[JNT_NUM_L];
        float g_CurrentJntAngle_R[JNT_NUM_R], g_CurrentJntRate_R[JNT_NUM_R], g_CurrentJntAcc_R[JNT_NUM_R];
		/*---------����"deg"�ĵ�λΪ����----------*/
		
		float gOut_fDesiredJntAngleDeg[JNT_NUM_L];   /*�����ؽڽǶ�*/
		float gOut_fDesiredJntRateDeg[JNT_NUM_L];  /*�����ؽڽ��ٶ�*/
		float gOut_fDesiredJntAccDeg[JNT_NUM_L];  /*�����ؽڽǼ��ٶ�*/
		 /*---------��"deg"�ĵ�λΪ��----------*/

		float gOut_fDesiredJntAngleDeg_L[JNT_NUM_L];   /*�����ؽڽǶ�*/
		float gOut_fDesiredJntRateDeg_L[JNT_NUM_L];  /*�����ؽڽ��ٶ�*/
		float gOut_fDesiredJntAccDeg_L[JNT_NUM_L];  /*�����ؽڽǼ��ٶ�*/
		 /*---------��"deg"�ĵ�λΪ��----------*/

		float gOut_fDesiredJntAngleDeg_R[JNT_NUM_L];   /*�����ؽڽǶ�*/
		float gOut_fDesiredJntRateDeg_R[JNT_NUM_L];  /*�����ؽڽ��ٶ�*/
		float gOut_fDesiredJntAccDeg_R[JNT_NUM_L];  /*�����ؽڽǼ��ٶ�*/
		 /*---------��"deg"�ĵ�λΪ��----------*/

float g_mfFirstVel[JNT_NUM_L] = {0, 0, 0, 0, 0, 0, 0};  /*	·���������ٶ�	*/
float g_mfSecondVel[JNT_NUM_L] = {0, 0, 0, 0, 0, 0, 0};  /*	·����ĵڶ����ٶ�*/
float g_mfLastVel[JNT_NUM_L] = {0, 0, 0, 0, 0, 0, 0};   /*	·������յ��ٶ�	*/
float g_mfRecipSecondVel[JNT_NUM_L] = {0, 0, 0, 0, 0, 0, 0};  /*	·����ĵ����ڶ��ڵ���ٶ�	*/
float g_mfFirstAcc[JNT_NUM_L] = {0, 0, 0, 0, 0, 0, 0};  /*	·����������ٶ�	*/
float g_mfLastAcc[JNT_NUM_L] = {0, 0, 0, 0, 0, 0, 0};   /*	·������յ���ٶ�	*/

	
float g_fSampleTime_OBC = 0.01f;
int g_iJntIndex = 0;   /*---���ؽ��˶�ʱ�Ĺؽ���ţ�0~6�ֱ��ʾ�ؽ�1~7---*/
int g_bFinishFlag = 0;
int g_bForceStopFlag = 0;
float g_CapTimeLen = 0;
int g_PlanningSpaceFlag = 1;   /*---��ʱ��3�ж��ã� =1���ؽڿռ䣬 =2���ѿ����ռ�*/



float m_SingleJntDomain[2] = {-10, 10};
int m_SingleJntDir = 1;
int m_JntMotionFlag = 1;


float g_mfMultiCartesianMovingData[MAX_POINT_NUM][7];     /*---��Ϊ�ѿ����˶�ʱͳһ��·����---*/
float g_mfMultiJntMovingData[MAX_POINT_NUM][JNT_NUM+1];  /*---��Ϊ��ؽ��˶�ʱͳһ��·���㣻ע�⣺ʱ�������һ��---*/
float g_mfMultiJntMovingData_L[MAX_POINT_NUM][JNT_NUM_L+1];
float g_mfMultiJntMovingData_R[MAX_POINT_NUM][JNT_NUM_R+1];
float g_mfSingleJntMovingData[MAX_POINT_NUM][2];          /*---��Ϊ���ؽ��˶�ʱͳһ��·����---*/


float g_mfMultiJntNode2[MAX_POINT_NUM][JNT_NUM+1];  /*---��ؽ��˶���㣬805���壬ʱ���ڵ�һ��---*/;

float g_fPlanRunTime_OBC = 0, g_fPlanRunTime_OBC2 = 0;  /*---��ֵ�㷨��ִ��ʱ��---*/
float g_fPlanStartTime_OBC = 0, g_fPlanStartTime_OBC2 = 0;  /*---��ֵ�Ŀ�ʼʱ��---*/
int g_iPlanFirstCircle = 0;
int g_bPlanBeyondTimeOut = 0;

int g_iPlanFirstCircle2 = 0;

float g_fTskStartTime_OBC = 0;  /*---����ʼʱ��---*/



float g_pfPoly5Param0[6*JNT_NUM_L], g_pfSplineMoment[MAX_POINT_NUM*JNT_NUM], g_pfPoly5ParamT[6*JNT_NUM];
float g_pfSinglePoly5Param0[6], g_pfSingleSplineMoment[MAX_POINT_NUM], g_pfSinglePoly5ParamT[6];
float g_fPosSeq[MAX_POINT_NUM*JNT_NUM];
float g_fSinglePosSeq[MAX_POINT_NUM];
float g_fTimeSeq[MAX_POINT_NUM*JNT_NUM]; 

float g_pfPoly5Param0_2[6*JNT_NUM_L], g_pfSplineMoment_2[MAX_POINT_NUM*JNT_NUM_L], g_pfPoly5ParamT_2[6*JNT_NUM_L];
float g_pfSinglePoly5Param0_2[6], g_pfSingleSplineMoment_2[MAX_POINT_NUM], g_pfSinglePoly5ParamT_2[6];
float g_fPosSeq_2[MAX_POINT_NUM*JNT_NUM_L];
float g_fSinglePosSeq_2[MAX_POINT_NUM];
float g_fTimeSeq_2[MAX_POINT_NUM*JNT_NUM_L]; 

float g_fJntErrLimit = 1;   /*----�ؽ����������ƣ����ڼ��鵱ǰ�ؽڽ���·�����֮���λ����-----*/
float g_fPoseErrLimit = 0.1f;   /*---ĩ��λ�����������ƣ����ڼ��鵱ǰ�ؽڽ���·�����֮���λ��100mm-----*/
int g_bCurrentIllEquInitErrorFlag = 0;

int g_bStartTimeIsZeros = 0;   /*---�����ж�ʱ�����еĳ�ʼʱ���Ƿ�Ϊ0�����ǣ���1----*/
int g_bTimeSeqRight = 1;   /*---�����ж�ʱ�����еĸ��ڵ��Ƿ���������ǣ���1----*/



float g_mfGivenVel[JNT_NUM_L] = { 0.0, -1.5f, 0.6f, 0.0, 0.3f, 0.0, 0.0 };
float g_mfSingleGivenVel = 0.0f;

int g_PayLoadGraspedFlag = 0;  


float HKSampleTime = 0.25;   /*---�пز�������---*/

void nfInitInterpolationPlan()    /*---ÿ������Ҫ���ò�ֵ�㷨ʱ��ִ�д˳�ʼ������----*/
{
	g_fPlanRunTime_OBC = 0;  /*---��ֵ�㷨��ִ��ʱ��---*/
	g_fPlanStartTime_OBC = 0;  /*---��ֵ�Ŀ�ʼʱ��---*/
	g_iPlanFirstCircle = 0;
	g_bForceStopFlag = 0;
	g_bFinishFlag = 0;
}



