// MultiJointPlanning.cpp: 定义控制台应用程序的入口点。
//
/*备注，本算法采用第一约束条件的三次样条曲线，通过追赶法进行求解获得的结果。*/
/*若想进行修改采用周期，进行差值的角度在主函数中的：fJntP2PNode数组中进行修改
其中Npath为中间节点的数量，若想生成的角度采样周期更小，则需要同时修改主函数
中的dt变量以及JointMultiPointPathPlan函数中的g_fSampleTime_OBC参数（外部全局变
量，需要到  Global2.CPP文件中进行修改），使其二者相等即可*/
/*若想修改规划的节点数量，则需要在头文件修改JNT_NUM至所需要的数量，并在fJntP2PNode
添加适当的参数，以及angle[0]等补充完全信息以及printf函数中进行参数的增减*/

// #include "stdafx.h"
#include "Global2.h"//定义头文件
#include "math.h"
#include "stdio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ros/ros.h"
#include "testcan/IpPos.h"
#include "iostream"
#include "fstream"

#define postoangle 1440000.0/360 
#define big 2498560.0/360
#define degreetoradius 180.0/3.1415926
#define radiustodegree 3.1415926/180.0

using namespace std;

typedef struct _RESTRAINPARA
{
	float	Time;			//时间约束
	float   ACC;			//关节加速度约束
	float   MaxVel;			//关节最大速度约束
							// 其他约束，可自由添加
} RESTRAINPARA;//限制条件


void nfChaseSolveEqu(float *pfArrIn_B, float *pfArrIn_C, float *pfArrIn_D, float *pfArrIn_P, int iNum, float *ArrOut_A)
{
	int i;
	
	float fArr_L[MAX_POINT_NUM], fArr_S[MAX_POINT_NUM], fArr_V[MAX_POINT_NUM];

	
	*(fArr_L + 0) = *(pfArrIn_B + 0);
	
	for (i = 0; i<iNum - 1; i++)
	{
		*(fArr_S + i) = (*(pfArrIn_C + i)) / (*(fArr_L + i));
		
		*(fArr_L + i + 1) = *(pfArrIn_B + i + 1) - (*(pfArrIn_D + i)) * (*(fArr_S + i));
		
	}

	*(fArr_V + 0) = (*(pfArrIn_P + 0)) / (*(fArr_L + 0));
	for (i = 0; i<iNum - 1; i++)
	{
		*(fArr_V + i + 1) = (*(pfArrIn_P + i + 1) - (*(pfArrIn_D + i)) * (*(fArr_V + i))) / (*(fArr_L + i + 1));
		
	}
		
	*(ArrOut_A + iNum - 1) = *(fArr_V + iNum - 1);
	
	for (i = iNum - 2; i >= 0; i--)
	{
		*(ArrOut_A + i) = *(fArr_V + i) - (*(fArr_S + i)) * (*(ArrOut_A + i + 1));
		}

	
}
void nfSplineFirstPlan(float *pfTimeSequence, float *pfPosSequence, int iNum, float dfFirstVel, float dfLastVel, float *pfBendingMoment, int iStep, float *pfDesiredJntAngle, float *pfDesiredJntRate, float *pfDesiredJntAcc)
{
	int iSup, iLow, iMid;
	float fTimeSupLim, fTimeLowLim;
	float fTimeItv, fTimeItv2;
	float fTimeL, fTimeL2, fTimeL3, fTimeS, fTimeS2, fTimeS3;

	float fPosL, fPosS, fBendMomentL, fBendMomentS;

	float fTime;
	fTime = iStep * g_fSampleTime_OBC;
	/*	采用二分法判断时间区间	*/
	iLow = 0;
	iSup = iNum;
	iMid = (iSup + iLow) / 2;
	while (abs(iSup - iLow - 1) > 1e-6)
	{
		if (fTime < *(pfTimeSequence + iMid)) {
			iSup = iMid;
		}
		else if (fTime > *(pfTimeSequence + iMid)) {
			iLow = iMid;
		}
		else {
			iLow = iMid;
			iSup = iMid + 1;
		}
		iMid = (iSup + iLow) / 2;
	}

	/*	最大值为零时	*/
	if (abs(iSup) < 1e-6) {
		*pfDesiredJntAngle = *pfPosSequence;
		*pfDesiredJntRate = dfFirstVel;
		*pfDesiredJntAcc = *pfBendingMoment;
	}

	/*	最小值时	*/
	else if (abs(iLow - iNum) < 1e-6) {
		*pfDesiredJntAngle = *(pfPosSequence + iNum);
		*pfDesiredJntRate = dfLastVel;
		*pfDesiredJntAcc = *(pfBendingMoment + iNum);
	}

	/*		*/
	else {
		fTimeLowLim = *(pfTimeSequence + iLow);
		fTimeSupLim = *(pfTimeSequence + iSup);
		fPosL = *(pfPosSequence + iLow);
		fPosS = *(pfPosSequence + iSup);
		fBendMomentL = *(pfBendingMoment + iLow);
		fBendMomentS = *(pfBendingMoment + iSup);
		fTimeItv = fTimeSupLim - fTimeLowLim;
		fTimeItv2 = fTimeItv * fTimeItv;
		fTimeL = fTime - fTimeLowLim;
		fTimeS = fTimeSupLim - fTime;
		fTimeL2 = fTimeL * fTimeL;
		fTimeL3 = fTimeL * fTimeL2;
		fTimeS2 = fTimeS * fTimeS;
		fTimeS3 = fTimeS * fTimeS2;

		*pfDesiredJntAngle = (fBendMomentL * fTimeS3 / 6.0f + fBendMomentS * fTimeL3 / 6.0f + (fPosL - fBendMomentL * fTimeItv2 / 6.0f) * fTimeS
			+ (fPosS - fBendMomentS * fTimeItv2 / 6.0f) * fTimeL) / fTimeItv;
		
		*pfDesiredJntRate = (-fBendMomentL * fTimeS2 / 2.0f + fBendMomentS * fTimeL2 / 2.0f + (fPosS - fPosL)) / fTimeItv
			- (fBendMomentS - fBendMomentL) * fTimeItv / 6.0f;
		
		*pfDesiredJntAcc = (fBendMomentL * fTimeS + fBendMomentS * fTimeL) / fTimeItv;
		
	}
}


void nfSplineFirstParam(float *pfTimeSequence, float *pfPosSequence, int iNum, float dfFirstVel, float dfLastVel, float *pfBendingMoment)
{
	int j;
	float fArr_B[MAX_POINT_NUM], fArr_C[MAX_POINT_NUM], fArr_D[MAX_POINT_NUM], fArr_P[MAX_POINT_NUM];
	float *fLambda, *fMu;

	float fTimeInterval, fTimeInterval_N, fTemp;
	float fFnTjpTj, fFnTjpTj_N, fFnTjpTjTjn;

	fLambda = &fArr_D[0];
	fMu = &fArr_C[1];

	fTimeInterval = *(pfTimeSequence + 1);
	
	fFnTjpTj = (*(pfPosSequence + 1) - *pfPosSequence) / fTimeInterval;
	fArr_C[0] = 1.0;
	fArr_P[0] = 6 * (fFnTjpTj - dfFirstVel) / fTimeInterval;
	fArr_B[0] = 2.0;
		
	for (j = 1; j<iNum; j++)
	{
		fTimeInterval_N = *(pfTimeSequence + j + 1) - *(pfTimeSequence + j);
		fTemp = fTimeInterval + fTimeInterval_N;

		*(fLambda + j - 1) = fTimeInterval / fTemp;
		*(fMu + j - 1) = fTimeInterval_N / fTemp;
		fFnTjpTj_N = (*(pfPosSequence + j + 1) - *(pfPosSequence + j)) / fTimeInterval_N;
		fFnTjpTjTjn = (fFnTjpTj_N - fFnTjpTj) / fTemp;
		fArr_P[j] = 6 * fFnTjpTjTjn;

		fArr_B[j] = 2.0;
		fTimeInterval = fTimeInterval_N;
		fFnTjpTj = fFnTjpTj_N;
	}	
	fArr_D[iNum - 1] = 1.0;
	fArr_P[iNum] = 6 * (dfLastVel - fFnTjpTj) / fTimeInterval;
	fArr_B[iNum] = 2.0;
	nfChaseSolveEqu(fArr_B, fArr_C, fArr_D, fArr_P, iNum + 1, pfBendingMoment);

}

int JointMultiPointPathPlan(float Tx, float Angle0[JNT_NUM], float Omega0[JNT_NUM], float fJntP2PNode[MAX_POINT_NUM][JNT_NUM + 1], int Npath, RESTRAINPARA RestPara, float JOINTCMD_Angle[], float JOINTCMD_Omega[], float JOINTCMD_Time[])
{

	int i, k;
	int iPlanStepMax;
	int iNodeNum;//规划的关节节点数量
	float dfFirstVel[JNT_NUM];//起始关节角速度
	float dfLastVel[JNT_NUM];//结束的关节角速度
	float fPlanTimeMax;
	int bTimeSeqRight = 1;
	int TmpFlag = 1;
	int PlanningSuccesFlg;
	float fJntErrRad = 0;/*----关节最大误差限制，用于检验当前关节角与路径起点之差，单位：弧度-----*/

	float dfJntAngleInit = 0, dfJntAngleFinal = 0;

	float fDesiredJntAngle[JNT_NUM], fDesiredJntRate[JNT_NUM], fDesiredJntAcc[JNT_NUM];//期望的关节角度、角速度和角加速度
		
	iNodeNum = Npath;//经过的节点数量
	fPlanTimeMax = fJntP2PNode[iNodeNum - 1][0];//确定规划时间
	iPlanStepMax = (int)(fPlanTimeMax / g_fSampleTime_OBC);  /*规划时间/规划时间间隔，g_fSampleTime_OBC为插值时间间隔，根据时间计算需要的最大步长-----*/

	if (g_iPlanFirstCircle == 0)   /*----(1)判断当前位置与路径的初始点；(2)判断时间起点及时间序列是否正确--*/
	{
		fJntErrRad = g_fJntErrLimit * PI / 180;   /*g_fJntErrLimit为关节最大误差限制----单位是弧度----*/

		if ((fabs(g_CurrentJntAngle[0] - fJntP2PNode[0][1])<fJntErrRad)
			&& (fabs(g_CurrentJntAngle[1] - fJntP2PNode[0][2])<fJntErrRad)
			&& (fabs(g_CurrentJntAngle[2] - fJntP2PNode[0][3])<fJntErrRad)
			&& (fabs(g_CurrentJntAngle[3] - fJntP2PNode[0][4])<fJntErrRad)
			&& (fabs(g_CurrentJntAngle[4] - fJntP2PNode[0][5])<fJntErrRad)
			&& (fabs(g_CurrentJntAngle[5] - fJntP2PNode[0][6])<fJntErrRad)
			&& (fabs(g_CurrentJntAngle[6] - fJntP2PNode[0][7])<fJntErrRad))//验证实际关节角度与期望关节角度误差范围之内
		{

			g_fPlanStartTime_OBC = Tx;//插值算法开始的时间
			g_fPlanRunTime_OBC = 0;//插值算法运行时间


								   /*	第一步时先判断时间起点及时间序列是否正确	*/


			for (i = 0; i<iNodeNum; i++)
			{
				g_fTimeSeq[i] = fJntP2PNode[i][0];//读取每个插值点的时间序列
												  /*	时间序列	*/
			}

			/************************************************************************/
			/* ---判断时间序列是否合理，时间序列应该是递增的                        */
			/************************************************************************/
			if (fabs(g_fTimeSeq[0]) >= 1e-6)    /*---初始时刻应为0，时间序列的第一个节点值应为零，按1e-6-----*/
			{
				g_bStartTimeIsZeros = 0;
			}
			else                               /*---用于判断时间序列的初始时刻是否为0，若不是，置1----*/
			{
				g_bStartTimeIsZeros = 1;

				for (i = 0; i<iNodeNum - 1; i++)
				{
					if (g_fTimeSeq[i]< g_fTimeSeq[i + 1])
					{
						TmpFlag = 1;
					}
					else
					{
						TmpFlag = 0;
					}
					bTimeSeqRight = bTimeSeqRight && TmpFlag;//表示时间序列的正确性，为1为正确的时间序列
				}
			}

			g_bTimeSeqRight = bTimeSeqRight;

			if (g_bStartTimeIsZeros && g_bTimeSeqRight)   /*----如果时间起点及时间序列合理，进行规划计算------*/
			{

				/************************************************************************/
				/*       采用三次样条进行规划                                           */
				/************************************************************************/

				for (k = 0; k<JNT_NUM; k++)
				{

					fJntP2PNode[0][k + 1] = g_CurrentJntAngle[k];//设定初始点位置=当前对应的关节叫
					for (i = 0; i<iNodeNum; i++)//iNodeNum=5，途径的关节点位置
					{
						g_fPosSeq[iNodeNum*k + i] = fJntP2PNode[i][k + 1];//将二维关节角转变成一维关节角

					}
					/*	位置序列，以当前位置为初始点	*/


					dfFirstVel[k] = g_mfFirstVel[k];//路径点的起点速度 float g_mfFirstVel[JNT_NUM_L] = {0, 0, 0, 0, 0, 0, 0};  /*	路径点的起点速度	*/
													/*	起点速度	*/

					dfLastVel[k] = g_mfLastVel[k];//路径点的终点速度 float g_mfLastVel[JNT_NUM_L] = {0, 0, 0, 0, 0, 0, 0};   /*	路径点的终点速度	*/
												  /*	终点速度	*/


					nfSplineFirstParam(g_fTimeSeq, g_fPosSeq + iNodeNum * k, iNodeNum - 1, *(dfFirstVel + k), *(dfLastVel + k), g_pfSplineMoment + iNodeNum * k);//g_pfSplineMoment的含义为末端节点的弯矩
																																								 /*	三次样条的参数计算	*/
																																								 /*对每一个进行三次样条计算，*/
				}

			}
			else
			{
				PlanningSuccesFlg = 0;  /*---不能进行正常规划；返回当前关节角----*/

				g_bFinishFlag = 1;

				for (i = 0; i<JNT_NUM; i++)
				{

					fDesiredJntAngle[i] = g_CurrentJntAngle[i];
					//fDesiredJntAngleDeg[i] = gOut_fDesiredJntAngle[i];   /*---返回上一次期望*/
					fDesiredJntRate[i] = 0;
					fDesiredJntAcc[i] = 0;
				}
				return PlanningSuccesFlg;
			}
		}

		else
		{
			g_bForceStopFlag = 1;
			g_bFinishFlag = 1;
			g_bCurrentIllEquInitErrorFlag = 1;

			PlanningSuccesFlg = 0;

			for (i = 0; i<JNT_NUM; i++)
			{
				fDesiredJntAngle[i] = g_CurrentJntAngle[i];
				//fDesiredJntAngleDeg[i] = gOut_fDesiredJntAngle[i];   /*---返回上一次期望*/
				fDesiredJntRate[i] = 0;
				fDesiredJntAcc[i] = 0;
			}

			return PlanningSuccesFlg;
		}

	}

	g_fPlanRunTime_OBC = Tx - g_fPlanStartTime_OBC;  /*---Timer begin----*/


	if (g_iPlanFirstCircle>iPlanStepMax)   /*----规划结束----*/
	{
		g_bFinishFlag = 1;
		PlanningSuccesFlg = 1;

		for (i = 0; i<JNT_NUM; i++)
		{
			fDesiredJntAngle[i] = fJntP2PNode[iNodeNum - 1][i + 1];
			fDesiredJntRate[i] = 0;
			fDesiredJntAcc[i] = 0;
		}

	}
	else
	{

		if (g_bStartTimeIsZeros && g_bTimeSeqRight)
		{

			PlanningSuccesFlg = 1;  /*---成功进行正常规划----*/


									/************************************************************************/
									/*       采用三次样条进行规划                                           */
									/************************************************************************/

			for (k = 0; k<JNT_NUM; k++)
			{
				nfSplineFirstPlan(g_fTimeSeq, g_fPosSeq + iNodeNum * k, iNodeNum - 1, *(g_mfFirstVel + k), *(g_mfFirstVel + k), g_pfSplineMoment + iNodeNum * k,
					g_iPlanFirstCircle,
					fDesiredJntAngle + k, fDesiredJntRate + k, fDesiredJntAcc + k);
				/*	三次样条计算期望值	*/
			}
		}
		else
		{
			PlanningSuccesFlg = 0;  /*---不能进行正常规划；返回当前关节角----*/
			g_bFinishFlag = 1;
			for (i = 0; i<JNT_NUM; i++)
			{
				fDesiredJntAngle[i] = g_CurrentJntAngle[i];
				//fDesiredJntAngleDeg[i] = gOut_fDesiredJntAngle[i];   /*---返回上一次期望*/
				fDesiredJntRate[i] = 0;
				fDesiredJntAcc[i] = 0;
			}
		}
	}


	/************************************************************************/
	/* 每个关节指令                                               */
	/************************************************************************/
	for (i = 0; i<JNT_NUM; i++)
	{

		JOINTCMD_Angle[i] = fDesiredJntAngle[i];
		JOINTCMD_Omega[i] = fDesiredJntRate[i];
		JOINTCMD_Time[i] = fPlanTimeMax;                /*----存的是运动时间----*/
	}

	g_iPlanFirstCircle++;

	return PlanningSuccesFlg;
}

testcan::IpPos ip_pos;
int sendangle(int id, double angle){
    ip_pos.id = id;
    ip_pos.pos = angle*big*degreetoradius;
    return 0;
}

int main(int argc, char *argv[])
{
    printf("hello\n");
    ros::init(argc, argv, "tort_gen_node");
    ros::NodeHandle nh_;
	printf("start");
    ros::Publisher pos_pub = nh_.advertise<testcan::IpPos>("ip_pos",1000);
	usleep(1000000);
	printf("start\n");

    	// usleep(1000000);*/
	sendangle(22, 0);
    pos_pub.publish(ip_pos);
	sendangle(21, 0);
    pos_pub.publish(ip_pos);
	sendangle(25, 0);
    pos_pub.publish(ip_pos);
	sendangle(23, 0);
    pos_pub.publish(ip_pos);
	sendangle(26, 0);
    pos_pub.publish(ip_pos);
	sendangle(27, 0);
    pos_pub.publish(ip_pos);
	sendangle(20, 0);
    pos_pub.publish(ip_pos);
	sendangle(24, 0);
    pos_pub.publish(ip_pos);
	usleep(5000000);
	// sendangle(17, 86.45*radiustodegree);
    // pos_pub.publish(ip_pos);
	// sendangle(12, -158.4*radiustodegree);
    // pos_pub.publish(ip_pos);
	// sendangle(15, -86.45*radiustodegree);
    // pos_pub.publish(ip_pos);
	// sendangle(10, 158.4*radiustodegree);
    // pos_pub.publish(ip_pos);

	// sendangle(14, -86.45*radiustodegree);
    // pos_pub.publish(ip_pos);
	// sendangle(13, 155*radiustodegree);
    // pos_pub.publish(ip_pos);
	// sendangle(11, 86.45*radiustodegree);
    // pos_pub.publish(ip_pos);
	// sendangle(16, -158.4*radiustodegree);
    // pos_pub.publish(ip_pos);
	// usleep(5000000);

	sendangle(17, 45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(12, -45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(15, -45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(10, 45.0*radiustodegree);
    pos_pub.publish(ip_pos);

	sendangle(14, -45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(13, 45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(11, 45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	sendangle(16, -45.0*radiustodegree);
    pos_pub.publish(ip_pos);
	usleep(10000000);

	float tf = 20;
	float Tx = 0;
	float dt = 0.01f;
	float JOINTCMD_Angle[JNT_NUM];
	float JOINTCMD_Omega[JNT_NUM];
	float JOINTCMD_Time[JNT_NUM];
	float Angle0[JNT_NUM] = { -0.7854,1.1854,-0.6896,0.8605,-0.7854,1.1854,-0.6896,0.8605 };//起始关节角
	float Omega0[JNT_NUM] = { 0, 0, 0, 0, 0, 0, 0,0 };//当前关节角速度
	double hip[4], knee[4];
	FILE *fp1;
	fp1 = fopen("catkin_ws/src/testcan/log/angle2.txt", "w");
	if (fp1 == NULL)
	{
		printf("CANNOT open the gps.txt file\n");
		exit(0);
	}
	else
	{
		printf("SUCCESS open the file\n");
	}
	//1-4-2-3
	float fJntP2PNode[41][9] = {
	{ 0   ,-0.7854,1.1854,-0.6896,0.8605,-0.7854,1.1854,-0.6896,0.8605 },
	{ 0.3 ,-0.7189,1.3077,-0.6893,0.8797,-0.7832,1.1826,-0.6903,0.8384 },
	{ 0.6 ,-0.6782,1.3809,-0.6895,0.8991,-0.7773,1.1743,-0.6919,0.8119 },
	{ 0.9 ,-0.6388,1.4509,-0.6902,0.9196,-0.7687,1.1623,-0.6950,0.7769 },//上升3厘米
	{ 1.2 ,-0.3019,1.7549,-0.6916,0.9415,-0.7584,1.1479,-0.7027,0.7169 },
	{ 1.5 ,-0.0360,1.8598,-0.6936,0.9632,-0.7474,1.1325,-0.7105,0.6704 },
	{ 1.8 ,-0.2224,1.3779,-0.6964,0.9848,-0.7368,1.1175,-0.7325,0.5678 },
	{ 2.1 ,-0.5215,0.8357,-0.6997,1.0045,-0.7274,1.1038,-0.7371,0.5494 },//开始下降
	{ 2.4 ,-0.5826,0.7280,-0.7036,1.0236,-0.7194,1.0908,-0.7533,0.4893 },
	{ 2.7 ,-0.6506,0.6247,-0.7049,1.0449,-0.7129,1.0774,-0.7707,0.4308 },
	{ 3.0 ,-0.7854,0.3851,-0.7081,1.0624,-0.7081,1.0624,-0.7854,0.3851 },//与地面接触
	{ 3.3 ,-0.7707,0.4308,-0.7129,1.0774,-0.7049,1.0449,-0.6506,0.6247 },
	{ 3.6 ,-0.7533,0.4893,-0.7194,1.0908,-0.7036,1.0236,-0.5826,0.7280 },
	{ 3.9 ,-0.7371,0.5494,-0.7274,1.1038,-0.6997,1.0045,-0.5215,0.8357 },
	{ 4.2 ,-0.7325,0.5678,-0.7368,1.1175,-0.6964,0.9848,-0.2224,1.3779 },
	{ 4.5 ,-0.7105,0.6704,-0.7474,1.1325,-0.6936,0.9632,-0.0360,1.8598 },
	{ 4.8 ,-0.7027,0.7169,-0.7584,1.1479,-0.6916,0.9415,-0.3019,1.7549 },
	{ 5.1 ,-0.6950,0.7769,-0.7687,1.1623,-0.6902,0.9196,-0.6388,1.4509 },
	{ 5.4 ,-0.6919,0.8119,-0.7773,1.1743,-0.6895,0.8991,-0.6782,1.3809 },
	{ 5.7 ,-0.6903,0.8384,-0.7832,1.1826,-0.6893,0.8797,-0.7189,1.3077 },
	{ 6.0 ,-0.6896,0.8605,-0.7854,1.1854,-0.6896,0.8605,-0.7854,1.1854 },
	{ 6.3 ,-0.6893,0.8797,-0.7189,1.3077,-0.6903,0.8384,-0.7832,1.1826 },
	{ 6.6 ,-0.6895,0.8991,-0.6782,1.3809,-0.6919,0.8119,-0.7773,1.1743 },
	{ 6.9 ,-0.6902,0.9196,-0.6388,1.4509,-0.6950,0.7769,-0.7687,1.1623 },
	{ 7.2 ,-0.6916,0.9415,-0.3019,1.7549,-0.7027,0.7169,-0.7584,1.1479 },
	{ 7.5 ,-0.6936,0.9632,-0.0360,1.8598,-0.7105,0.6704,-0.7474,1.1325 },
	{ 7.8 ,-0.6964,0.9848,-0.2224,1.3779,-0.7325,0.5678,-0.7368,1.1175 },
	{ 8.1 ,-0.6997,1.0045,-0.5215,0.8357,-0.7371,0.5494,-0.7274,1.1038 },
	{ 8.4 ,-0.7036,1.0236,-0.5826,0.7280,-0.7533,0.4893,-0.7194,1.0908 },
	{ 8.7 ,-0.7049,1.0449,-0.6506,0.6247,-0.7707,0.4308,-0.7129,1.0774 },
	{ 9.0 ,-0.7081,1.0624,-0.7854,0.3851,-0.7854,0.3851,-0.7081,1.0624 },
	{ 9.3 ,-0.7129,1.0774,-0.7707,0.4308,-0.6506,0.6247,-0.7049,1.0449 },
	{ 9.6 ,-0.7194,1.0908,-0.7533,0.4893,-0.5826,0.7280,-0.7036,1.0236 },
	{ 9.9 ,-0.7274,1.1038,-0.7371,0.5494,-0.5215,0.8357,-0.6997,1.0045 },
	{ 10.2,-0.7368,1.1175,-0.7325,0.5678,-0.2224,1.3779,-0.6964,0.9848 },
	{ 10.5,-0.7474,1.1325,-0.7105,0.6704,-0.0360,1.8598,-0.6936,0.9632 },
	{ 10.8,-0.7584,1.1479,-0.7027,0.7169,-0.3019,1.7549,-0.6916,0.9415 },
	{ 11.1,-0.7687,1.1623,-0.6950,0.7769,-0.6388,1.4509,-0.6902,0.9196 },
	{ 11.4,-0.7773,1.1743,-0.6919,0.8119,-0.6782,1.3809,-0.6895,0.8991 },
	{ 11.7,-0.7832,1.1826,-0.6903,0.8384,-0.7189,1.3077,-0.6893,0.8797 },
	{ 12.0,-0.7854,1.1854,-0.6896,0.8605,-0.7854,1.1854,-0.6896,0.8605 }
	};//二维数组，定义在特定时刻经过的关节角度
	int Npath = 41;//一共设立了5组中间节点		 

	RESTRAINPARA  RestrainPara;//限制参数
							   //JOINTCMD JointCmd;//关节参数
	int i;

	tf = fJntP2PNode[Npath - 1][0];//提取最末端的时间节点做tf

	RestrainPara.Time = tf;
	RestrainPara.MaxVel = 10 * PI / 180;
	RestrainPara.ACC = 0.5f*PI / 180;

	for (i = 0; i<JNT_NUM; i++)
	{

		g_CurrentJntAngle[i] = Angle0[i];//当前的关节角度

	}
    ros::Rate loop_rate(200);
	while (Tx <= tf)
	{
		JointMultiPointPathPlan(Tx, Angle0, Omega0, fJntP2PNode, Npath, RestrainPara, JOINTCMD_Angle, JOINTCMD_Omega, JOINTCMD_Time);
		hip[0] = -(PI / 2 + JOINTCMD_Angle[0]);
		knee[0] = JOINTCMD_Angle[1];
		hip[1] = PI / 2 + JOINTCMD_Angle[2];
		knee[1] = -JOINTCMD_Angle[3];
		hip[2] = PI / 2 + JOINTCMD_Angle[4];
		knee[2] = -JOINTCMD_Angle[5];
		hip[3] = -(PI / 2 + JOINTCMD_Angle[6]);
		knee[3] = JOINTCMD_Angle[7];

        sendangle(11,-hip[0]);
        pos_pub.publish(ip_pos);
		sendangle(17,-hip[3]);
        pos_pub.publish(ip_pos);
		sendangle(15,-hip[1]);
        pos_pub.publish(ip_pos);
		sendangle(14,-hip[2]);
        pos_pub.publish(ip_pos);
        sendangle(16,-knee[0]);
        pos_pub.publish(ip_pos);
        sendangle(12,-knee[3]);
        pos_pub.publish(ip_pos);
		sendangle(10,-knee[1]);
        pos_pub.publish(ip_pos);
        sendangle(13,-knee[2]);
        fprintf(fp1,"%f %f %f %f %f %f %f %f %f\n", Tx, hip[0], knee[0], hip[1], knee[1], hip[2], knee[2], hip[3], knee[3]);
		// printf("%f, %f, %f, %f, %f, %f, %f，%f, %f\n", Tx, JOINTCMD_Angle[0], JOINTCMD_Angle[1], JOINTCMD_Angle[2], JOINTCMD_Angle[3], JOINTCMD_Angle[4], JOINTCMD_Angle[5], JOINTCMD_Angle[6], JOINTCMD_Angle[7]);
		// fprintf(fp1, "%f, %f, %f, %f, %f, %f, %f，%f, %f\n", Tx, JOINTCMD_Angle[0], JOINTCMD_Angle[1], JOINTCMD_Angle[2], JOINTCMD_Angle[3], JOINTCMD_Angle[4], JOINTCMD_Angle[5], JOINTCMD_Angle[6], JOINTCMD_Angle[7]);
		Tx = Tx + dt;
        // loop_rate.sleep();
	}
	fclose(fp1);
    // ifstream infile("catkin_ws/src/testcan/data/angle.txt");
    // int a;
    for(int j = 1;j<10;j++)
    {
        ifstream infile("catkin_ws/src/testcan/data/angle.txt");
        ros::Rate loop_rate(100);
        while (!infile.eof()&&ros::ok())
        {
            infile>>Tx>>hip[0]>>knee[0]>>hip[1]>>knee[1]>>hip[2]>>knee[2]>>hip[3]>>knee[3];
            sendangle(11,-hip[0]);
            pos_pub.publish(ip_pos);
            sendangle(17,-hip[3]);
            pos_pub.publish(ip_pos);
            sendangle(15,-hip[1]);
            pos_pub.publish(ip_pos);
            sendangle(14,-hip[2]);
            pos_pub.publish(ip_pos);
            sendangle(16,-knee[0]);
            pos_pub.publish(ip_pos);
            sendangle(12,-knee[3]);
            pos_pub.publish(ip_pos);
            sendangle(10,-knee[1]);
            pos_pub.publish(ip_pos);
            sendangle(13,-knee[2]);
            pos_pub.publish(ip_pos);
            printf("%f %f %f %f %f %f %f %f %f\n", Tx, hip[0], knee[0], hip[1], knee[1], hip[2], knee[2], hip[3], knee[3]);
            loop_rate.sleep();
        }
        infile.close();
    }
}
