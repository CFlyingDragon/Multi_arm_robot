#include "force_sensor/global.h"

float g_fSampleTime_OBC=0.25f;									/*-- ���ݲ�ֵʱ���� --*/
float g_afJntRateMaxDeg_OBC= 10.0f;								/*-- �ؽڽ��ٶ����� -- */
float gOut_fVisEndErr_OBC[6];									/*-- ��ǰ�����Ӿ��ŷ����ѿ����ռ䣩 --*/
float g_afEndRotSpdMax_OBC = 0.08726646259972f;					/*-- ĩ���˶����ٶ����� --*/
float g_afEndVelMax_OBC =0.05f;									/*--  ĩ������˶��ٶ�40mm/s*/
float g_afEndZPosErrDisLimit_OBC = 0.04f;								/*�Ӿ��ŷ�Z�������������*/
float g_afVisKp_OBC[6]= {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f };					/*-- �Ӿ��ŷ����Ʊ���ϵ�� --*/
float g_fCapRunTime_OBC = 0.0f;									/*-- �����Ӿ��ŷ����Ƶ�ִ��ʱ�� --*/
float g_afStartTime_OBC = 3.0f;									/*-- ��������ʼʱ�� --*/
//float g_afVisThreshold_OBC[6] = {0.035f, 0.035f, 0.04f, 0.087266f, 0.087266f,0.087266f};/*-- �Ӿ��ŷ�ץ����Χ --*/
float g_afVisThreshold_OBC[6] = {1e-4f, 1e-4f, 1e-4f, 1e-4f, 1e-4f,1e-4f};/*-- �Ӿ��ŷ�ץ����Χ --*/
float g_afDesiredBethingPose_OBC[6]= {0.0f, 0, 0, 0, 0, 0};		/*-- ��צץ��Ŀ�����λ��ƫ�� --*/
float g_EndHisStepPara[3]={0.0f,0.5f,1.0f};                           /*�Ӿ�����ʱ�Ӳ���ϵ��*/
/* ����ʹ�õ�ȫ�ֱ���*/
float g_EndHisStep1[6]={ 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};				/*��һ�ε�ĩ���˶�����*/
float g_EndHisStep2 [6]={ 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};				/*�϶��ε�ĩ���˶�����*/
float g_EndHisStep3 [6]={ 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};				/*�����ε�ĩ���˶�����*/
float g_afZInnerVisThreshold_OBC = 0.06f;								/*�Ӿ��ŷ�Z���ڲ�ץ����Χ*/
float D_H[JNT_NUM][4] =	{
	// {0,		PI/2,		0,		143.5*0.001},
	// {PI,	PI/2,		0,		77*0.001},
	// {PI,	PI/2,		0,	    329*0.001},
	// {PI,	PI/2,		0,		0},
    // {PI,	PI/2,		0,		286.5*0.001},
	// {PI,	PI/2,		0,		0},
	// {0,		0,			0,		189*0.001}

	// {60.0/180.0*PI,			PI/2,		0,		143.5*0.001},
	// {180.0/180.0*PI,		PI/2,		0,		77*0.001},
	// {180.0/180.0*PI,		PI/2,		0,	    329*0.001},
	// {220.0/180.0*PI,		PI/2,		0,		0},
    // {200.0/180.0*PI,		PI/2,		0,		286.5*0.001},
	// {220.0/180.0*PI,		PI/2,		0,		0},
	// {0.0/180.0*PI,			0,			0,		189*0.001}

	{0.0/180.0*PI,			PI/2,		0,		143.5*0.001},
	{180.0/180.0*PI,		PI/2,		0,		77*0.001},
	{180.0/180.0*PI,		PI/2,		0,	    329*0.001},
	{180.0/180.0*PI,		PI/2,		0,		0},
    {180.0/180.0*PI,		PI/2,		0,		286.5*0.001},
	{180.0/180.0*PI,		PI/2,		0,		0},
	{0.0/180.0*PI,			0,			0,		189*0.001}
};//q,alpha,a,d
// float JointAngmax[JNT_NUM]={180*PI/180, 90*PI/180, 180*PI/180, 90*PI/180, 180*PI/180, 90*PI/180, 180*PI/180};
// float JointAngmin[JNT_NUM]={-180*PI/180, -90*PI/180, -180*PI/180,  -90*PI/180,  -180*PI/180, -90*PI/180, -180*PI/180}; 

float JointAngmax[JNT_NUM]={360*PI/180, 280*PI/180, 360*PI/180, 270*PI/180, 360*PI/180, 270*PI/180, 180*PI/180};
float JointAngmin[JNT_NUM]={0*PI/180, 80*PI/180, 0*PI/180,  90*PI/180,  0*PI/180, 90*PI/180, -180*PI/180}; 
