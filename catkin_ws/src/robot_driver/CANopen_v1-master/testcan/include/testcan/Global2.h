
#ifndef _Global_H
#define _Global_H

	#define ANGLE_RAD 0.017453292f
	#define RAD_ANGLE 57.29577951f
	
    #define JNT_NUM 8     /*---左机械臂关节角数目--*/
    #define JNT_NUM_L 7     /*---左机械臂关节角数目--*/
    #define JNT_NUM_R 7     /*---右机械臂关节角数目--*/
    #define JNT_NUMS  14   /*---总的关节角数目--*/   


    #define MAX_POINT_NUM 1000   /*--关节运动中最大离线运动节点数-*/ 
    #define MAX_LINE_POINT_NUM 10   /*--直线运动中最大离线运动节点数-*/ 

    #define IMG_L_SHOW_LEFT 0
    #define IMG_R_SHOW_LEFT IMG_L_SHOW_LEFT+IMG_WIDTH

    #define MOUSE_MODE_FREE			0
	#define MOUSE_MODE_TWO			1
	#define MOUSE_MODE_FOUR			2

	#define MOUSE_MODE_TWO_FIR		11
	#define MOUSE_MODE_FOUR_ZERO	20
	#define MOUSE_MODE_FOUR_ONE		21
	#define MOUSE_MODE_FOUR_TWO		22
	#define MOUSE_MODE_FOUR_THR		23

	#define IMG_WIDTH 512
	#define IMG_HIGH  512

 	#define GRAB_IMG_WIDTH 768
	#define GRAB_IMG_HIGH 576
	#define PIXEL_IS_BRIGHT		200		//最大灰度
	#define THRESHOLD_IS_MARK	200		//测量填充阈值
    #define THRESHOLD_IS_MARK_Calib	200
	#define THRESHOLD_IS_CLOSE 10       //用于Region_GrowXwf()
	
	#define Img_Calib_m 7               //标定图像光标点的m，n
	#define Img_Calib_n 7

	#define AREA_ARRAY_MAX_LEN	2000
	
	
	#define PI   3.14159f
    #define M_PI 3.14159
	#define RelPosErr 0.02   /*捕获时位置误差控制，对于捕获运动目标，10mm；对于静止目标，可以设为5mm*/
    #define RelAttErr 2*PI/180       /*捕获时姿态误差控制，对于捕获运动目标，5度；对于静止目标，可以设为1度*/

    #define RelPosErrRVD 0.02   /*捕获时位置误差控制，对于捕获运动目标，10mm；对于静止目标，可以设为5mm*/
    #define RelAttErrRVD 1*PI/180       /*捕获时姿态误差控制，对于捕获运动目标，5度；对于静止目标，可以设为1度*/
	
    #define RelVelErr 0.005   /*捕获时位置速度控制，5mm/s*/	
	#define RelAttWErr 1*PI/180       /*捕获时姿态速度误差控制，1度/s*/
	
	#define RelPosErr2 0.02   /*捕获时位置误差控制，对于捕获运动目标，10mm；对于静止目标，可以设为5mm*/
	#define RelAttErr2 1*PI/180       /*捕获时姿态误差控制，对于捕获运动目标，5度；对于静止目标，可以设为1度*/

	#define LAMDA 0.00             /*用于计算奇异回避的常数*/
	#define dTime 0.005             /*积分步长*/
	#define dOutTime 0.25          /*----动力学模型输出时间，最好是dTime的整数倍------*/
	#define Free_Floating 'A'
	#define Free_Flying 'B'
	#define BaseFixed 'C'
	#define UvErr 10
	#define VIEW_WIDTH 512
	#define VIEW_HEIGHT 512

	#define NImages 20
	#define ChessBoardSize_w 7
	#define ChessBoardSize_h 7
	#define NPoints ChessBoardSize_w*ChessBoardSize_h
	#define gBaseLine_Len 2*220e-3
    
    #define gBaseLine_Len_HE 2*100e-3

    #define TARGET_NUM 4
	#define Chaser_X_LEN 2400    /*---追踪星尺寸--*/
	#define Chaser_Y_LEN 1500
	#define Chaser_Z_LEN 1500
	#define Chaser_CAM_HIGH_LEN 100


    extern float g_DH_a_L[JNT_NUM_L],g_DH_d_L[JNT_NUM_L],g_DH_alpha_L[JNT_NUM_L],g_DH_q0_L[JNT_NUM_L];   /*----左臂的DH参数----*/
	extern float g_DH_a_R[JNT_NUM_R],g_DH_d_R[JNT_NUM_R],g_DH_alpha_R[JNT_NUM_R],g_DH_q0_R[JNT_NUM_R];   /*----右臂的DH参数----*/	
	extern float g_DH_L[JNT_NUM_L][4], g_DH_R[JNT_NUM_R][4];

	extern float g_T_Ref_0L[4][4], g_T_Ref_0R[4][4], g_T_0L_Ref[4][4], g_T_0R_Ref[4][4];
	extern float g_JntAngleReady_L[JNT_NUM_L], g_JntAngleReady_R[JNT_NUM_R];
	extern float g_JntAngleCapPL1_620_L[JNT_NUM_L], g_JntAngleCapPL1_620_R[JNT_NUM_R];/*---620m位置处对应的双臂捕获构型---*/
    extern float g_JntAngleCapPL1_720_L[JNT_NUM_L], g_JntAngleCapPL1_720_R[JNT_NUM_R];/*---720m位置处对应的双臂捕获构型---*/
	extern float g_JntAngleCapPL1_820_L[JNT_NUM_L], g_JntAngleCapPL1_820_R[JNT_NUM_R];/*---820m位置处对应的双臂捕获构型---*/

	extern float g_T_Ref_PL1_600[4][4], g_T_Ref_PL1_500[4][4], g_T_PL1_HL[4][4], g_T_PL1_HR[4][4];  /*--载荷位姿矩阵、载荷手柄相对于载荷体坐标系的位姿态-*/
	extern float g_T_PL0_HL[4][4], g_T_PL0_HR[4][4];
	extern int g_iPayLoadIndex; /*----载荷编号/标识---------------------------*/

	extern float g_JntAngleZheDie_L[JNT_NUM_L], g_JntAngleReady_L[JNT_NUM_L];


	extern float g_CurrentJntAngleWriten_L[1000][JNT_NUM_L+1], g_CurrentJntAngleWriten_R[1000][JNT_NUM_L+1];
    extern float gOut_fDesiredJntAngle_L[JNT_NUM_L];   /*期望关节角度*/
	extern float gOut_fDesiredJntRate_L[JNT_NUM_L];  /*期望关节角速度*/
	extern float gOut_fDesiredJntAcc_L[JNT_NUM_L];  /*期望关节角加速度*/

	extern float gOut_fDesiredJntAngle_R[JNT_NUM_L];   /*期望关节角度*/
	extern float gOut_fDesiredJntRate_R[JNT_NUM_L];  /*期望关节角速度*/
	extern float gOut_fDesiredJntAcc_R[JNT_NUM_L];  /*期望关节角加速度*/

	extern float gOut_fDesiredJntAngle[JNT_NUM_L];   /*期望关节角度*/
	extern float gOut_fDesiredJntRate[JNT_NUM_L];  /*期望关节角速度*/
	extern float gOut_fDesiredJntAcc[JNT_NUM_L];  /*期望关节角加速度*/

	extern float HKSampleTime; 
	extern int g_Runtimes;

	extern float g_cstarTime;
	extern float g_CurrentJntAngleDeg[JNT_NUM], g_CurrentJntRateDeg[JNT_NUM], g_CurrentJntAccDeg[JNT_NUM];
	extern float g_CurrentJntAngleDeg_L[JNT_NUM_L], g_CurrentJntRateDeg_L[JNT_NUM_L], g_CurrentJntAccDeg_L[JNT_NUM_L];
    extern float g_CurrentJntAngleDeg_R[JNT_NUM_R], g_CurrentJntRateDeg_R[JNT_NUM_R], g_CurrentJntAccDeg_R[JNT_NUM_R];
    /*---------加"deg"的单位为度----------*/

	extern float g_CurrentJntAngle_L[JNT_NUM_L], g_CurrentJntRate_L[JNT_NUM_L], g_CurrentJntAcc_L[JNT_NUM_L];
    extern float g_CurrentJntAngle_R[JNT_NUM_R], g_CurrentJntRate_R[JNT_NUM_R], g_CurrentJntAcc_R[JNT_NUM_R];
	extern float g_CurrentJntAngle[JNT_NUM], g_CurrentJntRate[JNT_NUM], g_CurrentJntAcc[JNT_NUM];
	/*---------不加"deg"的单位为弧度----------*/
	
	extern float gOut_fDesiredJntAngleDeg[JNT_NUM_L];   /*期望关节角度*/
	extern float gOut_fDesiredJntRateDeg[JNT_NUM_L];  /*期望关节角速度*/
	extern float gOut_fDesiredJntAccDeg[JNT_NUM_L];  /*期望关节角加速度*/
	 /*---------加"deg"的单位为度----------*/

	extern float gOut_fDesiredJntAngleDeg_L[JNT_NUM_L];   /*期望关节角度*/
	extern float gOut_fDesiredJntRateDeg_L[JNT_NUM_L];  /*期望关节角速度*/
	extern float gOut_fDesiredJntAccDeg_L[JNT_NUM_L];  /*期望关节角加速度*/
	 /*---------加"deg"的单位为度----------*/

	extern float gOut_fDesiredJntAngleDeg_R[JNT_NUM_L];   /*期望关节角度*/
	extern float gOut_fDesiredJntRateDeg_R[JNT_NUM_L];  /*期望关节角速度*/
	extern float gOut_fDesiredJntAccDeg_R[JNT_NUM_L];  /*期望关节角加速度*/
	 /*---------加"deg"的单位为度----------*/
		
	
	extern float g_MissionTimeLen_Total, g_MissionTimeLen;

	extern int g_iJntIndex;
	extern int g_StopFlag;
	extern int g_PayLoadGraspedFlag;




extern int g_PlanningSpaceFlag;
extern float g_cstarTime, g_CurrentJntAngleDeg[JNT_NUM], g_CurrentJntRateDeg[JNT_NUM];
extern float m_SingleJntDomain[2];
extern int m_SingleJntDir;
extern int m_JntMotionFlag;
extern int m_Plan_Arm_flag;   /*---路径规划臂型标志：=1，规划arm_a运动轨迹；=2，规划arm_b运动轨迹；=3，同时规划arm_a和arm_b运动轨迹；--*/
extern float g_mfMultiCartesianMovingData[MAX_POINT_NUM][7];

extern float g_mfMultiJntMovingData[MAX_POINT_NUM][JNT_NUM+1];
extern float g_mfMultiJntMovingData_L[MAX_POINT_NUM][JNT_NUM_L+1];
extern float g_mfMultiJntMovingData_R[MAX_POINT_NUM][JNT_NUM_R+1];
extern float g_mfSingleJntMovingData[MAX_POINT_NUM][2];  


extern float g_mfMultiJntNode2[MAX_POINT_NUM][JNT_NUM+1];  /*---多关节运动结点，805定义，时间在第一列---*/;

extern float g_E2PData_Test_a[5][JNT_NUM_L+1];
extern float g_E2PData_CartesianTest_a[5][7];
extern float g_fPlanRunTime_OBC, g_fPlanRunTime_OBC2;
extern float g_fTskRunTime_OBC;
extern float g_fPlanStartTime_OBC, g_fPlanStartTime_OBC2;
extern int g_iPlanFirstCircle, g_iPlanFirstCircle2;
extern int g_bPlanBeyondTimeOut;
extern float g_fTskStartTime_OBC;
extern int g_iTskFirstCircle;
extern int g_iTskFinish;
extern float g_fJntErrLimit; 
extern float g_fPoseErrLimit;
extern int g_bPlannSuccFlg, g_bPlannSuccFlg_L, g_bPlannSuccFlg_R;
extern float g_afJntP2PNode[100][JNT_NUM+1];/*关节指令集模式节点最后一列为时间*/
extern float g_afCartesianP2PNode[100][JNT_NUM+1]; /*末端位姿指令集模式节点最后一列为时间*/
extern float g_afRealTimeJntDesired[JNT_NUM]; /*实时关节角期望*/
extern float g_afRealTimeCartesianDesired[6]; /*实时末端位姿期望*/

extern float g_pfPoly5Param0[6*JNT_NUM_L], g_pfSplineMoment[MAX_POINT_NUM*JNT_NUM], g_pfPoly5ParamT[6*JNT_NUM];
extern float g_pfSinglePoly5Param0[6], g_pfSingleSplineMoment[MAX_POINT_NUM], g_pfSinglePoly5ParamT[6];
extern float g_fPosSeq[MAX_POINT_NUM*JNT_NUM];
extern float g_fSinglePosSeq[MAX_POINT_NUM];
extern float g_fTimeSeq[MAX_POINT_NUM*JNT_NUM]; 

extern float g_pfPoly5Param0_2[6*JNT_NUM_L], g_pfSplineMoment_2[MAX_POINT_NUM*JNT_NUM_L], g_pfPoly5ParamT_2[6*JNT_NUM_L];
extern float g_pfSinglePoly5Param0_2[6], g_pfSingleSplineMoment_2[MAX_POINT_NUM], g_pfSinglePoly5ParamT_2[6];
extern float g_fPosSeq_2[MAX_POINT_NUM*JNT_NUM_L];
extern float g_fSinglePosSeq_2[MAX_POINT_NUM];
extern float g_fTimeSeq_2[MAX_POINT_NUM*JNT_NUM_L]; 

extern int g_bStartTimeIsZeros;   /*---用于判断时间序列的初始时刻是否为0，若不是，置1----*/
extern int g_bTimeSeqRight;   /*---用于判断时间序列的各节点是否递增，若不是，置1----*/

extern int g_bStartTimeIsZeros2;   /*---用于判断时间序列的初始时刻是否为0，若不是，置1----*/
extern int g_bTimeSeqRight2;   /*---用于判断时间序列的各节点是否递增，若不是，置1----*/

extern unsigned int g_mfPlanArithmetic_OBC;/*当前规划算法标志（0--三次样条插值；1--535插值；2--五次多项式插值；3--梯形插值）*/
extern unsigned int g_afVisArithmetic_OBC; /*当前视觉伺服算法标志*/
extern unsigned int g_afHKTime_OBC[3]; /*中央控制器星上时间*/




extern float g_mfFirstVel[JNT_NUM_L];  /*	路径点的起点速度	*/
extern float g_mfSecondVel[JNT_NUM_L];  /*	路径点的第二点速度*/
extern float g_mfLastVel[JNT_NUM_L];   /*	路径点的终点速度	*/
extern float g_mfRecipSecondVel[JNT_NUM_L];  /*	路径点的倒数第二节点的速度	*/
extern float g_mfFirstAcc[JNT_NUM_L];  /*	路径点的起点加速度	*/
extern float g_mfLastAcc[JNT_NUM_L];   /*	路径点的终点加速度	*/

extern float g_mfSingleJntFirstVel;  /*	路径点的起点速度	*/
extern float g_mfSingleJntSecondVel;  /*	路径点的第二点速度*/
extern float g_mfSingleJntLastVel;   /*	路径点的终点速度	*/
extern float g_mfSingleJntRecipSecondVel;  /*	路径点的倒数第二节点的速度	*/
extern float g_mfSingleJntFirstAcc;  /*	路径点的起点加速度	*/
extern float g_mfSingleJntLastAcc;   /*	路径点的终点加速度	*/

extern int g_bFinishFlag ;
extern int g_bForceStopFlag ;
extern int g_bSettingErrorFlag;
extern int g_cSettingErrorCommandCode;
extern int g_fModeCode ;
extern int g_bCurrentIllEquInitErrorFlag;
extern float g_fSampleTime_OBC;/*数据插值时间间隔*/
extern float g_mfGivenVel[JNT_NUM_L];
extern float g_mfSingleGivenVel;  /*---单关节规划梯形插值用---*/
extern int g_aiPlanNodeNum;
extern int g_aiPlanNodeNum_L;   /*----关节/位姿指令集节点数目------*/
extern int g_aiPlanNodeNum_R;   /*----关节/位姿指令集节点数目------*/
 

	extern void nfInitInterpolationPlan(); 	
	extern void CalEulerVel(float dRbtBaseAtt[3], float wed_I[3], float EulerVel[3]);
	extern void CalEulerVelBody(float ZYX_Euler[3], float w0_0[3], float EulerVel[3]);

	extern float Rbt_math_dot(float v1[3], float v2[3]);
	extern void Rbt_Cross( float u[3], float v[3], float n[3] );
	extern void Rbt_EulerXyz2Tr(float Euler_xyz[3],float TransMtrx[3][3]);
	extern void Rbt_EulerZyx2Tr(float Euler_zyx[3],float TransMtrx[3][3]);
	extern void Rbt_Tr2EulerZyx(float dTr[3][3], float dEulerZyx[3]);	
	extern void Rbt_Tr2EulerXyz(float dTr[3][3], float dEulerXyz1[3], float dEulerXyz2[3]);
	extern void Rbt_MulMtrx( int m, int n, int p, float *A, float *B, float *C );
	extern int Rbt_InvMtrx( float *C, float *IC, int n );
	extern void Rbt_PInvMtrx67( float AA[6][7], float AA_pinv[7][6]);   /*---计算广义逆矩阵， 6X7， m《=n---*/

	
#endif