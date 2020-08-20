#ifndef __CANOPEN_VCI_ROS_H
#define __CANOPEN_VCI_ROS_H

#include "controlcan.h"
// #include "canopen_vci_ros.h"
#include "ros/ros.h"
#include "testcan/Frame.h"
#include "testcan/IpPos.h"
#include "geometry_msgs/Twist.h"
#include "boost/thread.hpp"
// #include "controllib.h"

#define MOP 0x6060
#define CW 0x6040

#define TP  0x607A 	//Ŀ��λ��
#define TV_V  0x60ff	//���ٶ�ģʽ�µ�Ŀ���ٶ�
#define TV_P  0x6081	//��λ��ģʽ�µ�Ŀ���ٶ�
#define TA  0x6083 //Ŀ����ٶ�
#define TDa 0x6084 //Ŀ����ٶ�

//�ŷ�ģʽ
#define SON 0x0f
#define SOFF 0x07
#define SREADY 0x06
#define Sstart 0x1f

#define PPM 0x01
#define PVM 0x03
#define PTM 0x04
#define HM 0x06
#define IPM 0x07

#define two_encoderatio_s 1440000.0/262144.0
#define two_encoderatio_l 2498560.0/262144.0

#define PI 3.14159265f
#define a_PARAMETER          (0.1265f)               
#define b_PARAMETER          (0.1370f) 
#define R_wheel (0.05075f)

class CANopenVCIROS
{
public:
    CANopenVCIROS();
    virtual ~CANopenVCIROS();
    //ROS消息发布器定义
    ros::Publisher canrecieve_pub,canrecieve2_pub;
    /*****************************************************
    *发送can消息发送
    *输入:    id------CAN ID
    *         CANx----CAN端口
    *       type-----数据类型
    *       pdata[8]------发送数据字符数组,
    *       frame_num-----一次发送的帧数
    ********************************************************/
    static int send(int id, int CANx, int type, const char *pdata,unsigned int frame_num);
    /*******************************************************
    *初始话PDO配置
    ****************************************************/
    int PDO_init(int dev_id);
    /*******************************************************
    *初始话CAN驱动
    ****************************************************/
    void CanDevInit();
    /*******************************************************
    *初始话ros节点
    ****************************************************/
    void rosnode();

    static testcan::Frame can_receive;
    static int motor_init_pos[28];
    // static VCI_CAN_OBJ psend[48];
    // static VCI_CAN_OBJ *psend = send;
    static int init_structure_motor_pos[28];
    /*******************************************************
    *记录电机上电绝对编码器初始位置
    ****************************************************/
    static void GetInitPos(testcan::Frame can_receive);
    /*******************************************************
    *CAN接受线程函数
    ****************************************************/
    static void *receive_func(void* param);

    static void cansendCallback(const testcan::Frame::ConstPtr& canmsg);
    /*******************************************************
    *接受IP_pos topic并转换为位置控制PDO发送
    ****************************************************/
    static void PosCmdCallback(const testcan::IpPos::ConstPtr& cmd);
    /*******************************************************
    *接受底盘cmd_vel指令并转换为CAN指令发送
    ****************************************************/
    static void veltowheelCallback(const geometry_msgs::Twist &vel);
    static void int2char(int dat,char a[]);
    int startRecieveCanThread();
    template <typename sdo>
        char* SDODataPack(sdo data, unsigned short index, char subindex,char (&sdoptr)[8]);    
private:
    void canRecieveThread();
    boost::thread canRecieveThread_;
    VCI_BOARD_INFO pInfo;//用来获取设备信息
    // static VCI_CAN_OBJ send[48];
    // static VCI_CAN_OBJ *psend = send;
    // ros::NodeHandle nh_;
    // ros::Publisher canrecieve_pub;
    // static int motor_init_pos[16];
    ros::Subscriber cansend_sub, ip_pos_sub, cmd_vel_sub;
    // bool enable_leg,enable_arm,enable_wheel,enable_AP_feedback,enable_RP_feedback,enable_I_feedback,enable_V_feedback,enable_wheelspeed_feedback;
    /* data */
};
// int CANopenVCIROS::init_structure_motor_pos[28];
    int CANopenVCIROS::init_structure_motor_pos[28] = {
        27052, //27611,//38534,//61995,//37542,
        258387,//259133,//258396
        216098,//216283,//216265,
        126361,//126354,//147464,//127610,
        119294,//31699,//85491,
        251718,//251963,//261785,//175998,
        24472,//10170,//21034,
        0,
        0,
        202326,
        106257,
        33616,
        249360,
        171466,
        195997,        164837,
        259432,
        0,
        0,
        34049,
        175063,
        120943,
        43647,//193599,
        198955,
        252825,
        235186,
        197955,
        0};



typedef enum{
    WriteSDO = 0x600,
    ReadSDO = 0x580,
    TPDO1 = 0x180,
    TPDO2 = 0x280,
    TPDO3 = 0x380,
    TPDO4 = 0x480,
    RPDO1 = 0x200,
    RPDO2 = 0x300,
    RPDO3 = 0x400,
    WriteML_VEL = 0X00
}CanSendType;

typedef enum{
    CAN1 = 0,
    CAN2 = 1
}CANx;

typedef struct {
  char cmd;	/**< sdo's command */
  short index;		/**< index */
  char subindex;		/**<subindex */
  char data[4]; /**< sdo's datas */
} SDOData;

//明朗驱动器指令
    #define  GID    (0x01)
    #define  GVER   (0x02)
    #define  GDTY   (0x03)
    #define  SADR   (0x04)
    #define  GADR   (0x05)
    #define  GNOD   (0x06)
    #define  SCBD   (0x07)
    #define  GCBD   (0x08)
    #define  GRBD   (0x09)
    #define  BAUD   (0x0A)
    #define  SNOT   (0x0B)
    #define  GNOT   (0x0C)
    #define  SSP    (0x16)//设置电机的最高允许速度（电机编码器相关配置）
    #define  GSP    (0x17)
    #define  SMV    (0x18)//设置电机的最低速度，当速度低于这个值时电机将制动
    #define  GMV    (0x19)
    #define  SPC    (0x1A)
    #define  GPC    (0x1B)
    #define  SCC   ( 0x1C)
    #define  GCC    (0x1D)
    #define  ENC   (0x1E)
    #define  GENC   (0x1F)
    #define  SMOD   (0x2A)//设置驱动器工作模式  0速控模式 256位控模式
    #define  GMOD   (0x2B)

    #define  SEIR   (0x2C)//IR模式
    #define  GEIR   (0x2D)
    #define  SIR    (0x2E)
    #define  GIR    (0x2F)

    #define  SMAV   (0x31)//驱动器各工作模式工作参数设定
    #define  GMAV   (0x32)
    #define  SPH    (0x33)
    #define  GPH    (0x34)
    #define  SPL    (0x35)
    #define  GPL    (0x36)
    #define  STW    (0x37)
    #define  GTW    (0x38)
    #define  SER    (0x39)
    #define  GER    (0x3A)
    #define  SSK    (0x3B)
    #define  GSK    (0x3C)
    #define  SPT    (0x3D)
    #define  GPT    (0x3E)
    #define  SPE    (0x3F)
    #define  GPE    (0x40)
    #define  SPHE   (0x41)
    #define  GPHE   (0x42)
    #define  GLR    (0x43)
    #define  SL     (0x44)
    //#define  SR     (0x45)
    #define  SIPM   (0x4E)
    #define  GIPM   (0x4F)
    #define  SST    (0x54)
    #define  GST    (0x55)
    #define  A      (0x58)//PID调试设置指令
    #define  GA     (0x59)
    #define  AP     (0X5A)
    #define  GAP    (0x5B)
    #define  AI     (0x5C)
    #define  GAI    (0x5D)
    #define  AD     (0x5E)
    #define  GAD    (0x5F)
    #define  P      (0x60)
    #define  GP     (0x61)
    #define  I      (0x62)
    #define  GI     (0x63)
    #define  D      (0x64)
    #define  GD     (0x65)
    #define  MP     (0x66)
    #define  GMP    (0x67)
    #define  MK     (0x68)
    #define  GMK    (0x69)
    #define  MI     (0x6A)
    #define  GMI    (0x6B)
    #define  MD     (0x6C)
    #define  GMD    (0x6D)

    #define  SPOF   (0x75)//设置开机寻找零位指令
    #define  GPOF   (0x76)
    #define  SOV    (0x77)
    #define  GOV    (0x78)
    #define  SOP    (0x79)
    #define  GOP    (0x7A)
    #define  SORG   (0x7B)
    #define  GORG   (0x7C)
    #define  MPO    (0x7D)
    #define  CMPO   (0x7E)

    #define  ENA    (0x80)//驱动器加载电机
    #define  DIS    (0x81)//驱动器释放电机
    #define  ESA    (0x82)
    #define  FCFG   (0x83)
    #define  SBS    (0x84)//软件急停
    #define  CBS    (0x85)//接解除软件急停

    #define  V      (0x90)//设置电机速度
    #define  GV     (0x91)//读取电机速度
    #define  GVE    (0x92)
    #define  GSV    (0x93)

    #define  AM     (0x95)//其他模式控制
    #define  EC     (0x96)

    #define  PO     (0x98)//设置绝对位置 设置电机当前位置为绝对位置 一般不用 因为这里位置用的是编码器磁栅
    #define  M      (0x99)
    #define  MR     (0x9A)
    #define  GM     (0x9B)
    #define  GME    (0x9C)
    #define  GSM    (0x9D)
    #define  SME    (0x9E)
    #define  GSE    (0x9F)

    #define  CPA    (0xC0)
    #define  CPM    (0xC1)
    #define  SNP    (0xC2)
    #define  GNP    (0xC3)
    #define  SLQ    (0xC4)
    #define  GLQ    (0xC5)
    #define  SLD    (0xC6)
    #define  GLD    (0xC7)

    #define  GC     (0xD0)
    #define  GT     (0xD1)
    #define  GEI    (0xD2)
    #define  GSI    (0xD3)



    #define Vmode 1
    #define Pmode 0


#endif
