#ifndef __CONTROLLIB_H
#define __CONTROLLIB_H

#include "can.h"
#include "sdo.h"

//����
#define MOP 0x6060
#define CW 0x6040

#define TP  0x607A 	//Ŀ��λ��
#define TV_V  0x60ff	//���ٶ�ģʽ�µ�Ŀ���ٶ�
#define TV_P  0x6081	//��λ��ģʽ�µ�Ŀ���ٶ�
#define TA  0x6083 //Ŀ����ٶ�
#define TDa 0x6084
#define AP 0x20A0 //Ŀ����ٶ�

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

//明朗驱动器指令
#define  GID    ((u8)0x01)
#define  GVER   ((u8)0x02)
#define  GDTY   ((u8)0x03)
#define  SADR   ((u8)0x04)
#define  GADR   ((u8)0x05)
#define  GNOD   ((u8)0x06)
#define  SCBD   ((u8)0x07)
#define  GCBD   ((u8)0x08)
#define  GRBD   ((u8)0x09)
#define  BAUD   ((u8)0x0A)
#define  SNOT   ((u8)0x0B)
#define  GNOT   ((u8)0x0C)
#define  SSP    ((u8)0x16)//设置电机的最高允许速度（电机编码器相关配置）
#define  GSP    ((u8)0x17)
#define  SMV    ((u8)0x18)//设置电机的最低速度，当速度低于这个值时电机将制动
#define  GMV    ((u8)0x19)
#define  SPC    ((u8)0x1A)
#define  GPC    ((u8)0x1B)
#define  SCC   ( (u8)0x1C)
#define  GCC    ((u8)0x1D)
#define  ENC   ((u8)0x1E)
#define  GENC   ((u8)0x1F)
#define  SMOD   ((u8)0x2A)//设置驱动器工作模式  0速控模式 256位控模式
#define  GMOD   ((u8)0x2B)

#define  SEIR   ((u8)0x2C)//IR模式
#define  GEIR   ((u8)0x2D)
#define  SIR    ((u8)0x2E)
#define  GIR    ((u8)0x2F)

#define  SMAV   ((u8)0x31)//驱动器各工作模式工作参数设定
#define  GMAV   ((u8)0x32)
#define  SPH    ((u8)0x33)
#define  GPH    ((u8)0x34)
#define  SPL    ((u8)0x35)
#define  GPL    ((u8)0x36)
#define  STW    ((u8)0x37)
#define  GTW    ((u8)0x38)
#define  SER    ((u8)0x39)
#define  GER    ((u8)0x3A)
#define  SSK    ((u8)0x3B)
#define  GSK    ((u8)0x3C)
#define  SPT    ((u8)0x3D)
#define  GPT    ((u8)0x3E)
#define  SPE    ((u8)0x3F)
#define  GPE    ((u8)0x40)
#define  SPHE   ((u8)0x41)
#define  GPHE   ((u8)0x42)
#define  GLR    ((u8)0x43)
#define  SL     ((u8)0x44)
//#define  SR     ((u8)0x45)
#define  SIPM   ((u8)0x4E)
#define  GIPM   ((u8)0x4F)
#define  SST    ((u8)0x54)
#define  GST    ((u8)0x55)
#define  A      ((u8)0x58)//PID调试设置指令
#define  GA     ((u8)0x59)
#define  AP     ((u8)0X5A)
#define  GAP    ((u8)0x5B)
#define  AI     ((u8)0x5C)
#define  GAI    ((u8)0x5D)
#define  AD     ((u8)0x5E)
#define  GAD    ((u8)0x5F)
#define  P      ((u8)0x60)
#define  GP     ((u8)0x61)
#define  I      ((u8)0x62)
#define  GI     ((u8)0x63)
#define  D      ((u8)0x64)
#define  GD     ((u8)0x65)
#define  MP     ((u8)0x66)
#define  GMP    ((u8)0x67)
#define  MK     ((u8)0x68)
#define  GMK    ((u8)0x69)
#define  MI     ((u8)0x6A)
#define  GMI    ((u8)0x6B)
#define  MD     ((u8)0x6C)
#define  GMD    ((u8)0x6D)

#define  SPOF   ((u8)0x75)//设置开机寻找零位指令
#define  GPOF   ((u8)0x76)
#define  SOV    ((u8)0x77)
#define  GOV    ((u8)0x78)
#define  SOP    ((u8)0x79)
#define  GOP    ((u8)0x7A)
#define  SORG   ((u8)0x7B)
#define  GORG   ((u8)0x7C)
#define  MPO    ((u8)0x7D)
#define  CMPO   ((u8)0x7E)

#define  ENA    ((u8)0x80)//驱动器加载电机
#define  DIS    ((u8)0x81)//驱动器释放电机
#define  ESA    ((u8)0x82)
#define  FCFG   ((u8)0x83)
#define  SBS    ((u8)0x84)//软件急停
#define  CBS    ((u8)0x85)//接解除软件急停

#define  V      ((u8)0x90)//设置电机速度
#define  GV     ((u8)0x91)//读取电机速度
#define  GVE    ((u8)0x92)
#define  GSV    ((u8)0x93)

#define  AM     ((u8)0x95)//其他模式控制
#define  EC     ((u8)0x96)

#define  PO     ((u8)0x98)//设置绝对位置 设置电机当前位置为绝对位置 一般不用 因为这里位置用的是编码器磁栅
#define  M      ((u8)0x99)
#define  MR     ((u8)0x9A)
#define  GM     ((u8)0x9B)
#define  GME    ((u8)0x9C)
#define  GSM    ((u8)0x9D)
#define  SME    ((u8)0x9E)
#define  GSE    ((u8)0x9F)

#define  CPA    ((u8)0xC0)
#define  CPM    ((u8)0xC1)
#define  SNP    ((u8)0xC2)
#define  GNP    ((u8)0xC3)
#define  SLQ    ((u8)0xC4)
#define  GLQ    ((u8)0xC5)
#define  SLD    ((u8)0xC6)
#define  GLD    ((u8)0xC7)

#define  GC     ((u8)0xD0)
#define  GT     ((u8)0xD1)
#define  GEI    ((u8)0xD2)
#define  GSI    ((u8)0xD3)



#define Vmode 1
#define Pmode 0


u8 SwitchControlMode(CAN_PORT CANx, u8 id, u8 mode);    //дģʽ
u8 SetPos(CAN_PORT CANx, u8 id, int32_t Pos);                //дĿ��λ��
u8 SetVel_posmod(CAN_PORT CANx, u8 id, int32_t vel);           //��λ��ģʽ��дĿ���ٶ�
u8 SetVel_velmod(CAN_PORT CANx, u8 id, int32_t vel);           //���ٶ�ģʽ��дĿ���ٶ�
u8 SetAcc(CAN_PORT CANx, u8 id, int32_t Acc);           //д���ٶ�
u8 SetDacc(CAN_PORT CANx, u8 id, int32_t Dacc);         //д���ٶ�
//u8 SetTOR(CAN_PORT CANx, u8 id);                     
u8 ServerStatus(CAN_PORT CANx, u8 id, uint16_t mode);  //д������
#endif
