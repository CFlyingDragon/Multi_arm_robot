#include "controlcan.h"
/*
ģʽ����
*/
VCI_CAN_OBJ packdata(uint8_t *data)
{
	
}
uint8_t SwitchControlMode(uint8_t CANx, uint8_t id, uint8_t mode){      //ģʽ�л�ѡ��
	SDOData data;
	u8 i,sdodata[8];
	data.cmd = 0x2f;//������
	data.index = MOP;//����
	data.subindex = 0x00;//������
	data.data[0] = mode;
	data.data[1] = 0x00;
	data.data[2] = 0x00;
	data.data[3] = 0x00;
	
	sdodata[0] = data.cmd;
	sdodata[1] = (data.index<<8)>>8;
	sdodata[2] =  data.index>>8;
	sdodata[3] = data.subindex;
	for (i = 4 ; i < 8 ; i++) {
		sdodata[i] =  data.data[i-4];
	}
	return SdoExpeditedDownload(CANx, sdodata, id);
}

u8 SetPos(CAN_PORT CANx, u8 id, int32_t Pos)  //����λ��
{
	SDOData data;
	u8 i,sdodata[8];
	data.cmd = 0x23;//������??
	data.index = TP;//����
	data.subindex = 0x00;//������
	data.data[0] = (Pos<<24)>>24;
	data.data[1] = (Pos<<16)>>24;
	data.data[2] = (Pos<<8)>>24;
	data.data[3] = Pos>>24;
	
	sdodata[0] = data.cmd;
	sdodata[1] = (data.index<<8)>>8;
	sdodata[2] =  data.index>>8;
	sdodata[3] = data.subindex;
	for (i = 4 ; i < 8 ; i++) {
		sdodata[i] =  data.data[i-4];
	}
	return SdoExpeditedDownload(CANx, sdodata, id);

}

u8 SetVel_posmod(CAN_PORT CANx, u8 id, int32_t vel){     //���ٶ�ģʽ��дĿ���ٶ�
	SDOData data;
	u8 i,sdodata[8];
	data.cmd = 0x23;
	data.index = TV_V;
	data.subindex = 0x00;
	data.data[0] = (vel<<24)>>24;
	data.data[1] = (vel<<16)>>24;
	data.data[2] = (vel<<8)>>24;
	data.data[3] = vel>>24;
	
	sdodata[0] = data.cmd;
	sdodata[1] = (data.index<<8)>>8;
	sdodata[2] =  data.index>>8;
	sdodata[3] = data.subindex;
	for (i = 4 ; i < 8 ; i++) {
		sdodata[i] =  data.data[i-4];
	}
	return SdoExpeditedDownload(CANx, sdodata, id);
}

u8 SetVel_velmod(CAN_PORT CANx, u8 id, int32_t vel){     //��λ��ģʽ��дĿ���ٶ�
	SDOData data;
	u8 i,sdodata[8];
	data.cmd = 0x23;
	data.index =TV_P;
	data.subindex = 0x00;
	data.data[0] = (vel<<24)>>24;
	data.data[1] = (vel<<16)>>24;
	data.data[2] = (vel<<8)>>24;
	data.data[3] = vel>>24;
	
	sdodata[0] = data.cmd;
	sdodata[1] = (data.index<<8)>>8;
	sdodata[2] =  data.index>>8;
	sdodata[3] = data.subindex;
	for (i = 4 ; i < 8 ; i++) {
		sdodata[i] =  data.data[i-4];
	}
	return SdoExpeditedDownload(CANx, sdodata, id);
}

u8 SetAcc(CAN_PORT CANx, u8 id, int32_t Acc)  //���ü��ٶ�
{
	SDOData data;
	u8 i,sdodata[8];
	data.cmd=0x23;
	data.index=TA;
  data.subindex = 0x00; 
	
	data.data[0] = (Acc<<24)>>24;
	data.data[1] = (Acc<<16)>>24;
	data.data[2] = (Acc<<8)>>24;
	data.data[3] = Acc>>24;
	
  sdodata[0] =  data.cmd;
	sdodata[1] = (data.index<<8)>>8;
	sdodata[2] =  data.index>>8;
	sdodata[3] = data.subindex;
	for (i = 4 ; i < 8 ; i++) {
		sdodata[i] =  data.data[i-4];
	}
	return SdoExpeditedDownload(CANx, sdodata, id);
}

u8 SetDacc(CAN_PORT CANx, u8 id, int32_t Dacc) //���ü��ٶ�
{
	SDOData data;
	u8 i,sdodata[8];
	data.cmd=0x23;
	data.index=TDa;
  data.subindex = 0x00; 
	
	data.data[0] = (Dacc<<24)>>24;
	data.data[1] = (Dacc<<16)>>24;
	data.data[2] = (Dacc<<8)>>24;
	data.data[3] = Dacc>>24;
	
  sdodata[0] =  data.cmd;
	sdodata[1] = (data.index<<8)>>8;
	sdodata[2] =  data.index>>8;
	sdodata[3] = data.subindex;
	for (i = 4 ; i < 8 ; i++) {
		sdodata[i] =  data.data[i-4];
	}
	return SdoExpeditedDownload(CANx, sdodata, id);
}


u8 ServerStatus(CAN_PORT CANx, u8 id, uint16_t mode){   //����״̬��
	SDOData data;
	u8 i,sdodata[8];
	data.cmd = 0x2b;
	data.index = CW;
	data.subindex = 0x00;
	data.data[0] = (mode<<8)>>8;
	data.data[1] = mode>>8;
	data.data[2] = 0x00;
	data.data[3] = 0x00;
	
	sdodata[0] = data.cmd;
	sdodata[1] = (data.index<<8)>>8;
	sdodata[2] =  data.index>>8;
	sdodata[3] = data.subindex;
	for (i = 4 ; i < 8 ; i++) {
		sdodata[i] =  data.data[i-4];
	}
	return SdoExpeditedDownload(CANx, sdodata, id);
}