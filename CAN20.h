#ifndef __STM32F10x_CAN20_H
#define __STM32F10x_CAN20_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x_can.h"
//#include "CAN20.h"

#ifndef NULL
#define NULL ((void *)0)
#endif

typedef unsigned char uint8 ;
typedef unsigned short int uint16 ;
typedef unsigned int uint32 ;

extern enum USER_statment
{
	NONE,
	CAN_start,
	CAN_stop,
	CAN_set,
	CAN_rece,
} user_statment;

typedef struct
{
	uint8 cmpCount ;  //�Ƚϸ���
	uint8 *cmpData ;  //Ҫ�Ƚϵ�����
	uint8 cmpReceCount ;  //�ظ�֡����
	uint16 *cmpCmdID ;  //�ظ�֡����������
	uint8 cmpCmdSpec ;  //�ظ��������⴦��
} compareStruct ;

typedef struct
{
	uint32 id ;  //CANid
	uint16 DLC ;  //�ظ����ݳ���
	uint8 *CommandData ;  //�ظ����ݵ�����
} CAN20_CommandStruct ;

typedef struct
{
	uint8 cmpOffset ;  //�ӵڼ����ֽڿ�ʼ�Ƚ�
	uint32 cmpTotal ;  //�Ƚϴ���
	uint32 CANid ;  //CANid
	compareStruct *CANCompareTab ;	//�Ƚϱ�
	CAN20_CommandStruct *CANCommandTab ;  //�����
} compareType ;



//void simu_CAN20();
void simu_CAN20_V2();
uint16 CAN20_CompareFunc( const uint8 *recvBuf, const compareType *compareGroup , CAN20_CommandStruct *cmdStruct );
uint8 CAN20_SendFCFrame( uint32 StdId );
uint32 CheckCANStdId( uint32 StdId );
//uint16 CAN20_SendCommand( CAN20_CommandStruct cmdStruct );
uint16 CAN20_SendCommand_V2( CAN20_CommandStruct cmdStruct );
uint16 get_35XX_eeprom_offset( const uint8* recvBuf );
void replace_35XX_recvCmd( CAN20_CommandStruct* cmdStruct , uint16 offset , uint8 subOffset );
void timeOut_Function();

#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_CAN20_H */
