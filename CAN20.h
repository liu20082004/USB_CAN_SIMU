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
	uint8 cmpCount ;  //比较个数
	uint8 *cmpData ;  //要比较的数据
	uint8 cmpReceCount ;  //回复帧计数
	uint16 *cmpCmdID ;  //回复帧命令索引表
	uint8 cmpCmdSpec ;  //回复命令特殊处理
} compareStruct ;

typedef struct
{
	uint32 id ;  //CANid
	uint16 DLC ;  //回复数据长度
	uint8 *CommandData ;  //回复数据的数据
} CAN20_CommandStruct ;

typedef struct
{
	uint8 cmpOffset ;  //从第几个字节开始比较
	uint32 cmpTotal ;  //比较次数
	uint32 CANid ;  //CANid
	compareStruct *CANCompareTab ;	//比较表
	CAN20_CommandStruct *CANCommandTab ;  //命令表
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
