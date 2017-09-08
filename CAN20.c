
#include "CAN20.h"
#include "simu_data.h"

extern CanRxMsg RxMessage_CAN;

static uint16 recvFrameCount = 0 ;  //֡����
static uint16 recvFrameLen = 0 ;  //֡����
static uint16 recvTatolFrame = 0 ;  //��֡��
static uint16 recvDataOffset = 0 ;  //�������ݵ�ƫ��
static uint8 recvData[0x2000] = {0} ;  //�������ݵ�buf
static uint8 isSendNext = 0 ;  //�Ƿ�ȴ�����FC֡
static uint16 sendFrameCount = 0 ;  //֡����
static uint16 sendFrameLen = 0 ;  //֡����
static uint16 sendTatolFrame = 0 ;  //��֡��
static uint16 sendDataOffset = 0 ;  //�������ݵ�ƫ��
static uint16 _35XX_EEPROM_offset = 0 ;  //35XX EEPROM��ƫ��


compareType compareGroup[] = {
	0x04, 0xc4, 0x714, cmp714_Tab, cmd77e_Tab,
	0x04, 0x06, 0x711, cmp711_Tab, cmd77b_Tab,
};


//void simu_CAN20()
//{
//	uint8 i = 0 , CANRecvState = 0 ;
//	uint16 CommandId = 0xffff ;
//	//CanTxMsg TxMessage ;
//	CAN20_CommandStruct commandStruct = {0,0,NULL} ;
//
//
//	//һ�������ж��Ƿ�����
//	if( !recvFrameCount )  //֡����Ϊ0,��Ϊ�ǵ�ǰ��һ֡
//	{
//		//���֡id
//		for( i=0 ; i<4 ; i++ )
//		{
//			*(recvData+i) = (uint8) (RxMessage_CAN.StdId>>(8*(3-i))) ;
//		}
//		memcpy( recvData+4 , RxMessage_CAN.Data , RxMessage_CAN.DLC ) ;
//		recvDataOffset = 4 + RxMessage_CAN.DLC ;
//		//����֡��
//		if( 0x10 == *(recvData+4)&0x10 )
//		{
//			//��֡
//			recvFrameLen = (*(recvData+4)<<0x8)|(*(recvData+5)) ;
//			recvTatolFrame = (recvFrameLen-6)/7 ;  //����ʣ���֡��
//			recvFrameCount++ ;
//			do
//			{
//				CANRecvState = CAN20_SendFCFrame( RxMessage_CAN.StdId ) ;  //����FC֡
//			}while( CAN_TxStatus_NoMailBox==CANRecvState );
//		}
//		else
//		{
//			//��֡
//			recvFrameLen = *(recvData+4) ;
//			CommandId = CAN20_CompareFunc( recvData, compareGroup, &commandStruct ) ;
//			recvTatolFrame = 0 ;  //��֡�Ļ�,����ͽ�������
//			recvFrameCount = 0 ;
//		}
//	}
//	else
//	{
//		//��֡�ĺ���֡
//		memcpy( recvData+recvDataOffset , RxMessage_CAN.Data , RxMessage_CAN.DLC ) ;
//		recvDataOffset += RxMessage_CAN.DLC ;
//		recvFrameCount++ ;
//		if( recvFrameCount==recvTatolFrame )
//		{
//			//һ֡�������
//			CommandId = CAN20_CompareFunc( recvData, compareGroup, &commandStruct ) ;
//			recvTatolFrame = 0 ;  //��֡�Ļ�,����ͽ�������
//			recvFrameCount = 0 ;
//		}
//	}
//
//	//�Ƚ���ɽ��з���
//	if( 0xffff != CommandId )
//	{
//		CAN20_SendCommand( commandStruct ) ;
//	}
//
//	CommandId = 0xffff ;
//	commandStruct.CommandData = NULL ;
//
//}
//
/*
CAN��������
*/
//uint16 CAN20_SendCommand( CAN20_CommandStruct cmdStruct )
//{
//	CanTxMsg TxMessage ;
//	CanRxMsg RxMessage ;
//	uint16 i = 1 , sendTotal = (cmdStruct.DLC-8)/7 , CANRecvState = 0 ;
//	//uint8 DLC = 8 ;
//
//	TxMessage.IDE = CAN_ID_STD ;  //��׼CAN
//	TxMessage.RTR = CAN_RTR_DATA ;  //����֡
//	TxMessage.DLC = 8 ;  //�ȹ̶�Ϊ��֡����
//	TxMessage.StdId = cmdStruct.id ;
//	memcpy( TxMessage.Data , cmdStruct.CommandData , TxMessage.DLC ) ;
//	CAN_Transmit( CAN2, &TxMessage) ;
//
//	if( 0x10 == (TxMessage.Data[0]&0x10) )
//	{
//		CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);                                     ///CAN2��������(����FC֡)
//		if( 0x30 != RxMessage.Data[0] )
//		{
//			return 0 ;
//		}
//	}
//
//	while( i<sendTotal )
//	{
//		TxMessage.DLC = 8 ;  //�ȹ̶�Ϊ��֡����
//		TxMessage.StdId = cmdStruct.id ;
//		TxMessage.Data[0] = 0x20|i ;
//		memcpy( TxMessage.Data , cmdStruct.CommandData+(8+7*(i-1)) , 7 ) ;  //
//		CANRecvState = CAN_Transmit( CAN2, &TxMessage) ;
//		i++ ;
//	}
//	return CANRecvState ;
//}
//

/*
*/
uint16 CAN20_CompareFunc( const uint8 *recvBuf, const compareType *compareGroup , CAN20_CommandStruct *cmdStruct )
{
	// uint8 groupNum = sizeof(compareGroup)/sizeof(compareType) , i = 0 , j = 0 , cmdCount = 0 ;
	uint8 groupNum = 2 , i = 0 , j = 0 , cmdCount = 0 ;
	uint32 recvId = 0 , cmpCount = 0 ;
	uint16 cmdId = 0xffff ;

	for( j=0 ; j<groupNum ; j++ )
	{
		//��ȡid
		for( i=0 ; i<4 ; i++ )
		{
			recvId |= (*(recvBuf+i))<<(8*(3-i)) ;
		}
		if( recvId == compareGroup[j].CANid )
		{
			for( cmpCount=0 ; cmpCount<compareGroup[j].cmpTotal ; cmpCount++ )
			{
				if( !memcmp( recvBuf+compareGroup[j].cmpOffset, compareGroup[j].CANCompareTab[cmpCount].cmpData, compareGroup[j].CANCompareTab[cmpCount].cmpCount ) )
				{
					//�Ƚϵ���ͬ�����������
					cmdCount = compareGroup[j].CANCompareTab[cmpCount].cmpReceCount ;  //��֡������յ��Ĵ���
					i = cmdCount % compareGroup[j].CANCompareTab[cmpCount].cmpCmdID[0] ;
					cmdId = compareGroup[j].CANCompareTab[cmpCount].cmpCmdID[i+1] ;
					//compareGroup[j].CANCompareTab[cmpCount].cmpReceCount = ((cmdCount+1)>compareGroup[j].CANCompareTab[cmpCount].cmpCmdID[0])?0:(cmdCount+1) ;

					if((cmdCount+1)>=compareGroup[j].CANCompareTab[cmpCount].cmpCmdID[0])
					{
						compareGroup[j].CANCompareTab[cmpCount].cmpReceCount = 0 ;
					}
					else
					{
						compareGroup[j].CANCompareTab[cmpCount].cmpReceCount = cmdCount+1 ;
					}

					break ;
				}
			}
			//�ȽϽ���
			if( 0xffff != cmdId )
			{
				//�ҵ���Ӧ������
				*cmdStruct = compareGroup[j].CANCommandTab[cmdId] ;
				switch ( compareGroup[j].CANCompareTab[cmpCount].cmpCmdSpec )
				{
					case 0x01:
						_35XX_EEPROM_offset = get_35XX_eeprom_offset( recvBuf+compareGroup[j].cmpOffset ) ;
						break ;
					case 0x02:
						replace_35XX_recvCmd( &compareGroup[j].CANCommandTab[cmdId] , _35XX_EEPROM_offset , *(recvBuf+6) ) ;
						break ;
					default:
						break ;
				}


			}
			break ;
		}
	}
	return cmdId ;
};

/*
*/
uint8 CAN20_SendFCFrame( uint32 StdId )
{
	CanTxMsg TxMessage;
	/*uint32 cmpOffset = 0 , FCId = 0 ;*/
	uint8 data[8] = { 0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } ;

	TxMessage.IDE = CAN_ID_STD ;  //��׼CAN
	TxMessage.RTR = CAN_RTR_DATA ;  //����֡
	TxMessage.StdId = CheckCANStdId( StdId ) ;
	TxMessage.DLC = 8 ;  //�̶���8���ֽ���
	memcpy( TxMessage.Data , data , TxMessage.DLC ) ;
	return CAN_Transmit( CAN2, &TxMessage) ;
};

uint32 CheckCANStdId( uint32 StdId )
{
	uint32 id = 0 ;
	switch(StdId)
	{
		case 0x7e0:
			id = 0x7e8 ;
			break ;
		case 0x241:
			id = 0x641 ;
			break ;
		//����4��
		case 0x714:
			id = 0x77e ;
			break ;
		case 0x711:
			id = 0x77b ;
			break ;
	}
	return id ;
}


void simu_CAN20_V2()
{
	uint8 i = 0 , CANRecvState = 0 ;
	uint16 CommandId = 0xffff ;
	//CanTxMsg TxMessage ;
	CAN20_CommandStruct commandStruct = {0,0,NULL} ;

	if( 0x30==RxMessage_CAN.Data[0] )
	{
		isSendNext = 1 ;
	}
	else if( (0x10==(RxMessage_CAN.Data[0]&0x10))&&(RxMessage_CAN.Data[0]<=0x1f) )
	{
		//��֡

		//���֡id
		for( i=0 ; i<4 ; i++ )
		{
			*(recvData+i) = (uint8) (RxMessage_CAN.StdId>>(8*(3-i))) ;
		}
		memcpy( recvData+4 , RxMessage_CAN.Data , RxMessage_CAN.DLC ) ;
		recvDataOffset = 4 + RxMessage_CAN.DLC ;
		recvFrameLen = ((*(recvData+4)&0xf)<<0x8)|(*(recvData+5)) ;
		recvTatolFrame = (recvFrameLen-6)/7+1 ;  //����ʣ���֡��
		recvTatolFrame = ((recvFrameLen-6)%7)?(recvTatolFrame+1):(recvTatolFrame) ;

		recvFrameCount++ ;
		do
		{
			CANRecvState = CAN20_SendFCFrame( RxMessage_CAN.StdId ) ;  //����FC֡
		}while( CAN_TxStatus_NoMailBox==CANRecvState );
	}
	else if( (0x20==(RxMessage_CAN.Data[0]&0x20))&&recvTatolFrame )
	{
		memcpy( recvData+recvDataOffset , RxMessage_CAN.Data+1 , RxMessage_CAN.DLC-1 ) ;
		if( 0x2f == RxMessage_CAN.Data[0]&0x2f )
		{
			do
			{
				CANRecvState = CAN20_SendFCFrame( RxMessage_CAN.StdId ) ;  //����FC֡
			}while( CAN_TxStatus_NoMailBox==CANRecvState );
		}
		recvFrameCount++ ;
		recvDataOffset += RxMessage_CAN.DLC-1 ;
	}
	else
	{
		//���֡id
		for( i=0 ; i<4 ; i++ )
		{
			*(recvData+i) = (uint8) (RxMessage_CAN.StdId>>(8*(3-i))) ;
		}
		memcpy( recvData+4 , RxMessage_CAN.Data , RxMessage_CAN.DLC ) ;
		recvFrameCount = 1 ;
		recvTatolFrame = 1 ;
	}

	if( ((recvFrameCount==recvTatolFrame)&&recvTatolFrame)||(isSendNext) )
	{
		//һ֡�������
		CommandId = CAN20_CompareFunc( recvData, compareGroup, &commandStruct ) ;
		recvTatolFrame = 0 ;  //��֡�Ļ�,����ͽ�������
		recvFrameCount = 0 ;

		//�Ƚ���ɽ��з���
		if( 0xffff != CommandId )
		{
			CAN20_SendCommand_V2( commandStruct ) ;
		}
	}

	CommandId = 0xffff ;
	commandStruct.CommandData = NULL ;
}


uint16 CAN20_SendCommand_V2( CAN20_CommandStruct cmdStruct )
{
	CanTxMsg TxMessage ;
	//CanRxMsg RxMessage ;
	uint16 i = 1 , sendTotal = ((cmdStruct.DLC-8)/7)+1 , CANRecvState = 0 ;
	//uint8 DLC = 8 ;

	TxMessage.IDE = CAN_ID_STD ;  //��׼CAN
	TxMessage.RTR = CAN_RTR_DATA ;  //����֡

	if(!isSendNext)
	{
		TxMessage.DLC = 8 ;  //�ȹ̶�Ϊ��֡����
		TxMessage.StdId = cmdStruct.id ;
		memcpy( TxMessage.Data , cmdStruct.CommandData , TxMessage.DLC ) ;
		CAN_Transmit( CAN2, &TxMessage) ;
		return CANRecvState ;
	}

	while( (i<sendTotal)&&(isSendNext) )
	{
		TxMessage.DLC = 8 ;  //�ȹ̶�Ϊ��֡����
		TxMessage.StdId = cmdStruct.id ;
		TxMessage.Data[0] = 0x20|i ;
		memcpy( TxMessage.Data+1 , cmdStruct.CommandData+(8+7*(i-1)) , 7 ) ;  //
		CANRecvState = CAN_Transmit( CAN2, &TxMessage) ;
		i++ ;
	}
	isSendNext = 0 ;
	return CANRecvState ;
}

//35XX����EEPROM���ݵ�ѡ��EEPROMλ��
uint16 get_35XX_eeprom_offset( const uint8* recvBuf )
{
	uint16 offset = 0 ;
	if( (0x23==(*recvBuf))&&(0x01==(*(recvBuf+1)))&&(0x01==(*(recvBuf+3))) )
	{
		switch( *(recvBuf+2) )
		{
			case 0x00 : offset = 0x0000  ; break ;
			case 0x01 : offset = 0x0010  ; break ;
			case 0x02 : offset = 0x00a0  ; break ;
			case 0x03 : offset = 0x0140  ; break ;
			case 0x04 : offset = 0x01B0  ; break ;
			case 0x05 : offset = 0x01F0  ; break ;
			case 0x06 : offset = 0x02D0  ; break ;
			case 0x07 : offset = 0x0320  ; break ;
			case 0x08 : offset = 0x0330  ; break ;
			case 0x09 : offset = 0x03C0  ; break ;
			case 0x0A : offset = 0x04B0  ; break ;
			case 0x0B : offset = 0x04E0  ; break ;
			case 0x0C : offset = 0x04F0  ; break ;
			case 0x0D : offset = 0x0510  ; break ;
			case 0x0E : offset = 0x05F0  ; break ;
			case 0x0F : offset = 0x0640  ; break ;
			case 0x10 : offset = 0x0660  ; break ;
			case 0x11 : offset = 0x0700  ; break ;
			case 0x12 : offset = 0x0790  ; break ;
			case 0x13 : offset = 0x0820  ; break ;
			case 0x14 : offset = 0x0870  ; break ;
			case 0x15 : offset = 0x08A0  ; break ;
			case 0x16 : offset = 0x08E0  ; break ;
			case 0x17 : offset = 0x0930  ; break ;
			case 0x18 : offset = 0x0980  ; break ;
			case 0x19 : offset = 0x0990  ; break ;
			case 0x1A : offset = 0x09B0  ; break ;
			case 0x1B : offset = 0x09C0  ; break ;
			case 0x1C : offset = 0x09E0  ; break ;
			case 0x1D : offset = 0x0A00  ; break ;
			case 0x1E : offset = 0x0A40  ; break ;
			case 0x1F : offset = 0x0A50  ; break ;
			case 0x20 : offset = 0x0A70  ; break ;
			case 0x21 : offset = 0x0a80  ; break ;
			case 0x22 : offset = 0x0AA0  ; break ;
			case 0x23 : offset = 0x0AB0  ; break ;
			case 0x24 : offset = 0x0AC0  ; break ;
			case 0x25 : offset = 0x0AD0  ; break ;
			case 0x26 : offset = 0x0B10  ; break ;
			case 0x27 : offset = 0x0B20  ; break ;
			case 0x28 : offset = 0x0b60  ; break ;
			case 0x29 : offset = 0x0B70  ; break ;
			case 0x2A : offset = 0x0B80  ; break ;
			case 0x2B : offset = 0x0BA0  ; break ;
			case 0x2C : offset = 0x0BB0  ; break ;
			case 0x2D : offset = 0x0BC0  ; break ;
			case 0x2E : offset = 0x0BD0  ; break ;
			case 0x2F : offset = 0x0BE0  ; break ;
			case 0x30 : offset = 0x0BF0  ; break ;
			case 0x31 : offset = 0x0C00  ; break ;
			case 0x32 : offset = 0x0C20  ; break ;
			case 0x33 : offset = 0x0C30  ; break ;
			case 0x34 : offset = 0x0C70  ; break ;
			case 0x35 : offset = 0x0C80  ; break ;
			case 0x36 : offset = 0x0C90  ; break ;
			case 0x37 : offset = 0x0CB0  ; break ;
			case 0x38 : offset = 0x0D10  ; break ;
			case 0x39 : offset = 0x0D20  ; break ;
			case 0x3A : offset = 0x0E10  ; break ;
			case 0x3B : offset = 0x0E20  ; break ;
			case 0x3C : offset = 0x0E30  ; break ;
			case 0x3D : offset = 0x0E70  ; break ;
			case 0x3E : offset = 0x0E80  ; break ;
			case 0x3F : offset = 0x0EA0  ; break ;
			case 0x40 : offset = 0x0EB0  ; break ;
			case 0x41 : offset = 0x0EC0  ; break ;
			case 0x42 : offset = 0x0F20  ; break ;
			case 0x43 : offset = 0x0F50  ; break ;
			case 0x44 : offset = 0x0F80  ; break ;
			case 0x45 : offset = 0x0F90  ; break ;
			case 0x46 : offset = 0x0FC0  ; break ;
			case 0x47 : offset = 0x1030  ; break ;
			case 0x48 : offset = 0x1040  ; break ;
			case 0x49 : offset = 0x1050  ; break ;
			case 0x4A : offset = 0x10D0  ; break ;
			case 0x4B : offset = 0x1100  ; break ;
			case 0x4C : offset = 0x1110  ; break ;
			case 0x4D : offset = 0x1120  ; break ;
			case 0x4E : offset = 0x1130  ; break ;
			case 0x4F : offset = 0x1140  ; break ;
			case 0x50 : offset = 0x1150  ; break ;
			case 0x51 : offset = 0x1160  ; break ;
			case 0x52 : offset = 0x1170  ; break ;
			case 0x53 : offset = 0x1180  ; break ;
			case 0x54 : offset = 0x1190  ; break ;
			case 0x55 : offset = 0x11A0  ; break ;
			case 0x56 : offset = 0x11B0  ; break ;
			case 0x57 : offset = 0x11C0  ; break ;
			case 0x58 : offset = 0x11D0  ; break ;
			case 0x59 : offset = 0x11E0  ; break ;
			case 0x5A : offset = 0x11F0  ; break ;
			case 0x5B : offset = 0x1200  ; break ;
			case 0x5C : offset = 0x1210  ; break ;
			case 0x5D : offset = 0x1220  ; break ;
			case 0x5E : offset = 0x1230  ; break ;
			case 0x5F : offset = 0x1240  ; break ;
			case 0x60 : offset = 0x1250  ; break ;
			case 0x61 : offset = 0x12E0  ; break ;
			case 0x62 : offset = 0x1370  ; break ;
			case 0x63 : offset = 0x13F0  ; break ;
			case 0x64 : offset = 0x1420  ; break ;
			case 0x65 : offset = 0x1430  ; break ;
			case 0x66 : offset = 0x1450  ; break ;
			case 0x67 : offset = 0x1480  ; break ;
			case 0x68 : offset = 0x1500  ; break ;
			case 0x69 : offset = 0x1510  ; break ;
			case 0x6A : offset = 0x1520  ; break ;
			case 0x6B : offset = 0x1530  ; break ;
			case 0x6C : offset = 0x1540  ; break ;
			case 0x6D : offset = 0x1550  ; break ;
			case 0x6E : offset = 0x1560  ; break ;
			default : offset = 0x0000 ; break ;
		}
	}
	return offset ;
}

//35XX����EEPROM���ݵ�ʱ����������滻�Ĺ���
void replace_35XX_recvCmd( CAN20_CommandStruct* cmdStruct , uint16 offset , uint8 subOffset )
{
	cmdStruct->CommandData[2] = subOffset ;
	memcpy( cmdStruct->CommandData+3 , _35XX_eeprom+offset+subOffset , 4 ) ;
}

