/*
 set_canopen_arg.cpp
 Author:Dayu Liu
 Date:2019.3.6
 Usage:This program was used to set canopenMaster to achieve canopen protocol.
 Email:308756172@qq.com Please contact me if there is any bug in this program.
 */

#include "cantest/set_canopen_arg.h"
#include <stdio.h>


//extern int count;
//extern int I;
//extern int k;
//extern double  motorvalue[3];

/****************************************************************************
* Function Name  : DisableTPDO
* Description    : disable TPDO
* Input          : none
* Return         : success 0, fail -1
****************************************************************************/
bool DisableTPDO(WORD Node_id)
{
		// disable TPDO1
		if((DisableTPdo(Node_id, TPDO1))!=0)
		{
			printf("dsiableTPDO1fail! \n");
			return -1;
		}

		// disable TPDO2
		if((DisableTPdo(Node_id, TPDO2))!=0)
		{
			printf("dsiableTPDO2fail! \n");
			return -1;
		}
		// disable TPDO3
		if((DisableTPdo(Node_id, TPDO3))!=0)
		{
			printf("dsiableTPDO3fail! \n");
			return -1;
		}

		// disable TPDO4
		if((DisableTPdo(Node_id, TPDO4))!=0)
		{
			printf("dsiableTPDO4fail! \n");
			return -1;
		}

		printf("disable TPDO success!---------------------------------------- \n");
	return 0;
}

/****************************************************************************
* Function Name  : map RPDO1 to sub-index 607A target position  
* Description    : map RPDO1 to sub-index 607A target position
* Input          : none
* Return         : success 0, fail -1
****************************************************************************/
bool MapRPDO1ToTargetposition(WORD Node_id)
{

		// 1 set index 0x1600 sub-index 0 to 0
		SDOdownload(Node_id, 0x1600, 0, UNSIGNED8, 0);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1600, 0)) != 0)
		{
			printf("0x1600sub-index0 set0 SDOdownload is fail! \n");
			return -1;
		}
		// 2 set index 0x1600 sub-index 1 to 0x607A0020
		SDOdownload(Node_id, 0x1600, 1, UNSIGNED32, 0x607A0020);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1600, 1)) != 0)
		{
			printf("0x1600sub-index1 set0x60400010 SDOdownload is fail! \n");
			return -1;
		}
		// 3 set index 0x1600 sub-index 0 to 1
		SDOdownload(Node_id, 0x1600, 0, UNSIGNED8, 1);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1600, 0)) != 0)
		{
			printf("0x1600sub-index0 set2 SDOdownload is fail! \n");
			return -1;
		}
		printf("MapRPDO1ToTargetposition success!---------------------------------------- \n");
	return 0; 
}

/****************************************************************************
* Function Name  : map RPDO3 to sub-index-1 0x6040(default) control word                    
* Description    : map RPDO3 to sub-index-1 0x6040(default) control word                   
* Input          : none
* Return         : success 0, fail -1
****************************************************************************/
bool MapRPDO3ToCONTROLWORD(WORD Node_id)
{

		// 1 set index 0x1602 sub-index 0 to 0
		SDOdownload(Node_id, 0x1602, 0, UNSIGNED8, 0);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1602, 0)) != 0)
		{
			printf("0x1602sub-index0 set0 SDOdownload is fail! \n");
			return -1;
		}

		// 2 set index 0x1602 sub-index 1 to 0x60400010
		/*SDOdownload(Node_id, 0x1602, 1, UNSIGNED32, 0x60400010);
		if((CheckSDOdownloadResponse(Node_id, 0x1602, 1)) != 0)
		{
			printf("0x1602sub-index1 set0x60400010 SDOdownload is fail;");
			return -1;
		}*/
        //3 set index 0x1601 sub-index 2 to 0x60810020
		/*SDOdownload(Node_id, 0x1601, 2, UNSIGNED32, 0x60810020);
		if((CheckSDOdownloadResponse(Node_id, 0x1601, 1)) != 0)
		{
			printf("0x1601sub-index2 set0x60810020 SDOdownload is fail;");
			return -1;
		}*/
		// 4 set index 0x1602 sub-index 0 to 1
		SDOdownload(Node_id, 0x1602, 0, UNSIGNED8, 1);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1602, 0)) != 0)
		{
			printf("0x1602sub-index0 set2 SDOdownload is fail! \n");
			return -1;
		}
		printf("MapRPDO3ToCONTROLWORD success!---------------------------------------- \n");
	return 0;
}
/****************************************************************************
* Function Name  : mapTPDO2
* Description    : map TPDO2 to motor position(0x6064),motor velocity(0x606C),
                   the transfer type is loop synchronization, send after receiving
                   a sync frame
* Input          : none
* Return         : success 0, fail -1
****************************************************************************/
bool MapTPDO2ToActualPositionSpeed(WORD Node_id)
{

		// 1 set index 0x1A01 sub-index 0 to 0
		SDOdownload(Node_id, 0x1A01, 0, UNSIGNED8, 0);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1A01, 0)) != 0)
		{
			printf("0x1A01sub-index0 set0 SDOdownload is fail! \n");
			return -1;
		}

		// 2 set index 0x1A01 sub-index 1 to 0x60640020
		SDOdownload(Node_id, 0x1A01, 1, UNSIGNED32, 0x60640020);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1A01, 1)) != 0)
		{
			printf("0x1A01sub-index1 set0X60640020 SDOdownload is fail! \n");
			return -1;
		}

		// 3 set index 0x1A01 sub-index 2 to 0x606C0020
		SDOdownload(Node_id, 0x1A01, 2, UNSIGNED32, 0x606C0020);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1A01, 2)) != 0)
		{
			printf("0x1A01sub-index2 set0X606C0020 SDOdownload is fail! \n");
			return -1;
		}
		// 4 set index 0x1801 sub-index 2 to 1, PDO would be transmitted on every SYNC message

		SDOdownload(Node_id, 0x1801, 2, UNSIGNED8, 1);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1801, 2)) != 0)
		{
			printf("0x1801sub-index2 set1 SDOdownload is fail! \n");
			return -1;
		}

		// 5 set index 0x1A01 sub-index 0 to 2
		SDOdownload(Node_id, 0x1A01, 0, UNSIGNED8, 2);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1A01, 0)) != 0)
		{
			printf("0x1A01sub-index0 set2 SDOdownload is fail! \n");
			return -1;
		}
		printf("MapTPDO2ToActualPositionSpeed success!---------------------------------------- \n");
	return 0;
}

/****************************************************************************
* Function Name  : MapTPDO1ToStateWord, the default TPDO1 is (0x6041)
* Description    : map TPDO1 to state word(0x6041)
 				   the transfer type is loop synchronization, send after receiving a sync frame
* Input          : none
* Return         : success 0, fail -1
****************************************************************************/
bool MapTPDO1ToStateWord(WORD Node_id)
{

		// 1 disable TPDO1(set index 0x1A00 sub-index 0 to 0)
		SDOdownload(Node_id, 0x1A00, 0, UNSIGNED8, 0);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1A00, 0)) != 0)
		{
			printf("0x1A00sub-index0 set0 SDOdownload is fail! \n");
			return -1;
		}

		// 2 map TPDO1
		// 2.1 map TPDO1 to state word(set index 0x1A00 sub-index 1 to 0x60410010)
		SDOdownload(Node_id, 0x1A00, 1, UNSIGNED32, 0x60410010);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1A00, 1)) != 0)
		{
			printf("0x1A00sub-index1 set0x60410010 SDOdownload is fail! \n");
			return -1;
		}

		// 3 set transmission type to periodic synchronous (set index 0x1800 sub-index 2 to 1)
		//only sub-index 2 can be set
		SDOdownload(Node_id, 0x1800, 2, UNSIGNED8, 1);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1800, 2)) != 0)
		{
			printf("0x1800sub-index2 set1 SDOdownload is fail1 \n");
			return -1;
		}

		// 4 enable the PDO (set index 0x1A00 sub-index 0 to 1)
		SDOdownload(Node_id, 0x1A00, 0, UNSIGNED8, 1);
		read_CAN_data(PCAN_DEVICE);
		if((CheckSDOdownloadResponse(Node_id, 0x1A00, 0)) != 0)
		{
			printf("0x1A00sub-index0 set1 SDOdownload is fail! \n");
			return -1;

		}
		printf("MapTPDO1ToStateWord success!---------------------------------------- \n");
	return 0;
}

/*********************************************************************************
* Function Name  : setControlWord
* Description    : setControlWord and operate mode(define in canopenMaster_test.h)
* Input          : control_word 0x6040 				      
* Return         : On success, the bytes written are returned (zero indicates
* 				   nothing was written). On error, -1 is returned, and error
* 				   is set appropriately.
**********************************************************************************/
int SetControlWord(int control_word,WORD Node_id)
{

		BYTE data[8] = {0};

		data[0] = control_word & 0xFF;
		data[1] = (control_word >> 8) & 0xFF;


		return RPDO(Node_id, RPDO3, data, 2);
}
/*set motor target position 0x607A**************************************/
int SetTargetPosition(int position,WORD Node_id)
{

		BYTE data[8] = {0};

		data[0] = position & 0xFF;
		data[1] = (position >> 8) & 0xFF;
		data[2] = (position >> 16) & 0xFF;
		data[3] = (position >> 24) & 0xFF;

		return RPDO(Node_id, RPDO1, data, 4);
}
/****************************************************************************
 int Clearbuff()
{
     count=0;
     I=0;
     k=0;
     return 0;
}
*/
/****************************************************************************
* Function Name  : initAdmDrive
* Description    : initAdmDrive
* Input          : none
* Return         : success 0, fail -1
****************************************************************************/
bool InitAdmDrive(WORD Node_id)
{
    //1clear buffer
	//Clearbuff();
	//DELAY_US(5000);

	//2 disable TPDO
	if(DisableTPDO(Node_id)!=0)
	{
		printf("Disable TPDO is fail! \n");
		return -1;
	}
    //map RPDO1 to target position
	if(MapRPDO1ToTargetposition(Node_id)!=0)
	{
		printf("MapRPDO1ToTargetposition is fail! \n");
		return -1;
	}

	// 3 map RPDO3 to control word
	if(MapRPDO3ToCONTROLWORD(Node_id)!=0)
	{
		printf("MapRPDO3ToCONTROLWORD is fail! \n");
		return -1;
	}

	//4 map TPDO1 to state word
	if(MapTPDO1ToStateWord(Node_id)!=0)
	{
		printf("MapTPDO1ToStateWord is fail! \n");
		return -1;
	}
	// 5 map TPDO2 to motor position(0x6064), motor velocity(0x606C)
	if(MapTPDO2ToActualPositionSpeed(Node_id)!=0)
	{
		printf("MapTPDO2ToActualPositionSpeed is fail! \n");
		return -1;
	}

	// 6 driver on
	if(SetControlWord(0x0f,Node_id)!=0)
	{
		printf("SetControlWord(0x0f) is fail! \n");
		return -1;
	}
	usleep(5000);//wait for 5 ms

	//SetTargetPosition(motorvalue[Node_id-1],Node_id);

	/*if(SetControlWord(0x1f,Node_id) == -1)
	{
		printf("SetControlWordO(0x1f) is fail! \n");
		return -1;
	}
	DELAY_US(5000);
	if(SetControlWord(0x0f,Node_id) == -1)
	{
		printf("SetControlWord(0x0f) is fail! \n");
		return -1;
	}*/
	
	return 0;
}