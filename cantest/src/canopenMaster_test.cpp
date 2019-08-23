/*
canopenMaster.cpp
Author:Dayu Liu
Date:2019.3.6
Usage:This program was used to set canopenMaster to achieve canopen protocol.
Email:308756172@qq.com Please contact me if there is any bug in this program.
*/


#include "cantest/canopenMaster_test.h"
#include "cantest/candata.h"

//define a bool to check the data right or not
bool data_error_flag;

//TPCANMsg* canp defined in cantest_getdata_pub
extern TPCANMsg* canp;
/*******************************************************************************
* Function Name  : MasterSendNMTstateChange
* Description    : Main site send command to change NMT state.
* Input          : node_id node id, cs NMT command character.
* Return         : On success, the bytes written are returned (zero indicates
* 				   nothing was written). On error, -1 is returned, and error
* 				   is set appropriately.
********************************************************************************/
int MasterSendNMTstateChange(WORD node_id, BYTE cs)
{
	BYTE data[2];
	data[0]=cs;
	data[1]=node_id;

    //0x00 is CAN-ID 000 means NMT network management command
    //NMT node state change command is two bytes, first byte is command character type
    //second byte is being controlled node's id
    //0x01(hex) is start cs, 0x02 is stop cs, 0x80 is pre-optional state, 0x81 is reset
    //node application, 0x82 is reset node communication 
	return write_CAN_data(0x00, data, 2);
}

/*******************************************************************************
* Function Name  : Sync
* Description    : Synchronization frame
* Input          : none
* Return         : On success, the bytes written are returned (zero indicates
* 				   nothing was written). On error, -1 is returned, and error
* 				   is set appropriately.
********************************************************************************/
int Sync()
{
	BYTE data[2] = {0};
    
    //0x80 is Synchronization frame's id, high priority and low transmit time
    //DLC is 0 or 1
	return write_CAN_data(0x80, data, 0);
}

/********************************************************************************
* Function Name  : RPDO
* Description    : Main site(NMT) send RPDO to node, node receive RPDO
* Input          : node_id, RPDO, buffer is data's first address,len
                   is length of data.
* Return         : On success, the bytes written are returned (zero indicates
* 				   nothing was written). On error, -1 is returned, and error
* 				   is set appropriately.
*********************************************************************************/
int RPDO(WORD node_id, WORD RPDO, BYTE *buffer, int len)
{
	return write_CAN_data(RPDO + node_id, buffer, len);
}

/********************************************************************************
* Function Name  : SDOupload
* Description    : Main site start to upload SDO(read object dictionary)
* Input          : node_id, index is the object dictionary's index we want to read,
                   sub_index is sub index.
* Return         : On success, the bytes written are returned (zero indicates
* 				   nothing was written). On error, -1 is returned, and error
* 				   is set appropriately.
*********************************************************************************/
int SDOupload(WORD node_id, WORD index, BYTE sub_index)
{
    //SDO: NMT(client) send CAN-ID 0x600+Node-ID message, Node-ID is service id, data 
    //length is eight bytes

	BYTE data[8] = {0};	//initialize unused SDO data with 0
	WORD cob_id = 0x600 + node_id;	//set the communication object's id

	data[0] = 0x40;					//0x40 is SDO upload command character(SDO get object
                                    // dictionary CS)
	data[1] = index & 0x00ff;		//index low eight
	data[2] = index >> 8;			//index high eight
	data[3] = sub_index;			//sub-index
    //data[4]-data[7] set 0 if do not use 
    
	return write_CAN_data(cob_id, data, 8);//call the write function to write CAN data to the CAN bus
}

/****************************************************************************
* Function Name  : GetSDOuploadResponseData
* Description    : get SDO upload response data
* Input          : the obj dictionary's index, sub_index, node_id
* Return         : data_error_flag is the flag to express GetSDOuploadResponseData
                   if success returns the data.
*****************************************************************************/
int GetSDOuploadResponseData(WORD node_id, WORD index, BYTE sub_index)
{
    //SDO: if service receive success, it will response CAN-ID 0x580+Node-ID message,
    //data length is eight bytes
	int data = 0;
	TPCANMsg s_data;
	BYTE rec_data[8];
	WORD server_id = 0x580 + node_id;

	/*int ret = 0;
	while((ret = ReadFrame()) <= 0)
	{
		if(ret == -1)
		{
			printf("ReadFrame error!");
			data_error_flag = false;
			return 0;
		}
	}*/

	s_data = *canp;//get pointer point to can data from this function

	if(s_data.ID == server_id)
	{       
		for(int i = 0; i < s_data.LEN; ++i)
		{
			rec_data[i] = s_data.DATA[i];
		}

		BYTE cs = rec_data[0];		// get command character
		WORD response_index = (rec_data[2] << 8) | rec_data[1];
		if((response_index == index) && (rec_data[3] == sub_index))
		{
			switch(cs)
			{
				// response a byte
				case 0x4F :	data = rec_data[4];
							data_error_flag = true;
							break;
				// response two byte
				case 0x4B :	data = (rec_data[5] << 8) | rec_data[4];
							data_error_flag = true;
							break;
				// response three byte
				case 0x47 :	data = (rec_data[6] << 16) | (rec_data[5] << 8) | rec_data[4];
							data_error_flag = true;
							break;
				// response four byte
				case 0x43 :	data = (rec_data[7] << 24) | (rec_data[6] << 16) | (rec_data[5] << 8) | rec_data[4];
							data_error_flag = true;
							break;
				// response doesn't distinguish between byte
				case 0x42 :	data = (rec_data[7] << 24) | (rec_data[6] << 16) | (rec_data[5] << 8) | rec_data[4];
							data_error_flag = true;
							break;
				// response abnormal
				case 0x80 : data = (rec_data[7] << 24) | (rec_data[6] << 16) | (rec_data[5] << 8) | rec_data[4];
							data_error_flag = false;
							printf("response abnormal! \n");
							break;
				// cs error
				default:
					printf("cs error! \n");
					data_error_flag= false;
					return -1;
			}				
		}
		else
		{
			data_error_flag= false;
			return -1;
		}
	}
	else
	{
		printf("s_data.id != server_id \n");
		data_error_flag= false;
		return -1;
	}
	return data;//return the data SDO upload(read obj dictionary) response
}

/****************************************************************************
* Function Name  : SDOdownload
* Description    : start SDOdownload(write obj dictionary)
* Input          : node_id, index, sub_index
* 				   data_type, data
* Return         : On success, the bytes written are returned (zero indicates
* 				   nothing was written). On error, -1 is returned, and error
* 				   is set appropriately.
****************************************************************************/
int SDOdownload(WORD node_id, WORD index, BYTE sub_index, BYTE data_type, long data)
{
	//why choose long data?
	//SDO NMT send CAN-ID 0x600+Node-ID to server
	BYTE send_data[8] = {0};
	WORD cob_id = 0x600 + node_id;	// calculate communication object

	send_data[1] = index & 0x00ff;		// index low eight
	send_data[2] = index >> 8;			// index high eight
	send_data[3] = sub_index;			// sub index

	switch(data_type)
	{
		case INTEGER8 :
		case UNSIGNED8 : send_data[0] = 0x2F;	// SDO download
					     send_data[4] = data;	// data need to write
					     break;
		case INTEGER16 :
		case UNSIGNED16 : send_data[0] = 0x2B;	// write two bytes, high byte behind
	     	 	 	 	  send_data[4] = data & 0xFF;	// data need to write
	     	 	 	 	  send_data[5] = (data >> 8) & 0xFF;	// data need to write
	     	 	 	 	  break;
		case INTEGER24 :
		case UNSIGNED24 : send_data[0] = 0x27;	// write three bytes, high byte behind
						  send_data[4] = data & 0xFF;	// data need to write
						  send_data[5] = (data >> 8) & 0xFF;	// data need to write
						  send_data[6] = (data >> 16) & 0xFF;	// data need to write
						  break;
		case INTEGER32 :
		case UNSIGNED32 : send_data[0] = 0x23;	// write three bytes, high byte behind
						  send_data[4] = data & 0xFF;	// data need to write
						  send_data[5] = (data >> 8) & 0xFF;	// data need to write
						  send_data[6] = (data >> 16) & 0xFF;	// data need to write
						  send_data[7] = (data >> 24) & 0xFF;	// data need to write
						  break;
		default :
			printf("no case datatype! \n");
			return 0;
	}
	return write_CAN_data(cob_id, send_data, 8);
}

/****************************************************************************
* Function Name  : SDOdownload
* Description    : start SDO download(write object dictionary)
* Input          : index, sub_index, node_id,
* 				   data_type, data
* Return         : success: 0, fail: -1
****************************************************************************/
int CheckSDOdownloadResponse(WORD node_id, WORD index, BYTE sub_index)	
{	//DELAY_US(5000);DSP delay_time function

	TPCANMsg s_data;
	BYTE rec_data[8];
	WORD server_id = 0x580 + node_id;

	/*int ret = 0;
	while((ret=ReadFrame()) <= 0)
	{
		if(ret == -1)
		{
			return false;
		}
	}*/

	s_data = *canp;//get pointer point to can data from this function
	
	if(s_data.ID == server_id)
	{
		printf("CheckSDOdownloadResponse: ID is 0x%08x ", s_data.ID);
		printf("LEN is %1d ", s_data.LEN);
		for(int i=0;i<s_data.LEN;i++)
            printf("%02x ", s_data.DATA[i]);
		printf("\n");	
		for(int i = 0; i < s_data.LEN; ++i)
		{
			rec_data[i] = s_data.DATA[i];
		}
		//Uchar cs = rec_data[0];		// get command character

		WORD response_index = (rec_data[2] << 8) | rec_data[1];
		if((response_index == index) && (rec_data[3] == sub_index))
		{
			if(rec_data[0] == 0x60)			// data response normal
			{
				printf("write SDO data response normal, write successed! \n");
				return 0;
			}
			else if(rec_data[0] == 0x80)	// response abnormal
				{
					printf("write SDO data response abnormal! \n");
					WORD data = (rec_data[7] << 24) | (rec_data[6] << 16) | (rec_data[5] << 8) | rec_data[4];
					return -1;
				}
				else
				{
					printf("other error! \n");
					return -1;
				}
		}
		else
		{
			printf("(response_index == index) && (rec_data[3] == sub_index) fail! \n");
			return -1;
		}
	}
	else
	{
		printf("s_data.id == server_id fail! \n");
		return -1;
	}
	return 0;
}

/****************************************************************************
* Function Name  : DisablePdo
* Description    : disable PDO
* Input          : node_id,
* 				   TPDO disable
* Return         : success
****************************************************************************/
bool DisableTPdo(WORD node_id, WORD TPDO)
{
	SDOdownload(node_id, TPDO, 0, UNSIGNED8, 0);
	//DELAY_US(5000);
	read_CAN_data(PCAN_DEVICE);
	if(CheckSDOdownloadResponse(node_id, TPDO, 0) != 0)
	{
		printf("CheckSDOdownloadResponse fail! \n");
		return -1;
	}
	return 0;
}

/**************************************************************************** 
This function is used to give power to the driver.position control
*****************************************************************************/
bool poweron_position_control(WORD node_id)
{
	printf("Ready to poweron motor %u \n", node_id);
	usleep(500000);
	//reset communication when power on
	MasterSendNMTstateChange(node_id,0x82);
	read_CAN_data(PCAN_DEVICE);

	//start remote node
	MasterSendNMTstateChange(node_id,0x01);

	//set position mode
	SDOdownload(node_id,0x6060,0,INTEGER8,1);
	read_CAN_data(PCAN_DEVICE);

	//0x6081 set motor velocity 204.8count/s
	SDOdownload(node_id,0x6081,0,UNSIGNED32,0x000800);
	read_CAN_data(PCAN_DEVICE);
	
	return 0;
}

/**************************************************************************** 
This function is used to give power to the driver.torque control
*****************************************************************************/
bool poweron_torque_control(WORD node_id)
{
	printf("Ready to poweron motor %u \n", node_id);
	//reset communication when power on
	MasterSendNMTstateChange(node_id,0x82);
	read_CAN_data(PCAN_DEVICE);

	//start remote node
	MasterSendNMTstateChange(node_id,0x01);

	//set torque mode
	SDOdownload(node_id,0x6060,0,INTEGER8,5);
	read_CAN_data(PCAN_DEVICE);

	//0x6081 set motor velocity 204.8count/s
	SDOdownload(node_id,0x6081,0,UNSIGNED32,0x000800);
	read_CAN_data(PCAN_DEVICE);
	
	return 0;
}