/*
canopenMaster.h
Author:Dayu Liu
Date:2019.3.6
Usage:This program was used to set canopenMaster to achieve canopen protocol.
Email:308756172@qq.com Please contact me if there is any bug in this program.
*/

#ifndef _CANOPENMASTER_TEST_H
#define _CANOPENMASTER_TEST_H

#include "cantest/cantest_pub_sub.h"
#include <stdio.h>

/* NMT Command Specifier, sent by master to change a slave state */
/* ------------------------------------------------------------- */
/* Should not be modified */
#define NMT_Start_Node              0x01
#define NMT_Stop_Node               0x02
#define NMT_Enter_PreOperational    0x80
#define NMT_Reset_Node              0x81
#define NMT_Reset_Comunication      0x82

//define the RPDO used
#define	RPDO1	0x200
#define	RPDO2	0x300
#define	RPDO3	0x400
#define	RPDO4	0x500

//define the TPDO used
#define	TPDO1	0x1A00
#define	TPDO2	0x1A01
#define	TPDO3	0x1A02
#define	TPDO4	0x1A03

//define the SDO command character(CS)
#define upload	0x40

//define when writing obj dictionary, object's data type
#define INTEGER8	0x02
#define INTEGER16	0x03
#define INTEGER24	0x10
#define INTEGER32	0x04
#define UNSIGNED8	0x05
#define UNSIGNED16	0x06
#define UNSIGNED24	0x16
#define UNSIGNED32	0x07

//canopen DSP402 Modes of operation
#define profile_position_mode	1
#define profile_velocity_mode	3
#define profile_torque_mode		4

    int MasterSendNMTstateChange(WORD node_id, BYTE cs);//Main site send command to change NMT state
    int Sync();//Synchronization frame
    int RPDO(WORD node_id, WORD RPDO, BYTE *buffer, int len);//RPDO function
    
    /* Fast SDO implementation*/
    int SDOupload(WORD node_id, WORD index, BYTE sub_index);//SDO upload(read object dictionary)
    int GetSDOuploadResponseData(WORD node_id, WORD index, BYTE sub_index);//get the data that SDO uploaded
    int SDOdownload(WORD node_id, WORD index, BYTE sub_index, BYTE data_type, long data);//start SDO download(write object dictionary)
    
    int CheckSDOdownloadResponse(WORD node_id, WORD index, BYTE sub_index);//check the response from SDO download(write object dictionary)
    bool DisableTPdo(WORD node_id, WORD TPDO);//Ban TPDO
    bool poweron_position_control(WORD node_id);//power-on to reset communication, start Node, set position mode and velocity
    bool poweron_torque_control(WORD node_id);//torque control

#endif