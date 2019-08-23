/*
 set_canopen_arg.h
 Author:Dayu Liu
 Date:2019.3.6
 Usage:This program was used to set canopenMaster to achieve canopen protocol.
 Email:308756172@qq.com Please contact me if there is any bug in this program.    
 */

#ifndef _SET_CANOPEN_ARG_H
#define _SET_CANOPEN_ARG_H

#include "canopenMaster_test.h"


	bool DisableTPDO(WORD Node_id);	// disable TPDO
	bool MapRPDO1ToTargetposition(WORD Node_id); // map RPDO1 to motor target position 0x607A
	bool MapRPDO3ToCONTROLWORD(WORD Node_id);// map RPDO3 to control word
	bool MapTPDO2ToActualPositionSpeed(WORD Node_id); // map TPDO2 to motor position(0x6064), motor velocity(0x606C)
	bool MapTPDO1ToStateWord(WORD Node_id); // map TPDO1 to state word
    bool InitAdmDrive(WORD Node_id);// init the driver

	int SetControlWord(int control_word,WORD Node_id);// set control word
	int SetTargetPosition(int position,WORD Node_id);	// set motor target position 
	//int Clearbuff();// clear data buffer

#endif /* SET_CANOPEN_ARG_H_ */