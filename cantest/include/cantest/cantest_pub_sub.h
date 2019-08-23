#ifndef _CANTEST_PUB_SUB_H
#define _CANTEST_PUB_SUB_H

#include "ros/ros.h"
#include "cantest/candata.h"
#include <PCANBasic.h>
#include <stdio.h>
#include "cantest/canopenMaster_test.h"
#include "cantest/set_canopen_arg.h"
#include "cantest/can_car_control.h"

#define PCAN_DEVICE PCAN_USBBUS1
//define length of the link unit:mm
#define L0		126.0 
#define L1		160.0
#define L2		167.0

#define ENCODER_RESOLUTION	 4096
#define PI 3.1415926


	//Function declaration
	//reads a message WITH TIMESTAMP from the CAN bus and we can decide whether to 
	//print message on screen and whether to publish the message.
	void print_message(TPCANMsg *m);
	void print_message_time(TPCANMsg *mr, TPCANTimestamp* mt);
	void init();

	void publish_candata(TPCANMsg *mr, ros::Publisher candata_pub);
	int read_online_message(TPCANHandle h);
	int print_online_message(TPCANMsg *mr, TPCANTimestamp* mt);//print online message
	int write_CAN_data(DWORD Write_CANdata_id, BYTE* Write_CANdata, int len);//write CAN data to the CAN bus
	TPCANMsg* save_candata(TPCANMsg *m);//set a data buffer

	bool check_status(TPCANMsg *m);
	int read_CAN_data(TPCANHandle h);//read can data
	int clear_can_queue(TPCANHandle h);//clear can data queue
	void AbsolutePositionControl(double give_position[],double Position_output_value[]);
	
	int read_car_data(TPCANHandle h);//read car information,current,velocity,position!

#endif