#include <stdio.h>
#include <PCANBasic.h>
#include "cantest/cantest_pub_sub.h"
#include "cantest/candata.h"
#include "ros/ros.h"


//Create a variable to store can data, candata is a message type we defined before in msg file
cantest::candata can_msg;
double Given_theta[3]={0};
double Position_output_value[3]={0};

//publish CAN data
void publish_candata(TPCANMsg *mr, ros::Publisher candata_pub){
    can_msg.id = mr->ID;
    for(__u8 i = 0;i<8;i++){
        can_msg.data[i] = mr->DATA[i];
    }
    candata_pub.publish(can_msg);
}

//print CAN message type, id, data length 
void print_message(TPCANMsg *m){   
    //%c means singal character, 0x%08x means 123->0x00000123
    //ID is CAN id, LEN is length of CAN data.
    //if CAN message is remote frame, printf 'R'
    printf("receive CAN message: %c %c 0x%08x %1d ",
            ((m->MSGTYPE & MSGTYPE_RTR)? 'r':'m')-((m->MSGTYPE)? 0x20:0),
            (m->MSGTYPE & MSGTYPE_EXTENDED)? 'e':'s',
            m->ID,
            m->LEN);

    //print any frames except remote frames, remote frames only have id.But if you 
    //want to receive extended frames, you need to make CAN bus work in extended first.
    //print CAN data
    if(!(m->MSGTYPE & MSGTYPE_RTR)){
        for(int i=0;i<m->LEN;i++)
            printf("%02x ", m->DATA[i]);
    }

    // PCAN message types define in PCANBasic.h		     
    //								     
    //#define PCAN_MESSAGE_STANDARD        0x00U  // The PCAN message is a CAN Standard Frame (11-bit identifier)
    //#define PCAN_MESSAGE_RTR             0x01U  // The PCAN message is a CAN Remote-Transfer-Request Frame
    //#define PCAN_MESSAGE_EXTENDED        0x02U  // The PCAN message is a CAN Extended Frame (29-bit identifier)
    //#define PCAN_MESSAGE_FD              0x04U  // The PCAN message represents a FD frame in terms of CiA Specs
    //#define PCAN_MESSAGE_BRS             0x08U  // The PCAN message represents a FD bit rate switch (CAN data at a higher bit rate)
    //#define PCAN_MESSAGE_ESI             0x10U  // The PCAN message represents a FD error state indicator(CAN FD transmitter was error active)
    //#define PCAN_MESSAGE_ERRFRAME        0x40U  // The PCAN message represents an error frame
    //#define PCAN_MESSAGE_STATUS          0x80U  // The PCAN message represents a PCAN status message

    printf("MSGTYPE: 0x%02x ", m->MSGTYPE);
    printf("\n");
}

//print a timestamp in msec, read only, wUsec remainder in micro-seconds
void print_message_time(TPCANMsg *mr, TPCANTimestamp *mt){
    printf("%u.%3u ", mt->millis, mt->micros);
    print_message(mr);
}

//this function was used to save CAN data
TPCANMsg* save_candata(TPCANMsg *m){

    static TPCANMsg sdata;
    sdata.ID = m->ID;
    for(__u8 i = 0;i<8;i++){
        sdata.DATA[i] = m->DATA[i];
    }
    sdata.LEN = m->LEN;
    sdata.MSGTYPE = m->MSGTYPE;

    TPCANMsg* p = &sdata;
    return p;
}

//control the position
void AbsolutePositionControl(double give_position[],double Position_output_value[])
{
    //inverse kinematic
	Given_theta[0] = atan(give_position[1]/give_position[0]);
	if(give_position[0]<0)
		Given_theta[0]=Given_theta[0]+PI;
	double temp = sqrt(give_position[0]*give_position[0]+give_position[1]*give_position[1]);
	Given_theta[1] = asin((give_position[2]-L0)/temp) + acos((temp*temp+L1*L1-L2*L2)/(2*L1*temp));
	Given_theta[2] = acos((L1*L1 + L2*L2 - (give_position[0]*give_position[0]+give_position[1]*give_position[1]))/(2*L1*L2)) - PI;

	Given_theta[1] = -Given_theta[1]+PI;//+2*pi
	Given_theta[2] = Given_theta[2];//0.281
	//convert into motor count
	Position_output_value[0] = (Given_theta[0]*ENCODER_RESOLUTION)/(2*PI);
	Position_output_value[1] = (Given_theta[1]*ENCODER_RESOLUTION)/(2*PI);
	Position_output_value[2] = (Given_theta[2]*ENCODER_RESOLUTION)/(2*PI);
}
