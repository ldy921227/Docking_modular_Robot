#include <stdio.h>
#include <iostream>
#include "cantest/cantest_pub_sub.h"
#include "cantest/pub_sub_class.h"
#include "cantest/candata.h"
#include "ros/ros.h"

//This program is used to subscribe the CAN data through topic cantest_getdata


int main(int argc,char* argv[])
{
    //TPCANRdMsg m;

    ros::init(argc, argv, "cantest_pub");

    candata_pub_sub Ctest;//remember the class candata_pub_sub definition can't put in to while(ros::ok()){} loop 


    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
        Ctest.candata_sub_spinner(1);//start to deal with interrupt, call function candata_msgCallback()
        //print_message_time(&m);
        loop_rate.sleep();

    }
    

    return 0;
}