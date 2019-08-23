#ifndef _PUB_SUB_CLASS_H
#define _PUB_SUB_CLASS_H
    
#include "ros/ros.h"
#include "cantest/candata.h"
#include "boost/function.hpp"
#include <stdio.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>    
    
    bool close_hand_flag = false;
	bool open_hand_flag = false;

    //class declaration
	class candata_pub_sub{
	public:
	    /*when boost::function was declared the first time , it doesn't save any functions.
		boost::function<void(const cantest::candata::ConstPtr&)> candata_pub_sub_msgCallbackFun
		means candata_pub_sub_msgCallbackFun can save a pointer point to a function and the function
		takes const cantest::candata::ConstPtr& as parameter and returns void.*/
		boost::function<void(const cantest::candata::ConstPtr&)> candata_pub_sub_msgCallbackFun;     
		candata_pub_sub();//default constructor
		//void candata_pub_sub_listener();
		//void candata_pub_sub_publisher(cantest::candata can_data);
		void candata_display(cantest::candata candata_msg, bool set);
		void candata_sub_spinner(char set);

	 private:
		ros::NodeHandle candata_handle;//create a node handle
		ros::Subscriber candata_subscriber;//create a subscriber
		//ros::Publisher candata_publisher;
		ros::SubscribeOptions candata_ops;//subscriberoptions 
		ros::AsyncSpinner *candata_spinner;
		ros::CallbackQueue candata_callbackqueue;
		void candata_msgCallback(const cantest::candata::ConstPtr& notice_msg);//callback function
			
	 };

     //define the constructor function
	 candata_pub_sub::candata_pub_sub(){
		 /*boost::bind and boost::function are always used together.boost::bind can provide 
		 parameter binding*/
		 candata_pub_sub_msgCallbackFun = boost::bind(&candata_pub_sub::candata_msgCallback, this, _1);
		 candata_ops = ros::SubscribeOptions::create<cantest::candata>(
				"CANdata_publish",
				1,
				candata_pub_sub_msgCallbackFun,
				ros::VoidPtr(),
				&candata_callbackqueue    
		 );
		 candata_subscriber = candata_handle.subscribe(candata_ops);
		 candata_spinner = new ros::AsyncSpinner(1,&candata_callbackqueue);

		 //candata_publisher = candata_handle.advertise<cantest::candata>("/notice",1);
	 }

	 /*void candata_pub_sub::candata_pub_sub_listener(){




	 }*/

	 /*void candata_pub_sub::candata_pub_sub_publisher(cantest::candata can_data){
		candata_publisher.publish(can_data);
	 }*/
    
	/*This function print the candata on screen.%u unsigned decimal number.%x unsigned hexadecimal number*/
	 void candata_pub_sub::candata_display(cantest::candata candata_msg, bool set){
		 if(set){
			 printf("REC Notice message,ID: %x , Data: ", candata_msg.id);
		 for(char i = 0;i<8;i++){
			 printf("%x ", candata_msg.data[i]);
			 if(i==7)
			 {
			 printf("\n");
			 }	
		 }    
		}
	 }

     //This callback() function definition
	 void candata_pub_sub::candata_msgCallback(const cantest::candata::ConstPtr& candata_msg){
		 cantest::candata candata_message;//candata definition
		 candata_message.id=0;//candata id initialization
		 //candata initialization
		 for(char i = 0;i<8;i++){
			 candata_message.data[i]=0;
		 }
         //assign the passed arguments to notice_message
		 candata_message.id = candata_msg->id;
		 for(char i = 0;i<8;i++){
			 candata_message.data[i]=candata_msg->data[i];
		 }

         //call the notice_display function to print the candata on the screen
		 candata_pub_sub::candata_display(candata_message, true);

		 if(candata_message.id==1&&candata_message.data[0]==1)
		 {
			 close_hand_flag=true;
		 }
		 if(candata_message.id==1&&candata_message.data[0]==0)
		 {
			 open_hand_flag=true;
		 }
	 }

	 void candata_pub_sub::candata_sub_spinner(char set)
	 {
		 if(set==1)
			 candata_spinner ->start();
		 if(set==0)
			candata_spinner ->stop();
	 }
#endif