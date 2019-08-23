#include "ros/ros.h"
#include "cantest/cantest_pub_sub.h"
#include "cantest/candata.h"
#include "cantest/pub_sub_class.h"
#include "cantest/set_canopen_arg.h"
#include <stdio.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <PCANBasic.h>
#include <iostream>
#include <asm/types.h>

/*
version:cantest_getdata_pub2.0

Author:Dayu Liu

Date:2019.3.20

Usage:This program was used to get data from CAN-bus and send CAN data to ROS through
topic.CAN data is composed of CAN id and 8 bytes data.
This version's different features from version1.0 are the API we used in program.In 
version1.0, we used libpcan.h and libpcan.so.The new version 2.0 use PCANBasic.h and
PCANBasicbasic.so which have more complete function than libpcan.so.

Email:308756172@qq.com Please contact me if there is any bug in this program.
*/
// Define mapping functions to map the function in PCANBasic.h
// Use typedef to create a new type, do not think it as type alias.If you can't 
// understand function pointer syntax and how to use it, check it out on google.
//*********************************************************************************
// This a new type function pointer point to function 
// TPCANStatus __stdcall CAN_Initialize(
//      TPCANHandle Channel, 
//      TPCANBaudrate Btr0Btr1, 
//      TPCANType HwType _DEF_ARG,
//		DWORD IOPort _DEF_ARG, 
//		WORD Interrupt _DEF_ARG) in PCANBasic.h 
// <summary>
// Initializes a PCAN Channel 
// </summary>
// <param name="Channel">"The handle of a PCAN Channel"</param>
// <param name="Btr0Btr1">"The speed for the communication (BTR0BTR1 code)"</param>
// <param name="HwType">"NON PLUG&PLAY: The type of hardware and operation mode"</param>
// <param name="IOPort">"NON PLUG&PLAY: The I/O address for the parallel port"</param>
// <param name="Interrupt">"NON PLUG&PLAY: Interrupt number of the parallel port"</param>
// <returns>"A TPCANStatus error code"</returns>
// _DEF_ARG = 0 by default 
typedef DWORD(*testCAN_Initialize_TYPE)(
	TPCANHandle Channel, 
	TPCANBaudrate Btr0Btr1);

//*********************************************************************************
// point to function TPCANStatus __stdcall CAN_Uninitialize(TPCANHandle Channel) in PCANBasic.h
// <summary>
// Uninitializes one or all PCAN Channels initialized by CAN_Initialize
// </summary>
// <remarks>Giving the TPCANHandle value "PCAN_NONEBUS", 
// uninitialize all initialized channels</remarks>
// <param name="Channel">"The handle of a PCAN Channel"</param>
// <returns>"A TPCANStatus error code"</returns>
typedef DWORD(*testCAN_Uninitialize_TYPE)(TPCANHandle Channel);

//*********************************************************************************
// point to function TPCANStatus __stdcall CAN_Reset(TPCANHandle Channel) in PCANBasic.h
// <summary>
// Resets the receive and transmit queues of the PCAN Channel  
// </summary>
// <remarks>
// A reset of the CAN controller is not performed.
// </remarks>
// <param name="Channel">"The handle of a PCAN Channel"</param>
// <returns>"A TPCANStatus error code"</returns>
typedef DWORD(*testCAN_Reset_TYPE)(TPCANHandle Channel);

//*********************************************************************************
// point to function TPCANStatus __stdcall CAN_GetStatus(TPCANHandle Channel) in PCANBasic.h
// <summary>
// Gets the current status of a PCAN Channel 
// </summary>
// <param name="Channel">"The handle of a PCAN Channel"</param>
// <returns>"A TPCANStatus error code"</returns>
typedef DWORD(*testCAN_GetStatus_TYPE)(TPCANHandle Channel);

//*********************************************************************************
// point to function TPCANStatus __stdcall CAN_Read(
//        TPCANHandle Channel, 
//        TPCANMsg* MessageBuffer, 
//        TPCANTimestamp* TimestampBuffer) in PCANBasic.h
// <summary>
// Reads a CAN message from the receive queue of a PCAN Channel 
// </summary>
// <param name="Channel">"The handle of a PCAN Channel"</param>
// <param name="MessageBuffer">"A TPCANMsg structure buffer to store the CAN message"</param>
// <param name="TimestampBuffer">"A TPCANTimestamp structure buffer to get 
// the reception time of the message. If this value is not desired, this parameter
// should be passed as NULL"</param>
// <returns>"A TPCANStatus error code"</returns>
typedef DWORD(*testCAN_Read_TYPE)(
	    TPCANHandle Channel, 
        TPCANMsg* MessageBuffer, 
        TPCANTimestamp* TimestampBuffer);

//**********************************************************************************
// point to function TPCANStatus __stdcall CAN_Write(
//        TPCANHandle Channel, 
//        TPCANMsg* MessageBuffer) in PCANBasic.h
// <summary>
// Transmits a CAN message 
// </summary>
// <param name="Channel">"The handle of a PCAN Channel"</param>
// <param name="MessageBuffer">"A TPCANMsg buffer with the message to be sent"</param>
// <returns>"A TPCANStatus error code"</returns>
typedef DWORD(*testCAN_Write_TYPE)(
	    TPCANHandle Channel, 
        TPCANMsg* MessageBuffer);

//**********************************************************************************
// point to function TPCANStatus __stdcall CAN_FilterMessages(
//        TPCANHandle Channel, 
//        DWORD FromID, 
//        DWORD ToID, 
//        TPCANMode Mode) in PCANBasic.h
// <summary>
// Configures the reception filter. 
// </summary>
// <remarks>The message filter will be expanded with every call to 
// this function. If it is desired to reset the filter, please use 
// the CAN_SetValue function</remarks>
// <param name="Channel">"The handle of a PCAN Channel"</param>
// <param name="FromID">"The lowest CAN ID to be received"</param>
// <param name="ToID">"The highest CAN ID to be received"</param>
// <param name="Mode">"Message type, Standard (11-bit identifier) or 
// Extended (29-bit identifier)"</param>
// <returns>"A TPCANStatus error code"</returns>
typedef DWORD(*testCAN_FilterMessages_TYPE)(
	    TPCANHandle Channel, 
        DWORD FromID, 
        DWORD ToID, 
        TPCANMode Mode);

//**********************************************************************************
// point to function TPCANStatus __stdcall CAN_GetValue(
//		  TPCANHandle Channel, 
//        TPCANParameter Parameter,  
//        void* Buffer, 
//        DWORD BufferLength) in PCANBasic.h
// <summary>
// Retrieves a PCAN Channel value
// </summary>
// <remarks>Parameters can be present or not according with the kind 
// of Hardware (PCAN Channel) being used. If a parameter is not available,
// a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
// <param name="Channel">"The handle of a PCAN Channel"</param>
// <param name="Parameter">"The TPCANParameter parameter to get"</param>
// <param name="Buffer">"Buffer for the parameter value"</param>
// <param name="BufferLength">"Size in bytes of the buffer"</param>
// <returns>"A TPCANStatus error code"</returns>
typedef DWORD(*testCAN_GetValue_TYPE)(
	    TPCANHandle Channel, 
        TPCANParameter Parameter,  
        void* Buffer, 
        DWORD BufferLength);

//**********************************************************************************
// point to function TPCANStatus __stdcall CAN_SetValue(
//		  TPCANHandle Channel,
//        TPCANParameter Parameter,
//        void* Buffer,
//		  DWORD BufferLength) in PCANBasic.h
// <summary>
// Configures or sets a PCAN Channel value 
// </summary>
// <remarks>Parameters can be present or not according with the kind 
// of Hardware (PCAN Channel) being used. If a parameter is not available,
// a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
// <param name="Channel">"The handle of a PCAN Channel"</param>
// <param name="Parameter">"The TPCANParameter parameter to set"</param>
// <param name="Buffer">"Buffer with the value to be set"</param>
// <param name="BufferLength">"Size in bytes of the buffer"</param>
// <returns>"A TPCANStatus error code"</returns>
typedef DWORD(*testCAN_SetValue_TYPE)(
        TPCANHandle Channel,
        TPCANParameter Parameter,
        void* Buffer,
		DWORD BufferLength);

//**********************************************************************************
// point to function TPCANStatus __stdcall CAN_GetErrorText(
//		  TPCANStatus Error, 
//        WORD Language, 
//        LPSTR Buffer) in PCANBasic.h
// <summary>
// Returns a descriptive text of a given TPCANStatus error 
// code, in any desired language
// </summary>
// <remarks>The current languages available for translation are: 
// Neutral (0x00), German (0x07), English (0x09), Spanish (0x0A),
// Italian (0x10) and French (0x0C)</remarks>
// <param name="Error">"A TPCANStatus error code"</param>
// <param name="Language">"Indicates a 'Primary language ID'"</param>
// <param name="Buffer">"Buffer for a null terminated char array"</param>
// <returns>"A TPCANStatus error code"</returns>
typedef DWORD(*testCAN_GetErrorText_TYPE)(
        TPCANStatus Error, 
        WORD Language, 
        LPSTR Buffer);

//**********************************************************************************
//define a pointer, it will be used in canopenMaster_test.cpp
TPCANMsg* canp = NULL;  

int iBuffer;
TPCANStatus result;
char strMsg[256];
int setPCAN_value = 0x01U;//PCAN_PARAMETER_ON 
char tMsg[256];
bool status_ok_flag = false;
bool write_data_flag = true;
static int check_count = 0;
TPCANMsg buffer[9] = {0};
static int read_loop_count = 0;
static int save_buffer_count = 0;
static int save_count = 0;
bool poweron_motor1_flag = false;
bool poweron_motor2_flag = false;
bool poweron_motor3_flag = false;
TPCANMsg Smes;//defined in PCANBasic.h 
TPCANTimestamp Smet;//defined in PCANBasic.h
__u32 mstatus;

//arm_control_var
double targetposition[3]={327,0,126};//define target position
double motorvalue[3]={0};
static int value[3]={0};

//car_control_var
short temp_pwm = 1000;
short Temp_Velocity =5;
short Real_Current_Value[4] = {0};
short Real_Velocity_Value[4] = {0};
long Real_Position_Value[4] = {0};
char Real_Online[4] = {0};
char Real_Ctl1_Value[4] = {0};
char Real_Ctl2_Value[4] = {0};

//function declaration
void init();
int read_data_loop(TPCANHandle h, ros::Publisher pub, bool publish_on);
void signal_handler(int signal);
void do_exit(void *file, TPCANHandle h, int error);
void save_buffer(TPCANMsg* m);

//define target device path
#define DEFAULT_NODE "/dev/pcan32"
#define HANDLE void*

//In libpcan.h we can see  #define HANDLE void*
HANDLE pcan_handle = NULL;
HANDLE libm_handle = NULL;

//Use the new type we defined before to define function pointer
testCAN_Initialize_TYPE testCAN_Initialize;
testCAN_Uninitialize_TYPE testCAN_Uninitialize;
testCAN_Reset_TYPE testCAN_Reset;
testCAN_GetStatus_TYPE testCAN_GetStatus;
testCAN_Read_TYPE testCAN_Read;
testCAN_Write_TYPE testCAN_Write;
testCAN_FilterMessages_TYPE testCAN_FilterMessages;
testCAN_GetValue_TYPE testCAN_GetValue;
testCAN_SetValue_TYPE testCAN_SetValue;
testCAN_GetErrorText_TYPE testCAN_GetErrorText;

/**
 * This main function get CAN data from CAN bus and send the data through topic.
 */
int main(int argc, char *argv[])
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "cantest_getdata");

	//We don't know how this function works for now and why we need this function.It
	//looks like this function is related to signal.h .
	init();

	//Use function dlopen to load PCANBasic.so
	//return: a handle point to PCANBasic.so
	//If dlopen() fails for any reason, it returns NULL.
	libm_handle = dlopen("libpcanbasic.so", RTLD_LAZY);

    char *errorInfo;//dlerror() will return a char pointer.

    //get error information from dlerror()
	errorInfo = dlerror();

    //If something wrong happened when open PCANBasic.so, dlerror() would get the error
	//information.
	if(!libm_handle){
		printf("Open lipcanbasic.so error: (%s)\n", errorInfo);
	}

	dlerror();//clear the error information

    //******************************************************************************
    //using dlsym function to get the function address defined in PCANBasic.so
	//get address of function CAN_Initialize 
	testCAN_Initialize = (testCAN_Initialize_TYPE)dlsym(libm_handle, "CAN_Initialize");
	if(!testCAN_Initialize){
		printf("dlsym get the CAN_Initialize address failed:(%s)\n", errorInfo);
	}
	dlerror();//clear the error information

	//get address of function CAN_Uninitialize 
	testCAN_Uninitialize = (testCAN_Uninitialize_TYPE)dlsym(libm_handle, "CAN_Uninitialize");
    if(!testCAN_Uninitialize){
		printf("dlsym get the CAN_Uninitialize address failed:(%s)\n", errorInfo);
	}
	dlerror();//clear the error information

	//get address of function CAN_Close
	testCAN_Reset = (testCAN_Reset_TYPE)dlsym(libm_handle, "CAN_Reset");
	if(!testCAN_Reset){
		printf("dlsym get the CAN_Reset address failed:(%s)\n", errorInfo);
	}
	dlerror();//clear the error information

	//get address of function CAN_GetStatus
	testCAN_GetStatus = (testCAN_GetStatus_TYPE)dlsym(libm_handle, "CAN_GetStatus");
	if(!testCAN_GetStatus){
		printf("dlsym get the CAN_GetStatus address failed:(%s)\n", errorInfo);
	}
	dlerror();//clear the error information

	//get address of function CAN_Read, the function returns the CAN bus status(error code)
	//CAN data will be saved in TPCANRdMsg mes, CAN_Read(PCAN_USBBUS1, &mes, NULL) 
	testCAN_Read = (testCAN_Read_TYPE)dlsym(libm_handle, "CAN_Read");
	if(!testCAN_Read){
		printf("dlsym get the CAN_Read address failed:(%s)\n", errorInfo);
	}
	dlerror();//clear the error information

	//get address of function CAN_Write, the function returns the CAN bus status(error code) 
	testCAN_Write = (testCAN_Write_TYPE)dlsym(libm_handle, "CAN_Write");
	if(!testCAN_Write){
		printf("dlsym get the CAN_Write address failed:(%s)\n", errorInfo);
	}
	dlerror();//clear the error information

	//get address of function CAN_FilterMessages
	testCAN_FilterMessages = (testCAN_FilterMessages_TYPE)dlsym(libm_handle, "CAN_FilterMessages");
	if(!CAN_FilterMessages){
		printf("dlsym get the CAN_FilterMessages address failed:(%s)\n", errorInfo);
	}
	dlerror();//clear the error information

    //get address of function CAN_GetValue
	testCAN_GetValue = (testCAN_GetValue_TYPE)dlsym(libm_handle, "CAN_GetValue");
	if(!testCAN_GetValue){
		printf("dlsym get the CAN_GetValue address failed:(%s)\n", errorInfo);
	}
	dlerror();//clear the error information

	//get address of function CAN_SetValue
	testCAN_SetValue = (testCAN_SetValue_TYPE)dlsym(libm_handle, "CAN_SetValue");
	if(!testCAN_SetValue){
		printf("dlsym get the CAN_SetValue address failed:(%s)\n", errorInfo);
	}
	dlerror();//clear the error information

	//get address of function CAN_GetErrorText
	testCAN_GetErrorText = (testCAN_GetErrorText_TYPE)dlsym(libm_handle, "CAN_GetErrorText");
	if(!CAN_GetErrorText){
		printf("dlsym get the CAN_GetErrorText address failed:(%s)\n", errorInfo);
	}
	dlerror();//clear the error information
	//*****************************************************************************

	//char txt[VERSIONSTRING_LEN];//store information of CAN version
	unsigned short wBTR0BTR1 = PCAN_BAUD_1M;//set the baud rate of CAN bus 1MBits/s
	//int nStandart = CAN_INIT_TYPE_ST;//set CAN message standart frame
	const char *szDeviceName = DEFAULT_NODE;//define const pointer point to device path
	
	//get the value of TPCANParameter PCAN_RECEIVE_STATUS
	result = testCAN_GetValue(PCAN_DEVICE, PCAN_RECEIVE_STATUS, &iBuffer, sizeof(iBuffer));
	if(result != PCAN_ERROR_OK)
	{
		//error happen
		testCAN_GetErrorText(result, 0, strMsg);
		printf("error: %s \n", strMsg);
	}
	else
	{
		printf("PCAN_RECEIVE_STATUS is set %u, incoming messages are forwarded to the user application. \n", iBuffer);
	}

	//Initialize the PCAN channel PCAN_USBBUS1
	if(wBTR0BTR1)
	{
		//baud rate 1MBit/s, PCAN_USBBUS1 initializes the CAN hardware 
		errno = testCAN_Initialize(PCAN_USBBUS1, wBTR0BTR1);
		if(!errno){
			printf("CAN_Initialize success, device info: PCAN_USBBUS1, CAN_BAUD_1M\n");
			}
		else{
			perror("testCAN_Initialize()");//print error message
		}
	}

	//get the pcan channel driver version information PCAN_CHANNEL_VERSION
	result = testCAN_GetValue(PCAN_DEVICE, PCAN_CHANNEL_VERSION, &tMsg, sizeof(tMsg));
	if(result != PCAN_ERROR_OK)
	{
		//error happen
		testCAN_GetErrorText(result, 0, strMsg);
		printf("error: %s \n", strMsg);
	}
	else
	{
		printf("Version information of PCAN Channel driver is %s \n", tMsg);
	}
	
	//get the value of TPCANParameter PCAN_BUSOFF_AUTORESET	
	result = testCAN_GetValue(PCAN_DEVICE, PCAN_BUSOFF_AUTORESET, &iBuffer, sizeof(iBuffer));
    if(result != PCAN_ERROR_OK)
	{
		//error happen
		testCAN_GetErrorText(result, 0, strMsg);
		printf("error: %s \n", strMsg);
	}
	else 
	{
		switch(iBuffer)
		{
			case PCAN_PARAMETER_OFF:
				printf("PCAN_BUSOFF_AUTORESET is set %u, you need to set AUTORESET on. \n", iBuffer);
				//set PCAN_BUSOFF_AUTORESET to PCAN_PARAMETER_ON (0x01U)
				result = testCAN_SetValue(PCAN_DEVICE, PCAN_BUSOFF_AUTORESET, &setPCAN_value,sizeof(setPCAN_value));
				if(result != PCAN_ERROR_OK)
				{
					testCAN_GetErrorText(result, 0, strMsg);
					printf("error: %s \n", strMsg);
				}
				else
				{
					//Reseting the hardware has a duration of ~500 milliseconds.After 
					//receiving the PCAN_ERROR_BUSOFF error, an application should wait
					//that time before trying to read and write again.
					usleep(500000);
					result = testCAN_GetValue(PCAN_DEVICE, PCAN_BUSOFF_AUTORESET, &iBuffer, sizeof(iBuffer));
					printf("PCAN_BUSOFF_AUTORESET is set %u, the PCAN driver can reset automatically the CAN controller of a PCAN channel when a bus-off state is detected.\n", iBuffer);
				}
				break;
			case PCAN_PARAMETER_ON:
				printf("PCAN_BUSOFF_AUTORESET is set PCAN_PARAMETER_ON already.\n");
				break;
		}
	}
	
	mstatus = testCAN_Read(PCAN_DEVICE, &Smes, &Smet);
	print_message_time(&Smes, &Smet);
	if(check_count != 1){
		status_ok_flag = check_status(&Smes);
	}

	if(read_online_message(PCAN_DEVICE) != 0)
	{
		printf("Read_online_message fail!---------------------------------------- \n");
	}

	/*****************************************************************************
									car_control
	Date:2019.6.12
	*****************************************************************************/                                  
	/*CAN_RoboModule_DRV_Reset(0,0);                      //driver reset
	usleep(500000);

	CAN_RoboModule_DRV_Config(0,1,100,0);               //1driver every 100ms send back data
    usleep(200000);                                     //
    CAN_RoboModule_DRV_Config(0,2,100,0);               //2driver every 100ms send back data
    usleep(200000);                                     //
    CAN_RoboModule_DRV_Config(0,3,100,0);               //3driver every 100ms send back data
    usleep(200000);                                     //
    CAN_RoboModule_DRV_Config(0,4,100,0);               //4driver every 100ms send back data
                                                        //
	CAN_RoboModule_DRV_Mode_Choice(0,0,Velocity_Mode);  //
	usleep(500000); 

	usleep(100000);
	printf("Ready to move car!---------------------------------------- \n");*/
	//CAN_RoboModule_DRV_Velocity_Mode(0,1,temp_pwm,Temp_Velocity);
    //CAN_RoboModule_DRV_Velocity_Mode(0,2,temp_pwm,Temp_Velocity);
    //CAN_RoboModule_DRV_Velocity_Mode(0,3,temp_pwm,Temp_Velocity);
    //CAN_RoboModule_DRV_Velocity_Mode(0,4,temp_pwm,Temp_Velocity);

	//*****************************************************************************

	//config the motor and initialize the driver                                  
	//poweron_position_control motor 0x01
	/*
	usleep(50000);
	if(poweron_position_control(0x01) != -1)
	{
		//read_CAN_data(PCAN_DEVICE);
		poweron_motor1_flag = true;
		printf("poweron 0x01 motor success!---------------------------------------- \n");
	}

	//InitAdmDrive 0x01
	if(poweron_motor1_flag != false)
	{
		if(InitAdmDrive(0x01) != 0)
		{
			printf("InitAdmDrive fail! \n");
		}

		printf("InitAdmDrive 0x01 success!---------------------------------------- \n");
	}
	clear_can_queue(PCAN_DEVICE);
	usleep(50000);*/

	//*****************************************************************************
	//config the motor and initialize the driver                                  
	//poweron_position_control motor 0x02
	usleep(50000);
	if(poweron_position_control(0x02) != -1)
	{
		//read_CAN_data(PCAN_DEVICE);
		poweron_motor1_flag = true;
		printf("poweron 0x02 motor success!---------------------------------------- \n");
	}

	//InitAdmDrive 0x01
	if(poweron_motor1_flag != false)
	{
		if(InitAdmDrive(0x02) != 0)
		{
			printf("InitAdmDrive fail! \n");
		}

		printf("InitAdmDrive 0x02 success!---------------------------------------- \n");
	}
	clear_can_queue(PCAN_DEVICE);
	usleep(50000);

	//*****************************************************************************
	//poweron_position_control motor 0x03
	/*
	usleep(50000);
	if(poweron_position_control(0x03) != -1)
	{
		//read_CAN_data(PCAN_DEVICE);
		poweron_motor3_flag = true;
		printf("poweron 0x03 motor success!---------------------------------------- \n");
	}

	//InitAdmDrive 0x03
	if(poweron_motor3_flag != false)
	{
		if(InitAdmDrive(0x03) != 0)
		{
			printf("InitAdmDrive fail! \n");
		}

		printf("InitAdmDrive 0x03 success!---------------------------------------- \n");
	}
	clear_can_queue(PCAN_DEVICE);

	usleep(50000);//wait for 50 ms

	printf("All motor poweron success!---------------------------------------- \n");*/

	//*****************************************************************************
	/*SetTargetPosition(value[0]- 1949,0x01);
	SetTargetPosition(value[1] + 766,0x02);
	SetTargetPosition(value[2]+ 1864 + 1263,0x03);//-980
	printf("Ready to move!---------------------------------------- \n");
	SetControlWord(0x0f,0x01);
	usleep(5000);//wait for 5 ms
	SetControlWord(0x1f,0x01);*/

	/*SetControlWord(0x0f,0x02);
	usleep(5000);//wait for 5 ms
	SetControlWord(0x1f,0x02);*/

	/*SetControlWord(0x0f,0x03);
	usleep(5000);//wait for 5 ms
	SetControlWord(0x1f,0x03);*/

	clear_can_queue(PCAN_DEVICE);

	/*****************************************************************************
									Test_motor_function					

	*****************************************************************************/
	if(SetControlWord(0x0f,0x02)!=0)
	{
		printf("SetControlWord(0x0f) is fail! \n");
		return -1;
	}

	usleep(50000);
	printf("Ready to move motor to 0 position!---------------------------------------- \n");
	SetTargetPosition(0x0, 0x02);
	usleep(50000);
	
	if(SetControlWord(0x1f,0x02) != 0)
	{
		printf("SetControlWordO(0x1f) is fail! \n");
		return -1;
	}
	usleep(50000);
	if(SetControlWord(0x0f,0x02) != 0)
	{
		printf("SetControlWordO(0x0f) is fail! \n");
		return -1;
	}

	usleep(50000);
	printf("Ready to move motor to 0x1000 position, the motor will move 1 circle!---------------------------------------- \n");
	SetTargetPosition(0x1000, 0x02);
	usleep(5000000);

	if(SetControlWord(0x1f,0x02) != 0)
	{
		printf("SetControlWordO(0x1f) is fail! \n");
		return -1;
	}
	usleep(50000);
	if(SetControlWord(0x0f,0x02) != 0)
	{
		printf("SetControlWordO(0x0f) is fail! \n");
		return -1;
	}

	usleep(50000);
	/*printf("Ready to move motor to 0 position again!---------------------------------------- \n");
	SetTargetPosition(0x0, 0x01);
	usleep(50000);

	if(SetControlWord(0x1f,0x01) != 0)
	{
		printf("SetControlWordO(0x1f) is fail! \n");
		return -1;
	}
	usleep(50000);*/

	printf("test-Finish!---------------------------------------- \n");

	//****************************************************************************

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle CAN_handle;
    
	/**
	 * The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.  advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().  Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.  If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	ros::Publisher CANdata_pub = CAN_handle.advertise<cantest::candata>("CANdata_publish", 1);

	ros::Rate loop_rate(20);//20Hz(50ms), when call loop_rate.sleep()
    
	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	int count = 0;
	//begin to receive CAN data and publish the CAN data through topic CANdata_publish
	while (ros::ok())
	{
		//HANDLE CAN_handle, ros::Publisher CANdata_pub, bool display_on, bool publish_on
        read_data_loop(PCAN_DEVICE, CANdata_pub, true);
		printf("loop count is %u \n", count);
		/*****************************************************************************
									Test_motor_function					

		*****************************************************************************/
		/*if(SetControlWord(0x0f,0x01)!=0)
		{
			printf("SetControlWord(0x0f) is fail! \n");
			return -1;
		}

		usleep(50000);
		printf("Ready to move motor to 0 position!---------------------------------------- \n");
		SetTargetPosition(0x0, 0x01);
		usleep(50000);
		
		if(SetControlWord(0x1f,0x01) != 0)
		{
			printf("SetControlWordO(0x1f) is fail! \n");
			return -1;
		}
		usleep(50000);

		if(SetControlWord(0x0f,0x01) != 0)
		{
			printf("SetControlWordO(0x0f) is fail! \n");
			return -1;
		}
		usleep(50000);
		printf("Ready to move motor to 0x1000 position, the motor will move 1 circle!---------------------------------------- \n");
		SetTargetPosition(0x1000, 0x01);
		usleep(50000);

		if(SetControlWord(0x1f,0x01) != 0)
		{
			printf("SetControlWordO(0x1f) is fail! \n");
			return -1;
		}

		printf("While  loop motor test-Finish!---------------------------------------- \n");*/

		ros::spinOnce();//callback periodically

		//************Move the docking mechanism************//
		/*SetTargetPosition(value[0]- 1949,0x01);
		SetTargetPosition(value[1] + 766,0x02);
		SetTargetPosition(value[2]+ 1864 + 1263,0x03);//-980
		/*SetControlWord(0x0f,0x01);
		usleep(5000);//wait for 5 ms
		SetControlWord(0x1f,0x01);

		SetControlWord(0x0f,0x02);
		usleep(5000);//wait for 5 ms
		SetControlWord(0x1f,0x02);

		SetControlWord(0x0f,0x03);
		usleep(5000);//wait for 5 ms
		SetControlWord(0x1f,0x03);*/

		loop_rate.sleep();
		++count;
		//try a method: the circle is every 50ms invoke the function read_data_loop
		//So what we can do in the time interval
		/*if(status_ok_flag != false)
		{
			if(write_data_flag != false)
			{
				if(poweron(0x01)!=1)
				{
					poweron_flag = true;
					printf("poweron 0x01 success! \n");
				}
				write_data_flag =false;
			}
		}*/
	}  	
	return 0;
}

//initialize at program start
void init(){
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
}

//read data loop
int read_data_loop(TPCANHandle h, ros::Publisher pub, bool publish_on)
{
	TPCANMsg mes;//defined in PCANBasic.h 
	TPCANTimestamp met;//defined in PCANBasic.h
    __u32 status;//__u32 defined in pcan.h DWORD __u32
	int read_limit = 0;

	loop_part:do{
					//get CAN data and store in mes, return TPCANStatus code.
					status = testCAN_Read(h, &mes, &met);
					usleep(50000);
					//if status == 0 means no error and get the CAN data normal
					if(status == PCAN_ERROR_OK)
					{
						print_message_time(&mes, &met);
						if(mes.DATA[0]=='#'&&mes.DATA[1]=='A')//'#A' is used to change the endpoint of the docking mechanism
						{
							switch(mes.DATA[2]-1)
							{
								/****************endpoint left****************/
								case 0 : value[0]=value[0]-11;
										motorvalue[0]=value[0];  //Y to left 1mm
										break;
								/****************endpoint up****************/
								case 1 : value[1]=value[1]-11;
										motorvalue[1]=value[1];  //Z to up 1mm
										break;
								/****************endpoint right****************/
								case 2 : value[0]=value[0]+11;
										motorvalue[0]=value[0];  //Y to right 1mm
										break;
								/**************endpoint down**************/
								case 3 : value[1]=value[1]+11;
										motorvalue[1]=value[1];  //Z to down 1mm
										break;
								/**************endpoint right**************/
								case 4 : value[0]=value[0]-33;
										motorvalue[0]=value[0];	 //Y to right 5mm
										break;
								/**************endpoint up**************/
								case 5 : value[1]=value[1]-33;
										motorvalue[1]=value[1];   //Z to up 5mm
										break;
								/**************endpoint right**************/
								case 6 : value[0]=value[0]+33;
										motorvalue[0]=value[0];	 //Y to right 5mm
										break;
								/**************endpoint down**************/
								case 7 : value[1]=value[1]+33;
										motorvalue[1]=value[1];  //Z to down 5mm
										break;
							}
						}
						if(mes.DATA[0]=='#'&&mes.DATA[1]=='B')//'#B' is used for locking device 
						{
							 switch(mes.DATA[2]-1)
							 {
								case 0:SetTargetPosition(-1450,0x05);
										SetControlWord(0x0f,0x05);
										usleep(5000);
										SetControlWord(0x1f,0x05);
									    break;
								case 1:SetTargetPosition(-1647,0x05);
										SetControlWord(0x0f,0x05);
										usleep(5000);
										SetControlWord(0x1f,0x05);
								        break;
							 }
						}
						if(mes.DATA[0]=='#'&&mes.DATA[1]=='C')//'#C'is used for coming function
						{
							
						}

						//publish CAN data through ROS topic
						if(publish_on)
						{
							publish_candata(&mes, pub);
						}
						//save can data to buffer for something
						save_buffer(&mes);
						break;
					}
					if(status == PCAN_ERROR_QRCVEMPTY)
					{
						read_limit += 1;
					}
				}while((status == PCAN_ERROR_QRCVEMPTY)&&(read_limit<20));//if queue is empty, try CAN_Read again until get data

	if(read_limit<20)
	{
		goto loop_part;
	}
	else
	{
		read_loop_count += 1;
		return 0;
	}
	
	//write CAN data test
    /*DWORD test_id = 25;
	BYTE test_data[8] = {123,123,123,123,123,123,123,123};
	int test_len = 8;
	write_CAN_data(test_id, test_data, test_len);*/

}

//signal handler for manual break Ctrl+C
void signal_handler(int signal){
    do_exit(libm_handle, PCAN_DEVICE, 0);
}

//exit, close CAN Hardware first, then close libpcanbasic.so
void do_exit(void *file, TPCANHandle h, int error){
	
	TPCANStatus result;
	char strMsg[256];
    if(h){
        result = testCAN_Uninitialize(h);//close CAN Hardware
		if(result != PCAN_ERROR_OK)
		{
			testCAN_GetErrorText(result, 0, strMsg);
			printf("error: %s \n", strMsg);
		}
		else
		{
			printf("PCAN_USBBUS1 (ch-1) was released! \n");
		}
    }
	else
	{
		printf("\nTPCANHandle is zero.CAN bus test finished (%d).\n\n", error);
	}

    dlclose(file);//close libpcanbasic.so
    exit(error);
}

//write CAN data to the CAN bus
int write_CAN_data(DWORD Write_CANdata_id, BYTE* Write_CANdata, int len)
{
	//*****************************************************************************
	//TPCANMsg is defined in pcan.h, it's a struct type.
    //	typedef struct pcan_msg {
	//	DWORD ID;              // 11/29 bit code
	//	BYTE  MSGTYPE;         // bits of MSGTYPE_*
	//	BYTE  LEN;             // count of data bytes (0..8)
	//	BYTE  DATA[8];         // data bytes, up to 8
    //	} TPCANMsg;              // for PCAN_WRITE_MSG
	//BYTE type is the unsigned char
	//WORD is unsigned int  
	//DWORD is unsigned long int

	TPCANMsg wm;//defined in pcan.h TPCANMsg is used for PCAN_WRITE_MSG 
	wm.ID = Write_CANdata_id;//25 is 19 in hexadecimal
	wm.MSGTYPE = 0x00;
	wm.LEN = len;
	for(int i = 0; i < len; i++){
		wm.DATA[i] = Write_CANdata[i];
	}
	//write CAN data to the CAN bus, polling
	if(testCAN_Write(PCAN_DEVICE, &wm)){
		perror("testCAN_Write receive: ");//print error information
		return errno;
	}
	return 0;
}

//check PCAN STATUS the first time,when CAN initialize success, CAN bus will return 
//PCAN_ERROR_OK len is 4, DATA is 00, ID is 1
bool check_status(TPCANMsg *m)
{

	if((m->MSGTYPE==0x80)&&(m->LEN==4)&&(m->ID==1))
	{
		for(int i = 0;i<4;i++)
		{
			if(m->DATA[i] != 0)
			{
				printf("status error occurs! \n");
				return false;
			}
		}
	}
	check_count = 1;
	printf("CAN-bus test OK! Start to communication with CAN-bus! \n");
	return true;
}

//save can data to buffer
void save_buffer(TPCANMsg *m)
{		
	if(m->MSGTYPE == 0x00)
	{
		if(save_count < 10)
		{
			buffer[save_count].ID = m->ID;
			buffer[save_count].LEN = m->LEN;
			for(int i=0; i<m->LEN; i++)
			{
				buffer[save_count].DATA[i] = m->DATA[i]; 
			}
		}
		else
		{
			save_count = 0;
			buffer[save_count].ID = m->ID;
			buffer[save_count].LEN = m->LEN;
			for(int i=0; i<m->LEN; i++)
			{
				buffer[save_count].DATA[i] = m->DATA[i]; 
			}
		}
		save_count += 1;
	}
	save_buffer_count += 1;
}

//read can data
int read_CAN_data(TPCANHandle h)
{
	TPCANMsg mes;//defined in PCANBasic.h
	TPCANTimestamp met;//defined in PCANBasic.h 
    __u32 status;//__u32 defined in pcan.h DWORD __u32
	int read_limit = 0;

	loop_part:do{
					//get CAN data and store in mes, return TPCANStatus code.
					status = testCAN_Read(h, &mes, &met);
					usleep(50000);
					//if status == 0 means no error and get the CAN data normal
					if(status == PCAN_ERROR_OK){
						print_message_time(&mes, &met);
						canp = save_candata(&mes);
						break;
					}
					if(status == PCAN_ERROR_QRCVEMPTY)
					{
						read_limit += 1;
					}
				}while((status == PCAN_ERROR_QRCVEMPTY)&&(read_limit<20));//if queue is empty, try CAN_Read again until get data

	if(read_limit<20)
	{
		goto loop_part;
	}
	else
	{
		return 0;
	}
}

//clear the can data queue
int clear_can_queue(TPCANHandle h)
{
	TPCANMsg mes;//defined in PCANBasic.h
	TPCANTimestamp met;//defined in PCANBasic.h 
    __u32 status;//__u32 defined in pcan.h DWORD __u32
	int read_limit = 0;

	printf("Clear can data queue!---------------------------------------- \n");

	loop_part:do{
					//get CAN data and store in mes, return TPCANStatus code.
					status = testCAN_Read(h, &mes, &met);
					usleep(50000);
					//if status == 0 means no error and get the CAN data normal
					if(status == PCAN_ERROR_OK){
						print_message_time(&mes, &met);
						//canp = save_candata(&mes);
						break;
					}
					if(status == PCAN_ERROR_QRCVEMPTY)
					{
						read_limit += 1;
					}
			}while((status == PCAN_ERROR_QRCVEMPTY)&&(read_limit<20));//if queue is empty, try CAN_Read again until get data

	if(read_limit<20)
	{
		goto loop_part;
	}
	else
	{
		printf("The can data queue is cleared!---------------------------------------- \n");
		return 0;
	}
}

//read_online_message
int read_online_message(TPCANHandle h)
{
	TPCANMsg mes;//defined in PCANBasic.h
	TPCANTimestamp met;//defined in PCANBasic.h 
    __u32 status;//__u32 defined in pcan.h DWORD __u32
	int read_limit = 0;

	printf("Read online message!---------------------------------------- \n");

	loop_part:do{
					//get CAN data and store in mes, return TPCANStatus code.
					status = testCAN_Read(h, &mes, &met);
					usleep(50000);
					//if status == 0 means no error and get the CAN data normal
					if(status == PCAN_ERROR_OK){
						print_online_message(&mes, &met);
						//print_message_time(&mes, &met);
						//canp = save_candata(&mes);
						break;
					}
					if(status == PCAN_ERROR_QRCVEMPTY)
					{
						read_limit += 1;
					}
			}while((status == PCAN_ERROR_QRCVEMPTY)&&(read_limit<20));//if queue is empty, try CAN_Read again until get data

	if(read_limit<20)
	{
		goto loop_part;
	}
	else
	{
		return 0;
	}
}

//print online message
int print_online_message(TPCANMsg *mr, TPCANTimestamp* mt)
{
	if((0x700+(mr->ID)&0x01) == mr->ID)
	{
		printf("Motor %u is online! \n", (mr->ID)&0x01);
		printf("Receive online ID: 0x%08x ", mr->ID);
		for(int i=0;i<mr->LEN;i++)
		{
            printf("%02x ", mr->DATA[i]);
		}
		printf("\n");
	}
	else
	{
		printf("Get other message when read online message! \n");
		print_message_time(mr, mt);
	}
	return 0;
}

//read car information,current,velocity,position!
//receive data function,4 driver as default,0group,1,2,3,4
int read_car_data(TPCANHandle h)
{
    TPCANMsg rx_message;//defined in PCANBasic.h
	TPCANTimestamp met;//defined in PCANBasic.h 
    __u32 status;//__u32 defined in pcan.h DWORD __u32
	int read_limit = 0;

	loop_part:do{
					//get CAN data and store in mes, return TPCANStatus code.
					status = testCAN_Read(h, &rx_message, &met);
					usleep(100000);
					//if status == 0 means no error and get the CAN data normal
					if(status == PCAN_ERROR_OK){
						print_message_time(&rx_message, &met);
						if((rx_message.MSGTYPE == 0x00U)&&(rx_message.LEN == 8))
						{
							if(rx_message.ID == 0x1B)
							{
								Real_Current_Value[0] = (rx_message.DATA[0]<<8)|(rx_message.DATA[1]);
								Real_Velocity_Value[0] = (rx_message.DATA[2]<<8)|(rx_message.DATA[3]);
								Real_Position_Value[0] = ((rx_message.DATA[4]<<24)|(rx_message.DATA[5]<<16)|(rx_message.DATA[6]<<8)|(rx_message.DATA[7]));
							}
							else if(rx_message.ID == 0x2B)
							{
								Real_Current_Value[1] = (rx_message.DATA[0]<<8)|(rx_message.DATA[1]);
								Real_Velocity_Value[1] = (rx_message.DATA[2]<<8)|(rx_message.DATA[3]);
								Real_Position_Value[1] = ((rx_message.DATA[4]<<24)|(rx_message.DATA[5]<<16)|(rx_message.DATA[6]<<8)|(rx_message.DATA[7]));
							}
							else if(rx_message.ID == 0x3B)
							{
								Real_Current_Value[2] = (rx_message.DATA[0]<<8)|(rx_message.DATA[1]);
								Real_Velocity_Value[2] = (rx_message.DATA[2]<<8)|(rx_message.DATA[3]);
								Real_Position_Value[2] = ((rx_message.DATA[4]<<24)|(rx_message.DATA[5]<<16)|(rx_message.DATA[6]<<8)|(rx_message.DATA[7]));
							}
							else if(rx_message.ID == 0x4B)
							{
								Real_Current_Value[3] = (rx_message.DATA[0]<<8)|(rx_message.DATA[1]);
								Real_Velocity_Value[3] = (rx_message.DATA[2]<<8)|(rx_message.DATA[3]);
								Real_Position_Value[3] = ((rx_message.DATA[4]<<24)|(rx_message.DATA[5]<<16)|(rx_message.DATA[6]<<8)|(rx_message.DATA[7]));
							}
							else if(rx_message.ID == 0x1F)
							{
								Real_Online[0] = 1;
							}
							else if(rx_message.ID == 0x2F)
							{
								Real_Online[1] = 1;
							}
							else if(rx_message.ID == 0x3F)
							{
								Real_Online[2] = 1;
							}
							else if(rx_message.ID == 0x4F)
							{
								Real_Online[3] = 1;
							}
							else if(rx_message.ID == 0x1C)
							{
								Real_Ctl1_Value[0] = rx_message.DATA[0];
								Real_Ctl2_Value[0] = rx_message.DATA[1];
							}
							else if(rx_message.ID == 0x2C)
							{
								Real_Ctl1_Value[1] = rx_message.DATA[0];
								Real_Ctl2_Value[1] = rx_message.DATA[1];
							}
							else if(rx_message.ID == 0x3C)
							{
								Real_Ctl1_Value[2] = rx_message.DATA[0];
								Real_Ctl2_Value[2] = rx_message.DATA[1];
							}
							else if(rx_message.ID == 0x4C)
							{
								Real_Ctl1_Value[3] = rx_message.DATA[0];
								Real_Ctl2_Value[3] = rx_message.DATA[1];
							}

						}     
						break;
					}
					if(status == PCAN_ERROR_QRCVEMPTY)
					{
						read_limit += 1;
					}
				}while((status == PCAN_ERROR_QRCVEMPTY)&&(read_limit<5));//if queue is empty, try CAN_Read again until get data

	if(read_limit<5)
	{
		goto loop_part;
	}
	else
	{
		return 0;
	}
}