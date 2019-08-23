#include "cantest/can_car_control.h"
#include "math.h"

/****************************************************************************************
                                       Reset the driver
Group    0-7
Number   0-15    Number==0  
*****************************************************************************************/
void CAN_RoboModule_DRV_Reset(unsigned char Group,unsigned char Number)
{
    DWORD can_id = 0x000;
    TPCANMsg tx_message;
    
    tx_message.MSGTYPE = 0x00U;//Standard frame
    tx_message.LEN = 0x08;//length of frame
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.ID = can_id;      //CAN_ID
    
    tx_message.DATA[0] = 0x55;
    tx_message.DATA[1] = 0x55;
    tx_message.DATA[2] = 0x55;
    tx_message.DATA[3] = 0x55;
    tx_message.DATA[4] = 0x55;
    tx_message.DATA[5] = 0x55;
    tx_message.DATA[6] = 0x55;
    tx_message.DATA[7] = 0x55;

    write_CAN_data(can_id,tx_message.DATA,0x08);
    usleep(1000);
}

/****************************************************************************************
                                     Mode choice 
Group    0-7
Number   0-15 Number==0 publish

Mode    range

OpenLoop_Mode                       0x01
Current_Mode                        0x02
Velocity_Mode                       0x03
Position_Mode                       0x04
Velocity_Position_Mode              0x05
Current_Velocity_Mode               0x06
Current_Position_Mode               0x07
Current_Velocity_Position_Mode      0x08
*****************************************************************************************/
void CAN_RoboModule_DRV_Mode_Choice(unsigned char Group,unsigned char Number,unsigned char Mode)
{
    DWORD can_id = 0x001;
    TPCANMsg tx_message;
    
    tx_message.MSGTYPE = 0x00U;//Standard frame
    tx_message.LEN = 0x08;//length of frame
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.ID = can_id;//CAN_ID
    
    tx_message.DATA[0] = Mode;
    tx_message.DATA[1] = 0x55;
    tx_message.DATA[2] = 0x55;
    tx_message.DATA[3] = 0x55;
    tx_message.DATA[4] = 0x55;
    tx_message.DATA[5] = 0x55;
    tx_message.DATA[6] = 0x55;
    tx_message.DATA[7] = 0x55;

    write_CAN_data(can_id,tx_message.DATA,0x08);
    usleep(1000);
}

/****************************************************************************************
                                   OpenLoop_Mode
Group   0-7

Number  0-15 Number==0 publish

temp_pwmµ Range is:
-5000 ~ +5000 max is 5000 temp_pwm = +-5000 max is voltage supply

*****************************************************************************************/
void CAN_RoboModule_DRV_OpenLoop_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
    DWORD can_id = 0x002;
    TPCANMsg tx_message;
    
    tx_message.MSGTYPE = 0x00U;//Standard frame
    tx_message.LEN = 0x08;//length of frame
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.ID = can_id;      //CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    tx_message.DATA[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.DATA[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.DATA[2] = 0x55;
    tx_message.DATA[3] = 0x55;
    tx_message.DATA[4] = 0x55;
    tx_message.DATA[5] = 0x55;
    tx_message.DATA[6] = 0x55;
    tx_message.DATA[7] = 0x55;
    
    write_CAN_data(can_id,tx_message.DATA,0x08);

    usleep(1000);
}

/****************************************************************************************
                                   Velocity_Mode
Group   0-7

Number  0-15 Number==0 publish

temp_pwm Range is
0 ~ +5000 5000is max,temp_pwm = 5000,max is voltage supply

temp_velocity Range is
-32768 ~ +32767
*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    DWORD can_id = 0x004;
    TPCANMsg tx_message;
    
    tx_message.MSGTYPE = 0x00U;//Standard frame
    tx_message.LEN = 0x08;//length of frame
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.ID = can_id;//CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.DATA[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.DATA[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.DATA[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.DATA[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.DATA[4] = 0x55;
    tx_message.DATA[5] = 0x55;
    tx_message.DATA[6] = 0x55;
    tx_message.DATA[7] = 0x55;
    
    write_CAN_data(can_id,tx_message.DATA,0x08);
    
    usleep(1000);
}

/****************************************************************************************
                                  Velocity_Position_Mode
Group   0-7

Number  0-15 Number==0 publish

temp_pwm Range is
0 ~ +5000 5000is max,temp_pwm = 5000,max is voltage supply

temp_velocity Range is
0 ~ +32767 RPM

temp_position Range is
-2147483648~+2147483647 qc 
*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    DWORD can_id = 0x006;
    TPCANMsg tx_message;
    
    tx_message.MSGTYPE = 0x00U;//Standard frame
    tx_message.LEN = 0x08;//length of frame
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.ID = can_id;//CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    tx_message.DATA[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.DATA[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.DATA[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.DATA[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.DATA[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.DATA[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.DATA[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.DATA[7] = (unsigned char)(Temp_Position&0xff);
    
    write_CAN_data(can_id,tx_message.DATA,0x08);

    usleep(1000);
}

/****************************************************************************************
                                  Current_Velocity_Position_Mode
Group   0-7

Number  0-15 Number==0 publish
temp_current Range is
0 ~ +32767 mA

temp_velocity Range is
0 ~ +32767 RPM

temp_position Range is
-2147483648~+2147483647 qc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    DWORD can_id = 0x009;
    TPCANMsg tx_message;
    
    tx_message.MSGTYPE = 0x00U;//Standard frame
    tx_message.LEN = 0x08;//length of frame
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.ID = can_id;      //Ö¡IDÎªŽ«Èë²ÎÊýµÄCAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    tx_message.DATA[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.DATA[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.DATA[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.DATA[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.DATA[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.DATA[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.DATA[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.DATA[7] = (unsigned char)(Temp_Position&0xff);


    write_CAN_data(can_id,tx_message.DATA,0x08);
    usleep(1000);
}

/****************************************************************************************
                                      DRV_Config
Temp_Time1 Range is: 0 ~ 255 set 0 is shut down Current_Velocity_Position_Mode feedback
Temp_Time2 Range is: 0 ~ 255 set 0 is shut down Limit Signal feedback
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time1,unsigned char Temp_Time2)
{
    DWORD can_id = 0x00A;
    TPCANMsg tx_message;
    
    tx_message.MSGTYPE = 0x00U;//Standard frame
    tx_message.LEN = 0x08;//length of frame
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.ID = can_id;
    
    tx_message.DATA[0] = Temp_Time1;
    tx_message.DATA[1] = Temp_Time2;
    tx_message.DATA[2] = 0x55;
    tx_message.DATA[3] = 0x55;
    tx_message.DATA[4] = 0x55;
    tx_message.DATA[5] = 0x55;
    tx_message.DATA[6] = 0x55;
    tx_message.DATA[7] = 0x55;
    
    write_CAN_data(can_id,tx_message.DATA,0x08);
    
    usleep(1000);
}

/****************************************************************************************
                                      Online_Check
*****************************************************************************************/
void CAN_RoboModule_DRV_Online_Check(unsigned char Group,unsigned char Number)
{
    DWORD can_id = 0x00F;
    TPCANMsg tx_message;
    
    tx_message.MSGTYPE = 0x00U;//Standard frame
    tx_message.LEN = 0x08;//length of frame
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.ID = can_id;      //CAN_ID
    
    tx_message.DATA[0] = 0x55;
    tx_message.DATA[1] = 0x55;
    tx_message.DATA[2] = 0x55;
    tx_message.DATA[3] = 0x55;
    tx_message.DATA[4] = 0x55;
    tx_message.DATA[5] = 0x55;
    tx_message.DATA[6] = 0x55;
    tx_message.DATA[7] = 0x55;

    write_CAN_data(can_id,tx_message.DATA,0x08);
    
    usleep(1000);
}
