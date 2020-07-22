
#include<mutex>
#include"Serial.h"
#include"CRC_Check.h"
#include"ImageProcess.h"
#include "Constant.h"
using namespace Robomaster;

extern ProcState procState;
extern std::mutex exchangeMutex; //数据交换锁
//extern Mode tMode;
//交换数据
extern VisionData visionData;
extern ReceivedData recivedData;

Serial::Serial():
    portNum(0),
    fd(open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NONBLOCK))
{}


int Serial::paraReceiver(){
    if(fd == -1) exit(0);
    ConfigurePort();
    while(1){       

        exchangeMutex.lock();
        ReciveData(recivedData);
        exchangeMutex.unlock();

        if(procState == ProcState::FINISHED){
            procState = ProcState::ISPROC;
            SendData(visionData);             //发送数据
        }

    }
    
    close(fd);

}


/**
*   @brief:打开串口
*   @param  Portname    类型 const char     串口的位置
*/
int Serial::OpenPort(){
    
    portNum = open(PortName, O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    if(portNum == -1){
        printf("The port open error!\n");
    }
    else{
        fcntl(portNum,F_SETFL,0);   //读取串口的信息
    }

    return portNum;
}


/**
*   @brief:配置串口,无奇偶校验
*/
int Serial::ConfigurePort(){
    struct termios PortSetting;

    //波特率设置
    cfsetispeed(&PortSetting,B115200);
    cfsetospeed(&PortSetting,B115200);
    //No parity
    PortSetting.c_cflag &= ~PARENB;         //无奇偶校验
    PortSetting.c_cflag &= ~CSTOPB;         //停止位:1bit
    PortSetting.c_cflag &= ~CSIZE;          //清除数据位掩码
    PortSetting.c_cflag |=  CS8;                      //数据位

    tcsetattr(portNum,TCSANOW,&PortSetting);
    return (portNum);
}




/**
*   @brief:发送数据
*   @param:
*          send_bytes[0] 0xff
*          send_bytes[1]--send_bytes[4]为pitch数据
*          send_bytes[5] pitch符号位
*          send_bytes[6]--send_bytes[9]为yaw数据
*          send_bytes[10]  yaw符号位
*          send_bytes[11]是否监测到
*          send_bytes[12]距离
*          send_bytes[13]打弹
*          send_bytes[14]-[15]校验
>>>>>>> 47dad157eee92bce1fca6030481f3b2978a34bec
*/
void Serial::SendData(VisionData & data)
{
    if(data.pitchData.f<0){
        pitch_bit_ = 0x00;
        data.pitchData.f=-data.pitchData.f;
    }
    else{
        pitch_bit_ = 0x01;
    }

    if(data.yawData.f<0){
        yaw_bit_ = 0x00;
        data.yawData.f=-data.yawData.f;
    }
    else{
            yaw_bit_ = 0x01;
        }
    send_bytes[0] = 0xff;

    send_bytes[1] = data.pitchData.uc[0];
    send_bytes[2] = data.pitchData.uc[1];
    send_bytes[3] = data.pitchData.uc[2];
    send_bytes[4] = data.pitchData.uc[3];
    send_bytes[5] = pitch_bit_;

    send_bytes[6] = data.yawData.uc[0];
    send_bytes[7] = data.yawData.uc[1];
    send_bytes[8] = data.yawData.uc[2];
    send_bytes[9] = data.yawData.uc[3];
    send_bytes[10] = yaw_bit_;

    send_bytes[11] = data.IsHaveArmor;    
    send_bytes[12] = data.distance;
    send_bytes[13] = data.shoot;


    Append_CRC16_Check_Sum(send_bytes, 16);

    write(portNum, send_bytes, 16);
    for(int i=0;i<16;i++){
        printf("%X ",send_bytes[i]);
    }
    std::cout<<"\n";
}


/**
*   @brief:串口PC端接收
*   @param:
*          send_bytes[0] 0xff 
*          send_bytes[1]--send_bytes[4]为pitch数据
*          send_bytes[5] pitch符号位
*          send_bytes[6]--send_bytes[9]为yaw数据
*          send_bytes[10]  yaw符号位
*          send_bytes[11] 等级
*          send_bytes[12] 0xff
*/
void Serial::ReciveData(ReceivedData & data){

    int bytes;
    ioctl(portNum, FIONREAD, &bytes);
    if(bytes == 0) return;

    bytes = read(portNum,rec_bytes,13);

    if(rec_bytes[0] == 0xff && rec_bytes[12] == 0xff){
        data.pitch.uc[0] = rec_bytes[0];
        data.pitch.uc[1] = rec_bytes[1];
        data.pitch.uc[2] = rec_bytes[2];
        data.pitch.uc[3] = rec_bytes[3];
        if(rec_bytes[5] == 0x00){
            data.pitch.f *= -1.0;
        }
        data.yaw.uc[0] = rec_bytes[6];
        data.yaw.uc[1] = rec_bytes[7];
        data.yaw.uc[2] = rec_bytes[8];      
        data.yaw.uc[3] = rec_bytes[9];
        if(rec_bytes[10] == 0x00){
            data.yaw.f *= -1.0;
        }

        data.level = rec_bytes[11];
    }

    ioctl(portNum, FIONREAD, &bytes);

    if(bytes>0){
        read(portNum,rec_bytes,bytes);
    }

}

