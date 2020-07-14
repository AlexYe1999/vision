#include"Serial.h"
#include"CRC_Check.h"
#include"ImageProcess.h"

extern ProcState procState;
extern Mode tMode;

int Serial::paraReceiver(){
    const int fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd == -1){
        exit(0);
    }

    ConfigurePort();

    while(1){       

        static ReceivedData flag;
        ReciveData(flag);

        if(procState == ProcState::FINISHED){
            procState = ProcState::ISPROC;
            SendData(target);             //发送数据
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
*          send_bytes[0]
*          send_bytes[1]--send_bytes[4]为pitch数据
*          send_bytes[5] pitch标志位
*          send_bytes[6]--send_bytes[9]为yaw数据
*          send_bytes[10]  yaw标志位
*          send_bytes[11]--send_bytes[12]为CRC校验码
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
    send_bytes[0] = 0xFF;

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

    //send_bytes[12] = data.distance;
    send_bytes[12] = data.shoot;
    send_bytes[13] = data.IsHaveArmor;

    Append_CRC16_Check_Sum(send_bytes, 16);

    write(portNum, send_bytes, 16);
    for(int i=0;i<16;i++){
        printf("%X ",send_bytes[i]);
    }
    std::cout<<"\n";
}


/**
*   @brief:串口PC端接收
*/
void Serial::ReciveData(ReceivedData & data){

    int bytes;
    ioctl(portNum, FIONREAD, &bytes);
    if(bytes == 0) return;
    bytes = read(portNum,rec_bytes,6);
    if(rec_bytes[0] = 0xaa){
        data.mode  = (int)rec_bytes[1];
        data.status = (int)rec_bytes[3];
    }

    ioctl(portNum, FIONREAD, &bytes);

    if(bytes>0){
        read(portNum,rec_bytes,bytes);
    }
}

