#include<mutex>
#include"ImageProcess.h"
#include"ArmorDector.h"
#include"Constant.h"
using namespace Robomaster;

#define COMPETITION //比赛
#define CAMERA //相机调试
#define RECORD //录像

//------------------------------全局变量--------------------------------------
std::mutex AnglelLock;
std::mutex CopyLock;

volatile Enum::Mode tMode = Enum::Mode::Armor;

volatile Enum::ProcState procState = Enum::ProcState::ISPROC;

volatile Class::ExchangeData exchangeData;

//图片缓冲
cv::Mat MatBuffer;
Struct::Angle AngleBuffer;

//------------------------------全局变量---------------------------------------

/**
 * @brief 生产者
 * 
 */
void ImageProcess::ImageProducer(){
    CopyLock.unlock();
    DaHengCamera Camera;

    while(!Camera.StartDevice());
    Camera.SetResolution();
    while(!Camera.StreamOn());
    Camera.SetExposureTime();
    //设置曝光增益000
    Camera.SetGain();
    //设置是否自动白平衡
    Camera.Set_BALANCE_AUTO(0);
    //手动设置白平衡通道及系数，此之前需关闭自动白平衡
    // camera.Set_BALANCE(0,40);


    //控制曝光值动态变化
    int exp_time = 3000;
    int exp_time_value = 3000;
    //控制曝光增益动态变化
    int gain = 10;
    int gain_value = 10;
    cv::namedWindow("Exposure Adjust");
    cv::createTrackbar("曝光","Exposure Adjust",&exp_time_value, 3000);
    cv::createTrackbar("曝光增益","Exposure Adjust",&gain_value, 1000);


    while(true){
        
        if(exp_time != exp_time_value){
            exp_time = exp_time_value;
            Camera.SetExposureTime(exp_time);
        }
        if(gain != gain_value){
            gain = gain_value;
            Camera.SetGain(3, gain);
        }

        CopyLock.lock();
        switch(tMode){  //射击模式
            case Enum::Mode::Armor:{
                Camera.GetMat(MatBuffer);
                break;
            }
            case Enum::Mode::Rune:{

                break;
            }
            default:{
                break;
            } 
        }
        CopyLock.unlock();


    }

}


/**
 * @brief 消费者
 * 
 */
void ImageProcess::ImageConsumer(){

    ArmorDector armorDector;
    cv::Point2f aimPos;
    cv::Mat srcFrame;

    while(true){


        CopyLock.lock();
        srcFrame = MatBuffer.clone();
        CopyLock.unlock();
        
        armorDector.SetMode(tMode);
        armorDector.SetAngle(AngleBuffer);

        procState = Enum::ProcState::ISPROC;

        if(!armorDector.StartProc(srcFrame, aimPos)) continue;
         
        //填数

        procState = Enum::ProcState::FINISHED;
        //发数

    }

    cv::namedWindow("Src");
    cv::imshow("Src",srcFrame);
}