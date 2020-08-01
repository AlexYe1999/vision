#include<mutex>
#include<opencv2/opencv.hpp>
#include"ImageProcess.h"
#include"ArmorDector.h"
#include"Constant.h"
#include"Debug.h"
using namespace Robomaster;

//------------------------------全局变量--------------------------------------

std::mutex exchangeMutex; //数据交换锁
volatile ProcState procState = ProcState::ISPROC;

//图片缓冲
cv::Mat MatBuffer[5];
volatile int64_t MatRear = 0;
volatile int64_t MatFront = 0;

//交换数据
VisionData visionData;
ReceivedData recivedData;

//---------------------------------------------------------------------------------
#ifdef SHOW_IMAGE
int gamma_g = 450;
cv::Mat Rune;
#endif


/**
 * @brief 生产者
 * 
 */
void ImageProcess::ImageProducer(){

#ifdef COMPETITION_ON
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
    Camera.SetExposureTime(Constants::ExposureTime);
    Camera.SetGain(3, Constants::ExposureGain);


    while(true){
        switch(tMode){  //射击模式
            case Mode::Armor:{
                Camera.GetMat(MatBuffer[MatRear%5]);
                MatRear++;
                break;
            }
            case Mode::Rune:{

                break;
            }
            default:{
                break;
            } 
        }
    }

#else

#ifdef CAMERA_ON
initCamera:
    DaHengCamera Camera;
    while(!Camera.StartDevice());
    Camera.SetResolution();
    while(!Camera.StreamOn());
    Camera.SetExposureTime();
    //设置曝光增益
    Camera.SetGain();
    //设置是否自动白平衡
    Camera.Set_BALANCE_AUTO(0);
    //手动设置白平衡通道及系数，此之前需关闭自动白平衡
    // camera.Set_BALANCE(0,40);

    //控制曝光值动态变化
    int exp_time = 3000;
    int exp_time_value = 3000;
    //控制曝光增益动态变化
    int gain = 100;
    int gain_value = 100;
    cv::namedWindow("Exposure Adjust");
#ifdef SHOW_IMAGE
    cv::createTrackbar("set gamma", "Exposure Adjust", &gamma_g, 500);
#endif    
    cv::createTrackbar("曝光","Exposure Adjust",&exp_time_value, 10000);
    cv::createTrackbar("曝光增益","Exposure Adjust",&gain_value, 1000);    

#ifdef TIME_COST    
        int64_t tick = cv::getTickCount();
#endif

    while(true){
        
#ifdef TIME_COST    
        tick = cv::getTickCount();
#endif

        if(exp_time != exp_time_value){
            exp_time = exp_time_value;
            Camera.SetExposureTime(exp_time);
        }
        if(gain != gain_value){
            gain = gain_value;
            Camera.SetGain(3, gain);
        }

        switch(Mode::Armor){  //射击模式
            case Mode::Armor:{
                if(!Camera.GetMat(MatBuffer[MatRear%5])){
                    Camera.~DaHengCamera();
                    goto initCamera;                  
                }
                MatRear++;
                break;
            }
            case Mode::Rune:{

                break;
            }
            default:{
                break;
            } 
        }
#ifndef COMPETITION_ON
        cv::imshow("Exposure Adjust",MatBuffer[(MatRear-1)%5]);
#endif
#ifdef TIME_COST
        std::cout<<"product time:"<<static_cast<double>((cv::getTickCount() - tick)*1000/cv::getTickFrequency())<<" ms\n";
#endif    
    }
#endif

#ifdef RECORD
    cv::Size videoSize(1280, 720);
    char VideoFile[255];
    time_t timep;
    struct tm * p;
    time(&timep);
    p = gmtime(&timep);
    std::sprintf(VideoFile, "video/%d月%d日%d时%d分%d秒.avi", 1 + p->tm_mon, p->tm_mday, 8 + p->tm_hour, p->tm_min, p->tm_sec);
    cv::VideoWriter writer(VideoFile, CV_FOURCC('M', 'J', 'P', 'G'), recordRate, videoSize);
#endif

#ifndef CAMERA_ON
    cv::VideoCapture cap("/home/yeahoo/src.mp4");

    while(true){
    }
#endif
#endif
}


/**
 * @brief 消费者
 * 
 */
void ImageProcess::ImageConsumer(){

    ArmorDector armorDector;
    Eigen::Vector3f angle;
    cv::Mat srcFrame;
#ifdef TIME_COST 
    int64_t tick;
#endif

    while(true){

#ifdef TIME_COST
        tick = cv::getTickCount();
#endif


        while(MatRear <= MatFront+1 && MatRear < 1);
        MatFront = MatRear;

        srcFrame = MatBuffer[(MatFront-1)%5].clone();

#ifdef SHOW_IMAGE
        Rune = srcFrame.clone();
        std::cout<<"\n\n-------------------------------------------------------------------------\n"
                          <<"-------------------------------------------------------------------------\n\n\n";
#endif
        exchangeMutex.lock();
        armorDector.ConfigureParam(recivedData);
        exchangeMutex.unlock();

        armorDector.StartProc(srcFrame, angle);

        //填数
        armorDector.ConfigureData(visionData, angle);
        //发数
        procState = ProcState::FINISHED;

#ifdef SHOW_IMAGE
    cv::imshow("All Armor",Rune);
    cv::waitKey(1);
#endif

#ifdef TIME_COST
        std::cout<<"process time:"<<static_cast<double>((cv::getTickCount() - tick)*1000/cv::getTickFrequency())<<" ms\n";
#endif

    }

}