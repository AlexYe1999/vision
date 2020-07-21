#include<mutex>
#include<opencv2/opencv.hpp>
#include"ImageProcess.h"
#include"ArmorDector.h"
#include"Constant.h"
#include"Debug.h"
using namespace Robomaster;

//------------------------------全局变量--------------------------------------

volatile Mode tMode = Mode::Armor;
volatile ProcState procState = ProcState::ISPROC;

volatile Class::ExchangeData exchangeData;

//图片缓冲
cv::Mat MatBuffer[5];
volatile int64_t MatRear = 0;
volatile int64_t MatFront = 0;
Struct::Angle AngleBuffer;

//---------------------------------------------------------------------------------
#ifdef SHOW_IMAGE
cv::Mat Rune;
#endif
#ifdef Calibration
unsigned int CMAT_COUNT = 1;
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
    DaHengCamera Camera;
    
initCamera:
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
    int gain = 100;
    int gain_value = 100;
    cv::namedWindow("Exposure Adjust");
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
            if(!Camera.SetGain(3, gain)){
                goto initCamera;
            }
        }

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

#ifdef Calibration
    char key = cv::waitKey(1);
    if(key == ' '){
        char VideoFile[255];
        std::sprintf(VideoFile, "../image/%d.jpg",CMAT_COUNT);
        cv::imwrite(VideoFile,MatBuffer[(MatRear-1)%5]);
        CMAT_COUNT++;
    }
#endif

        cv::imshow("Exposure Adjust",MatBuffer[(MatRear-1)%5]);
        cv::waitKey(1);
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
    cv::Point2f aimPos;
    cv::Mat srcFrame;

#ifdef TIME_COST 
    int64_t tick;
#endif

    while(true){

#ifdef TIME_COST
        tick = cv::getTickCount();
#endif


        while(MatRear <= MatFront+1);
        MatFront = MatRear;

        srcFrame = MatBuffer[(MatFront-1)%5].clone();

#ifdef SHOW_IMAGE
        Rune = srcFrame.clone();
#endif

        armorDector.SetMode(tMode);
        armorDector.SetAngle(AngleBuffer);

        procState = ProcState::ISPROC;


        //if(!armorDector.StartProc(srcFrame, aimPos)) continue;
        armorDector.StartProc(srcFrame, aimPos);
        //填数

        procState = ProcState::FINISHED;
        //发数


#ifdef TIME_COST
        std::cout<<"process time:"<<static_cast<double>((cv::getTickCount() - tick)*1000/cv::getTickFrequency())<<" ms\n";
#endif
    }

}