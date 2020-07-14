#ifndef ARMORDECTOR_H
#define ARMORDECTOR_H
#include<opencv2/opencv.hpp>
#include<eigen3/Eigen/Eigen>
#include<Eigen/Core>
#include<Eigen/Dense>
#include"ImageProcess.h"
#include"KalmanFilter.h"
#include"Constant.h"
using namespace Robomaster;

//装甲信息
/**
 * @brief 装甲信息
 */
class ArmorData{
public:    

    Enum::ArmorCatglory armorCatglory; //装甲板种类
    cv::Point2f leftLed[2];     //左灯条
    cv::Point2f rightLed[2];        //右灯条

    float tx;       //x 轴偏移量
    float ty;       //y 轴偏移量
    float tz;       //z 轴偏移量


    //float pitch = 0;        //pitch 轴偏差角度
    //float yaw = 0;      //yaw 轴偏差角度
    //float distance = 0; //实际距离
    ArmorData():armorCatglory(Enum::ArmorCatglory::SMALL),tx(0),ty(0),tz(0){};
};

//保存上一目标
class Target{
public:  
    float x;
    float y;
    float z;
    Struct::Angle lastAngle;
    Enum::ArmorCatglory armorCat;
    Target():x(0),y(0),z(0),lastAngle(){}
};

class RuneData{

};

//LDE类
class LedData{
public:
    double angle; //LED灯斜率
    double length; //led长度
    cv::Point2f center; //LED中心
    cv::Point2f upPoint; //上边中点
    cv::Point2f downPoint; //下边中点
    cv::Point2f point[4]; //LED灯四个点

    void LedSort(){
        std::sort(point, point + 4, LedCmpY);
        std::sort(point, point+2, LedCmpX);
        std::sort(point+2, point+4, LedCmpX);
    }
    static bool LedCmpY(const cv::Point2f a, const cv::Point2f b){
        return a.y > b.y;
    }
    static bool LedCmpX(const cv::Point2f a, const cv::Point2f b){
        return a.x < b.x;
    }
};



/*
*@brief:装甲板处理类
*/
class ArmorDector{

public:
    ArmorDector();
    ~ArmorDector() = default;

public:
    bool StartProc(const cv::Mat& frame, cv::Point2f& aimPos); //开始
    inline void SetMode(volatile Enum::Mode& tMode);
    inline Enum::Mode GetMode() const;
    inline void SetAngle(Struct::Angle& latestAngle);

private:
    void GetArmorData(const cv::Mat & frame, Eigen::Vector3f& Pos); 
    unsigned short GetRuneData(const cv::Mat & frame, ArmorData allArmor[]); 

private:
    void SeparateColor(const cv::Mat & frame, cv::Mat & binaryImage);
    void TansformImage(const cv::Mat & binaryImage, cv::Mat & altimateImage);
    unsigned short GetLedArray(const cv::Mat & altimateImage, LedData rectArray[]);
    unsigned short CombinateLED(LedData ledArray[], ArmorData armorArray[] , const unsigned short & LedArraySize); 

private:
    void GetAngleData(ArmorData allArmor[], const unsigned short & ArmorSize);
    void get2dPointData(const ArmorData & allArmor, std::vector<cv::Point2f> & point2D);      //图像坐标系
    void get3dPointData(const ArmorData & armor, std::vector<cv::Point3f> & point3D);       //世界坐标系
    void solveAngle(ArmorData & armor, const std::vector<cv::Point3f>& point3D, const std::vector<cv::Point2f>& point2D);
private:
    Enum::PredictStatus selectBestArmor(const ArmorData allArmor[], const unsigned short & ArmorSize,Eigen::Vector3f& bestPos);


private:
    inline void RotatedByYaw();
    inline void RotatedByPitch();
    
private:
    static bool ledCmp(const LedData a, const LedData b){
        return a.length > b.length; //长的优先
    }

private:
    Enum::Mode mode;
    Enum::PredictStatus predictStatus;

private:
    unsigned short lostTarget;
    bool isFindtarget;
    float startDegree;
    Struct::Angle latestAngle;
    Target lastTarget;

};


#endif