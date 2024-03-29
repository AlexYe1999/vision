#ifndef ARMORDECTOR_H
#define ARMORDECTOR_H
#include<opencv2/opencv.hpp>
#include<eigen3/Eigen/Eigen>
#include<Eigen/Core>
#include<Eigen/Dense>
#include"ImageProcess.h"
#include"Prediction.h"
#include"Constant.h"
#include"Serial.h"
using namespace Robomaster;
using namespace Robomaster;
class Prediction;

//装甲信息
/**
 * @brief 装甲信息
 */
class ArmorData{
public:    

    ArmorCatglory armorCatglory; //装甲板种类
    cv::Point2f leftLed[2];     //左灯条
    cv::Point2f rightLed[2];        //右灯条

    float tx;       //x 轴偏移量
    float ty;       //y 轴偏移量
    float tz;       //z 轴偏移量


    //float pitch = 0;        //pitch 轴偏差角度
    //float yaw = 0;      //yaw 轴偏差角度
    //float distance = 0; //实际距离
    ArmorData():armorCatglory(ArmorCatglory::SMALL),tx(0),ty(0),tz(0){};
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
    void ConfigureParam(ReceivedData & data);
    bool StartProc(cv::Mat& frame, Eigen::Vector3f& angle); //开始
    void ConfigureData(VisionData &data,const Eigen::Vector3f &vec);

    void SetMode(volatile Mode& tMode);
    Mode GetMode() const;
    void SetAngle(Struct::Angle& latestAngle);

private:
    void GetArmorData(const cv::Mat & frame); 
    unsigned short GetRuneData(const cv::Mat & frame, ArmorData allArmor[]); 

private:
    void SeparateColor(cv::Mat frame, cv::Mat & binaryImage);
    void TansformImage(const cv::Mat & binaryImage, cv::Mat & altimateImage);
    unsigned short GetLedArray(const cv::Mat & altimateImage, LedData rectArray[]);
    unsigned short CombinateLED(LedData ledArray[], ArmorData armorArray[] , const unsigned short & LedArraySize); 

private:
    void GetAngleData(ArmorData allArmor[], const unsigned short & ArmorSize);
    void get2dPointData(const ArmorData & allArmor, std::vector<cv::Point2f> & point2D);      //图像坐标系
    void get3dPointData(const ArmorData & armor, std::vector<cv::Point3f> & point3D);       //世界坐标系
    void solveAngle(ArmorData & armor, const std::vector<cv::Point3f>& point3D, const std::vector<cv::Point2f>& point2D);
private:
    PredictStatus selectBestArmor(const ArmorData allArmor[], const unsigned short & ArmorSize);

private:
    Prediction predict;
private:
    inline void SetM(float &yaw, float &pitch);
    inline void ReverseRotate(Eigen::Vector3f& vec);
    inline void Rotate(Eigen::Vector3f& vec);


private:
    static bool ledCmp(const LedData a, const LedData b){
        return a.length > b.length; //长的优先
    }

private:
    void checkImage(cv::Mat mat);
    void GammaTransf(cv::Mat & img, const unsigned int & gamma);

private:
    unsigned short lostCount;
    unsigned int AverGray;
    bool OpenGamma;

private:
    Mode mode;
    unsigned short level;
    float bulletVelocity;
    
private:
    double f(double x);
    double f_(double x);
    void set(double y_, double t, double V_, double Z_);
    double y;                           
    double V;
    double Z;
    double T;
private:
    PredictStatus predictStatus;
    unsigned short lostTarget;
    bool isFindtarget;
    Struct::Angle latestAngle;
    Target lastTarget;
    Target target;
    Eigen::Matrix3f Myaw;
    Eigen::Matrix3f Mpitch;

};


#endif