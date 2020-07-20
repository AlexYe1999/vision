#include<algorithm>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<eigen3/Eigen/Eigen>
#include"ArmorDector.h"
#include"Constant.h"
using namespace Robomaster;

ArmorDector::ArmorDector():
    mode(Enum::Mode::Armor),
    predictStatus (Enum::PredictStatus::NONE),
    lostTarget(0),
    isFindtarget(0),
    startDegree(0.0),
    lastTarget(),
    latestAngle(),
    Myaw(),
    Mpitch(),
    frameCount(0)
{}


/**
 *@brief 设置射击模式 
 */
void ArmorDector::SetMode(volatile Enum::Mode& tMode){
    if(tMode != mode){
        predictStatus = Enum::PredictStatus::NONE;
    }

    mode = tMode;
}

/**
 *@brief 返回处于的模式
 */
Enum::Mode ArmorDector::GetMode() const {
    return mode;
}

/**
* @brief:设置获得照片时的角度
*/
void ArmorDector::SetAngle(Struct::Angle& latestAngle){
        this->latestAngle = latestAngle;
}

/**
 * @brief 分部函数
 * @param frame 当前图像的副本
 * @param mode 射击模式
 * @param aimPos 射击点
 * @return 是否成功
 */
bool ArmorDector::StartProc(const cv::Mat & frame, cv::Point2f & aimPos){
    switch(mode){
        case Enum::Mode::Armor:{
            GetArmorData(frame);
            break;
        }
        case Enum::Mode::Rune:{


            break;
        }
        default:
            return false;
    }

    switch(predictStatus)   {
        case Enum::PredictStatus::NEW:{

        }break;
        case  Enum::PredictStatus::FIND:{

        }break;
        case Enum::PredictStatus::UNCLEAR:{

        }break;
        case Enum::PredictStatus::NONE:{

        }break;
        default:{

        }break;
    }

}

/**
 * @brief 伽马值评估
 * 
 */
void GetGamma(){




}


/**
 * @brief 伽马变换
 * 
 */
void GammaTransf(){



}

/**
 * @brief 分层总函数
 * @param frame 当前图像
 * @param aimPos 射击点
 * @return 是否找到 
 */
void ArmorDector::GetArmorData(const cv::Mat & frame){
    unsigned short armorNum;
    unsigned short ledNum;    
    ArmorData allArmor[10];
    LedData ledArray[20];    
    cv::Mat binaryImage;
    cv::Mat altimateImage;

    SeparateColor(frame, binaryImage); //找出对应颜色
    TansformImage(binaryImage, altimateImage); //进一步操作

    ledNum = GetLedArray(altimateImage, ledArray); //取出led数组

    if(ledNum < 2){
        if(isFindtarget && lostTarget < Constants::LostRange){ //丢三帧以内
            predictStatus = Enum::PredictStatus::UNCLEAR;
            lostTarget++;
        }
        else{   //超过误差
            lostTarget = 0;
            isFindtarget = false;
            predictStatus = Enum::PredictStatus::NONE;
        }
        return;
    }

    unsigned short armorNum;
    armorNum = CombinateLED(ledArray, allArmor, ledNum);

    if(armorNum < 2){
        if(isFindtarget && lostTarget < Constants::LostRange){ //丢三帧以内
            predictStatus = Enum::PredictStatus::UNCLEAR;
            lostTarget++;
        }
        else{   //超过误差
            lostTarget = 0;
            isFindtarget = false;
            predictStatus = Enum::PredictStatus::NONE;
        }
        return;
    }

    GetAngleData(allArmor, armorNum);

    predictStatus =  selectBestArmor(allArmor,  armorNum);

}

unsigned short ArmorDector::GetRuneData(const cv::Mat & frame, ArmorData allArmor[]){


}


/**
 * @brief:分离出对应颜色
 * @param frame 输入图像
 * @param binaryImage 输出目标颜色的二值化图像
 */
void ArmorDector::SeparateColor(const cv::Mat & frame, cv::Mat & binaryImage){
    cv::Mat splitIamge[3];
    split(frame, splitIamge);
    
    switch(Constants::enemyColor){
        case Enum::EnemyColor::BLUE:{
            binaryImage = splitIamge[0] - splitIamge[2];
            break;
        }
        case Enum::EnemyColor::RED:{
            binaryImage = splitIamge[2] - splitIamge[0];
            break;
        }
    }

}

/**
 * @brief: 二值化前的进一步操作
 * @param binaryImage 输入二值化的图像
 * @param altimateImage 输出最终图像
 */
void ArmorDector::TansformImage(const cv::Mat & binaryImage, cv::Mat & altimateImage){
    altimateImage = binaryImage > Constants::BinaryRange;
}



/**
 * @brief 取得图中的可能灯条
 * @param image 输入的图形
 * @param ledArray 输出的灯条数组
 */
unsigned short ArmorDector::GetLedArray(const cv::Mat & image,LedData ledArray[]){

    std::vector<std::vector<cv::Point>> LEDContours;
    std::vector<cv::Vec4i> Hierarchy;
    cv::RotatedRect ledRect;
    unsigned short TargetNum = 0;

    findContours(image, LEDContours, Hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    unsigned short contoursSize = static_cast<short>(LEDContours.size());

    for(size_t contoursNum = 0; contoursNum < contoursSize && TargetNum < 20; contoursNum++){

        double area = contourArea(LEDContours[contoursNum]);
        if(area < Constants::MinLedArea || area > Constants::MaxLedArea) continue;

        ledRect = minAreaRect(LEDContours[contoursNum]);

        LedData MayTarget;
        ledRect.points(MayTarget.point);
        MayTarget.LedSort(); 

        double whRatio; //灯条长宽比
        double width; //灯条宽度
        MayTarget.upPoint = (MayTarget.point[0]+MayTarget.point[1]) / 2; //灯条上中点
        MayTarget.downPoint = (MayTarget.point[2]+MayTarget.point[3]) / 2; //灯条下中点
        MayTarget.center = (MayTarget.upPoint+MayTarget.downPoint) / 2; //灯条中心

        cv::Point2f wMinus = MayTarget.point[0] + MayTarget.point[2] - MayTarget.point[1] - MayTarget.point[3]; //灯条横向差
        cv::Point2f hMinus = MayTarget.upPoint - MayTarget.downPoint; //灯条纵向差
        MayTarget.length = sqrt(hMinus.x*hMinus.x + hMinus.y*hMinus.y); //灯条长度
        MayTarget.angle =  hMinus.x / hMinus.y; //灯条斜率^-1

        width = sqrt(wMinus.x*wMinus.x + wMinus.y*wMinus.y);
        whRatio = MayTarget.length / width;

        if(whRatio < Constants::LedMinRatioRange || whRatio > Constants::LedMaxRatioRange || abs(MayTarget.angle) > Constants::LedAngleRange) continue;

        ledArray[TargetNum++] = MayTarget;
    }

    return TargetNum;
}

unsigned short ArmorDector::CombinateLED(LedData ledArray[], ArmorData armorArray[] , const unsigned short & LedArraySize){
    
    unsigned short armorSize = 0;
    bool SelectedTarget[LedArraySize];
    memset(SelectedTarget,false,sizeof(SelectedTarget));

    std::sort(ledArray, ledArray + LedArraySize, ledCmp);

    for(short i = 0; i < LedArraySize; i++){
        for(short j = i + 1; j < LedArraySize; j++){

            if(SelectedTarget[i] || SelectedTarget[j]) continue; //未被配对

            float lRatio = ledArray[i].length / ledArray[j].length; //长度相近
            if(lRatio > Constants::LengthRatio || lRatio < 1/Constants::LengthRatio) continue;

            float AngleMinus = ledArray[i].angle - ledArray[j].angle; //角度相近
            if(abs(AngleMinus) > Constants::AngleRatioDelta) continue;

            cv::Point2f Minus = ledArray[i].center-ledArray[j].center;

            float centerDeltaY = abs(Minus.y);
            float centerDeltaX = abs(Minus.x);
            float centerDistance = sqrt(Minus.y * Minus.y + Minus.x * Minus.x);
            float length = (ledArray[i].length+ledArray[j].length) / 2;
            float hRatio =  centerDeltaY / length;
            float wRatio =  centerDeltaX / length;
            float whRatio = centerDistance / length;

            if( hRatio > Constants::DeltaYRatio || wRatio > Constants::DeltaXRatio 
            || whRatio > Constants::ArmorMaxRatio || whRatio < Constants::ArmorMinRatio) continue;
            
            SelectedTarget[i] = 1;
            SelectedTarget[j] = 1;
            
            if(whRatio > 3){
                armorArray[armorSize].armorCatglory = Enum::ArmorCatglory::LARGE;
            }
            else{
                armorArray[armorSize].armorCatglory = Enum::ArmorCatglory::SMALL;
            }
            
            if(ledArray[i].center.x < ledArray[j].center.x){
                armorArray[armorSize].leftLed[0] = (ledArray[i].point[0]+ledArray[i].point[1])/2;
                armorArray[armorSize].leftLed[1] = (ledArray[i].point[2]+ledArray[i].point[3])/2;
                armorArray[armorSize].rightLed[0] = (ledArray[j].point[0]+ledArray[j].point[1])/2;
                armorArray[armorSize].rightLed[1] = (ledArray[j].point[2]+ledArray[j].point[3])/2;
            }
            else{
                armorArray[armorSize].leftLed[0] = (ledArray[j].point[0]+ledArray[j].point[1])/2;
                armorArray[armorSize].leftLed[1] = (ledArray[j].point[2]+ledArray[j].point[3])/2;
                armorArray[armorSize].rightLed[0] = (ledArray[i].point[0]+ledArray[i].point[1])/2;
                armorArray[armorSize].rightLed[1] = (ledArray[i].point[2]+ledArray[i].point[3])/2;
            }
            armorSize++;
            break;
        }
        if(armorSize == 10) return armorSize;
    }
    return armorSize;
}

void ArmorDector::GetAngleData(ArmorData allArmor[], const unsigned short & ArmorSize){
    std::vector<cv::Point2f>point2D;
    std::vector<cv::Point3f>point3D;

    for(int i = 0; i < ArmorSize;i++){
        get2dPointData(allArmor[i], point2D);
        get3dPointData(allArmor[i], point3D);
        solveAngle(allArmor[i], point3D, point2D);
    }
}

inline void ArmorDector::get2dPointData(const ArmorData & allArmor, std::vector<cv::Point2f> & point2D){
    point2D.push_back(allArmor.leftLed[0]);
    point2D.push_back(allArmor.rightLed[0]);
    point2D.push_back(allArmor.rightLed[1]);
    point2D.push_back(allArmor.leftLed[1]);
}

inline void ArmorDector::get3dPointData(const ArmorData & armor, std::vector<cv::Point3f> & point3D){
    float fHalfX = 0;
    float fHalfY = 0;

    switch(armor.armorCatglory){
        case Enum::ArmorCatglory::SMALL:{
            fHalfX = Constants::rSmallArmorWidth / 2;
            fHalfY = Constants::rSmallArmorHeight / 2;
            break;
        }
        case Enum::ArmorCatglory::LARGE:{
            fHalfX = Constants::rLargeArmorWidth / 2;
            fHalfY = Constants::rLargeArmorHeight / 2;
            break;
        }
        case Enum::ArmorCatglory::RUNE:{
            fHalfX = Constants::rRuneWidth / 2;
            fHalfY = Constants::rRuneHeight / 2;
            break;
        }
        default:
            break;
    }

    point3D.push_back(cv::Point3f(-fHalfX,-fHalfY, 0.0));
    point3D.push_back(cv::Point3f(fHalfX,-fHalfY, 0.0));
    point3D.push_back(cv::Point3f(fHalfX,fHalfY, 0.0));
    point3D.push_back(cv::Point3f(-fHalfX,fHalfY, 0.0));

}

void ArmorDector::solveAngle(ArmorData & armor, const std::vector<cv::Point3f>& point3D, const std::vector<cv::Point2f>& point2D){

    cv::Mat rvecs = cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat tvecs = cv::Mat::zeros(3,1,CV_64FC1);
    double tx;
    double ty;
    double tz;

    switch(mode){
        case Enum::Mode::Armor:
        {
            //!大能量机关
            cv::Mat caremaMatrix = Constants::caremaMatrix_shoot;
            cv::Mat distCoeffs = Constants::distCoeffs_shoot;
            cv::solvePnP(point3D, point2D, caremaMatrix, distCoeffs, rvecs, tvecs);  //解算x，y，z 三轴偏移量

            tx = tvecs.ptr<double>(0)[0];
            ty = tvecs.ptr<double>(0)[1];
            tz = tvecs.ptr<double>(0)[2];

        }
        case Enum::Mode::Rune:{
            //!辅助射击
            cv::Mat caremaMatrix = Constants::caremaMatrix_shoot;
            cv::Mat distCoeffs = Constants::distCoeffs_shoot;
            cv::solvePnP(point3D, point2D, caremaMatrix, distCoeffs, rvecs, tvecs);

            tx = tvecs.ptr<double>(0)[0];
            ty = tvecs.ptr<double>(0)[1];
            tz = tvecs.ptr<double>(0)[2];

        }
    }

    armor.tx = static_cast<float>(tx)-Constants::CompensationFactor_X;
    armor.ty = static_cast<float>(ty)-Constants::CompensationFactor_Y;
    armor.tz = static_cast<float>(tz)-Constants::CompensationFactor_Z;

}

Enum::PredictStatus ArmorDector::selectBestArmor(const ArmorData allArmor[], const unsigned short & ArmorSize){

    Eigen::Vector3f correctedPos(lastTarget.x, lastTarget.y, lastTarget.z);
    SetM(latestAngle.yaw,latestAngle.pitch);
    Rotate(correctedPos);
    float x = correctedPos[0];
    float y = correctedPos[1];
    float z = correctedPos[2];
    float X;
    float Y;
    float Z;
    unsigned short distanceT;
    unsigned short distanceSame = 0xffff;
    unsigned short distanceDifferent = 0xffff;

    unsigned short targetSameInex;
    unsigned short targetDifferentIndex;
    for(unsigned short i = 0; i < ArmorSize; i++){
        
        X = allArmor[i].tx-x;
        Y = allArmor[i].ty-y;
        Z = allArmor[i].tz-z;
        distanceT = static_cast<unsigned short>(sqrt(X*X+Y*Y+Z*Z));

        if(lastTarget.armorCatglory == allArmor[i].armorCatglory){
            if(distanceT < distanceSame){
                targetSameInex = i;
            }
        }
        else{
            if(distanceT < distanceDifferent){
                targetDifferentIndex = i;
            }
        }
        
        if(predictStatus == Enum::PredictStatus::NONE){
            if(distanceSame > distanceDifferent){
                Eigen::Vector3f t(allArmor[distanceDifferent].tx, allArmor[distanceDifferent].ty, allArmor[distanceDifferent].tz);
                ReverseRotate(t);
                lastTarget.x = t[0];
                lastTarget.y = t[1];
                lastTarget.z = t[2];
                lastTarget.armorCatglory = allArmor[distanceDifferent].armorCatglory;
                return Enum::PredictStatus::NEW;          
            }
            else if(distanceSame < distanceDifferent){
                Eigen::Vector3f t(allArmor[distanceDifferent].tx, allArmor[distanceDifferent].ty, allArmor[distanceDifferent].tz);
                ReverseRotate(t);
                lastTarget.x = t[0];
                lastTarget.y = t[1];
                lastTarget.z = t[2];
                lastTarget.armorCatglory = allArmor[distanceDifferent].armorCatglory;
                return Enum::PredictStatus::NEW;          
            }
            return  Enum::PredictStatus::NONE;
        }

        if(distanceSame != 0xffff && distanceSame < Constants::RangeOfCorrect){
            lastTarget.x = allArmor[distanceSame].tx;
            lastTarget.y = allArmor[distanceSame].ty;
            lastTarget.z = allArmor[distanceSame].tz;
            return Enum::PredictStatus::FIND;
        }
        else if(distanceDifferent != 0xffff){
            if(isFindtarget && lostTarget < Constants::LostRange){
                lostTarget++;                
                return Enum::PredictStatus::UNCLEAR;
            }
            else{
                Eigen::Vector3f t(allArmor[distanceDifferent].tx, allArmor[distanceDifferent].ty, allArmor[distanceDifferent].tz);
                ReverseRotate(t);
                lastTarget.x = t[0];
                lastTarget.y = t[1];
                lastTarget.z = t[2];
                lastTarget.armorCatglory = allArmor[distanceDifferent].armorCatglory;
                return Enum::PredictStatus::NEW;                
            }
        }
        return Enum::PredictStatus::NONE;
    }

}

void ArmorDector::SetM(float &yaw, float &pitch){
    Myaw <<   cos(yaw), 0, -sin(yaw),
                    0,          1,          0, 
                    sin(yaw), 0, cos(yaw);

    Mpitch <<   1,          0,          0,
                    0, cos(pitch), sin(pitch),
                    0, -sin(pitch),  cos(pitch);
}

void ArmorDector::ReverseRotate(Eigen::Vector3f& vec){
    vec = Myaw*Mpitch*vec;
}

void ArmorDector::Rotate(Eigen::Vector3f& vec){
    vec = Mpitch.inverse()*Myaw.inverse()*vec;
}
