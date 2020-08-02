#include<algorithm>
#include<mutex>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<eigen3/Eigen/Eigen>
#include"ArmorDector.h"
#include"Constant.h"
#include"KalmanFilter.h"
#include"Debug.h"
#include"Serial.h"
using namespace Robomaster;

extern std::mutex exchangeMutex; //数据交换锁
extern ReceivedData recivedData;

#ifdef SHOW_IMAGE
extern int gamma_g;
extern cv::Mat Rune;
#endif

ArmorDector::ArmorDector():
    mode(Mode::Armor),
    predictStatus (PredictStatus::NONE),
    lostTarget(0),
    isFindtarget(0),
    lastTarget(),
    target(),
    latestAngle(),
    Myaw(),
    Mpitch(),
    lostCount(0),
    level(0),
    predict(),
    bulletVelocity(8),
    AverGray(0),
    OpenGamma(false),
    y(0),
    V(0),
    Z(0),
    T(0)
{}

/**
 * @brief 分部函数
 * @param frame 当前图像的副本
 * @param mode 射击模式
 * @param aimPos 射击点
 * @return 是否成功
 */
bool ArmorDector::StartProc(cv::Mat & frame, Eigen::Vector3f & pos){

#ifdef GAMMA_TRANSFORM    
#ifdef SHOW_IMAGE
        GammaTransf(frame, gamma_g);
        cv::imshow("gamatransform", frame);
#else
    if(predictStatus == PredictStatus::NONE 
    && lostCount > Constants::LostCountBuffer){
        OpenGamma = true;
        lostCount = 0;
        return false;
    }
#endif    
#endif

    switch(mode){
        case Mode::Armor:{
            GetArmorData(frame);
            break;
        }
        case Mode::Rune:{
            break;
        }
        default:
            return false;
    }
    float time;
    switch(predictStatus)   {
        case PredictStatus::NEW:{
            predict.init3D(lastTarget);
            isFindtarget = true;

        }break;
        case  PredictStatus::FIND:{
            pos = predict.predict3D(lastTarget, bulletVelocity, time);
            isFindtarget = true;

        }break;
        case PredictStatus::UNCLEAR:{
            lostCount++;
            pos = predict.predict3D(bulletVelocity, time);
            isFindtarget = true;

        }break;
        case PredictStatus::NONE:{
            lostCount++;
            pos = Eigen::Vector3f(0,0,1);
            isFindtarget = false;

        }break;
        default:{
        }break;


    }

    exchangeMutex.lock();
    ConfigureParam(recivedData);
    exchangeMutex.unlock();    

    Rotate(pos);
    float distance;
    float yaw;
    float pitch;
    double x = 0;
    double x1 = 1.0;
    unsigned short count = 0;


    double a =-0.5*Constants::Gravity*(pow(pos[0], 2)+pow(pos[2],2));
    double b = sqrt(pow(pos[0],2)+pow(pos[2],2))*pow(bulletVelocity,2);
    double c = -0.5*Constants::Gravity*(pow(pos[0],2)+pow(pos[2],2))-pow(bulletVelocity,2)*pos[1];
    double delta = pow(b,2)-4*a*c;


    double theta1 = atan((-b + sqrt(delta))/(2*a))/Constants::Radian;
    double theta2 = atan((-b + sqrt(delta))/(2*a))/Constants::Radian; 

/*    set(pos[1], time, bulletVelocity, pos[2]);

    while(abs(x-x1) > 0.001 && count < 30){
        x1 = x;
        x = x - f(x) / f_(x);
        count++;
    }
    isFindtarget = false;
    if(count  > 30 ) return false;
 */
    distance =sqrt(pos[0]*pos[0]+pos[1]*pos[1]+pos[2]*pos[2])/100;
    yaw = atan2(pos[0],pos[2])/Constants::Radian+1;
//    pitch = (x-latestAngle.pitch*0.015)/Constants::Radian;
    pitch = theta1-3;

#ifdef SHOW_IMAGE
    char text[255];
    sprintf(text,"px: %.2f   py: %.2f   pz: %.2f",pos[0],pos[1],pos[2]);
    cv::putText(Rune,text,cv::Point(10,10),CV_FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255),1,1);
    sprintf(text,"yaw: %.2f   pitch: %.2f   distance: %.2f", yaw, pitch, distance);
    cv::putText(Rune,text,cv::Point(10,30),CV_FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255),1,1);

#endif
    pos[0] = pitch;
    pos[1] = yaw;
    pos[2] = distance;
}


void ArmorDector::ConfigureParam(ReceivedData & data){
#ifndef NO_RECIVE_ANGLE
    latestAngle.yaw = data.yaw.f*Constants::Radian;
    latestAngle.pitch = data.pitch.f*Constants::Radian;
#else
    latestAngle.yaw = 0.0f;
    latestAngle.pitch = 0.0f;
#endif
    
    level = static_cast<unsigned int>(data.level);
    switch (level){
        case 0x00:{
            bulletVelocity = Constants::shootVelocityLevel_0*100.0f;
        }break;
        case 0x01:{
            bulletVelocity = Constants::shootVelocityLevel_1*100.0f;
        }
        case 0x02:{
            bulletVelocity = Constants::shootVelocityLevel_2*100.0f;
        }break;
        case 0x03:{
            bulletVelocity = Constants::shootVelocityLevel_3*100.0f;
        }break;
        default:{
        }break;
    }

#ifdef SHOW_IMAGE
    char text[255];
    sprintf(text,"NowYaw: %.2f   NowPitch: %.2f ",latestAngle.pitch,latestAngle.yaw);
    cv::putText(Rune,text,cv::Point(10,50),CV_FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255),1,1);
#endif
}

void ArmorDector::ConfigureData(VisionData &data,const Eigen::Vector3f &vec){
    data.distance = static_cast<char>(vec[2]);
    if(isFindtarget == true){
        data.pitchData.f = vec[0];
        data.yawData.f = vec[1];
        data.IsHaveArmor = 0x01;
    }
    else{
        data.pitchData.f = 0.0f;
        data.yawData.f = 0.0f;
        data.IsHaveArmor = 0x00;

    }
    if(vec[0] < Constants::RangeOfShoot && vec[1] < Constants::RangeOfShoot){
        data.shoot = true;
    }
    else{
        data.shoot = false;
    }
}


void ArmorDector::checkImage(cv::Mat mat){
    cv::Mat gray;
    cv::cvtColor(mat, gray, CV_BGR2GRAY);
    for(int i = 0; i < gray.rows; i++)
    {
        const uchar * p = gray.ptr<uchar>(i);
        for(int j = 0; j < gray.step; j++)
        {
            AverGray += p[j];
        }
    }
    AverGray = AverGray / (gray.rows * gray.cols * gray.channels());
}

/**
 * @brief 伽马值评估
 * 
 */
void ArmorDector::GammaTransf(cv::Mat & img, const unsigned int & gamma){
    cv::Mat lut(1, 256, CV_8U);
    uchar * p = lut.data;

    for(int i = 0; i < 256; i++)
    {
        p[i] = int(pow(float(i) / 255.0, gamma * 0.01) * 255.0);
    }
    cv::LUT(img, lut, img);
}
/**
 *@brief 设置射击模式 
 */
void ArmorDector::SetMode(volatile Mode& tMode){
    if(tMode != mode){
        predictStatus = PredictStatus::NONE;
    }

    mode = tMode;
}

/**
 *@brief 返回处于的模式
 */
Mode ArmorDector::GetMode() const {
    return mode;
}

/**
* @brief:设置获得照片时的角度
*/
void ArmorDector::SetAngle(Struct::Angle& latestAngle){
        this->latestAngle = latestAngle;
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
            predictStatus = PredictStatus::UNCLEAR;
            lostTarget++;
        }
        else{   //超过误差
            lostTarget = 0;
            isFindtarget = false;
            predictStatus = PredictStatus::NONE;
        }
        return;
    }

    armorNum = CombinateLED(ledArray, allArmor, ledNum);

    if(armorNum == 0){
        if(isFindtarget && lostTarget < Constants::LostRange){ //丢三帧以内
            predictStatus = PredictStatus::UNCLEAR;
            lostTarget++;
        }
        else{   //超过误差
            lostTarget = 0;
            isFindtarget = false;
            predictStatus = PredictStatus::NONE;
        }
        return;
    }
#ifdef SHOW_IMAGE    
    GetAngleData(allArmor, armorNum);
#endif

    predictStatus =  selectBestArmor(allArmor,  armorNum);

}

unsigned short ArmorDector::GetRuneData(const cv::Mat & frame, ArmorData allArmor[]){


}


/**
 * @brief:分离出对应颜色
 * @param frame 输入图像
 * @param binaryImage 输出目标颜色的二值化图像
 */
void ArmorDector::SeparateColor(cv::Mat frame, cv::Mat & binaryImage){
    cv::Mat splitIamge[3];

#ifdef GAMMA_TRANSFORM    
#ifdef SHOW_IMAGE
        GammaTransf(frame, gamma_g);
        cv::imshow("gamatransform", frame);
#else
    if(OpenGamma){
        GammaTransf(frame, Constants::Gamma);
    }
#endif
#endif    
    
    split(frame, splitIamge);
    
    switch(Constants::enemyColor){
        case EnemyColor::BLUE:{
            binaryImage = splitIamge[0] - splitIamge[2];
            break;
        }
        case EnemyColor::RED:{
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

#ifdef SHOW_IMAGE
    cv::imshow("binary",altimateImage);
#endif
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

#ifdef SHOW_IMAGE
            cv::line(Rune,MayTarget.upPoint,MayTarget.downPoint,cv::Scalar(255,255,255),3,8);
#endif

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
                armorArray[armorSize].armorCatglory = ArmorCatglory::LARGE;
            }
            else{
                armorArray[armorSize].armorCatglory = ArmorCatglory::SMALL;
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

#ifdef SHOW_IMAGE
    for(int i = 0;i < armorSize;i++){
        for(int i = 0;i < 4;i++){
            cv::line(Rune,armorArray[i].leftLed[0],armorArray[i].leftLed[1],cv::Scalar(0,0,255),1,8);
            cv::line(Rune,armorArray[i].rightLed[0],armorArray[i].rightLed[1],cv::Scalar(0,0,255),1,8);
            cv::line(Rune,armorArray[i].leftLed[0],armorArray[i].rightLed[0],cv::Scalar(0,0,255),1,8);
            cv::line(Rune,armorArray[i].leftLed[1],armorArray[i].rightLed[1],cv::Scalar(0,0,255),1,8);
        }
    }
    std::cout<<"Aromor Count: "<<armorSize<<"\n";
#endif

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
        case ArmorCatglory::SMALL:{
            fHalfX = Constants::rSmallArmorWidth / 2;
            fHalfY = Constants::rSmallArmorHeight / 2;
            break;
        }
        case ArmorCatglory::LARGE:{
            fHalfX = Constants::rLargeArmorWidth / 2;
            fHalfY = Constants::rLargeArmorHeight / 2;
            break;
        }
        case ArmorCatglory::RUNE:{
            fHalfX = Constants::rRuneWidth / 2;
            fHalfY = Constants::rRuneHeight / 2;
            break;
        }
        default:
            break;
    }

    point3D.push_back(cv::Point3f(-fHalfX,fHalfY, 0.0));
    point3D.push_back(cv::Point3f(fHalfX,fHalfY, 0.0));
    point3D.push_back(cv::Point3f(fHalfX,-fHalfY, 0.0));
    point3D.push_back(cv::Point3f(-fHalfX,-fHalfY, 0.0));

}

void ArmorDector::solveAngle(ArmorData & armor, const std::vector<cv::Point3f>& point3D, const std::vector<cv::Point2f>& point2D){

    cv::Mat rvecs = cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat tvecs = cv::Mat::zeros(3,1,CV_64FC1);
    double tx;
    double ty;
    double tz;

    switch(mode){
        case Mode::Armor:
        {
           //!辅助射击
            cv::Mat caremaMatrix = Constants::caremaMatrix_shoot;
            cv::Mat distCoeffs = Constants::distCoeffs_shoot;
            cv::solvePnP(point3D, point2D, caremaMatrix, distCoeffs, rvecs, tvecs);  //解算x，y，z 三轴偏移量

            tx = tvecs.ptr<double>(0)[0];
            ty = tvecs.ptr<double>(0)[1];
            tz = tvecs.ptr<double>(0)[2];

        }
        case Mode::Rune:{
            //!大能量机关
            cv::Mat caremaMatrix = Constants::caremaMatrix_shoot;
            cv::Mat distCoeffs = Constants::distCoeffs_shoot;
            cv::solvePnP(point3D, point2D, caremaMatrix, distCoeffs, rvecs, tvecs);

            tx = tvecs.ptr<double>(0)[0];
            ty = -tvecs.ptr<double>(0)[1];
            tz = tvecs.ptr<double>(0)[2];

        }
    }

    armor.tx = static_cast<float>(tx)-Constants::CompensationFactor_X;
    armor.ty = static_cast<float>(ty)-Constants::CompensationFactor_Y;
    armor.tz = static_cast<float>(tz)+Constants::CompensationFactor_Z;
}

PredictStatus ArmorDector::selectBestArmor(const ArmorData allArmor[], const unsigned short & ArmorSize){

    Eigen::Vector3f correctedPos(lastTarget.x, lastTarget.y, lastTarget.z);
    SetM(latestAngle.yaw,latestAngle.pitch);
    Rotate(correctedPos);// 变换到当前帧

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
                distanceSame = distanceT;
            }
        }
        else{
            if(distanceT < distanceDifferent){
                targetDifferentIndex = i;
                distanceDifferent = distanceT;
            }
        }
        
#ifdef SHOW_IMAGE
        std::cout<<"distanceSame: "<< distanceSame <<std::endl;
        std::cout<<"distanceDifferent: "<< distanceDifferent <<std::endl;        
#endif

        if(predictStatus == PredictStatus::NONE){
            if(distanceSame > distanceDifferent){
                Eigen::Vector3f t(allArmor[targetDifferentIndex].tx, allArmor[targetDifferentIndex].ty, allArmor[targetDifferentIndex].tz);
                ReverseRotate(t);
                lastTarget.x = t[0];
                lastTarget.y = t[1];
                lastTarget.z = t[2];
                lastTarget.armorCatglory = allArmor[targetDifferentIndex].armorCatglory;
                //target = lastTarget;
                return PredictStatus::NEW;          
            }
            else if(distanceSame < distanceDifferent){
                Eigen::Vector3f t(allArmor[targetSameInex].tx, allArmor[targetSameInex].ty, allArmor[targetSameInex].tz);
                ReverseRotate(t);
                lastTarget.x = t[0];
                lastTarget.y = t[1];
                lastTarget.z = t[2];
                lastTarget.armorCatglory = allArmor[targetSameInex].armorCatglory;
                //target = lastTarget;
                return PredictStatus::NEW;
            }
            else{
                return  PredictStatus::NONE;                
            }
        }

#ifdef SHOW_IMAGE
        std::cout<<"select Catglory"<<std::endl;
#endif

        if(distanceSame != 0xffff && distanceSame < Constants::RangeOfCorrect){
            Eigen::Vector3f t(allArmor[targetSameInex].tx, allArmor[targetSameInex].ty, allArmor[targetSameInex].tz);
            ReverseRotate(t);
/*             target.x = allArmor[targetSameInex].tx;
            target.y = allArmor[targetSameInex].ty;
            target.z = allArmor[targetSameInex].tz; */
            lastTarget.x = t[0];
            lastTarget.y = t[1];
            lastTarget.z = t[2];
            return PredictStatus::FIND;
        }
        else if(distanceDifferent != 0xffff){
            if(isFindtarget && lostTarget < Constants::LostRange){
                lostTarget++;
                return PredictStatus::UNCLEAR;
            }
            else{
                Eigen::Vector3f t(allArmor[targetDifferentIndex].tx, allArmor[targetDifferentIndex].ty, allArmor[targetDifferentIndex].tz);
                ReverseRotate(t);
                lastTarget.x = t[0];
                lastTarget.y = t[1];
                lastTarget.z = t[2];
                lastTarget.armorCatglory = allArmor[targetDifferentIndex].armorCatglory;
                return PredictStatus::NEW;                
            }
        }
        else if(lostTarget < Constants::LostRange){
            lostTarget++;                
            return PredictStatus::UNCLEAR;
        }
        return PredictStatus::NONE;
    }
}

void ArmorDector::SetM(float &yaw, float &pitch){
    Myaw <<     cos(yaw), 0, -sin(yaw),
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

double ArmorDector::f(double x){
    return y + Constants::Gravity_Half*T*T-V*sin(x)*T-Z*tan(x);
}

double ArmorDector::f_(double x){
    return Constants::Gravity*T-V*cos(x)-Z*(1/(cos(x)*cos(x)));
}

void ArmorDector::set(double y_, double t, double V_, double Z_){
    y = y_;
    T = t;
    Z = Z_;
    V = V_;
}