#ifndef KALMAN_H
#define KALMAN_H

#include<iostream>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<eigen3/Eigen/Eigen>
#include<ctime>
#include"Constant.h"


using namespace Eigen;
/**
 * @brief:卡尔曼滤波
 */
class KalmanFiler{
public:
    KalmanFiler();

    void init(float &init);
    Vector3f predict(float &currentAngle);
    Vector3f predict_notarget();


private:
    const float R;//系统误差的方差 视觉
    const float R_p;
    const float q; //过程误差(计算误差的方差)
    const float p;//初始化时初始位置误差的平方
    const float t;//时间间隔

    Vector3f H;
    Vector3f x;//当前状态
    Vector3f x_;//预测值
    MatrixXf K;//卡尔曼增益
    Matrix<float, 3,3> E;//单位矩阵
    Matrix<float, 3,3> P;//状态协方差
    Matrix<float, 3,3> P_;//预测协方差
    Matrix<float, 3,3> F;//状态转移矩阵
    Matrix<float, 3,3> Q;//状态转移协方差矩阵

};

#endif // KALMAN_H
