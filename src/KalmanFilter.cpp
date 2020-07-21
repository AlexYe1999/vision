#include"KalmanFilter.h"
/**
 * @brief:构造函数
 */
KalmanFiler::KalmanFiler()
        :R(5),
        R_p(9),
        q(10),
        p(25),
        t(0.01),
        angleYaw(0),
        anglePitch(0),
        H(Eigen::Vector3f(1.0,0.0,0.0)),
        x(Eigen::Vector3f::Ones()),
        x_(Eigen::Vector3f::Ones()),
        P(Eigen::Matrix3f::Identity(3,3)),
        E(Eigen::Matrix3f::Identity(3,3))
{
/*P = Eigen::Matrix3f::Identity(3,3);
            //    1,0,0,//状态协方差 ,初始，位置，速读，加速度误差
            //    0,1,0,
            //    0,0,1;

        E <<1, 0 , 0, //单位矩阵
                0, 1, 0,
                0, 0, 1;
*/
        F <<1,t,0.5*t*t,  // 1,1,0.5t^2
                0,1,t,     // 0,1,t
                0,0,1;     // 0,0,1

        Q <<q,0,0, //状态转移协方差矩阵
                0,q,0,
                0,0,q;
}

/**
 * @brief:第一次识别到装甲
 * @param angle 角度目标
 * @return 下一状态
 */
void KalmanFiler::init(float &init){
        x << init,0,0; //初始化状态
        P << 1,0,0,  //不确定度
                  0,1,0,
                  0,0,1;
        P_ = P+Q; //预测不确定度
        x_ = F*x; //预测下一个状态
}


/**
 * @brief: 预测函数
 * @param currentAngle 测量值
 * @param deltaTime 时间间隔
 * @return 此刻状态
 */
Eigen::Vector3f KalmanFiler::predict(float &currentAngle){        
        K = P_*H/(H.transpose()*P_*H+R);
        x = x_+K*(currentAngle-H.transpose()*x);
        P = (E-K*H.transpose())*P_;
        P_ = F*P*F.transpose()+Q;
        x_=F*x;
 
//        std::cout<<"predict "<<std::endl<<"matrix = "<<x<<std::endl<<"matrix_ = "<<x_<<std::endl;
        return x;
}

/**
 * @brief: 丢失目标预测函数
 * @param deltaTime 时间间隔
 * @return 此刻状态
 */
Eigen::Vector3f KalmanFiler::predict_notarget(){
        float angle = x_(0);
        
        K = P_*H/(H.transpose()*P_*H+R_p);
        x = x_+K*(angle-H.transpose()*x);     
        P = (E-K*H.transpose())*P_;     
        P_ = F*P*F.transpose()+Q;
        x_=F*x;
//        std::cout<<"predict_notarget "<<std::endl<<"matrix = "<<x<<std::endl<<"matrix_ = "<<x_<<std::endl;
        return x;
}



Prediction::Prediction():
        X(),
        Y(),
        Z()
{}

void Prediction::init3D(Target &vec){
        X.init(vec.x);
        Y.init(vec.y);
        Z.init(vec.z);
}

Eigen::Vector3f Prediction::predict3D(Target &vec,float & velocity){
        Xstate = X.predict(vec.x);
        Ystate = Y.predict(vec.y);
        Zstate = Z.predict(vec.z);
        float t;
        float a = Xstate[1]*Xstate[1] + Ystate[1]*Ystate[1] + Zstate[1]*Zstate[1] - velocity*velocity;
        float b = Xstate[0]*Xstate[1] + Ystate[0]*Ystate[1] + Zstate[0]*Zstate[1];
        float c = Xstate[0]*Xstate[0] + Ystate[0]*Ystate[0] + Zstate[0]*Zstate[0];
        t = (-b+sqrt(b*b-4*a*c))/(2*a);
        return Eigen::Vector3f(Xstate[0]+t*Xstate[1], Ystate[0]+t*Ystate[1], Zstate[0]+t*Zstate[1]);
}

Eigen::Vector3f Prediction::predictNotarget3D(float & velocity){
        Xstate = X.predict_notarget();
        Ystate = Y.predict_notarget();
        Zstate = Z.predict_notarget();
        float t;
        float a = Xstate[1]*Xstate[1] + Ystate[1]*Ystate[1] + Zstate[1]*Zstate[1] - velocity*velocity;
        float b = Xstate[0]*Xstate[1] + Ystate[0]*Ystate[1] + Zstate[0]*Zstate[1];
        float c = Xstate[0]*Xstate[0] + Ystate[0]*Ystate[0] + Zstate[0]*Zstate[0];
        t = (-b+sqrt(b*b-4*a*c))/(2*a);
        return Eigen::Vector3f(Xstate[0]+t*Xstate[1], Ystate[0]+t*Ystate[1], Zstate[0]+t*Zstate[1]);

}