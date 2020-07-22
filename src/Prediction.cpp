#include <Prediction.h>


Prediction::Prediction():
        X(),
        Y(),
        Z(),
        tx(0),
        ty(0),
        tz(0)
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
        tx = Xstate[0]+t*Xstate[1];
        ty = Ystate[0]+t*Ystate[1];
        tz = Zstate[0]+t*Zstate[1];
        
        return Eigen::Vector3f(tx ,ty, tz);
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
        tx = Xstate[0]+t*Xstate[1];
        ty = Ystate[0]+t*Ystate[1];
        tz = Zstate[0]+t*Zstate[1];
        return Eigen::Vector3f(tx ,ty, tz);

}