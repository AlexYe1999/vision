#include"KalmanFilter.h"
#include"Constant.h"
using namespace Robomaster;
using namespace Robomaster::Class;
class Prediction{
public:
    Prediction();
    void init3D(Target &vec);
    Eigen::Vector3f predict3D(Target &vec, float & velocity);
    Eigen::Vector3f predict3D(float & velocity);

private:
    float tx;
    float ty;
    float tz;
    KalmanFiler X;
    KalmanFiler Y;
    KalmanFiler Z;
    Eigen::Vector3f Xstate;
    Eigen::Vector3f Ystate;
    Eigen::Vector3f Zstate;
};