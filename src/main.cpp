#include"ImageProcess.h"
#include"Serial.h"
#include<thread>

int main(){
    ImageProcess process; //处理
    std::thread t1(&ImageProcess::ImageProducer, process);
    std::thread t2(&ImageProcess::ImageConsumer, process);

#ifdef SERIAL
    Serial serial; //收发数据
    std::thread t3(&RemoteController::paraReceiver, serial);
#endif
    t1.join();
    t2.join();
#ifdef SERIAL
    t3.join();
#endif
    return 0;
}