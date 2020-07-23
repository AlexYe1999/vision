#include<thread>
#include"ImageProcess.h"
#include"Serial.h"
#include"Debug.h"

int main(){
    ImageProcess process; //处理
    std::thread t1(&ImageProcess::ImageProducer, process);
    std::thread t2(&ImageProcess::ImageConsumer, process);

#ifndef SERIAL_CLOSE
    Serial serial; //收发数据
    std::thread t3(&Serial::paraReceiver, serial);
#endif

    t1.join();
    t2.join();

#ifndef SERIAL_CLOSE
    t3.join();
#endif

    return 0;
}