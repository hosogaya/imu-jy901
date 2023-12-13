#pragma once

#include <memory>
#include <imu_jy901/imu.h>
#include <TeensyThreads.h>

namespace imu_interface
{
class ImuDriver
{
public:
    ImuDriver();
    ~ImuDriver();

    void setup(const long baudrate, HardwareSerial& serial);
    bool ready(const float period); // milli second
    void start();
    void stop();
    void release();

    // std::array<float,3> getPos();
    // std::array<float,3> getPosVel();
    // std::array<float,3> getAcc();
    bool getData(std::array<float, 3>& pos, std::array<float, 3>& pos_vel, std::array<float, 3>& acc); 
    bool read(std::array<float, 3>& pos, std::array<float, 3>& pos_vel, std::array<float, 3>& acc); 

    struct Info
    {
        bool work_;
        std::shared_ptr<IMU> imu_;
        std::array<float, 3> posture_;
        std::array<float, 3> posture_vel_;
        std::array<float, 3> acc_;
        bool stable_;
        long period_;
        Threads::Mutex mx_;
    };

private:
};

}