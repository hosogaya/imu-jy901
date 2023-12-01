#include <imu_jy901/imu_driver.h>

extern Threads threads;
namespace imu_interface
{

ImuDriver::Info info_;
void threadRead()
{
    long start;
    while (info_.work_)
    {
        start = micros();
        info_.mx_.lock();
        info_.stable_ = info_.imu_->sensing();
        info_.mx_.unlock();
        if (info_.stable_)
        {
            info_.imu_->getPosture(info_.posture_);
            info_.imu_->getPostureVel(info_.posture_vel_);
            info_.imu_->getAccel(info_.acc_);
        }
        while (micros() - start < info_.period_) threads.yield();
    }
}

ImuDriver::ImuDriver() {}

ImuDriver::~ImuDriver() {}

void ImuDriver::setup(const long baudrate, HardwareSerial& serial)
{
    imu_ = std::make_shared<IMU>(baudrate, serial);
}

bool ImuDriver::ready(const float period) // milli second
{
    info_.period_ = period*1e3; // milli -> micro second
    return true;
}

void ImuDriver::start() 
{
    threads.addThread(threadRead);   
}

bool ImuDriver::getData(std::array<float, 3>& pos, std::array<float, 3>& pos_vel, std::array<float, 3>& acc)
{
    info_.mx_.lock();
    if (!info_.stable_) {
        info_.mx_.unlock();
        return false;
    }
    pos = info_.posture_;
    pos_vel = info_.posture_vel_;
    acc = info_.acc_;

    info_.mx_.unlock();
    return true;
}

}