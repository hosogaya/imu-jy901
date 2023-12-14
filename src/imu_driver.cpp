#include <imu_jy901/imu_driver.h>

extern Threads threads;
namespace imu_interface
{

ImuDriver::Info info_;
void threadRead()
{
    long start;
    threads.setSliceMicros(10);
    while (info_.work_)
    {
        start = micros();
        info_.mx_.lock();
        info_.stable_ = info_.imu_->sensing();
        info_.mx_.unlock();
        if (info_.stable_)
        {
            info_.posture_ = info_.imu_->pos_;
            info_.posture_vel_ = info_.imu_->pos_vel_;
            info_.acc_ = info_.imu_->acc_;
        }
        while (micros() - start < info_.period_) threads.yield();
    }
}

ImuDriver::ImuDriver() {}

ImuDriver::~ImuDriver() {}

void ImuDriver::setup(const long baudrate, HardwareSerial& serial)
{
    info_.imu_ = std::make_shared<IMU>(baudrate, serial);
    info_.imu_->timeout_ = 0; //milli second
}

bool ImuDriver::ready(const float period) // milli second
{
    info_.period_ = period*1e3; // milli -> micro second
    return true;
}

void ImuDriver::start() 
{
    info_.work_ = true;
    info_.stable_ = true;
    threads.addThread(threadRead, 0, 2048);   
}

void ImuDriver::stop()
{
    info_.work_ = false;
}

void ImuDriver::release()
{
    info_.imu_.reset();
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

bool ImuDriver::read(std::array<float, 3>& pos, std::array<float, 3>& pos_vel, std::array<float, 3>& acc)
{
    if (info_.work_) return false;
    if (!info_.imu_->sensing()) return false;
    pos = info_.imu_->pos_;
    pos_vel = info_.imu_->pos_vel_;
    acc = info_.imu_->acc_;

    return true;
}

}