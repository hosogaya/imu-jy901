#pragma once

#include <array>
#include "arduino-JY901-master/JY901.h"
#include <cmath>


class IMU {
    public:
        typedef enum  {
            ExitControlMode = 0x00,
            EnterGyroscopeAndAccelerometerCalibrationMode=0x01,
            EnterMagneticCalibrationMode=0x02,
            SetHeightTo0 = 0x03
        } CalibrateReg;

    private:
        const float kGravity_ = 9.80665f;
        const float kPI180_ = M_PI/180.0f;
        HardwareSerial* serial_;
        
    public:
        bool is_unsafe_ = false;
        uint32_t timeout_ = 0; // milliseconds
        bool get_data_ = true;
        std::array<float, 3> acc_;
        std::array<float, 3> pos_;
        std::array<float, 3> pos_vel_;
        
    public:
        IMU(){}
        IMU(const unsigned long baudrate, HardwareSerial& s);
        bool setup(const unsigned long baudrate, HardwareSerial& s);
        
        bool callibrate(const CalibrateReg reg);
        bool enableGyroscopeAutomaticCalibration();
        bool disableGyroscopeAutomaticCalibration();
        bool set6axisAlgorithm();
        bool set9axisAlgorithm();

        void Rx(std::array<float,3>& v, const float rad);
        void Ry(std::array<float,3>& v, const float rad);
        void Rz(std::array<float,3>& v, const float rad);

        void getAccel(std::array<float, 3>& acc);
        bool getPosture(std::array<float, 3>& pos);
        void getPostureVel(std::array<float, 3>& pos_vel);
        bool setReturnHz(const CJY901::ReturnHz hz);
        bool setBaudrate(const CJY901::BaudRate baudrate);
        bool sensing();
};