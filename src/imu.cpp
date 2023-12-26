#include <imu_jy901/imu.h>
#include <cmath>

IMU::IMU(const unsigned long baudrate, HardwareSerial& s):serial_(&s){
    this->setup(baudrate, s);
}

bool IMU::setup(const unsigned long baudrate, HardwareSerial& s)
{
    serial_ = &s;
    serial_->begin(baudrate);
    JY901.attach(*serial_);
    while (! *serial_);
    // while (!setReturnHz(CJY901::ReturnHz::_200));
    // while (!setBaudrate(CJY901::BaudRate::_230400));
}

bool IMU::setReturnHz(const CJY901::ReturnHz hz) {
    uint8_t buff_return_rate[5] ={0xFF, 0xAA, CJY901::Reg::ReturnRateReg, hz, 0x00};
    size_t num = serial_->write(buff_return_rate, 5);
    serial_->flush();
    return (num == 5);
}

bool IMU::setBaudrate(const CJY901::BaudRate buadrate) {
    uint8_t buff_baudrate[5] = {0xFF, 0xAA, CJY901::Reg::BaudRateReg, buadrate, 0x00};
    size_t num = serial_->write(buff_baudrate, 5);
    serial_->flush();

    return (num == 5);
}

bool IMU::callibrate(const CalibrateReg reg) {
    uint8_t buff[5] = {0xFF, 0xAA, 0x01, reg, 0x00};
    size_t num = serial_->write(buff, 5);
    return num == 5;
}

bool IMU::enableGyroscopeAutomaticCalibration() {
    uint8_t buff[5] = {0xFF, 0xAA, 0x63, 0x00, 0x00};
    size_t num = serial_->write(buff, 5);
    serial_->flush();

    return num == 5;
}

bool IMU::disableGyroscopeAutomaticCalibration() {
    uint8_t buff[5] = {0xFF, 0xAA, 0x63, 0x01, 0x00};
    size_t num = serial_->write(buff, 5);
    serial_->flush();

    return num == 5;
}

bool IMU::set6axisAlgorithm() {
    uint8_t buff[5] = {0xFF, 0xAA, 0x24, 0x01, 0x00};
    size_t num = serial_->write(buff, 5);
    return num == 5;    
}

bool IMU::set9axisAlgorithm() {
    uint8_t buff[5] = {0xFF, 0xAA, 0x24, 0x00, 0x00};
    size_t num = serial_->write(buff, 5);
    serial_->flush();

    return num == 5;
}

void IMU::Rx(std::array<float,3>& vec, const float rad) {
    float _cos = std::cos(rad);
    float _sin = std::sin(rad);
    float temp = -vec[1] * _sin + vec[2] * _cos;
    vec[1] = vec[1] * _cos + vec[2] * _sin;
    vec[2] = temp;
}

void IMU::Ry(std::array<float,3>& vec, const float pitch) {
    float _cos = std::cos(pitch);
    float _sin = std::sin(pitch);
    float temp = vec[0] * _sin + vec[2] * _cos;
    vec[0] = vec[0] * _cos - vec[2] * _sin;
    vec[2] = temp;
}

void IMU::Rz(std::array<float,3>& vec, const float yaw) {
    float _cos = std::cos(yaw);
    float _sin = std::sin(yaw);
    float temp = -vec[0] * _sin + vec[1] * _cos;
    vec[0] = vec[0] * _cos + vec[1] * _sin;
    vec[1] = temp;
}

void IMU::getAccel(std::array<float, 3>& acc) {
    acc[0] = JY901.getAccX();
    acc[1] = JY901.getAccY();   
    acc[2] = JY901.getAccZ();

    for (size_t i=0; i<acc.size(); ++i) acc[i] *= kGravity_*1e3f; // [g] -> [mm/s^2]
}

bool IMU::getPosture(std::array<float, 3>& pos) {
    pos[0] = static_cast<float>(JY901.getRoll());
    pos[1] = static_cast<float>(JY901.getPitch());
    pos[2] = static_cast<float>(JY901.getYaw());
    for (size_t i = 0; i < 3; ++i){
        if (pos[i] > 180) pos[i] -= 360;
        if (std::abs(pos[i]) > 75 && i != 2) {return false;} // ignore yaw angular
        pos[i] *= kPI180_;
    }
    return true;
}

void IMU::getPostureVel(std::array<float, 3>& pos_vel) {
    pos_vel[0] = JY901.getGyroX();
    pos_vel[1] = JY901.getGyroY();
    pos_vel[2] = JY901.getGyroZ();

    for (size_t i = 0; i < 3; ++i) {
        pos_vel[i] *= kPI180_;
    }
}

bool IMU::sensing() {
    is_unsafe_ = false;
    uint32_t start = millis();
    while (JY901.receiveSerialData() == false)
        if (millis() - start > timeout_ && timeout_ > 0) 
            return false;

    std::array<float, 3> pos, vel, acc;
    if (!getPosture(pos)) is_unsafe_ = true;
    getPostureVel(vel);
    pos[2] = 0.0f;
    // vel[2] = 0.0f;
    pos_vel_ = vel;
    pos_ = pos;

    getAccel(acc);
    acc_ = acc;
    get_data_ = true;
    return true;
}