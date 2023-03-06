#ifndef _JY901_H_
#define _JY901_H_
#include <Arduino.h>

extern const uint8_t JY901_save_conf_cmd[5];
extern const uint8_t JY901_imu_cali_cmd[5];
extern const uint8_t JY901_mag_cali_cmd[5];
extern const uint8_t JY901_quit_cali_cmd[5];

class CJY901
{
  public:
	CJY901();                                       //构造函数
	void     attach(Stream & Serial_temp);          //绑定串口
	void     startIIC(uint8_t address = 0x50);      //设定0x50地址的IIC初始化
	bool     readSerialData(uint8_t data);          //处理接收的数据
	bool     receiveSerialData(void);               //接收串口数据
	void     readData(uint8_t address,              //address地址
	                  uint8_t length,               //长度length
	                  uint8_t data[]);              //手动读取ucLength长度的数据
	uint16_t getTime(const char*);                  //获取时间，'Y'年，'M'月，'D'天，'h'时，'m'分，'s'秒，'l'毫秒
	double   getAccX();                             //获取加速度
	double   getAccY();                             //获取加速度
	double   getAccZ();                             //获取加速度
	double   getGyroX();                            //获取角速度
	double   getGyroY();                            //获取角速度
	double   getGyroZ();                            //获取角速度
	double   getMagX();                             //获取磁场
	double   getMagY();                             //获取磁场
	double   getMagZ();                             //获取磁场
	int16_t  getAccRawX();                          //获取加速度计原始数据
	int16_t  getAccRawY();                          //获取加速度计原始数据
	int16_t  getAccRawZ();                          //获取加速度计原始数据
	int16_t  getGyroRawX();                         //获取陀螺仪原始数据
	int16_t  getGyroRawY();                         //获取陀螺仪原始数据
	int16_t  getGyroRawZ();                         //获取陀螺仪原始数据
	int16_t  getMagRawX();                          //获取磁力计原始数据
	int16_t  getMagRawY();                          //获取磁力计原始数据
	int16_t  getMagRawZ();                          //获取磁力计原始数据
	double   getRoll();
	double   getPitch();
	double   getYaw();
	int32_t  getPressure();                         //获取压力
	int32_t  getAltitude();                         //获取高度
	int16_t  getD0Status();                         //获取端口输出
	int16_t  getD1Status();                         //获取端口输出
	int16_t  getD2Status();                         //获取端口输出
	int16_t  getD3Status();                         //获取端口输出
	int32_t  getLon();                              //获取经度
	int32_t  getLat();                              //获取纬度
	double   getGPSH();                             //GPS 高度
	double   getGPSY();                             //GPS 航向角
	double   getGPSV();                             //GPS 地速
	void     saveConf();                            //保存配置
	void     quitCali();                            //退出校准
	void     caliIMU();                             //IMU校准
	void     caliMag();                             //磁力计校准
	unsigned long getLastTime();


  private:

	Stream * Serial_ = NULL;
	uint8_t address_ = 0x50;  //默认设备地址0x50
	bool    transferMode_ = 0;
	unsigned long lastTime;
	uint8_t rxBuffer[12]={0};
	uint8_t rxCnt = 0;
	void readRegisters(uint8_t deviceAddr,uint8_t addressToRead, uint8_t bytesToRead, uint8_t * dest);
	void writeRegister(uint8_t deviceAddr,uint8_t addressToWrite,uint8_t bytesToRead, uint8_t *dataToWrite);
	struct
	{
		struct
		{
			uint8_t  year;
			uint8_t  month;
			uint8_t  day;
			uint8_t  hour;
			uint8_t  minute;
			uint8_t  second;
			uint16_t milisecond;
		}time;
		struct
		{
			int16_t x;
			int16_t y;
			int16_t z;
			int16_t temperature;
		}acc;
		struct
		{
			int16_t x;
			int16_t y;
			int16_t z;
			int16_t temperature;
		}gyro;
		struct
		{
			int16_t roll;
			int16_t pitch;
			int16_t yaw;
			int16_t temperature;
		}angle;
		struct
		{
			int16_t x;
			int16_t y;
			int16_t z;
			int16_t temperature;
		}mag;
		struct
		{
			int16_t d_0;
			int16_t d_1;
			int16_t d_2;
			int16_t d_3;
		}dStatus;
		int32_t pressure;
		int32_t altitude;
		int32_t lon;
		int32_t lat;

		int16_t GPSHeight;
		int16_t GPSYaw;
		int32_t GPSVelocity;
	}JY901_data;

	// added by hosogaya
	public:
		enum Reg {
			ReturnRateReg = 0x03, 
			ReturnContentReg = 0x02,
			BaudRateReg = 0x04
		};
		enum ReturnHz {
			_0_1 = 0x01,  // 0.1 Hz
			_0_5 = 0x02,
			_1 = 0x03,// 1Hz
			_2= 0x04,
			_5= 0x05,
			_10= 0x06,
			_20= 0x07, 
			_50= 0x08,
			_100= 0x09,
			_125= 0x0a,
			_200= 0x0b,
			Single = 0x0c, 
			NotOutput= 0x0d
		};
		enum Content { //0x50 ?
			Time = 0x01,
			Acceleration = 0x02,
			AngularVelocity = 0x04,
			Angle = 0x08,
			Mangnetic = 0x10,
			PortStatus = 0x20,
			AtomosphericPressureHeight = 0x40,
			LongitudeLatitude = 0x80,
			GPSSpeed = 0x0100, 
			Quaternion = 0x0200,
			SatellitePositoniAccuracy = 0x0400
		};
		enum BaudRate {
			_2400 = 0x00,
			_4800, 
			_9600,
			_19200, 
			_38400, 
			_57600, 
			_115200, 
			_230400, 
			_460800, 
			_921600
		};
};
extern CJY901 JY901;
#endif
