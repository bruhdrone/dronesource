#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "cJSON.h"

typedef struct {
	uint16_t port;
	char ipv4[20]; // xxx.xxx.xxx.xxx
} PARAMETER_t;


typedef struct {
	float quatx;
	float quaty;
	float quatz;
	float quatw;
	float roll;
	float pitch;
	float yaw;
} POSE_t;

extern QueueHandle_t xQueueTrans;
extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "IMU";

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050.h" // not necessary if using MotionApps include file
//#include "MPU6050_6Axis_MotionApps20.h"

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;	// set true if DMP init was successful
uint8_t mpuIntStatus;	// holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;			// [w, x, y, z]			quaternion container
VectorInt16 aa;			// [x, y, z]			accel sensor measurements
VectorInt16 aaReal;		// [x, y, z]			gravity-free accel sensor measurements
VectorInt16 aaWorld;	// [x, y, z]			world-frame accel sensor measurements
VectorFloat gravity;	// [x, y, z]			gravity vector
float euler[3];			// [psi, theta, phi]	Euler angle container
float ypr[3];			// [yaw, pitch, roll]	yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// display quaternion values in easy matrix form: w x y z
// void getQuaternion() {
// 	mpu.dmpGetQuaternion(&q, fifoBuffer);
// 	printf("quat x:%6.2f y:%6.2f z:%6.2f w:%6.2f\n", q.x, q.y, q.z, q.w);
// }

// display Euler angles in degrees
// void getEuler() {
// 	mpu.dmpGetQuaternion(&q, fifoBuffer);
// 	mpu.dmpGetEuler(euler, &q);
// 	printf("euler psi:%6.2f theta:%6.2f phi:%6.2f\n", euler[0] * RAD_TO_DEG, euler[1] * RAD_TO_DEG, euler[2] * RAD_TO_DEG);
// }

// display Euler angles in degrees
void getYawPitchRoll() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#if 0
	float _roll = ypr[2] * RAD_TO_DEG;
	float _pitch = ypr[1] * RAD_TO_DEG;
	float _yaw = ypr[0] * RAD_TO_DEG;
	ESP_LOGI(TAG, "roll:%f pitch:%f yaw:%f",_roll, _pitch, _yaw);
#endif
	//printf("ypr roll:%3.1f pitch:%3.1f yaw:%3.1f\n",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
	ESP_LOGI(TAG, "roll:%f pitch:%f yaw:%f",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
}

// display real acceleration, adjusted to remove gravity
void getRealAccel() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	printf("areal x=%d y:%d z:%d\n", aaReal.x, aaReal.y, aaReal.z);
}

// display initial world-frame acceleration, adjusted to remove gravity
// and rotated based on known orientation from quaternion
// void getWorldAccel() {
// 	mpu.dmpGetQuaternion(&q, fifoBuffer);
// 	mpu.dmpGetAccel(&aa, fifoBuffer);
// 	mpu.dmpGetGravity(&gravity, &q);
// 	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
// 	mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
// 	printf("aworld x:%d y:%d z:%d\n", aaWorld.x, aaWorld.y, aaWorld.z);
// }

void mpu6050(float[] *outstruct){
	// Initialize mpu6050
	mpu.initialize();

	// Get Device ID
	uint8_t buffer[1];
	I2Cdev::readByte(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_WHO_AM_I, buffer);
	ESP_LOGI(TAG, "getDeviceID=0x%x", buffer[0]);

	// Initialize DMP
	devStatus = mpu.dmpInitialize();
	ESP_LOGI(TAG, "devStatus=%d", devStatus);
	if (devStatus != 0) {
		ESP_LOGE(TAG, "DMP Initialization failed [%d]", devStatus);
		while(1) {
			vTaskDelay(1);
		}
	}

	// This need to be setup individually
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXAccelOffset(-2889);
	mpu.setYAccelOffset(-444);
	mpu.setZAccelOffset(698);
	mpu.setXGyroOffset(149);
	mpu.setYGyroOffset(27);
	mpu.setZGyroOffset(17);

	// Calibration Time: generate offsets and calibrate our MPU6050
	mpu.CalibrateAccel(6);
	mpu.CalibrateGyro(6);
	mpu.setDMPEnabled(true);

	while(1){
		if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
			getYawPitchRoll();
			float _roll = ypr[2] * RAD_TO_DEG;
			float _pitch = ypr[1] * RAD_TO_DEG;
			float _yaw = ypr[0] * RAD_TO_DEG;
			outstruct[0]= _pitch;
			outstruct[1]= _roll;
			outstruct[2]= _yaw;
			getRealAccel();
			outstruct[4]= aaReal.x;
			outstruct[5]= aaReal.y;
			outstruct[6]= aaReal.z;

		}

		// Best result is to match with DMP refresh rate
		// Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
		// Now its 0x13, which means DMP is refreshed with 10Hz rate
		vTaskDelay(100/portTICK_PERIOD_MS);
	}

	// Never reach here
	vTaskDelete(NULL);
}
