/* Created by: Zongyao Jin
 *
 */
 
#ifndef MPU9250_H_
#define MPU9250_H_
#include "I2CDevice.h"

#define MPU9250_ADDRESS   0x68  // Device address when ADO = 0

#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D

#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40

#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48

#define CONFIG_BUFFER_SIZE      3       // config registers: 27 gyro, 28 acc1, 29 acc2
#define CONFIG_START_ADDRESS    0x1B    // 1st config register address
#define ACC_BUFFER_SIZE         6       // acc data registers: 59-64
#define ACC_START_ADDRESS       0x3B    // 1st acc register address
#define GYRO_BUFFER_SIZE        6       // gyro data registers: 67-72
#define GYRO_START_ADDRESS      0x43    // 1st gyro register address

namespace ZJ {

    /**
     * @class MPU9250
     * @brief Specific class for the MPU9250 that is a child of the I2CDevice class
     * Protected inheritance means that the public I2CDevice methods are not publicly accessible
     * by an object of the MPU9250 class.
     */

    class MPU9250 : protected I2CDevice{

        public:

            /// An enumeration to define the gravity range of the accelerometer 
            enum ACCRANGE {
                PLUSMINUS_2_G=0, PLUSMINUS_4_G=1, PLUSMINUS_8_G=2, PLUSMINUS_16_G=3
            };

            enum GYRORANGE {
                PLUSMINUS_250_DPS=0, PLUSMINUS_500_DPS=1, PLUSMINUS_1000_DPS=2, PLUSMINUS_2000_DPS=3
            }; 

        private:

            // Data - bus 
            unsigned int I2CBus, I2CAddress;
            unsigned char *configRegisters; // acc gyro config
            unsigned char *accRegisters; // acc data
            unsigned char *gyroRegisters; // gyro data

            // Data - setting 
            MPU9250::ACCRANGE accRange;
            MPU9250::GYRORANGE gyroRange;

            // Data - raw (should be regular binary, not 2's complement, check) 
            short rawXa, rawYa, rawZa; 
            short rawOmegaX, rawOmegaY, rawOmegaZ;

            // Data - converted 
            float accXa, accYa, accZa;                                 
            float gyroOmegaX, gyroOmegaY, gyroOmegaZ;

            // Data - calculated roll and pitch using trig 
            // yaw can't be derived only from aX,aY,aZ
            float accRoll, accPitch; 

            // Methods - binary calculation 
            short combineRegisters(unsigned char msb, unsigned char lsb);

            // Methods - get raw accelerometer data
            void getRawAccData();
            // Methods - convert raw Xa, Ya, Za to decimal in (g=9.81)
            void convertAccData();
            // Methods - calculate orientation from accelerometer using trig 
            void calAccOrientation(); 

            // Methods - get raw gyro data
            void getRawGyroData();
            // Methods - convert raw OmegaX, omegaY, omegaZ to decimal in deg/s 
            // deg/s -> coz the measuring range of the device is dps
            void convertGyroData();

            // I think it's better to do the sampling related calculation
            // in the control loop, not in this class
            // The class is better off just outputting the measured data
            // and data that is static w.r.t. measured data

        public:

            // Constructor - bus number (0, 1, 2) + device address (0x68 when AD0 is 0)
            MPU9250(unsigned int I2CBus, unsigned int I2CAddress = 0x68); 

            // Methods - wrapping all data acquisition and conversion to one
            void readSensor();

            // Device setting - range
            virtual void setAccRange(MPU9250::ACCRANGE accRange);
            virtual void setGyroRange(MPU9250::GYRORANGE gyroRange);

            // User getting - data from accelerometer
            virtual float getAccXa() { return accXa; }
            virtual float getAccYa() { return accYa; }
            virtual float getAccZa() { return accZa; }
            virtual float getAccRoll() { return accRoll; }
            virtual float getAccPitch() { return accPitch; }

            // User getting - data from gyro
            virtual float getGyroOmegaX() { return gyroOmegaX; } 
            virtual float getGyroOmegaY() { return gyroOmegaY; }
            virtual float getGyroOmegaZ() { return gyroOmegaZ; }

            // Display - debugging 
            virtual void displayRollPitchOmegaXYZ(int iterations = 500);
            virtual void displayRawData(int iterations = 500);
            virtual void displayConvertedData(int iterations = 500);
            virtual void displayConvertedDataOneSpace(int iterations = 500);

            // Destructor
            virtual ~MPU9250();
    };

} /* namespace ZJ */

#endif /* MPU9250_H_ */
