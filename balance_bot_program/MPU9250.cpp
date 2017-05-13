/* Created by: Zongyao Jin
 *
 */

#include "MPU9250.h"
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <iomanip>

using namespace std;

namespace ZJ {

    /**
     * Method to combine two 8-bit registers into a single short, 16-bits on the BBB. 
     * It shifts the MSB 8-bits to the left and then ORs the result with the LSB.
     * @param msb an unsigned character that contains the most significant byte
     * @param lsb an unsigned character that contains the least significant byte
     */

    short MPU9250::combineRegisters(unsigned char msb, unsigned char lsb){
        //shift the MSB left by 8 bits and OR with LSB
        return ((short)msb<<8)|(short)lsb;
    }

    void MPU9250::setAccRange(MPU9250::ACCRANGE range) {
        this->accRange = range;
        switch(this->accRange){
            case MPU9250::PLUSMINUS_16_G: // +-16g: 000-11-000 -> 0x18
                this->writeRegister(ACCEL_CONFIG, 0x18); break;
            case MPU9250::PLUSMINUS_8_G: // +-8g: 000-10-000 -> 0x10
                this->writeRegister(ACCEL_CONFIG, 0x10); break;
            case MPU9250::PLUSMINUS_4_G: // +-4g: 000-01-000 -> 0x08
                this->writeRegister(ACCEL_CONFIG, 0x08); break;
            default: // +-2g: 000-00-000 -> 0x00
                this->writeRegister(ACCEL_CONFIG, 0x00); break;
        }
    }

    void MPU9250::setGyroRange(MPU9250::GYRORANGE range) {
        this->gyroRange = range;
        switch(this->gyroRange){
            case MPU9250::PLUSMINUS_2000_DPS: // +-2000dps: 000-11-000 -> 0x18
                this->writeRegister(GYRO_CONFIG, 0x18); break;
            case MPU9250::PLUSMINUS_1000_DPS: // +-1000dps: 000-10-000 -> 0x10 
                this->writeRegister(GYRO_CONFIG, 0x10); break;
            case MPU9250::PLUSMINUS_500_DPS: // +-500dps: 000-01-000 -> 0x08 
                this->writeRegister(GYRO_CONFIG, 0x08); break;
            default: // +-250dps: 000-00-000 -> 0x00 
                this->writeRegister(GYRO_CONFIG, 0x00); break;
        }
    }

    void MPU9250::getRawAccData() {
        this->accRegisters = this->readRegisters(ACC_BUFFER_SIZE, ACC_START_ADDRESS);
        this->rawXa = this->combineRegisters(*accRegisters, *(accRegisters+1));
        this->rawYa = this->combineRegisters(*(accRegisters+2), *(accRegisters+3));
        this->rawZa = this->combineRegisters(*(accRegisters+4), *(accRegisters+5));
    }

    void MPU9250::getRawGyroData() {
        this->gyroRegisters = this->readRegisters(GYRO_BUFFER_SIZE, GYRO_START_ADDRESS);
        this->rawOmegaX = this->combineRegisters(*gyroRegisters, *(gyroRegisters+1));
        this->rawOmegaY = this->combineRegisters(*(gyroRegisters+2), *(gyroRegisters+3));
        this->rawOmegaZ = this->combineRegisters(*(gyroRegisters+4), *(gyroRegisters+5));
    }

    void MPU9250::convertAccData(){
        float gravity_range;
        switch(MPU9250::accRange){
            case MPU9250::PLUSMINUS_16_G: gravity_range=32.0f; break;
            case MPU9250::PLUSMINUS_8_G: gravity_range=16.0f; break;
            case MPU9250::PLUSMINUS_4_G: gravity_range=8.0f; break;
            default: gravity_range=4.0f; break;
        }

        float resolution = 65536.0f; // 16 bits -> 65536
        float factor = gravity_range/resolution; // 1 in raw data -> 1g*factor 

        // Since getting the roll/pitch uses ratio (trig functions)
        // so, there is no need converting to m/s in getting orientation
        // Zero position corresponds to half way through the range -> 32768
        accXa = (this->rawXa) * factor - 0.020905081065089f;
        accYa = (this->rawYa) * factor - 0.017156292721893f;
        accZa = (this->rawZa) * factor + 0.031369908284024f;
    }

    void MPU9250::convertGyroData(){
        float dps_range;
        switch(MPU9250::gyroRange){
            case MPU9250::PLUSMINUS_2000_DPS: dps_range=4000.0f; break;
            case MPU9250::PLUSMINUS_1000_DPS: dps_range=2000.0f; break;
            case MPU9250::PLUSMINUS_500_DPS: dps_range=1000.0f; break;
            default: dps_range=500.0f; break;
        }

        float resolution = 65536.0f; // 16 bits -> 65536
        float factor = dps_range/resolution; // 1 in raw data -> 1g*factor 

        // Zero position corresponds to half way through the range -> 32768
        gyroOmegaX = (this->rawOmegaX) * factor + 1.095382340236689f;
        gyroOmegaY = (this->rawOmegaY) * factor + 0.553469692307694f;
        gyroOmegaZ = (this->rawOmegaZ) * factor - 0.907762547337277f;
        // The calibration is good for +-2000dps only
    }

    void MPU9250::calAccOrientation() {
        this->accRoll = 57.29578f * atan2(accYa, sqrt(pow(accXa,2)+pow(accZa,2)));
        this->accPitch = 57.29578f * atan2(-accXa, sqrt(pow(accYa,2)+pow(accZa,2)));
    }

    /**
     * The constructor for the MPU9250 object. It passes the bus number and the
     * device address (defined in .h) to the constructor of I2CDevice. All of the states
     * are initialized and the registers are updated.
     * @param I2CBus The bus number that the MPU9250 device is 0, 1 or 2
     * @param I2CAddress The address of the IMU device 
     */

    MPU9250::MPU9250(unsigned int I2CBus, unsigned int I2CAddress):
        I2CDevice(I2CBus, I2CAddress){   
            this->I2CAddress = MPU9250_ADDRESS;
            this->I2CBus = I2CBus;
            this->rawXa = 0.0f, this->rawYa = 0.0f, this->rawZa = 0.0f;
            this->rawOmegaX = 0.0f, this->rawOmegaY = 0.0f, this->rawOmegaZ = 0.0f;
            this->accXa = 0.0f, this->accYa = 0.0f, this->accZa = 0.0f;
            this->gyroOmegaX = 0.0f, this->gyroOmegaY = 0.0f, this->gyroOmegaZ = 0.0f;
            this->accRoll = 0.0f, this->accPitch = 0.0f;
            this->configRegisters = NULL;
            this->accRange = MPU9250::PLUSMINUS_16_G;
            this->gyroRange = MPU9250::PLUSMINUS_2000_DPS;
            this->setAccRange(this->accRange);
            this->setGyroRange(this->gyroRange);
        }

    void MPU9250::readSensor() {
        getRawAccData();
        convertAccData();
        calAccOrientation();
        getRawGyroData();
        convertGyroData();
    }

    void MPU9250::displayRollPitchOmegaXYZ(int iterations){
        int count = 0;
        while(count < iterations){
            // cout << "aX: " << this->getAccXa() << ", aY: " << this->getAccYa() << ", aZ: " << this->getAccZa() << endl;
            cout << "Roll: " << setw(12) << this->getAccRoll() 
                 << ", Pitch: " << setw(12) << this->getAccPitch();
            cout << ", wX: " << setw(12) << this->getGyroOmegaX() 
                 << ", wY: " << setw(12) << this->getGyroOmegaY() 
                 << ", wZ: " << setw(12) << this->getGyroOmegaZ() << endl;
            usleep(100000);
            this->readSensor();
            count++;
        }
    }

    void MPU9250::displayConvertedData(int iterations) {
        int count = 0;
        cout << "Ax-Ay-Az-Wx-Wy-Wz" << endl;
        while(count < iterations) {
            cout << "aX: " << setw(15) << this->accXa
                 << ",  aY: " << setw(15) << this->accYa
                 << ",  aZ: " << setw(15) << this->accZa 
                 << ",  wX: " << setw(15) << this->gyroOmegaX
                 << ",  wY: " << setw(15) << this->gyroOmegaY
                 << ",  wZ: " << setw(15) << this->gyroOmegaZ << endl; 
            usleep(100000);
            this->readSensor();
            count++;
        }
    }    

    void MPU9250::displayConvertedDataOneSpace(int iterations) {
        int count = 0;
        cout << "Ax-Ay-Az-Wx-Wy-Wz" << endl;
        while(count < iterations) {
            cout << this->accXa
                 << " " << this->accYa
                 << " " << this->accZa 
                 << " " << this->gyroOmegaX
                 << " " << this->gyroOmegaY
                 << " " << this->gyroOmegaZ << endl; 
            usleep(1000);
            this->readSensor();
            count++;
        }
    }    

    void MPU9250::displayRawData(int iterations) {
        int count = 0;
        while(count < iterations) {
            cout << "RawXa: " << setw(6) << this->rawXa
                 << ", RawYa: " << setw(6) << this->rawYa
                 << ", RawZa: " << setw(6) << this->rawZa 
                 << ", RawWx: " << setw(6) << this->rawOmegaX
                 << ", RawWy: " << setw(6) << this->rawOmegaY
                 << ", RawWz: " << setw(6) << this->rawOmegaZ << endl;
            usleep(100000);
            this->readSensor();
            count++;
        }
    }

    MPU9250::~MPU9250() {}

} /* namespace ZJ */
