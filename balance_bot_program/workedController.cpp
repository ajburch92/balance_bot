// Created by: Zongyao Jin
// Created on: 05/12/2017
// License: Attribution-NonCommercial-NoDerivs 2.0 Generic (CC BY-NC-ND 2.0)
// Creative Commons 2.0 License - My code has a license! Suprise!


#include <iostream>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <ratio>
#include <iomanip>
#include <math.h>
#include <time.h>
#include "MPU9250.h"
#include "L298N.h"
#include "eqep.h"

using namespace ZJ;
using namespace std;

int main() {

	typedef std::chrono::high_resolution_clock Time;
    typedef std::chrono::nanoseconds ns;
    typedef std::chrono::duration<float> fsec;

	L298N motorL("pwm2a", 27, 65);
	L298N motorR("pwm1a", 66, 67);

	EQEP encoderR(0);
	
	MPU9250 imu(2, 0x68);
	usleep(1*1000000);
	imu.setAccRange(MPU9250::PLUSMINUS_16_G);
    imu.setGyroRange(MPU9250::PLUSMINUS_2000_DPS);
    usleep(2*1000000);

    cout << "go" << endl;

	auto start = Time::now();

	float theta = 0.0f;
	float dTheta = 0.0f;
	float phi = 0.0f;
	float thetaAccum = 0.0f;
	float thetaAdjusted = 0.0f;
	float thetaGyro = 0.0f;
	float thetaComp = 0.0f;

	float u = 0;

	for (int i = 1; i < 6000; i++) {

		auto t1 = Time::now();

		imu.readSensor();

		phi = encoderR.getRad();
		
		thetaAdjusted = 0.001f*thetaAccum;
		theta = imu.getAccRoll() * 0.017453292519943f - thetaAdjusted - 0.00722f;
		// compensating for the "always there error - calibration flaw"

		dTheta = imu.getGyroOmegaX() * 0.017453292519943f;
		thetaAccum += theta;
		thetaGyro += imu.getGyroOmegaX() * 0.017453292519943f * 0.01;
		// Numerical integration

		thetaComp = 0.95*thetaGyro + 0.05*theta;
		// Complementary filter - considering low sampling rate

		if (phi > 2.0f) { 
			phi = 2.0f;
		} else if (phi < -2.0f) {
			phi = -2.0f;
		}
		// I came up with this - lie to the control law

		
		// u = 450.0f*thetaComp + 0.8f*dTheta + 1.5f*thetaAccum + 8.0f*phi; // Also worked
		u = 400.0f*thetaComp + 0.8f*dTheta + 1.5f*thetaAccum + 10.0f*phi;
		// Tuning parameters:
		// 1. not regulating the wheels, and see how much potential 
		// 2. then regulating the wheels, and some simple calcultaion
		// 3. principle - let all terms be active in the control law
		// cont'd - even some paramters are large
		// cont'd - like when wheels 2.0 rad away, 10*2=20 Voltages seems ridiculous
		// cont'd - since it's saturating the motor
		// cont'd - but likely there is 2 deg pendulum off, and some omega
		// cont'd - so |u| < 12, and all terms are contributing to the control law

		cout << u << "    " << phi << "    " << theta << endl;
		
		if (u > 0.001f) {
			motorR.runByVoltage(1.05f*u+1.1f); // Stall voltage - suggested by Bradford - Thanks!
			motorL.runByVoltage(u+1.1f);
		} else if (u < -0.001f) {
			motorR.runByVoltage(1.05f*u-1.1f);
			motorL.runByVoltage(u-1.1f);
		}
		// Why I set |u| > 0.001f as range of action
		// there is numerical accumulated error I think sometimes say 0.0f * 2.2f
		// also maybe numercial error introduced by other calculations above
		// overtime will not be 0, so I set |u|<0.001f as numercial error region

		auto t2 = Time::now();

		usleep(10000 - std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()); 
		// to hold 100Hz, wait for whatever (0.01s - code execution time of the loop)
	}

	motorL.release();
	motorR.release();

	auto finish = Time::now();
    std::cout << "It took me " << std::chrono::duration_cast<ns>(finish-start).count() << " ns\n";

	return 0;
}