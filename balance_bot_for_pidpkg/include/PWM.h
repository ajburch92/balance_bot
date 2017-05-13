/* Revised by: Zongyao Jin
 *
 * PWM.h  Created on: 29 Apr 2014
 * Copyright (c) 2014 Derek Molloy (www.derekmolloy.ie)
 */

#ifndef PWM_H_
#define PWM_H_
#include <string>
#include <iostream>
#include <fstream>
using std::string;
using std::ofstream;

// Work for PWM1A @ P9_14 (specify as 0), PWM1B @ P9_16 (specify as 1)

#define PWM1_PATH "/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/pwmchip2/"
#define PWM2_PATH "/sys/devices/platform/ocp/48304000.epwmss/48304200.ehrpwm/pwm/pwmchip0/"
#define PWM_PERIOD "period"
#define PWM_DUTY "duty_cycle"
#define PWM_POLARITY "polarity"
#define PWM_RUN "enable"

namespace ZJ {

/**
 * @class PWM
 * @brief A class to control a basic PWM output -- you must know the exact sysfs filename
 * for the PWM output.
 */
class PWM {
public:
	enum POLARITY{ ACTIVE_HIGH=0, ACTIVE_LOW=1 };

private:
	string name, path;
	float analogFrequency;  //defaults to 100,000 Hz
	float analogMax;        //defaults to 12V
	ofstream stream;
	unsigned int periodInNs;

public:
	// Either "pwm0" P9_14, or "pwm1" P9_16
	PWM(string pinName);

	virtual int setPeriod(unsigned int period_ns);
	virtual unsigned int getPeriod();
	virtual int setFrequency(float frequency_hz);
	virtual float getFrequency();
	virtual int setDutyCycle(unsigned int duration_ns);
	virtual int setDutyCyclePercentage(float percentage);
	virtual unsigned int getDutyCycle();
	virtual float getDutyCyclePercent();

	// Advanced OUTPUT: Faster write by keeping the stream alive (~20X)
	virtual int streamOpen();
	virtual int streamDutyCyclePercentage(float percentage);
	virtual int streamClose();

	virtual int setPolarity(PWM::POLARITY);
	virtual void invertPolarity();
	virtual PWM::POLARITY getPolarity();

	virtual void setAnalogFrequency(float frequency_hz) { this->analogFrequency = frequency_hz; }
	virtual int calibrateAnalogMax(float analogMax); //must be between 11 and 12
	virtual int analogWrite(float voltage);

	virtual int run();
	virtual bool isRunning();
	virtual int stop();

	virtual ~PWM();

private:
	float period_nsToFrequency(unsigned int);
	unsigned int frequencyToPeriod_ns(float);
	int exportPwmPin(string pinName);
	int unexportPwmPin(string pinName);

};

} /* namespace ZJ */

#endif /* PWM_H_ */
