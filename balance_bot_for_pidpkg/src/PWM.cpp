/* Revised by: Zongyao Jin
 *
 * PWM.cpp  Created on: 29 Apr 2014
 * Copyright (c) 2014 Derek Molloy (www.derekmolloy.ie)
 */

#include "PWM.h"
#include "util.h"
#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <errno.h>
#include <string>

namespace ZJ {

PWM::PWM(string pinName) {
	this->name = pinName;
	this->exportPwmPin(pinName);

	if (pinName == "pwm1a") this->path = PWM1_PATH + std::string("pwm0/");
	else if (pinName == "pwm1b") this->path = PWM1_PATH + std::string("pwm1/");
	else if (pinName == "pwm2a") this->path = PWM2_PATH + std::string("pwm0/");
	else if (pinName == "pwm2b") this->path = PWM2_PATH + std::string("pwm1/");
	else perror("Wrong PWM channel input. Error!");

	// this->path = PWM_PATH + this->name + "/";
	this->setFrequency(20000);
	// std::cout << this->getPeriod() << std::endl;
	this->periodInNs = this->getPeriod();
	this->analogFrequency = this->getFrequency();
	this->analogMax = 12;
	this->streamOpen();
	this->streamDutyCyclePercentage(0);
	this->run();
}

int PWM::setPeriod(unsigned int period_ns){
	return write(this->path, PWM_PERIOD, period_ns);
}

unsigned int PWM::getPeriod(){
	return atoi(read(this->path, PWM_PERIOD).c_str());
}

float PWM::period_nsToFrequency(unsigned int period_ns){
	float period_s = (float)period_ns/1000000000;
	return 1.0f/period_s;
}

unsigned int PWM::frequencyToPeriod_ns(float frequency_hz){
	float period_s = 1.0f/frequency_hz;
	return (unsigned int)(period_s*1000000000);
}

int PWM::setFrequency(float frequency_hz){
	return this->setPeriod(this->frequencyToPeriod_ns(frequency_hz));
}

float PWM::getFrequency(){
	return this->period_nsToFrequency(this->getPeriod());
}

int PWM::setDutyCycle(unsigned int duty_ns){
	return write(this->path, PWM_DUTY, duty_ns);
}

int PWM::setDutyCyclePercentage(float percentage){
	if ((percentage>100.0f)||(percentage<0.0f)) return -1;
	// std::cout << this->getPeriod() << std::endl;
	float duty_ns = periodInNs * (percentage/100.0f);
	this->setDutyCycle((unsigned int) duty_ns );
	return 0;
}

unsigned int PWM::getDutyCycle(){
	return atoi(read(this->path, PWM_DUTY).c_str());
}

float PWM::getDutyCyclePercent(){
	unsigned int period_ns = this->getPeriod();
	unsigned int duty_ns = this->getDutyCycle();
	return 100.0f * (float)duty_ns/(float)period_ns;
}

//----------------------------------

int PWM::streamOpen(){
	stream.open((path + "duty_cycle").c_str());
	return 0;
}

int PWM::streamDutyCyclePercentage(float percentage){
	if ((percentage>100.0f)||(percentage<0.0f)) return -1;
	unsigned int duty_ns = periodInNs * (percentage/100.0f);
	stream << duty_ns << std::flush;
	// std::cout << duty_ns << std::endl;
	return 0;
}

int PWM::streamClose(){
	stream.close();
	return 0;
}

//----------------------------------

int PWM::setPolarity(PWM::POLARITY polarity){
	return write(this->path, PWM_POLARITY, polarity);
}

void PWM::invertPolarity(){
	if (this->getPolarity()==PWM::ACTIVE_LOW) this->setPolarity(PWM::ACTIVE_HIGH);
	else this->setPolarity(PWM::ACTIVE_LOW);
}

PWM::POLARITY PWM::getPolarity(){
	if (atoi(read(this->path, PWM_POLARITY).c_str())==0) return PWM::ACTIVE_LOW;
	else return PWM::ACTIVE_HIGH;
}

int PWM::calibrateAnalogMax(float analogMax){ 
	if((analogMax<11.0f) || (analogMax>12.0f)) return -1;
	else this->analogMax = analogMax;
	return 0;
}

int PWM::analogWrite(float voltage){
	if ((voltage<0.0f)||(voltage>12.0f)) return -1;
	this->setFrequency(this->analogFrequency);
	this->setPolarity(PWM::ACTIVE_LOW);
	this->setDutyCycle((100.0f*voltage)/this->analogMax);
	return this->run();
}

int PWM::run(){
	return write(this->path, PWM_RUN, 1);
}

bool PWM::isRunning(){
	string running = read(this->path, PWM_RUN);
	return (running=="1");
}

int PWM::stop(){
	return write(this->path, PWM_RUN, 0);
}

int PWM::exportPwmPin(string pinName) {
	// std::cout << this->path << pinName << "export" << std::endl;

	if (pinName == "pwm1a") return write(PWM1_PATH, "export", 0);
	else if (pinName == "pwm1b") return write(PWM1_PATH, "export", 1);
	else if (pinName == "pwm2a") return write(PWM2_PATH, "export", 0);
	else if (pinName == "pwm2b") return write(PWM2_PATH, "export", 1);
	else {
		perror("Wrong PWM channel input. Error!");
		return -1;
	}

}

int PWM::unexportPwmPin(string pinName) {
	// std::cout << this->path << pinName << "export" << std::endl;

	if (pinName == "pwm1a") return write(PWM1_PATH, "unexport", 0);
	else if (pinName == "pwm1b") return write(PWM1_PATH, "unexport", 1);
	else if (pinName == "pwm2a") return write(PWM2_PATH, "unexport", 0);
	else if (pinName == "pwm2b") return write(PWM2_PATH, "unexport", 1);
	else {
		perror("Wrong PWM channel input. Error!");
		return -1;
	}

}

PWM::~PWM() {
	// this->streamClose();
	//this->unexportPwmPin(this->name);
}

} /* namespace ZJ */
