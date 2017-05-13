/* Revised by: Zongyao Jin
 *
 * GPIO.cpp  Created on: 29 Apr 2014
 * Copyright (c) 2014 Derek Molloy (www.derekmolloy.ie)
 */

#include "GPIO.h"
#include "util.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/epoll.h>
#include <pthread.h>
using namespace std;

namespace ZJ {
/**
 * The constructor will set up the states and export the pin.
 * @param number The GPIO number to be exported
 */
GPIO::GPIO(int number) {

	this->number = number;
	
	ostringstream s;
	s << "gpio" << number;
	this->name = string(s.str());
	this->path = GPIO_PATH + this->name + "/";
	this->exportGPIO();
	this->streamOpen();
	// need to give Linux time to set up the sysfs structure
	usleep(250000); // 250ms delay
	// std::cout << "path -> " << this->path << std::endl;
}

int GPIO::exportGPIO(){
   return write(GPIO_PATH, "export", this->number);
}

int GPIO::unexportGPIO(){
   return write(GPIO_PATH, "unexport", this->number);
}

int GPIO::setDirection(GPIO::DIRECTION dir){
   switch(dir){
   case INPUT: return write(this->path, "direction", "in");
      // std::cout << "writing direction" << std::endl;
      break;
   case OUTPUT:return write(this->path, "direction", "out");
      // std::cout << "writing direction" << std::endl;
      break;
   }
   return -1;
}

int GPIO::setValue(GPIO::VALUE value){
   switch(value){
   case HIGH: return write(this->path, "value", "1");
      break;
   case LOW: return write(this->path, "value", "0");
      break;
   }
   return -1;
}

int GPIO::setActiveLow(bool isLow){
   if(isLow) return write(this->path, "active_low", "1");
   else return write(this->path, "active_low", "0");
}

int GPIO::setActiveHigh(){
   return this->setActiveLow(false);
}

GPIO::VALUE GPIO::getValue(){
	string input = read(this->path, "value");
	if (input == "0") return LOW;
	else return HIGH;
}

GPIO::DIRECTION GPIO::getDirection(){
	string input = read(this->path, "direction");
	if (input == "in") return INPUT;
	else return OUTPUT;
}

int GPIO::streamOpen(){
	stream.open((path + "value").c_str());
	return 0;
}

int GPIO::streamWrite(GPIO::VALUE value){
	stream << value << std::flush;
	return 0;
}

int GPIO::streamClose(){
	stream.close();
	return 0;
}

GPIO::~GPIO() {
	// this->streamClose();
	// this->unexportGPIO();
}

} /* namespace ZJ */
