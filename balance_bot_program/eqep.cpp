/* Created by: Zongyao Jin
 *
 */

#include "eqep.h"
#include "util.h"
#include <iostream>
#include <fstream>
#include <cstdlib>

namespace ZJ {

EQEP::EQEP(bool i) {

	if (i == 0) this->path = EQEP_A;
	else this->path = EQEP_B;

	this->setZero();
	this->position = this->getPosition();
	// this->streamOpen();

}

int EQEP::setZero(){
	return write(this->path, EQEP_POSITION, 0);
}

int EQEP::getPosition(){
	return atoi(read(this->path, EQEP_POSITION).c_str());
}

void EQEP::updateSensor() {
	position = this->getPosition();
}

float EQEP::getRad(){
	this->updateSensor();
	return (position/COUNTS_PER_REV * 2.0f * PI);
}

float EQEP::getDeg(){
	this->updateSensor();
	return (position/COUNTS_PER_REV * 360.0f);
}

// ----------------------

// int EQEP::streamOpen() {
// 	is.open((path + "position").c_str(), std::ifstream::binary);
// 	std::cout << "in streamOpen() " << std::endl;
// 	return 0;
// }

// int EQEP::streamPosition() {

// 	std::cout << "in streamPosition() 1" << std::endl;
// 	// get length of file:
//     is.seekg (0, is.end);
//     int length = is.tellg();
//     is.seekg (0, is.beg);
//     std::cout << "in streamPosition() 2" << std::endl;

//     char * buffer = new char [length];
//     is.read(buffer,length);
//     std::cout << int(&buffer) << std::endl;

//     if (is) {
//     	int(&buffer) >> this->position;
//     	return 0;
//     } else {
//     	return -1;
//     }
//     std::cout << "in streamPosition() 4" << std::endl;

//     std::cout << buffer << std::endl;

//     delete[] buffer; 
// }

// float EQEP::streamRad() {
// 	this->streamPosition();
// 	return (position/COUNTS_PER_REV * 2.0f * PI);
// }

// float EQEP::streamDeg() {
// 	std::cout << "in streamDeg() " << std::endl;
// 	this->streamPosition();
// 	return (position/COUNTS_PER_REV * 360.0f);
// }


// int EQEP::streamClose(){
// 	is.close();
// 	return 0;
// }

// ----------------------

// int EQEP::streamOpen() {
// 	// Read the position
// 	fp = fopen((this->path + "/position").c_str(), "r");

// 	if(fp == NULL)
//     {
//         // Error, break out
//         std::cerr << "[eQEP " << this->path << "] Unable to open position for read" << std::endl;
//         return -1;
//     }

//     return 0;
// }

// int EQEP::streamPosition() {
// 	// Write the desired value to the file
//     fscanf(fp, "%d", &position);
//     return this->position; 
// }

// float EQEP::streamRad() {
// 	this->streamPosition();
// 	return (position/COUNTS_PER_REV * 2.0f * PI);
// }

// float EQEP::streamDeg() {
// 	this->streamPosition();
// 	return (position/COUNTS_PER_REV * 360.0f);
// }


// int EQEP::streamClose(){
// 	fclose(fp);
// 	return 0;
// }

// ----------------------



int EQEP::run(){
	// It seems that if the DTO is deployed, enable is always 1.
	// And we don't have permission to enable it again.
	return write(this->path, EQEP_ENABLE, 1);
}

bool EQEP::isRunning(){
	string running = read(this->path, EQEP_ENABLE);
	return (running=="1");
}

int EQEP::stop(){
	return write(this->path, EQEP_ENABLE, 0);
}

EQEP::~EQEP() {}

} /* namespace ZJ */
