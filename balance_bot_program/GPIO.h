/* Revised by: Zongyao Jin
 *
 * Created on: 29 Apr 2014
 * Copyright (c) 2014 Derek Molloy (www.derekmolloy.ie)
 */

#ifndef GPIO_H_
#define GPIO_H_
#include <string>
#include <fstream>
#include <iostream>
using std::string;
using std::ofstream;

#define GPIO_PATH "/sys/class/gpio/"

namespace ZJ {

/**
 * @class GPIO
 * @brief GPIO class for input and output functionality on a single GPIO pin
 */
class GPIO {
public:
	enum DIRECTION{ INPUT, OUTPUT };
	enum VALUE{ LOW=0, HIGH=1 };
	enum EDGE{ NONE, RISING, FALLING, BOTH };

private:
	int number;			/**< The GPIO number of the object */
	string name;		/**< The name of the GPIO e.g. gpio50 */
	string path;  		/**< The full path to the GPIO e.g. /sys/class/gpio/gpio50/ */

public:
	GPIO(int number);
	virtual int getNumber() { return number; } /**< Returns the GPIO number as an int. */

	// General Input and Output Settings
	virtual int setDirection(GPIO::DIRECTION);
	virtual GPIO::DIRECTION getDirection();
	virtual int setValue(GPIO::VALUE);

	virtual GPIO::VALUE getValue();
	virtual int setActiveLow(bool isLow=true);  //low=1, high=0
	virtual int setActiveHigh(); //default

	// Advanced OUTPUT: Faster write by keeping the stream alive (~20X)
	virtual int streamOpen();
	virtual int streamWrite(GPIO::VALUE);
	virtual int streamClose();

	virtual ~GPIO();  //destructor will unexport the pin

private:

	int exportGPIO();
	int unexportGPIO();
	ofstream stream;

};

} /* namespace ZJ */

#endif /* GPIO_H_ */
