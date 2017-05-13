// Created by: Zongyao Jin on 4/24/2017

#ifndef L298N_H
#define L298N_H

#include "GPIO.h"
#include "PWM.h"
#include <cstdlib>

namespace ZJ {

	class L298N {

	public:
        L298N(std::string pwm, int in1, int in2);
		~L298N();
		void release();
		void runByVoltage(float voltage);

	private:
		PWM* pwmControl;
		GPIO* in1Control;
		GPIO* in2Control;
		void setCW();
		void setCCW();

};

} /* namespace ZJ */

#endif /* L298N_H */
