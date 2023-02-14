#include "controller.h"

controller::controller(const double kp, const double ki, const double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
	error_sub_ = 0.0;
	previous_error_ = 0.0;
	integral_ = 0.0;
	differential_ = 0.0;
	first_init_ = true;
}

double controller::calculateThrottleBreak(const std::vector<RefPoint>& targetPath, const msg_gen::gps &gps) {
	int index = 0;
	double min_dis = (std::numeric_limits<int>::max)();
	for (int i = 0; i < targetPath.size(); ++i) {
		double dis = pow(targetPath[i].x - gps.posX, 2) + pow(targetPath[i].y - gps.posY, 2);
		if (dis < min_dis) {

			min_dis = dis;
			index = i;
		}
	}

	return PID_Control(targetPath[index].speed, gps.v);
}

double controller::PID_Control(double value_target, double value_now) {
	
	double dt = 0.01;
	if (std::fabs(integral_) > 5) {
		reset();
	}
	this->error_sub_ = (value_target - value_now) - this->previous_error_;
	this->integral_ = this->integral_ + dt * (value_target - value_now);
	if (this->first_init_) {
		this->first_init_ = false;
	}
	else {
		this->differential_ = this->error_sub_ / dt;
	}

	double control_value = this->kp_ * (value_target - value_now) + this->ki_ * this->integral_ + this->kd_ * this->differential_;
	this->previous_error_ = value_target - value_now;
	return control_value;

	// 以下设计存在错误，但是在实际使用中效果还比较好
	//double dt = 0.01;
	//double kp = 0.30;
	//double ki = 0.1;
	//double kd = 0.01;

	//double value_p = (value_target - value_now) / value_target;
	//value_i += (value_target - value_now) * dt / value_target;
	//double value_d = (value_now - value_last) / dt;

	//double control_value = kp * value_p + ki * value_i + kd * value_d;
	////std::cout << "control_value is : " << control_value << std::endl;
	//if (control_value > 1) control_value = 1;
	//if (control_value < -1) control_value = -1;
	////std::cout << "control_value after limit is : " << control_value << std::endl;
	//std::cout << " target v:" << value_target << " now v:" << value_now << " control_value:" << control_value << std::endl;
	//value_last = value_now;
	//return control_value;
}

void controller::reset() {
	integral_ = 0;
	previous_error_ = 0;
	first_init_ = true;
}

