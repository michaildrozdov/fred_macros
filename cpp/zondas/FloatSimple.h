#pragma once

#include <cstring>

class FloatSignal
{
public:
	FloatSignal(int _window, int _point_num);

	~FloatSignal();
	
	FloatSignal(const FloatSignal& rhs);

	void operator=(float value);

	void operator=(float* data);

	void operator=(const FloatSignal& rhs);

	FloatSignal operator*(const float& rhs);
	


	FloatSignal operator*(const FloatSignal& rhs);

	FloatSignal operator/(const float& rhs);
	FloatSignal operator+(const FloatSignal& rhs);

	FloatSignal operator-(const FloatSignal& rhs);

	float* float_data;
	float window;
	int point_num;
};