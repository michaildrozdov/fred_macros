#pragma once

#include "base_type.h"
#include "FloatSimple.h"

class ComplexSignal
{
public:
	ComplexSignal(int _window, int _point_num);
	ComplexSignal(const ComplexSignal& rhs);
	~ComplexSignal();
	void operator=(const float rhs);
	void operator=(const Ipp32fc* rhs);
	void operator=(const ComplexSignal& rhs);
	ComplexSignal operator-(const ComplexSignal& rhs);
	ComplexSignal operator+(const ComplexSignal& rhs);
	ComplexSignal operator*(const FloatSignal& rhs);
	ComplexSignal operator/(const float& rhs);

	Ipp32fc* complex_data;
	float window;
	int point_num;
};