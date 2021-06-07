
#include "FloatSimple.h"

FloatSignal::FloatSignal(int _window, int _point_num)
{
	point_num = _point_num;
	window = _window;
	float_data = new float[point_num];

	for (int i = 0; i < point_num; ++i)
		float_data[i] = 0;
}

FloatSignal::~FloatSignal()
{
	delete float_data;
}

FloatSignal::FloatSignal(const FloatSignal& rhs)
{
	point_num = rhs.point_num;
	float_data = new float[point_num];

	memcpy(float_data, rhs.float_data, point_num * sizeof(float));
	window = rhs.window;
}

void FloatSignal::operator=(float value)
{
	for (int i = 0; i < point_num; ++i)
		float_data[i] = value;
}

void FloatSignal::operator=(float* data)
{
	for (int i = 0; i < point_num; ++i)
		float_data[i] = data[i];
}

void FloatSignal::operator=(const FloatSignal& rhs)
{
	delete[] float_data;
	point_num = rhs.point_num;
	float_data = new float[point_num];
	memcpy(float_data, rhs.float_data, point_num * sizeof(float));
	window = rhs.window;
}

FloatSignal FloatSignal::operator*(const float& rhs)
{
	FloatSignal result(this->window, this->point_num);
	for (int i = 0; i < this->point_num; ++i)
		result.float_data[i] = this->float_data[i] * rhs;
	return result;
}


FloatSignal FloatSignal::operator*(const FloatSignal& rhs)
{
	FloatSignal result(this->window, this->point_num);
	for (int i = 0; i < this->point_num; ++i)
		result.float_data[i] = this->float_data[i] * rhs.float_data[i];
	return result;
}

FloatSignal FloatSignal::operator+(const FloatSignal& rhs)
{
	FloatSignal result(this->window, this->point_num);
	for (int i = 0; i < this->point_num; ++i)
		result.float_data[i] = this->float_data[i] + rhs.float_data[i];
	return result;
}

FloatSignal FloatSignal::operator/(const float& rhs)
{
	FloatSignal result(this->window, this->point_num);
	for (int i = 0; i < this->point_num; ++i)
		result.float_data[i] = this->float_data[i] / rhs;
	return result;
}

FloatSignal FloatSignal::operator-(const FloatSignal& rhs)
{
	FloatSignal result(this->window, this->point_num);
	for (int i = 0; i < this->point_num; ++i)
		result.float_data[i] = this->float_data[i] - rhs.float_data[i];
	return result;
}