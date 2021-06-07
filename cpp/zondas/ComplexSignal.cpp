
#include "ComplexSignal.h"

ComplexSignal::ComplexSignal(int _window, int _point_num)
{
	point_num = _point_num;
	window = _window;
	complex_data = new Ipp32fc[point_num];

	for (int i = 0; i < point_num; ++i)
	{
		complex_data[i].re = 0;
		complex_data[i].im = 0;
	}
}

ComplexSignal::ComplexSignal(const ComplexSignal& rhs)
{
	point_num = rhs.point_num;
	window = rhs.window;
	complex_data = new Ipp32fc[point_num];

	memcpy(complex_data, rhs.complex_data, point_num * sizeof(Ipp32fc));
}

ComplexSignal::~ComplexSignal()
{
	delete[] complex_data;
}
/*FloatSignal(const FloatSignal& rhs)
{
	memcpy(float_data, rhs.float_data, point_num * sizeof(float));
}*/

void ComplexSignal::operator=(const float rhs)
{

	for (int i = 0; i < point_num; ++i)
	{
		complex_data[i].re = rhs;
		complex_data[i].im = 0;
	}
}

void ComplexSignal::operator=(const Ipp32fc* rhs)
{

	for (int i = 0; i < point_num; ++i)
	{
		complex_data[i].re = rhs[i].re;
		complex_data[i].im = rhs[i].im;
	}
}

void ComplexSignal::operator=(const ComplexSignal& rhs)
{
	point_num = rhs.point_num;
	window = rhs.window;
	delete[] complex_data;
	complex_data = new Ipp32fc[point_num];

	for (int i = 0; i < point_num; ++i)
	{
		complex_data[i].re = rhs.complex_data[i].re;
		complex_data[i].im = rhs.complex_data[i].im;
	}
}

ComplexSignal ComplexSignal::operator-(const ComplexSignal& rhs)
{
	ComplexSignal result(0, this->point_num);
	for (int i = 0; i < this->point_num; ++i)
	{
		result.complex_data[i].re = this->complex_data[i].re -
			rhs.complex_data[i].re;
		result.complex_data[i].im = this->complex_data[i].im -
			rhs.complex_data[i].im;
	}

	return result;
}

ComplexSignal ComplexSignal::operator*(const FloatSignal& rhs)
{
	ComplexSignal result(0, this->point_num);
	for (int i = 0; i < this->point_num; ++i)
	{
		result.complex_data[i].re = this->complex_data[i].re -
			rhs.float_data[i];
		result.complex_data[i].im = this->complex_data[i].im -
			rhs.float_data[i];
	}

	return result;
}

ComplexSignal ComplexSignal::operator+(const ComplexSignal& rhs)
{
	ComplexSignal result(0, this->point_num);
	for (int i = 0; i < this->point_num; ++i)
	{
		result.complex_data[i].re = this->complex_data[i].re +
			rhs.complex_data[i].re;
		result.complex_data[i].im = this->complex_data[i].im +
			rhs.complex_data[i].im;
	}

	return result;
}

ComplexSignal ComplexSignal::operator/(const float& rhs)
{
	ComplexSignal result(0, this->point_num);
	for (int i = 0; i < this->point_num; ++i)
	{
		result.complex_data[i].re = this->complex_data[i].re /
			rhs;
		result.complex_data[i].im = this->complex_data[i].im /
			rhs;
	}

	return result;
}