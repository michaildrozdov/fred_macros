

//#include "Mmatrix.h"
#include <iostream>

#include "FloatSimple.h"
#include "MatrixSimple.h"
//foc ceil pow
#include <math.h>
#include <string>

void clear_variables()
{

}

void out(float var)
{
	std::cout << var << "\n";
}

float pi()
{
	return 3.1415926535f;
}








float* ch1 = 0; float* ch2 = 0; float* ch3 = 0; 
float* ch4 = 0; float* ch5 = 0; float* ch6 = 0;
float* ch7 = 0; float* ch8 = 0;




void Pause()
{

}




void print_line(...)
{

}



void insert(ComplexSignal& src_sig, int point_from, int point_to, ComplexSignal& dst_sig, int dst_from)
{
	for (unsigned int i = point_from; i <= point_to; ++i)
		dst_sig.complex_data[dst_from + i - point_from] = src_sig.complex_data[i];

}

void insert(FloatSignal src_sig, int point_from, int point_to, FloatSignal& dst_sig, int dst_from)
{
	for (unsigned int i = point_from; i <= point_to; ++i)
		dst_sig.float_data[dst_from + i - point_from] = src_sig.float_data[i];

}

void draw(MatrixSimple val)
{

}

int time()
{
	return 100;
}