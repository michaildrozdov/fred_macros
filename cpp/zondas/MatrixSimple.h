#pragma once

#include "ComplexSignal.h"

class MatrixSimple
{
public:
	MatrixSimple(int _row_num, int _col_num);
	MatrixSimple(const MatrixSimple& rhs);
	~MatrixSimple();
	int row_num;
	int col_num;
	Ipp32fc* data;
	void operator=(int value);
	void operator=(const MatrixSimple& rhs);
	MatrixSimple operator-(const MatrixSimple& rhs);
	MatrixSimple operator+(const MatrixSimple& rhs);
	MatrixSimple operator*(const MatrixSimple& rhs);
	void set_matrix_row(int row, const FloatSignal& float_data);
	void set_matrix_row(int row, const ComplexSignal& _data);
	ComplexSignal get_matrix_row(int row);
	FloatSignal get_matrix_row_real(int row);
	static MatrixSimple transpose(MatrixSimple& mt);
	ComplexSignal get_matrix_row(MatrixSimple mt, int row);
	void set_matrix_row(MatrixSimple mt, int row, const ComplexSignal& _data);
	void set_matrix_row(MatrixSimple mt, int row, const FloatSignal& _data);
	void SetElement(unsigned int row, unsigned int col, Ipp32fc value);
	const Ipp32fc GetElement(unsigned int row, unsigned int col)const;
	MatrixSimple inverse();
	MatrixSimple inverse_mkl();
};