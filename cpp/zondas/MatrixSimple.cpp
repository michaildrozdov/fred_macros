
#include "MatrixSimple.h"
#include <mkl.h>
#include <ipp.h>

MatrixSimple::MatrixSimple(int _row_num, int _col_num)
{
	row_num = _row_num;
	col_num = _col_num;
	data = new Ipp32fc[row_num * col_num];

	const int size = row_num * col_num;
	for (int i = 0; i < size; ++i)
	{
		data[i].re = 0;
		data[i].im = 0;
	}

}

MatrixSimple::MatrixSimple(const MatrixSimple& rhs)
{
	row_num = rhs.row_num;
	col_num = rhs.col_num;
	data = new Ipp32fc[row_num * col_num];

	const int size = row_num * col_num;

	memcpy(data, rhs.data, size * sizeof(Ipp32fc));

}

MatrixSimple::~MatrixSimple()
{
	delete data;
}

int row_num;
int col_num;
Ipp32fc* data;

void MatrixSimple::operator=(int value)
{
	const int size = row_num * col_num;
	for (int i = 0; i < size; ++i)
	{
		data[i].re = 0;
		data[i].im = 0;
	}
}

void MatrixSimple::operator=(const MatrixSimple& rhs)
{
	delete[] data;
	row_num = rhs.row_num;
	col_num = rhs.col_num;
	data = new Ipp32fc[row_num * col_num];
	memcpy(data, rhs.data, sizeof(Ipp32fc) * row_num * col_num);
}

MatrixSimple MatrixSimple::operator+(const MatrixSimple& rhs)
{
	unsigned int rownum1 = this->row_num;
	unsigned int colnum1 = this->col_num;

	MatrixSimple result(rownum1, colnum1);

	Ipp32fc res;
	res.re = 0;
	res.im = 0;

	for (unsigned int i = 0; i < rownum1; ++i)
	{
		for (unsigned int j = 0; j < colnum1; ++j)
		{
			res.re = rhs.GetElement(i, j).re + this->GetElement(i, j).re;
			res.im = rhs.GetElement(i, j).im + this->GetElement(i, j).im;
			result.SetElement(i, j, res);
		}
	}

	return result;
}

MatrixSimple MatrixSimple::operator-(const MatrixSimple& rhs)
{
	MatrixSimple result(this->row_num,this->col_num);
	const int point_num = (this->row_num * this->col_num);

	for (int i = 0; i < point_num; ++i)
	{
		result.data[i].re = this->data[i].re - rhs.data[i].re;
		result.data[i].im = this->data[i].im - rhs.data[i].im;
	}

	return result;
}

void MatrixSimple::set_matrix_row(int row, const FloatSignal& float_data)
{
	//if(data.get_type() == Mfloat::TYPE_SIGNAL)
	//	mt.SetMatrixRow(row_num, data.GetRealData());
	Ipp32fc val;
	val.im = 0;
	for (int col_idx = 0; col_idx < col_num; ++col_idx)
	{
		val.re = float_data.float_data[col_idx];
		data[col_num * row + col_idx] = val;

	}
}

void MatrixSimple::set_matrix_row(int row, const ComplexSignal& _data)
{
	//if(data.get_type() == Mfloat::TYPE_SIGNAL)
	//	mt.SetMatrixRow(row_num, data.GetRealData());

	for (int col_idx = 0; col_idx < col_num; ++col_idx)
	{
		data[col_num * row + col_idx] = _data.complex_data[col_idx];
	}
}

ComplexSignal MatrixSimple::get_matrix_row(int row)
{
	ComplexSignal result(0, col_num);
	for (int col_idx = 0; col_idx < col_num; ++col_idx)
	{
		result.complex_data[col_idx] = data[col_num * row + col_idx];
	}

	return result;
}

FloatSignal MatrixSimple::get_matrix_row_real(int row)
{
	FloatSignal result(0, col_num);
	for (int col_idx = 0; col_idx < col_num; ++col_idx)
	{
		result.float_data[col_idx] = data[col_num * row + col_idx].re;
	}

	return result;
}


MatrixSimple MatrixSimple::transpose(MatrixSimple& mt)
{
	unsigned int rownum = mt.row_num;
	unsigned int colnum = mt.col_num;
	MatrixSimple newMatrix(colnum, rownum);
	Ipp32fc element;
	element.re = 0;
	element.im = 0;

	for (unsigned int i = 0; i < rownum; ++i)
	{
		for (unsigned int j = 0; j < colnum; ++j)
		{
			//element.re = mt.GetElement(i, j).re;
			element.re = mt.data[mt.col_num * i + j].re;
			element.im = mt.data[mt.col_num * i + j].im;

			newMatrix.data[rownum * j + i].re = element.re;
			newMatrix.data[rownum * j + i].im = element.im;
		}
	}

	return newMatrix;
}


ComplexSignal MatrixSimple::get_matrix_row(MatrixSimple mt, int row)
{
	ComplexSignal result(0, mt.col_num);
	for (int col_idx = 0; col_idx < mt.col_num; ++col_idx)
	{
		result.complex_data[col_idx] = mt.data[mt.col_num * row + col_idx];
	}

	return result;
}

void MatrixSimple::set_matrix_row(MatrixSimple mt, int row, const ComplexSignal& _data)
{
	//if(data.get_type() == Mfloat::TYPE_SIGNAL)
	//	mt.SetMatrixRow(row_num, data.GetRealData());

	for (int col_idx = 0; col_idx < mt.col_num; ++col_idx)
	{
		mt.data[mt.col_num * row + col_idx] = _data.complex_data[col_idx];
	}
}

void MatrixSimple::set_matrix_row(MatrixSimple mt, int row, const FloatSignal& _data)
{
	//if(data.get_type() == Mfloat::TYPE_SIGNAL)
	//	mt.SetMatrixRow(row_num, data.GetRealData());

	for (int col_idx = 0; col_idx < mt.col_num; ++col_idx)
	{
		mt.data[mt.col_num * row + col_idx].re = _data.float_data[col_idx];
		mt.data[mt.col_num * row + col_idx].im = 0;
	}
}

void MatrixSimple::SetElement(unsigned int row, unsigned int col, Ipp32fc value)
{
	data[col_num * row + col] = value;

}

const Ipp32fc MatrixSimple::GetElement(unsigned int row, unsigned int col)const
{
	return data[col_num * row + col];
}

MatrixSimple MatrixSimple::operator*(const MatrixSimple& rhs)
{
	unsigned int rownum1 = this->row_num;
	unsigned int colnum2 = rhs.col_num;
	unsigned int colnum1 = this->col_num;

	MatrixSimple result(rownum1, colnum2);

	Ipp32fc res;
	res.re = 0;
	res.im = 0;

	for (unsigned int i = 0; i < rownum1; ++i)
	{
		for (unsigned int j = 0; j < colnum2; ++j)
		{
			for (unsigned int k = 0; k < colnum1; ++k)
			{
				res.re += this->GetElement(i, k).re * rhs.GetElement(k, j).re - this->GetElement(i, k).im * rhs.GetElement(k, j).im;
				res.im += this->GetElement(i, k).im * rhs.GetElement(k, j).re + this->GetElement(i, k).re * rhs.GetElement(k, j).im;
				result.SetElement(i, j, res);
			}
			res.re = 0;
			res.im = 0;
		}
	}
	return result;
}




MatrixSimple MatrixSimple::inverse_mkl()
{
	const int input_col_num = this->col_num;
	const int input_row_num = this->row_num;
	const int matrix_size = input_col_num * input_row_num;

	double* matrix_col_major = new double[input_col_num * input_row_num];

	for (int i = 0; i < matrix_size; ++i)
	{
		int v1 = i / (input_row_num);
		int v2 = i % (input_row_num);
		matrix_col_major[i] = this->data[v1 + v2 * input_col_num].re;
	}

	int larger_dim = 0;
	if (input_col_num > input_row_num)
		larger_dim = input_col_num;
	else
		larger_dim = input_row_num;


	int lda = larger_dim;
	int* ipiv = new int[larger_dim];
	int order = larger_dim;
	double* work = new double[64 * larger_dim];
	int lwork = 64 * larger_dim;
	int info = 0;

	dgetrf(&row_num, &col_num, matrix_col_major, &lda, ipiv, &info);
	dgetri(&order, matrix_col_major, &lda, ipiv, work, &lwork, &info);

	delete[] ipiv;
	delete[] work;

	const int result_col_num = input_col_num;
	const int result_row_num = input_row_num;

	


	const int new_mat_row_num = input_col_num;
	const int new_mat_col_num = input_row_num;

	MatrixSimple result(new_mat_row_num, new_mat_col_num);

	for (int i = 0; i < matrix_size; ++i)
	{
		int v1 = i % (new_mat_col_num);
		int v2 = i / (new_mat_row_num);
		result.data[i].re = matrix_col_major[v1*new_mat_col_num + v2];
	}

	delete[] matrix_col_major;
	return result;
}

MatrixSimple inverse_mkl(MatrixSimple& mt)
{
	const int input_col_num = mt.col_num;
	const int input_row_num = mt.row_num;
	const int matrix_size = input_col_num * input_row_num;

	double* matrix_col_major = new double[input_col_num * input_row_num];

	for (int i = 0; i < matrix_size; ++i)
	{
		int v1 = i / (input_row_num);
		int v2 = i % (input_row_num);
		matrix_col_major[i] = mt.data[v1 + v2 * input_col_num].re;
	}

	int larger_dim = 0;
	if (input_col_num > input_row_num)
		larger_dim = input_col_num;
	else
		larger_dim = input_row_num;


	int lda = larger_dim;
	int* ipiv = new int[larger_dim];
	int order = larger_dim;
	double* work = new double[64 * larger_dim];
	int lwork = 64 * larger_dim;
	int info = 0;

	dgetrf(&input_row_num, &input_col_num, matrix_col_major, &lda, ipiv, &info);
	dgetri(&order, matrix_col_major, &lda, ipiv, work, &lwork, &info);

	delete[] ipiv;
	delete[] work;

	const int result_col_num = input_col_num;
	const int result_row_num = input_row_num;




	const int new_mat_row_num = input_col_num;
	const int new_mat_col_num = input_row_num;

	MatrixSimple result(new_mat_row_num, new_mat_col_num);

	for (int i = 0; i < matrix_size; ++i)
	{
		int v1 = i % (new_mat_col_num);
		int v2 = i / (new_mat_row_num);
		result.data[i].re = matrix_col_major[v1*new_mat_col_num + v2];
	}

	delete[] matrix_col_major;
	return result;
}


MatrixSimple inverse(MatrixSimple& mt)
{
	int rownum = mt.row_num;
	int colnum = mt.col_num;
	int elementCount = rownum * colnum;

	MatrixSimple result(rownum, colnum);

	/*Ipp32fc value;
	value.re = 0;
	value.im = 0;


	int count = 0;
	float* pSrc = new float[elementCount];

	for (unsigned int i = 0; i < rownum; ++i)
	{
		for (unsigned int j = 0; j < colnum; ++j)
		{
			pSrc[count] = mt.GetElement(i, j).re;
			count++;
		}
	}
	count = 0;



	int widthHeight = rownum; // Matrix dimensions, rownum = colnum, so doens't which one to use
	int srcStride2 = sizeof(Ipp32f);
	int srcStride1 = widthHeight * sizeof(Ipp32f);

	Ipp32f* pDst = new Ipp32f[elementCount];
	int dstStride2 = sizeof(Ipp32f);
	int dstStride1 = widthHeight * sizeof(Ipp32f);

	Ipp32f* pBuffer = new Ipp32f[widthHeight*widthHeight + widthHeight];

	IppStatus status = ippmInvert_m_32f((const Ipp32f*)pSrc, srcStride1,
		srcStride2, pBuffer, pDst, dstStride1, dstStride2, widthHeight);

	if (status != ippStsNoErr)
		throw 1;

	for (unsigned int i = 0; i < rownum; ++i)
	{
		for (unsigned int j = 0; j < colnum; ++j)
		{
			value.re = pDst[count];
			value.im = 0;
			result.SetElement(i, j, value);
			count++;
		}
	}
	count = 0;
	delete[] pSrc, pDst, pBuffer;*/
	return result;
}


MatrixSimple MatrixSimple::inverse()
{
	int rownum = this->row_num;
	int colnum = this->col_num;
	int elementCount = rownum * colnum;
	
	MatrixSimple result(row_num, col_num);

	/*Ipp32fc value;
	value.re = 0;
	value.im = 0;


	int count = 0;
	float* pSrc = new float[elementCount];

	for (unsigned int i = 0; i < rownum; ++i)
	{
		for (unsigned int j = 0; j < colnum; ++j)
		{
			pSrc[count] = GetElement(i, j).re;
			count++;
		}
	}
	count = 0;

	

	int widthHeight = rownum; // Matrix dimensions, rownum = colnum, so doens't which one to use
	int srcStride2 = sizeof(Ipp32f);
	int srcStride1 = widthHeight * sizeof(Ipp32f);

	Ipp32f* pDst = new Ipp32f[elementCount];
	int dstStride2 = sizeof(Ipp32f);
	int dstStride1 = widthHeight * sizeof(Ipp32f);

	Ipp32f* pBuffer = new Ipp32f[widthHeight*widthHeight + widthHeight];

	IppStatus status = ippmInvert_m_32f((const Ipp32f*)pSrc, srcStride1,
		srcStride2, pBuffer, pDst, dstStride1, dstStride2, widthHeight);

	if (status != ippStsNoErr)
		throw 1;

	for (unsigned int i = 0; i < rownum; ++i)
	{
		for (unsigned int j = 0; j < colnum; ++j)
		{
			value.re = pDst[count];
			value.im = 0;
			result.SetElement(i, j, value);
			count++;
		}
	}
	count = 0;
	delete[] pSrc, pDst, pBuffer;*/
	return result;
}

void set_matrix_row(int row, const FloatSignal& float_data)
{
	//if(data.get_type() == Mfloat::TYPE_SIGNAL)
	//	mt.SetMatrixRow(row_num, data.GetRealData());
	Ipp32fc val;
	val.im = 0;
	for (int col_idx = 0; col_idx < col_num; ++col_idx)
	{
		val.re = float_data.float_data[col_idx];
		data[col_num * row + col_idx] = val;

	}
}

void set_matrix_row(int row, const ComplexSignal& _data)
{
	//if(data.get_type() == Mfloat::TYPE_SIGNAL)
	//	mt.SetMatrixRow(row_num, data.GetRealData());

	for (int col_idx = 0; col_idx < col_num; ++col_idx)
	{
		data[col_num * row + col_idx] = _data.complex_data[col_idx];
	}
}

ComplexSignal get_matrix_row(int row)
{
	ComplexSignal result(0, col_num);
	for (int col_idx = 0; col_idx < col_num; ++col_idx)
	{
		result.complex_data[col_idx] = data[col_num * row + col_idx];
	}

	return result;
}

FloatSignal get_matrix_row_real(int row)
{
	FloatSignal result(0, col_num);
	for (int col_idx = 0; col_idx < col_num; ++col_idx)
	{
		result.float_data[col_idx] = data[col_num * row + col_idx].re;
	}

	return result;
}




ComplexSignal get_matrix_row(MatrixSimple& mt, int row)
{
	ComplexSignal result(0, mt.col_num);
	for (int col_idx = 0; col_idx < mt.col_num; ++col_idx)
	{
		result.complex_data[col_idx] = mt.data[mt.col_num * row + col_idx];
	}

	return result;
}

void set_matrix_row(MatrixSimple& mt, int row, const ComplexSignal& _data)
{
	//if(data.get_type() == Mfloat::TYPE_SIGNAL)
	//	mt.SetMatrixRow(row_num, data.GetRealData());

	for (int col_idx = 0; col_idx < mt.col_num; ++col_idx)
	{
		mt.data[mt.col_num * row + col_idx] = _data.complex_data[col_idx];
	}
}

void set_matrix_row(MatrixSimple& mt, int row, const FloatSignal& _data)
{
	//if(data.get_type() == Mfloat::TYPE_SIGNAL)
	//mt.SetMatrixRow(row_num, _data.float_data);

	//memcpy(mt.data + row * mt.col_num,_data.float_data,_data.point_num * sizeof())

	int dst_col_num = _data.point_num < mt.col_num ? _data.point_num : mt.col_num;

	for (int col_idx = 0; col_idx < dst_col_num; ++col_idx)
	{
		mt.data[mt.col_num * row + col_idx].re = _data.float_data[col_idx];
		mt.data[mt.col_num * row + col_idx].im = 0;
	}
}


float get_matrix(MatrixSimple& mt,int r, int c)
{
	return mt.data[mt.col_num * r + c].re;
}

void set_matrix(MatrixSimple& mt, int r, int c,float value)
{
	mt.data[mt.col_num * r + c].re = value;
	mt.data[mt.col_num * r + c].im = 0;
}