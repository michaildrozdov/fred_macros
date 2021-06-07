#pragma once

#include <string>

#include "FloatSimple.h"
#include "ComplexSignal.h"
#include "MatrixSimple.h"

#include <math.h>

extern void clear_variables(void);
extern void out(float var);
//extern FloatSignal sig_read(std::string& path, int index);
extern float pi();
extern void insert(ComplexSignal& src_sig, int point_from, int point_to, ComplexSignal& dst_sig, int dst_from);
extern void insert(FloatSignal src_sig, int point_from, int point_to, FloatSignal& dst_sig, int dst_from);
extern void draw(MatrixSimple val);
extern void set_matrix_row(MatrixSimple& mt, int row, const FloatSignal& _data);
extern void set_matrix_row(MatrixSimple& mt, int row, const ComplexSignal& _data);
extern ComplexSignal get_matrix_row(MatrixSimple& mt, int row);
extern MatrixSimple transpose(MatrixSimple& mt);
extern FloatSignal get_matrix_row_real(int row);
extern ComplexSignal get_matrix_row(int row);
extern float get_matrix(MatrixSimple& mt, int r, int c);
//extern void set_matrix_row(MatrixSimple& mt, int row, const ComplexSignal& _data);
//extern void set_matrix_row(MatrixSimple& mt, int row, const FloatSignal& _data);
extern void sig_add(std::string pathOut, FloatSignal& aa1);
extern void sig_delete_all(std::string path);
//extern int sig_get_count(std::string path_t);
extern ComplexSignal fft(FloatSignal signal);
extern ComplexSignal fft(ComplexSignal signal);
extern void print_line(...);
extern int time();
extern void Pause();
extern FloatSignal mag(ComplexSignal signal);
extern int count(FloatSignal signal);
extern FloatSignal phase(const ComplexSignal& signal);
extern float mean(FloatSignal signal);
extern float max_user(FloatSignal input, int* pos);
extern FloatSignal real(ComplexSignal spectr_1);
extern FloatSignal imag(ComplexSignal spectr_1);

extern void create_kaiser(float* output);
extern void set_matrix(MatrixSimple& mt, int r, int c, float value);
extern MatrixSimple inverse(MatrixSimple& arg);
extern MatrixSimple inverse_mkl(MatrixSimple& mt);
//extern MatrixSimple inverse(MatrixSimple& mt);