
#include "ComplexSignal.h"
#include "FloatSimple.h"

#include <mkl.h>
#include "mkl_dfti.h"

#include <math.h>

ComplexSignal fft(FloatSignal signal)
{
	DFTI_DESCRIPTOR_HANDLE descriptor;
	MKL_LONG status;

	const int point_num = signal.point_num;

	status = DftiCreateDescriptor(&descriptor, DFTI_SINGLE, DFTI_REAL, 1, point_num); //Specify size and precision
	status = DftiSetValue(descriptor, DFTI_PLACEMENT, DFTI_NOT_INPLACE); //Out of place FFT
	status = DftiCommitDescriptor(descriptor); //Finalize the descriptor

	float* input = signal.float_data;
	const float output_window = 1;
	ComplexSignal result(output_window, point_num);
	Ipp32fc* output = result.complex_data;

	float koeff = signal.window / point_num;

	status = DftiComputeForward(descriptor, input, output); //Compute the Forward FFT
	status = DftiFreeDescriptor(&descriptor); //Free the descriptor

	for (int i = 0; i < point_num; ++i)
	{
		output[i].re = output[i].re * koeff;
		output[i].im = output[i].im * koeff;

	}
	return result;
}

ComplexSignal fft(ComplexSignal signal)
{
	DFTI_DESCRIPTOR_HANDLE descriptor;
	MKL_LONG status;

	const int point_num = signal.point_num;

	status = DftiCreateDescriptor(&descriptor, DFTI_SINGLE, DFTI_COMPLEX, 1, point_num); //Specify size and precision
	status = DftiSetValue(descriptor, DFTI_PLACEMENT, DFTI_NOT_INPLACE); //Out of place FFT
	status = DftiSetValue(descriptor, DFTI_FORWARD_SCALE, 1.f / point_num); //Out of place FFT
	//float cf = 0;
	//DftiGetValue(descriptor, DFTI_FORWARD_SCALE,&cf);
	status = DftiCommitDescriptor(descriptor); //Finalize the descriptor

	Ipp32fc* input = signal.complex_data;

	const float window = 1 / signal.window * (point_num - 1);

	ComplexSignal result(signal.window, point_num);
	Ipp32fc* output = result.complex_data;


	status = DftiComputeForward(descriptor, input, output); //Compute the Forward FFT
	status = DftiFreeDescriptor(&descriptor); //Free the descriptor


	return result;
}

void mkl_test()
{

}


FloatSignal mag(ComplexSignal signal)
{


	float* rResult = new float[signal.point_num];

	//ippsMagnitude_32fc(signal.complex_data, rResult, signal.point_num);

	for (int i = 0; i < signal.point_num; ++i)
	{
		rResult[i] = sqrt(signal.complex_data[i].re * signal.complex_data[i].re +
			signal.complex_data[i].im * signal.complex_data[i].im);
	}

	FloatSignal result(signal.window, signal.point_num);
	result = rResult;

	delete[] rResult;
	return result;


}


int count(FloatSignal signal)
{
	return signal.point_num;
}

FloatSignal phase(const ComplexSignal& signal)
{
	FloatSignal result(0, signal.point_num);
	ippsPhase_32fc(signal.complex_data, result.float_data, signal.point_num);
	/*for (int i = 0; i < signal.point_num; ++i)
	{
		result.float_data[i] = atan(signal.complex_data[i].im / signal.complex_data[i].re);
	}*/
	return result;
}

float mean(FloatSignal signal)
{
	float result = 0;
	//ippsMean_32f(signal.float_data, signal.point_num, &result, ippAlgHintFast);
	for (int i = 0; i < signal.point_num; ++i)
	{
		result += signal.float_data[i];
	}
	result = result / signal.point_num;
	return result;
}


/*max name is alreqady exists in current namespace*/
float max_user(FloatSignal input, int* pos)
{
	float value = 0;
	//ippsMaxIndx_32f(input.float_data, input.point_num, &value, pos);
	float max_value = input.float_data[0];
	int max_pos = 0;
	int i = 0;
	for (i = 0; i < input.point_num; ++i)
	{
		if (input.float_data[i] > max_value)
		{
			max_value = input.float_data[i];
			max_pos = i;

		}
	}

	value = max_value;
	*pos = max_pos;

	return value;
}

FloatSignal real(ComplexSignal spectr_1)
{
	FloatSignal result(spectr_1.window, spectr_1.point_num);
	//ippsReal_32fc(spectr_1.complex_data, result.float_data, spectr_1.point_num);
	for (int i = 0; i < spectr_1.point_num; ++i)
	{
		result.float_data[i] = spectr_1.complex_data[i].re;
	}
	return result;

}

//tt
FloatSignal imag(ComplexSignal spectr_1)
{
	FloatSignal result(spectr_1.window, spectr_1.point_num);
	//ippsImag_32fc(spectr_1.complex_data, result.float_data, spectr_1.point_num);
	for (int i = 0; i < spectr_1.point_num; ++i)
	{
		result.float_data[i] = spectr_1.complex_data[i].im;
	}
	return result;

}