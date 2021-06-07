
#include <math.h>

int T = 10;
const int M = 1024;

//W = signal(T, M, false);
float W[M];

float Al = 2;

float a0 = 0.39894228f;
float a1 = 0.01328592f;
float a2 = 0.00225319f;
float a3 = -0.00157565f;
float a4 = 0.00916281f;
float a5 = -0.02057706f;
float a6 = 0.02635537f;
float a7 = -0.01647633f;
float a8 = 0.00392377f;

float b0 = 1.f;
float b1 = 3.5156229f;
float b2 = 3.0899424f;
float b3 = 1.2067492f;
float b4 = 0.2659732f;
float b5 = 0.0360768f;
float b6 = 0.0045813f;

extern float pi();


float max_k(float* d,int size,int* pos)
{
	int pos_val = 0;
	float max_val = d[0];
	for (int i = 0; i < size; ++i)
	{
		if (d[i] > max_val)
		{
			max_val = d[i];
			pos_val = i;

		}
	}

	*pos = pos_val;
	return max_val;
}

void create_kaiser(float* output)
{
	float x = 0;
	float t = 0;
	float I0 = 0;
	float t2 = 0;

	for (int i = 0; i < M; i = i + 1)
	{
		x = Al * pi()*sqrt(1 - (2 * i / (float)M - 1)*(2 * i / (float)M - 1));

		if(x > 3.75f)
		{
			t = 3.75 / x;
			I0 = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) +
				a5 * pow(t, 5) + a6 * pow(t, 6) + a7 * pow(t, 7) + a8 * pow(t, 8);
			W[i] = I0 / sqrt(x)*exp(x);
		}
		else
		{
		t = x / 3.75;
		t2 = pow(t, 2);// t ^ 2;
		I0 = b0 + b1 * t2 + b2 * pow(t2, 2) + b3 * pow(t2, 3) + b4 * pow(t2, 4) +
			b5 * pow(t2, 5) + b6 * pow(t2, 6);
		W[i] = I0;
		}
	}
	int pos = 0;
	float Amp = max_k(W, M,&pos);
	
	//W = W / Amp;
	for (int i = 0; i < M; ++i)
	{
		W[i] /= Amp;
		output[i] = W[i];
	}
//	r16 = W;


}