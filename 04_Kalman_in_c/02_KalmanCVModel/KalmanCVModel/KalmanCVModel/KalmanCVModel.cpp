// KalmanCVModel.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "kalman.h"

int main()
{
	
	int sz = 10;

	for (int i = 0; i < sz; i++)
	{
		float t0 = data_t[i];
		float px = data_px[i];
		float py = data_py[i];

		printf("%d\t%f\t%f\t%f\n", i, t0, px, py);

	}
	return 0;
}

