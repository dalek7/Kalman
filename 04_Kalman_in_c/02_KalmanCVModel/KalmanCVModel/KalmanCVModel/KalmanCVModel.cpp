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

		status vout = KalmanFilter(t0, px, py);
		printf("%d\t%f\t%f\t%f\t-->\t%f\t%f\n", i, t0, px, py, vout.v[0], vout.v[1]);

	}
	return 0;
}

