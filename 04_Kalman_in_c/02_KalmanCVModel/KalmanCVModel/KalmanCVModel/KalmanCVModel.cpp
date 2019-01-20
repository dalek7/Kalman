// KalmanCVModel.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "kalman.h"
#include "MatLib.h"
int main()
{
	//testinv();
	//return 1;
	int sz = 10;
	float last_t = -1;
	state state1;
	param param1;
	param1.P = Zeros();

	Desc(param1.P);

	state1.v[0] = 0;
	state1.v[1] = 0;
	state1.v[2] = 0;
	state1.v[3] = 0;

	
	for (int i = 0; i < sz; i++)
	{
		float t0 = data_t[i];
		float px = data_px[i];
		float py = data_py[i];

		//loat t, float x, float y, state state1, param param1, float previous_t)
		printf("==%d==\n\r", i);
		 KalmanFilter(t0, px, py, &state1, &param1, last_t);
		//printf("%d\t%f\t%f\t%f\t-->\t%f\t%f\n", i, t0, px, py, vout.v[0], vout.v[1]);
		//Desc(param1.P);
		printf("\n\r");
		//state1 = vout;

		last_t = t0;
	}
	
	return 0;
}

