// KalmanCVModel.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "kalman.h"
#include "MatLib.h"
#include "data2load.h"

int main()
{
	//int sz = 10;
	int sz = sizeof(data_t) / sizeof(float);
	float last_t = -1;
	state state1;
	param param1;
	param1.P = Zeros();

	//Desc(param1.P);
	SetState(&state1, 0, 0, 0, 0);
	
	for (int i = 0; i < sz; i++)
	{
		float t0 = data_t[i];
		float px = data_px[i];
		float py = data_py[i];

		//loat t, float x, float y, state state1, param param1, float previous_t)
		KalmanFilter(t0, px, py, &state1, &param1, last_t);
		//if (i == 0)
		

		//printf("%d\t%f\t%f\t%f\t-->\t%f\t%f\n", i, t0, px, py, state1.v[0], state1.v[1]);
		printf("%d\t%f\t%f\t%f\t%f\n", i, state1.v[0], state1.v[1], state1.v[2], state1.v[3]);
		Desc(param1.P);
		
		printf("\n\n");

		last_t = t0;
	}
	
	//printf("Final P =\n");
	//Desc(param1.P);

	return 0;
}
