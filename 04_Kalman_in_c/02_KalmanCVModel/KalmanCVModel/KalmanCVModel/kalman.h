#ifndef __SNOW_KALMAN__
#define __SNOW_KALMAN__

#include "MatLib.h"

static float data_t[] = { 0,0.033426,0.066893,0.10036,0.13387,0.16729,0.20072,0.23414,0.26757,0.30113,0.33455,0.36799,0.40142,0.43484,0.46825,0.50168,0.5351,0.56853,0.602,0.63545,0.66887,0.70233,0.73575
};

static float data_px[] = { 0.52801,0.55901,0.59869,0.6268,0.65788,0.69128,0.71235,0.74407,0.81412,0.8336,0.85724,0.87474,0.90252,0.94156,0.96077,0.98929,1.0185,1.0296,1.0869,1.0586,1.0788,1.1064,1.1486
};

static float data_py[] = { -0.10764,-0.11525,-0.12134,-0.12416,-0.13478,-0.13629,-0.13921,-0.14293,-0.15084,-0.15287,-0.15805,-0.15892,-0.1614,-0.1676,-0.16888,-0.17015,-0.17548,-0.17647,-0.18173,-0.1757,-0.17761,-0.18182,-0.18522
};

typedef struct __state
{
	float v[4];

} state;


typedef struct __kalmanparam
{
	Mat4x4 P;
	

} param;

state KalmanFilter(float t, float x, float y, state _state1, param _param1, float previous_t)
{
	state state1;

	if (previous_t < 0)
	{
		state1.v[0] = x;
		state1.v[1] = y;
		state1.v[2] = 0;
		state1.v[3] = 0;

		SetEye(&_param1.P, 100.0f);

		printf("Initialized..\n");
		
		return state1;
	}
	float dt = t - previous_t;
	state1 = _state1;
	Mat A;
	SetMat(&A, 1, 0, dt, 0,
				0, 1, 0, dt,
				0, 0, 1, 0,
				0, 0, 0, 1);
	
	// observation model (measurement model)
	Mat2x4 C;
	SetMat(&C,	1, 0, 0, 0,
				0, 1, 0, 0);

	//Desc(A);
	//Desc(C);

	//state1 = _state1;
	// TODO : implement this function.. 
	
	Mat Q;
	SetEye(&Q, 10.0f);
	
	// measurement noise cov.
	Mat2x2 R;
	SetEye(&R, 0.01);

	//R = 0.01* eye(2); % 100    fail 6 / 15
	//% 0.01 pass  15 / 15


	// project the state ahead
	//X = A * state';
	Vec4 X = MultVec(A, state1.v);

	// Uncertainty matrix, P
	// project the error covariance ahead
	// P = A*param.P*A' + Q;

	//A: 4x4
	//param.P : 4x4
	//Q: 4x4
	Mat4x4 P = AddMat(MultMat(MultMat(A, _param1.P), TransposeOf(A)), Q);

	// Measurement update(correction)
	// Kalman gain
	// S = C * P * C' + R;
	// K = P * C' * inv( S);

	// S : 2x2
	// C : 2x4
	// P : 4x4
	// C': 4x2
	// CP : 2x4
	Mat2x4 CP = MultMat(C, P);
	Mat4x2 Ct = TransposeOf(C);
	Mat2x2 CPCt = MultMat(CP, Ct);
	Mat2x2 S = AddMat(CPCt, R);
	

	// TODO : inv Mat2x2


	state1.v[0] = X.x;
	state1.v[1] = X.y;
	state1.v[2] = X.z;
	state1.v[3] = X.w;
	

	return state1;
}

#endif