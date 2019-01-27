#ifndef __SNOW_KALMAN__
#define __SNOW_KALMAN__

#include "MatLib.h"

typedef struct __state
{
	float v[4];

} state;


typedef struct __kalmanparam
{
	Mat4x4 P;
	
} param;

void SetState(state* pstate, float a, float b, float c, float d)
{
	pstate->v[0] = a;
	pstate->v[1] = b; 
	pstate->v[2] = c; 
	pstate->v[3] = d; 
}


void KalmanFilter(float t, float x, float y, state* _state1, param* _param1, float previous_t)
{
	//state state1;

	if (previous_t < 0)
	{
		/*
		// position measurement
		_state1->v[0] = x;
		_state1->v[1] = y;
		_state1->v[2] = 0;
		_state1->v[3] = 0;
		*/
		// vel measurement
		_state1->v[0] = 0;
		_state1->v[1] = 0;
		_state1->v[2] = x;
		_state1->v[3] = y;

		SetEye(&_param1->P, 100.0f);
		printf("P=\n\r");
		//Desc(_param1->P);
		printf("Initialized..\n");
		
		return;
		//return state1;
	}
	Mat4x4 P = _param1->P;
	float dt = t - previous_t;
	//state1 = _state1;
	Mat A;
	SetMat(&A,	1, 0, dt, 0,
				0, 1, 0, dt,
				0, 0, 1, 0,
				0, 0, 0, 1);
	
	// observation model (measurement model)
	Mat2x4 C;
	/*
	SetMat(&C,	1, 0, 0, 0,
				0, 1, 0, 0);
	*/
	SetMat(&C,	0, 0, 1, 0,
				0, 0, 0, 1);

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
	Vec4 X = MultVec(A, _state1->v);

	//printf("X : \t%f,%f,%f,%f\n", X.x, X.y, X.z, X.w);
	//
	// Uncertainty matrix, P
	// project the error covariance ahead
	// P = A*param.P*A' + Q;

	//A: 4x4
	//param.P : 4x4
	//Q: 4x4
	//Desc(_param1->P);
	Mat4x4 AP = MultMat(A, P);
	Mat4x4 At = TransposeOf(A);
	//Desc(AP);

	//Mat4x4 APAt = MultMat(MultMat(A, _param1->P), TransposeOf(A));
	//Desc(APAt);
	
	P = AddMat(MultMat(MultMat(A, P), TransposeOf(A)), Q);
	
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
	
	Mat4x2 PCt = MultMat(P, Ct);

	//Mat2x2 Sinv = Inverse(S);//okay
	Mat4x2 K = MultMat(PCt, Inverse(S)); //PCtSinv = P * C' * inv( S);
	//Desc(K);
	//Inverse(S)
	
	// update the estimate via z
	// C : 2x4
	// X : 4x1
	Vec2 Z;
	Z.x = x;
	Z.y = y;

	//residual = Z - (C * X); % innovation or residual
	Vec2 CX = MultVec(C, X);
	Vec2 residual;
	residual.x = Z.x - CX.x;
	residual.y = Z.y - CX.y;
	//printf("residual : \t%f,%f\n", residual.x, residual.y);

	// state = X + K * residual;
	// K : 4x2
	// resdidual : 2x1
	// X : 4x1
	// (4x2) x (2x1) = (4x1) 
	X = AddVec(X, MultVec(K, residual));

	// Update the error covariance
	//P = P - K*C*P
	// K : 4x2
	// C : 2x4
	// P : 4x4

	Mat4x4 KC = MultMat(K, C);
	Mat4x4 KCP = MultMat(KC, P);

	//Desc(KC);
	//Desc(KCP);//okay
	
	//SubtractMat(&_param1->P, P, KCP);

	P = SubtractMat(P, KCP);
	_param1->P = P;

	_state1->v[0] = X.x;
	_state1->v[1] = X.y;
	_state1->v[2] = X.z;
	_state1->v[3] = X.w;
	
	//return state1;
}

#endif