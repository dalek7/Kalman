#ifndef __SNOW_MATH__
#define __SNOW_MATH__

#define INNER_PRODUCT(a,b,r,c) \
 ((a).v[r][0] * (b).v[0][c]) \
+((a).v[r][1] * (b).v[1][c]) \
+((a).v[r][2] * (b).v[2][c]) \
+((a).v[r][3] * (b).v[3][c])

typedef struct _vec4
{
	float x;
	float y;
	float z;
	float w;
} Vec4;


typedef struct _mat4x4
{
	float v[4][4];
} Mat4x4;

typedef struct _mat2x4
{
	float v[2][4];
} Mat2x4;

typedef struct _mat2x2
{
	float v[2][2];
} Mat2x2;

typedef Mat4x4 Mat;

void SetMat(Mat4x4* mat,
		  float a00, float a01, float a02, float a03,
		  float a10, float a11, float a12, float a13,
		  float a20, float a21, float a22, float a23,
		  float a30, float a31, float a32, float a33);

void SetMat(Mat2x4* mat,
	float a00, float a01, float a02, float a03,
	float a10, float a11, float a12, float a13);

void SetVec(Vec4* vec, float *v);
void SetMatRow(Mat4x4* mat, int _nRow, Vec4 _vec);
void SetMatCol(Mat4x4* mat, int _nCol, Vec4 _vec);




void Transpose(Mat4x4* mat);
Mat4x4 TransposeOf(Mat4x4 _mat);

Mat4x4 MultMat(Mat4x4 lhs, Mat4x4 rhs);
Vec4 MultVec(Mat4x4 _mat1, Vec4 _vec1);

// to be added
/*
matrix sum
vector sum
matrix inverse
inner product
cross product
*/



void SetVec(Vec4* vec, float *v)
{
	vec->x = v[0];
	vec->y = v[1];
	vec->z = v[2];
	vec->w = v[3];
}

void SetMatRow(Mat4x4* mat, int _nRow, float *_v)
{
	mat->v[_nRow][0] = _v[0];
	mat->v[_nRow][1] = _v[1];
	mat->v[_nRow][2] = _v[2];
	mat->v[_nRow][3] = _v[3];
}

void SetMatRow(Mat4x4* mat, int _nRow, float v1, float v2, float v3, float v4)
{
	mat->v[_nRow][0] = v1;
	mat->v[_nRow][1] = v2;
	mat->v[_nRow][2] = v2;
	mat->v[_nRow][3] = v3;
}

void SetMatRow(Mat4x4* mat, int _nRow, Vec4 _vec)
{
	mat->v[_nRow][0] = _vec.x;
	mat->v[_nRow][1] = _vec.y;
	mat->v[_nRow][2] = _vec.z;
	mat->v[_nRow][3] = _vec.w;
}

void SetMatCol(Mat4x4* mat, int _nCol, Vec4 _vec)
{
	mat->v[0][_nCol] = _vec.x;
	mat->v[1][_nCol] = _vec.y;
	mat->v[2][_nCol] = _vec.z;
	mat->v[3][_nCol] = _vec.w;
}

// Row-wise
void SetMat(Mat4x4* mat,
	float a00, float a01, float a02, float a03,
	float a10, float a11, float a12, float a13,
	float a20, float a21, float a22, float a23,
	float a30, float a31, float a32, float a33)
{

	Vec4 row0 = { a00, a01, a02, a03 };
	Vec4 row1 = { a10, a11, a12, a13 };
	Vec4 row2 = { a20, a21, a22, a23 };
	Vec4 row3 = { a30, a31, a32, a33 };

	SetMatRow(mat, 0, row0 );
	SetMatRow(mat, 1, row1);
	SetMatRow(mat, 2, row2);
	SetMatRow(mat, 3, row3);

	/*
	SetMatRow(mat, 0, (Vec4) { a00, a01, a02, a03 });
	SetMatRow(mat, 1, (Vec4) { a10, a11, a12, a13 });
	SetMatRow(mat, 2, (Vec4) { a20, a21, a22, a23 });
	SetMatRow(mat, 3, (Vec4) { a30, a31, a32, a33 });
	*/

}

void SetMat(Mat2x4* mat,
	float a00, float a01, float a02, float a03,
	float a10, float a11, float a12, float a13)
{
	Vec4 row0 = { a00, a01, a02, a03 };
	Vec4 row1 = { a10, a11, a12, a13 };

	mat->v[0][0] = a00;
	mat->v[0][1] = a01;
	mat->v[0][2] = a02;
	mat->v[0][3] = a03;

	mat->v[1][0] = a10;
	mat->v[1][1] = a11;
	mat->v[1][2] = a12;
	mat->v[1][3] = a13;
}

void SetEye(Mat4x4* mat, float v)
{
	SetMatRow(mat, 0, { v, 0, 0, 0 });
	SetMatRow(mat, 1, { 0, v, 0, 0 });
	SetMatRow(mat, 2, { 0, 0, v, 0 });
	SetMatRow(mat, 3, { 0, 0, 0, v });
}

void SetEye(Mat2x2* mat, float v)
{
	mat->v[0][0] = v;
	mat->v[0][1] = 0;
	mat->v[1][0] = 0;
	mat->v[1][1] = v;
}

Mat4x4 Zeros()
{
	Mat4x4 m1;
	for(int i=0; i<4; i++)
		for (int j = 0; j < 4; j++)
		{
			m1.v[i][j] = 0.0f;
		}
	
	return m1;
}

void Transpose(Mat4x4* _mat)
{
	Mat4x4 dst;

	SetMat(&dst,
		_mat->v[0][0], _mat->v[1][0], _mat->v[2][0], _mat->v[3][0],
		_mat->v[0][1], _mat->v[1][1], _mat->v[2][1], _mat->v[3][1],
		_mat->v[0][2], _mat->v[1][2], _mat->v[2][2], _mat->v[3][2],
		_mat->v[0][3], _mat->v[1][3], _mat->v[2][3], _mat->v[3][3]);

	_mat = &dst;
}

Mat4x4 TransposeOf(Mat4x4 _mat)
{
	Mat4x4 dst;

	SetMat(&dst,
		_mat.v[0][0], _mat.v[1][0], _mat.v[2][0], _mat.v[3][0],
		_mat.v[0][1], _mat.v[1][1], _mat.v[2][1], _mat.v[3][1],
		_mat.v[0][2], _mat.v[1][2], _mat.v[2][2], _mat.v[3][2],
		_mat.v[0][3], _mat.v[1][3], _mat.v[2][3], _mat.v[3][3]);

	return dst;
}


Mat4x4 AddMat(Mat4x4 lhs, Mat4x4 rhs)
{
	Mat4x4 _mat;

	for (int ii = 0; ii<4; ii++)
	{
		for (int jj = 0; jj<4; jj++)
		{
			_mat.v[ii][jj] = lhs.v[ii][jj] + rhs.v[ii][jj];
			
		}
		printf("\r\n");
	}


	

	return;
}
Mat4x4 MultMat(Mat4x4 lhs, Mat4x4 rhs)
{
	Mat4x4 _mat;

	_mat.v[0][0] = INNER_PRODUCT(lhs, rhs, 0, 0);
	_mat.v[0][1] = INNER_PRODUCT(lhs, rhs, 0, 1);
	_mat.v[0][2] = INNER_PRODUCT(lhs, rhs, 0, 2);
	_mat.v[0][3] = INNER_PRODUCT(lhs, rhs, 0, 3);

	_mat.v[1][0] = INNER_PRODUCT(lhs, rhs, 1, 0);
	_mat.v[1][1] = INNER_PRODUCT(lhs, rhs, 1, 1);
	_mat.v[1][2] = INNER_PRODUCT(lhs, rhs, 1, 2);
	_mat.v[1][3] = INNER_PRODUCT(lhs, rhs, 1, 3);

	_mat.v[2][0] = INNER_PRODUCT(lhs, rhs, 2, 0);
	_mat.v[2][1] = INNER_PRODUCT(lhs, rhs, 2, 1);
	_mat.v[2][2] = INNER_PRODUCT(lhs, rhs, 2, 2);
	_mat.v[2][3] = INNER_PRODUCT(lhs, rhs, 2, 3);

	_mat.v[3][0] = INNER_PRODUCT(lhs, rhs, 3, 0);
	_mat.v[3][1] = INNER_PRODUCT(lhs, rhs, 3, 1);
	_mat.v[3][2] = INNER_PRODUCT(lhs, rhs, 3, 2);
	_mat.v[3][3] = INNER_PRODUCT(lhs, rhs, 3, 3);

	return _mat;
}

Vec4 MultVec(Mat4x4 _mat, Vec4 v)
{
	Vec4 _vec;

	_vec.x = (_mat.v[0][0] * v.x + _mat.v[1][0] * v.y + _mat.v[2][0] * v.z + _mat.v[3][0] * v.w);
	_vec.y = (_mat.v[0][1] * v.x + _mat.v[1][1] * v.y + _mat.v[2][1] * v.z + _mat.v[3][1] * v.w);
	_vec.z = (_mat.v[0][2] * v.x + _mat.v[1][2] * v.y + _mat.v[2][2] * v.z + _mat.v[3][2] * v.w);
	_vec.w = (_mat.v[0][3] * v.x + _mat.v[1][3] * v.y + _mat.v[2][3] * v.z + _mat.v[3][3] * v.w);

	return _vec;
}


Vec4 MultVec(Mat4x4 _mat, float *v)
{
	Vec4 _vec;

	_vec.x = (_mat.v[0][0] * v[0] + _mat.v[1][0] * v[1] + _mat.v[2][0] * v[2] + _mat.v[3][0] * v[3]);
	_vec.y = (_mat.v[0][1] * v[0] + _mat.v[1][1] * v[1] + _mat.v[2][1] * v[2] + _mat.v[3][1] * v[3]);
	_vec.z = (_mat.v[0][2] * v[0] + _mat.v[1][2] * v[1] + _mat.v[2][2] * v[2] + _mat.v[3][2] * v[3]);
	_vec.w = (_mat.v[0][3] * v[0] + _mat.v[1][3] * v[1] + _mat.v[2][3] * v[2] + _mat.v[3][3] * v[3]);

	return _vec;
}


void Desc(Vec4 v1)
{
	printf("%f, %f, %f, %f\n", v1.x, v1.y, v1.z, v1.w);
}



void Desc(Mat4x4 _mat)
{
	for (int ii = 0; ii<4; ii++)
	{
		for (int jj = 0; jj<4; jj++)
		{
			printf("%f\t", _mat.v[ii][jj]);
		}
		printf("\r\n");
	}
}

void Desc(Mat2x4 _mat)
{
	for (int ii = 0; ii<2; ii++)
	{
		for (int jj = 0; jj<4; jj++)
		{
			printf("%f\t", _mat.v[ii][jj]);
		}
		printf("\r\n");
	}
}


void testmat()
{
	Vec4 v1;
	float val[4] = { 0.1,1,2,3 };

	SetVec(&v1, val);
	Desc(v1);

	Mat4x4 m1;
	SetMat(&m1, 1, 0, 0, 0,
		0, 2, 0, 0,
		0, 0, 3, 0,
		0, 0, 0, 4);
	Desc(m1);

	printf("\n");


	Mat m2;

	SetEye(&m2, 100);
	Desc(m2);
}
#endif