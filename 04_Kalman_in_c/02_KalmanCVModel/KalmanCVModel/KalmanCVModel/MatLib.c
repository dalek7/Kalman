#include "MatLib.h"

void SetMat(Mat4x4* mat,
		  float a00, float a01, float a02, float a03,
		  float a10, float a11, float a12, float a13,
		  float a20, float a21, float a22, float a23,
		  float a30, float a31, float a32, float a33)
{
	SetMatRow(mat, 0, (Vec4){a00, a01, a02, a03});
	SetMatRow(mat, 1, (Vec4){a10, a11, a12, a13});
	SetMatRow(mat, 2, (Vec4){a20, a21, a22, a23});
	SetMatRow(mat, 3, (Vec4){a30, a31, a32, a33});
}

void SetVec(Vec4* vec, float *v)
{
	vec->x = v[0];
	vec->y = v[1];
	vec->z = v[2];
	vec->w = v[3];
}

void SetMatRow(Mat4x4* mat, int _nRow, Vec4 _vec)
{
	mat->Mat[_nRow][0] = _vec.x;
	mat->Mat[_nRow][1] = _vec.y;
	mat->Mat[_nRow][2] = _vec.z;
	mat->Mat[_nRow][3] = _vec.w;
}

void SetMatCol(Mat4x4* mat, int _nCol, Vec4 _vec)
{
	mat->Mat[0][_nCol] = _vec.x;
	mat->Mat[1][_nCol] = _vec.y;
	mat->Mat[2][_nCol] = _vec.z;
	mat->Mat[3][_nCol] = _vec.w;
}

void Transpose(Mat4x4* _mat)
{
	Mat4x4 dst;
	
	SetMat(&dst,
		  _mat->Mat[0][0], _mat->Mat[1][0], _mat->Mat[2][0], _mat->Mat[3][0], 
		  _mat->Mat[0][1], _mat->Mat[1][1], _mat->Mat[2][1], _mat->Mat[3][1], 
		  _mat->Mat[0][2], _mat->Mat[1][2], _mat->Mat[2][2], _mat->Mat[3][2], 
		  _mat->Mat[0][3], _mat->Mat[1][3], _mat->Mat[2][3], _mat->Mat[3][3]);
	
	_mat = &dst;
}

Mat4x4 TransposeOf(Mat4x4 _mat)
{
	Mat4x4 dst;
	
	SetMat(&dst,
		  _mat.Mat[0][0], _mat.Mat[1][0], _mat.Mat[2][0], _mat.Mat[3][0], 
		  _mat.Mat[0][1], _mat.Mat[1][1], _mat.Mat[2][1], _mat.Mat[3][1], 
		  _mat.Mat[0][2], _mat.Mat[1][2], _mat.Mat[2][2], _mat.Mat[3][2], 
		  _mat.Mat[0][3], _mat.Mat[1][3], _mat.Mat[2][3], _mat.Mat[3][3]);
	
	return dst;
}

Mat4x4 MultMat(Mat4x4 lhs, Mat4x4 rhs)
{
	Mat4x4 _mat;
	
	_mat.Mat[0][0] = INNER_PRODUCT(lhs, rhs, 0, 0);
	_mat.Mat[0][1] = INNER_PRODUCT(lhs, rhs, 0, 1);
	_mat.Mat[0][2] = INNER_PRODUCT(lhs, rhs, 0, 2);
	_mat.Mat[0][3] = INNER_PRODUCT(lhs, rhs, 0, 3);
	
	_mat.Mat[1][0] = INNER_PRODUCT(lhs, rhs, 1, 0);
	_mat.Mat[1][1] = INNER_PRODUCT(lhs, rhs, 1, 1);
	_mat.Mat[1][2] = INNER_PRODUCT(lhs, rhs, 1, 2);
	_mat.Mat[1][3] = INNER_PRODUCT(lhs, rhs, 1, 3);
	
	_mat.Mat[2][0] = INNER_PRODUCT(lhs, rhs, 2, 0);
	_mat.Mat[2][1] = INNER_PRODUCT(lhs, rhs, 2, 1);
	_mat.Mat[2][2] = INNER_PRODUCT(lhs, rhs, 2, 2);
	_mat.Mat[2][3] = INNER_PRODUCT(lhs, rhs, 2, 3);
	
	_mat.Mat[3][0] = INNER_PRODUCT(lhs, rhs, 3, 0);
	_mat.Mat[3][1] = INNER_PRODUCT(lhs, rhs, 3, 1);
	_mat.Mat[3][2] = INNER_PRODUCT(lhs, rhs, 3, 2);
	_mat.Mat[3][3] = INNER_PRODUCT(lhs, rhs, 3, 3);
	
	return _mat;
}

Vec4 MultVec(Mat4x4 _mat, Vec4 v)
{
	Vec4 _vec;
	
	_vec.x = (_mat.Mat[0][0]*v.x + _mat.Mat[1][0]*v.y + _mat.Mat[2][0]*v.z + _mat.Mat[3][0]*v.w);
	_vec.y = (_mat.Mat[0][1]*v.x + _mat.Mat[1][1]*v.y + _mat.Mat[2][1]*v.z + _mat.Mat[3][1]*v.w);
	_vec.z = (_mat.Mat[0][2]*v.x + _mat.Mat[1][2]*v.y + _mat.Mat[2][2]*v.z + _mat.Mat[3][2]*v.w);
	_vec.w = (_mat.Mat[0][3]*v.x + _mat.Mat[1][3]*v.y + _mat.Mat[2][3]*v.z + _mat.Mat[3][3]*v.w);
		
	return _vec;
}