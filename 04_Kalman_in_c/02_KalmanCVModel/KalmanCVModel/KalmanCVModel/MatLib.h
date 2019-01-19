#ifndef __SNOW_MATH__
#define __SNOW_MATH__

#define INNER_PRODUCT(a,b,r,c) \
 ((a).Mat[r][0] * (b).Mat[0][c]) \
+((a).Mat[r][1] * (b).Mat[1][c]) \
+((a).Mat[r][2] * (b).Mat[2][c]) \
+((a).Mat[r][3] * (b).Mat[3][c])

typedef struct _vec4
{
	float x;
	float y;
	float z;
	float w;
} Vec4;


typedef struct _mat4x4
{
	float Mat[4][4];
} Mat4x4;


void SetMat(Mat4x4* mat,
		  float a00, float a01, float a02, float a03,
		  float a10, float a11, float a12, float a13,
		  float a20, float a21, float a22, float a23,
		  float a30, float a31, float a32, float a33);
			  
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



#endif