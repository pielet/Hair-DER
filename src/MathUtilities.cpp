#include "MathUtilities.h"


//=============================================================================
// _root3, root3 from http://prografix.narod.ru
//=============================================================================
static double _root3(double x)
{
	double s = 1.;
	while (x < 1.)
	{
		x *= 8.;
		s *= 0.5;
	}
	while (x > 8.)
	{
		x *= 0.125;
		s *= 2.;
	}
	double r = 1.5;
	r -= 1. / 3. * (r - x / (r * r));
	r -= 1. / 3. * (r - x / (r * r));
	r -= 1. / 3. * (r - x / (r * r));
	r -= 1. / 3. * (r - x / (r * r));
	r -= 1. / 3. * (r - x / (r * r));
	r -= 1. / 3. * (r - x / (r * r));
	return r * s;
}

double root3(double x)
{
	if (x > 0) return _root3(x); 
	else if (x < 0) return-_root3(-x); 
	else return 0.;
}

//=================================================================================
// solveSquareEquation, solveCubicEquation from 
// http://math.ivanovo.ac.ru/dalgebra/Khashin/poly/index.html
// ================================================================================
// -----------------------------------------------------------------
// x - array of size 2
// return 2: 2 real roots x[0], x[1]
// return 0: pair of complex roots: x[0]±i*x[1]
int solveSquareEquation(Vector2s& x, scalar a, scalar b)
{
	double D = 0.25*a*a - b;
	if (D >= 0) {
		D = sqrt(D);
		x[0] = -0.5*a + D;
		x[1] = -0.5*a - D;
		return 2;
	}
	x[0] = -0.5*a;
	x[1] = sqrt(-D);
	return 0;
}

//------------------------------------------------------------------
// x - array of size 3
// In case 3 scalar roots: => x[0], x[1], x[2], return 3
//         2 scalar roots: x[0], x[1],          return 2
//         1 scalar root : x[0], x[1] ± i*x[2], return 1
int solveCubicEquation(Vector3s& x, scalar a, scalar b, scalar c) // solve cubic equation x ^ 3 + a * x ^ 2 + b * x + c = 0
{
	scalar a2 = a * a;
	scalar q = (a2 - 3 * b) / 9;
	scalar r = (a*(2 * a2 - 9 * b) + 27 * c) / 54;
	// equation x^3 + q*x + r = 0
	scalar r2 = r * r;
	scalar q3 = q * q * q;
	scalar A, B;
	if (r2 <= (q3 + eps)) {//<<-- FIXED!
		scalar t = r / sqrt(q3);
		if (t < -1) t = -1;
		if (t > 1) t = 1;
		t = acos(t);
		a /= 3; q = -2 * sqrt(q);
		x[0] = q * cos(t / 3) - a;
		x[1] = q * cos((t + 2 * PI) / 3) - a;
		x[2] = q * cos((t - 2 * PI) / 3) - a;
		return(3);
	}
	else {
		//A = -pow(fabs(r) + sqrt(r2-q3), 1./3); 
		A = -root3(fabs(r) + sqrt(r2 - q3));
		if (r < 0) A = -A;
		B = (A == 0 ? 0 : B = q / A);

		a /= 3;
		x[0] = (A + B) - a;
		x[1] = -0.5*(A + B) - a;
		x[2] = 0.5*sqrt(3.)*(A - B);
		if (fabs(x[2]) < eps) { x[2] = x[1]; return(2); }
		return(1);
	}
}

//====================================================================================
// lineLineIntersect from http://paulbourke.net/geometry/pointlineplane/
//====================================================================================
//--------------------------------------------------------------
// Calculate the line segment PaPb that is the shortest route between two lines P1P2 and P3P4
// Calculate also the values of mua and mub where
//		Pa = P1 + s(P2 - P1)
//		Pb = P3 + t(P4 - P3)
// Return false if no solution exists.
bool lineLineIntersect(const Vector3s& p1, const Vector3s& p2, const Vector3s& p3, const Vector3s& p4, scalar& s, scalar& t)
{
	Vector3s p13 = p1 - p3;
	Vector3s p43 = p4 - p3;
	if (isSmall(p43.squaredNorm())) return false;
	Vector3s p21 = p2 - p1;
	if (isSmall(p21.squaredNorm())) return false;

	scalar d1343 = p13.dot(p43);
	scalar d4321 = p43.dot(p21);
	scalar d1321 = p13.dot(p21);
	scalar d4343 = p43.squaredNorm();
	scalar d2121 = p21.squaredNorm();

	scalar denom = denom = d2121 * d4343 - d4321 * d4321;
	if (isSmall(denom)) return false;
	scalar numer = d1343 * d4321 - d1321 * d4343;

	s = numer / denom;
	t = (d1343 + d4321 * s) / d4343;

	return true;
}

namespace mathutils
{

	bool approxSymmetric( const MatrixXs& A, const scalar& eps )
	{
	  for( int i = 0; i < A.rows(); ++i ) for( int j = i+1; j < A.cols(); ++j ) if( fabs(A(i,j)-A(j,i)) >= eps ) return false;
	  return true;
	}

}

// explicit instantiations at bottom
template struct int_Vectors<2>;
template struct int_Vectors<3>;
