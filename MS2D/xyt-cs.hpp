#pragma once
/*****************************************************************************************************
** Compute 3dimensional configuration space of a floor robot(2d translation and rotation)			**
** Primitives of the 3d cspace will be surfaces														**
**																									**
******************************************************************************************************/

#include "MS2D.h"
using namespace std;

using CircularArc = ms::CircularArc;
/*
Arc ADT used:
	cx	cy	cr
	atan0	atan1
*/

using Point = ms::Point;
/*
Point ADT used:
	x()
	y()
*/

using real = double; // just in case there is a need for a higher-precision-fp-lib


/*
Def : a point in (R^2 X S^1) space

Assume :
	t loops at 2n*pi
*/
class xyt
{
public:
	// data access
	inline double& x() { return _x; }
	inline double& y() { return _y; }
	inline double& t() { return _t; }
	inline double* v3d() { return &_x; }

	// constructor/desturctor
	xyt() = default;
	~xyt() = default;
	inline xyt(real x, real y, real t) : _x(x), _y(y), _t(t) {}
	
	// operator overloading
	inline xyt operator+(xyt& rhs)
	{
		return { this->x() + rhs.x(), this->y() + rhs.y(), this->t() + rhs.t() };
	}
	inline xyt operator-(xyt& rhs)
	{
		return { this->x() - rhs.x(), this->y() - rhs.y(), this->t() - rhs.t() };
	}
	inline xyt operator*(double rhs)
	{
		return { this->x() * rhs, this->y() * rhs, this->t() * rhs };
	}
	inline xyt operator/(double rhs)
	{
		double m = 1 / rhs;
		return { this->x() * m, this->y() * m, this->t() * m };
	}
	friend inline xyt operator*(double lhs, xyt& rhs);

	// math
	inline xyt cross(xyt rhs)
	{
		double X = y()* rhs.t() - t() * rhs.y();
		double Y = t()* rhs.x() - x() * rhs.t();
		double T = x()* rhs.y() - y() * rhs.x();

		return xyt(X, Y, T);
	}
	inline xyt normalize()
	{
		double l = sqrt(_x * _x + _y * _y + _t * _t);
		return xyt(_x / l, _y / l, _t / l);
	}


private:
	// do not change ordering of variables.
	double _x;
	double _y;
	double _t;
};
inline xyt operator*(double lhs, xyt& rhs)
{
	return rhs * lhs;
}

/*
Def : contains three vertex and its normals
Optimized for drawing, rather building step.
*/
class triNormal
{
public:
	inline double* x(int idx)
	{
		switch (idx)
		{
		case 0:
			return _data;
		case 1:
			return _data + 6;
		case 2:
		default:
			return _data + 12;
		}
	}
	inline double* n(int idx)
	{
		switch (idx)
		{
		case 0:
			return _data + 3;
		case 1:
			return _data + 9;
		case 2:
		default:
			return _data + 15;
		}
	}
	inline double* x0() {return _data + 0;}
	inline double* x1() {return _data + 6;}
	inline double* x2() {return _data + 12;}
	inline double* n0() {return _data + 3;}
	inline double* n1() {return _data + 9;}
	inline double* n2() {return _data + 15;}

	triNormal() = default;
	inline triNormal(xyt& x0, xyt& n0, xyt& x1, xyt& n1, xyt& x2, xyt& n2)
	{
		_data[ 0] = x0.x();
		_data[ 1] = x0.y();
		_data[ 2] = x0.t();
		_data[ 3] = n0.x();
		_data[ 4] = n0.y();
		_data[ 5] = n0.t();
		_data[ 6] = x1.x();
		_data[ 7] = x1.y();
		_data[ 8] = x1.t();
		_data[ 9] = n1.x();
		_data[10] = n1.y();
		_data[11] = n1.t();
		_data[12] = x2.x();
		_data[13] = x2.y();
		_data[14] = x2.t();
		_data[15] = n2.x();
		_data[16] = n2.y();
		_data[17] = n2.t();
	}

private:
	double _data[18];
};

using triangulationXYT = vector<vector<xyt>>;

/*****************************************************************************************************
**																									**
**											csSurface												**
**																									**
******************************************************************************************************/

/*
Def :
	Describes a surface in xyt space, which is formed from two convex arcs(angle < 180).

Desc :
	Not actually a cylinder.. but similar some how...
	What would stacking convolution of two arcs be like(with first arc being rotatable)???
		1. They would be a subset of a cylinder R(:=r1+r2), but as theta changes, the center of the cylinder keeps changing.
		2. It is a parallelogram in the parameter space. (theta-phi is the param space)
			Theta is the rotation
			This is because... hard to explain...
			Notice that the topology of param-space is actually a T2... since theta and phi both loops
Assume:
	Input CircularArc's angle < 180 degree;
		If > 180, just cut it half
How to use:
	1. constructor
	2. tessellate (the theta will probably be out of range [0,2pi]
	3. surface

*/
class csSurf
{
public:

	/* Inline Alias */
	inline Point&
		c0() { return _center0; }
	inline Point&
		c1() { return _center1; }
	inline double
		r() { return _rad; }
	inline Point&
		c(double thetaRad) { return _center0 + (_center1.rotate(thetaRad)); }
	inline Point&
		x(double thetaRad, double phiRad) { return c(thetaRad) + r() * Point(cos(phiRad), sin(phiRad)); }
	inline triangulationXYT&
		surface() { if (!_flagTrianglesFound) cerr << "!!!!!!!!ERROR: _flagTrianglesFound" << endl; return _tess; }
	inline triangulationXYT&
		normals() { if (!_flagTrianglesFound) cerr << "!!!!!!!!ERROR: _flagTrianglesFound" << endl; return _normals; }


	/* Constructor/Destructor */
	csSurf() = default;
	csSurf(CircularArc& robot, CircularArc& obs, Point& robotCenter);
	~csSurf() = default;

	/* functions to get some geometry */
	void 
		tessellation(int nPhi = 10, int nTheta = 10);
	vector<triNormal>* 
		drawingSet();
	CircularArc 
		slice(real theta);

private:
	/* FLAGS */
	bool
		_flagEquationFound = false,
		_flagTrianglesFound = false;

	/* Var for equation of the surface */
	Point	_center0;
	Point	_center1;	// circle center at theta : _center0 + _center1.rotate(theta)
	real	_rad;		// rad = r1+r2 (for now, convex only)

	/* Var for domain of the surface */
	// domain : {phi0 < \phi < phi1 } \and {tmp0 < \theta - \phi < tmp1}
	//		=> a band with grad = 1 in phi-theta space;
	real
		_phi0,
		_phi1,
		_tmp0,
		_tmp1;

	/* Var to store Triangles. (needed for */
	triangulationXYT _tess;
	triangulationXYT _normals;

	/* Construction Info */
	CircularArc
		* arcR,
		* arcO;
	Point
		robotCenter;
private:
	//void setEquation

};


/*****************************************************************************************************
**																									**
**										configSpaceCalculator										**
**																									**
******************************************************************************************************/

/*
Def :
	Compute Configuration space obstacles
	Stack of Mink-sum of two objects(one which is a rotatable robot, the other is just the obstacles)
Assume :
	Input Arcs are convex arcs, and the loops itself are convex.
	The 3d-CS-Objects do not intersect each other.
How to use :
	1. setRobot
	2. setObs
	3. setOut
	(4. isInputProper)
	5. calculate
Description :

+:
	3 ptr are not allocated by this class. No need to free mem.


*/
class configSpaceCalculator
{
public:
		configSpaceCalculator();
	virtual 
		~configSpaceCalculator() = default;

	inline void
		setRobot(vector<CircularArc>& robot) { this->_ptrRob = &robot; };
	inline void
		setObstacle(vector<CircularArc>& obstacles) { this->_ptrObs = &obstacles; };
	inline void
		setOutput(vector<csSurf>& output) { this->_ptrOut = &output; };

	bool
		isInputProper();
	void
		calculate();

private:
	vector<CircularArc>* _ptrRob;
	vector<CircularArc>* _ptrObs;
	vector<csSurf>* _ptrOut;
};


/*****************************************************************************************************
**																									**
**												LSS													**
**																									**
******************************************************************************************************/


/*****************************************************************************************************
**																									**
**										 Voronoi on CSpace											**
**																									**
******************************************************************************************************/

/*



*/