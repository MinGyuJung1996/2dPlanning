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
	inline double& x() { return _x; }
	inline double& y() { return _y; }
	inline double& t() { return _t; }

	xyt() = default;
	~xyt() = default;
	inline xyt(real x, real y, real t) : _x(x), _y(y), _t(t) {}

private:
	double _x;
	double _y;
	double _t;
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


	/* Constructor/Destructor */
	csSurf() = default;
	csSurf(CircularArc& robot, CircularArc& obs, Point& robotCenter);
	~csSurf() = default;

	/* functions to get some geometry */
	void tessellation(int nPhi = 10, int nTheta = 10);
	CircularArc slice(real theta);

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
	virtual ~configSpaceCalculator() = default;

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