/*****************************************************************************************************
** Compute 3dimensional configuration space of a floor robot(2d translation and rotation)			**
** Primitives of the 3d cspace will be surfaces														**
**																									**
******************************************************************************************************/

#include "xyt-cs.hpp"
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
	inline double& x() {return _x;}
	inline double& y() {return _y;}
	inline double& t() {return _t;}
	
	xyt() = default;
	~xyt() = default;
	inline xyt(real x, real y, real t): _x(x), _y(y), _t(t) {}

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

/*
Def : constructor
	find equation and domain
*/
csSurf::csSurf(CircularArc& robot, CircularArc& obs, Point& robotCenter)
{
	// 0. set arcR, arcO, _flagEquationFound
	{
		arcR = &robot;
		arcO = &obs;
		this->robotCenter = robotCenter;
	}
	_flagEquationFound = true;

	// 1. set primitive var;
	real
		x0, y0, r0, b0, e0,
		x1, y1, r1, b1, e1,
		cx, cy;
	{
		auto& temp = robot;
		x0 = temp.cx();
		y0 = temp.cy();
		r0 = temp.cr();
		if (temp.lccw())
		{
			b0 = temp.atan0();
			e0 = temp.atan1();
		}
		else
		{
			b0 = temp.atan1();
			e0 = temp.atan0();
		}
		while (e0 < b0)
			e0 += PI2;
	}
	{
		auto& temp = obs;
		x1 = temp.cx();
		y1 = temp.cy();
		r1 = temp.cr();
		if (temp.lccw())
		{
			b1 = temp.atan0();
			e1 = temp.atan1();
		}
		else
		{
			b1 = temp.atan1();
			e1 = temp.atan0();
		}
		while (e1 < b1)
			e1 += PI2;
	}
	{
		cx = robotCenter.x();
		cy = robotCenter.y();
	}

	/*
	Notice that : 
	-pi < b0, b1 < pi
	-pi < e0, e1 < 2pi
	*/
	// 2. set saved var
	_phi0 = b1;
	_phi1 = e1;
	_tmp0 = -e0;
	_tmp1 = -b0;
	_center0 = Point(cx + x1, cy + y1);
	_center1 = Point(x0 - cx, y0 - cy);
	_rad = r0 + r1;
}

/*
Def : from the equation/domain found, get points on the suraface
Notice that :
	-pi < phi < 2pi
	-2pi < tmp < pi
	-3pi < theta < 3pi
*/
void csSurf::tessellation(int nPhi, int nTheta)
{
	// 0. check error
	if (_flagEquationFound == false)
	{
		cerr << "!!!!!!!!ERROR : _flagEquationFound is false" << endl;
	}
	else
		_flagTrianglesFound = true;

	// 1. resize array;
	int &I = nTheta,
		&J = nPhi;
	_tess.resize(J+1);
	for (auto& temp : _tess)
		temp.resize(I+1);

	// 2. evaluate point and save it
	for (size_t j = 0; j <= J; j++)
	{
		// 2-1. get tess cord (i,j) -> (theta, phi)
		real phi =
			(_phi0 * (J - j) + _phi1 * (j)) / J;
		for (size_t i = 0; i <= I; i++)
		{
			real tmp =
				(_tmp0 * (I - i) + _tmp1 * (i)) / I;
			real theta =
				tmp + phi;

			// 2-2. get point coord xyt.
			auto xy = x(theta, phi);
			_tess[j][i] = xyt(xy.x(), xy.y(), theta);
		}

	}

}



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
		setRobot (vector<CircularArc>& robot) { this->_ptrRob = &robot; };
	inline void 
		setObstacle (vector<CircularArc>& obstacles) { this->_ptrObs = &obstacles; };
	inline void 
		setOutput(vector<csSurf>& output) { this->_ptrOut = &output; };

	bool 
		isInputProper();
	void 
		calculate();

private:
	vector<CircularArc>* _ptrRob;
	vector<CircularArc>* _ptrObs;
	vector<csSurf>*		 _ptrOut;
};

/*
Def : constructor
*/
configSpaceCalculator::configSpaceCalculator()
{
	_ptrRob = nullptr;
	_ptrObs = nullptr;
	_ptrOut = nullptr;
}


/*
Def : Check whether we are ready for calling "calculate()"

Not done yet.
*/
bool configSpaceCalculator::isInputProper()
{
	// 1. check whether I/O is registered.
	if (_ptrRob == nullptr ||
		_ptrObs == nullptr ||
		_ptrOut == nullptr)
	{
		cerr << "configSpaceCalculator : "
			<< "ptr not set." << endl;
		return false;
	}

	// 2. check whether all arcs are <180;


	// 99. all test passed.
	return true;
}

/*
Def :
*/

void configSpaceCalculator::calculate()
{
	// 0. init stuff
	auto startTime = chrono::high_resolution_clock().now();
	auto& out = *_ptrOut;
	auto& rob = *_ptrRob;
	auto& obs = *_ptrObs;
	auto origin = Point(0, 0);

	// 1.
	for(auto& r : rob)
		for (auto& o : obs)
		{
			out.emplace_back(r, o, origin);
		}

	// 99.
	auto endTime = chrono::high_resolution_clock().now();
	auto duration = (endTime - startTime);
	cout << "csCalc Time : " << duration.count() << endl; //std::chrono::duration_cast<std::milli>(duration)
}


/*****************************************************************************************************
**																									**
**												LSS													**
**																									**
******************************************************************************************************/