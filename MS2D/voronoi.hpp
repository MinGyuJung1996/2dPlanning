#pragma once
#include "MS2D.h"


namespace planning
{

#define dbg		// Mark that this part of the code is used for debugging purpose, not a piece consisting final result.
#define _dbg	// similar to above, usually denotes the end of a debugging section.

#define INPUT	// Mark input  of a function in param list
#define OUTPUT  // Mark output of a function in param list
#define LOOP	// Mark loop variables. To Be vigilant updating the loop variables 

	extern int drawBifurCircle;
	extern int drawVoronoiSingleBranch;
	extern int drawMinkowski;
	extern int forwardTime;
	extern int drawBoundary;
	extern int drawTransition;

	extern bool keyboardflag[256];

	using namespace std;
	using namespace ms;


	/* Def : class passed as vornoi part's input
	*/
	struct VR_IN
	{
		vector<CircularArc> arcs;
		vector<int>			left;
		vector<double>		color;
	};

	/* Def : represents a point on a curve.
	*/
	struct pointOnCurve
	{
		int    c; // index of the curve. used as spiral[c].
		double t; // parameter of curve. t ~ [0,1]. 

		bool operator< (const pointOnCurve & r) const;
	};
	using poc = pointOnCurve;

	/* Def : geomoetry info of a pointOnCurve
	*/
	struct pointGeometry
	{
		Point  x; // position
		Point  v; // tangent   : vector length = 1
		Point  n; // normal    : rotate90(tangent)
		double k; // curvature : signed curvature
	};

	/* Def : represents a conic section's segment. (ellipse or hyperbola only)
		Ellipse   : x2/a2 + y2/b2 = 1;
		Hyperbola : x2/a2 - y2/b2 = 1;
	*/
	struct conic
	{
		Point 
			c, // center
			x, // semi-major-axis direction
			y; // semi-minor-axis direction
		double 
			a, // semi-major-axis length
			b, // semi-minor-axis length
			f; // foci
		int
			type; // 0 : ellipse. 1 : hyperbola. 2: line 

		double
			t0, // segment start
			t1;	// segment end

		conic(CircularArc &a, CircularArc &b);
		Point operator()(double t);
		void draw();
	};



	void					convertMsOutput_Clockwise(deque<ArcSpline>& INPUT in, vector<CircularArc>& OUTPUT returned);
	CircularArc				flipArc(CircularArc& in);
	template<class f> bool	isZero(f a);

	void _Convert_MsOut_To_VrIn(vector<deque<ArcSpline>>& INPUT msOut, vector<bool>& INPUT isBoundary, VR_IN& OUTPUT vrIn);
	void _Medial_Axis_Transformation(VR_IN& INPUT in);
	Point getTouchingDiskCenter(Point p, Point q, Point v);

}
