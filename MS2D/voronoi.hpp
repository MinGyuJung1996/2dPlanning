#pragma once
#include "MS2D.h"
#include <fstream>


namespace planning
{

#define dbg		// Mark that this part of the code is used for debugging purpose, not a piece consisting final result.
#define _dbg	// similar to above, usually denotes the end of a debugging section.

#define INPUT	// Mark input  of a function in param list
#define OUTPUT  // Mark output of a function in param list
#define LOOP	// Mark loop variables. To Be vigilant updating the loop variables 

// change below to false when release. All Error message will be excluded in compile time.
#define PRINT_ERRORS					false
#define ERR_ARC_END_POINTS				true
#define ERR_FOOT_C_BELOW_1				true
#define ERR_ARC_NORMAL_INTER_DTHETA		true
#define ERR_GET_POINT_GEOM_DTHETA		true
#define ERR_ARC_NORMAL_TO_PARAM			true

	extern int drawBifurCircle;
	extern int drawVoronoiSingleBranch;
	extern int drawMinkowski;
	extern int forwardTime;
	extern int drawBoundary;
	extern int drawTransition;

	extern std::vector<ms::Circle> circlesToDraw;

	extern bool keyboardflag[256];	
	extern double rfbTerminationEps; //originally 1e-18
	extern double rfbTerminationEps2; // origirnally 2.5e-3;

	extern int lineSegCnt;


	using namespace std;
	using namespace ms;

	namespace output_to_file
	{
		extern int m0, m1, cf;
		extern bool flag;
		extern double zoom;
		extern int width, hegiht;

		// int number_of_obstacles; 
		extern vector<vector<int>> objSize;
		extern vector<vector<CircularArc>> obj; //obj[objNo][arcNo];
		extern vector<vector<vector<CircularArc>>> ms_obj; // ms_obj[slice][objNo][arcNo];

		void start();
		void end();
	};

#define OUTPUT_TO_FILE (ms::t0 == planning::output_to_file::m0 && ms::t1 == planning::output_to_file::m1)



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
