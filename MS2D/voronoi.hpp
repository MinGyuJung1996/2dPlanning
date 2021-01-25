#pragma once
#include "MS2D.h"
#include <fstream>
#include "stb_image_write.h"


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
	extern bool keyboardflag_last[256];
	extern double rfbTerminationEps; //originally 1e-18
	extern double rfbTerminationEps2; // origirnally 2.5e-3;

	extern int lineSegCnt;
	
	extern std::vector<ms::CircularArc> voronoiBoundary;

	using namespace std;
	using namespace ms;

	struct VR_IN;

	namespace output_to_file
	{
#define NUMBER_OF_SLICES 360
		extern int m0, m1, cf;
		extern bool flag;
		extern double zoom;
		extern int width, hegiht;

		struct v_edge
		{
			Point v0, v1;	// endpoints for this edge.
			int idx[2];		// two circular arc indexes which makes this v_edge
		};

		struct bifur_point
		{
			Point p; // the bifurcation point
			vector<int> idx; // circular arc's indices which makes this bifur pt
		};

		extern vector<vector<int>> objSize;		// objSize[Model_approx's number][obj number] //initialized in ms::init;
		extern vector<CircularArc> boundary;
		extern vector<CircularArc> robot;			// robot's c-arc		// init in start()
		extern vector<vector<CircularArc>> obj;	// obj[objNo][arcNo];	// init in start()
		extern vector<vector<vector<CircularArc>>> ms_obj; // ms_obj[slice][objNo][arcNo];		// built at display call back
		extern vector<vector<v_edge>> v_edges; // v_edges[slice][i] = line seg		// built at rfb
		extern vector<vector<bifur_point>> bifur_points;			// b_pts[slice][i]					// built at rfb
		extern vector<VR_IN> vrIn;					// vrIn[slice]	// built in display callback // TODO redundant with ms_obj;


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
		vector<int>			arcsPerLoop;

		inline bool isBoundaryArc(int i)
		{
			if (color[i] == 1.0)
				return true;
			else
				return false;
		}
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


#define coneVoronoiDepthBehindSign -
	//just in case. object behind has - sign, while front has +

	class coneVoronoi
	{
	public:
		const double PI = 3.14159265358979323846264;
		const double PI2 = 2 * 3.14159265358979323846264;
		const double PI_half = 0.5 * 3.14159265358979323846264;

		int colorType = 0;
		double dtheta = 0.05;
		double thetaOffset = 0.001;
		double coneRad = 100.0;

		/*
		Assume : 
			Right region (from tangent's view) is inside object, while left = free, clear space
			arc exists in only one quadrant
		*/
		void drawArcToCone(INPUT CircularArc& c, INPUT void* colors);

		void coneVoronoi::drawCone(INPUT Point p, INPUT double theta0, INPUT double theta1, INPUT void* color);

		void drawVoronoi(INPUT VR_IN & v);

	};


	double getClosestArcParameter(Point& p, CircularArc &c);

	void					convertMsOutput_Clockwise(deque<ArcSpline>& INPUT in, vector<CircularArc>& OUTPUT returned);
	CircularArc				flipArc(CircularArc& in);
	template<class f> bool	isZero(f a);


	void _Convert_VectorCircularArc_G0(INPUT vector<CircularArc>& input, OUTPUT vector<CircularArc>& output, INPUT int globalCCW_Option = -1);
	void _Convert_VectorCircularArc_G1(INPUT vector<CircularArc>& input, OUTPUT vector<CircularArc>& output);
	void _Convert_VectorCircularArc_G1_new(INPUT vector<CircularArc>& input, OUTPUT vector<CircularArc>& output);
	void _Convert_VectorCircularArc_To_MsInput(INPUT vector<CircularArc>& input, OUTPUT vector<ArcSpline>& output, INPUT double rotationDegree = 0);
	void _Convert_MsOut_To_VrIn(vector<deque<ArcSpline>>& INPUT msOut, vector<bool>& INPUT isBoundary, VR_IN& OUTPUT vrIn);
	void _Medial_Axis_Transformation(VR_IN& INPUT in);
	Point getTouchingDiskCenter(Point p, Point q, Point v);

	bool isNormalBetween(Point n0, Point n1, Point n);

}
