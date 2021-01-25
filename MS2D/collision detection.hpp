#include "voronoi.hpp"

namespace cd
{
	/*
	This header is for:
		To check collision of (Robot, Obstacle) when moving between slices
	Contains:
	
	*/
	/*
	Situation : a robot is able to rotate and translate
	What we want to know : is there collision with other stuffs?
	Assume : 
	1. everything is from circular arc
	2. The change in configuration of the robot is not too dramatic, letting us only test the boundary (This assumption may not be needed)

	How (new):

	How (old):
	1. compute superset(CircularArc) of the swept-volume-boundary of the robot
		1.1 It is a superset, in that it may contain some circular arcs that does not form the swept-volume-boundary.
		1.1.1. But all those arcs should be inside the swept-volume-boundary
	2. test the superset against the obstacles

	Optimize :
		cutoff useless stuffs in superset with cross(point.normal, point.position)
		precomputing stuff with mink(obj, superset)
	
	*/

	/****************************************************************************************
	**	0. Some Aliases
	****************************************************************************************/

	using namespace std;

	using CircularArc = ms::CircularArc;
	using Circle = ms::Circle;
	using Point = ms::Point;

	using Robot = vector<CircularArc>;
	using Obstacle = vector<CircularArc>;
	using Environment = vector<Obstacle>;

	using lineSegment = std::pair<Point, Point>;
	using superSet = std::pair<vector<CircularArc>, vector<lineSegment>>;

	extern decltype(planning::isNormalBetween)* isNormalBetween;

	/**********************************************************************************
	**  0-1. helper func(transform primitives )
	***********************************************************************************/

	// All of them generate new instances.

	CircularArc
		scaleArc(CircularArc& arc, double scale);
	CircularArc	
		rotateArc(CircularArc& arc, double degree);
	Point
		rotatePoint(Point& p, double rotationDegree);
	Point		
		rotatePoint(Point& p, double cosine, double sine);
	CircularArc 
		translateArc(CircularArc& arc, Point& translation);

	/**********************************************************************************
	**  0-2. helper func(math)
	***********************************************************************************/
	pair<double, double>
		getParameterOfArc(CircularArc& arc);
	double
		getParameterOfArc(CircularArc& arc, Point& normal);
	double
		getParameterOfArc(CircularArc& arc, Point& normal, double theta0);

	bool
		less(double a, double b); // used as function pointer
	bool
		greater(double a, double b); // used as function pointer

	inline double
		cross(Point& a, Point& b) { return a^b; } // det| a b |
	
	/**********************************************************************************
	**  0-3. helper func(abstraction)
	***********************************************************************************/

	CircularArc 
		constructArc(Point& center, double rad, double radian0, double radian1);
	CircularArc 
		constructArc(CircularArc& arc, double radian0, double radian1);
	CircularArc
		constructArc(Point& center, Point& x0, Point& x1, bool ccw);
	CircularArc 
		constructArc(Point& x0, Point& x1, Point& t0);
	CircularArc
		constructArc(Point& x0, Point& x1, double r, bool ccw, bool chooseSmallerArc = true);

	/**********************************************************************************
	**  1. Main func
	***********************************************************************************/

	void 
		cdRobotObstacle();		//TODO
	void 
		cdRobotEnvironment();	//TODO

	/**********************************************************************************
	**  2. Building Superset(or trimmed superset)
	***********************************************************************************/

	std::pair<vector<CircularArc>, vector<lineSegment>> 
		getSuperSet(vector<CircularArc> robot, double degree, Point translation);

	vector<CircularArc> 
		getRotationSuperSet(Robot& robot, double degree);

	vector<CircularArc>
		getRotationBoundary(Robot& robot, double degree, int grb_debug = 0);
	vector<deque<CircularArc>>
		trimRotationSupersetIntoLoops(vector<CircularArc>& superset, vector<CircularArc>& original, double rotationRadian, int grb_debug = 0);

	
	std::pair<vector<CircularArc>, vector<lineSegment>> 
		getTranslationSuperSet(vector<CircularArc> arcs, Point translation);


	/***********************************************************************************
	** 3. Testing (intersection)
	***********************************************************************************/

	bool 
		testSuperset(superSet& ss, Obstacle& obs);				//TODO
	bool 
		test_LineSegment_CircularArc(lineSegment& lhs, CircularArc& rhs);
	bool 
		test_CircularArc_CircularArc(CircularArc& lhs, CircularArc& rhs);	//TODO
	bool
		test_LineElement_LineSegment(Point& x, Point& p, Point& q);
	bool
		test_CircleElement_CircularArc(Point& x, CircularArc& a);

	extern int trsiInterestIdx;

	/***********************************************************************************
	** 4. For refinement of vornoi edges
	***********************************************************************************/

	class aabb
	{
	public:
		inline double& xmin() { return bound[0]; }
		inline double& xmax() { return bound[1]; }
		inline double& ymin() { return bound[2]; }
		inline double& ymax() { return bound[3]; }

		inline bool 
			isLeafNode() { return child.size() == 0; }
		inline int& 
			data() { return idx; }
		bool 
			testLineSegment(Point& p, Point& q);

	public:
		vector<aabb> 
			child;

	private:
		double 
			bound[4];
		int 
			idx = -1; //leaf node : contains idx of primitive(arc)
	};

	
	class lineSegmentCollisionTester
	{
		// alias
		using ve		= planning::output_to_file::v_edge;
		using VR_IN		= planning::VR_IN;
		using container = vector<ve>; // just in case of type change. (uses it-1, so need random-access-iter?)
		using iter		= container::iterator;

	public:

		bool
			optionCheckCompleteInclusionOfLineSegment = true;

	public:
		// construct/destructor := does nothing. (call initialize instead) 
		lineSegmentCollisionTester() = default;
		~lineSegmentCollisionTester() = default;

		void 
			initialize(VR_IN& vrin);
		bool 
			test(iter& it0, iter& it1);



	private:
		vector<CircularArc> data;	//contains all arcs;
		vector<CircularArc> voronoiBoundary; // Meaningful? Since Voronoi boundary is for computational performance, They really don't exist in the scene.
		aabb rootObstacleBVH;

	private:
		void 
			recursivelyBuildBVH(aabb& currentNode, vector<int>& arcIndices);
		bool 
			recursivelyTestBVH_lineSegment(aabb& currentNode, lineSegment& ls);
	};
};

