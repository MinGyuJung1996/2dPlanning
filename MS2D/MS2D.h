#pragma once
#include <cfloat>
#include <functional>
#include <tuple>
#include <array>
#include <vector>
#include <set>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <time.h>
#include <chrono>
//#include "glad/glad.h" //due to legacy stuff being unusable, (glvertex...), this part was commented.
#include "GL/glut.h"

static double dbgBlock[100];

static const double PI		=		3.14159265358979323846264;
static const double PI2		= 2 *	3.14159265358979323846264;
static const double PI_half	= 0.5 * 3.14159265358979323846264;

namespace ms
{
	class CircularArc;
	class Circle;
	class Point;
}

void
	readArcModel(const char* arcIn, const char* circIn, std::vector<ms::CircularArc>& arcOut, std::vector<ms::Circle>& circOut);
void 
	readArcModelAndProcessIt(const char* arcIn, const char* circIn, std::vector<ms::CircularArc>& arcOut, std::vector<ms::Circle>& circOut);
void
	appendArcModel(std::vector<ms::CircularArc>& sceneOriginal, std::vector<ms::Circle>& sceneCircles, std::vector<ms::CircularArc>& arcs, std::vector<ms::Circle>& circs, double scale, double rotationDegree, ms::Point translation);
void 
	initializeRobotObstacles(int RobotIdx = 0, int ObstaclesIdx = 0);

#define grid_half_size 3.25

namespace ms {
	extern int
		&t0,
		&t1,
		&t2;
	extern int dbgcnt;
	extern std::vector<CircularArc> Model_vca[8];
	extern bool Model_from_arc[8];
	extern Point clickedPoint;

	// Error Bound
	static const double EPSILON = 1e-5;

	// Numerical error bound
	static const double N_PRESCISION = 1e-8;
	static const double N_HIGH_PRESCISION = 1e-8;

	//// PI
	//static const double PI		=		3.14159265358979323846264;
	//static const double PI2		= 2 *	3.14159265358979323846264;
	//static const double PI_half	= 0.5 * 3.14159265358979323846264;

	// Draw Resolution (For Circular Arc)
	// RES : The number of Points used to draw single Circular Arc
	extern int RES;

	// Draw Resolution (For Circle)
	static const int CRES = 100;

	// The Number of Frame for each Model
	static const int numofframe = 360;

	/* For Cache Data Structure */
	// cacheSize : The number of Cache
	static const int cacheSize = 8;

	/* For Grid Data Structure */
	// The number of grid along x / y axis
	static const int grid = 16;
	// Scope of Grid (Min / Max)
	static const double grid_min_x = -grid_half_size;
	static const double grid_max_x = +grid_half_size;
	static const double grid_min_y = -grid_half_size;
	static const double grid_max_y = +grid_half_size;

	// File Descriptor to Export data
	extern FILE *f;

	/* Classes and Structures */
	class Point;
	class Line;
	struct Geometry;
	class Circle;
	class CircularArc;
	enum CtrlPtType;
	class BezierCrv;
	enum SegmentTypes;
	class Segment;
	struct Geometry;
	class ArcSpline;
	class ConvolutionCurve;
	class BCA;
	class dividePts;
	class CacheCircles;
	class Grid;

	/* Type Definition */
	typedef Point Vector;

	/* External Variables from main.c */
	// Data About Current Model
	extern int ModelInfo_CurrentFrame;
	extern std::pair<int, int> ModelInfo_CurrentModel;
	extern bool ModelInfo_Identical;

	// Imported Data
	extern std::vector<BezierCrv> Models_Imported[8];
	extern std::vector<Circle> InteriorDisks_Imported[8];

	// Interior Disks
	extern std::vector<Circle> InteriorDisks_Rotated[numofframe];
	extern std::vector<Circle> InteriorDisks_Convolution;

	// Model Data (Approximated)
	extern std::vector<BezierCrv> Models_Rotated[numofframe];
	extern std::vector<BezierCrv> Models[8];
	extern std::vector<ArcSpline> Models_Rotated_Approx[numofframe];
	extern std::vector<ArcSpline> Models_Approx[8];

	// Data Structures to Accelerate Trimming Process
	extern Grid Grid_Trimming;
	extern CacheCircles Cache_Trimming;

	/* Final Result */
	// Model_Result : each loop is expressed as deque of Arcs
	extern std::vector<std::deque<ArcSpline>> Model_Result;
	// ModelInfo : Determine whether the model is inner loop or Boundary (outer loop)
	// if Model_Result[i] is Boundary, ModelInfo_Boundary is true
	extern std::vector<bool> ModelInfo_Boundary;


	/* global function */
	void initialize();
	void postProcess(int i, int j);
	void save();
	void minkowskisum(int frame, int figure2);
	void minkowskisum_id(int frame, int figure2);


	/*!
	*	\class Point
	*	\brief 2차원 위치를 표현하는 클래스
	*
	*	\author 한상준(manolike@snu.ac.kr)
	*	\date 1 Aug., 2017
	*/
	class Point
	{
		// 프렌드
		friend bool counterclockwise(Point &p, Point &q);
		friend double distance(Point &p, Point &q);
		friend double operator *(Point &lhs, Point &rhs);
		friend double operator ^(Point &lhs, Point &rhs);
		friend Point operator *(double s, Point &rhs);
		friend Point operator *(Point &lhs, double s);
		friend Point operator /(Point &lhs, double s);
		friend Point operator +(const Point &lhs, const Point &rhs);
		friend Point operator -(const Point &lhs, const Point &rhs);
		friend std::ostream &operator <<(std::ostream &os, const Point &p);

	public:
		// 생성자 및 소멸자
		Point(double x = 0.0, double y = 0.0);
		Point(const Point &cpy);
		Point(Line &l, Line &m);
		Point(CircularArc &c);
		virtual ~Point();

		// 연산자
		Point operator -() const;
		Point &operator =(const Point &rhs);
		double &operator [](int idx) { return P[idx]; };
		bool operator ==(Point &rhs);
		bool operator <(Point &rhs);
		bool operator >(Point &rhs);

		// 멤버 함수
		bool close(Point &rhs);
		bool exact(Point &rhs);
		double length();
		Point &normalize();
		Point rotate();
		Point rotate(double angle);
	public:
		/*! \brief 2차원 포인트의 위치를 저장하는 실수 배열 */
		double P[2];
		//double& x = P[0];
		//double& y = P[1];

		// stuff I added are below
		inline double& x() { return P[0]; }
		inline double& y() { return P[1]; }
		inline double length2() { return P[0] * P[0] + P[1] * P[1]; }
		bool operator< (const Point& rhs) const;
	};

	/*!
	*	\class Line
	*	\brief 2차원 직선 표현하는 클래스 (homogeneous coordinates)
	*
	*	\author 한상준(manolike@snu.ac.kr)
	*	\date 2 Aug., 2017
	*/
	class Line
	{
		//프렌드
		friend double distance(Line &l, Point &p);
		friend Line shoot(Point &p, Point &dir);
		friend Line bisector(Point &p, Point &q);
		friend class Point;
	public:
		//생성자 및 소멸자
		Line();
		Line(Point &p, Point &q);
		virtual ~Line();

		//연산자
		bool operator <(const Line &rhs);
		Line operator -();
		Point operator ()(double t);

		//멤버 함수

	public:
		/*! \brief 2차원 직선을 저장하는 실수 배열 (homogeneous coordinates) */
		double L[3];
		Point P[2];
	};

	/*!
	*	\class Circle
	*	\brief 2차원 원을 표현하는 클래스
	*
	*	\author 한상준(manolike@snu.ac.kr)
	*	\date 3 Aug., 2017
	*/
	class Circle {
		//프렌드
		friend class CircularArc;
		friend std::tuple<Point, Point, int> intersection_collision(Circle &lhs, Circle &rhs);
		friend std::tuple<Point, Point, int> intersection_self(Circle &lhs, Circle &rhs);
		friend std::vector<Circle> importCircles(std::string filename);
		friend std::vector<Circle> operator +(std::vector<Circle> &lhs, std::vector<Circle> &rhs);
		friend std::ostream &operator <<(std::ostream &os, const Circle &p);

	public:
		//생성자 및 소멸자
		Circle(Point &_c = Point(), double _r = 0.0);
		Circle(Point &_c, Point &p);
		Circle(Point &p, Point &q, Point &v);
		//Circle(Geometry &g1, Geometry &g2); MA를 구할 때, Touching Point 와 관련
		virtual ~Circle();

		//연산자
		Point operator()(double angle);
		bool operator <(const Circle &rhs);
		bool operator >(const Circle &rhs);
		//멤버 함수
		bool contain(Point &p);
		bool contain_trimming(Point &p);
		bool contain(CircularArc &Arc, Point &p);
		Point projection(Point &p);
		Point localDirection(Point &p);
		void draw();

		// my func
		inline bool operator==(Circle& rhs)
		{
			if (this->c == rhs.c && fabs(this->r - rhs.r) < 1e-8)
				return true;
			else
				return false;
		}

	public:
		/*! \brief 원의 중심좌표 */
		Point c;

		/*! \brief 원의 반지름의 크기 */
		double r;
	};

	/*!
	*	\class Arc
	*	\brief 2차원 호를 표현하는 클래스
	*
	*	\author 한상준(manolike@snu.ac.kr)
	*	\date 3 Aug., 2017
	*/
	class CircularArc {
		//프렌드
		friend struct Geometry;
		friend std::tuple<Point, Point, int> intersection_CircularArc(CircularArc &lhs, CircularArc &rhs);
		friend bool intersection_bool(CircularArc &lhs, CircularArc &rhs, Point &p);
		friend std::ostream &operator <<(std::ostream &os, const CircularArc &p);

	public:
		//생성자 및 소멸자
		CircularArc() = default;
		CircularArc(CircularArc &arc, Point &p);
		CircularArc(Point &i, Point &e, Point &t);	// THIS FUNCTION IS DANGEROUS => NEVER USE IT.
		CircularArc(Point &_c, double _r, Vector normal1, Vector normal2);
		virtual ~CircularArc();

		//연산자
		Point operator()(double t);
		CircularArc operator-();
		bool operator <(CircularArc &rhs);

		//멤버 함수
		void refineNormal();
		bool trimmingTest();
		bool contain(Point &p);
		bool isCCW();
		bool isXQuardrants();
		bool isYQuardrants();
		bool isOuterBoundary(int _case);
		std::pair<CircularArc, CircularArc> subDiv(double t);
		void draw();
		void draw2(float z = 0.0f);

	public:
		/*! \brief Arc를 포함하는 원 */
		Circle c;

		/*! \brief Arc의 시작점과 끝점 */
		Point x[2];

		/*! \brief Arc의 시작점과 끝점에서의 normal */
		Point n[2];

		/*! \brief 시계반대방향이면 true, 아니면 false */
		bool ccw; //local-ccw // but this may differ in Minksum 

		/*! \brief Arc가 Boundary에서 ccw라는 것이 보증되면 true */
		bool boundary;

		/********************************************************/
		// What Mg Jung added are below
		/********************************************************/

		/* ccw of this arc in the output loop, */
		bool globalccw = true;

		bool convex = true;

		//debug
		CircularArc *lhs, *rhs;

		//used in MAT
		int originalIndex;

		/****************************************************/
		//Alias below
		/****************************************************/

		inline Point& n0() { return n[0]; }
		inline Point& n1() { return n[1]; }
		inline Point& x0() { return x[0]; }
		inline Point& x1() { return x[1]; }
		inline double& cx() { return c.c.x(); }
		inline double& cy() { return c.c.y(); }
		inline double& cr() { return c.r; }
		inline double atan0() { return atan2(n0().y(), n0().x()); }
		inline double atan1() { return atan2(n1().y(), n1().x()); }
		inline bool& lccw() { return ccw; } // local-ccw, might need some caution as mink-sum part it seems like he used gccw/lccw mixed for "bool ccw"
		inline bool& cvx() { return convex; }

	};

	typedef std::pair<CircularArc, CircularArc> biArc;

	typedef std::pair<biArc, biArc> biLens;


	enum CtrlPtType {
		CTRL_PT_E1 = 0,
		CTRL_PT_E2,
	};


	/*!
	*	\class BezierCrv
	*	\brief 다항식을 표현하는 클래스. (Bernstein form)
	*
	*	\author 한상준(manolike@snu.ac.kr)
	*	\date 7 Aug., 2017
	*/
	class BezierCrv
	{
		friend BezierCrv operator +(const BezierCrv &lhs, const BezierCrv &rhs);
		friend BezierCrv operator -(const BezierCrv &lhs, const BezierCrv &rhs);
		friend BezierCrv operator *(const BezierCrv &lhs, const BezierCrv &rhs);
		friend BezierCrv operator ^(const BezierCrv &lhs, const BezierCrv &rhs);
		friend BezierCrv diff(const BezierCrv &Crv);
		friend std::ostream &operator <<(std::ostream &os, const BezierCrv &c);
		friend std::vector<BezierCrv> import_Crv(std::string filename);
		friend bool collision(std::vector<BezierCrv> &lhs, std::vector<BezierCrv> &rhs, Point &p);

	public:
		// 생성자 및 소멸자
		BezierCrv(int deg = 0, CtrlPtType ptype = CTRL_PT_E1);
		BezierCrv(const BezierCrv &cpy);
		BezierCrv(Point a[]);
		BezierCrv(const BezierCrv &Crv, const Point &p);
		virtual ~BezierCrv();

		// 연산자
		BezierCrv &operator =(const BezierCrv &rhs);
		Point operator ()(double t);
		BezierCrv operator *(double rhs);

		// 멤버 함수
		Point tangentialVector_sp();
		Point tangentialVector_ep();
		std::pair<BezierCrv, BezierCrv> subDiv(double t = .5);
		double curvature(double t);
		bool isSatisfyingErrorBound();
		bool isSatisfyingErrorBoundBilens();
		BezierCrv &reverse();
		BezierCrv &reduceControlPt();
		std::vector<double> solve(double i = 0.0, double e = 1.0);
		std::vector<double> solvePararell(Vector &normal);
		BezierCrv getX();
		BezierCrv getY();
		bool aabbtest(BezierCrv &rhs);
		bool aabbtest(BezierCrv &rhs, Point &p);
		std::pair<CircularArc, CircularArc> BiArc();
		void draw();
		void segmentation(std::vector<BezierCrv> &a);
		std::vector<BezierCrv> integrityTest();
		void freeMemory();
	public:
		/*! \brief 베지에 곡선의 차수 */
		int Deg;

		/*! \brief 제어점의 차원 (CTRL_PT_E1: 1차원, CTRL_PT_E2: 2차원) */
		CtrlPtType PType;

		/*! \brief 베지에 곡선의 제어점 리스트 */
		std::vector<Point> P;

		/*! \brief 베지에 곡선의 끝 점(마지막)의 정보. 일반적인 함수의 경우 default (= 0) */
		SegmentTypes endType;

		/*! \brief 분할한 베지에 함수의 주소 */
		BezierCrv* child[2];

		/*! \bried biArc의 주소 */
		std::pair<CircularArc, CircularArc> Arcs;

		/*! \brief 곡선의 방향. 반시계방향인 경우 true */
		bool ccw;
	};


	enum SegmentTypes {
		defaultPt = 0,
		xyExtremePt,
		inflectionPt,
		curvatureExtremePt,
	};
	class Segment {
		friend bool operator <(const Segment &lhs, const Segment &rhs);
	public:
		//생성자 및 소멸자
		Segment() = default;
		Segment(double _value);
		Segment(double _value, SegmentTypes _St);
		virtual ~Segment();

		//연산자
	public:
		double value;
		SegmentTypes St;
	};

	/*!
	*	\struct Geometry
	*	\brief 2차원 곡선위의 한 점에 대한 기하학적 정보를 갖는 구조체
	*
	*	\author 한상준(manolike@snu.ac.kr)
	*	\date 7 Aug., 2017
	*/
	struct Geometry {
	public:
		//생성자 및 소멸자
		Geometry() = default;
		Geometry(BezierCrv &Crv, double t);
		Geometry(CircularArc &a, double t);
		virtual ~Geometry();

		//멤버 함수
		Circle osculatingCircle();

	public:
		/*! \brief geometry의 위치, 속도, 가속도, 법선, 곡률중심 */
		Point x, v, a, n, e;

		/*! \brief geometry의 곡률반지름 */
		double r;
	};



	/*!
	*	\clase ArcSpline
	*	\brief 연결된 Arc묶음의 정보를 저장하는 클래스.
	*
	*	\author 한상준(manolike@snu.ac.kr)
	*	\date 28 Aug., 2017
	*/
	class ArcSpline {
		//프렌드 함수
		friend bool connected(ArcSpline &lhs, ArcSpline &rhs);
		friend void selfIntersectionPts(ArcSpline &lhs, ArcSpline &rhs);
		friend void overlapTest(std::vector<ArcSpline> &source, ArcSpline &lhs, ArcSpline &rhs);
		friend void overlapTestR(std::vector<ArcSpline> &source, ArcSpline &lhs, ArcSpline &rhs);
		friend void overlapTestIden(std::vector<ArcSpline> &source, ArcSpline &lhs);
		friend short overlapCase(ArcSpline &lhs, ArcSpline &rhs);
		friend short overlapCaseR(ArcSpline &lhs, ArcSpline &rhs);
		friend void Convolution_ArcSpline(std::vector<ArcSpline> &source, ArcSpline & lhs, ArcSpline & rhs, int ls[2], int rs[2], bool reverse);
		friend bool aabbtest(ArcSpline &lhs, ArcSpline &rhs);
		friend bool aabbtest(ArcSpline &lhs, ArcSpline &rhs, std::pair<int, int> &left, std::pair<int, int> &right);
		friend bool aabbtest(CircularArc &lhs, ArcSpline &rhs, std::pair<int, int> &right);
		friend std::ostream &operator <<(std::ostream &os, const ArcSpline &p);
		//for extract statistics data
		friend void circleTrimming(std::vector<ArcSpline> &input, std::vector<ArcSpline> &trimmed);

		// code added for 2dplanning
		friend bool collision(std::vector<ArcSpline>& lhs, std::vector<ArcSpline>& rhs, Point& p);
		friend bool aabbtest(ArcSpline& lhs, ArcSpline& rhs, Point& p);
	public:
		//생성자 및 소멸자
		ArcSpline();
		ArcSpline(BezierCrv &Crv);
		ArcSpline(ArcSpline& originCrv, dividePts& begin, dividePts& end);
		virtual ~ArcSpline();

		//연산자

		//멤버함수
		Point &init();
		Point &end();
		Point &mid();
		bool contain(Vector &norm);
		void draw();
		int findIdx(Vector &norm);
		void finalTrimming_Simple();
		void finalTrimmingSuccessive(bool relPos, bool cut);
		std::vector<ArcSpline> finalTrimming_Complex();
		std::vector<ArcSpline> integrityTest();

	public:
		/*! \brief Arc의 집합 */
		std::vector<CircularArc> Arcs;

		/*! \brief index, Point, bool의 tuple. intersection이 있을 때 index의 Point 이후에 boundary를 형성하면 true, 아니면 false */
		std::vector<dividePts> intersections;

		/*! \brief 연결된 ArcSpline의 index */
		ArcSpline* neighbor[2];

		/*! \brief normal (반시계 방향으로) */
		Point n[2];

		/*! \brief 가장 가까운 ArcSpline과의 거리를 저장 */
		double neighborDist[2];

		/*! \brief 시작점과 끝점이 반대 ArcSpline의 끝점과 연결되어 있으면 false, 아니면 true */
		bool relativePosition[2];

		/*! \brief 연결된 ArcSpline의 주소가 채워졌는지를 판단 */
		bool _neighbor[2];

		/*! \brief ArcSpline의 방향 */
		bool xQuardrants, yQuardrants;

		/*! \brief 첫 번째는 ArcSpline의 회전방향 & convolution 이후의 spiral의 내부외부 판정에 이용 */
		bool ccw;

		/*! \brief Models_Rotated_Approx의 순서가 역방향인지 정방향인지를 판단 */

		/*! \brief ArcSpline이 무효한지를 판단. self intersection test에서 원래 ArcSpline이 쪼개져 무효한 경우 true, 아닌 경우 false */
		bool splited;

		/*! \brief final trimming 과정에서 참조된 경우 true, 아니면 false */
		bool referenced;
	};

	/*!
	*	\clase BCA
	*	\brief 연결된 Arc묶음의 정보를 저장하는 클래스.
	*
	*	\author 한상준(manolike@snu.ac.kr)
	*	\date 28 Aug., 2017
	*/
	class BCA {
		//프렌드 함수
		friend bool Collision_BCA(BCA &lhs, BCA &rhs);
		friend bool Collision_BCA(CircularArc &lhs, BCA &rhs);

	public:
		//생성자 및 소멸자
		BCA() = default;
		BCA(ArcSpline &s);
		BCA(ArcSpline &s, std::pair<int, int> &Idx);
		BCA(BezierCrv &Crv);
		virtual ~BCA();

		//연산자

		//멤버 함수
		bool contain(Point &P);
		double thickness();

	public:
		/*! \brief 바깥쪽 Circular Arc */
		CircularArc outer;

		/*! \brief 안쪽 Circular Arc */
		CircularArc inner;

		/*! \brief 시작점과 끝점 */
		Point x[2];

	};

	/*!
	*	\clase dividePts
	*	\brief Arc Spline 위의 Self Intersection Point의 정보를 저장하는 클래스
	*
	*	\author 한상준(manolike@snu.ac.kr)
	*	\date 28 Aug., 2017
	*/
	class dividePts {
		friend bool operator <(dividePts &lhs, dividePts &rhs);

	public:
		dividePts() = default;
		dividePts(int _idx, Point &_dividePt, bool _cutAfterPt);
	public:
		/*! \brief dividePts를 포함하는 Arc Spline의 시작점 - dividePts의 순서를 Sorting할 때 사용됨 */
		Point initPt;

		/*! \brief Self Intersection Point */
		Point dividePt;

		/*! \brief Arc Spline Segment의 Arc중 dividePts를 포함하는 Arc의 Index */
		int idx;

		/*! \brief dividePts 다음 부분이 지워지면 true, dividePts를 지나기 전의 부분이 지워지면 false */
		bool cutAfterPt;
	};

	/*!
	*	\clase CacheCircles
	*	\brief 가장 최근에 Circular Arc를 Trimming한 Circle의 주소를 담아놓는 클래스. Trimming시에 CacheCircles 안에 포함된 Circle에 포함되는지를 먼저 판단해본다
	*
	*	\author 한상준(manolike@snu.ac.kr)
	*	\date 21 Dec., 2017

	MG JUNG:
		Be aware, there shoudl be initialization outside of the constructor... (see minkSum())

	*/
	class CacheCircles {
		//연산자

	public:
		//멤버함수
		void reset();

	public:
		/*! \brief Cache_Trimming로 사용할 원 */
		Circle* cache[cacheSize];

		/*! \brief second chance algorithm 구현을 위한 bool */
		bool check[cacheSize];

		/*! \brief second chance algorithm의 index */
		int idx;
	};

	/*!
	*	\clase Grid
	*	\brief Circle을 담는 Grid의 정보를 저장하는 클래스
	*
	*	\author 한상준(manolike@snu.ac.kr)
	*	\date 11 Dec., 2018
	*/
	class Grid {

	public:
		// 멤버함수
		Grid();
		void insert(Circle * input);
		std::vector<int> find(const Point& p1, const Point& p2, const Point& p3);
		std::pair<Circle*, bool> trimming(Point& p1, Point& p2, Point& p3, int i, int j, bool singleGrid);

		/* 각 nodes의 좌표 */
		Point nodes[grid + 1][grid + 1];

		/* Grid를 모두 포함하는 Circle(만약 존재한다면!)의 주소 */
		Circle* coverCircle[grid][grid];

		/* 각 node의 x, y 경계 */
		double x[grid + 1], y[grid + 1];

		/* 각 grid에 포함되는 원의 address */
		std::vector< Circle* > gCircles[grid][grid];

		/* grid가 완전히 채워진 여부 판단. cover된 경우, insert도 안함! */
		bool cover[grid][grid];

		/* covercheck: internally used variable */ // Seems like unused?
		bool covercheck;

		/* Code by MG Jung*/
		std::vector<int> find(const Point& p1);
	};

}