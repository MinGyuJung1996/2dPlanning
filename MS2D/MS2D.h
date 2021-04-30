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
	*	\brief 2���� ��ġ�� ǥ���ϴ� Ŭ����
	*
	*	\author �ѻ���(manolike@snu.ac.kr)
	*	\date 1 Aug., 2017
	*/
	class Point
	{
		// ������
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
		// ������ �� �Ҹ���
		Point(double x = 0.0, double y = 0.0);
		Point(const Point &cpy);
		Point(Line &l, Line &m);
		Point(CircularArc &c);
		virtual ~Point();

		// ������
		Point operator -() const;
		Point &operator =(const Point &rhs);
		double &operator [](int idx) { return P[idx]; };
		bool operator ==(Point &rhs);
		bool operator <(Point &rhs);
		bool operator >(Point &rhs);

		// ��� �Լ�
		bool close(Point &rhs);
		bool exact(Point &rhs);
		double length();
		Point &normalize();
		Point rotate();
		Point rotate(double angle);
	public:
		/*! \brief 2���� ����Ʈ�� ��ġ�� �����ϴ� �Ǽ� �迭 */
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
	*	\brief 2���� ���� ǥ���ϴ� Ŭ���� (homogeneous coordinates)
	*
	*	\author �ѻ���(manolike@snu.ac.kr)
	*	\date 2 Aug., 2017
	*/
	class Line
	{
		//������
		friend double distance(Line &l, Point &p);
		friend Line shoot(Point &p, Point &dir);
		friend Line bisector(Point &p, Point &q);
		friend class Point;
	public:
		//������ �� �Ҹ���
		Line();
		Line(Point &p, Point &q);
		virtual ~Line();

		//������
		bool operator <(const Line &rhs);
		Line operator -();
		Point operator ()(double t);

		//��� �Լ�

	public:
		/*! \brief 2���� ������ �����ϴ� �Ǽ� �迭 (homogeneous coordinates) */
		double L[3];
		Point P[2];
	};

	/*!
	*	\class Circle
	*	\brief 2���� ���� ǥ���ϴ� Ŭ����
	*
	*	\author �ѻ���(manolike@snu.ac.kr)
	*	\date 3 Aug., 2017
	*/
	class Circle {
		//������
		friend class CircularArc;
		friend std::tuple<Point, Point, int> intersection_collision(Circle &lhs, Circle &rhs);
		friend std::tuple<Point, Point, int> intersection_self(Circle &lhs, Circle &rhs);
		friend std::vector<Circle> importCircles(std::string filename);
		friend std::vector<Circle> operator +(std::vector<Circle> &lhs, std::vector<Circle> &rhs);
		friend std::ostream &operator <<(std::ostream &os, const Circle &p);

	public:
		//������ �� �Ҹ���
		Circle(Point &_c = Point(), double _r = 0.0);
		Circle(Point &_c, Point &p);
		Circle(Point &p, Point &q, Point &v);
		//Circle(Geometry &g1, Geometry &g2); MA�� ���� ��, Touching Point �� ����
		virtual ~Circle();

		//������
		Point operator()(double angle);
		bool operator <(const Circle &rhs);
		bool operator >(const Circle &rhs);
		//��� �Լ�
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
		/*! \brief ���� �߽���ǥ */
		Point c;

		/*! \brief ���� �������� ũ�� */
		double r;
	};

	/*!
	*	\class Arc
	*	\brief 2���� ȣ�� ǥ���ϴ� Ŭ����
	*
	*	\author �ѻ���(manolike@snu.ac.kr)
	*	\date 3 Aug., 2017
	*/
	class CircularArc {
		//������
		friend struct Geometry;
		friend std::tuple<Point, Point, int> intersection_CircularArc(CircularArc &lhs, CircularArc &rhs);
		friend bool intersection_bool(CircularArc &lhs, CircularArc &rhs, Point &p);
		friend std::ostream &operator <<(std::ostream &os, const CircularArc &p);

	public:
		//������ �� �Ҹ���
		CircularArc() = default;
		CircularArc(CircularArc &arc, Point &p);
		CircularArc(Point &i, Point &e, Point &t);	// THIS FUNCTION IS DANGEROUS => NEVER USE IT.
		CircularArc(Point &_c, double _r, Vector normal1, Vector normal2);
		virtual ~CircularArc();

		//������
		Point operator()(double t);
		CircularArc operator-();
		bool operator <(CircularArc &rhs);

		//��� �Լ�
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
		/*! \brief Arc�� �����ϴ� �� */
		Circle c;

		/*! \brief Arc�� �������� ���� */
		Point x[2];

		/*! \brief Arc�� �������� ���������� normal */
		Point n[2];

		/*! \brief �ð�ݴ�����̸� true, �ƴϸ� false */
		bool ccw; //local-ccw // but this may differ in Minksum 

		/*! \brief Arc�� Boundary���� ccw��� ���� �����Ǹ� true */
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
	*	\brief ���׽��� ǥ���ϴ� Ŭ����. (Bernstein form)
	*
	*	\author �ѻ���(manolike@snu.ac.kr)
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
		// ������ �� �Ҹ���
		BezierCrv(int deg = 0, CtrlPtType ptype = CTRL_PT_E1);
		BezierCrv(const BezierCrv &cpy);
		BezierCrv(Point a[]);
		BezierCrv(const BezierCrv &Crv, const Point &p);
		virtual ~BezierCrv();

		// ������
		BezierCrv &operator =(const BezierCrv &rhs);
		Point operator ()(double t);
		BezierCrv operator *(double rhs);

		// ��� �Լ�
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
		/*! \brief ������ ��� ���� */
		int Deg;

		/*! \brief �������� ���� (CTRL_PT_E1: 1����, CTRL_PT_E2: 2����) */
		CtrlPtType PType;

		/*! \brief ������ ��� ������ ����Ʈ */
		std::vector<Point> P;

		/*! \brief ������ ��� �� ��(������)�� ����. �Ϲ����� �Լ��� ��� default (= 0) */
		SegmentTypes endType;

		/*! \brief ������ ������ �Լ��� �ּ� */
		BezierCrv* child[2];

		/*! \bried biArc�� �ּ� */
		std::pair<CircularArc, CircularArc> Arcs;

		/*! \brief ��� ����. �ݽð������ ��� true */
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
		//������ �� �Ҹ���
		Segment() = default;
		Segment(double _value);
		Segment(double _value, SegmentTypes _St);
		virtual ~Segment();

		//������
	public:
		double value;
		SegmentTypes St;
	};

	/*!
	*	\struct Geometry
	*	\brief 2���� ����� �� ���� ���� �������� ������ ���� ����ü
	*
	*	\author �ѻ���(manolike@snu.ac.kr)
	*	\date 7 Aug., 2017
	*/
	struct Geometry {
	public:
		//������ �� �Ҹ���
		Geometry() = default;
		Geometry(BezierCrv &Crv, double t);
		Geometry(CircularArc &a, double t);
		virtual ~Geometry();

		//��� �Լ�
		Circle osculatingCircle();

	public:
		/*! \brief geometry�� ��ġ, �ӵ�, ���ӵ�, ����, ����߽� */
		Point x, v, a, n, e;

		/*! \brief geometry�� ��������� */
		double r;
	};



	/*!
	*	\clase ArcSpline
	*	\brief ����� Arc������ ������ �����ϴ� Ŭ����.
	*
	*	\author �ѻ���(manolike@snu.ac.kr)
	*	\date 28 Aug., 2017
	*/
	class ArcSpline {
		//������ �Լ�
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
		//������ �� �Ҹ���
		ArcSpline();
		ArcSpline(BezierCrv &Crv);
		ArcSpline(ArcSpline& originCrv, dividePts& begin, dividePts& end);
		virtual ~ArcSpline();

		//������

		//����Լ�
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
		/*! \brief Arc�� ���� */
		std::vector<CircularArc> Arcs;

		/*! \brief index, Point, bool�� tuple. intersection�� ���� �� index�� Point ���Ŀ� boundary�� �����ϸ� true, �ƴϸ� false */
		std::vector<dividePts> intersections;

		/*! \brief ����� ArcSpline�� index */
		ArcSpline* neighbor[2];

		/*! \brief normal (�ݽð� ��������) */
		Point n[2];

		/*! \brief ���� ����� ArcSpline���� �Ÿ��� ���� */
		double neighborDist[2];

		/*! \brief �������� ������ �ݴ� ArcSpline�� ������ ����Ǿ� ������ false, �ƴϸ� true */
		bool relativePosition[2];

		/*! \brief ����� ArcSpline�� �ּҰ� ä���������� �Ǵ� */
		bool _neighbor[2];

		/*! \brief ArcSpline�� ���� */
		bool xQuardrants, yQuardrants;

		/*! \brief ù ��°�� ArcSpline�� ȸ������ & convolution ������ spiral�� ���οܺ� ������ �̿� */
		bool ccw;

		/*! \brief Models_Rotated_Approx�� ������ ���������� ������������ �Ǵ� */

		/*! \brief ArcSpline�� ��ȿ������ �Ǵ�. self intersection test���� ���� ArcSpline�� �ɰ��� ��ȿ�� ��� true, �ƴ� ��� false */
		bool splited;

		/*! \brief final trimming �������� ������ ��� true, �ƴϸ� false */
		bool referenced;
	};

	/*!
	*	\clase BCA
	*	\brief ����� Arc������ ������ �����ϴ� Ŭ����.
	*
	*	\author �ѻ���(manolike@snu.ac.kr)
	*	\date 28 Aug., 2017
	*/
	class BCA {
		//������ �Լ�
		friend bool Collision_BCA(BCA &lhs, BCA &rhs);
		friend bool Collision_BCA(CircularArc &lhs, BCA &rhs);

	public:
		//������ �� �Ҹ���
		BCA() = default;
		BCA(ArcSpline &s);
		BCA(ArcSpline &s, std::pair<int, int> &Idx);
		BCA(BezierCrv &Crv);
		virtual ~BCA();

		//������

		//��� �Լ�
		bool contain(Point &P);
		double thickness();

	public:
		/*! \brief �ٱ��� Circular Arc */
		CircularArc outer;

		/*! \brief ���� Circular Arc */
		CircularArc inner;

		/*! \brief �������� ���� */
		Point x[2];

	};

	/*!
	*	\clase dividePts
	*	\brief Arc Spline ���� Self Intersection Point�� ������ �����ϴ� Ŭ����
	*
	*	\author �ѻ���(manolike@snu.ac.kr)
	*	\date 28 Aug., 2017
	*/
	class dividePts {
		friend bool operator <(dividePts &lhs, dividePts &rhs);

	public:
		dividePts() = default;
		dividePts(int _idx, Point &_dividePt, bool _cutAfterPt);
	public:
		/*! \brief dividePts�� �����ϴ� Arc Spline�� ������ - dividePts�� ������ Sorting�� �� ���� */
		Point initPt;

		/*! \brief Self Intersection Point */
		Point dividePt;

		/*! \brief Arc Spline Segment�� Arc�� dividePts�� �����ϴ� Arc�� Index */
		int idx;

		/*! \brief dividePts ���� �κ��� �������� true, dividePts�� ������ ���� �κ��� �������� false */
		bool cutAfterPt;
	};

	/*!
	*	\clase CacheCircles
	*	\brief ���� �ֱٿ� Circular Arc�� Trimming�� Circle�� �ּҸ� ��Ƴ��� Ŭ����. Trimming�ÿ� CacheCircles �ȿ� ���Ե� Circle�� ���ԵǴ����� ���� �Ǵ��غ���
	*
	*	\author �ѻ���(manolike@snu.ac.kr)
	*	\date 21 Dec., 2017

	MG JUNG:
		Be aware, there shoudl be initialization outside of the constructor... (see minkSum())

	*/
	class CacheCircles {
		//������

	public:
		//����Լ�
		void reset();

	public:
		/*! \brief Cache_Trimming�� ����� �� */
		Circle* cache[cacheSize];

		/*! \brief second chance algorithm ������ ���� bool */
		bool check[cacheSize];

		/*! \brief second chance algorithm�� index */
		int idx;
	};

	/*!
	*	\clase Grid
	*	\brief Circle�� ��� Grid�� ������ �����ϴ� Ŭ����
	*
	*	\author �ѻ���(manolike@snu.ac.kr)
	*	\date 11 Dec., 2018
	*/
	class Grid {

	public:
		// ����Լ�
		Grid();
		void insert(Circle * input);
		std::vector<int> find(const Point& p1, const Point& p2, const Point& p3);
		std::pair<Circle*, bool> trimming(Point& p1, Point& p2, Point& p3, int i, int j, bool singleGrid);

		/* �� nodes�� ��ǥ */
		Point nodes[grid + 1][grid + 1];

		/* Grid�� ��� �����ϴ� Circle(���� �����Ѵٸ�!)�� �ּ� */
		Circle* coverCircle[grid][grid];

		/* �� node�� x, y ��� */
		double x[grid + 1], y[grid + 1];

		/* �� grid�� ���ԵǴ� ���� address */
		std::vector< Circle* > gCircles[grid][grid];

		/* grid�� ������ ä���� ���� �Ǵ�. cover�� ���, insert�� ����! */
		bool cover[grid][grid];

		/* covercheck: internally used variable */ // Seems like unused?
		bool covercheck;

		/* Code by MG Jung*/
		std::vector<int> find(const Point& p1);
	};

}