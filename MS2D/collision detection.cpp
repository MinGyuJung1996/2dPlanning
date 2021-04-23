#include "collision detection.hpp"
using namespace std;


namespace cd
{
	const double PI			= 3.14159265358979323846264;
	const double PI2		= 2 * 3.14159265358979323846264;
	const double PI_half	= 0.5 * 3.14159265358979323846264;
	const double RAD_TO_DEG = 180 / PI;
	const double DEG_TO_RAD = PI / 180;


	std::tuple<Point, Point, int>
		_tRSI_getCircleIntersecion(Circle& lhs, Circle& rhs);

	decltype(planning::isNormalBetween)*	isNormalBetween				= planning::isNormalBetween;
	decltype(ms::intersection_CircularArc)*	getIntersectionOfArcs		= ms::intersection_CircularArc; // ms::intersection_circularArcs may be dangerous for arcs with large d(theta)
	decltype(ms::intersection_self)*		getIntersectionOfCircles	= cd::_tRSI_getCircleIntersecion;

//deprecated 
//#define TrigInput(rotation) ((rotation) * DEG_TO_RAD) // currently rotation = degrees, but maybe for the future
//	// inside each function, everything is considered to be in "radian" not "degree".
//	// input param of this header will be of any unit, with changing the macro above

	// just some tags with no affect to code
#define INPUT
#define OUTPUT
#define LOOP
#define DEBUG


	/*
	Def:
	Assume:
	Param:
	*/
	void interSliceCollisionDetection(Robot& robot, double rotation, Point& translation, Environment& env);


	/**********************************************************************************/
	//**  0-1. helper func(transform primitives) 
	/**********************************************************************************/
	
	/*
	Def : scale arc with the center as origin.
	*/
	CircularArc	scaleArc(CircularArc& arc, double scale)
	{
		CircularArc ret = arc;
		ret.c.c = ret.c.c * scale;
		ret.c.r = ret.c.r * scale;
		ret.x0() = ret.x0() * scale;
		ret.x1() = ret.x1() * scale;

		return ret;
	}

	/*
	Def : rotate "arc" around the origin with "rotation" degrees
	*/
	CircularArc rotateArc(CircularArc& arc, double degree)
	{
		double rotation = DEG_TO_RAD * degree;
		auto c = cos(rotation);
		auto s = sin(rotation);

		CircularArc ret = arc;
		ret.c.c = rotatePoint(ret.c.c, c, s);
		ret.x[0] = rotatePoint(ret.x[0], c, s);
		ret.x[1] = rotatePoint(ret.x[1], c, s);
		ret.n[0] = rotatePoint(ret.n[0], c, s);
		ret.n[1] = rotatePoint(ret.n[1], c, s);

		return ret;
	}

	/*
	Def : rotate Point with an angle
	*/
	Point rotatePoint(Point& p, double rotationDegree)
	{
		double rotationRadian = DEG_TO_RAD * rotationDegree;
		auto cosine = cos(rotationRadian);
		auto sine	= sin(rotationRadian);

		Point ret;
		ret.x() = cosine * p.x() - sine * p.y();
		ret.y() = cosine * p.y() + sine * p.x();
		return ret;
	}

	/*
	Def : rotate a point given its cos, sin
	*/
	Point rotatePoint(Point& p, double cosine, double sine)
	{
		Point ret;
		ret.x() = cosine * p.x() - sine * p.y();
		ret.y() = cosine * p.y() + sine * p.x();
		return ret;
	}

	/*
	Def : translate arc in world-space
	*/
	CircularArc translateArc(CircularArc& arc, Point& translation)
	{
		CircularArc ret = arc;

		ret.c.c = ret.c.c + translation;
		ret.x[0] = ret.x[0] + translation;
		ret.x[1] = ret.x[1] + translation;

		//TODO
		return ret;
	}

	/**********************************************************************************/
	//**  0-2. helper func(simple math)
	/**********************************************************************************/

	/*
	Def:
		Arc starts from tetha0 and travles to theta1, with its ccw-info and with continuous theta.
		return pair(theta0, theta1)
			theta0 will be inside [-pi, +pi]
			whilst, theta1 can be inside [theta0 - 2pi, theta0 + 2pi] = [-3pi, +3pi]
			(this is to address the problem of discontinuity of "pi + 0 = -pi" in trigonometry functions)
	Assume:
		counterclockwise info is correct.
	*/
	pair<double, double>
		getParameterOfArc(CircularArc& arc)
	{
		auto theta0 = atan2(arc.n0().y(), arc.n0().x());
		auto theta1 = atan2(arc.n1().y(), arc.n1().x());

		if (arc.ccw)
		{	
			//theta 1 should be larger
			if (theta1 < theta0)
				theta1 += PI2;
		}
		else
		{
			// clockwise: theta is decreasing
			if (theta1 > theta0)
				theta1 -= PI2;
		}

		return make_pair(theta0, theta1);
	}

	/*
	Def: get theta value of normal inside the interval of theta of arc
	*/
	double
		getParameterOfArc(CircularArc& arc, Point& normal)
	{
		auto theta0 = atan2(arc.n0().y(), arc.n0().x());
		
		return getParameterOfArc(arc, normal, theta0);
	}

	/*
	Def: get theta value of normal inside the interval of theta of arc, when theta0 is precalculated(common-case)
	*/
	double
		getParameterOfArc(CircularArc& arc, Point& normal, double theta0)
	{
		auto theta1 = atan2(normal.y(), normal.x());

		if (arc.ccw)
		{
			//theta 1 should be larger
			if (theta1 < theta0)
				theta1 += PI2;
		}
		else
		{
			// clockwise: theta is decreasing
			if (theta1 > theta0)
				theta1 -= PI2;
		}

		return theta1;
	}

	// used as function pointer
	bool
		less(double a, double b) { return a <= b; };

	// used as function pointer
	bool
		greater(double a, double b) { return a >= b; };



	/**********************************************************************************
	**  0-3. helper func(abstraction)
	***********************************************************************************/
	/*
	Def: build arc from 
		Circle info
		Starting angle(rad)
		Finishing angle(rad)
	Assume:
		radius can be positive or negative.
			if(negative) flip signs of (rad, normal0, normal1)
		convex/concave is determined by the caller, not by this func
		ccw set.
	*/
	CircularArc constructArc(Point& center, double radius, double radian0, double radian1)
	{
		Point normal0(cos(radian0), sin(radian0));
		Point normal1(cos(radian1), sin(radian1));

		CircularArc ret;
		if (radius < 0)
			ret = CircularArc(center, -radius, -normal0, -normal1);
		else
			ret = CircularArc(center, radius, normal0, normal1);

		// since constructor's ccw is strange.
		if (radian1 > radian0)
			ret.ccw = true;
		else
			ret.ccw = false;

		return ret;
	}

	/*
	Def: given an arc, contstruct another arc that is same except for the starting/ending angle
		Used in trimRotationSupersetIntoLoops 3-2, to make a segment of a known arc
		Sets
			basic 4 info (center, rad, theta0, theta1)
			ccw, convexity
	Assume:
		ccw info does not change
			radian0 and radian1 should not be swapped.
		radian should follow the way used in tRSIL
	*/
	CircularArc constructArc(CircularArc& arc, double radian0, double radian1)
	{
		CircularArc ret;

		ret.c = arc.c;
		ret.convex = arc.convex;

		if (radian1 > radian0)
			ret.ccw = true;
		else
			ret.ccw = false;

		ret.n0() = Point(cos(radian0), sin(radian0));
		ret.n1() = Point(cos(radian1), sin(radian1));
		ret.x0() = ret.c.c + (ret.c.r * ret.n0());
		ret.x1() = ret.c.c + (ret.c.r * ret.n1());

		return ret;
	}

	/*
	Def : given an circle and two point on that circle(x0 and x1), and how thet are traveresed(ccw)
	*/
	CircularArc constructArc(Point& center, Point& x0, Point& x1, bool ccw)
	{
		CircularArc arc;
		arc.c.c = center;
		arc.c.r = sqrt((x0 - center).length2());
		arc.x0() = x0;
		arc.x1() = x1;
		arc.n0() = (x0 - center) / arc.c.r;
		arc.n1() = (x1 - center) / arc.c.r;
		arc.ccw = ccw;

		return arc;
	}


	/*
	Def : given two endpoints of an arc, and a tangent at one of its endpoints, this uniquely decides and arc. return it.
		(Actually there is a freedom in ccw. But we assume that dTheta < 180degree)
	Does almost the same thing with the constructor(of same argument type) of "CircularArc"
		But that constructor was buggy, so this is used.
	*/
	CircularArc constructArc(Point& x0, Point& x1, Point& t0)
	{
		CircularArc ret;
		Circle c(x0, x1, t0);
		ret.c = c;
		ret.x0() = x0;
		ret.x1() = x1;
		ret.n0() = c.localDirection(x0);
		ret.n1() = c.localDirection(x1);
		if (!counterclockwise(ret.n[0], ret.n[1]))
			ret.ccw = false;
		else
			ret.ccw = true;
		ret.boundary = false;

		return ret;
	}

	/*
	Def : given two endpoints of an arc, radius, and its ccw, construct an arc which is shorter(there are 2 possible arcs under this configuration)
		The center of the circle can lies in both the left/right side of the chord(p,q).
		We choose left/right placement of the center, so that the arc is smaller. (this can be changed by altering chooseSmallerArc)
		Since the function's purpose is to make an arc that is almost a straight line.
	Desc :
	p, q, r => 2 circles can be possible. 4 arcs can be possible
	counterclockwise info and chooseSmallerArc will decide which arc is used.

	Assume : 
	radius > dist(p,q)/2
	*/
	CircularArc constructArc(Point& x0, Point& x1, double r, bool ccw, bool chooseSmallerArc)
	{
		// get center with pythagoras
		Point dx = x1 - x0;
		double bsquare = (dx.length2()) / 4;
		double asquare = r * r - bsquare;
		double a = sqrt(asquare); // dist from circle_center to chord

		// get normal direction using ccw
		Point normal;
		if (ccw ^ chooseSmallerArc)
			normal = cd::rotatePoint(dx, -90).normalize();
		else
			normal = cd::rotatePoint(dx, 90).normalize();

		Point center = (x1 + x0) * 0.5 + normal * a;

		return constructArc(center, x0, x1, ccw);
	}

	/**********************************************************************************/
	//**  1. Main func
	/**********************************************************************************/
	
	void cdRobotObstacle()
	{
		// 1. compute superset of (swept-volume-boundary)

		// 2. do test : superset vs obstacle
	}
	void cdRobotEnvironment()
	{
		// 1. compute superset

		// 2. find sets of arcs to be tested

		// 2. do test : superset vs Env
	}

	/**********************************************************************************/
	//**  2. Building Superset(or trimmed superset)
	/**********************************************************************************/

	/*
	Def:
		Computes the superset used in the test
	*/
	std::pair<vector<CircularArc>, vector<lineSegment>> 
		getSuperSet(vector<CircularArc> robot, double degree, Point translation)
	{
		auto RS = getRotationSuperSet(robot, degree);
		return getTranslationSuperSet(RS, translation);
	}

	/*
	Def: given a robot and rotation, compute its superset
	Description:
		Output is composed of 3.
		1. robot
		2. robot rotated by "double rotation"
		3. some arcs
			3.1 Only those points that have normals parallel to its position(since center of rot is model center) will form an arc
	*/
	vector<CircularArc> 
		getRotationSuperSet(Robot& robot, double degree)
	{
		double rotation = DEG_TO_RAD * degree;

		// 0. output
		vector<CircularArc> output;
		// 0-1. some commonly used variables
		Point Origin(0, 0);
		double COS = cos(rotation);
		double SIN = sin(rotation);

		// 1. add original robot & rotated robot
		{
			for (auto &arc : robot)
			{
				output.push_back(arc);
				output.push_back(rotateArc(arc, rotation));
			}
		}

		// 2. find points tanget to moving dir
		{
			for (auto &arc : robot)
			{
				// 2-1. such point is inside the line, formed by (origin, circle-center)
				auto normal = arc.c.c;
				auto len = sqrt(normal.length2());
				normal = normal / len;
				
				// 2-2-1. R = dist + r 
				if (planning::isNormalBetween(arc.n[0], arc.n[1], normal))
				{
					auto r = len + arc.c.r;
					output.emplace_back(Origin, r, normal, rotatePoint(normal, COS, SIN));
				}
				// 2-2-2. R = dist - r;
				if (planning::isNormalBetween(arc.n[0], arc.n[1], -normal))
				{
					auto r = len - arc.c.r;
					if (r < 0)
					{
						output.emplace_back(Origin, -r, -normal, rotatePoint(-normal, COS, SIN));
					}
					else
					{
						output.emplace_back(Origin, r, normal, rotatePoint(normal, COS, SIN));
					}
				}
			}
		}

		//TODO
		return output;
	}

	/*
	Def: 
		Similar to getRotationSuperSet, but trim some with cross-product-info
	Desc: 
		Consider a circle. Its rotational sweep's superset would be {C0, C1, A0, A1} (2 circles at end position & 2 intermediate arcs)
			For points p of C0, if(cross(C0, p) > 0) trim
			For points p of C1, if(cross(C1, p) < 0) trim
		Do this for all arcs
	Assume:
		Rotation Angle isn't that big. (Think of what would happen, if (robot is circle, angle is 360 deg))
		ccw is correct ofr each arc.
		"double rotation" > 0 : this is for code simplexity
	TODO:
		maybe some collision detection should be added for better results with big rotation angle.
			maybe some codes in MinkSum.selfCollision might help.
	*/
	vector<CircularArc>	
		getRotationBoundary(Robot& robotToBeRotated, double degree, DEBUG int grb_debug)
	{
		// -2. rotation
		double rotation = DEG_TO_RAD * degree; // rotation is now guaranteed to be "radian"

		// -1. pre-process: if rotation < 0, pre-rotate robot by rotation & use -rotation
		Robot temporaryContainerForMinusRotation; // if(rot<0) this container is referenced, do not use this symbol
		// lambda to use if & ref
		auto tempLambda = [&robotToBeRotated, &temporaryContainerForMinusRotation, &rotation]() -> Robot&
		{
			if (rotation < 0)
			{
				for (auto& arc : robotToBeRotated)
					temporaryContainerForMinusRotation.push_back(rotateArc(arc, (rotation)));
				rotation = -rotation;
				return temporaryContainerForMinusRotation;
			}
			else return robotToBeRotated;
		};
		Robot& robot = tempLambda();

		// 0. init stuff.
		Robot ret;
		ret.reserve(robot.size());
		Point origin(0, 0);

		// 0-1. commonly used loop variable
		//bool ccw; // initialized in loop;
		//auto comp =[&ccw](double lhs, double rhs) {return (lhs<rhs) != ccw}

		// dbg_out
		{
			int v = 0, c = 0;
			for (auto& a : robot)
				if (a.convex)
					v++;
				else
					c++;
			cout << "ROBOT conv conc : " << v << "   " << c << endl;

		}
		// ~dbg_out

		DEBUG int dbg_c = 0, dbg_v = 0;

		for (auto& arc : robot)
		if(arc.convex)
		{

			// 1. get t0, t1
			auto minmax = getParameterOfArc(arc); 
			auto t0 = minmax.first; // note that t0 > t1 might be possible (with clockwise arc)
			auto t1 = minmax.second;

			// 2. get tp(theta of center) and tm(theta of -center)
			auto& center = arc.c.c;
			auto tp = atan2(center.y(), center.x());
			//auto tm = tp + PI; // notice that this is between [0, 2pi]
			auto tm = atan2(-center.y(), -center.x()); // notice that this was not equivalent to "auto tm = tp + PI"
			//// debug
			//cout << t0 << " " << tp << " " << tm << " " << t1 << endl;

			// 3. arrange tp,tm so that tp, tm, t1 is on the same direction from t0
			bool ccw = arc.ccw;
			{
				if (ccw)
				{
					//theta should increase
					while (tp < t0) tp += PI2;
					while (tm < t0) tm += PI2;
				}
				else
				{
					//theta should decrease
					while (tp > t0) tp -= PI2;
					while (tm > t0) tm -= PI2;
				}
			}

			// 4. divide into cases
			/*
			Possible orderings : 6 case
			t0 t1 tp tm (or) t0 t1 tm tp 
			t0 tp t1 tm (or) t0 tm t1 tp
			t0 tp tm t1 (or) t0 tm tp t1
			*/
			int count = 0;
			bool tpFirst = false; //tp is closer to t0 then tm.

			// 4-1. build comparator func ptr: comp(t0,t1) = true (regardless of ccw)
			decltype(cd::greater)* comp;
			if (ccw) comp = cd::less;
			else comp = cd::greater;

			// 4-2. build count & tpFirst
			if (comp(tp, t1)) count++;
			if (comp(tm, t1)) count++;
			if (comp(tp, tm)) tpFirst = true;

			////debug
			//cout << "grb: convex case division count : " << count << endl;
			//if (count == 0)
			//	cout << "after PI2 : " << t0 << " " << tp << " " << tm << " " << t1 << endl;

			// 4-3. build tList;
			vector<double> tList;
			{
				switch (count)
				{
				case 0:
					tList.push_back(t0);
					tList.push_back(t1);
					break;
				case 1:
					tList.push_back(t0);
					if (tpFirst)
						tList.push_back(tp);
					else
						tList.push_back(tm);
					tList.push_back(t1);
					break;
				case 2:
					tList.push_back(t0);
					if (tpFirst)
					{
						tList.push_back(tp);
						tList.push_back(tm);
					}
					else
					{
						tList.push_back(tm);
						tList.push_back(tp);
					}
					tList.push_back(t1);

				}
			}

			// 5. evalueate case
			{
				// consider 
				// 1. ordering of arcs -> this seems to be done later... after trimming. Or a code to find loop?
				// 2. ccw of new arcs -> taken care of when making swept-point
				// 3. rotation < 0? -> do it by preprocessing in section -1. -> now rot > 0
				LOOP double end = t1, start;
				LOOP double pointSweepRadius;
				double centerDist = sqrt(arc.c.c.length2());

				LOOP bool CrossSignPositive;
				LOOP double averageT;
				//auto _Cross = cross(arc.c.c, arc.x1());
				//if (_Cross > 0)
				//	CrossSignPositive = true;
				//else if (_Cross < 0)
				//	CrossSignPositive = false;
				//else
				//{
				//	// hard case when arc end point is parallel to center dir
				//	
				//	//dbg_out
				//	cout << "hello" << endl;
				//}


				// points with postive cross will be rotated with "rotation"
				// this is true since "rotation" is guaranteed to be >0 at this point.
				// should be flipped after each case label.

				switch (count)
				// switch not having 'break' is intentional, as labels above are supersets.
				// e.g.) if (t0 tp tm t1) : 3 arcs from (tm, t1) (tp, tm) (t0, tp) + 2 arcs from (sweep tm) (sweep tp)
				{
				case 2:
					
					// 5-1. build Arc piece from the original "arc" : (t1, tp) or (t1, tm)
					if (tpFirst) start = tm;
					else start = tp;

					averageT = (tList[3] + tList[2]) / 2;
					CrossSignPositive = cross(arc.c.c, Point(cos(averageT), sin(averageT))) > 0;

					if(CrossSignPositive)
						ret.push_back(rotateArc(constructArc(arc.c.c, arc.c.r, start, end), degree));
					else
						ret.push_back(constructArc(arc.c.c, arc.c.r, start, end));

					// 5-2. build arc by sweeping point (tp) or (tm)
					// check ccw so that it is same with original arc
					if (tpFirst) pointSweepRadius = arc.c.r - centerDist; // this should be in this order: since for tm case & cd > arc.r => normal dir should be flipped... if cd < arc.r, normal doesnot need to be flipped
					else pointSweepRadius = centerDist + arc.c.r;
					if (CrossSignPositive)
						ret.push_back(constructArc(origin, pointSweepRadius, start + rotation, start));
					else
						ret.push_back(constructArc(origin, pointSweepRadius, start, start + rotation));

					// 5-2.5 take care of LOOP_VARIABLES
					//CrossSignPositive = !CrossSignPositive;
					end = start;

					//TODO when (sweeping tm) becomes concave...
				case 1:

					// 5-3.
					if (tpFirst) start = tp;
					else start = tm;


					averageT = (tList[2] + tList[1]) / 2;
					CrossSignPositive = cross(arc.c.c, Point(cos(averageT), sin(averageT))) > 0;

					if (CrossSignPositive)
						ret.push_back(rotateArc(constructArc(arc.c.c, arc.c.r, start, end), degree));
					else
						ret.push_back(constructArc(arc.c.c, arc.c.r, start, end));

					//CrossSignPositive = !CrossSignPositive;

					// 5-4.
					if (tpFirst) pointSweepRadius = centerDist + arc.c.r;
					else pointSweepRadius =  arc.c.r - centerDist; // see 5-2 for this order
					if (CrossSignPositive)
						ret.push_back(constructArc(origin, pointSweepRadius, start + rotation, start));
					else
						ret.push_back(constructArc(origin, pointSweepRadius, start, start + rotation));

					//dbg_out
					cout << pointSweepRadius << endl;

					end = start;
				case 0:

					// 5-5.
					start = t0;

					averageT = (tList[1] + tList[0]) / 2;
					CrossSignPositive = cross(arc.c.c, Point(cos(averageT), sin(averageT))) > 0;

					if (CrossSignPositive)
						ret.push_back(rotateArc(constructArc(arc.c.c, arc.c.r, start, end), degree));
					else
						ret.push_back(constructArc(arc.c.c, arc.c.r, start, end));
					break;

				default:
					break;
				}
			}

		}//end of convex-arc case
		else
		{

			//concave case;

			/*
			// for now just add all that can be.
			ret.push_back(arc);
			ret.push_back(rotateArc(arc, degree));

			auto r0 = sqrt(arc.x0().length2());
			auto theta0 = atan2(arc.x0().y(), arc.x0().x());
			ret.push_back(constructArc(origin, r0, theta0, theta0 + rotation));
			auto r1 = sqrt(arc.x1().length2());
			auto theta1 = atan2(arc.x1().y(), arc.x1().x());
			ret.push_back(constructArc(origin, r1, theta1, theta1 + rotation));
			*/

			// similar to that of convex, just, no added arcs(with origin center), and rotating policies for original arcs has changed.

			// 1. get t0, t1
			auto minmax = getParameterOfArc(arc);
			auto t0 = minmax.first; // note that t0 > t1 might be possible (with clockwise arc)
			auto t1 = minmax.second;

			// 2. get tp(theta of center) and tm(theta of -center)
			auto& center = arc.c.c;
			auto tp = atan2(center.y(), center.x());
			auto tm = tp + PI; // notice that this is between [0, 2pi]

			// 3. arrange tp,tm so that tp, tm, t1 is on the same direction from t0
			bool ccw = arc.ccw;
			{
				if (ccw)
				{
					//theta should increase
					while (tp < t0) tp += PI2;
					while (tm < t0) tm += PI2;
				}
				else
				{
					//theta should decrease
					while (tp > t0) tp -= PI2;
					while (tm > t0) tm -= PI2;
				}
			}

			// 4. divide into cases
			/*
			Possible orderings : 6 case
			t0 t1 tp tm (or) t0 t1 tm tp
			t0 tp t1 tm (or) t0 tm t1 tp
			t0 tp tm t1 (or) t0 tm tp t1
			*/
			int count = 0;
			bool tpFirst = false; //tp is closer to t0 then tm.

			// 4-1. build comparator func ptr: comp(t0,t1) = true (regardless of ccw)
			decltype(cd::greater)* comp;
			if (ccw) comp = cd::less;
			else comp = cd::greater;

			// 4-2. build count & tpFirst
			if (comp(tp, t1)) count++;
			if (comp(tm, t1)) count++;
			if (comp(tp, tm)) tpFirst = true;

			// 4-3. build tList;
			vector<double> tList;
			{
				switch (count)
				{
				case 0:
					tList.push_back(t0);
					tList.push_back(t1);
					break;
				case 1:
					tList.push_back(t0);
					if (tpFirst)
						tList.push_back(tp);
					else
						tList.push_back(tm);
					tList.push_back(t1);
					break;
				case 2:
					tList.push_back(t0);
					if (tpFirst)
					{
						tList.push_back(tp);
						tList.push_back(tm);
					}
					else
					{
						tList.push_back(tm);
						tList.push_back(tp);
					}
					tList.push_back(t1);

				}
			}

			// 5. evalueate case
			{
				//similar to 
				LOOP double end = t1, start;
				LOOP double pointSweepRadius;
				double centerDist = sqrt(arc.c.c.length2());

				LOOP bool CrossSignPositive;
				LOOP double averageT;


				// points with postive cross will be rotated with "rotation"
				// this is true since "rotation" is guaranteed to be >0 at this point.
				// should be flipped after each case label.

				switch (count)
					// switch not having 'break' is intentional, as labels above are supersets.
					// e.g.) if (t0 tp tm t1) : 3 arcs from (tm, t1) (tp, tm) (t0, tp) + 2 arcs from (sweep tm) (sweep tp)
				{
				case 2:

					// 5-1. build Arc piece from the original "arc" : (t1, tp) or (t1, tm)
					if (tpFirst) start = tm;
					else start = tp;

					averageT = (tList[3] + tList[2]) / 2;
					CrossSignPositive = cross(arc.c.c, Point(cos(averageT), sin(averageT))) > 0;

					if (!CrossSignPositive)
						ret.push_back(rotateArc(constructArc(arc.c.c, arc.c.r, start, end), degree));
					else
						ret.push_back(constructArc(arc.c.c, arc.c.r, start, end));
					(ret.end() - 1)->convex = false;

					DEBUG dbg_c++;

					end = start;
				case 1:

					// 5-3.
					if (tpFirst) start = tp;
					else start = tm;


					averageT = (tList[2] + tList[1]) / 2;
					CrossSignPositive = cross(arc.c.c, Point(cos(averageT), sin(averageT))) > 0;

					if (!CrossSignPositive)
						ret.push_back(rotateArc(constructArc(arc.c.c, arc.c.r, start, end), degree));
					else
						ret.push_back(constructArc(arc.c.c, arc.c.r, start, end));
					(ret.end() - 1)->convex = false;

					DEBUG dbg_c++;

					end = start;
				case 0:

					// 5-5.
					start = t0;

					averageT = (tList[1] + tList[0]) / 2;
					CrossSignPositive = cross(arc.c.c, Point(cos(averageT), sin(averageT))) > 0;

					if (!CrossSignPositive)
						ret.push_back(rotateArc(constructArc(arc.c.c, arc.c.r, start, end), degree));
					else
						ret.push_back(constructArc(arc.c.c, arc.c.r, start, end));
					(ret.end() - 1)->convex = false;

					DEBUG dbg_c++;

					break;
				default:
					break;
				}
			}

		}

		// debug
		if (grb_debug == 1)
		{
			// dbg_out
			{
				int v = 0, c = 0;
				for (auto& a : ret)
					if (a.convex)
						v++;
					else
						c++;
				cout << "grb_debug == 1 : conv conc : " << v << "   " << c << endl;
				cout << "construct arc called by concave case " << dbg_c << endl;

			}

			// ~dbg_out

			return ret;
		}

		// 6. trim stuffs from ret;
		auto loops = trimRotationSupersetIntoLoops(ret, robot, rotation, grb_debug);
		ret.resize(0);
		for (auto& loop : loops)
			for (auto& arc : loop)
				ret.push_back(arc);

		return ret;
	}

#pragma region trimming stuff

	// TODO LIST
	// 1. point_inside_RSV : 3-else
	// 2. TRSI section 7
	// 3. *** divArc is not computed correctly.

	struct neighbor
	{
		int idx[2]; // idx of left, right neighbor
		double dist[2];	//squared dist -> to save computation time
		bool opposite01[2];	// opp01[i] = j means that 
							//	 = (idx[i])'s j-th endpoint is connected to this arc.
							// opposite01 can be either 0 or 1.

		inline neighbor()
		{
			idx[0] = idx[1] = -1; // -1: not initialized
		}
		inline bool exists(int leftOrRight)
		{
			return idx[leftOrRight] > -1;
		}
	};

	struct arcIntersection
	{
		double interParam;	// Def: The point where arc has intersection
		bool trimForward;	// Def: If true, trim the segment from (intPoint) to (arc.x1)
		
		// Def : constructor
		arcIntersection(double interParam, bool trimForward) : interParam(interParam), trimForward(trimForward) 
		{}

		// Def : simple less<this>
		inline bool operator<(const arcIntersection& rhs) const
		{
			return this->interParam < rhs.interParam;
		}
	};

	// As arcs can be ccw, cw... sorting order of radians should be different
	struct arcIntersectionComparator
	{
		bool ccw;
		arcIntersectionComparator(bool ccw) : ccw(ccw) {}
		bool operator()(const arcIntersection& lhs, const arcIntersection& rhs) const
		{
			if (ccw)
				return lhs.interParam < rhs.interParam;
			else
				return lhs.interParam > rhs.interParam;
		}
		arcIntersectionComparator& operator=(const arcIntersectionComparator& a)
		{
			ccw = a.ccw;
			return *this;
		}
	};

	const double errorNeigh = 1e-8;
	//const double errorCricleIntersection = 1e-8;
	const double error_tRSI_point_inside_RSV = 1e-4;

	const bool _tRSI_perform_valid_input_test = true;
	const bool _tRSI_print_errors = true;
	
	void 
		_tRSI_valid_input_test(vector<CircularArc>& superset);
	void 
		_tRSI_buildNeighbor(INPUT vector<CircularArc>& superset, OUTPUT vector<neighbor>& neigh);
	bool 
		_tRSI_point_inside_RSV(Point& p, vector<CircularArc> object, double rotationRadian);
	std::tuple<Point, Point, int> 
		_tRSI_getCircleIntersecion(Circle& lhs, Circle& rhs);

	DEBUG int trsiInterestIdx = 0;

	/*
	Def:

	Param:
		superset: superset of rotational swept volume, that needs trimming.
		original: original model before rotation (used in point-inclusion-test)

	Assume: correct ccw, convex
		correct normals of arcs(a.x - a.c.c being parallel to a.n)
	Desc:
		There are many arcs in the 'superset', that may 1. some-of-it-are-boundary-but-some-not 2. completely-boundary 3. completely-not-boundary
		But we don't know which part of it forms the boundary.
		First, we divide the arcs, using all of its intersection points.
		As divided_arcs do not have intersection with other arcs, they should be fully-not-a-boundary or fully-a-boundary.
			Consider yourself as a point traveling through any arc, your is_boundary only changes when you penetrate the true-boundary.
			Removing all intersection(even those arcs that may not be true-boundary) guarantees a uniform is_boundary value in that arc.
		Before dividing arcs we should save "bool trimForward" for each intersection points.
			Which means that we should trim the forward part (forward as in, arc param traveling direction) from the intersection point.
			This can be known in the intersection step, by looking at the opponent's convex/concaveness
			This is because 1. we know the is_boundary being uniform and 2. at least one of (p +- epsilon * p') will be in the inside region (p is intersection-point, p' is tangent)
			This might not hold near the original arc's endpoints, so we pre-compute arcs which share endpoints and exclude them in this step.
		
		For a divided_arc, if one of the endpoint says (with trimForward) that this should be trimmed => trim it.
			For the original endpoints of arcs in superset
				arc.x[0] trimForward = false;
				arc.x[1] trimForward = true;
				So that arcs from endpoints don't get trimed
		Find loops with divided_arcs.	
	*/
	vector<deque<CircularArc>>
		trimRotationSupersetIntoLoops(vector<CircularArc>& superset, vector<CircularArc>& original, double rotationRadian, DEBUG int grb_debug)
	{

		// -1. check errors in input.
		if (_tRSI_perform_valid_input_test)
			_tRSI_valid_input_test(superset);

		// 0. init
		int nArcs = superset.size();


		// 0-1. compute arc param
		//	Each arcs starting angle(-pi <= t0 <= +pi), and ending angle(-3pi <= t1, <= +3pi).
		//		So that traveling from t0 to t1 is continuous, in ints cw/ccw direction.
		vector<pair<double, double>> arcParam;
		arcParam.resize(nArcs);
		{
			for (size_t i = 0; i < nArcs; i++)
			{
				// 0-1-1. atan2
				auto& a = superset[i];
				auto
					theta0 = atan2(a.n0().y(), a.n0().x()),
					theta1 = atan2(a.n1().y(), a.n1().x());

				// 0-1-2. add/sub 2*pi
				if (a.ccw)	// theta1 > theta0
					while (theta1 <= theta0)
						theta1 += PI2;
				else		// theta1 < theta0
					while (theta1 >= theta0)
						theta1 -= PI2;

				// 0-1-3. save
				arcParam[i].first  = theta0;
				arcParam[i].second = theta1;
			}
		}

		// 1. build neighborhood info
		vector<neighbor> neigh;	// holds all neighbor info
		_tRSI_buildNeighbor(superset, neigh);

		// 2. compute intersection points (for each arc)
		vector<set<arcIntersection, arcIntersectionComparator>> inter; // holds all intersection for each arc. ref as inter[arcNo][intersectionNo];
		
		{
			// 2-0. first build "inter" up to size nArcs
			arcIntersectionComparator aIC_ccw(true);
			arcIntersectionComparator aIC_cw(false);
			for (size_t i = 0; i < nArcs; i++)
				if (superset[i].ccw)
					inter.push_back(set<arcIntersection, arcIntersectionComparator>(aIC_ccw));
				else
					inter.push_back(set<arcIntersection, arcIntersectionComparator>(aIC_cw));


			LOOP Point  temp[2];
			LOOP int    nInter = 0, nCircleInter = 0;
			LOOP double t0[2], t1[2];	// tN[i] = param of point in arcN in temp[i]
			LOOP bool   valid[2];		// valid[i] = temp[i] is valid for both arcs
			LOOP bool	reverseParamOrdering[2];	// rPO[i] = false means : traversing the arc is done in this order: start -> ti[0] -> ti[1] -> end. 
													//		if rPO[i] = true: start -> ti[0] -> ti[1] -> end

			// for (each arc_pair)
			for (size_t i = 0  ; i < nArcs; i++)
			for (size_t j = i+1; j < nArcs; j++)
			{
				// if (they are not neighbors)
				if (!(neigh[i].idx[0] == j || neigh[i].idx[1] == j))
				{
					auto
						& arc0 = superset[i],
						& arc1 = superset[j];

					
					// 2-1. first get circle's intersection;
					auto
						circInter = getIntersectionOfCircles(arc0.c, arc1.c);
					temp[0] = get<0>(circInter);
					temp[1] = get<1>(circInter);
					nCircleInter  = get<2>(circInter);
					nInter = 0;

					// dbg_out
					if (i == 9 && j == 11)
					{
						cout << "Circ inter result : " << endl;
						cout << temp[0] << endl;
						cout << temp[1] << endl;
						cout << "nCircInter : " << nCircleInter << endl;
					}
					//~dbg

					// 2-2. check if circles intersection points are valid for arcs
					for (size_t k = 0; k < nCircleInter; k++)
					{
						//2-2-1. turn circle_inter_pts into each arc's param
						auto
							direction0 = temp[k] - arc0.c.c,
							direction1 = temp[k] - arc1.c.c;

						auto
							theta0 = atan2(direction0.y(), direction0.x()),
							theta1 = atan2(direction1.y(), direction1.x());

						if (arc0.ccw)
							while (theta0 <= arcParam[i].first) // <= instead of < : since theta0 = arcParam[i].first just leads to an arc with 0 length.
								theta0 += PI2;
						else
							while (theta0 >= arcParam[i].first)
								theta0 -= PI2;

						if (arc1.ccw)
							while (theta1 <= arcParam[j].first)
								theta1 += PI2;
						else
							while (theta1 >= arcParam[j].first)
								theta1 -= PI2;

						// 2-2-2. check wheter intersection points's param are inside arc's param
						bool
							valid0 = arc0.ccw ? theta0 < arcParam[i].second : theta0 > arcParam[i].second,
							valid1 = arc1.ccw ? theta1 < arcParam[j].second : theta1 > arcParam[j].second;

						// 2-2-3. save info
						// if (intersection point is valid for both arcs)
						t0[k] = theta0;
						t1[k] = theta1;
						if (valid0 && valid1)
						{
							nInter++;
							valid[k] = true;

							//// 2-2-3. swap if ... => changed imple => so that 2-3 becomes easier
							//// if(temp[0] not valid && temp[1] valid) do swap
							//if (k == 1 && nInter == 1) // kcould be 0 or 1, nInter could be 1 or 2 at this point. => only (1,1) case needs to be taken care of
							//{
							//	t0[0] = t0[1];
							//	t1[0] = t1[1];
							//	temp[0] = temp[1];
							//}
						}
						else
							valid[k] = false;

					} // end of 2-2
 
					// 2-3. find out "bool trimForward values and save it"
					//	As two arc's ccw can be 0/1 => 4 case
					switch (nCircleInter)
					{
					case 2:
						// rule of thumb :
						// basically, the arc is divided into 3 : (start, inter0), (inter0, inter1), (inter1, end)
						// if (opposite sides arcs is convex ) leave (start,  inter0) (inter1, end)
						// if (opposite sides arcs is concave) leave (inter0, inter1)

						//dbg_out
						if (i == 11 || j == 11)
						{
							if (valid[0] || valid[1])
							{
								cout << "i , j, v0, v1: " << i << "   " << j << "        " << valid[0] << "   " << valid[1] << endl;
								//arc0.c.draw();
								cout << "ccw arc0 arc1 : " << arc0.ccw << arc1.ccw << endl;
								cout << "convex arc0 arc1 : " << arc0.convex << arc1.convex << endl;
								cout << "arc0 tb te : " << t0[0] << "  " << t0[1] << endl;
								cout << "arc1 tb te : " << t1[0] << "  " << t1[1] << endl;
							}
						}
						//~dbg
						
						// 2-3-1. take care of arc0
						{
							// two intersection points divides circle into two arcs.
							// but, with only the intersection points, we don't know which one of the arcs fall inside the other circle.
							// Below code assumes that traveling in the ccw/cw direction of the arc, from tn[0] to tn[1], is inside the opposite circle.
							// But actually, that may not hold.
							// flag needed to flip.
							bool arcT0T1NotInOpposite;
							{
								double thetaAvg = (t0[0] + t0[1]) * 0.5;
								auto normal = Point(cos(thetaAvg), sin(thetaAvg));
								auto queryPoint = arc0.c.c + arc0.c.r * normal;
								if ((arc1.c.c - queryPoint).length2() <= arc1.c.r * arc1.c.r)
									arcT0T1NotInOpposite = false;
								else
									arcT0T1NotInOpposite = true;
							}

							// get reverseParamOrdering
							if (arc0.ccw)
							{
								if (t0[0] < t0[1])
									reverseParamOrdering[0] = false;
								else
									reverseParamOrdering[0] = true;
							}
							else
							{
								if (t0[0] < t0[1])
									reverseParamOrdering[0] = true;
								else
									reverseParamOrdering[0] = false;
							}
							// Note : simply rPO[0] = (t0[0] < t0[1]) ^ arc0.ccw; is possible... but not good for interpretation.

							// if (opposite is convex)
							if (arc1.convex)
							{
								if (valid[0])
								{
									if (reverseParamOrdering[0])
										inter[i].insert(arcIntersection(t0[0], false ^ arcT0T1NotInOpposite));
									else
										inter[i].insert(arcIntersection(t0[0], true ^ arcT0T1NotInOpposite));
								}

								if (valid[1])
								{
									if (reverseParamOrdering[0])
										inter[i].insert(arcIntersection(t0[1], true ^ arcT0T1NotInOpposite));
									else
										inter[i].insert(arcIntersection(t0[1], false ^ arcT0T1NotInOpposite));
								}
							}
							else // concave opponent
							{
								if (valid[0])
								{
									if (reverseParamOrdering[0])
										inter[i].insert(arcIntersection(t0[0], true ^ arcT0T1NotInOpposite));
									else
										inter[i].insert(arcIntersection(t0[0], false ^ arcT0T1NotInOpposite));
								}

								if (valid[1])
								{
									if (reverseParamOrdering[0])
										inter[i].insert(arcIntersection(t0[1], false ^ arcT0T1NotInOpposite));
									else
										inter[i].insert(arcIntersection(t0[1], true ^ arcT0T1NotInOpposite));
								}
							}
							// Note : this could also be simplified... as below
							/*
							if (valid[0])
								inter[i].insert(arcIntersection(t0[0], reverseParamOrdering[0] ^ arc1.convex));
							if (valid[1])
								inter[i].insert(arcIntersection(t0[1], !(reverseParamOrdering[0] ^ arc1.convex)))
							*/
						}

						// 2-3-2. take care of arc1 (similar to that of arc0, just change : t0 -> t1, i->j, arc0<->arc1, rPO[0] -> rPO[1],  )
						{
							bool arcT0T1NotInOpposite;
							{
								double thetaAvg = (t1[0] + t1[1]) * 0.5;
								auto normal = Point(cos(thetaAvg), sin(thetaAvg));
								auto queryPoint = arc1.c.c + arc1.c.r * normal;
								if ((arc0.c.c - queryPoint).length2() <= arc0.c.r * arc0.c.r)
									arcT0T1NotInOpposite = false;
								else
									arcT0T1NotInOpposite = true;
							}
							
							// get reverseParamOrdering
							if (arc1.ccw)
							{
								if (t1[0] < t1[1])
									reverseParamOrdering[1] = false;
								else
									reverseParamOrdering[1] = true;
							}
							else
							{
								if (t1[0] < t1[1])
									reverseParamOrdering[1] = true;
								else
									reverseParamOrdering[1] = false;
							}
							// Note : simply rPO[1] = (t1[0] < t1[1]) ^ arc1.ccw; is possible... but not good for interpretation.

							// if (opposite is convex)
							if (arc0.convex)
							{
								if (valid[0])
								{
									if (reverseParamOrdering[1])
										inter[j].insert(arcIntersection(t1[0], false ^ arcT0T1NotInOpposite));
									else
										inter[j].insert(arcIntersection(t1[0], true ^ arcT0T1NotInOpposite));
								}

								if (valid[1])
								{
									if (reverseParamOrdering[1])
										inter[j].insert(arcIntersection(t1[1], true ^ arcT0T1NotInOpposite));
									else
										inter[j].insert(arcIntersection(t1[1], false ^ arcT0T1NotInOpposite));
								}
							}
							else // concave opponent
							{
								if (valid[0])
								{
									if (reverseParamOrdering[1])
										inter[j].insert(arcIntersection(t1[0], true ^ arcT0T1NotInOpposite));
									else
										inter[j].insert(arcIntersection(t1[0], false ^ arcT0T1NotInOpposite));
								}

								if (valid[1])
								{
									if (reverseParamOrdering[1])
										inter[j].insert(arcIntersection(t1[1], false ^ arcT0T1NotInOpposite));
									else
										inter[j].insert(arcIntersection(t1[1], true ^ arcT0T1NotInOpposite));
								}
							}
							// Note : this could also be simplified... as below
							/*
							if (valid[0])
								inter[j].insert(arcIntersection(t1[0], reverseParamOrdering[1] ^ arc0.convex));
							if (valid[1])
								inter[j].insert(arcIntersection(t1[1], !(reverseParamOrdering[1] ^ arc0.convex)))
							*/
						}

						// TODO : see if above 2-3-1,2 works. If so use simpler imple.

						break;
					case 1: // when two circles share only a point, this is equivalent to the case of no_intersection
					case 0:
					default:
						break;
					}



				}
			}
		}

		// Now, inter is built.

		// 2.5. simplify inter
		// This will only reduce number of arcs in 'divArcs'... And Theoretically not change the final result.
		
		// TODO

		// 3. 1st-Trimming using arcIntersection for each point
		vector<CircularArc> divArcs; // set of arcs divided & trimmed to have no intersection. 
		{
			vector<bool> referenced(nArcs, false);
			
			// debug
			if (_tRSI_print_errors)
			{
				for (auto i : referenced)
					if (i) cerr << "ARRAY REFERENCED NOT INIT CORRECTLY" << endl;
			}

			// for (all original arcs) show different behavior for number of intersection(0, 1, 2-or-more)
			for (size_t i = 0; i < nArcs; i++)
			{
				//dbg_out : trimForward set strange case.
				if (i == trsiInterestIdx)
				{
					glLineWidth(5.0f);
					glColor3f(1, 0, 1);
					auto a = superset[i];
					a.draw();
					cout << "******trsiIdx : " << trsiInterestIdx << endl;
					cout << "is.size() : " << inter[i].size() << endl;
					cout << a.x0() << endl;
					cout << a.x1() << endl;
					for (auto& intersection : inter[i])
					{
						cout << "param : " << intersection.interParam / PI * 180 
							<< "     trimForward : " << intersection.trimForward
							<< endl;
					}
				}
				//~dbg

				// 3-1. if this arc is referenced by another arc, just skip it
				if (referenced[i]) continue;
				referenced[i] = true;

				// 3-2.take care of arcs that are between intersection points.
				auto& is = inter[i];
				if (is.size() > 1)
				{
					// Notice that iter will be sorted with ccw/cw information in mind. If original arc was ccw, is.begin() would be the element with smallest arcParam.
					//	Also trimForward's 'Forward' is equivalent to the forward direction of the set 'is'.
					LOOP auto it0 = is.begin();
					LOOP auto it1 = ++is.begin(); //bugfixed -> post-increment to pre-inc
					while (it1 != is.end())
					{
						// 3-2-1. leave segments not guaranteed to be in the inner-region-of-object.
						// if (left_intersetcion points left && right_intersection points right) = (both endpoints are not sure about trimming this segment)
						// here "points" mean that pointing to the to-be-trimmed part
						// also arcParam is said to be traveled from left-to-right in comments below.
						if ((!(it0->trimForward) && it1->trimForward))
						{
							divArcs.push_back(constructArc(superset[i], it0->interParam, it1->interParam));

							
						}


						it0++;
						it1++;
					}
				}

				// 3-3 and 3-4 take care of dividedArcs which contain original arc's endpoint
				// if (there is at least one intersection) insert arc(start, inter0), arc(inter_last, end)
				if (is.size() > 0)
				{
					// 3-3. take care of (start, intersection0)

					// 3-3-1. if (not trimmed) add it to vector
					if (is.begin()->trimForward)
						divArcs.push_back(constructArc(superset[i], arcParam[i].first, is.begin()->interParam));
					// 3-3-2. if (trimmed) trim neighbors without intersections
					else
					{
						LOOP int currentArc = i;
						LOOP int currentEnd = 0;

						// while(currentArc's endpoint(cureentEnd) has a neighbor)
						while (neigh[currentArc].exists(currentEnd))
						{
							int nextArc = neigh[currentArc].idx[currentEnd];

							// if(next arc was not processed, and it is intersetion-free)
							if (referenced[nextArc] == false && inter[nextArc].size() == 0)
							{
								referenced[nextArc] = true; //this is equivalent to trimming the whole arc.

								currentEnd = ! neigh[currentArc].opposite01[currentEnd];
								currentArc = nextArc;

							}
							else // if (next arc was not trimmed) stop propagation
								break;
						}
					}

				
					// 3-4. take care of (intersection_last, end)
					auto iterLast = is.end();
					iterLast--;
					// 3-4-1. if (not trimmed) add it to vector
					if (!(iterLast->trimForward))
						divArcs.push_back(constructArc(superset[i], iterLast->interParam, arcParam[i].second));
					// 3-4-2. if (trimmed) trim neighbors without intersections
					else
					{
						LOOP int currentArc = i;
						LOOP int currentEnd = 1;

						// while(currentArc's endpoint(cureentEnd) has a neighbor)
						while (neigh[currentArc].exists(currentEnd))
						{
							int nextArc = neigh[currentArc].idx[currentEnd];

							// if(next arc was not processed, and it is intersetion-free)
							if (referenced[nextArc] == false && inter[nextArc].size() == 0)
							{
								referenced[nextArc] = true; //this is equivalent to trimming the whole arc.

								currentEnd = ! neigh[currentArc].opposite01[currentEnd];
								currentArc = nextArc;

							}
							else // if (next arc was not trimmed) stop propagation
								break;
						}
					}
				}
				else // if (arc has no intersection with others) simply insert the whole arc => will be trimmed later
				{
					divArcs.push_back(superset[i]);
				}

			}// end: for (size_t i = 0; i < nArcs; i++)
		}// end: section 3		

		// debug
		if (grb_debug == 2)
		{
			// print inerpoints
			if (planning::keyboardflag['z'])
			for (size_t i = 0; i < inter.size(); i++)
			{
				cout << "arc param : " << arcParam[i].first;
				for (auto interp : inter[i])
				{
					double par = interp.interParam;
					auto norm = Point(cos(par), sin(par));
					auto p = superset[i].c.c + superset[i].c.r * norm;

					if (interp.trimForward)
						glColor3f(0, 0, 1);
					else
						glColor3f(0, 1, 0);

					DEBUG
					// dgb_out
					glPointSize(5);
					glBegin(GL_POINT);
					glVertex2d(p.x(), p.y());
					glEnd();
					//cout << "inter p : " << p << endl;
					cout << " " << par;
				}
				cout << " " << arcParam[i].second << endl;
			}

			vector<deque<CircularArc>> ret;
			ret.push_back(deque<CircularArc>(divArcs.begin(), divArcs.end()));
			return ret;
		}

		// 4. Recompute neighborhood info
		vector<neighbor> divNeigh;
		_tRSI_buildNeighbor(divArcs, divNeigh);

		// 5. Build candidate-loops (these lists of arcs may form a loop, but most won't)
		vector<deque<CircularArc>> loops;
		{
			auto 
				nDivArcs = divArcs.size();
			vector<bool> 
				referenced (nDivArcs, false);

			for (size_t i = 0; i < nDivArcs; i++)
			{
				// 5-1. don't re-use arcs
				if (referenced[i])
					continue;
				referenced[i] = true;

				// 5-2. loop : starts with only divArcs[i];
				deque<CircularArc> loop;
				loop.push_back(divArcs[i]);

				// 5-3 procede toward right direction and add neighbors
				{
					LOOP bool procedeNext = divNeigh[i].exists(1);
					LOOP int  arcNext	  = divNeigh[i].idx[1];

					// while (there is some arc on the right side)
					while (procedeNext)
					{
						// 5-3-1. add arc on right to this loop
						loop.push_back(divArcs[arcNext]);
						referenced[arcNext] = true;

						// 5-3-2. manage LOOP variables
						// if (arcNext has both neighbors) find next arc; else stop
						if (divNeigh[arcNext].exists(0) && divNeigh[arcNext].exists(1))
						{
							// Note that there is global/local left-and-right. We want to travel right globally. 
							//	There is no guarantee that local parameterization direction of an arc matches the global direction.
							//	Therefore we have to find out, whether the arc globally-on-the-right is idx[0](locally-left) or idx[1](locally-right)
							//	We can exploit referenced[] here.
							//		At least one of the neighbors are referenced=true
							//		=> find the one that has referenced=false
							//		=> if both nieghbors are true, stop.
							if (!referenced[divNeigh[arcNext].idx[0]])
								arcNext = divNeigh[arcNext].idx[0];
							else if (!referenced[divNeigh[arcNext].idx[1]])
								arcNext = divNeigh[arcNext].idx[1];
							else
								procedeNext = false;
						}
						else
							procedeNext = false;
					}
				}

				// 5-4 procede toward left direction and add neighbors (similar to right direction)
				// dif w/ 5-3: push_back -> push_front
				{
					LOOP bool procedeNext = divNeigh[i].exists(0);
					LOOP int  arcNext	  = divNeigh[i].idx[0];

					// while (there is some arc on the left side)
					while (procedeNext)
					{
						// 5-4-1. add arc on right to this loop
						loop.push_front(divArcs[arcNext]);
						referenced[arcNext] = true;

						// 5-4-2. manage LOOP variables
						// if (arcNext has both neighbors) find next arc; else stop
						if (divNeigh[arcNext].exists(0) && divNeigh[arcNext].exists(1))
						{
							// see 5-3-2
							if (!referenced[divNeigh[arcNext].idx[0]])
								arcNext = divNeigh[arcNext].idx[0];
							else if (!referenced[divNeigh[arcNext].idx[1]])
								arcNext = divNeigh[arcNext].idx[1];
							else
								procedeNext = false;
						}
						else
							procedeNext = false;
					}
				}

				// 5-5. push the completed loop into loops
				loops.push_back(loop);
			}

		}

		// 5.5 find some loops that have max/min x,y and exclued those in step 6 
		// lacking this step shouldn't change the final result.

		// TODO

		// 6. Trim "candidate-loop"s that have points in the inside-region
		//	For each loop pick a point.
		//  If that point is inside the inside-region of the rotational-swept-volume, trim that loop.
		//	How to know if a point is inside rot-swept-vol?
		//		Change the test from (point vs rot-swept-vol) to (arc vs original model)
		//		Rotate the point by -rotationRadian to form an arc. Test this against the original model
		//			Since a point in rot-swept-vol is a result of rotating a point inside original model by some angle inside the interval [0, rotationRadian].
		vector<bool> trimLoop(loops.size(), false);
		for (size_t i = 0, length = loops.size(); i < length; i++)
		{
			Point testPoint = loops[i][0].x0(); // This might affect sth?
			bool trimThisLoop = _tRSI_point_inside_RSV(testPoint, original, rotationRadian);

			
			if (trimThisLoop)
				trimLoop[i] = true;
			
			//debug dbg_out
			cout << trimThisLoop;
		}
		//debug
		cout << endl;

		// 7. Trim "candidate-loop"s which fail to form a loop
		for (size_t i = 0, length = loops.size(); i < length; i++)
		{
			if (trimLoop[i]) continue;
			// TODO

		}

		// 8. build return value.
		vector<deque<CircularArc>> ret;
		for (size_t i = 0, length = loops.size(); i < length; i++)
		{
			if (!trimLoop[i])
			{
				ret.push_back(loops[i]);
			}
		}
		
		////debug
		//{
		//	ret.resize(0);
		//	ret.push_back(deque<CircularArc>(divArcs.begin(), divArcs.end()));
		//}

		return ret;
	}


	/*
	Def: check whether some assumptions are valid in the input of 
	Desc:
		Doesn't correct it, just says sth is wrong.
	*/
	void _tRSI_valid_input_test(vector<CircularArc>& superset)
	{
		// 1. TEST : correct normal direction (since some cases have normal swapped)
		for (auto& a : superset)
		{
			auto
				& n0 = a.n0(),
				& n1 = a.n1(),
				n_test = (a.x0()-a.c.c) / a.c.r;

			auto
				dot0 = n_test * n0, //if input is correct, this should be bigger
				dot1 = n_test * n1;

			if (dot1 > dot0)
			{
				cerr << "_tRSI_valid_input_test : " << "TEST1 failed" << endl;
				cout << n0 << n1 << n_test << endl;
				break;
			}
		}

		// 2. ASSUMES valid (local) ccw info

		// 3. convex/concave test
	}

	/*
	Def: for each arc, find a closest arc (which should be at least closer than errorNeigh) for each endpoints
	Ret: vector of struct neighbor
	*/
	void _tRSI_buildNeighbor(INPUT vector<CircularArc>& superset, OUTPUT vector<neighbor>& neigh)
	{
		int nArcs = superset.size();
		neigh.resize(0);
		neigh.resize(nArcs);

		// for (each arc_pair)
		for (size_t i = 0  ; i < nArcs; i++)
		for (size_t j = i+1; j < nArcs; j++)
		{
			auto
				& arc0 = superset[i],
				& arc1 = superset[j];

			// for (each endpoint pair)
			for (size_t e0 = 0; e0 < 2; e0++)
			for (size_t e1 = 0; e1 < 2; e1++)
			{
				auto
					& p0 = arc0.x[e0],
					& p1 = arc1.x[e1];

				// if (two endpoints are close enough)
				if (fabs(p0.x() - p1.x()) < errorNeigh && fabs(p0.y() - p1.y()) < errorNeigh)
				{
					auto
						dist = (p0 - p1).length2();

					auto
						& nei0 = neigh[i],
						& nei1 = neigh[j];

					// 1-1. check if updatable
					if (nei0.exists(e0) && nei0.dist[e0] < dist)
						continue;
					if (nei1.exists(e1) && nei1.dist[e1] < dist)
						continue;

					// At this point, we know that both endpoints(e0, e1) are either 1.not having neighbor, 2.have a further neighbor
					// -> set this pair.

					// 1-2. clear neighbor's info pointing to our current pair
					if (nei0.exists(e0))
						neigh[nei0.idx[e0]].idx[nei0.opposite01[e0]] = -1;
					if (nei1.exists(e1))
						neigh[nei1.idx[e1]].idx[nei1.opposite01[e1]] = -1;

					// 1-3. now set nei0 & nei1, that they are connected
					nei0.idx[e0] = j;
					nei1.idx[e1] = i;
					nei0.opposite01[e0] = e1;
					nei1.opposite01[e1] = e0;
					nei0.dist[e0] = nei1.dist[e1] = dist;
				}
			}
		}
	}

	/*
	Def:
		returns true if p is strictly-inside(on obj boundary =/= inside) the inside-region(opposite of collision-free-space) of a rotationalSweepVolume
		returns false if p is on the loop it self, or outside the loop
			This is because the function is used to distinguish between 1. points that form the boundary 2. point inside the boundary.
				Points outisde the boundary actually won't(shouldn't) be an input to this function.
	Param:
		p : point to be tested
		object : object that will be rotated (to form the RSV)
		rotationRadian: rotation in radian unit.
	Assume:
		object is a single loop. (contains no extra arcs that does not form the loop)
		Among the 3 possible p:
			1. p inside RSV
			2. p on boundary of RSV
			3. p outside RSV
			only 1, 2 is input to this function.
			Since this function is called by tRSI's section 6.
			Basically this function, therefore, returns true upon case 1 and false on case 2 
	*/
	bool _tRSI_point_inside_RSV(Point& p, vector<CircularArc> object, double rotationRadian)
	{
		// 1. change the problem from (point, RSV) to (arc, original_object)
		auto O = Point(0, 0);
		auto OP_dist = sqrt(p.length2());
		auto theta0 = atan2(p.y(), p.x());
		auto theta1 = theta0 - rotationRadian;

		CircularArc unrotatedP = constructArc(O, OP_dist, theta0, theta1);

		Point
			tangent0(0, 0), // +1 * tangent at x0 of unrotatedP (used to distinguish II-i and II-ii of polar-collision-case) : direction from (endpoint) to rest part 
			tangent1(0, 0); // -1 * tangent at x1 of unrotatedP (used to distinguish II-i and II-ii of polar-collision-case) : direction from (endpoint) to rest part
		if (unrotatedP.ccw)
		{
			// rotate n0 +90 degrees
			tangent0 = Point(-unrotatedP.n0().y(), +unrotatedP.n0().x());
			// rotate n1 -90 degrees
			tangent1 = Point(+unrotatedP.n1().y(), -unrotatedP.n1().x());
		}
		else
		{
			// rotate n0 -90 degrees
			tangent0 = Point(+unrotatedP.n0().y(), -unrotatedP.n0().x());
			// rotate n1 +90 degrees
			tangent1 = Point(-unrotatedP.n1().y(), +unrotatedP.n1().x());
		}

		// There are 3 cases. (arc-object-cases)
		// I.	There is intersection between (unrotatedP, circularArcs of object)
		//	==> if intersection point is very close to unrotateP endpoints... this isn't considered a collision
		//	==> if intersection point is somewhat in the middle of unrotateP... this should be a collision.
		//	====> if intersection point is in [+eps, 1-eps] => return true; else go to determine step of II or III
		// II.	UnrotatedP is completely strictly-inside object
		//	==> return true;
		// III.	UnrotatedP is completely strictly-outside object
		//	==> unlikely to happen... such points won't be input to this function in the first place.

		// The problem is simplified using the polar coordinate.
		// unrotatedP will have a uniform r in the polar-coord.
		// considering only r = OP_dist is enough.
		//	consider only the inside-region of the object, which has r = OP_DIST
		//	The object's inside-region is now just a set of intervals (in theta space).
		//	unrotateP is just an interval as well.
		//	But notice that collision(btw arcs) will be given as a point with r = OP_DIST, which may have the inner-region in its left or right (unknown)
		// Div into cases (polar-collision-cases)
		//	I. unrotateP has collisions(r, theta_c), but not at its endpoint
		//	=> There is a point (r, theta_c) +- (0, epsilon), which is (in the inside-region) && (not the endpoint of unrotateP)
		//	==> sweeping this point with "rotationRadian" guarantees "p" being inside the inner-region
		//	===> return true; (right away)
		//	II. unrotateP has collisions(r, theta_c), at one of its endpoint
		//	=> need to check convex/concave info of colliding_arc
		//		II-i. if the rest of unrotateP is inside the inner-region of colliding_arc
		//		=> return true; (immediately)
		//		II-ii. rest of unrotateP is inside the outer-region of colliding_arc
		//		=> mark this node to be a candidate false. (but if we meet case like I, II-i after this, we immediately return true);
		//		==> return false, after seeing all other object arcs not having a collision with unrotatedP.
		//	III. there is no collision btw (unrotateP & object)
		//	=> either
		//		III-i. unrotateP is completely inside object;
		//		III-ii. unrotateP is completely outside object;
		//		WE ASSUME THAT III-ii IS NOT GIVEN AS INPUT => automatically III-i => true;
		//	IV. unrotateP has collisions at both endpoints with each endpoint having the configuration of II-ii
		//		probably an etremely rare case... but a possible worst-case-scenario
		//		whether p might be a boundary or not seems to change with
		//			1. touching circle's curvature...
		//			2. and maybe wheter there is a tangentially contacting arc?
		//		For safety, lets return false so that the loop does not get trimmed.

		// 2. Check collision against all arcs. If there is collision (not in endpoints) return true;
		bool validTheta0Collision = false; // There was collision with unrotatedP at theta0, which turned out to be that rest of the arcs weren't in the inner-region
		bool validTheta1Collision = false;
		for (size_t i = 0, length = object.size(); i < length; i++)
		{
			// 2-1. do circle collision detection
			auto& collision_arc = object[i];
			auto circleInter = getIntersectionOfCircles(unrotatedP.c, collision_arc.c);
			Point
				& inter0 = get<0>(circleInter),
				& inter1 = get<1>(circleInter);
			int nCircleInter = get<2>(circleInter);

			if (nCircleInter == 0)
				continue;
			if (nCircleInter == 1)
			{
				continue;
				// This is actually the hard case, where two arcs meets tangentially.
				// We can ignore this case and do nothing.
				//
				// First, whether the (tangent_collision_point == endpoint of unrotateP) does not matter
				//	=> note that tangent contact means that collisioin point + (0, +-eps) will have the same is_inner_region.
				//	=> if we somehow figure out the is_inner_region of the infinitely close two point, whether p is on the endpoint or not won't affect the final result. 
				//
				// case I : the rest of the part (very close to the contact point) is in the inside-region
				//	=> should return true;
				//	I-i. there are other collisions that aren't endpoints of unrotatedP = > returns true;
				//  I-ii. the only collision is at the endpoint. => the rest of unrotatedP will be eventually determined to be inside the inner-region => returns true; 
				//  I-iii. no collision at all => returns true;
				// case II : rest part close are outer-region
				//	=> can't determine return value at this point.
				//	II-i: non-endpoint-collision found => immediately returns true; => this is correct;
				//	II-ii. only-one-endpoint-collision => (similar way as I-ii) => returns false; => this is correct.
				//  II-iii. no collision => all points in unrotateP are either 1. on-the-boundary 2. in-the-outer-region => should return false => returns false => correct;
			}
			
			// at this point nCircleInter is guranteed to be 2

			// 2-2. see if circle-cd-points are valid points on both arcs
			bool
				valid0 = true,
				valid1 = true;
			// 2-2-1. check validity for unrotatedP
			{
				auto arcPtr = &unrotatedP;

				Point
					dir0 = inter0 - arcPtr->c.c,
					dir1 = inter1 - arcPtr->c.c;
				double
					thetaInter0 = atan2(dir0.y(), dir0.x()),
					thetaInter1 = atan2(dir1.y(), dir1.x());

				if (arcPtr->ccw)
				{
					while (thetaInter0 < theta0)
						thetaInter0 += PI2;
					while (thetaInter1 < theta0)
						thetaInter1 += PI2;

					if (thetaInter0 > theta1)
						valid0 = false;
					if (thetaInter1 > theta1)
						valid1 = false;
				}
				else
				{
					while (thetaInter0 > theta0)
						thetaInter0 -= PI2;
					while (thetaInter1 > theta0)
						thetaInter1 -= PI2;

					if (thetaInter0 < theta1)
						valid0 = false;
					if (thetaInter1 < theta1)
						valid1 = false;
				}
			}
			// 2-2-2. check validity for collision_arc (only difference is arcPtr initialization)
			{
				auto arcPtr = &collision_arc;

				Point
					dir0 = inter0 - arcPtr->c.c,
					dir1 = inter1 - arcPtr->c.c;
				double
					thetaInter0 = atan2(dir0.y(), dir0.x()),
					thetaInter1 = atan2(dir1.y(), dir1.x());

				if (arcPtr->ccw)
				{
					while (thetaInter0 < theta0)
						thetaInter0 += PI2;
					while (thetaInter1 < theta0)
						thetaInter1 += PI2;

					if (thetaInter0 > theta1)
						valid0 = false;
					if (thetaInter1 > theta1)
						valid1 = false;
				}
				else
				{
					while (thetaInter0 > theta0)
						thetaInter0 -= PI2;
					while (thetaInter1 > theta0)
						thetaInter1 -= PI2;

					if (thetaInter0 < theta1)
						valid0 = false;
					if (thetaInter1 < theta1)
						valid1 = false;
				}
			}

			// 2-3. if (inter0 is valid) see which polar-collision-case it falls into
			if (valid0)
			{
				// Alias for code reusability;
				Point& inter = inter0;

				// if (arc's first endpoint is in some errorbound-distance with this intersection PT)
				if ((inter - unrotatedP.x0()).length2() < error_tRSI_point_inside_RSV)
				{
					// 2-3-1. get dot product
					auto dot = (inter - collision_arc.c.c) * tangent0; // only the sign matters

					// 2-3-2. see where the rest of unrotatedP is.
					// if (dot > 0) the rest part escapes circle's interior.
					// if (dot < 0) the rest part goes penetrated into the circle.
					// dot == 0 case should not be cared as it should fall into "if (nCircleInter == 1)" part.
					bool restOfUnrotatedP_Inside_CollisionArcCircle = true;
					if (dot > 0)
						restOfUnrotatedP_Inside_CollisionArcCircle = false;

					// 2-3-3. compare above with collisionArc's convex/concave
					if (collision_arc.convex ^ restOfUnrotatedP_Inside_CollisionArcCircle)
					{
						validTheta0Collision = true;
					}
					else
					{
						return true;
					}
				}
				// else if (arc's 2nd endpoint is in some errorbound-distance with this intersection PT)
				//	change from above: x0->x1 // tangent0->1 // validTheta0Coliision -> validTheta1Collision
				else if ((inter - unrotatedP.x1()).length2() < error_tRSI_point_inside_RSV)
				{

					// 2-3-1. get dot product
					auto dot = (inter - collision_arc.c.c) * tangent1; // only the sign matters

					// 2-3-2. see where the rest of unrotatedP is.
					// if (dot > 0) the rest part escapes circle's interior.
					// if (dot < 0) the rest part goes penetrated into the circle.
					// dot == 0 case should not be cared as it should fall into "if (nCircleInter == 1)" part.
					bool restOfUnrotatedP_Inside_CollisionArcCircle = true;
					if (dot > 0)
						restOfUnrotatedP_Inside_CollisionArcCircle = false;

					// 2-3-3. compare above with collisionArc's convex/concave
					if (collision_arc.convex ^ restOfUnrotatedP_Inside_CollisionArcCircle)
					{
						validTheta1Collision = true;
					}
					else
					{
						return true;
					}
				}
				else // if(intersection point isn't at endpoint) p is not boundary.
				{
					return true;
				}


			}

			// 2-4. if (inter1 is valid) see which polar-collision-case it falls into // difference with 2-3 : just change alias "inter"
			if (valid1)
			{
				// Alias for code reusability;
				Point& inter = inter1;

				// if (arc's first endpoint is in some errorbound-distance with this intersection PT)
				if ((inter - unrotatedP.x0()).length2() < error_tRSI_point_inside_RSV)
				{
					// 2-3-1. get dot product
					auto dot = (inter - collision_arc.c.c) * tangent0; // only the sign matters

					// 2-3-2. see where the rest of unrotatedP is.
					// if (dot > 0) the rest part escapes circle's interior.
					// if (dot < 0) the rest part goes penetrated into the circle.
					// dot == 0 case should not be cared as it should fall into "if (nCircleInter == 1)" part.
					bool restOfUnrotatedP_Inside_CollisionArcCircle = true;
					if (dot > 0)
						restOfUnrotatedP_Inside_CollisionArcCircle = false;

					// 2-3-3. compare above with collisionArc's convex/concave
					if (collision_arc.convex ^ restOfUnrotatedP_Inside_CollisionArcCircle)
					{
						validTheta0Collision = true;
					}
					else
					{
						return true;
					}
				}
				// else if (arc's 2nd endpoint is in some errorbound-distance with this intersection PT)
				//	change from above: x0->x1 // tangent0->1 // validTheta0Coliision -> validTheta1Collision
				else if ((inter - unrotatedP.x1()).length2() < error_tRSI_point_inside_RSV)
				{

					// 2-3-1. get dot product
					auto dot = (inter - collision_arc.c.c) * tangent1; // only the sign matters

					// 2-3-2. see where the rest of unrotatedP is.
					// if (dot > 0) the rest part escapes circle's interior.
					// if (dot < 0) the rest part goes penetrated into the circle.
					// dot == 0 case should not be cared as it should fall into "if (nCircleInter == 1)" part.
					bool restOfUnrotatedP_Inside_CollisionArcCircle = true;
					if (dot > 0)
						restOfUnrotatedP_Inside_CollisionArcCircle = false;

					// 2-3-3. compare above with collisionArc's convex/concave
					if (collision_arc.convex ^ restOfUnrotatedP_Inside_CollisionArcCircle)
					{
						validTheta1Collision = true;
					}
					else
					{
						return true;
					}
				}
				else // if(intersection point isn't at endpoint) p is not boundary.
				{
					return true;
				}

			}

		}

		// At this point, there is no intersections at non-endpoints of unrotatedP.

		// 3. check if there was ...
		if (validTheta0Collision || validTheta1Collision)
			return false;
		else
		{

			// it is important that false is returned for real boundary (if not, boudnary will be gone)
			//	=> validTheta0/1Collision setting is important
			//	=> but for those configurations of validThetaCollision = true, it is prone to numerical errors.
			//	==> since circle-Intersection-points that are very close to endpoints... may not pass inside-arc-validation step
			//	=> increasing the interval (theta0, theta1) to (theta-eps, theta+eps) could cause other problems.
			//	==> This may cause some of those cases that returns false(endpoint-intersection) to be determined as intersection-exists-in-non-endpoint case.
			//	=> if cases (which should make validThetaCollision true) were not processed correctly due to numerical errors, they would reach here.
			//	==> As the program failed to catch intersections near the endpoint, unrotatedP is consider to have no intersections with arcs in object.
			//	===> we should distinguish between arc-object-case II and III 

			auto thetaMid = (theta0 + theta1) / 2;

			//TODO

			// dbg_out
			cout << "reached zero-inter point" << endl;

			// dbg true->false
			//return true;
			return false;
		}
	}

	/*
	Def: Get circle(not arc) intersection by : cosine law on triangle(center - center line, radius1, radius2)
		Almost similar to ms::intersection_self, but more strictly takes care of the case where number of intersecion is 1.
			=> actually no change in code... since it can take care of lhs == rhs case...
			==> but still... it is not correct when (two circles share a point) && (one circle includes the other)

	Desc:
		localX : has direction connecting two circle centers
		localY : perpendicular to localX
	What happens if we put two identical circles? => returns (NULL, NULL, 0)
	*/
	const double error_getCircleIntersection = 1e-8;
	std::tuple<Point, Point, int> _tRSI_getCircleIntersecion(Circle & lhs, Circle & rhs)
	{
		std::tuple<Point, Point, int> result;

		double d = distance(lhs.c, rhs.c);

		// if (center_dist > sum of radius)
		if (d > (lhs.r + rhs.r + error_getCircleIntersection) * (lhs.r + rhs.r + error_getCircleIntersection))
			result = std::make_tuple(NULL, NULL, 0);
		// else if (center_dist < sum of radius)
		else if (d < (lhs.r + rhs.r - error_getCircleIntersection) * (lhs.r + rhs.r - error_getCircleIntersection)) 
		{
			// if(center_dist < diff_of_rad + eps)
			if (std::sqrt(d) + std::min(lhs.r, rhs.r) < std::max(lhs.r, rhs.r) + error_getCircleIntersection)
				result = std::make_tuple(NULL, NULL, 0);
			else
			{
				Point localUnitX = (rhs.c - lhs.c).normalize();
				Point localUnitY = localUnitX.rotate();
				Point localX = localUnitX * ((d + lhs.r * lhs.r - rhs.r * rhs.r) / (2 * std::sqrt(d)));
				Point localY = localUnitY * (sqrt(lhs.r * lhs.r - localX.length()));
				result = std::make_tuple(lhs.c + localX + localY, lhs.c + localX - localY, 2);
			}
		}
		// else (center_dist == sum_of_radius)
		else
			result = std::make_tuple(lhs.projection(rhs.c), NULL, 1);

		return result;
	}

	/*
	Def:
	Given the rotation superset(set of arcs, probably boundary will form a loop), find its translation superset
	Param:
	arcs : set of arcs
	translation : assume model's center moves from (0,0) to "translation"
	notice that, in the real scene, the starting point of the robot may not actually be the origin. (This should be taken care of by the callee)
	Returned:
	Super set (of circular arcs and lineSegments) which forms the boundary of the swept-volume-by-translation.
	Again this "superset" thing is similar to that of getRotationSuperSet;
	*/
	std::pair<vector<CircularArc>, vector<lineSegment>> 
		getTranslationSuperSet(vector<CircularArc> arcs, Point translation)
	{
		// 0. returned val & aliases.
		std::pair<vector<CircularArc>, vector<lineSegment>> ret;
		auto & rArcs = ret.first;
		auto & rLines = ret.second;

		// 0-1. precompute stuff
		auto normal = Point(-(translation.y()), translation.x()).normalize();

		// 1. fill rArcs w/ original arcs & translated arcs
		for (auto &arc : arcs)
		{
			rArcs.push_back(arc);
			rArcs.push_back(translateArc(arc, translation));
		}

		// 2. line segs formed by arc's point tangent to line seg
		for (auto &arc : arcs)
		{
			if (isNormalBetween(arc.n[0], arc.n[1], normal))
			{
				auto p = arc.c.c + (arc.c.r * normal);
				auto q = p + translation;
				rLines.emplace_back(p, q);
			}

			if (isNormalBetween(arc.n[0], arc.n[1], -normal))
			{
				auto p = arc.c.c + (arc.c.r * -normal);
				auto q = p + translation;
				rLines.emplace_back(p, q);
			}
		}

		//TODO
		return ret;
	}
		
	/***********************************************************************************/
	//** 3. Testing (intersection)
	/***********************************************************************************/

	/*
	Def : 
	*/
	bool 
		testSuperset(superSet& ss, Obstacle& obs)
	{
		// simply test all possible configuration;
		for (auto& o : obs)
		{
			for (auto& i : ss.first)
				if (test_CircularArc_CircularArc(i, o)) return true;
			for (auto& i : ss.second)
				if (test_LineSegment_CircularArc(i, o)) return true;
		}

		//TODO
		return false;
	}

	/*
	Def :

	Assume:
		When the circle and lien meets tangentially, false is returned (as the purpose of this was to check penetration into inner-region)
		p1 =/= p0
	*/
	bool 
		test_LineSegment_CircularArc(lineSegment& lhs, CircularArc& rhs)
	{
		// ALIAS
		auto
			& p0 = lhs.first,
			& p1 = lhs.second;

		////dbg_out
		//cout << "Test_LS_CA : " << lhs.first << " , " << lhs.second << endl;
		//cout << rhs << endl;
		////~dbg
		
		// calculate stuff.
		double
			lineLength = sqrt((p1 - p0).length2());
		if (lineLength <= 1e-1000) return false; //error check

		Point
			tangent = (p1 - p0) / lineLength,
			normal = rotatePoint(tangent, 90.0);

		double
			lineOffset = p0 * normal,
			circleOffset = rhs.c.c * normal,
			d = fabs(lineOffset- circleOffset);

		// 1. if(center-line dist > r) return false;
		if (d >= rhs.c.r) return false;

		// 2. get line-circle intersection and check if it is inside both of (lineSeg, circularArc)
		Point intersectionCandidate[2];
		Point midIntersection = (lineOffset - circleOffset) * normal + rhs.c.c; // midPoint of two intersections;

		double l = sqrt(rhs.c.r * rhs.c.r - d * d);
		intersectionCandidate[0] = midIntersection - (l * tangent);
		intersectionCandidate[1] = midIntersection + (l * tangent);

		// 3. now check if intersectionCandidate is actually a part of lineSeg & Arc
		for (int i = 0; i < 2; i++)
		{
			Point& inter = intersectionCandidate[i];

			bool insideLS = test_LineElement_LineSegment(inter, p0, p1);
			bool insideCA = test_CircleElement_CircularArc(inter, rhs);

			if (insideLS && insideCA)
				return true;
		}
		return false;

	}

	/*
	Def : NOT DONE
	*/
	bool 
		test_CircularArc_CircularArc(CircularArc& lhs, CircularArc& rhs)
	{
		// 1. check if circle overlaps;
		auto centerDist = sqrt((lhs.c.c - rhs.c.c).length2());
		auto sumRad = lhs.c.r + rhs.c.r;
		if (centerDist > sumRad)
			return false;

		// 2. 
		
		//TODO
		return false;
	}

	///*
	//Def : 
	//	given a point x 
	//	and a line segment with endpoints p, q.
	//	If the three points are known to be collinear (x is in the line defined by p and q)
	//	Return true iff x is between p, q
	//Assume :
	//	Three points are collinear.
	//	For the case of x == p or x == q, the answer could be both true or false.
	//		This is to reduce calculation
	//		And for the purpose of this function(p and q are on voronoi edge), it is not likely to happen.
	//		And as it is floating-point arithmetic, exact-match case is trivial.
	//	Do not insert p == q case.
	//*/
	//bool
	//	test_LineElement_LineSegment(Point& x, Point& p, Point& q)
	//{
	//	//if (lineSegment not parallel to y-axis)
	//	if (p.x() != q.x())
	//	{
	//		bool flag0 = x.x() < p.x();
	//		bool flag1 = x.x() < q.x();
	//		return flag0 ^ flag1;
	//	}
	//	else
	//	{
	//		bool flag0 = x.y() < p.y();
	//		bool flag1 = x.y() <= q.y(); //just-in-case of p==q==x case;
	//		return flag0 ^ flag1;
	//	}
	//}


	/*
	Def : 
		newer implementation version of above.
		uses some errors to rule out those cases where x is almost p or q
	*/
	double EPS_test_LineElement_LineSegment = 1e-3;
	bool
		test_LineElement_LineSegment(Point& x, Point& p, Point& q)
	{

		if (q.x() < p.x())
		{
			double
				eps = (p.x() - q.x()) * EPS_test_LineElement_LineSegment;
			if (x.x() < p.x() - eps &&
				x.x() > q.x() + eps)
				return true;
			else
				return false;
		}
		else if (q.x() > p.x())
		{
			double
				eps = (q.x() - p.x()) * EPS_test_LineElement_LineSegment;
			if (x.x() < q.x() - eps &&
				x.x() > p.x() + eps)
				return true;
			else
				return false;
		}
		else // p.x == q.x
		{
			double 
				eps = fabs(q.y() - p.y()) * EPS_test_LineElement_LineSegment;

			if (x.y() < q.y() - eps &&
				x.y() > p.y() + eps)
				return true;
			if (x.y() > q.y() - eps &&
				x.y() < p.y() + eps)
				return true;

			return false;
		}
	}

	/*
	Def :
		given a point x and an arc a,
		assume that they are on a circle.
		Return true iff x is on the arc.
	Assume :
		x is on a.c
		a.c.r != 0;
	
	*/
#define ErrorCheck_test_CircleElement_CircularArc true
	bool 
		test_CircleElement_CircularArc(Point& x, CircularArc& a)
	{
		if (ErrorCheck_test_CircleElement_CircularArc)
		{
			bool
				test0 = (a.c.r == 0.0);

			if (test0)
			{
				cerr << "POTENTIAL ERROR in test_CircleElement_CircularArc" << endl;
			}
		}

		Point n = x - a.c.c;
		auto t = getParameterOfArc(a);
		double
			& theta0 = t.first,
			& theta1 = t.second,
			thetax = getParameterOfArc(a, n, theta0);

		return (theta0 < thetax) ^ (theta1 < thetax);

	}


	/***********************************************************************************
	** 4. For refinement of vornoi edges
	***********************************************************************************/

	/*
	Def : Use seperating axis test to check whether lineSegment (p,q) intersects with box;

	Desc :
	Project to 3 axis and check overlap.
		1. x
		2. y
		3. perp(q-p)
	If no overlap in any axis, return false;
	If all has overlap, return true;

	Real-time rendering 3rd ed page 745.
	*/
	bool aabb::testLineSegment(Point& p, Point& q)
	{
		// 1. make origin = AABB center;
		Point aabbCenter(xmin() + xmax(), ymin() + ymax());
		aabbCenter = aabbCenter * 0.5;
		Point c = (p + q) * 0.5 - aabbCenter;
		Point w = (q - p) * 0.5;
		Point h = Point(xmax() - xmin(), ymax() - ymin()) * 0.5;

		// 2. do test

		// if(|cx| > |wx| + hx)
		if (fabs(c.x()) > fabs(w.x()) + h.x())
			return false;

		// if(|cy| > |wy| + hy)
		if (fabs(c.y()) > fabs(w.y()) + h.y())
			return false;

		// if(|cxwy - cywx| > hx|wy| + hy|wx|)
		if (fabs(c.x() * w.y() - c.y() * w.x()) > h.x() * fabs(w.y()) + h.y() * fabs(w.x()))
			return false;

		return true;
	}



	/*
	Def : initialize "data" and build BVH for fast col-det

	Assume :
		arcs are x,y monotone.
	*/
	void 
		lineSegmentCollisionTester::initialize(VR_IN& vrin)
	{
		// 1. set data
		data = vrin.arcs;

		// 2. set boundary
		for (int i = vrin.arcs.size() - 1; i >= 0; i--)
		{
			if (vrin.isBoundaryArc(i))
				voronoiBoundary.push_back(vrin.arcs[i]);
			else
				break;
		}

		// 3. build BVH
		vector<int> indices;
		for (int i = 0; i < data.size(); i++)
			indices.push_back(i);

		recursivelyBuildBVH(rootObstacleBVH, indices);

	}

	/*
	Def : 
		The half-open-interval [it0, it1) of v_edges defines a single strip of v-edges, which will be reduced to a single line segment.
		The function returns true iff the line segment has a collision with any circular arcs in the scene(obstacles or boundary)

	Desc :
		2 cases.
		1. The line segment needs to be tested for only those arcs that formed the vornoi-edges.
			<==> the maximal touching disk at it0->v0 is divided into two areas, by the footpoint_from_center---circleCenter---the_other_footpoint. (hard to explain)
				The new line segment lies in the same area with v0-v1 in this case.
		2. The line segment needs to be tested for all of the arcs in the scene.

	Assumes :
		data contains v_edge with same order as VR_IN used to find v_edges.

	Warning :
		This function may become dangerous when :
			bifurcation point is included in the reduced voronoi edge.
		This function may become slower if :
			it0~it1 is too long
				=> test(Point& p, Point& q) instead.
		Do not plug in : the case where it0->v0 and (it-1)->v1 are both a point on obstacle and voronoi (they are both an non-g1-point of obstacle),
			The line connecting two points may be completely inside the loop
				Therefore the collision test will return "NO_COLLISION"
				But the line actually passes through the osbstacle.
			So this is an error case...
				but test(Point& p, Point& q) using general points (which has nothing to do with voronoi) is not compatible to judge complete-inclusion/exclusion in O(1) time...
			So this is left to the caller side.
	*/
	bool 
		lineSegmentCollisionTester::test(iter& it0, iter& it1)
	{
		// 0. Alias;
		Point
			& lineSegmentBegin = it0->v0,
			& lineSegmentEnd = (it1-1)->v1;
		lineSegment
			ls = make_pair(lineSegmentBegin, lineSegmentEnd);
		

		// 1. which case?
		bool isEasyCase;
		bool v0isNonG1Point;
		{

			// 1.1 Align theta for each directions.

			Point
				dir0 = data[it0->idx[0]].c.c	- lineSegmentBegin,
				dir1 = data[it0->idx[1]].c.c	- lineSegmentBegin,
				dirv = it0->v1					- lineSegmentBegin,
				dirx = lineSegmentEnd			- lineSegmentBegin;

			// what if (distance(lsBegin, closestPointOnModel) == 0) ?
			//	falsely falling into hardCase is always okay.
			//	falsely falling into easy case? => possible.
			//		can it intersect with arcs other than those that form MAT_LOOP? => possible
			//		=> hard case.

			double
				closestDist = sqrt(dir0.length2()) - data[it0->idx[0]].c.r; // be aware this might be negative due to numerical error.

			// if (v0 is non-g1-point of model)
			v0isNonG1Point = closestDist < epsNonG1PointVoronoiEdgeDistance;
			if (v0isNonG1Point)
				isEasyCase = false;
			else
			{

				double
					theta0 = atan2(dir0.y(), dir0.x()),
					theta1 = atan2(dir1.y(), dir1.x()),
					thetav = atan2(dirv.y(), dirv.x()),
					thetax = atan2(dirx.y(), dirx.x());

				while (theta1 < theta0)
					theta1 += PI2;
				while (thetav < theta0)
					thetav += PI2;
				while (thetax < theta0)
					thetax += PI2;

				// 1.2. Travel starting from theta0 in ccw-dir,
				//	Check whether thetav and thetax lies on the same side of theta1
				bool
					flag0 = thetav < theta1,
					flag1 = thetax < theta1;

				isEasyCase = !(flag0 ^ flag1);
			}
		}

		//dbg_out
		DEBUG
			cout << "isEasyCase : " << isEasyCase << endl;


		// 2. take care of easy case.
		if (isEasyCase)
		{
			// 2.1 check collision for those arcs that made v_edges
			for (auto it = it0; it != it1; it++)
			{
				if (test_LineSegment_CircularArc(ls, data[it->idx[0]]))
					return true;
				if (test_LineSegment_CircularArc(ls, data[it->idx[1]]))
					return true;
			}

			// notice that for easy case, complete-inclusion-case does not occur (as v0 is guaranteed to be outside of loop)
			return false;
		}

		// 3. hard case, test all using BVH.
		bool arcIntersectionsResult = recursivelyTestBVH_lineSegment(rootObstacleBVH, ls);
		return arcIntersectionsResult;


		// optionCheckCompleteInclusionOfLineSegment part => deprecated
		//// 3.1 having intersection with arc ==> guaranteed collision
		//if (arcIntersectionsResult)
		//	return true;

		//// if(should check complete inclusion case)
		//if (optionCheckCompleteInclusionOfLineSegment)
		//{
		//	if (!v0isNonG1Point)
		//		return false; // since v0 should be outside of model.
		//	else
		//	{

		//	}
		//}
		//else
		//{
		//	return false;
		//}
	}


	/*
	Def : 
		Returns true iff a line defined by two endpoint p and q has a collision with an object in the scene (scene is set with initialize())

	Assumes :
		At least one of p or q will be case-1(outside of objects)
			A point will fall into 3 case
				1. On the Outside of objects
				2. On the Boundary of an object 
				3. On the Inside of an objects
			The function only checks collision with the obstacle's boundary.
			If both of p and q are case-2 or case-3, the line may be inside the object but not having collision with the boundary.
				Judging such case probably will take more than O(1) time.
			Such case are assumed to be not given as an input to this function.
				=> Rather devise a function that judges if a point is inside/outside of obstacles later, and use it as a combination with this function.
	*/
	bool
		lineSegmentCollisionTester::test(Point& p, Point& q)
	{
		lineSegment ls = make_pair(p, q);
		return recursivelyTestBVH_lineSegment(rootObstacleBVH, ls);
	}

	/*
	Def : "currentNode" should be a BV which contains arcs in "arcs"
		find BV info, then do recursive calls.
	Assumes :
		x,y monotone arcs.

	*/
	void 
		lineSegmentCollisionTester::recursivelyBuildBVH(aabb& currentNode, vector<int>& arcIndices)
	{
		// 0. trivial cases
		int nArcs = arcIndices.size();

		// 0-1. error-case (not called recursively, but the case where root itself was empty)
		if (nArcs == 0)
			return; // actually don't add to child(call this func), if arcs.size() == 0

		
		// 1. find BV
		double
			xmin = +1e+300,
			xmax = -1e+300,
			ymin = +1e+300,
			ymax = -1e+300;

		double
			x0,
			x1,
			y0,
			y1;

		for (auto& arcIdx : arcIndices)
		{
			auto& arc = data[arcIdx];
			x0 = arc.x0().x();
			x1 = arc.x1().x();
			y0 = arc.x0().y();
			y1 = arc.x1().y();

			// compare & update
			if (x0 < xmin)
				xmin = x0;
			if (x0 > xmax)
				xmax = x0;

			if (x1 < xmin)
				xmin = x1;
			if (x1 > xmax)
				xmax = x1;

			if (y0 < ymin)
				ymin = y0;
			if (y0 > ymax)
				ymax = y0;

			if (y1 < ymin)
				ymin = y1;
			if (y1 > ymax)
				ymax = y1;
		}

		currentNode.xmin() = xmin;
		currentNode.xmax() = xmax;
		currentNode.ymin() = ymin;
		currentNode.ymax() = ymax;


		////dbg_out
		//if (true )//dbgBlock[0] == 185)
		//{
		//	if (find(arcIndices.begin(), arcIndices.end(), 38) != arcIndices.end())
		//	{
		//		cout << "arcIndices content : ";
		//		for (auto i : arcIndices)
		//			cout << i << " ";
		//		cout << endl;

		//		cout << "BV info : x : " << xmin << " , " << xmax << " :: y : " << ymin << ", " << ymax << endl;
		//	}
		//}
		////~dbg

		// 0-2. leaf node case;
		if (nArcs == 1)
		{
			currentNode.data() = arcIndices[0];
		////dbg_out
		//	if (xmin < -0.4 && xmax > -0.4)
		//	{
		//		cout << "such an arc is : " << arcIndices[0] << endl;
		//		cout << data[arcIndices[0]] << endl;
		//	}
		//// dbg
			return;
		}
		
		// 2. divide into childs.

		// 2-1. which child node an arc goes into? ==> compare arc.x0() with AABB center
		constexpr int maxNumberOfChildern = 4;
		vector<int> arcIndicesChild[maxNumberOfChildern];
		double
			xCenterAABB = (xmin + xmax) * 0.5,
			yCenterAABB = (ymin + ymax) * 0.5;

		for (auto& arcIdx : arcIndices)
		{
			auto& arc = data[arcIdx];
			int flag0 = (arc.x0().x() < xCenterAABB);
			int flag1 = (arc.x0().y() < yCenterAABB);
			int idx = (flag1 * 2) + flag0;
			arcIndicesChild[idx].push_back(arcIdx);
		}

		// 2-2. ERROR CHECK INFINITY LOOP (When all elements of arc goes into one child)
		int numberOfChildren = 0;
		for (int i = 0; i < maxNumberOfChildern; i++)
		{
			if (arcIndicesChild[i].size() != 0)
				numberOfChildren++;
		}

		// 2-2-1. if so, intervene.
		if (numberOfChildren == 1)
		{
			//notice that we ruled out the case when this is a leaf node.
			// in this case, just divide the array into half
			// -> since vr_in was well-ordered, this may be okay?

			// empty all
			for (auto& a : arcIndicesChild)
				a.resize(0);

			int half = nArcs / 2;

			for (int i = 0; i < half; i++)
			{
				arcIndicesChild[0].push_back(arcIndices[i]);
			}

			for (int i = half; i < nArcs; i++)
			{
				arcIndicesChild[1].push_back(arcIndices[i]);
			}

			numberOfChildren = 2;
		}

		// at this point number of children >= 2 (guaranteed)

		// 3. recursive call
		currentNode.child.resize(numberOfChildren); //pre-allocate
		int childIdx = 0;
		for (int i = 0; i < maxNumberOfChildern; i++)
		{
			if (arcIndicesChild[i].size() == 0)
				continue;
			
			auto& nextNode = currentNode.child[childIdx];
			childIdx++;

			recursivelyBuildBVH(nextNode, arcIndicesChild[i]);
		}

	}


	/*
	Def : test whether a lineSegment has a collision with arcs, using BVH
	returns true iff soem arc has collision with ls
	*/
	bool lineSegmentCollisionTester::recursivelyTestBVH_lineSegment(aabb& currentNode, lineSegment& ls)
	{
		// 1. leaf case.
		if (currentNode.isLeafNode())
		{
			if (currentNode.testLineSegment(ls.first, ls.second))
			{
				auto
					& arc = data[currentNode.data()];

				return cd::test_LineSegment_CircularArc(ls, arc);
			}

			return false;
		}

		// 2. test current BV
		// if(collision with BV)
		if (currentNode.testLineSegment(ls.first, ls.second))
		{
			for (auto& child : currentNode.child)
			{
				// 2-1. if there is any collision find, return true
				if (recursivelyTestBVH_lineSegment(child, ls))
					return true;
			}
			return false;
		}

		// if(no collision with BV)
		return false;
	}

	///*
	//Def : 
	//given a line segment defined by two point(p, q)
	//return true iff (the line segment has collision with any object in the scene)

	//*/
	//bool vornoiCollisionTest(Point& p, Point& q, lineSegmentCollisionTester& tester, vector<v_edge>)



} //end of namespace cd.