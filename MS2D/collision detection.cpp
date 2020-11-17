#include "collision detection.hpp"
using namespace std;


namespace cd
{
	const double PI			= 3.14159265358979323846264;
	const double PI2		= 2 * 3.14159265358979323846264;
	const double PI_half	= 0.5 * 3.14159265358979323846264;
	const double RAD_TO_DEG = 180 / PI;
	const double DEG_TO_RAD = PI / 180;

	decltype(planning::isNormalBetween)* isNormalBetween = planning::isNormalBetween;

//deprecated 
//#define TrigInput(rotation) ((rotation) * DEG_TO_RAD) // currently rotation = degrees, but maybe for the future
//	// inside each function, everything is considered to be in "radian" not "degree".
//	// input param of this header will be of any unit, with changing the macro above

	// just some tags with no affect to code
#define INPUT
#define OUTPUT
#define LOOP


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
		getRotationBoundary(Robot& robotToBeRotated, double degree)
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

					//dbg_out
					cout << pointSweepRadius << endl;

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

			// for now just add all that can be.
			ret.push_back(arc);
			ret.push_back(rotateArc(arc, degree));

			auto r0 = sqrt(arc.x0().length2());
			auto theta0 = atan2(arc.x0().y(), arc.x0().x());
			ret.push_back(constructArc(origin, r0, theta0, theta0 + rotation));
			auto r1 = sqrt(arc.x1().length2());
			auto theta1 = atan2(arc.x1().y(), arc.x1().x());
			ret.push_back(constructArc(origin, r1, theta1, theta1 + rotation));
		}

		return ret;
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
				if (testCircularArc(i, o)) return true;
			for (auto& i : ss.second)
				if (testLineSegment(i, o)) return true;
		}
	}

	/*
	Def :
	*/
	bool 
		testLineSegment(lineSegment& lhs, CircularArc& rhs)
	{

	}

	/*
	Def :
	*/
	bool 
		testCircularArc(CircularArc& lhs, CircularArc& rhs)
	{
		// 1. check if circle overlaps;
		auto centerDist = sqrt((lhs.c.c - rhs.c.c).length2());
		auto sumRad = lhs.c.r + rhs.c.r;
		if (centerDist > sumRad)
			return false;

		// 2. 
	}
}