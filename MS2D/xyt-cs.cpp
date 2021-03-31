/*****************************************************************************************************
** Compute 3dimensional configuration space of a floor robot(2d translation and rotation)			**
** Primitives of the 3d cspace will be surfaces														**
**																									**
******************************************************************************************************/

#include "xyt-cs.hpp"

/*****************************************************************************************************
**																									**
**											csSurface												**
**																									**
******************************************************************************************************/

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
	_normals.resize(J + 1);
	for (auto& temp : _normals)
		temp.resize(I + 1);

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

			// 2-3. get normal by cross producting the thing;
			xy = _center1.rotate(theta + PI_half);
			xyt grad_theta = xyt(xy.x(), xy.y(), 1);
			xyt grad_phi = xyt(_rad * (-sin(phi)), _rad * cos(phi), 0);
			xyt norm = grad_phi.cross(grad_theta);

			////debug 
			//norm.x() = -norm.x();
			//norm.y() = -norm.y();
			//norm.t() = -norm.t();

			_normals[j][i] = norm.normalize();
		}

	}

}

/*
Def : get the triangles of the surface, while qualifying : -pi < \theta < +pi
	The intention : to make the drawing set needed for rendering (As the theta loops after -pi,+pi)
Desc :
	1. check which of the 3 regions theta is in
		1. [-3pi,-pi) 2.[-pi,pi) 3.[pi, 3pi)
	2. check vertice positions
		2.1 if all 3 vert in same region => simply offset theta-coordinate
		2.2 else slice the triangles(now the slices reside only in a region) => offset
Assumption:
	None new.
	Two convoluted arcs(input to csSurf) both < 180 still holds.
*/
vector<triNormal>* csSurf::drawingSet()
{
	// 0. check validity of function call.
	if (!_flagTrianglesFound)
	{
		tessellation();
	}

	// 1. alias/allocation stuff
	auto ret = new vector<triNormal>;
	auto& vtn = *ret;

	vector<vector<int>> affiliation;
	affiliation.resize(_tess.size());
	for (auto& aff : affiliation)
		aff.resize(_tess[0].size());

	// 2. calc which area each vertex is affiliated with.
	for (size_t i = 0; i < affiliation.size(); i++)
	{
		for (size_t j = 0; j < affiliation[i].size(); j++)
		{
			auto& theta = _tess[i][j].t();
			if (theta < -PI)
				affiliation[i][j] = -1;
			else if (theta > PI)
				affiliation[i][j] = +1;
			else
				affiliation[i][j] = 0; // not needed. Just for better code.
		}
	}

	// 3. evaluate triangles
	// i, j : quad mesh idx.
	//	k : tri index in a quad(0 or 1)
	for (size_t i = 0; i < affiliation.size() - 1; i++)
	for (size_t j = 0; j < affiliation[i].size() - 1; j++)
	for (size_t k = 0; k < 2; k++)
	{
		auto& a0 = affiliation[i + k][j + k];
		auto& a1 = affiliation[i + 0][j + 1];
		auto& a2 = affiliation[i + 1][j + 0];

		// 3-1. case : all are in same area
		if (a0 == a1 && a1 == a2)
		{
			auto v0 = _tess[i + k][j + k];
			auto v1 = _tess[i + 0][j + 1];
			auto v2 = _tess[i + 1][j + 0];
			
			// 3-1-1. check a0 and offset
			if (a0 == -1)
			{
				v0.t() += PI2;
				v1.t() += PI2;
				v2.t() += PI2;
			}
			else if (a0 == +1)
			{
				v0.t() -= PI2;
				v1.t() -= PI2;
				v2.t() -= PI2;
			}

			auto& n0 = _normals[i + k][j + k];
			auto& n1 = _normals[i + 0][j + 1];
			auto& n2 = _normals[i + 1][j + 0];

			if(k==1)
				vtn.emplace_back(v0, n0, v1, n1, v2, n2);
			else
				vtn.emplace_back(v0, n0, v2, n2, v1, n1); // since we draw with GL_TRIANGLES (not fan), we have to manage ccw for back/front face decision step of GL
		}

		// 3-2. case : one vertex is in diff area;
		// notice that in our configuration of the problem(with above assumptions), the triangles can exist up to two areas. (not three or more)
		else
		{
			// 3-2-1. find out which idx is the one in diff area.
			int diffIdx;
			int eqIdx0, eqIdx1;
			{
				if (a0 == a1)
				{
					diffIdx = 2;
					eqIdx0 = 0;
					eqIdx1 = 1;
				}
				else if (a1 == a2)
				{
					diffIdx = 0;
					eqIdx0 = 1;
					eqIdx1 = 2;
				}
				else
				{
					diffIdx = 1;
					eqIdx0 = 2;
					eqIdx1 = 0;
				}
			}

			// 3-2-2. find surface : t = theta_k;
			int a[] = { a0, a1, a2 };
			double theta_k = -PI;
			if (a[diffIdx] == 1 || a[eqIdx0] == 1)
				theta_k = PI;

			// 3-2-3. find intersections
			//			inter0 = line(diff, eq0) vs z = theta_k
			//			inter1 = line(diff, eq1) vs z = theta_k
			// alias
			xyt* v[] = { &_tess[i + k][j + k],	  &_tess[i + 0][j + 1],	   &_tess[i + 1][j + 0] };
			xyt* n[] = { &_normals[i + k][j + k], &_normals[i + 0][j + 1], &_normals[i + 1][j + 0] };
			// lerp
			auto inter0 = (v[diffIdx]->t() - theta_k) * v[eqIdx0][0] + (theta_k - v[eqIdx0]->t()) * v[diffIdx][0];
			inter0 = inter0 / (v[diffIdx]->t() - v[eqIdx0]->t());
			auto inter1 = (v[diffIdx]->t() - theta_k) * v[eqIdx1][0] + (theta_k - v[eqIdx1]->t()) * v[diffIdx][0];
			inter1 = inter1 / (v[diffIdx]->t() - v[eqIdx1]->t());

			auto normal0 = (v[diffIdx]->t() - theta_k) * n[eqIdx0][0] + (theta_k - v[eqIdx0]->t()) * n[diffIdx][0];
			normal0 = normal0 / (v[diffIdx]->t() - v[eqIdx0]->t()); // this is needed, else normal direction +- will change
			normal0 = normal0.normalize();
			auto normal1 = (v[diffIdx]->t() - theta_k) * n[eqIdx1][0] + (theta_k - v[eqIdx1]->t()) * n[diffIdx][0];
			normal1 = normal1 / (v[diffIdx]->t() - v[eqIdx1]->t()); // this is needed, else normal direction +- will change
			normal1 = normal1.normalize();

			// 3-2-4. should triangle with diff be offset?
			//	or the eqs be offset??
			// also, find offset amount
			int offsetDiff = true;
			if (a[diffIdx] == 0)
				offsetDiff = false;

			xyt o; // offset
			o.x() = o.y() = 0.0;
			if (theta_k == PI)
				o.t() = -PI2;
			else
				o.t() = PI2;

			// 3-2-5. Three tri: (need to offset theta)
			//			diff  , inter0, inter1
			//			eqIdx0, eqIdx1, inter0
			//			inter0, eqIdx1, inter1 
			// intsert all
			if (k == 1)
			{
				if (offsetDiff)
				{
					vtn.emplace_back(v[diffIdx][0] + o, n[diffIdx][0], inter0 + o, normal0, inter1 + o, normal1);
					//vtn.emplace_back(v[diffIdx][0] + o,	n[diffIdx][0],	inter1 + o, normal1, inter0 + o, normal0);
				}
				else
				{
					vtn.emplace_back(v[diffIdx][0], n[diffIdx][0], inter0, normal0, inter1, normal1);
					//vtn.emplace_back(v[diffIdx][0], n[diffIdx][0], inter1, normal1, inter0, normal0);
				}

				if (offsetDiff)
				{
					vtn.emplace_back(v[eqIdx0][0], n[eqIdx0][0], v[eqIdx1][0], n[eqIdx1][0], inter0, normal0);
					vtn.emplace_back(inter0, normal0, v[eqIdx1][0], n[eqIdx1][0], inter1, normal1);
				}
				else
				{
					vtn.emplace_back(v[eqIdx0][0] + o, n[eqIdx0][0], v[eqIdx1][0] + o, n[eqIdx1][0], inter0 + o, normal0);
					vtn.emplace_back(inter0 + o, normal0, v[eqIdx1][0] + o, n[eqIdx1][0], inter1 + o, normal1);
				}
			}
			else
			{
				if (offsetDiff)
				{
					vtn.emplace_back(v[diffIdx][0] + o, n[diffIdx][0], inter1 + o, normal1, inter0 + o, normal0);
					//vtn.emplace_back(v[diffIdx][0] + o,	n[diffIdx][0],	inter1 + o, normal1, inter0 + o, normal0);
				}
				else
				{
					vtn.emplace_back(v[diffIdx][0], n[diffIdx][0], inter1, normal1, inter0, normal0);
					//vtn.emplace_back(v[diffIdx][0], n[diffIdx][0], inter1, normal1, inter0, normal0);
				}

				if (offsetDiff)
				{
					vtn.emplace_back(v[eqIdx0][0], n[eqIdx0][0], inter0, normal0, v[eqIdx1][0], n[eqIdx1][0]);
					vtn.emplace_back(inter0, normal0, inter1, normal1, v[eqIdx1][0], n[eqIdx1][0]);
				}
				else
				{
					vtn.emplace_back(v[eqIdx0][0] + o, n[eqIdx0][0], inter0 + o, normal0, v[eqIdx1][0] + o, n[eqIdx1][0]);
					vtn.emplace_back(inter0 + o, normal0, inter1 + o, normal1, v[eqIdx1][0] + o, n[eqIdx1][0]);
				}
			}
		}

	}

	return ret;



}



/*****************************************************************************************************
**																									**
**										configSpaceCalculator										**
**																									**
******************************************************************************************************/

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
instead of "throw", simply "cerr + return"

Stuffs could be added anytime.
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
Def : Do the actual calculation.
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