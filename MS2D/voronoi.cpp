#include "voronoi.hpp"
#include <filesystem>
#include "collision detection.hpp"
#include <iomanip>

namespace planning
{


	int dbgmode = 0;
	dbg int piececnt = 0;
	dbg int cur_depth = -1;
	dbg int bifurcnt = 0;
	dbg int case2from = 0; // only true when evaluzating size=2 case, that is made from a size = 3 case

	int drawBifurCircle = false;
	int drawVoronoiSingleBranch = true;
	int drawMinkowski = true;
	int forwardTime = false;
	int drawBoundary = true;
	int drawTransition = false;

	std::vector<ms::Circle> circlesToDraw;

	bool keyboardflag[256];
	bool keyboardflag_last[256];

	// Error/Epsilon values used.
	double rfbTerminationEps = 1e-18; //originally 1e-18
	double rfbTerminationEps2 = 2.5e-3; // origirnally 2.5e-3;
	const double RVCA_EndpointError = 1e-8;

	vector<ms::CircularArc> voronoiBoundary;

	constexpr bool flag_vmqa = true; //Voronoi-Multi-Quadrant-Arcs.
	constexpr bool use_simplifyVCA = true;
	constexpr bool flag_findMTD_concave_check = true;
	constexpr bool flag_findMTD_nonG1_divide_case = true;

	namespace output_to_file
	{
		// tag to distinguish variables that should be configured before calling start() end()
#define namespaceConfigurable 

		namespaceConfigurable int m0 = 0, m1 = 7;
		namespaceConfigurable double zoom = 2.0, tx = 0, ty = 0;
		namespaceConfigurable int width = 1024, height = 1024;
		namespaceConfigurable int clustersPicked = 2;
		
		int curF; // change m0 & m1 in cpp file to change output configuration
		
		bool flag; //indicates that info should be gathered

		// int number_of_obstacles; 
		vector<vector<int>> objSize;		// objSize[Model_approx's number][obj number] //initialized in ms::init;
		vector<CircularArc> boundary;
		vector<CircularArc> robot;			// robot's c-arc		// init in start()
		vector<vector<CircularArc>> obj;	// obj[objNo][arcNo];	// init in start()
		vector<vector<vector<CircularArc>>> ms_obj; // ms_obj[slice][objNo][arcNo];		// built at display call back
		vector<vector<v_edge>> v_edges; // v_edges[slice][i] = line seg		// built at rfb
		vector<vector<bifur_point>> bifur_points;			// b_pts[slice][i]					// built at rfb

		vector<VR_IN> vrIn;					// vrIn[slice]	// built in display callback // TODO redundant with ms_obj; (ms_obj doesn't have boundary though)

		//set flags, start gathering stuff (should be called at t2 ==0); 
		void start(/*double z, int w, int h*/)
		{
			
			robot.resize(0);
			obj.resize(0);
			ms_obj.resize(0);
			v_edges.resize(0);
			bifur_points.resize(0);
			vrIn.resize(0);

			/*
			zoom = z;
			width = w;
			height = h;
			*/

			// build boundary;
			{
				double r = 2;
				CircularArc a(Point(0, 0), r, Point(+1, +0), Point(+0, +1));
				CircularArc b(Point(0, 0), r, Point(+0, +1), Point(-1, +0));
				CircularArc c(Point(0, 0), r, Point(-1, +0), Point(+0, -1));
				CircularArc d(Point(0, 0), r, Point(+0, -1), Point(+1, +0));
				boundary.push_back(a);
				boundary.push_back(b);
				boundary.push_back(c);
				boundary.push_back(d);
			}
			
			// build robot
			for (auto& as : Models_Approx[m0])
			{
				for (auto& arc : as.Arcs)
					robot.push_back(arc);
			}

			// build obj
			auto& arcSize = objSize[m1];
			auto numberOfObjects = arcSize.size();
			obj.resize(numberOfObjects);
			size_t j = 0;
			for (size_t i = 0; i < numberOfObjects; i++)
			{
				int bound = arcSize[i];
				for (; j < Models_Approx[m1].size(); j++)
				{
					auto& as = Models_Approx[m1][j];
					for (auto& arcs : as.Arcs)
					{
						obj[i].push_back(arcs);
					}
					if (obj[i].size() >= bound)
						break;
				}
			}

			//resize
			ms_obj.resize(360);
			v_edges.resize(360);
			bifur_points.resize(360);
			vrIn.resize(360);

			flag = true;
		}
		void end()
		{
			// 1. set things before outputing file
			flag = false;

			std::ofstream fout("exchange.txt");

			auto writeArc = [&](CircularArc & c)	//write arc to file
			{
				fout << c.c.c.P[0] << " " << c.c.c.P[1] << " " << c.c.r << " " << atan2(c.n[0].P[1], c.n[0].P[0]) << " " << atan2(c.n[1].P[1], c.n[1].P[0]) << " " << c.ccw << endl;
			};
			auto pixel2world = [&](int x, int y)	//given pixel coord, change it to world coord
			{
				auto xx = (double(x)+0.5) / width;
				auto yy = (double(y)+0.5) / height;
				auto xxx = 2 * zoom*xx - zoom + tx;
				auto yyy = 2 * zoom*yy - zoom + ty;
				return Point(xxx, yyy);
			};
			auto arcPointDist = [&](Point& p, CircularArc& c, double t) // function dedicated to use after getClosestArcParam
			{
				if (t >= 1.0) return sqrt((p - c.x[1]).length());
				else if (t <= 0)return sqrt((p - c.x[0]).length());
				else return fabs(sqrt((p - c.c.c).length()) - c.c.r);
			};
			auto arcNoToMSObjNo = [&](VR_IN & vrIn, int arcNo) // given arcNo in the whole arc array, find its chunk(msObj) and its local idx
			{
				int accum = 0;
				for (size_t i = 0, length = vrIn.arcsPerLoop.size(); i < length; i++)
				{
					if (arcNo < vrIn.arcsPerLoop[i])
					{
						return make_pair(i, arcNo);
					}
					else
					{
						arcNo -= vrIn.arcsPerLoop[i];
					}
				}
			};

			auto imagesData  = new unsigned char*[NUMBER_OF_SLICES];
			auto imagesColor = new unsigned char*[NUMBER_OF_SLICES];
			for (size_t i = 0, length = NUMBER_OF_SLICES; i < length; i++)
			{
				imagesData[i]  = new unsigned char[4 * height*width];
				imagesColor[i] = new unsigned char[4 * height*width];
			}
			stbi_flip_vertically_on_write(1);

			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			gluOrtho2D(-zoom + tx, zoom + tx, -zoom + ty, zoom + ty);
			glViewport(0, 0, width, height);
			glClearColor(0, 0, 0, 1);

			coneVoronoi cv;

			// 2. start writing stuff
			// 2-1.
			fout << zoom << endl;
			fout << width << " " << height << endl;

			fout << "robot" << endl;
			fout << robot.size() << endl;
			for (auto & arc : robot)
				writeArc(arc);

			fout << "boundary" << endl;
			fout << boundary.size() << endl;
			for (auto & arc : boundary)
				writeArc(arc);

			fout << "obstacle" << endl;
			fout << obj.size() << endl;
			for (auto & o : obj)
			{
				fout << o.size() << endl;
				for (auto &arc : o)
					writeArc(arc);
			}

			// 2-2.
			for (int i_fake = 1; i_fake <= NUMBER_OF_SLICES; i_fake++)
			{
				int i = i_fake % NUMBER_OF_SLICES;	// this is idx to arrays containing data
				int sliceNo = i_fake - 1;			// this is the actual sliceNo
				// this is because theta=0:i=1, theta=1:i=2, ..., theta=358:i=359, theta=359:i=0....

				fout << "slice " << sliceNo << endl;
				//fout << ms_obj[i].size() << endl;
				//for (size_t j = 0, length = ms_obj[i].size(); j < length; j++)
				//{
				//	fout << "mink " << j << endl;
				//	fout << " ?" << endl; // TODO Later
				//	fout << ms_obj[i][j].size() << endl;
				//	for (auto &a : ms_obj[i][j])
				//		writeArc(a);
				//}
				fout << vrIn[i].arcsPerLoop.size() - 1 << endl;
				
				// 2-2-1.
				int k_end = 0;
				for (size_t j = 0, length = vrIn[i].arcsPerLoop.size() - 1 /*last one is outer boundary*/; j < length; j++)
				{
						fout << "mink " << j << endl;
						//fout << " ?" << endl; // TODO Later
						fout << vrIn[i].arcsPerLoop[j] << endl;
						int k = k_end;
						k_end += vrIn[i].arcsPerLoop[j];
						for (; k < k_end; k++)
						{
							writeArc(vrIn[i].arcs[k]);
						}
				}

				// 2-2-2. render
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				cv.colorType = 0;
				cv.drawVoronoi(vrIn[i]);
				glBegin(GL_LINES);
				glColor3f(1, 0, 1);
				for (auto v : v_edges[i]) {
					glVertex3d(v.v0.P[0], v.v0.P[1], 0.1);
					glVertex3d(v.v1.P[0], v.v1.P[1], 0.1);
				}
				glEnd();
				glBegin(GL_POINTS);
				glColor3f(1, 1, 0);
				for (auto b : bifur_points[i])
					glVertex3d(b.p.P[0], b.p.P[1], 0.2);
				glEnd();
				glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, imagesColor[i]);

				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				cv.colorType = 1;
				cv.drawVoronoi(vrIn[i]);
				glBegin(GL_LINES);
				int j = 0;
				for (auto v : v_edges[i]) {
					glColor3ub(3, (unsigned char)(j / 256), (unsigned char)(j % 256));
					glVertex3d(v.v0.P[0], v.v0.P[1], 0.1);
					glVertex3d(v.v1.P[0], v.v1.P[1], 0.1);
					j++;
				}
				glEnd();
				glBegin(GL_POINTS);
				j = 0;
				for (auto b : bifur_points[i])
				{
					glColor3ub(4, (unsigned char)(j / 256), (unsigned char)(j % 256));
					glVertex3d(b.p.P[0], b.p.P[1], 0.2);
					j++;
				}
				glEnd();
				glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, imagesData[i]);
				stbi_write_png((string("./images/color") + to_string(sliceNo) + string(".png")).c_str(), width, height, 4, imagesColor[i], 4 * width);
				stbi_write_png((string("./images/data")  + to_string(sliceNo) + string(".png")).c_str(), width, height, 4, imagesData[i],  4 * width);

				// 2-2-3.
				fout << "pixel" << endl;
				for (size_t yi = 0; yi < height; yi++)
					for (size_t xi = 0; xi < width; xi++)
					{
						auto ptr = imagesData[i] + (4 * (yi*width + xi));
						auto r = ptr[0];
						auto g = ptr[1];
						auto b = ptr[2];
						auto p = pixel2world(xi, yi);

						//	Red			Green			Blue			Description
						//	0			?				?				Error ? (clear color)
						//	1			arcNo / 256		arcNo % 256		R component 1 denotes : pixel is ouside of object, pixel's closest point = arcNo
						//	2			loopNo							R = 2 : pixel is inside loop
						//	3			vLineNo / 256	vLineNo % 256	R = 3 : voronoi edge
						//	4			bifPtNo / 256	bifPtNo % 256	R = 4 : on bifur Pt

						switch (r)
						{
						case 0:
							//Error case: error(numerical) caused some points to be not colored.
							fout << 0 << " " << -1 << endl;
							break;
						case 1:
						{
							fout << 1 << endl;
							int arcNo = int(g) * 256 + b;
							double t = getClosestArcParameter(p, vrIn[i].arcs[arcNo]);
							auto dist = arcPointDist(p, vrIn[i].arcs[arcNo], t);
							auto objNoAndArcNo = arcNoToMSObjNo(vrIn[i], arcNo);
							fout << dist << " " << objNoAndArcNo.first << " " << objNoAndArcNo.second << " " << t << endl;
							break;
						}
						case 3:
						{
							fout << 2 << endl;
							int vedgeNo = int(g) * 256 + b;
							auto& ve = v_edges[i][vedgeNo];
							for (size_t j = 0; j < 2; j++)
							{
								int arcNo = ve.idx[j];
								double t = getClosestArcParameter(p, vrIn[i].arcs[arcNo]);
								auto dist = arcPointDist(p, vrIn[i].arcs[arcNo], t);
								auto objNoAndArcNo = arcNoToMSObjNo(vrIn[i], arcNo);
								fout << dist << " " << objNoAndArcNo.first << " " << objNoAndArcNo.second << " " << t << endl;
							}

							break;
						}
						case 4:
						{
							fout << 3 << " " << endl;
							int bpNo = int(g) * 256 + b;
							auto bp = bifur_points[i][bpNo];
							for (size_t j = 0; j < 3; j++)
							{
								int arcNo = bp.idx[j];
								double t = getClosestArcParameter(p, vrIn[i].arcs[arcNo]);
								auto dist = arcPointDist(p, vrIn[i].arcs[arcNo], t);
								auto objNoAndArcNo = arcNoToMSObjNo(vrIn[i], arcNo);
								fout << dist << " " << objNoAndArcNo.first << " " << objNoAndArcNo.second << " " << t << endl;
							}
							break;
						}
						case 2:
						default:
							fout << 0 << " " << 0 << endl;
							break;
						}
					}

				fout << "voronoi" << endl;
				fout << v_edges[i].size() << endl;
				for (auto& ve : v_edges[i])
				{
					auto temp0 = arcNoToMSObjNo(vrIn[i], ve.idx[0]);
					auto temp1 = arcNoToMSObjNo(vrIn[i], ve.idx[1]);

					fout
						<< ve.v0.P[0]
						<< " " << ve.v0.P[1]
						<< " " << ve.v1.P[0]
						<< " " << ve.v1.P[1]
						<< " " << temp0.first
						<< " " << temp0.second
						<< " " << temp1.first
						<< " " << temp1.second
						<< endl;
				}
			}

			// 2-3.
			//fout << "connectivity" << endl; // TODO

			// 3. closing part
			fout.close();
		}
	}


#pragma region math functions

	/* Def: compute a, b, c (from ax+by+c=0) of a line which passes 'p' and has direction 'v'
	*/
	struct line
	{
		double a, b, c;
	};
	line getLineEquationFromPointVector(Point INPUT p, Point INPUT v)
	{
		line l;
		l.a = -v.P[1];
		l.b = v.P[0];
		l.c = v.P[1] * p.P[0] - v.P[0] * p.P[1];
		return l;
	}
	double det2(double a, double b, double c, double d)
	{
		return a*d - b*c;
	}
	Point getLineIntersectionPoint(line l0, line l1)
	{
		Point p0; //return

		double
			a0 = det2(l0.b, l0.c, l1.b, l1.c),
			b0 = det2(l0.c, l0.a, l1.c, l1.a),
			c0 = det2(l0.a, l0.b, l1.a, l1.b);
		p0 = Point(a0 / c0, b0 / c0);
		if (fabs(c0) < 1e-300) //two lines are parallel or identical
			p0.P[0] = NAN;

		if (PRINT_ERRORS)
		{
			if (fabs(c0) < 1e-300)
			{
				if (fabs(a0) > 1e-8 || fabs(b0) > 1e-8)
					cerr << "ERROR : in getLineIntersection, two lines might be parallel " << endl;
			}
		}

		return p0;
	}


#pragma endregion

#pragma region class functions
	/* Def: less<poc>
	*/
	bool pointOnCurve::operator< (const pointOnCurve & r) const
	{
		if (this->c < r.c) return true;
		else if (this->c > r.c) return false;
		else
		{
			if (this->t < r.t) return true;
			else return false;
		}
	}
	
	/* (NOT DONE) Def: draws given conic
	*/
	void conic::draw()
	{
		double dt = 0.01;
		int n = floor((t1 - t0) / dt);
	}

	/* (NOT DONE) Def: get conic from two circularArcs which share a disk at the end.
	*/
	conic::conic(CircularArc &a, CircularArc &b)
	{
		// 1. get beginning, end points of conic
		Point p0, p1;
		{
			// 1-1. find lines.
			line
				l0 = getLineEquationFromPointVector(a.x[0], a.n[0]),
				l1 = getLineEquationFromPointVector(a.x[1], a.n[1]),
				l2 = getLineEquationFromPointVector(b.x[0], b.n[0]),
				l3 = getLineEquationFromPointVector(b.x[1], b.n[1]);

			// 1-2. find end points
			// 1-2-1. l0 & l3
			{
				double
					a0 = det2(l0.b, l0.c, l3.b, l3.c),
					b0 = det2(l0.c, l0.a, l3.c, l3.a),
					c0 = det2(l0.a, l0.b, l3.a, l3.b);
				if (fabs(c0) < 1e-8) //two lines are parallel or identical
					p0 = (a.x[0] + b.x[1]) * 0.5;
				else
					p0 = Point(a0 / c0, b0 / c0);

				if (PRINT_ERRORS)
				{
					if (fabs(c0) < 1e-8)
					{
						if (fabs(a0) > 1e-8 || fabs(b0) > 1e-8)
							cerr << "ERROR : two lines might be parallel " << endl;
					}
				}
			}
			// 1-2-2. l1 & l2
			{
				double
					a0 = det2(l1.b, l1.c, l2.b, l2.c),
					b0 = det2(l1.c, l1.a, l2.c, l2.a),
					c0 = det2(l1.a, l1.b, l2.a, l2.b);
				if (fabs(c0) < 1e-8) //two lines are parallel or identical
					p1 = (a.x[1] + b.x[0]) * 0.5;
				else
					p1 = Point(a0 / c0, b0 / c0);

				if (PRINT_ERRORS)
				{
					if (fabs(c0) < 1e-8)
					{
						if (fabs(a0) > 1e-8 || fabs(b0) > 1e-8)
							cerr << "ERROR : two lines might be parallel " << endl;
					}
				}
			}
		}

		// 2. check whether p0 is inside circle.

	}
	
	Point conic::operator()(double t)
	{
		//TODO
		return Point();
	}
/*

	const static double coneVoronoi::PI = 3.14159265358979323846264;
	const static double coneVoronoi::PI2 = 2 * 3.14159265358979323846264;
	const static double coneVoronoi::PI_half = 0.5 * 3.14159265358979323846264;*/
	
#define cvColorType unsigned char
#define cvColorFunction glColor3ubv

	/*
	Assume : 
		Right region (from tangent's view) is inside object, while left = free, clear space
		arc exists in only one quadrant
	*/
	void coneVoronoi::drawArcToCone(INPUT CircularArc& c, INPUT void* colors)
	{
		//cout << "hi\n";
		auto colorInside = (cvColorType *)colors;
		auto colorOutside = colorInside + 3;
		//glColor3usv(colors);

		// 1. build lists of theta
		vector<double> thetas;
		{
			double theta0;
			double theta1;
			if (c.n[0].P[1] * c.n[1].P[1] < 0)
			{	//take care of theta oscilating btw -pi & +pi
				if (fabs(c.n[0].P[1]) > fabs(c.n[1].P[1]))
				{
					theta0 = atan2(c.n[0].P[1], c.n[0].P[0]);
					theta1 = atan2(-c.n[1].P[1], c.n[1].P[0]);
				}
				else
				{
					theta0 = atan2(-c.n[0].P[1], c.n[0].P[0]);
					theta1 = atan2(c.n[1].P[1], c.n[1].P[0]);
				}
			}
			else if (c.n[0].P[1] * c.n[1].P[1] < 1e-300)
			{
				if (fabs(c.n[0].P[1]) > fabs(c.n[1].P[1]))
				{
					auto sign = signbit(c.n[0].P[1]) ? -1.0e-300 : 1.0e-300;
					theta0 = atan2(c.n[0].P[1], c.n[0].P[0]);
					theta1 = atan2(sign, c.n[1].P[0]);
				}
				else
				{
					auto sign = signbit(c.n[1].P[1]) ? -1.0e-300 : 1.0e-300;
					theta0 = atan2(sign, c.n[0].P[0]);
					theta1 = atan2(c.n[1].P[1], c.n[1].P[0]);
				}
			}
			else
			{
				theta0 = atan2(c.n[0].P[1], c.n[0].P[0]);
				theta1 = atan2(c.n[1].P[1], c.n[1].P[0]);
			}
			
			if (theta0 > theta1) swap(theta0, theta1);

			if (PRINT_ERRORS)
			{
				if (theta1 - theta0 > 1.6)
					cerr << "ERROR : in drawArcToCone 1 " << endl;
			}

			//dbg cout << theta0 << " " << theta1 << endl;

			auto theta = theta0;
			while (theta < theta1)
			{
				thetas.push_back(theta);
				theta += dtheta;
			}
			thetas.push_back(theta1);
		}

		// 2. build vertices
		double v0[3];
		vector<double> v1, v2; // size should be 3n
		{
			// 2-1.
			v0[0] = c.c.c.P[0];
			v0[1] = c.c.c.P[1];
			v0[2] = (coneVoronoiDepthBehindSign c.c.r) / coneRad;

			// 2-2. 2-3.
			double z = coneRad - c.c.r;
			if (z < 0) cerr << "ERROR : coneRad was set too small" << endl;
			for (auto t : thetas)
			{
				auto nx = cos(t);
				auto ny = sin(t);

				v1.push_back(v0[0] + c.c.r * nx);
				v1.push_back(v0[1] + c.c.r * ny);
				v1.push_back(.0);

				v2.push_back(v0[0] + coneRad * nx);
				v2.push_back(v0[1] + coneRad * ny);
				v2.push_back((coneVoronoiDepthBehindSign z) / coneRad);
			}
		}

		// 3. draw
		// 

		// 3-1. draw sharp part
		if (c.ccw)
			cvColorFunction(colorOutside);
		else
			cvColorFunction(colorInside);
		glBegin(GL_TRIANGLE_FAN);
		glVertex3dv(v0);
		auto p = &(v1[0]);
		for (size_t i = 0, length = v1.size(); i < length; i+=3)
		{
			glVertex3dv(p+i);
		}
		glEnd();

		// 3-2. draw tunc-cone part
		if (c.ccw)
			cvColorFunction(colorInside);
		else
			cvColorFunction(colorOutside);
		glBegin(GL_TRIANGLE_STRIP);
		auto p1 = &(v1[0]);
		auto p2 = &(v2[0]);
		for (size_t i = 0, length = v1.size(); i < length; i += 3)
		{
			glVertex3dv(p1 + i);
			glVertex3dv(p2 + i);
		}
		glEnd();

	}

	/*

		Assume: glColor is already called
	*/
	void coneVoronoi::drawCone(INPUT Point p, INPUT double theta0, INPUT double theta1, INPUT void* color)
	{
		cvColorFunction((cvColorType *)color);
		glBegin(GL_TRIANGLE_FAN);
		glVertex3d(p.P[0], p.P[1], coneVoronoiDepthBehindSign 1e-100);
		for (double t = theta0 - thetaOffset; t < theta1 + thetaOffset; t+=dtheta)
		{
			glVertex3d(p.P[0] + coneRad * cos(t), p.P[1] + coneRad * sin(t), coneVoronoiDepthBehindSign 1.0); // z is actually conerad/conerad = 1.0
		}
		glVertex3d(p.P[0] + coneRad * cos(theta1), p.P[1] + coneRad * sin(theta1), coneVoronoiDepthBehindSign 1.0); // z is actually conerad/conerad = 1.0
		glEnd();
	}

	void coneVoronoi::drawVoronoi(INPUT VR_IN & v)
	{
		auto usm = UCHAR_MAX;
		int i_end = 0;

		switch (colorType)
		{
		case 1: /* case 1 : use diff colors btw arcs*/
			/*
			each color component is unsigned byte
			Red			Green		Blue		Description
			0			?			?			Error?(clear color)
			1			arcNo/256	arcNo%256	R component 1 denotes : pixel is ouside of object, pixel's closest point = arcNo
			2			loopNo					R = 2 : pixel is inside loop
			3			vLineNo/256	vLineNo%256	R = 3 : voronoi edge
			4			bifPtNo/256	bifPtNo%256	R = 4 : on bifur Pt

			in this function, only r = 1, r = 2 is taken care of.
			*/

			for (size_t loop = 0, length = v.arcsPerLoop.size(); loop < length; loop++)
			{

				cvColorType colors[6];
				colors[0] = 2;
				colors[1] = loop;
				colors[2] = 0;

				int i = i_end;
				i_end += v.arcsPerLoop[loop];
				//dbg if (loop != length - 1) continue;
				while (i < i_end)
				{
					// 1.0 set color
					colors[3] = 1;
					colors[4] = cvColorType(i / 256);
					colors[5] = cvColorType(i % 256);

					// 1-1. draw cone seg (w/ color as abov)

					drawArcToCone(v.arcs[i], (void*)colors);

					// 1-2. for non-g1 draw cone at t = 0 of that arc;
					{
						auto l = v.left[i];
						if (fabs(v.arcs[i].n[0] * v.arcs[l].n[1]) < 1 - 1e-3)
						{
							auto nl = (v.arcs[l].ccw ? -1.0 : 1.0) * v.arcs[l].n[1];
							auto nr = (v.arcs[i].ccw ? -1.0 : 1.0) * v.arcs[i].n[0];

							if ((nl ^ nr) < 0.0) // = there is empty space outside
							{
								auto theta0 = atan2(nl.P[1], nl.P[0]);
								auto theta1 = atan2(nr.P[1], nr.P[0]);

								if (theta1 < theta0) theta1 += PI2;
								if (PRINT_ERRORS && theta1 - theta0 > PI)
								{
									cerr << "ERROR : in drawCone, theta not right" << endl;
								}
								drawCone(v.arcs[i].x[0], theta0, theta1, colors + 3);
							}
							else
							{
								auto theta0 = atan2(nr.P[1], nr.P[0]);
								auto theta1 = atan2(nl.P[1], nl.P[0]);
								if (theta1 < theta0) theta1 += PI2;
								if (PRINT_ERRORS && theta1 - theta0 > PI)
								{
									cerr << "ERROR : in drawCone, theta not right" << endl;
								}
								drawCone(v.arcs[i].x[0], theta0, theta1, colors);
							}
						}
					}
					i++;
				}
			}



			break;
		case 0: /* case 0 : use same colors for a loop, but diff btw in/out */
		default:
			for (size_t loop = 0, length = v.arcsPerLoop.size(); loop < length; loop++)
			{
				cvColorType colors[6];
				colors[0] = colors[3] = cvColorType(usm * loop / length);
				colors[1] = colors[5] = cvColorType(usm) - colors[0];
				colors[2] = colors[4] = 0;

				int i = i_end;
				i_end += v.arcsPerLoop[loop];
				//dbg if (loop != length - 1) continue;
				while (i < i_end )
				{
					drawArcToCone(v.arcs[i], (void*)colors);

					// TODO
					// for non-g1 draw cone at t = 0 of that arc;
					{
						auto l = v.left[i];
						if (fabs(v.arcs[i].n[0] * v.arcs[l].n[1]) < 1 - 1e-3)
						{
							auto nl = (v.arcs[l].ccw ? -1.0 : 1.0) * v.arcs[l].n[1];
							auto nr = (v.arcs[i].ccw ? -1.0 : 1.0) * v.arcs[i].n[0];

							if ((nl ^ nr) < 0.0) // = there is empty space outside
							{
								auto theta0 = atan2(nl.P[1], nl.P[0]);
								auto theta1 = atan2(nr.P[1], nr.P[0]);

								if (theta1 < theta0) theta1 += PI2;
								if(PRINT_ERRORS && theta1 - theta0 > PI)
								{
									cerr << "ERROR : in drawCone, theta not right" << endl;
								}
								drawCone(v.arcs[i].x[0], theta0, theta1, colors + 3);
							}
							else
							{
								auto theta0 = atan2(nr.P[1], nr.P[0]);
								auto theta1 = atan2(nl.P[1], nl.P[0]);
								if (theta1 < theta0) theta1 += PI2;
								if (PRINT_ERRORS && theta1 - theta0 > PI)
								{
									cerr << "ERROR : in drawCone, theta not right" << endl;
								}
								drawCone(v.arcs[i].x[0], theta0, theta1, colors);
							}
						}
					}
					i++;
				}
			}

			break;
		}
	}
#pragma endregion

#pragma region 1
	/* Def: flips an arc cw <=> ccw
	note that 'bool boundary' is not set
	*/
	CircularArc flipArc(CircularArc& in)
	{
		/*
		members of CircularArc are as below;
		Circle	c;
		Point	x[2];
		Point	n[2];
		bool	ccw;
		bool	boundary;
		*/

		CircularArc returned;
		returned.c = in.c;
		returned.x[0] = in.x[1];
		returned.x[1] = in.x[0];
		returned.n[0] = in.n[1];
		returned.n[1] = in.n[0];
		returned.ccw = !in.ccw;
		returned.globalccw = !in.globalccw;

		return returned;

	}

	/* checks if a float or double is almost zero
	*/
	template<class f>
	bool isZero(f a)
	{
		return (fabs(a) < 1.0e-4);
	}

	void drawCircle(Circle& c)
	{
		glBegin(GL_LINE_STRIP);
		for (size_t i = 0, length = CRES; i <= length; i++)
		{
			auto t = 6.28 * float(i) / CRES;
			glVertex2f(c.c.P[0] + c.r * cos(t), c.c.P[1] + c.r * sin(t));
		}
		glEnd();
	}
#pragma endregion

	/* Def: change output of MS2D, so that it could be used as input of voronoi In
		Assume: Arcs in arcsplines are well ordered, but arcsplines aren't well ordered. ->tested
		Assume: cw/ccw of each arcs are well set.
		Assume: we only need to flip arcsplines, not reorder them. ->tested.
	*/
	void convertMsOutput_Clockwise(deque<ArcSpline>& INPUT in, vector<CircularArc>& OUTPUT returned)
	{
		// 0. check if the loop from MS output is globally ccw or cw (the order of acrSplines is cw or ccw)
		// rather, loop finding was done CW/CCw in minkowskiSum()

		// 0-1. statistically build loopCCW (since there are someimes 1 ccw that is wrong & it might be first discovered)
		bool loopCCW;
		{
			int count = 0;
			int sum = 0;
			bool lastGlobalCCW = in[0].Arcs[0].globalccw;
			// 0-1-1. if two succeeding arcsplines share a ccw, compare end/begin & vote ccw/cw
			for (size_t i = 1; i < in.size(); i++)
			{
				bool globalCCW = in[i].Arcs[0].globalccw;
				if (lastGlobalCCW == globalCCW)
				{
					auto EndOfLastArc = (in[i - 1].Arcs.end() - 1)->x[1];
					auto BegOfThisArc = in[i].Arcs[0].x[0];
					if (EndOfLastArc == BegOfThisArc)
					{
						sum += globalCCW;
						count++;
					}
					else
					{
						sum += !globalCCW;
						count++;
					}


					////debug
					//using namespace ms;
					//if (ms::t0 == 5 && ms::t1 == 2 && ms::t2 == 62)
					//{
					//	cout << EndOfLastArc << endl;
					//	cout << BegOfThisArc << endl;
					//	cout << (EndOfLastArc == BegOfThisArc) << endl;
					//	cout << "LOOP CCW = " << loopCCW << endl;
					//}

					////~debug

					////break;
				}
				else
					lastGlobalCCW = globalCCW;
			}

			// 0-1-2. take care of Error cases (this might happen when loop is very short)
			//	-> simply recount count & sum
			if (count == 0)
			{
				cout << "ERR : convertMsOutput_Clockwise's count = 0 at" << t0 << ", " << t1 << ", " << t2 << endl;
				//This means that all odd/even Arcsplines each share a same ccw -> just flip them all

				// flip all
				for (size_t i = 0; i < in.size(); i+=2)
				{
					auto& as = in[i];
					as.ccw = !as.ccw;
					auto& arcs = as.Arcs;
					reverse(arcs.begin(), arcs.end());
					for (auto& i : arcs)
					{
						i = flipArc(i);
					}
				}

				// redo counting
				//count = 0;
				sum = 0;
				bool lastGlobalCCW = in[0].Arcs[0].globalccw;
				for (size_t i = 1; i < in.size(); i++)
				{
					bool globalCCW = in[i].Arcs[0].globalccw;
					if (lastGlobalCCW == globalCCW)
					{
						auto EndOfLastArc = (in[i - 1].Arcs.end() - 1)->x[1];
						auto BegOfThisArc = in[i].Arcs[0].x[0];
						if (EndOfLastArc == BegOfThisArc)
						{
							sum += globalCCW;
							count++;
						}
						else
						{
							sum += !globalCCW;
							count++;
						}
					}
					else
						lastGlobalCCW = globalCCW;
				}


			}

			// 0-1-3. get loopCCW
			double poss = double(sum) / count;
			if (poss > 0.5) loopCCW = true;
			else loopCCW = false;

			/*
			RESULT...

			fake func
			minkowski_id
			minkowski_id
			minkowski_id
			minkowski_id
			minkowski_id
			LOOP CCW = 0
			minkowski_id
			minkowski_id
			ERROR : sth is not cont. @ 1 with model info 7 2 26
			x values at i = 1   -0.199407, -0.188159
			ERROR : sth is not cont. @ 9 with model info 7 2 26
			x values at i = 9   -0.200642, -0.201132
			ERROR : sth is not cont. @ 13 with model info 7 2 26
			x values at i = 13   -0.179916, -0.183561
			ERROR : sth is not cont. @ 17 with model info 7 2 26
			x values at i = 17   -0.17462, -0.174276
			minkowski_id
			ERROR : sth is not cont. @ 2285 with model info 7 7 1
			x values at i = 2285   0.954926, 0.955126
			
			7 2 26 : This seems to be because count = 0;
			7 7 1  : This has error of loop begin/end of ~ 1e-4... doesn'y seem like prob of ordering?

			So we will stick to this imple.

			*/
		}
		
		// not used anymore
		//{
		//	// use this : https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
		//  // result : doesn't seem to work right.
		//
		//	const int sampleSize = in.size();
		//	int stride = in.size() / sampleSize;
		//
		//	double accumulated = 0;
		//	for (size_t i = 0; i < sampleSize - 1; i++)
		//	{
		//		auto a = in[stride*(i + 0)].Arcs[0].x[0];
		//		auto b = in[stride*(i + 1)].Arcs[0].x[0];
		//		accumulated += (b.P[0] - a.P[0]) * (b.P[1] + a.P[1]);
		//	}
		//	auto a = in[stride*(sampleSize - 1)].Arcs[0].x[0];
		//	auto b = in[0].Arcs[0].x[0];
		//	accumulated += (b.P[0] - a.P[0]) * (b.P[1] + a.P[1]);
		//
		//	if (accumulated < 0)
		//		loopCCW = true;
		//	else
		//		loopCCW = false;
		//}

		//debug
		//using namespace ms;
		//if (ms::t0 == 5 && ms::t1 == 2 && ms::t2 == 62)
		//{
		//	cout << "LOOP CCW = " << loopCCW << endl;
		//}
		//
		//~debug


		// 0-2. flip order of ArcSplines if finding loop was done CCW
		if (loopCCW)
		{
			reverse(in.begin(), in.end());
		}

		// 1. flip most of it to clockwise with globalCCW info (saved while building CCW
		for (int i1 = 0; i1 < in.size(); i1++)
		{
			auto& i = in[i1];
			bool flip = i.Arcs[0].globalccw; //flip if globalccw // assumes Arcs in arcspline share same ccw


			if (flip)
			{
				auto it  = i.Arcs.rbegin();
				auto end = i.Arcs.rend();
				int i2 = 0;

				while (it != end)
				{
					returned.push_back(flipArc(*it));
					it++; i2++;
				}
			}
			else
			{
				auto it  = i.Arcs.begin();
				auto end = i.Arcs.end();
				int i2 = 0;

				while (it != end)
				{
					returned.push_back(*it);
					it++; i2++;
				}
			}
		}

		//// 2. reverse some of arcs left (not in order)
		//// this is because there are 6?(of 700 arc) left un-ordered
		//// TODO : currently doesn't understand why this happens... a mere guess that this happens when the arc is almost line seg? -> check later...
		//int searchingEndIter = false;
		//auto reverseBegin = returned.begin(); 
		//
		//auto it = returned.begin();
		//auto end = returned.end();
		//end--; // doesn't check last element
		//while (it != end)
		//{
		//	if (it->x[1] == (it + 1)->x[0])
		//	{}
		//	else
		//	{
		//		if (searchingEndIter)
		//		{
		//			auto reverseEnd = it + 1;
		//			for (auto i = reverseBegin; i != reverseEnd; i++)
		//			{
		//				*i = flipArc(*i);
		//			}
		//			reverse(reverseBegin, reverseEnd);
		//
		//			searchingEndIter = !searchingEndIter;
		//		}
		//		else
		//		{
		//			reverseBegin = it + 1;
		//
		//			searchingEndIter = !searchingEndIter;
		//		}
		//	}
		//
		//	it++;
		//}
		//
		//// if it ends loop while finding endIter of reverse... just use .end();
		//if (searchingEndIter)
		//{
		//	auto reverseEnd = returned.end();
		//	for (auto i = reverseBegin; i != reverseEnd; i++)
		//	{
		//		*i = flipArc(*i);
		//	}
		//	reverse(reverseBegin, reverseEnd);
		//}

		// 3. last ArcSpline might be redundant
		int firstArcSplineSize = in[0].Arcs.size();
		auto redundantCandidate = returned.end() - firstArcSplineSize;
		if (firstArcSplineSize != 0 && redundantCandidate->x[0] == returned[0].x[0]) // only check the first point of arcspline... to be extremely cautious, we should probably check all elements... but Since the input should be a loop, this is okay?
		{
			returned.erase(redundantCandidate, returned.end());
		}


		//debug		see connected
		//cout << "Done converting" << endl;
		
		int rs = returned.size();
		//cout << "Number of Arcs at mod1 = " << ms::ModelInfo_CurrentModel.first << ", mod2 = " << ms::ModelInfo_CurrentModel.second << ", degree = " << ms::ModelInfo_CurrentFrame << " : " << rs << endl;
		static int count = 0;
		bool err = false;
		for (int i = 0; i < returned.size(); i++)
		{
			int next = (i + 1) % rs;
			auto diff = returned[i].x[1] - returned[next].x[0];
			auto xeql = isZero(diff.P[0]);
			auto yeql = isZero(diff.P[1]);
		
		
			if (!(xeql && yeql))
			{
				cout << "ERROR : sth is not cont. @ " << i << " with model info " << ms::ModelInfo_CurrentModel.first << " " << ms::ModelInfo_CurrentModel.second << " " << ms::ModelInfo_CurrentFrame << endl;
				cout << "x values at i = " << i << "   " << returned[i].x[0].P[0] << ", " << returned[i].x[1].P[0] << " with x,y diff " << diff.P[0] << ", " << diff.P[1]  << endl;
				err = true;
			}
			//cout << i << "   " << returned[i].x[1].P[0] - returned[next].x[0].P[0] << " , " << returned[i].x[1].P[1] - returned[next].x[0].P[1] << endl;
		
		}
		//if (!err) cout << "No error in " << count++ << endl;

		//
		///*for (int i = 309; i <= 315; i++)
		//{
		//	auto& a = returned[i];
		//	cout << i << " " <<
		//		"ccw : " << a.ccw <<
		//		", rad : " << a.c.r <<
		//		", cen : " << a.c.c << endl;
		//}*/
		////~debug
	}

	/*
	Def:
		reduce g0 arcs which have the same center, rad into one.
	Assume:
		vector of CA are somewhat-well-ordered
			For those arcs that are G0-continuous, arcs[i].x1() == arcs[1+1].x0() holds (will be tested with some error)
	Desc:
		Same arcs divided into multiples could cause some topological errors in the vor-diag(to be specific, findMTD)
	Date:
		2021-0420: made
	*/
	double _H_SVCA_PT_ERROR = 1e-12;
	double _H_SVCA_RAD_ERROR = 1e-6;
	void simplifyVCA(vector<CircularArc>& INPUT in, vector<CircularArc>& OUTPUT out)
	{
		// Alias
		auto& err0 = _H_SVCA_PT_ERROR;
		auto& err1 = _H_SVCA_RAD_ERROR;

		// 1. compare neighboring arcs and then merge them if they sould be.
		int length = in.size();
		// for (all input)
		for (int i = 0; i < length; /*i = iUpdate*/)
		{
			int iUpdate = i + 1; // next i value. need initialization for i = length -1;
			// iUpdate could be optimized to j.

			auto arc = in[i];

			// for (following arcs)
			for (int j = i + 1; j < length; j++)
			{
				// 1-1. check whether merging is possible between (arc & in[j])
				if ((arc.c.r - in[j].c.r) < err1 &&				// rad check
					(arc.c.c - in[j].c.c).length2() < err0 &&	// center check
					(arc.x1() - in[j].x0()).length2() < err0)	// g0 check
				{
					////dbg_out
					//cout << "~~~~~~~~~~~Merged :" << endl << arc << endl << in[j] << endl;

					// 1-1-1. merge arc;
					arc.x1() = in[j].x1();
					arc.n1() = in[j].n1();
					iUpdate = j+1;
				}
				else
				{
					// 1-1-2. if merging is not possible, the next i value should be current j; Also, stop adding to "arc"
					break;
				}
			}

			// at this point, "arc" is merged.
			out.push_back(arc);
			i = iUpdate;
		}

		// 2. The last and first arc could be merged.
		if (out.size() > 2)
		{
			auto& a0 = out.front();
			auto& a1 = out.back();

			if ((a1.c.r -  a0.c.r) < err1 &&			// rad check
				(a1.c.c -  a0.c.c).length2() < err0 &&	// center check
				(a1.x1() - a0.x0()).length2() < err0)	// g0 check
			{
				a0.x0() = a1.x0();
				a0.n0() = a1.n0();
				out.pop_back();
			}
		}
	}

#pragma region 2

	constexpr double _pi = 3.1415926535897932384;
	constexpr double _2pi = 3.14159265358979323846 * 2.0;
	constexpr double _pi2 = 3.14159265358979323846 / 2.0;

	/* Def : rotate +90 degrees
	*/
	Point rotate_p90(Point p)
	{
		return Point(-p.P[1], p.P[0]);
	}
	
	/* Def : rotate -90 degrees
	*/
	Point rotate_m90(Point p)
	{
		return Point(p.P[1], -p.P[0]);
	}

	/* Def : given 3 normals n0, n1, n. test if n is between n0 and n1 (consider the smaller angle n0 and n1 forms)
	   Assume : n0, n1's angle difference is < 90 (may not be needed)
				all vec normalized
	strict version (see errorAllowedVersion for comparsion)
	*/
	bool isNormalBetween(Point n0, Point n1, Point n)
	{
		auto n0n = n0 ^ n;
		auto nn1 = n  ^ n1;
		
		//old imple
		auto n0n1 = n0 ^ n1;
		if (n0n == 0 || nn1 == 0) return true;

		auto a = (n0n1) >= 0;
		auto b = (n0n)  >= 0;
		auto c = (nn1)  >= 0;

		if (a == b && a == c) return true;
		else return false;
		
		//
		//// new imple
		//if (n0n >= -1e-6 && nn1 >= -1e-6) return true;
		//if (n0n <=  1e-6 && nn1 <=  1e-6) return true;
		//return false;
		
	}

	/*
	Def:
		Similar to the func with same name, But does not restrict delta_theta
	Date:
		2021-0421: made
	*/
	bool isNormalBetween(CircularArc& arc, Point n)
	{
		double
			t0 = arc.atan0(),
			t1 = arc.atan1(),
			tn = atan2(n.y(), n.x());

		if (arc.ccw)
		{
			while (t1 < t0)
				t1 += PI2;
			while (tn < t0)
				tn += PI2;

			if (tn < t1)
				return true;
			else
				return false;
		}
		else
		{
			while (t1 > t0)
				t1 -= PI2;
			while (tn > t0)
				tn -= PI2;

			if (tn > t1)
				return true;
			else
				return false;
		}
	}

	bool isNoramlBetweenArc(CircularArc& arc, Point& n)
	{
		// due to some prob with funciton name overloading?
		return isNormalBetween(arc, n);
	}


	/*
	Def : 
		assume that a circular arc is parameterized, find that parameter of nt (btw [0,1])
	Assume: 
		the function is called with t values which are [0,1], since we are interested in arc only.
			3 inputs need to be normalized.
			input arc is segmented (x,y-ex)
	*/
	double arcNormalToParameter(Point n0, Point n1, Point nt)
	{
		auto n0nt = n0*nt;
		auto n0n1 = n0*n1;
		auto numerator   = acos(n0nt);
		auto denominator = acos(n0n1);
		if (n0nt >= 1.0) numerator = 0.0; //preventing nan from acos output
		if (denominator < 1e-7) numerator = 0;	// when arc is too smal...
		auto t = numerator / denominator;
		if (n0n1 >= 1.0) t = 0;  //preventing nan from acos output

		if (PRINT_ERRORS && ERR_ARC_NORMAL_TO_PARAM)
		{
			if (denominator > _pi2 + 1e-10  || numerator > _pi2 + 1e-10 || t < -1e-6 || t > 1+1e-6)
			{
				cerr << "ERROR : arcNormalToParameter " << denominator << " " << numerator << endl;
				cerr << "      : n0, n1, nt " << n0 << " " << n1 << " " << nt << endl;

			}
		}

		return t;
	}

	/*
	Def:
		Similar to the func with same name, But does not restrict delta_theta
	Date:
		2021-0421: made
	*/
	double arcNormalToParameter(CircularArc& arc, Point n)
	{
		double
			t0 = arc.atan0(),
			t1 = arc.atan1(),
			tn = atan2(n.y(), n.x());

		if (arc.ccw)
		{
			while (t1 < t0)
				t1 += PI2;
			while (tn < t0)
				tn += PI2;
		}
		else
		{
			while (t1 > t0)
				t1 -= PI2;
			while (tn > t0)
				tn -= PI2;
		}

		double
			numer = tn - t0,
			denom = t1 - t0;
		if (denom == 0.0)
			return 1.0;
		return (numer / denom);
	}

	/* Def: given an arc A with A(0) = center + rad * n0 && A(1) = center + rad * n1, find nt such that A(t) = center + rad * nt. 
	+) built from getPointGeometry
	*/
	Point arcNormalInterpolation(Point n0, Point n1, double t)
	{
		if (t == 0) return n0;
		if (t == 1) return n1;

		// 1. get thetas
		auto theta0 = atan2(n0.P[1], n0.P[0]);
		auto theta1 = atan2(n1.P[1], n1.P[0]);

		// 2. take care of those cases where +pi and -pi might be swapped.
		if (theta1 - theta0 > _pi)
		{
			theta1 = theta1 - _2pi; // note that theta won't be btw [-pi, +pi]
		}
		if (theta0 - theta1 > _pi)
		{
			theta1 = theta1 + _2pi;
		}

		// 3. Error check
		if (PRINT_ERRORS && ERR_ARC_NORMAL_INTER_DTHETA)
		{
			auto dtheta = fabs(theta0 - theta1) / _pi * 180.0;
			if (dtheta > 90.0) 
			{
				auto 
					t0 = theta0 / _pi * 180.0,
					t1 = theta1 / _pi * 180.0;
				cerr << "ERROR : in arcNormalInterp, |dtheta| > 90, piececnt, depth, case2from " << piececnt << " " << cur_depth << " " << case2from << endl
					<<  "      : n0, n1, t, theta0, theta1 " << n0 << n1 << " " << t << " " << t0 << " " << t1 << endl;
			};
		}

		// 4. return
		auto theta = (t)* theta1 + (1 - t) * theta0;
		Point ret (cos(theta), sin(theta));


		if (PRINT_ERRORS)
		{
			auto q = isNormalBetween(n0, n1, ret) || piececnt == 204;
			if (!q) cerr << "ERROR : in arcNormalInterp, returned normal not between input normals" << endl << "      : n0 n1 t result piececnt depth" << n0 << n1 << " " << t << " " << ret << " " <<piececnt << " " << cur_depth << endl;
		}

		return ret;
	}

	/*
	Def: 
		similar to the above version, but this one can support arcs that are not bounded to a quadrant

	Date:
		2021-0421 : made
	*/
	Point arcNormalInterpolation(CircularArc& arc, double t)
	{
		// 0. Take care of trivial cases
		if (t == 0) return arc.n0();
		if (t == 1) return arc.n1();

		// 1. get Theta0, Theta1
		double theta0, theta1;
		{
			theta0 = atan2(arc.n0().y(), arc.n0().x());
			theta1 = atan2(arc.n1().y(), arc.n1().x());
			if (arc.ccw)
			{
				while (theta1 < theta0)
					theta1 += PI2;
			}
			else
			{
				while (theta1 > theta0)
					theta1 -= PI2;
			}
		}

		// 2. do interpolation;
		auto theta = (t)*theta1 + (1 - t) * theta0;
		Point ret(cos(theta), sin(theta));
		return ret;
	}

	/* Def : given a point p and an arc c, first find the point on the arc closest from the point, then return its parameter t ~ [0,1];
	*/
	double getClosestArcParameter(Point& p, CircularArc &c)
	{
		auto n = (p - c.c.c).normalize();
		if (isNormalBetween(c.n[0], c.n[1], n))
		{
			return arcNormalToParameter(c.n[0], c.n[1], n);
		}
		else
		{
			auto d0 = (p - c.x[0]).length();
			auto d1 = (p - c.x[1]).length();
			if (d0 < d1) return 0.0;
			else return 1.0;
		}
	}

	/* 
	Def:
		return one circular arc corresponding to interval [t0, t1] of c
	Date:
		?
		2021-0421 : modified vmqa
	*/
	CircularArc subdivCircularArc(CircularArc& c, double t0, double t1)
	{
		if (PRINT_ERRORS)
		{
			if (t0 < -1e-32 || t0 > 1 + 1e-32 || t1 < -1e-32 || t1 > 1 + 1e-32 || t0 > t1)
			{
				cerr << "ERROR : in subdivCircularArc1, t0 t1 " << t0 << " " << t1 << endl;
			}
		}


		CircularArc ret = c;
		Point n0, n1;
		if (flag_vmqa)
		{
			//debugging;
			n0 = arcNormalInterpolation(c, t0);
			n1 = arcNormalInterpolation(c, t1);

		}
		else
		{
			n0 = arcNormalInterpolation(c.n[0], c.n[1], t0);
			n1 = arcNormalInterpolation(c.n[0], c.n[1], t1);
		}


		ret.n[0] = n0;
		ret.n[1] = n1;
		ret.x[0] = ret.c.c + (ret.c.r * n0);
		ret.x[1] = ret.c.c + (ret.c.r * n1);


		if (PRINT_ERRORS && dbgmode == 1)
		{
			cerr << "c.n0 c.n1 t0 t1 res.n0 res.n1" << c.n[0] << c.n[1] << " " << t0 << " " << t1 << " " << ret.n[0] << ret.n[1] << endl;
			cerr << "c.x0 c.x1 t0 t1 res.x0 res.x1" << c.x[0] << c.x[1] << " " << t0 << " " << t1 << " " << ret.x[0] << ret.x[1] << endl;
			cerr << "c.c + c.r * c.n[0], c.c + c.r * c.n[1] " << c.c.c + c.c.r * c.n[0] << c.c.c + c.c.r * c.n[1] << endl;
		}

		if (PRINT_ERRORS)
		{
			if
				(
					isnan(ret.n[0].P[0]) || isnan(ret.n[0].P[1]) ||
					isnan(ret.n[1].P[0]) || isnan(ret.n[1].P[1]) ||
					isnan(ret.x[0].P[0]) || isnan(ret.x[0].P[1]) ||
					isnan(ret.x[1].P[0]) || isnan(ret.x[1].P[1])
					)
				cerr << "ERROR : in subdivcircularArc, nan returned n0 n1"<< n0 << n1 << endl;
		}

		return ret;
	}

	/* 
	Def: 
		return two circular arcs [0, t] and [t, 1]
	Date:
		?
		2021-0421 : modified vmqa
	*/
	pair<CircularArc, CircularArc> subdivCircularArc(CircularArc& c, double t)
	{
		pair<CircularArc, CircularArc> ret;
		ret.first  = c;
		ret.second = c;

		Point n;
		if (flag_vmqa)
		{
			// debugging.
			n = arcNormalInterpolation(c, t);
		}
		else
		{
			n = arcNormalInterpolation(c.n[0], c.n[1], t);
		}

		ret.first .n[1] =
		ret.second.n[0] = n;
		ret.first .x[1] = 
		ret.second.x[0] = 
			c.c.c + c.c.r * n;

		return ret;
	}


	/* Def: given a point on an arc, compute its geometry info.
		Poistion
		Tangent		: should be normalized.
		Normal		: rotate+90 tangent. (This is diff def of noraml, compared to that in CircularArc class)
		curavture	: signed, ccw = positive
	*/
	pointGeometry getPointGeometry(CircularArc& arc, double t)
	{
		pointGeometry ret;

		// 1. get theta
		auto theta0 = atan2(arc.n[0].P[1], arc.n[0].P[0]);
		auto theta1 = atan2(arc.n[1].P[1], arc.n[1].P[0]);
		constexpr double _pi = 3.1415926535897932384;
		constexpr double _2pi = 3.14159265358979323846 * 2.0;
		
		if (flag_vmqa)
		{
			if (arc.ccw)
			{
				while (theta1 < theta0)
					theta1 += PI2;
			}
			else
			{
				while (theta1 > theta0)
					theta1 -= PI2;
			}
		}
		else
		{
			// 1-1. take care of those cases where +pi and -pi might be swapped.
			if (theta1 - theta0 > _pi)
			{
				theta1 = theta1 - _2pi; // note that theta won't be btw [-pi, +pi]
			}
			if (theta0 - theta1 > _pi)
			{
				theta1 = theta1 + _2pi;
			}
			// 1-2. Error chck
			if (PRINT_ERRORS && ERR_GET_POINT_GEOM_DTHETA)
			{
				auto dtheta = fabs(theta0 - theta1) / _pi * 180.0;
				if (dtheta > 90.0) cerr << "ERROR : in getPointGeometry, |dtheta| > 90" << endl <<
					"      : n0 n1 theta0 theta1 " << arc.n[0] << arc.n[1] << " " << theta0 / _pi * 180 << " " << theta1 / _pi * 180 << endl;
			}
		}
		auto theta = (t)* theta1 + (1 - t) * theta0;

		// 2. build ret
		Point normal(cos(theta), sin(theta));
		// 2-1. position
		ret.x = arc.c.c + arc.c.r * normal;
		if (arc.ccw)
		{
			// 2-2. tangent
			ret.v = rotate_p90(normal);
			// 2-3. normal
			ret.n = -normal;
			// 2.4. curvature
			ret.k = 1.0 / arc.c.r;
		}
		else
		{
			// 2-2. tangent
			ret.v = rotate_m90(normal);
			// 2-3. normal
			ret.n = normal;
			// 2.4. curvature
			ret.k = -1.0 / arc.c.r;
		}

		return ret;

	}


	/* Def : returns true when arc is very small
	*/
	bool epsilon(CircularArc& c)
	{
		auto dx = c.x[0] - c.x[1];
		return (dx*dx < 1e-18);
	}

	/* Def : squared length
	*/
	double sqlength(Point p)
	{
		return p*p;
	}

	/*
	Def: 
		given an arc, get its theta values t0, t1 of n0, n1, 
		so that traveling from x(0) to x(1) shows a continuous theta
	Retrun:
		ret.first  := double in [-pi,  + pi]
		ret.second := double in [-3pi, +3pi]

		if (ccw) ret.2nd > ret.1st
		else 1st > 2nd
	Assume:
		Correct n0, n1, and ccw
	*/
	pair<double, double> getArcTheta_cont(CircularArc& arc)
	{
		double
			theta0 = atan2(arc.n0().y(), arc.n0().x()),
			theta1 = atan2(arc.n1().y(), arc.n1().x());

		if (arc.ccw)
			while (theta1 < theta0)
				theta1 = theta1 + PI2;
		else
			while (theta1 > theta0)
				theta1 = theta1 - PI2;

		return { theta0, theta1 };
	}

	/* Def : test if endpoint in a cycle share a max touching
	*/
	void testValidCycle(vector<CircularArc> cycle, bool draw = true)
	{
		for (int i = 0; i < cycle.size(); i++)
		{
			int ni = (i + 1) % cycle.size();
			auto center = getTouchingDiskCenter(cycle[i].x[1], cycle[ni].x[0], cycle[i].n[1]);
			auto d0 = (cycle[i].x[1] - center).length();
			auto d1 = (cycle[ni].x[0] - center).length();
			cerr << "	TestValidCycle " << i << " " << d0 << " " << d1 << endl;


			//dbg_draw
			if (false)
			{
				if (i == 0)
				{
					if (keyboardflag['z'])
					{
						glBegin(GL_LINES);
						glColor3f(0, 0, 1);
						glVertex2dv(center.P);
						glVertex2dv(cycle[i].x[1].P);
						glColor3f(0, 1, 0.5);
						glVertex2dv(center.P);
						glVertex2dv(cycle[ni].x[0].P);
						glEnd();
					}
				}
				else if (i == 1)
				{
					if (keyboardflag['x'])
					{
						glBegin(GL_LINES);
						glColor3f(0, 0, 1);
						glVertex2dv(center.P);
						glVertex2dv(cycle[i].x[1].P);
						glColor3f(0, 1, 0.5);
						glVertex2dv(center.P);
						glVertex2dv(cycle[ni].x[0].P);
						glEnd();
					}
				}
				else if (i == 2)
				{
					if (keyboardflag['c'])
					{
						glBegin(GL_LINES);
						glColor3f(0, 0, 1);
						glVertex2dv(center.P);
						glVertex2dv(cycle[i].x[1].P);
						glColor3f(0, 1, 0.5);
						glVertex2dv(center.P);
						glVertex2dv(cycle[ni].x[0].P);
						glEnd();
					}
				}
				else
				{
					glBegin(GL_LINES);
					glColor3f(0, 0, 1);
					glVertex2dv(center.P);
					glVertex2dv(cycle[i].x[1].P);
					glColor3f(0, 1, 0.5);
					glVertex2dv(center.P);
					glVertex2dv(cycle[ni].x[0].P);
					glEnd();
				}
			}
		}
	}
#pragma endregion

	/*
	Def: 
		given a chunk-of-arcs that is known to be 
			G0-continuous(in other words, forms a single closed loop without some arcs dangling out of loop)
				= for each of the arc endpoint, there exists exactly one endpoint (from another arc) such that the two endpoints are close enough (under some errorBound RVCA_EndpointError)
			but the order of arcs inside vector<CircularArc> does nothing to do with order in the loop
				CircularArcs are ordered randomly inside "input"
		Reorder the order of arcs so that output[i].x[1] == output[i+1].x[0] (applies same for the last/first arc pair)

	Param: 
		globalCCW_Option :
			ordering of the arcs should be done so that the loop is (option)
			-1 : doesn't matter ccw or cw (from arc at input[0] to arc connected to x1() of input[0]
			0: cw
			1: ccw
	Assumes:
		For each arc, below infos should be filled
		1. x[0]
		2. x[1]
		3. ccw (localccw)
			so that we can know the tangent direction's +- at one point.

	Dependency?:
		Some code(section 1) is from cd::_trsi_buildneighbor,
	*/
	void _Convert_VectorCircularArc_G0(INPUT vector<CircularArc>& input, OUTPUT vector<CircularArc>& output, INPUT int globalCCW_Option)
	{
		if (input.size() < 2) return;

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
		vector<neighbor> neigh;
		auto& superset = input;
		auto& errorNeigh = RVCA_EndpointError;

		// 1. find neighbor info (this is mostly eqaul to cd::_trsi_buildneighbor, but changed to use a different error value)
		{
			int nArcs = superset.size();
			neigh.resize(0);
			neigh.resize(nArcs);

			// for (each arc_pair)
			for (size_t i = 0; i < nArcs; i++)
			for (size_t j = i + 1; j < nArcs; j++)
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

		// 2. push_back arcs into output with neighbor info.
		{
			size_t length = input.size();
			
			LOOP vector<bool> pushed(length, false);
			LOOP int pushCount = 0;
			LOOP int cur = 0;
			while (true)
			{
				// 2-1. check end condition;
				if (cur < 0 || pushed[cur])
					break;

				// 2-2. push
				pushCount++;
				output.push_back(input[cur]);
				pushed[cur] = true;

				// 2-3. find next cur;
				auto
					& candidate1 = neigh[cur].idx[1],
					& candidate0 = neigh[cur].idx[0];

				if (candidate1 < 0)
				{
					cur = candidate0;
				}
				else if (candidate0 < 0)
				{
					cur = candidate1;
				}
				else
				{
					if (pushed[candidate1])
						cur = candidate0;
					else
						cur = candidate1;
				}
			}

			// check errors that may happen
			// probably from input errors... (not froming single loops or more than two endpoints in [RVCA_EP_err, RVCA_EP_err] grid.
			if (pushCount != length)
				cout << "ERROR : in _Reorder_VectorCircularArc : not all arcs were pushed" << endl;

		}

		// 3. flip arcs so that output[i].x[1] == output[i+1].x[0];
		// 3-1. for (i.end and (i+1).start)
		for (size_t i = 0, length = output.size() - 1; i < length; i++)
		{
			//size_t iNext = (i + 1) % length;
			size_t iNext = i + 1;
			
			// if(next arc's x1 is closer than x0) flip
			if ((output[i].x1() - output[iNext].x1()).length2() < (output[i].x1() - output[iNext].x0()).length2())
				output[iNext] = flipArc(output[iNext]);
		}
		// 3-2. for the first/last arc pair...
		// on second thought, there is no need to do this.
		//	as at this point pair (0,1) , (1,2) ... (last-1, last) are all in right order.

		// 4. find/set global ccw or cw;
		if (globalCCW_Option < 0)
			return;
		else
		{
			// If the model is globally counter-clock-wise, the inner-region-of-object will be on the left it's tangent direction.
			// we can know the tangent, but we can't know which side is the inner-region.
			// But the point with maximum y value is easy to know its inner region
			//	I. such point is G1 : simply use tangent of that point and (-y vector as a vector pointing to inner region)
			//  II. max-y-point was not G1 (notice !G1 => max-y == endpoint, but being endpoint =/=> !G1)
			//		a bit complex, use y component's size of tangent

			// 4-1. find max-y
			LOOP int localPosition = 2; // 0 : x[0], 1 : x[1], 2: at pi/2
			LOOP int bestIdx = -1;
			LOOP double bestY = -1e+100;

			for (int i = 0; i < output.size(); i++)
			{
				auto& arc = output[i];

				// 4-1-1. find if arc contains pi/2;
				auto theta0 = atan2(arc.n0().y(), arc.n0().x());
				auto theta1 = atan2(arc.n1().y(), arc.n1().x());
				auto thetaP = PI / 2;
				if (arc.ccw)
				{
					while (theta1 < theta0)
						theta1 += PI2;
					while (thetaP < theta0)
						thetaP += PI2;
				}
				else
				{

					while (theta1 > theta0)
						theta1 -= PI2;
					while (thetaP > theta0)
						thetaP -= PI2;
				}

				// 4-1-2. record which was max y
				bool halfPiInside = arc.ccw ? theta1 >= thetaP : thetaP >= theta1; // equal case can be evaluated as the same way, so use >=
				// dbg_out
				cout << "halfPiInside : " << halfPiInside << endl;
				if (halfPiInside)
				{
					double y = arc.c.c.y() + arc.c.r;
					if (y > bestY)
					{
						bestY = y;
						bestIdx = i;
						localPosition = 2;
					}
				}
				else
				{
					if (arc.x0().y() > bestY)
					{

						bestY = arc.x0().y();
						bestIdx = i;
						localPosition = 0;
					}

					if (arc.x1().y() > bestY)
					{

						bestY = arc.x1().y();
						bestIdx = i;
						localPosition = 1;
					}
				}
			}

			// 4-2. with maxY, find current globalCCW info.
			bool currentGlobalCCW = false;
			// dbg_out
			cout << "locPos : " << localPosition << endl;
			cout << "bestY : " << bestY << endl;
			if (localPosition == 2)
			{
				// 4-2-1. the max y point is in the middle of an arc.
				//	At max-y-point, inner region is on -y dir, so lets just see tangent direction
				//	Tangent direction is decided by local-ccw of bestIdx;
				//	Also, in this case arc is convex, so just directly assign ccw
				currentGlobalCCW = output[bestIdx].ccw;
			}
			else
			{
				// 4-2-2. when max-y-point is from an end
				//	As an endpoint is shared, one arc will rise-to-max-y and one arc will descend-from-max-y.
				//	As the global traversal direction is unified in "output" at this point, above is guaranteed.
				//	Notice that the angle between two tangent vectors can vary from [0, 180) degrees.
				//	But a vector with smaller absolute-y-value(since one is positive and one is negative), which is somewhat 'closer' to y = max_y line, 
				//		will have an inner-region in its -y direction, regardless of the other tangent 
				
				// find idx of arcs sharing max-y-point
				int
					idx0, // arc rising to max-y (in current globalCCW)
					idx1; // arc descending from max-y (cur gloCCW)

				// if (the max-y is an endpoint x1() of arc bestIdx)
				if(localPosition == 1)
				{
					idx0 = bestIdx;
					idx1 = (bestIdx + 1) % output.size();
				}
				else
				{
					idx1 = bestIdx;
					idx0 = (bestIdx - 1) % output.size();
				}

				auto
					& arc0 = output[idx0],
					& arc1 = output[idx1];

				// find tangent
				Point
					tangent0, //tangentN = tangent at max-y-point of arc=idxN
					tangent1;

				if (arc0.ccw)
					tangent0 = rotate_p90(arc0.x1() - arc0.c.c).normalize();
				else
					tangent0 = rotate_m90(arc0.x1() - arc0.c.c).normalize();

				if (arc1.ccw)
					tangent1 = rotate_p90(arc1.x0() - arc1.c.c).normalize();
				else
					tangent1 = rotate_m90(arc1.x0() - arc1.c.c).normalize();

				// find which tangent has smaller ||y||
				Point&
					tangent = (fabs(tangent0.y()) < fabs(tangent1.y())) ? tangent0 : tangent1;

				// now tangent is guaranteed to have an inner-region in -y;
				if (tangent.x() < 0)
					currentGlobalCCW = true;
				else
					currentGlobalCCW = false;
			}

			// 4-3. check wheter current global-ccw matches with the request.
			bool requestCCW = globalCCW_Option != 0;
			//dbg
			cout << "req: " << requestCCW << endl;
			cout << "curccw : " << currentGlobalCCW << endl;
			//~dbg_out
			if (requestCCW ^ currentGlobalCCW)
			{
				// 4-3-1. flip order in output;
				reverse(output.begin(), output.end());

				// 4-3-2. flip each arc
				for (auto& arc : output)
					arc = flipArc(arc);
			}
			else return;

		}
	}

	/*
	Def: given an ordered vector<CircularArc>, add some small arcs between non-g1-neighbors, to make it G1-continuous

	Assumes:
		input : contains circularArcs G0-continuous, such that
			input[i].x[1] == input[i+1].x[0] (for i in [0, input.size() -1)
			input[0].x[0] == input[last].x[1]
	
	*/
	void _Convert_VectorCircularArc_G1(INPUT vector<CircularArc>& input, OUTPUT vector<CircularArc>& output)
	{
		if (input.size() < 2) return;

		const double MVCAG1_Error = 1 - 1e-4;
		const double MVCAG1_EPS_ARC_RAD = 1e-10;

		for (size_t i = 0, length = input.size(); i < length; i++)
		{
			auto iNext = (i + 1) % length;

			// 1. original arc
			output.push_back(input[i]);

			// 2. if (not g1) push eps arc
			if (fabs(input[i].n1() * input[iNext].n0()) < MVCAG1_Error)
			{
				auto
					& arc0 = input[i],
					& arc1 = input[iNext];


				// 2-1. get tangent at each arc endpoint;
				Point tangent0 =
					arc0.ccw ?
					/*rotate_p90(arc0.x1() - arc0.c.c).normalize():
					rotate_m90(arc0.x1() - arc0.c.c).normalize();*/
					rotate_p90(arc0.n1()):
					rotate_m90(arc0.n1());
				Point tangent1 =
					arc1.ccw ?
					rotate_p90(arc1.n0()):
					rotate_m90(arc1.n0());/*
					rotate_p90(arc1.x0() - arc1.c.c).normalize():
					rotate_m90(arc1.x0() - arc1.c.c).normalize();*/

				// 2-2. get ccw of new arc
				CircularArc newArc;
				newArc.ccw = ((tangent0 ^ tangent1) > 0);

				// 2-3. fill in infos
				if (newArc.ccw)
				{
					newArc.n0() = rotate_m90(tangent0);
					newArc.n1() = rotate_m90(tangent1);
				}
				else
				{
					newArc.n0() = rotate_p90(tangent0);
					newArc.n1() = rotate_p90(tangent1);
				}
				
				newArc.c.c = arc0.x1();
				newArc.c.r = MVCAG1_EPS_ARC_RAD;

				newArc.x0() = newArc.c.c + (newArc.c.r * newArc.n0()); 
				newArc.x1() = newArc.c.c + (newArc.c.r * newArc.n1());
				// note that the endpoints won't exactly match...
				// we need to carve some of the original arcs if we need that...
				// currently rad was picked so that the eps-arc endpoints arc considered the same point for Point-operator==

				//// dbg : use whole-circle at non-g1 point
				//newArc.c.r = 0;
				//newArc.x0() = newArc.x1() = Point(0,0);
				//newArc.n0() = Point(-1, -1e-20);
				//newArc.n1() = Point(-1, 1e-20);
				//newArc.ccw = true;
				//// ~dbg

				// 2-4. push
				output.push_back(newArc);

			}
		}
	}

	
	/*
	Def: similar to older version of convert_g1, but more strictly g0 by cutting original arcs 

	If an eps-arc is put, original arc's x0 or x1 (which is the relevant) also shrinks so that it is more g0

	Experiment:
		for the case where I used an arc with radius 100, the newer version was slightly better or eqaul...
		but as radius 10 was used (in this case the straight line's curvedness could be seen with human sense), older version was better.
	*/
	void _Convert_VectorCircularArc_G1_new(INPUT vector<CircularArc>& input, OUTPUT vector<CircularArc>& output)
	{
		if (input.size() < 2) return;

		// controll

		const double MVCAG1_Error = 1 - 1e-4;
		const double SHIFT_LEN = 0.01; // notice that this is somewhat a hyperparameter

		// 1. build info about input

		//vector<bool> insertNext; // insertNext[i] == true := put small arc btw arc_i and arc_(i+1) 
		vector<bool> trim0(input.size(), false);	// cut a little bit of arc_i near that arc's x0
		vector<bool> trim1(input.size(), false);	// cut a little bit of arc_i near that arc's x1

		for (size_t i = 0, length = input.size(); i < length; i++)
		{
			auto iNext = (i + 1) % length;
			// if(g1)
			if (fabs(input[i].n1() * input[iNext].n0()) < MVCAG1_Error)
			{
				trim1[i] = true;
				trim0[iNext] = true;

				//dbg
				cout << "found none g1 conn" << endl;
			}
		}

		// 2. build arcs
		for (size_t i = 0, length = input.size(); i < length; i++)
		{
			auto iNext = (i + 1) % length;

			// 2-1. trim original arc by changing arc's theta
			auto inserted = input[i];
			auto thetas = getArcTheta_cont(inserted);
			auto
				theta0 = thetas.first,
				theta1 = thetas.second;
			auto
				signed_shift_theta = SHIFT_LEN / inserted.c.r;

			// 2-1-1. if (ccw) theta0 inc / theta1 decrease
			if (inserted.ccw)
			{
				// do nothing
			}
			else
			{
				// flip sign simply
				signed_shift_theta *= -1;
			}

			// 2-1-2. d(theta)
			if (trim0[i])
			{
				theta0 += signed_shift_theta;
			}
			if (trim1[i])
			{
				theta1 -= signed_shift_theta;
			}

			// 2-1-3. check erroroneous case : TODO simplify with XOR
			bool unsolvable = false;
			if (inserted.ccw)
			{
				if (theta0 > theta1)
					unsolvable = true;
			}
			else
			{
				if (theta0 < theta1)
					unsolvable = true;
			}

			if (unsolvable)
			{
				// currently just prints error
				cerr << "ERROR: in CVAG1, original arc may be trimmed too much. ccw : " << inserted.ccw << endl;
				cerr << fixed << setprecision(20);
				cerr << "before : " << thetas.first << "   " << thetas.second << endl;
				cerr << "after  : " << theta0 << "    " << theta1 << endl;
			}

			// 2-1-4. build trimmed & insert
			{
				inserted.n0() = Point(cos(theta0), sin(theta0));
				inserted.n1() = Point(cos(theta1), sin(theta1));
				inserted.x0() = inserted.c.c + inserted.c.r * inserted.n0();
				inserted.x1() = inserted.c.c + inserted.c.r * inserted.n1();
				if (unsolvable)
					inserted.ccw = !inserted.ccw; // this is just too make bug-case work better... should not be executed with good-input.
			}

			output.push_back(inserted);

			// 2-2. build eps-arc
			// if(should insert eps-arc next)
			if (trim1[i])
			{
				// eps-arc :
				//	its endpoints : trimmed ends of arcs that weren't g1;
				//  use tangent at inserted.x1(). to compute center.
				
				Point
					eps_start,
					eps_end,
					tangent_start;

				tangent_start = inserted.ccw ? rotate_p90(inserted.n1()) : rotate_m90(inserted.n1());
				eps_start = inserted.x1();

				// have to calculate eps_end similar to 2-1, but for the arc that comes next;
				{
					auto& nextArc = input[iNext];
					auto theta0 = atan2(nextArc.n0().y(), nextArc.n0().x());
					auto signed_shift_theta = SHIFT_LEN / nextArc.c.r;
					if (nextArc.ccw)
						theta0 += signed_shift_theta;
					else
						theta0 -= signed_shift_theta;
					auto normal = Point(cos(theta0), sin(theta0));
					eps_end = nextArc.c.c + nextArc.c.r * normal;
				}

				output.push_back(cd::constructArc(eps_start, eps_end, tangent_start));
			
			}

		}

	}

	/*
	given a G0 vector<arc> (with continuous end points), change it to vec<as>, so that it is an valid MS input
	I guess it should be
		1. x mono
		2. y mono
		3. no curvatur ext
		4. curvature monotone
	=> simply devide arcs with x,y ext and make an arcspline with it.

	referenced : ArcSpline::ArcSpline(BezierCrv & Crv)
	ccw and normals... see => CircularArc::CircularArc(Point & i, Point & e, Point & t)
	Assumes that
		arc.ccw is local ccw

	Notes about proper input of mink sum:
	seems like arc should all be counter-clock-wise (globally)
		but this does not mean that ccw value should be true
	Arcspline:
		should set 5: ccw, n0, n1, xquad, yquad
	Order of arcs in "input" does not seem to matter. (see model: square swept circle)
	Convex/Concave arcs are both okay, as long as they are globally counterclockwise (see model o=0)

	*/
	void _Convert_VectorCircularArc_To_MsInput(INPUT vector<CircularArc>& input, OUTPUT vector<ArcSpline>& output, double rotationDegree)
	{
		Point
			n0(1, 0),
			n1(0, 1),
			n2(-1, 0),
			n3(0, -1);

		double extremeAngle[4] = { 0, PI / 2 , PI, - PI / 2 };

		for (auto arcToBeRotated : input)
		{
			auto arc = cd::rotateArc(arcToBeRotated, rotationDegree);
			//ArcSpline ins; // to be inserted to ouput
			// check x, y monotone

			//// dbg : see if model has to be globally clock-wise
			//arc = flipArc(arc);
			//// ~dbg : result: it should be 

			// 1. build radians
			vector<double> radians;
			
			// 1-1. end p
			bool localCCW = arc.ccw;
			double theta0 = atan2(arc.n0().y(), arc.n0().x());
			double theta1 = atan2(arc.n1().y(), arc.n1().x());
			if (localCCW)
				while (theta1 < theta0)
					theta1 += PI * 2;
			else
				while (theta1 > theta0)
					theta1 -= PI * 2;

			radians.push_back(theta0);
			radians.push_back(theta1);

			// 1-2. extreme p
			for (int i = 0; i < 4; i++)
			{
				double temp = extremeAngle[i];
				if (localCCW)
				{
					while (temp < theta0)
						temp += PI * 2;
					if (temp < theta1)
						radians.push_back(temp);
				}
				else
				{
					while (temp > theta0)
						temp -= PI * 2;
					if (temp > theta1)
						radians.push_back(temp);
				}

			}

			// 1-3. sort
			if(localCCW)
				sort(radians.begin(), radians.end(), std::less<double>());
			else
				sort(radians.begin(), radians.end(), std::greater<double>());

			// 2 build arc spline
			for (int i = 0; i < radians.size() - 1; i++)
			{
				auto& theta0 = radians[i];
				auto& theta1 = radians[i+1];
				auto cutArc = cd::constructArc(arc, theta0, theta1);
				
				//setting strange ccw from (CircularArc::CircularArc(Point & i, Point & e, Point & t))
				{
					if (!counterclockwise(cutArc.n[0], cutArc.n[1]))
					{
						cutArc.ccw = false;

						auto tempP = cutArc.n[0];
						cutArc.n[0] = cutArc.n[1];
						cutArc.n[1] = tempP;
						//std::swap(cutArc.n[0], cutArc.n[1]);
					}
					else
						cutArc.ccw = true;
					cutArc.boundary = false;
				}
				ArcSpline asTemp;
				asTemp.Arcs.push_back(cutArc);

				// from ArcSpline::ArcSpline(BezierCrv & Crv)
				asTemp.ccw = cutArc.ccw;
				Vector test = asTemp.Arcs.front().n[0] + asTemp.Arcs.back().n[1];
				asTemp.xQuardrants = (test.P[0] > 0);
				asTemp.yQuardrants = (test.P[1] > 0);
				asTemp.n[0] = asTemp.ccw ? asTemp.Arcs.front().n[0] : asTemp.Arcs.back().n[0];
				asTemp.n[1] = asTemp.ccw ? asTemp.Arcs.back().n[1]  : asTemp.Arcs.front().n[1];

				output.push_back(asTemp);
			}





		}
	}

	/*	Def: Reduce the difference btw "output of MS" & "input of vrIN"
		Par:
			msOut		: model_Result glo var.
			isBoundary	: it's actually, a bool wheter it is the "outer" boundary (not inner loop)
			vrIn		: just empty VR_IN should be okay.
		Assume:
			Vector<CircularArc> voronoiBoundary is already set.
		Date:
		2021
			0421: added code to handle continuous arcs with same r/center (can be turned off with use_simplifyVCA = false)
	*/
	void _Convert_MsOut_To_VrIn(vector<deque<ArcSpline>>& INPUT msOut, vector<bool>& INPUT isBoundary, VR_IN& OUTPUT vrIn)
	{
		

		// 1. count number of outer boundaries & check valid input
		{
		int nBoundary = 0;
			for (auto i : isBoundary)
				if (i) nBoundary++;
			if (PRINT_ERRORS && nBoundary == 0) cerr << "ERROR 0 : There is no outer Boundary in Convert_MsOut_To_VrIn" << endl;
		}

		// 2. mash up msOutput into a big chunk circularArc
		const size_t length = msOut.size();
		if (PRINT_ERRORS && length > isBoundary.size()) cerr << "ERROR 1 : Arr Size not equal" << endl;

		int LOOP leftOffset = 0;
		for (size_t i = 0; i < length; i++)
		{
			
			if (!isBoundary[i]) 
			{
				// // ignore inner loops, since we are doing motion planning => this is changed
				// continue;

				// similar to below else-case. just flip local/global direction
				// should be simplified to 
				//		convertMS_cw
				//		if(!boundary) flip
				//		(other stuff...)
				{
					// 2-1. build vrIn.Arcs & arcsPerLoop
					vector<CircularArc> temp,temp2;
					if (!use_simplifyVCA)
					{
						convertMsOutput_Clockwise(msOut[i], temp); // HARD
					}
					else
					{
						convertMsOutput_Clockwise(msOut[i], temp2);
						//set local ccw info (since mink sum code uses the var for glo/loc ccw, but in vor, it is assumed to be local)
						for (auto& i : temp2)
						{
							auto ccw = (i.n[0] ^ i.n[1]) > 0;
							i.ccw = ccw;
						}
						simplifyVCA(temp2, temp);
					}
					//vector<CircularArc> temp2;
					//for(auto& as : msOut[i])
					//	for (auto& arc : as.Arcs)
					//	{
					//		temp2.push_back(arc);
					//	}
					//_Convert_VectorCircularArc_G0(temp2, temp, 1);
					// 2-1-1. flip if boundary
					/*reverse(temp.begin(), temp.end());
					for (auto& arc : temp)
						arc = flipArc(arc);*/

					vrIn.arcs.insert(vrIn.arcs.end(), temp.begin(), temp.end());
					vrIn.arcsPerLoop.push_back(temp.size());

					// 2-2. build vrIn.left
					vrIn.left.resize(vrIn.arcs.size());
					vrIn.left[leftOffset] = vrIn.left.size() - 1; // left[first element of this loop] = last element.
					for (size_t j = 1, length = temp.size(); j < length; j++)
					{
						vrIn.left[leftOffset + j] = leftOffset + j - 1; // left[element] = element - 1 
					}

					// 2-3. build vrIn.color
					vrIn.color.resize(vrIn.arcs.size());
					auto color = double(i) / length;
					for (size_t j = 0, length = temp.size(); j < length; j++)
					{
						vrIn.color[j] = color;
					}

					// 2-4. update llop variables.
					leftOffset = vrIn.arcs.size();

				}

			}
			else
			{

				// 2-1. build vrIn.Arcs & arcsPerLoop
				vector<CircularArc> temp, temp2;
				if (!use_simplifyVCA)
				{
					convertMsOutput_Clockwise(msOut[i], temp); // HARD
				}
				else
				{
					convertMsOutput_Clockwise(msOut[i], temp2);
					//set local ccw info (since mink sum code uses the var for glo/loc ccw, but in vor, it is assumed to be local)
					for (auto& i : temp2)
					{
						auto ccw = (i.n[0] ^ i.n[1]) > 0;
						i.ccw = ccw;
					}
					simplifyVCA(temp2, temp);
				}
				vrIn.arcs.insert(vrIn.arcs.end(), temp.begin(), temp.end());
				vrIn.arcsPerLoop.push_back(temp.size());

				// 2-2. build vrIn.left
				vrIn.left.resize(vrIn.arcs.size());
				vrIn.left[leftOffset] = vrIn.left.size() - 1; // left[first element of this loop] = last element.
				for (size_t j = 1, length = temp.size(); j < length; j++)
				{
					vrIn.left[leftOffset + j] = leftOffset + j - 1; // left[element] = element - 1 
				}

				// 2-3. build vrIn.color
				vrIn.color.resize(vrIn.arcs.size());
				auto color = double(i) / length;
				for (size_t j = 0, length = temp.size(); j < length; j++)
				{
					vrIn.color[j] = color;
				}

				// 2-4. update llop variables.
				leftOffset = vrIn.arcs.size();


			}
		}

		// 3. add boundary(ccw) for 
		if(true)
		{
			//double r = 2;
			//CircularArc a(Point(0, 0), r, Point(+1, +0), Point(+0, +1));
			//CircularArc b(Point(0, 0), r, Point(+0, +1), Point(-1, +0));
			//CircularArc c(Point(0, 0), r, Point(-1, +0), Point(+0, -1));
			//CircularArc d(Point(0, 0), r, Point(+0, -1), Point(+1, +0));
			//auto s = vrIn.arcs.size();
			//vrIn.arcs.push_back(a);
			//vrIn.arcs.push_back(b);
			//vrIn.arcs.push_back(c);
			//vrIn.arcs.push_back(d);
			//vrIn.left.push_back(s + 3);	// left(a) = d
			//vrIn.left.push_back(s + 0);	// left(b) = a
			//vrIn.left.push_back(s + 1);	// left(c) = b
			//vrIn.left.push_back(s + 2); // left(d) = c
			//vrIn.color.push_back(1);
			//vrIn.color.push_back(1);
			//vrIn.color.push_back(1);
			//vrIn.color.push_back(1);
			//vrIn.arcsPerLoop.push_back(4);

			auto s = vrIn.arcs.size();
			size_t length = voronoiBoundary.size();
			for (size_t i = 0; i < length; i++)
			{
				vrIn.arcs.push_back(voronoiBoundary[i]);
				vrIn.left.push_back(s + ((i - 1) % length));
				vrIn.color.push_back(1.0);
			}
			vrIn.arcsPerLoop.push_back(length);
		}


		// 4. fix bugs in output
		for (auto& i : vrIn.arcs)
		{
			if (!use_simplifyVCA)
			{
				// 4-1. some with strange ccw
				auto ccw = (i.n[0] ^ i.n[1]) > 0;
				//if (PRINT_ERRORS && ccw != i.ccw) cerr << "ERROR : ccw of arc not correct\n      : n0 n1 arc.ccw" << i.n[0] << i.n[1] << " " << i.ccw << endl;
				i.ccw = ccw;
			}

			// 4-2. some with strange normal (i.x[] is true value)
			auto n0 = i.x[0] - i.c.c;
			auto n1 = i.x[1] - i.c.c;
			i.n[0] = n0 / sqrt(n0*n0);
			i.n[1] = n1 / sqrt(n1*n1);
		}

		// 5. test?
		int eps = 0;
		for (auto i : vrIn.arcs)
			if (epsilon(i)) eps++;
		cout << "eps arc : " << eps << endl;
		return;
	}


	/* Def: given a point on an arc, compute the maximal touching disk of that point. The return another point touching that disk.

	For arc > 90degree compatibility:
		getPointGeom
		2-6
		arcNormalToParam
	Date:
	2021
		0421 added code 2-2.5 : since arcs from same circles won't produce v-edge
		0421 added flag_findMTD_concave_check: convex arcs won't get max-disk-rad bigger than itself
	*/
	double _h_fmdsp_g1 = 1e-8; //hyperParam, _epsilon
	pointOnCurve findMaximalDiskSharingPoint(vector<CircularArc>& INPUT arcs, pointOnCurve INPUT poc, int INPUT left, int INPUT right)
	{

		//1. init stuff
		// alias
		auto& i = poc.c;
		auto& t = poc.t;

		// get geometry
		auto g = getPointGeometry(arcs[i], t);
		auto p = g.x;
		auto v = g.v;
		auto n = g.n;
		auto k = g.k;

		bool convex = !arcs[i].ccw; //since objects are winded cw


		//tag g1inc
		if (left != poc.c && t < 1e-10)
		{
			if (!flag_findMTD_nonG1_divide_case)
			{
				if (fabs(arcs[left].n[1] * arcs[poc.c].n[0]) < 1 - _h_fmdsp_g1) //check if not g1
					return { left, 1 };
			}
			else
			{
				if (fabs(arcs[left].n[1] * arcs[poc.c].n[0]) < 1 - _h_fmdsp_g1)//check if not g1
				{
					// 2cases are possible : Look at the shape of a 'V'. This is two tangents meeting at the bottom point.
					// 1. voronoi is drawn on the upper side of 'V' : The radius of MTD is 0 -> return {left,1}
					// 2. on the lower side of 'V' shape : Do not return.

					auto n2 = arcs[left].n1();
					Point t2; // tangent at left's x1()
					if (arcs[left].ccw)
						t2 = rotate_p90(n2);
					else
						t2 = rotate_m90(n2);

					if (n * t2 < 0) // dot btw (normal & left's tangent) could be used to decide
						return { left,1 };

				}
			}

		}

		if (PRINT_ERRORS && (n*n > 1.01 || n*n < 0.99)) cerr << "ERROR 3 : normal  not normalized " << endl;
		//dbg if (PRINT_ERRORS && i == 97)
		//{
		//	cout << " i = 97 p v n k " << p << v << n << k << endl;

		//	auto arc = arcs[i];
		//	auto theta0 = atan2(arc.n[0].P[1], arc.n[0].P[0]);
		//	auto theta1 = atan2(arc.n[1].P[1], arc.n[1].P[0]);
		//	constexpr double _pi = 3.1415926535897932384;
		//	constexpr double _2pi = 3.14159265358979323846 * 2.0;
		//	if (theta1 - theta0 > _pi)
		//	{
		//		theta1 = theta1 - _2pi; // note that theta won't be btw [-pi, +pi]
		//	}
		//	if (theta0 - theta1 > _pi)
		//	{
		//		theta1 = theta1 + _2pi;
		//	}

		//	auto theta = (t)* theta1 + (1 - t) * theta0;
		//	cout << " n[0], n[1], theta0, theta1, theta"
		//		<<arc.n[0] << " " << arc.n[1] << " " << theta0 / _pi * 180 << " " << theta1 / _pi * 180 << " " << theta / _pi * 180 << endl;



		//	glBegin(GL_LINES);
		//	glVertex2d(0, 0);
		//	glVertex2dv(p.P);
		//	glEnd();

		//	glBegin(GL_LINES);
		//	glVertex2dv(p.P);
		//	glVertex2dv((p+n).P);
		//	glEnd();

		//	glBegin(GL_LINE_STRIP);
		//	glVertex2dv(arcs[i].x[0].P);
		//	glVertex2dv(arcs[i].x[1].P);
		//	glVertex2dv(arcs[i].c.c.P);
		//	glVertex2dv(arcs[i].x[0].P);
		//	glEnd();
		//}

		//dbg
		//	//test if G-1 continuity matters
		//	// if not g-1 cont with left, just return left.
		//if (doG1test && t < 1e-10)
		//{
		//	auto n0 = arcs[left].n[1];
		//	auto n1 = arcs[i].n[0];
		//	auto cont = (1 - fabs(n0*n1)) < 1e-2;
		//	if (!cont) return { left, 1 };
		//}
		//_dbg

		// 2. get maxdisk
		double r_init = 1e+10;
		Point   LOOP center(0, 0);
		double	LOOP r = r_init;	// currently best r
		bool	LOOP atMiddle;		// t not 0 nor 1 // if this is true just use middle
		bool	LOOP atLeft = false;		// t = 0		 // not used if atMiddle = true;
		int		LOOP bestJ;
		bool    LOOP bestZIC;       // zeroInsideCricle saved for bestJ;
		for (size_t j = 0, length = arcs.size(); j < length; j++)
		{
			// 2-1. continue if neighbor
			if (j == left || j == right)continue;

			// 2-2. skip if too far
			auto tempPoint = center - arcs[j].c.c;
			auto distBtwCenters = sqrt(tempPoint * tempPoint);
			if (r != r_init && distBtwCenters > r + arcs[j].c.r) continue;
			//if (distBtwCenters < fabs(r - arcs[j].c.r)) continue;

			// 2-2.5 another check. check if from same circle
			if (arcs[i].c == arcs[j].c)
				continue;

			// 2-3. distance infos
			Point L = arcs[j].c.c - p;
			double R = arcs[j].c.r;
			double d = sqrt(L * L);

			// 2-4. change coord (O = query point, x = tan, y = nor)
			auto a = L * v;
			auto b = L * n;
			if (b < -R) continue; // the circle is completely below x-axis

								  // 2-5. get y coord of disk center
			bool zeroInsideCircle = d < R;
			double y;
			if (zeroInsideCircle)
				y = (a*a + b*b - R*R) / (2 * (b - R));
			else
				y = (a*a + b*b - R*R) / (2 * (b + R));

			if (PRINT_ERRORS)
			{
				if (piececnt == 205 && cur_depth == 3)
					cerr << "in findMaximal, y b R " << y << " " << b << " " << R << endl;
			}

			// 2-6. compute normal of touching point & see if arc includes that normal
			Point normal = p + y * n - arcs[j].c.c;
			normal = normal / sqrt(normal*normal);

			bool normalInArc;
			double eps = 1e-180;
			if (flag_vmqa)
			{
				//debugging
				normalInArc = isNormalBetween(arcs[j], normal);
			}
			else
			{
				if (arcs[j].ccw)
					normalInArc = ((arcs[j].n[0] ^ normal) >= -eps) && ((normal ^ arcs[j].n[1]) >= -eps);
				else
					normalInArc = ((arcs[j].n[0] ^ normal) <= eps) && ((normal ^ arcs[j].n[1]) <= eps);
			}

			// 2-7. decide which point is closest
			if (normalInArc)
			{
				if (PRINT_ERRORS)
					if (y < 0)
						cerr << "ERROR : y < 0 at 2-7 of findMaximal..." << endl;
				// 2-7-1. use that point
				if (y < r && y >0) // TODO :: where to place... if !normalInArc, r1 >y, r2 >y... but is the query(y<r) necessary? could it be already done? ->probably since in 2-2, we tested bounding circle, not arc...
				{
					r = y;
					bestJ = j;
					center = p + (n * r);
					atMiddle = true;
					bestZIC = zeroInsideCircle;
					//dbg if (i == 97) drawCircle(Circle(center, r));
					//dbg if (i == 97) cout << "j, r " << j << " " << r << endl;

				}
			}
			else
			{
				//dbg continue;

				// 2-7-2. use endpoints;
				auto d0 = arcs[j].x[0] - p;
				auto d1 = arcs[j].x[1] - p;
				auto r0 = (d0*d0) / (2 * (n*d0));
				auto r1 = (d1*d1) / (2 * (n*d1));
				bool
					r0pos = r0 > 0,
					r1pos = r1 > 0,
					use_r0;

				// 2-7-2-1. choose r0 or r1, the one smaller.
				if (r0pos && r1pos)
				{
					if (r0 < r1)
						use_r0 = true;
					else
						use_r0 = false;
				}
				else if (r0pos)
				{
					use_r0 = true;
				}
				else if (r1pos)
				{
					use_r0 = false;
				}
				else
					continue; // when both points are on -y

				// 2-7-2-2. compare it with current r
				// set loop variables.
				if (use_r0)
				{
					//bool eqDist = fabs(sqrt((arcs[j].x[0] - (p + (n * r0))).length()) - r0) < 1e-8;
					if (r0 < r /*&& eqDist*/)
					{
						r = r0;
						center = p + (n * r);
						bestJ = j;
						atMiddle = false;
						atLeft = true;
						bestZIC = zeroInsideCircle;
						//dbg if (i == 97) drawCircle(Circle(center, r));
						//dbg drawCircle(Circle(center, r));
					}
				}
				else
				{
					//bool eqDist = fabs(sqrt((arcs[j].x[1] - (p + (n * r1))).length()) - r1) < 1e-8;
					if (r1 < r /*&& eqDist*/)
					{
						r = r1;
						bestJ = j;
						center = p + (n * r);
						atMiddle = false;
						atLeft = false;
						bestZIC = zeroInsideCircle;
						//dbg if (i == 97) drawCircle(Circle(center, r));
						//dbg drawCircle(Circle(center, r));
					}
				}
			}
		} //~for(;;)

		  // 3. retrun
		if (r == r_init)
		{
			//no update was done. -> just means that no arcs exist on this side.
			return { -1, 0.0 };
		}

		if (flag_findMTD_concave_check)
		{
			if (!convex && r >= arcs[i].cr() - 1e-8)
			{
				return { i, 1.0 };
			}
		}

		/*dbg if (i == 97)
		{
			Circle c(center, r);
			drawCircle(c);
		}*/

		if (atMiddle)
		{
			//dbg cout << bestJ << " " << i << " " << t << endl;
			// 3-1. if in middle, find t
			Point normal = center - arcs[bestJ].c.c;
			normal = normal / sqrt(normal*normal);
			double t;
			if (flag_vmqa)
			{
				t = arcNormalToParameter(arcs[bestJ], normal);
			}
			else
			{
				t = arcNormalToParameter(arcs[bestJ].n[0], arcs[bestJ].n[1], normal);
			}
			if (PRINT_ERRORS)
			{
				auto denominator = acos(arcs[bestJ].n[0] * arcs[bestJ].n[1]);
				auto numer = acos(arcs[bestJ].n[0] * normal);
				if (piececnt == 205 && cur_depth == 3)
					cerr << "in findMaximal, t " << t << " " << arcs[bestJ].n[0] << arcs[bestJ].n[1] << " " << denominator << normal << " " << numer << " " << numer/denominator << " " << arcs[bestJ].n[0] * normal << endl;
			}

			dbg if (PRINT_ERRORS && arcs[bestJ].n[0].P[0] < -0.9939 && arcs[bestJ].n[0].P[0] > -0.9940 && normal.P[0] > 0.993587 && normal.P[0] < 0.993591)
			{
				auto j = bestJ;
				bool normalInArc;
				if (arcs[j].ccw)
					normalInArc = ((arcs[j].n[0] ^ normal) >= 0) && ((normal ^ arcs[j].n[1]) >= 0);
				else
					normalInArc = ((arcs[j].n[0] ^ normal) <= 0) && ((normal ^ arcs[j].n[1]) <= 0);
				cout << "i j nia" << i << " " << j << " " << normalInArc << endl;
				cout << "ccw 0^n n^1 " << arcs[j].ccw << " " << (arcs[j].n[0] ^ normal) << " " << (normal ^ arcs[j].n[1]) << endl;
			}

			return { bestJ, t };
		}
		else
		{
			if (PRINT_ERRORS && ERR_ARC_END_POINTS)
			{
				Point p2, v2;
				if (atLeft)
				{
					p2 = arcs[bestJ].x[0];
					v2 = arcs[bestJ].n[0];
				}
				else
				{
					p2 = arcs[bestJ].x[1];
					v2 = arcs[bestJ].n[1];
				}
				auto l0 = getLineEquationFromPointVector(p, v);
				auto l1 = getLineEquationFromPointVector(p2, v2);
				auto inter = getLineIntersectionPoint(l0, l1);
				cerr << "ERROR : arc end points were returned in findMaximal... sqdist from line inter point " << (inter-v).length() << " " << (inter-v2).length() << endl;

			}
			//above may actually be possible when tangent happens really close to endpoints...

			// 3-2. if left or right, just return 0 or 1
			if (atLeft)
			{
				return { bestJ, 0.0 };
			}
			else
			{
				return { bestJ, 1.0 };
			}
		}

	}

	/* Def : error when b is bifurcation point of size3 vector c
		subroutine of recursivelyFindBisector
	*/
	double computeBifurcationPointError(Point& b, double r, vector<CircularArc> c)
	{



		double sign0 = c[0].ccw ? -1 : 1;
		double sign1 = c[1].ccw ? -1 : 1;
		double sign2 = c[2].ccw ? -1 : 1;

		auto dist0 = sqrt((b - c[0].c.c).length());
		auto dist1 = sqrt((b - c[1].c.c).length());
		auto dist2 = sqrt((b - c[2].c.c).length());

		auto true0 = c[0].c.r + sign0*r;
		auto true1 = c[1].c.r + sign1*r;
		auto true2 = c[2].c.r + sign2*r;

		auto e0 = true0 - dist0;
		auto e1 = true1 - dist1;
		auto e2 = true2 - dist2;


		dbg if (bifurcnt == 1)
		{
			cerr << "CCWs " << c[0].ccw << c[1].ccw << c[2].ccw << endl;
			cerr << "rad "
				<< " " << c[0].c.r
				<< " " << c[1].c.r
				<< " " << c[2].c.r << endl;

			cerr << "b r " << b << " " << r << endl;
			cerr << "d t " << dist0 << " " << true0 << endl;
			cerr << "d t " << dist1 << " " << true1 << endl;
			cerr << "d t " << dist2 << " " << true2 << endl;
		}



		return e0*e0 + e1*e1 + e2*e2;
		
	}

	/* Def: a point p (with normal v) is knwon to share a maximal disk with point q. Find radius of the disk
	Assume: it should be already knwon that they share a disk.
			v is normalized, but sign doesn't matter
	*/
	double getTouchingDiskRadius(Point p, Point q, Point v)
	{
		auto pq = p - q;
		auto len = sqrt(pq.length());
		//if (len == 0) return 0;
		pq = pq / len;
		auto cosine = fabs(pq * v);
		if (PRINT_ERRORS)
		{
			if (len == 0) cerr << "ERROR: in getTOuchingDiskRadius, p q are same point" << endl;
			if (cosine == 0) cerr << "ERROR: isosceles with 90 angle" << endl;
			if (isnan(len * 0.5 / cosine))
			{
				cerr << "ERROR: in getTouchingDiskRad, Nan returned. len cosine " << len << " " << cosine << endl;
				cerr << "     : p q v " << p << q << v << endl;
			}
		}

		return len * 0.5 / cosine;
	}

	/* Def: similar to above, but uses lines intersections... & returns center coord
	*/
	Point getTouchingDiskCenter(Point p, Point q, Point v)
	{

		auto dir = p - q;
		auto len = (p - q).length();
		if (len < 1e-80) return (p+q)*0.5;
		dir = dir / sqrt(len);
		auto l0 = getLineEquationFromPointVector(p, v);
		auto l1 = getLineEquationFromPointVector((p + q)*0.5, rotate_p90(dir));
		auto ret = getLineIntersectionPoint(l0, l1);
		if (isnan(ret.P[0])) //errorcase;
			return (p + q)*0.5;
		return ret;

	}


	/* Def: given a set of circularArcs, known to produce some bisectors in the original input, compute it.
		currently draws the bisector
	*/
	void recursivelyFindBisector(vector<CircularArc> cycle, vector<double> color, int depth = 0)
	{

		// 1. check cases and return if necessary
		if (depth >= 20) return;
		if (cycle.size() == 3 && depth >= 15)
		{
			if (epsilon(cycle[0]) || epsilon(cycle[1]) || epsilon(cycle[2]))
				return;

			/*dbg
				cerr << cycle[0].c.c << endl
				<< cycle[1].c.c << endl
				<< cycle[2].c.c << endl << endl;
			_dbg*/

			auto p0 = getTouchingDiskCenter(cycle[0].x[1], cycle[1].x[0], cycle[0].n[1]);
			auto p1 = getTouchingDiskCenter(cycle[1].x[1], cycle[2].x[0], cycle[1].n[1]);
			auto p2 = getTouchingDiskCenter(cycle[2].x[1], cycle[0].x[0], cycle[2].n[1]);

			auto mid0 = cycle[0](0.5);
			auto mid1 = cycle[1](0.5);
			auto mid2 = cycle[2](0.5);
			
			auto l0 = getLineEquationFromPointVector((mid0 + mid1)*0.5, rotate_p90(mid0 - mid1));
			auto l1 = getLineEquationFromPointVector((mid2 + mid1)*0.5, rotate_p90(mid2 - mid1));
			
			auto q = getLineIntersectionPoint(l0, l1);
			if (isnan(q.P[0]))
			{
				dbg return;
				q = p0;
			}

			if (drawVoronoiSingleBranch)
			{
				glBegin(GL_LINES);
				glColor3f(1, 1, 0);
				glVertex2dv(p0.P);
				glVertex2dv(q.P);
				glVertex2dv(p1.P);
				glVertex2dv(q.P);
				glVertex2dv(p2.P);
				glVertex2dv(q.P);
				glEnd();
			}
			if (planning::output_to_file::flag)
			{
				// bifur point
				planning::output_to_file::bifur_point p;
				p.p = q;
				for (int i = 0; i < 3; i++)
				{
					p.idx.push_back(cycle[i].originalIndex);
				}
				planning::output_to_file::bifur_points[t2].push_back(p);

				// v_edge x 3
				{
					planning::output_to_file::v_edge v;
					v.v0 = p0;
					v.v1 = q;
					v.idx[0] = cycle[0].originalIndex;
					v.idx[1] = cycle[1].originalIndex;
					planning::output_to_file::v_edges[t2].push_back(v);
				}
				{
					planning::output_to_file::v_edge v;
					v.v0 = p1;
					v.v1 = q;
					v.idx[0] = cycle[1].originalIndex;
					v.idx[1] = cycle[2].originalIndex;
					planning::output_to_file::v_edges[t2].push_back(v);
				}
				{
					planning::output_to_file::v_edge v;
					v.v0 = p2;
					v.v1 = q;
					v.idx[0] = cycle[2].originalIndex;
					v.idx[1] = cycle[0].originalIndex;
					planning::output_to_file::v_edges[t2].push_back(v);
				}
			}
			return;
		}
		if (cycle.size() == 3 && false  /*&& depth >= 15*/)
		{
			if (epsilon(cycle[0]) || epsilon(cycle[1]) || epsilon(cycle[2]))
				return;
			//
			//auto p0 = (cycle[0].x[1] + cycle[1].x[0]) * 0.5;
			//auto p1 = (cycle[1].x[1] + cycle[2].x[0]) * 0.5;
			//auto p2 = (cycle[2].x[1] + cycle[0].x[0]) * 0.5;
			//
			//auto mid0 = cycle[0](0.5);
			//auto mid1 = cycle[1](0.5);
			//auto mid2 = cycle[2](0.5);
			//
			//auto l0 = getLineEquationFromPointVector((mid0 + mid1)*0.5, rotate_p90(mid0 - mid1));
			//auto l1 = getLineEquationFromPointVector((mid2 + mid1)*0.5, rotate_p90(mid2 - mid1));
			//
			//auto q = getLineIntersectionPoint(l0, l1);
			//if (isnan(q.P[0]))
			//	q = p0;
			//
			//glBegin(GL_LINES);
			//glVertex2dv(p0.P);
			//glVertex2dv(q.P);
			//glVertex2dv(p1.P);
			//glVertex2dv(q.P);
			//glVertex2dv(p2.P);
			//glVertex2dv(q.P);
			//glEnd();

			// 0. check degeneracy and take care
			{
				// case 1 : circular arcs are from same

			}

			// 1. find bifurcation point
			Point bifur(0, 0);
			double R;
			bool success = true; //default, true. whenever sth fails, changed to false & next steps are skipped -> skipped ones just gets subdiv now
			{
				// + if 
				// # := +- 
				// $ := -+
				// 3 unkown x, y, R (point, R)
				// a0^2 - 2a0x + x^2 + b0^2 - 2b0y + y^2 = r0^2 + 2 * s0 * r0R + R^2 ... (1)
				// a1^2 - 2a1x + x^2 + b1^2 - 2b1y + y^2 = r1^2 + 2 * s1 * r1R + R^2 ... (2)
				// a2^2 - 2a2x + x^2 + b2^2 - 2b2y + y^2 = r2^2 + 2 * s2 * r2R + R^2 ... (3)
				//
				// (a0^2 - a1^2) -2(a0 - a1)x + (b0^2 - b1^2) -2(b0 - b1)y = (r0^2 - r1^2) + 2(s0r0-s1r1)R ...(4)
				// (a1^2 - a2^2) -2(a1 - a2)x + (b1^2 - b2^2) -2(b1 - b2)y = (r1^2 - r2^2) + 2(s1r1-s2r2)R ...(5)
				//
				// [ 2(a0 - a1) 2(b0 - b1) ] [x]  =  [ -2(s0r0-s1r1)R + {(a0^2 - a1^2) + (b0^2 - b1^2) - (r0^2 - r1^2)}
				// [ 2(a1 - a2) 2(b1 - b2) ] [y]     [ -2(s1r1-s2r2)R +	{(a1^2 - a2^2) + (b1^2 - b2^2) - (r1^2 - r2^2)} ... (6)
				// Az = cR + d; [x, y] = [f(r), g(r)]
				// if 3 circle centers are in a straight line... (do something else)
				//
				// Q) plugging x, y in (6) to (1)~(3) will give two solutions of R... it will give one pos & one negative i guess...? -> since solution for (s0,s1,s2) case r0,r1 has duality with solution for (-s0,-s1,-s2)'s -r0,-r1 
				//

				// 1-1. first check if three circle centers are colinear.
				bool isColinear;
				{
					double det = (cycle[0].c.c - cycle[1].c.c).normalize() ^ (cycle[1].c.c - cycle[2].c.c).normalize();
					isColinear = fabs(det) < 1e-20;
					if(isColinear) success = false;
				}

				if (success) // TODO? when colinear
				{
					// 1-2. make x, y = cr + d

					// 1-2-1. find sign ($ above)
					double sign0 = cycle[0].ccw ? -1 : 1;
					double sign1 = cycle[1].ccw ? -1 : 1;
					double sign2 = cycle[2].ccw ? -1 : 1;

					// 1-2-2. apply inv mat
					double
						&a0 = cycle[0].c.c.P[0], &b0 = cycle[0].c.c.P[1], &r0 = cycle[0].c.r,
						&a1 = cycle[1].c.c.P[0], &b1 = cycle[1].c.c.P[1], &r1 = cycle[1].c.r,
						&a2 = cycle[2].c.c.P[0], &b2 = cycle[2].c.c.P[1], &r2 = cycle[2].c.r;

					auto det = (a0 - a1) * (b1 - b2) - (b0 - b1) * (a1 - a2);
					auto c0 = -(b1 - b2) * (sign0 * r0 - sign1 * r1) + (b0 - b1) * (sign1 * r1 - sign2 * r2); c0 /= det;
					auto c1 = +(a1 - a2) * (sign0 * r0 - sign1 * r1) - (a0 - a1) * (sign1 * r1 - sign2 * r2); c1 /= det;
					auto e0 = +(a0 - a1) * (a0 + a1) + (b0 - b1) * (b0 + b1) - (r0 - r1) * (r0 + r1);
					auto e1 = +(a1 - a2) * (a1 + a2) + (b1 - b2) * (b1 + b2) - (r1 - r2) * (r1 + r2);
					auto d0 = +(b1 - b2) * e0 - (b0 - b1) * e1; d0 /= 2 * det;
					auto d1 = -(a1 - a2) * e0 + (a0 - a1) * e1; d1 /= 2 * det;

					// 1-3. solve (1) (= aR^2+bR+c) with x = c0*R + d0, y = c1*R + d1;
					// TODO : can save some inst by using b/2 = b'
					d0 -= a0;
					d1 -= b0;

					double
						a = c0*c0 + c1*c1 - 1 * 1,
						b = 2 * c0*d0 + 2 * c1*d1 - 2 * sign0*r0,
						c = d0*d0 + d1*d1 - r0*r0;
					d0 += a0;
					d1 += b0;

					if (PRINT_ERRORS)
					{
						if (a == 0 && b == 0)
							cerr << "ERROR : quadratic 0*r^2 + 0*r + c = 0" << endl;
						//cerr << "bifur a b c bb-4ac" << a << " "  << b << " " <<  c << " " << b*b-4*a*c << endl;
						if (b*b - 4 * a*c < 0)
						{
							//// case when bb-4ac <0
							////dbg_draw
							//glLineWidth(20);
							//cycle[0].draw();
							//cycle[1].draw();
							//cycle[2].draw();
							//glLineWidth(2);
							///*cycle[0].c.draw();
							//cycle[1].c.draw();
							//cycle[2].c.draw();*/
							//cerr << "bb-4ac < 0 case : ***********************" << endl;
							//cerr << cycle[0] << cycle[1] << cycle[2] << endl;
						}
					}

					if (fabs(a) < 1e-100)
					{
						R = -c / b;
						bifur = Point(c0*R + d0, c1*R + d1);
					}
					else if (b*b - 4*a*c >= 0)
					{
						auto 
							R0 = ((-b + sqrt(b*b - 4 * a*c)) / a) * 0.5,
							R1 = ((-b - sqrt(b*b - 4 * a*c)) / a) * 0.5;
						auto
							bifur0 = Point(c0*R0 + d0, c1*R0 + d1),
							bifur1 = Point(c0*R1 + d0, c1*R1 + d1);
						/*
						auto
							error0 = computeBifurcationPointError(bifur0, R0, cycle),
							error1 = computeBifurcationPointError(bifur1, R1, cycle);
						*/
						double error0=0, error1=0;
						// TODO : optimize
						{
							auto& b = bifur0;
							auto& r = R0;
							auto& error = error0;

							Point v0 = b - cycle[0].c.c; v0.normalize();
							Point v1 = b - cycle[1].c.c; v1.normalize();
							Point v2 = b - cycle[2].c.c; v2.normalize();

							{
								auto& c = cycle[0];
								if (!isNormalBetween(c.n[0], c.n[1], v0))
								{
									auto temp0 = v0 * c.n[0];
									auto temp1 = v0 * c.n[1];
									if (temp0 > temp1) error += 1 - temp0;
									else error += 1 - temp1;
								}
								if (PRINT_ERRORS && piececnt == 204)
								{
									// why 180 case?
									cerr << "why 180 case : n0 n1 v0 " << c.n[0] << " " << c.n[1] << " " << v0 << endl;
								}
							}
							{
								auto& c = cycle[1];
								if (!isNormalBetween(c.n[0], c.n[1], v0))
								{
									auto temp0 = v1 * c.n[0];
									auto temp1 = v1 * c.n[1];
									if (temp0 > temp1) error += 1 - temp0;
									else error += 1 - temp1;
								}
								if (PRINT_ERRORS && piececnt == 204)
								{
									// why 180 case?
									cerr << "why 180 case : n0 n1 v0 " << c.n[0] << " " << c.n[1] << " " << v0 << endl;
								}
							}
							{
								auto& c = cycle[2];
								if (!isNormalBetween(c.n[0], c.n[1], v0))
								{
									auto temp0 = v2 * c.n[0];
									auto temp1 = v2 * c.n[1];
									if (temp0 > temp1) error += 1 - temp0;
									else error += 1 - temp1;
								}
								if (PRINT_ERRORS && piececnt == 204)
								{
									// why 180 case?
									cerr << "why 180 case : n0 n1 v0 " << c.n[0] << " " << c.n[1] << " " << v0 << endl;
								}
							}

						}
						// TODO : optimize
						{
							auto& b = bifur1;
							auto& r = R1;
							auto& error = error1;

							Point v0 = b - cycle[0].c.c; v0.normalize();
							Point v1 = b - cycle[1].c.c; v1.normalize();
							Point v2 = b - cycle[2].c.c; v2.normalize();

							{
								auto& c = cycle[0];
								if (!isNormalBetween(c.n[0], c.n[1], v0))
								{
									auto temp0 = v0 * c.n[0];
									auto temp1 = v0 * c.n[1];
									if (temp0 > temp1) error += 1 - temp0;
									else error += 1 - temp1;
								}
								if (PRINT_ERRORS && piececnt == 204)
								{
									// why 180 case?
									cerr << "why 180 case : n0 n1 v0 " << c.n[0] << " " << c.n[1] << " " << v0 << endl;
								}
							}
							{
								auto& c = cycle[1];
								if (!isNormalBetween(c.n[0], c.n[1], v0))
								{
									auto temp0 = v1 * c.n[0];
									auto temp1 = v1 * c.n[1];
									if (temp0 > temp1) error += 1 - temp0;
									else error += 1 - temp1;
								}
								if (PRINT_ERRORS && piececnt == 204)
								{
									// why 180 case?
									cerr << "why 180 case : n0 n1 v0 " << c.n[0] << " " << c.n[1] << " " << v0 << endl;
								}
							}
							{
								auto& c = cycle[2];
								if (!isNormalBetween(c.n[0], c.n[1], v0))
								{
									auto temp0 = v2 * c.n[0];
									auto temp1 = v2 * c.n[1];
									if (temp0 > temp1) error += 1 - temp0;
									else error += 1 - temp1;
								}
								if (PRINT_ERRORS && piececnt == 204)
								{
									// why 180 case?
									cerr << "why 180 case : n0 n1 v0 " << c.n[0] << " " << c.n[1] << " " << v0 << endl;
								}
							}

						}
						
						
						if (error0 < error1)
						{
							R = R0;
							bifur = bifur0;
						}
						else
						{
							R = R1;
							bifur = bifur1;
						}


					}
					else success = false;
					// 1-4. compute point of bifurcation

				}

			}

			// 1.5. draw bifur circle for testing
			if (drawBifurCircle && success && bifurcnt<2000)
			{
				cerr << "test : drawing bifur circles..." << endl;
				bifurcnt++;
				drawCircle(Circle(bifur, R));
			}

			// 2. from bifur point subdiv
			if (success)
			{
				// 2-1. find sbdiv point
				auto n0 = bifur - cycle[0].c.c;
				auto n1 = bifur - cycle[1].c.c;
				auto n2 = bifur - cycle[2].c.c;

				n0.normalize();
				n1.normalize();
				n2.normalize();


				/*
				Check whether new normal is between...
				*/

				bool err = false;
				if (!isNormalBetween(cycle[0].n[0], cycle[0].n[1], n0)) err = true;
				if (!isNormalBetween(cycle[1].n[0], cycle[1].n[1], n1)) err = true;
				if (!isNormalBetween(cycle[2].n[0], cycle[2].n[1], n2)) err = true;

				if (err && PRINT_ERRORS)
				{
					cerr << "ERROR : bifurcation case, normal not between " << endl;
					cerr << "{{{{" << endl;
					cerr << cycle[0] << endl << "CCW " << cycle[0].ccw << endl;
					cerr << cycle[1] << endl << "CCW " << cycle[1].ccw << endl;
					cerr << cycle[2] << endl << "CCW " << cycle[2].ccw << endl;
					cerr << "bifur " << bifur << "    r " << R << endl;
					cerr << "piececnt" << piececnt << "  depth" << depth << endl;
					cerr << "cycle0 " << "center_dist R+r r-R " << sqrt((bifur-cycle[0].c.c).length()) << " " << R + cycle[0].c.r << " " << cycle[0].c.r - R << " n0 n1 n " << cycle[0].n[0] << cycle[0].n[1] << n0 << endl;
					cerr << "cycle1 " << "center_dist R+r r-R " << sqrt((bifur-cycle[1].c.c).length()) << " " << R + cycle[1].c.r << " " << cycle[1].c.r - R << " n0 n1 n " << cycle[1].n[0] << cycle[1].n[1] << n1 << endl;
					cerr << "cycle2 " << "center_dist R+r r-R " << sqrt((bifur-cycle[2].c.c).length()) << " " << R + cycle[2].c.r << " " << cycle[2].c.r - R << " n0 n1 n " << cycle[2].n[0] << cycle[2].n[1] << n2 << endl;
					cerr << "}}}}" << endl;

					////dbg_draw
					//cycle[0].draw();
					//cycle[1].draw();
					//cycle[2].draw();
					//glBegin(GL_LINES);
					//glVertex2dv(cycle[0].c.c.P);
					//glVertex2dv(bifur.P);
					//glVertex2dv(cycle[1].c.c.P);
					//glVertex2dv(bifur.P);
					//glVertex2dv(cycle[2].c.c.P);
					//glVertex2dv(bifur.P);
					//glEnd();
				}

				if (err) success = false;

				if (success)
				{
					// 2-2. subdiv to 3 of single branch case
					vector<CircularArc> v0(2);
					vector<CircularArc> v1(2);
					vector<CircularArc> v2(2);

					v0[0] = cycle[0];
					v0[1] = cycle[1];
					v1[0] = cycle[1];
					v1[1] = cycle[2];
					v2[0] = cycle[2];
					v2[1] = cycle[0];

					v0[0].n[0] = n0;
					v0[1].n[1] = n1;
					v1[0].n[0] = n1;
					v1[1].n[1] = n2;
					v2[0].n[0] = n2;
					v2[1].n[1] = n0;

					v0[0].x[0] = v0[0].c.r * n0;
					v0[1].x[1] = v0[1].c.r * n1;
					v1[0].x[0] = v1[0].c.r * n1;
					v1[1].x[1] = v1[1].c.r * n2;
					v2[0].x[0] = v2[0].c.r * n2;
					v2[1].x[1] = v2[1].c.r * n0;

					// 2-3. set color
					vector<double> c0(2);
					vector<double> c1(2);
					vector<double> c2(2);

					c0[0] = color[0];
					c0[1] = color[1];
					c1[0] = color[1];
					c1[1] = color[2];
					c2[0] = color[2];
					c2[1] = color[0];

					// 2-4. recursive call

					//reverse(v0.begin(), v0.end());
					//reverse(v1.begin(), v1.end());
					//reverse(v2.begin(), v2.end());

					dbg	if (false)
					{
						auto l0 = getLineEquationFromPointVector(cycle[0].x[1], cycle[0].n[1]);
						auto l1 = getLineEquationFromPointVector(cycle[1].x[0], cycle[1].n[0]);
						auto l2 = getLineEquationFromPointVector(cycle[1].x[1], cycle[1].n[1]);
						auto l3 = getLineEquationFromPointVector(cycle[2].x[0], cycle[2].n[0]);
						auto l4 = getLineEquationFromPointVector(cycle[2].x[1], cycle[2].n[1]);
						auto l5 = getLineEquationFromPointVector(cycle[0].x[0], cycle[0].n[0]);

						auto p0 = getLineIntersectionPoint(l0, l1);
						auto p1 = getLineIntersectionPoint(l2, l3);
						auto p2 = getLineIntersectionPoint(l4, l5);

						//override
						p0 = (cycle[0].x[1] + cycle[1].x[0]) * 0.5;
						p1 = (cycle[1].x[1] + cycle[2].x[0]) * 0.5;
						p2 = (cycle[2].x[1] + cycle[0].x[0]) * 0.5;

						auto q = bifur;

						glBegin(GL_LINES);
						glVertex2dv(p0.P);
						glVertex2dv(q.P);
						glVertex2dv(p1.P);
						glVertex2dv(q.P);
						glVertex2dv(p2.P);
						glVertex2dv(q.P);
						glEnd();

						cerr << "p0 p1 p2" << p0 << p1 << p2 << endl;
					}

					dbg case2from = 1;
					recursivelyFindBisector(v0, c0, 0);
					recursivelyFindBisector(v1, c1, 0);
					recursivelyFindBisector(v2, c2, 0);
					dbg case2from = 0;

					return;
				}
			}
		}
		if (cycle.size() == 2)
		{
			if (epsilon(cycle[0]) || epsilon(cycle[1]))
				return;

			// 1-1. sample some points on input curve.
			auto p00 = cycle[0].x[0]; 
			auto p01 = cycle[0].x[1];
			auto p10 = cycle[1].x[0];
			auto p11 = cycle[1].x[1];

			// 1-2. endpoints of bisector
			auto p = getTouchingDiskCenter(cycle[0].x[1], cycle[1].x[0], cycle[0].n[1]);
			auto q = getTouchingDiskCenter(cycle[0].x[0], cycle[1].x[1], cycle[0].n[0]);

			if (PRINT_ERRORS)
			{
				//check if max touch circle center distance is diff?
				auto l0 = q - cycle[0].x[0];
				auto l1 = p - cycle[0].x[1];
				auto l2 = p - cycle[1].x[0];
				auto l3 = q - cycle[1].x[1];
				auto len0 = l0.length();
				auto len1 = l1.length();
				auto len2 = l2.length();
				auto len3 = l3.length();
				auto diff0 = fabs(len0 - len3);
				auto diff1 = fabs(len1 - len2);
				if (diff0 > 1e-8 || diff1 > 1e-8)
				{
					cerr << "ERROR : in recFB, distance from circle center diff!!! pcnt dpth c2from diff0 diff1 " << piececnt << " " << depth << " " << case2from << " " << diff0 << " " << diff1 << endl;
				}
				if (p*p > 4 || q*q > 4)
					cerr << "ERROR : in recFB, out of boundary " << len0 << " " << len1 << " " << len2 << " " << len3 << endl;
				/*if (piececnt == 48)
				{
					cerr << "TEST : at pcnt48 diff0 diff1 " << diff0 << " " << diff1 << endl;
					cerr << "     : " << p << q << endl;
				}*/

			}

			/*
			if (PRINT_ERRORS)
			{
				glBegin(GL_LINES);
				glVertex2dv(p.P);
				glVertex2dv(q.P);
				glEnd();
			}*/

			// 1-3. check terminal condition
			bool terminal = false;

			if (sqlength(p00-p11)< rfbTerminationEps) {
				//tag oscrfb
	/*			Point normal;
				if (cycle[0].ccw)
					p = cycle[0].c.c;
				else
					p = cycle[0].x[0] + cycle[0].c.r * cycle[0].n[0];
				swap(p, q);*/
				//draw(p);
				terminal = true;
			}
			else if (sqlength(p01 - p10)<rfbTerminationEps) {
				//tag oscrfb
				/*Point normal;
				if (cycle[0].ccw)
					q = cycle[0].c.c;
				else
					q = cycle[0].x[1] + cycle[0].c.r * cycle[0].n[1];*/
				//draw(q);
				terminal = true;
			}
			else if (sqlength(p00 - p01)<rfbTerminationEps)
				terminal = true;
			else if (sqlength(p10 - p11)<rfbTerminationEps)
				terminal = true;


			// 1-4. if (terminal) draw it approx & return
			if (sqlength(p-q)<rfbTerminationEps2 || terminal) {// || terminal){
				//toc(); // do not count drawing time

				if (drawVoronoiSingleBranch)
				{
					if (case2from && PRINT_ERRORS)
						glColor3f(1, 1, 0);
					else if (piececnt == 48 && PRINT_ERRORS)
						glColor3f(0, 0, 0);
					else
						glColor3f(1, 0, 1);
					glBegin(GL_LINES);
					glVertex2dv(p.P);
					glVertex2dv(q.P);
					glEnd();
					glColor3f(0, 0.7, 0);
					lineSegCnt++;

					if (PRINT_ERRORS)
					{
						//7,7,261 prob -> 4070
						if (sqlength(p) > 4 || sqlength(q) > 4)
						{
							cerr << "ERROR : out_of_boundary case in findMaximal : piececnt p q " << piececnt << " " << p << q << endl;
							////dbg_draw
							//cycle[0].draw();
							//cycle[1].draw();
						}
					}
				}
				if (planning::output_to_file::flag)
				{
					planning::output_to_file::v_edge v;
					v.v0 = p;
					v.v1 = q;
					v.idx[0] = cycle[0].originalIndex;
					v.idx[1] = cycle[1].originalIndex;
					planning::output_to_file::v_edges[t2].push_back(v);
				}
				//tic();
				return;
			}

			// 1-5. if the endpoints of a curve are too close, return.
			if (epsilon(cycle[0]) || epsilon(cycle[1]))
				return;
		}

		// 2. if not returned above, subdiv
		if (true)
		{
			if (PRINT_ERRORS)
				cur_depth = depth;

			/*dbg if(cycle.size() > 2)
			{
				int	sampleArc = 0;
				double cur_max = -1;
				for (int i = 0; i < cycle.size(); i++)
				{
					double chord_length = (cycle[i].x[0] - cycle[i].x[1]).length();
					if (chord_length > cur_max)
					{
						cur_max = chord_length;
						sampleArc = i;
					}
				}
				if (sampleArc != 0)
				{
					vector<CircularArc> newCyc;
					newCyc.push_back(cycle[sampleArc]);
					for (int i = 0; i < cycle.size(); i++)
					{
						if (i != sampleArc)
							newCyc.push_back(cycle[i]);
					}
					cycle.swap(newCyc);
				}
			}*/

			auto foot = findMaximalDiskSharingPoint(cycle, poc{ 0 , 0.5 }, 0, 0);

			if (PRINT_ERRORS)
			{
				/*auto & temp = cycle[0], &temp2 = cycle[foot.c;
				if (temp.n[0].length < 1e-100 ||
					temp.n[1].length < 1e-100) 1;*/
				if (depth == 3 && piececnt == 212 && false)
				{
					glLineWidth(20.0f);
					glColor3f(0, 0, 1);
					cerr << "error case 205/4" << endl;
					for (size_t i = 1; i < cycle.size(); i++)
					{
						cycle[i].draw();
					}
					glColor3f(0, 0.7, 0);
					glLineWidth(2.0f);

					glLineWidth(30.0f);
					glColor3f(1, 1, 0);
					cycle[0].draw();
					glColor3f(0, 0.7, 0);
					glLineWidth(2.0f);
				}
			}

			 // Error case when curve[0] == some other curve above
			if (foot.c < 1) {
				if (PRINT_ERRORS && ERR_FOOT_C_BELOW_1)
					cerr << "ERROR : foot.c < 1 at recursivelyFindBisector piececnt depth case2from " << piececnt << " " << depth << " " << case2from << endl
						 << "      : foot.c, cycle.size() " << " " << foot.c << " " <<cycle.size() << endl;
				//if (PRINT_ERRORS &&piececnt == 205 && depth == 4)
				//{
				//	/*
				//		ERROR : foot.c < 1 at recursivelyFindBisector piececnt depth 205 4
				//		      : foot.c, cycle.size()  -1 4
				//	*/
				//	// dbg_draw
				//	cerr << "test: drawing case 205 4 \n";
				//	auto geom = getPointGeometry(cycle[0], 0.5);
				//	glBegin(GL_LINES);
				//	glVertex2dv(geom.x.P);
				//	glVertex2dv((geom.x + geom.v).P);
				//	glVertex2dv(geom.x.P);
				//	glVertex2dv((geom.x - geom.v).P);
				//	glVertex2dv(geom.x.P);
				//	glVertex2dv((geom.x + geom.n).P);
				//	glEnd();
				//	for (auto i : cycle)
				//		i.draw();
				//}

				if (PRINT_ERRORS && cycle.size() == 2)
				{
					//when it fails to 
					/*
					for thes cases:
						ERROR : foot.c < 1 at recursivelyFindBisector piececnt depth 217 0
						      : foot.c, cycle.size()  -1 2
					*/
					auto r0 = getTouchingDiskRadius(cycle[0].x[0], cycle[1].x[1], cycle[0].n[0]);
					auto r1 = getTouchingDiskRadius(cycle[1].x[0], cycle[0].x[1], cycle[1].n[0]);
					auto p0 = (cycle[0].x[0] + r0 * (cycle[0].ccw ? -1 : 1) * cycle[0].n[0]) - cycle[1].x[1];
					auto p1 = (cycle[1].x[0] + r1 * (cycle[1].ccw ? -1 : 1) * cycle[1].n[0]) - cycle[0].x[1];
					auto d0 = sqrt(p0.length());
					auto d1 = sqrt(p1.length());
					cerr << "when findDisk failed & cycle = 2 : r0 d0 r1 d1 " << r0 << " " << d0 << " " << r1 << " " << d1 << endl;


				}

				if (PRINT_ERRORS && dbgcnt < 1)
				{
					dbgcnt++;
				}
				return;
			}

			if (PRINT_ERRORS)
			{
				if (foot.t < -1e-32 || foot.t > 1 + 1e-32)
					cerr << "ERROR : in recursivelyFindBisector, foot point out of [0,1]" << endl;
			}


			auto first = subdivCircularArc(cycle[0], 0.5); // subdiv curve[0] at t = 0.5
			auto second = subdivCircularArc(cycle[foot.c], foot.t); // subdiv some other curve, where it shares a disk with curve[0] at t = 0.5

			if (PRINT_ERRORS)
			{
				if (piececnt == 205 && depth == 3)
					cerr << "foot.t " << foot.t << endl;
			}

			vector<CircularArc> sub[2];
			vector<double> subcolor[2];
			// build [0] : from right segment of curve[0] ~ left seg of other curve
			{
				for (int i = 1; i < foot.c; i++) {
					sub[0].push_back(cycle[i]);
					subcolor[0].push_back(color[i]);
				}
				sub[0].push_back(second.first);			//curve[other]'s left
				subcolor[0].push_back(color[foot.c]);
				sub[0].push_back(first.second);			//curve[0]'s right
				subcolor[0].push_back(color[0]);
			}
			// build [1] : leftovers from [0]
			{
				for (size_t i = foot.c + 1; i < cycle.size(); i++) {
					sub[1].push_back(cycle[i]);
					subcolor[1].push_back(color[i]);
				}
				sub[1].push_back(first.first);	//curve[0]'s left
				subcolor[1].push_back(color[0]);
				sub[1].push_back(second.second);//curve[other]'s right
				subcolor[1].push_back(color[foot.c]);
			}

			recursivelyFindBisector(sub[0], subcolor[0], depth + 1);
			recursivelyFindBisector(sub[1], subcolor[1], depth + 1);

			if (PRINT_ERRORS)
			{
				if (depth == 2)
				{
					for (int i = 0; i< 2; i++)
					if (sub[i].size() == 3)
					{
						if ((sub[i][2].x[1] - Point(0.389704, 0.621707)).length() < 1e-8)
						{
							/*cerr << "err case ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
							glLineWidth(10.0f);
							glColor3f(0, 0, 1);
							cerr << "subdiv errcase cycle.size() = " << cycle.size() << endl;
							for (size_t i = 0; i < cycle.size(); i++)
							{
								cycle[i].draw();
							}
							glColor3f(0, 0.7, 0);
							glLineWidth(2.0f);*/

							cout << "weird point, depth " << cycle[0].x[0] << depth << endl; //  (0.389066, 0.623063) 2
						}
					}
				}

				if (depth == 1)
				{
					for (int i = 0; i< 2; i++)
						if (true)
						{
							if ((sub[i][0].x[0] - Point(0.389066, 0.623063)).length() < 1e-8)
							{
								/*cerr << "err case2 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
								glLineWidth(10.0f);
								glColor3f(0, 0, 1);
								cerr << "subdiv errcase cycle.size() = " << cycle.size() << endl;
								for (size_t i = 0; i < cycle.size(); i++)
								{
									cycle[i].draw();
								}
								glColor3f(0, 0.7, 0);
								glLineWidth(2.0f);*/

								cout << "weird point, depth " << cycle[0].x[0] << depth << endl; // (0.382202, 0.614886) 1
							}
						}
				}
				if (depth == 0)
				{
					for (int i = 0; i< 2; i++)
						if (true)
						{
							if ((sub[i][0].x[0] - Point(0.382202, 0.614886)).length() < 1e-8)
							{
								cerr << "err case3 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
								/*glLineWidth(10.0f);
								glColor3f(0, 0, 1);
								cerr << "subdiv errcase cycle.size() = " << cycle.size() << endl;
								for (size_t i = 0; i < 1; i++)
								{
									cycle[i].draw();
								}
								glColor3f(0, 0.7, 0);
								glLineWidth(2.0f);*/

								cout << "weird point, depth " << cycle[0].x[0] << depth << endl; // (0.382202, 0.614886) 1
								// cycle[1] is the long thing.
							}
						}
				}

				
			}
		}
		else // use depth % cycle.size() instead of 0 for sampling foot
		{
			

			if (PRINT_ERRORS)
				cur_depth = depth;

			if (cycle.size() == 1) return;
			
			// def : sampleArc : arc to be subdivided at 0.5
			int sampleArc = depth % cycle.size();
			sampleArc = 0;
			double cur_max = -1;
			for (int i = 0 ; i < cycle.size(); i++)
			{
				double chord_length = (cycle[i].x[0] - cycle[i].x[1]).length();
				if (chord_length > cur_max)
				{
					cur_max = chord_length;
					sampleArc = i;
				}
			}


			auto foot = findMaximalDiskSharingPoint(cycle, poc{ sampleArc , 0.5 }, 0, 0);


			// Error case when curve[0] == some other curve above
			if (foot.c == -1 || foot.c == sampleArc) {
				if (PRINT_ERRORS)
					cerr << "Unable to find optimal arc" << endl;
				return;
			}

			if (PRINT_ERRORS)
			{
				if (foot.t < -1e-32 || foot.t > 1 + 1e-32)
					cerr << "ERROR : in recursivelyFindBisector, foot point out of [0,1]" << endl;
			}

			vector<CircularArc> sub[2];
			vector<double> subcolor[2];
			pair<CircularArc, CircularArc> first, second;
			int fi, si;

			if (foot.c > sampleArc)
			{
				first  = subdivCircularArc(cycle[sampleArc], 0.5); // subdiv curve[0] at t = 0.5
				second = subdivCircularArc(cycle[foot.c], foot.t); // subdiv some other curve, where it shares a disk with curve[0] at t = 0.5
				fi = sampleArc;
				si = foot.c;
			}
			else
			{
				first  = subdivCircularArc(cycle[foot.c], foot.t);
				second = subdivCircularArc(cycle[sampleArc], 0.5);
				fi = foot.c;
				si = sampleArc;
			}

			// build [0] : from right segment of curve[0] ~ left seg of other curve
			{
				for (int i = fi + 1; i < si; i++) {
					sub[0].push_back(cycle[i]);
					subcolor[0].push_back(color[i]);
				}
				sub[0].push_back(second.first);			//curve[other]'s left
				subcolor[0].push_back(color[foot.c]);
				sub[0].push_back(first.second);			//curve[0]'s right
				subcolor[0].push_back(color[sampleArc]);
			}
			// build [1] : leftovers from [0]
			{
				for (size_t i = si + 1; i < cycle.size(); i++) {
					sub[1].push_back(cycle[i]);
					subcolor[1].push_back(color[i]);
				}
				for (size_t i = 0; i < fi; i++) {
					sub[1].push_back(cycle[i]);
					subcolor[1].push_back(color[i]);
				}
				sub[1].push_back(first.first);	//curve[0]'s left
				subcolor[1].push_back(color[sampleArc]);
				sub[1].push_back(second.second);//curve[other]'s right
				subcolor[1].push_back(color[foot.c]);
			}

			recursivelyFindBisector(sub[0], subcolor[0], depth + 1);
			recursivelyFindBisector(sub[1], subcolor[1], depth + 1);

		}
	}

	/* Def:
		Perform Medial Axis Transformation
	*/
	dbg int lineSegCnt;
	void _Medial_Axis_Transformation(VR_IN& INPUT in)
	{
		//dbg stuff
		dbg bifurcnt = 0;
		dbg lineSegCnt = 0;

		// 0. alias
		auto& spiral = in.arcs;
		auto& left   = in.left;
		auto& color  = in.color;

		// 1. build transition & piece
		map<pointOnCurve, pointOnCurve>
			transition,
			piece;
		{
			vector<set<double>> domain(spiral.size());

			// 1-1. build domain & transition
			for (int i = 0, length = spiral.size(); i < length; i++)
			{
				auto foot = findMaximalDiskSharingPoint(spiral, pointOnCurve{ i, 0.0 }, left[i], i);

				//dbg if (i == 97) cout << "closest to 97 is " << foot.c << endl;

				// 1-2. build domain
				domain[i].insert(0);
				domain[i].insert(1);
				if (foot.c >= 0)
					domain[foot.c].insert(foot.t);
				dbg else if (PRINT_ERRORS)
				{
					cerr << "ERROR : foot.c < 0 while building domain/transition" << endl;
				}
				// 1-2-2. set origIdx for later use
				spiral[i].originalIndex = i;

				// 1-3. build transition
				if (PRINT_ERRORS)
				{
					if (transition.find(poc{ left[i], 1 }) != transition.end()) cerr << "ERROR : while building transition, overwriting happened" << endl;
					if (transition.find(foot) != transition.end()) cerr << "ERROR : while building transition, overwriting happened" << endl;

					// case50 of cycle==1
					if (left[i] == 46) cerr << "poc(46, 1)'s foot : " << foot.c << " " << foot.t << "    at i = " << i << "   foot.t -1 = " << foot.t - 1  << endl;
				}
				transition[poc{left[i], 1}] = foot;
				transition[foot] = poc{ i, 0 };

				if (drawTransition && foot.c >= 0)
				{
					auto p = spiral[i].x[0];
					auto q = getPointGeometry(spiral[foot.c], foot.t).x;
					auto mid = getTouchingDiskCenter(p, q, spiral[i].n[0]);
					glBegin(GL_LINES);
					glColor3f(0, 0.3, 0.7);
					glVertex2dv(p.P);
					glVertex2dv(mid.P);
					glColor3f(0, 0.8, 0.2);
					glVertex2dv(q.P);
					glVertex2dv(mid.P);
					glEnd();

					if (PRINT_ERRORS)
					{
						// dbg_draw
						// tag0544 - 1
						if ((mid - Point(1.35744, 0.0685376)).length() < 1e-4)
						{
							auto r = (p - mid).length();
							r = sqrt(r);
							/*Circle(mid, r).draw();*/
						}
					}
				}
			}

			// 1-4. build piece
			for (int i =0, length = spiral.size(); i < length; i++) {
				auto& d = domain[i];
				for (auto j = d.begin(), k = ++d.begin(); k != d.end(); ++j, ++k) {
					piece[poc{ i, *j }] = poc{ i, *k };
				}
			}
		}

		// dbg : why no cycle.size == 1
		dbg
			auto piece2 = piece;
		//// dbg_draw
		//{

		//	auto arc = spiral[46];
		//	arc.draw();
		//	glBegin(GL_LINES);
		//	glVertex2dv(arc.c.c.P);
		//	glVertex2dv(arc.x[0].P);
		//	glVertex2dv(arc.c.c.P);
		//	glVertex2dv(arc.x[1].P);
		//	glEnd();
		//}

		_dbg

		// 2.
		piececnt = -1;
		while (!piece.empty())
		{
			dbg piececnt++;

			vector<CircularArc> cycle;
			vector<double> col;

			auto begin = *piece.begin();
			piece.erase(begin.first);
			auto left  = begin.first;
			auto right = begin.second;

			//dbg cout << left.c << endl;
			if (PRINT_ERRORS)
			{
				//if (piececnt == 49 || piececnt == 50 || piececnt == 294 || piececnt == 295)
				//{
				//	/*
				//	ERROR : foot.c < 1 at recursivelyFindBisector piececnt depth case2from 49 0 0
				//	: foot.c, cycle.size()  -1 1
				//	*/
				//	auto new_left = transition[right];
				//	auto new_next = piece2.find(new_left);
				//	auto originally_existed = new_next != piece2.end();
				//	cerr << "CASE " << piececnt << " : left, right, orig_exist " << left.c << " " << left.t << " " << right.c << " " << right.t << " " << originally_existed << endl;
				//	//dbg_draw
				//	if (piececnt == 50)
				//	{

				//	}
				//}
			}

			if (PRINT_ERRORS) // to track depth
				cur_depth = -1;

			//if (PRINT_ERRORS && piececnt == 48 && drawBoundary) //case where line end doesn't meet
			//{
			//	cerr << "case line-end : left.c left.t right.t r-l " << left.c << " " << left.t << " " << right.t << " " << right.t - left.t << endl;
			//	//dbg_draw
			//	auto arc = subdivCircularArc(spiral[left.c], left.t, right.t);
			//	auto geom0 = getPointGeometry(arc, 0);
			//	auto geom1 = getPointGeometry(arc, 1);

			//	arc.draw();
			//	glBegin(GL_LINES);
			//	glColor3f(1, 1, 0);
			//	glVertex2dv(geom0.x.P);
			//	glVertex2dv((geom0.x + geom0.n).P);
			//	//glColor3f(0, 0, 1);
			//	//glVertex2dv(geom1.x.P);
			//	//glVertex2dv((geom1.x + geom1.n).P);
			//	glEnd();
			//}

			cycle.push_back(subdivCircularArc(spiral[left.c], left.t, right.t)); // note that left.first == right.first
			col.push_back(color[left.c]);

			for (int i = 0;; i++) 
			{
				left = transition[right];
				auto next = piece.find(left);

				if (next == piece.end())
					break;
				
				right = next->second;
				piece.erase(left);

				//if (PRINT_ERRORS && piececnt == 48 && drawBoundary) //case where line end doesn't meet
				//{
				//	cerr << "case line-end : left.c left.t right.t r-l " << left.c << " " << left.t << " " << right.t << " " << right.t - left.t << endl;
				//	//dbg_draw
				//	auto arc = subdivCircularArc(spiral[left.c], left.t, right.t);
				//	auto geom0 = getPointGeometry(arc, 0);
				//	auto geom1 = getPointGeometry(arc, 1);

				//	arc.draw();
				//	glBegin(GL_LINES);
				//	//glColor3f(1, 0, 0);
				//	//glVertex2dv(geom0.x.P);
				//	//glVertex2dv((geom0.x + geom0.n).P);
				//	glColor3f(0, 1, 0);
				//	glVertex2dv(geom1.x.P);
				//	glVertex2dv((geom1.x + geom1.n).P);
				//	glEnd();
				//}

				//if (PRINT_ERRORS) // same case above
				//{
				//	Point p(-1.26725, 0.501537), q(-1.26083, 0.500384); 
				//	glBegin(GL_LINES);
				//	glColor3f(0, 1, 1);
				//	glVertex2dv(p.P);
				//	glVertex2dv(q.P);
				//	glEnd();
				//}

				cycle.push_back(subdivCircularArc(spiral[left.c], left.t, right.t));
				col.push_back(color[left.c]);

				if (PRINT_ERRORS && false)
				{
					if (piececnt == 47)
					{
						glLineWidth(10.0f);
						glColor3f(0, 0, 1);
						spiral[left.c].draw();
						glColor3f(0, 0.7, 0);
						glLineWidth(2.0f);
					}
				}
			}

			dbg dbgmode = 0;	
			if (PRINT_ERRORS && false)
			{
				if (false && cycle.size() == 5)
				{
					if ((cycle[0].x[0] - Point(0.382346, 0.636719)).length() < 1e-8)
					{
						cerr << "piececnt : " << piececnt << endl;
					}
				}

				glLineWidth(20.0f);
				glColor3f(1, 0, 1);
				for (auto i : cycle)
					i.draw();
				glColor3f(0, 0.7, 0);
				glLineWidth(2.0f);
			}
			if (PRINT_ERRORS && piececnt == 4070)
			{
				cerr << "ERROR : 4070 case : cycle.size() " << cycle.size() << endl;
			}
			if (PRINT_ERRORS && piececnt == 1145 && cycle.size() > 3)
			{
				// tag0544 dbg_draw
				{
					cycle[0].draw();
					cycle[1].draw();
					cycle[2].draw();

					cerr << "TEST : tag0544 " << endl;
					testValidCycle(cycle);
				}
			}

			recursivelyFindBisector(cycle, col); // usually called 700 times?
		}
	}


}

///*****************************************************************************************************************************
//**																															**
//**												  ~~~~~~~~~~~~~~~~~~~~~~													**
//**												  ~~Voronoi Calculator~~													**
//**												  ~~~~~~~~~~~~~~~~~~~~~~													**
//**																															**
//** Author : Mingyu Jung																										**
//** Def: An integrated Class which keeps information or byproduct during Voronoi Diagram calculation.						**
//**																															**
//** Other API used :																											**
//**	CircularArc :																											**
//**	Point : length2()																										**
//**	PointOnCurve : 																											**
//**	VoronoiEdge : v0, v1, idx[0], idx[1]																					**
//**  ETC : 																													**
//**	 subdivCircularArc, findMaximalDiskSharingPoint, 																		**
//**																															**
//** Assume :																													**
//**	All objects form a loop (of circular Arcs). The inside/outisde should be distinguishable								**
//**	Objects that will have voronoiEdges outside of it should be parameterized clock-wise									**
//**																															**
//** How to use :																												**
//**	1. construct class instance : voronoiCalculator vc;																		**
//**	2. vc.initialize();																										**
//**	3. vc.setInput(...);																									**
//**	4. vc.setOutput(...);																									**
//**	5. (optional) if(!vc.isProperInput()) ...																				**
//**	6. vc.calculate() (or other subroutines)																				**
//**	7. ... vc.getOutput() ...																								**
//**																															**
//** About Subroutine naming:																									**
//** 	Build____ : builds some internal variable/structrue/class that this class contains or has a pointer to.					**
//** 	Find_____ : finds some value or structure value. Does not alter contents of this class.									**
//** 																															**
//** TODO:																													**
//**	clean up findBisectorRecursively																						**
//*****************************************************************************************************************************/
//
//constexpr bool VORONOI_VERBOSE_ERRORS = false;
//
//using pointOnCurve	= planning::pointOnCurve;
//using poc			= pointOnCurve;
//using CircularArc	= ms::CircularArc;
//using VoronoiEdge	= planning::output_to_file::v_edge;
//
//using retTypeFBR	= std::deque<VoronoiEdge>;
//
//class voronoiCalculator
//{
//public:
//	/* main API */
//	bool
//		isInputProper();
//	void
//		initialize();
//	void
//		calculate();
//
//	/* I/O */
//	void
//		voronoiCalculator::setInput(std::vector<CircularArc>& arcs, std::vector<int>& leftArcIdx, std::vector<double>& loopIdx);
//	void
//		voronoiCalculator::setOutput(std::vector<retTypeFBR>& output);
//
//	/* Access functions to I/O */
//	inline std::vector<CircularArc>&
//		getArcs()	 { return *_arcs; }
//	inline std::vector<int>&
//		getLeft()	 { return *_leftArc; }
//	inline std::vector<double>&
//		getLoopIdx() { return *_loopIdx; }
//	inline std::vector<retTypeFBR>&
//		getOutput()  { return *_output; }
//
//	/* Access functions to byproducts */
//	inline std::vector<std::set<double>>&
//		getDomains()	 { return _domains; }
//	inline std::map<poc,poc>&
//		getTransitions() { return _transitions; }
//	inline std::map<poc,poc>&
//		getPieces()		 { return _pieces; }
//	inline std::vector<std::vector<CircularArc>>&
//		getCycles()		 { return _cycles; }
//
//	/* Constructor Destructor*/
//	inline
//		voronoiCalculator() = default;
//	virtual
//		~voronoiCalculator() = default;
//
//	/* sub routines */
//	poc  
//		findTransitionPoint(poc p, int left, int right);
//	retTypeFBR
//		findBisectorRecursively(std::vector<CircularArc>& cycle, int depth = 0);
//
//	void buildTransitions();
//	void buildPieces();
//	void buildCycles();
//	void buildVoronoiEdges();
//
//private:
//	
//	/* INPUT */
//	std::vector<CircularArc>* 
//		_arcs;
//	std::vector<int>*
//		_leftArc;
//	std::vector<double>*
//		_loopIdx;
//
//	/* OUTPUT */
//	std::vector<retTypeFBR>*
//		_output;
//
//	/* Intermeditae data*/
//	std::map<poc, poc>
//		_pieces,
//		_transitions;
//	std::vector<std::set<double>>
//		_domains;
//	std::vector<std::vector<CircularArc>>
//		_cycles;
//	// type _cycle; ???
//	
//	/* FLAGS and flags-handling functions */
//	bool
//		_flags[6];
//
//	static constexpr int
//		_flagPtrSet				= 0,
//		_flagDomainsBuilt		= 1,
//		_flagTransitionsBuilt	= 2,
//		_flagPiecesBuilt		= 3,
//		_flagCyclesBuilt		= 4,
//		_flagVoronoiEdgesBuilt	= 5;
//
//	inline bool& _isPtrSet()			{ return _flags[_flagPtrSet];			 }
//	inline bool& _isDomainsBuilt()		{ return _flags[_flagDomainsBuilt];		 }
//	inline bool& _isTransitionsBuilt()	{ return _flags[_flagTransitionsBuilt];  }
//	inline bool& _isPiecesBuilt()		{ return _flags[_flagPiecesBuilt];		 }
//	inline bool& _isCyclesBuilt()		{ return _flags[_flagCyclesBuilt];		 }
//	inline bool& _isVoronoiEdgesBuilt() { return _flags[_flagVoronoiEdgesBuilt]; }
//
//	/* simple helper funcs and options used during code*/
//	double _optEpsArcCriteria = 1e-18;
//
//	inline bool _EpsArc(CircularArc& c)
//	{
//		/* Def : returns true when arc is very small */
//		return ((c.x0() - c.x1()).length2() < _optEpsArcCriteria);
//	}
//};

/*
Def :
	Checks whether the input is proper(does not fix it)
Return :
	False IF there is some error(true doesn't mean that input will always succeed calc) 
Summary :
	1. pointer
*/
bool 
	voronoiCalculator::isInputProper()
{
	// 1. check I/O ptr
	if(	_arcs	 == nullptr ||
		_leftArc == nullptr ||
		_loopIdx == nullptr ||
		_output	 == nullptr		)
		return false;
}

/*
Def : 
	Initializes variable (to Nullptr... false...)
Warning :
	Access prviate variables directly instead of get___().
*/
void 
	voronoiCalculator::initialize()
{
	_arcs	 = nullptr;
	_leftArc = nullptr;
	_loopIdx = nullptr;
	_output	 = nullptr;

	_pieces.clear(),
	_transitions.clear();
	_domains.clear();
	_cycles.clear();

	for (auto& f : _flags)
		f = false;
}

/*
Def : 
	Finds the voronoi edges. (and store it to _output)
Desc :
	find voronoiEdges as a set(soup of edges)
*/
void 
	voronoiCalculator::calculate()
{
	//// 0. alias
	//auto& arcs	= getArcs();
	//auto& left	= getLeft();
	//auto& loop	= getLoopIdx();
	//
	//auto& transition = getTransitions();
	//auto& piece = getPieces();

	if (_output == nullptr)
	{
		std::cerr << "!!! ERROR : voronoiCalculator.calculate : output pointer not set . !!!" << std::endl;
	}

	// 1. build transition & piece
	buildTransitions();
	// 2.
	buildPieces();
	// 3.
	buildCycles();
	// 4.
	buildVoronoiEdges();
}

/*
Def :
	Finds the voronoi edges and topology
Desc : 
	instead of returning a soup ( like calculate() ),
	find the structure of the graph and return it.
	Terms used:
		curves : just a vector<vEdge>. They will form a curve.
		segment curves : curves that comes out from evaluating a cycle.
		voronoi curve : sum of some segCurves. So that the endpoints connect vertices in the graph structure.
*/
void
voronoiCalculator::calculateG()
{
	double pointDiffError = 1e-12;
	double pointDiffError2 = 1e-12;
	//// 0. alias
	//auto& arcs	= getArcs();
	//auto& left	= getLeft();
	//auto& loop	= getLoopIdx();
	//
	//auto& transition = getTransitions();
	//auto& piece = getPieces();

	if (_outputG == nullptr)
	{
		std::cerr << "!!! ERROR : voronoiCalculator.calculate : output pointer not set . !!!" << std::endl;
	}

	// 1. build transition & piece
	buildTransitions();
	// 2.
	buildPieces();
	// 3.
	buildCycles();

	// 4. set vcResultG ready
	auto& out = getOutputG();
	auto& cycles = getCycles();
	out.segmentCurves.clear();
	out.segmentCurves.resize(cycles.size());

	//dbg_out
	std::cout << "dbg out : cycles.size()" << cycles.size() << std::endl;

	// 5. fill segmentcurves
	for (size_t i = 0; i < cycles.size(); i++)
	{
		out.currentCycleNo = i;
		out.arcPairToLocalIdx.clear();
		std::vector<int> ci;
		for (int j = 0; j < cycles[i].size(); j++)
			ci.push_back(j);
		buildCalculateGSegCurves(cycles[i], ci, 0);
	}


	////err check dbg_out
	//{
	//	using namespace std;
	//	int cnt = 0;
	//	for(auto& a : out.segmentCurves)
	//		for (auto& b : a)
	//		{
	//			cnt+=b.size();
	//		}
	//	cout << "CNT : " << cnt << endl;
	//}

	// 6. sort segment curves; and store it to temp;
	std::deque<std::deque<VoronoiEdge>> temp;
	for (auto& curvesInACycle : out.segmentCurves)
	{
		for (auto& singleCurve : curvesInACycle)
		{
			temp.push_back({});
			temp.back().swap(sortVectorVoronoiEdge(singleCurve, pointDiffError));

		}
	}

	// err check dbg_out
	// check whether sorted correctly
	{
		using namespace std;

		int count = 0;
		double max2 = -1.0;
		for (auto& sc : temp)
		{
			double max = -1.0;
			for (int i = 0; i < sc.size() - 1; i++)
			{
				auto& v0 = sc[i];
				auto& v1 = sc[i+1];

				auto err = (v0.v1 - v1.v0).length2();
				if (err > max)
					max = err;
			}
			if (max2 < max)
				max2 = max;

			count++;
		}
		cout << "checking v-edge sort. max err : " << max2 << endl;
	}
	// ~dbg : conclusion : max err < 1e-13 => prob seems to be in merging
	
	// 7. mark endpoints of curves, if they are bifur; (so that it is not connected at 8)
	std::deque<std::pair<int, int>> mark;
	mark.resize(temp.size());
	for (int i = 0; i < temp.size(); i++)
	{
		auto& p = temp[i].front().v0;
		auto& q = temp[i].back().v1;

		mark[i].first = -1;
		mark[i].second = -1;
		for (size_t j = 0; j < out.bifurcationPoints.size(); j++)
		{
			//if ( p == out.bifurcationPoints[j]) // bug_fixed : use 1e-12 instead of 1e-16
			if ((p - out.bifurcationPoints[j]).length2() < pointDiffError2)
				mark[i].first = j;
			if ((q - out.bifurcationPoints[j]).length2() < pointDiffError2)
				mark[i].second = j;
		}
	}

	// 8. find neighbors
	int offset = 100000000; // 2 << 28; // to make difference btw curve/bifurPoint
	//int offset2 = 2 << 27; //
	double maxErr = 1e-4;
	double diffErr = pointDiffError *1.0e-1;
	double errMult = 9.0;
	while(diffErr < maxErr)
	{
		for (size_t i = 0; i < temp.size(); i++)
		{
			// for left part of temp[i]
			if (mark[i].first == -1)
			{
				auto& x = temp[i].front().v0;
				for (size_t j = i + 1; j < temp.size(); j++)
				{
					auto& p = temp[j].front().v0;
					if ((p - x).length2() < diffErr && mark[j].first == -1)
					{
						mark[i].first = offset + j;
						mark[j].first = offset + i;
						break;
					}

					auto& q = temp[j].back().v1;
					if ((q - x).length2() < diffErr && mark[j].second == -1)
					{
						mark[i].first = offset + j;
						mark[j].second = offset + i;
						break;
					}
				}
			}

			// for right part of temp[i]
			if (mark[i].second == -1)
			{
				auto& x = temp[i].back().v1;
				for (size_t j = i + 1; j < temp.size(); j++)
				{
					auto& p = temp[j].front().v0;
					if ((p - x).length2() < diffErr && mark[j].first == -1)
					{
						mark[i].second = offset + j;
						mark[j].first = offset + i;
						break;
					}

					auto& q = temp[j].back().v1;
					if ((q - x).length2() < diffErr && mark[j].second == -1)
					{
						mark[i].second = offset + j;
						mark[j].second = offset + i;
						break;
					}
				}
			}
		}
		diffErr = errMult * diffErr;
	}

	// 9. merge lists
	// used to compute danling point idx
	int offset3 = out.bifurcationPoints.size();
	int nDangling = 0;
	// alias for saved data;
	std::deque<std::deque<VoronoiEdge>>& E = out.voronoiCurves;
	std::deque<std::pair<int, int>>& E2 = out._E2V;
	// to check redundancy;
	std::vector<bool> used(temp.size(), false);

	//// dbg_out
	//for (auto i : mark)
	//	std::cout << i.first << " " << i.second << std::endl;
	//// ~dbg_out

	for(int i = 0; i<temp.size(); i++)
	{
		if (used[i]) continue;
		used[i] = true;
		auto& m = mark[i];

		// 9-1. check left (keep connecting segcurves to the left side)
		int lastLeft = i;
		while (m.first >= offset)
		{
			// 9-1-1. find neighbor
			int neigh = m.first - offset;
			if (used[neigh])break; //dbg
			used[neigh] = true;

			// 9-1-2. check if it should be attached right away or flipped.
			// if(my leftmost part is connected to the right-side-of-neigh)
			if (mark[neigh].second - offset == lastLeft)
			{
				////dbg_out
				//auto& p = temp[neigh].back().v1;
				//auto& q = temp[i].front().v0;
				//if ((p - q).length2() > maxErr)
				//	std::cout << "!!! merging arr error at 9-1-2-1 with err value : " << (p - q).length2() << std::endl;
				////~dbg
				temp[i].insert(temp[i].begin(), temp[neigh].begin(), temp[neigh].end());
				m.first = mark[neigh].first;
				lastLeft = neigh;
			}
			// if(my leftmost part is connected to the left-side-of-neigh)
			else
			{
				////dbg_out
				//auto& p = temp[neigh].back().v1;
				//auto& q = temp[i].front().v0;
				//if ((p - q).length2() > maxErr)
				//	std::cout << "!!! merging arr error at 9-1-2-2 with err value : " << (p - q).length2() << std::endl;
				////~dbg
				
				//for (auto it = temp[neigh].rbegin(); it != temp[neigh].rend(); it++)
				for (auto it = temp[neigh].begin(); it != temp[neigh].end(); it++)
				{
					temp[i].push_front(it->flip());
				}
				//temp[i].insert(temp[i].begin(), temp[neigh].rbegin(), temp[neigh].rend());
				m.first = mark[neigh].second;
				lastLeft = neigh;
			}

		}

		// 9-2. check right
		int lastRight = i;
		while (m.second >= offset)
		{
			// 9-2-1. find neighbor
			int neigh = m.second - offset;
			if (used[neigh])break; //dbg
			used[neigh] = true;

			// 9-2-2. check if it should be attached right away or flipped.
			if (mark[neigh].first - offset == lastRight)
			{
				temp[i].insert(temp[i].end(), temp[neigh].begin(), temp[neigh].end());
				m.second = mark[neigh].second;
				lastRight = neigh;
			}
			else
			{
				for (auto it = temp[neigh].rbegin(); it != temp[neigh].rend(); it++)
				{
					temp[i].push_back(it->flip());
				}
				//temp[i].insert(temp[i].end(), temp[neigh].rbegin(), temp[neigh].rend());
				m.second = mark[neigh].first;
				lastRight = neigh;
			}

		}

		//dbg
		if (m.first >= offset)
			m.first = -1;
		if (m.second >= offset)
			m .second = -1;
		//~dbg

		// 9-3. now both are connected to a bifur or/dangling
		//if (m.first < offset && m.second < offset) ==> no need to check
		{
			// 9-3-1. check dangling
			if (m.first == -1)
			{
				// i. check if there is some bifurcation point close enough

				m.first = offset3 + nDangling;
				nDangling++;
				out.danglingPoints.push_back(temp[i].front().v0);
			}

			// 9-3-1.
			if (m.second == -1)
			{
				m.second = offset3 + nDangling;
				nDangling++;
				out.danglingPoints.push_back(temp[i].back().v1);
			}
		}

		// 9-4. add to final
		//if (m.first > -1 && m.second > -1 && m.first < offset && m.second < offset) //dbg
		{
			E2.push_back(m);
			E.push_back({});
			E.back().swap(temp[i]);
		}
	}

	// 10. final stuff: build V and V2E
	out.allPoints.insert(out.allPoints.end(), out.bifurcationPoints.begin(), out.bifurcationPoints.end());
	out.allPoints.insert(out.allPoints.end(), out.danglingPoints.begin(), out.danglingPoints.end());

	out._V2E.resize(out.allPoints.size());
	for (auto& a : out._V2E)
		a.resize(out.allPoints.size(), -1);

	for (size_t i = 0; i < E.size(); i++)
	{
		auto& m = out._E2V[i];
		out._V2E[m.first][m.second] = i;
		out._V2E[m.second][m.first] = i;
	}

}


/* 
Def : 
	a subroutine of calculateG, which build Vedge lists;
*/
void
	voronoiCalculator::buildCalculateGSegCurves(std::vector<CircularArc>& cycle, std::vector<int>& cycleIdx, int depth)
{
	// need to erase these.
	using namespace planning;
	using namespace std;


	// 1. check cases and return if necessary
	if (cycle.size() < 2) return;

	// 1-1. recursive depth enough
	// not this case
	if (depth >= 20)
		return;

	// 1-2. bifur && dpeth enough
	if ((cycle.size() == 3 && depth >= 15) || (cycle.size() >= 3 && depth == 18))
	{
		if (_EpsArc(cycle[0]) || _EpsArc(cycle[1]) || _EpsArc(cycle[2]))
			return;

		// 1-2-1. find q (bifurcation point)
		
		auto mid0 = cycle[0](0.5);
		auto mid1 = cycle[1](0.5);
		auto mid2 = cycle[2](0.5);
		
		auto l0 = getLineEquationFromPointVector((mid0 + mid1)*0.5, rotate_p90(mid0 - mid1));
		auto l1 = getLineEquationFromPointVector((mid2 + mid1)*0.5, rotate_p90(mid2 - mid1));
		
		auto q = getLineIntersectionPoint(l0, l1);
		if (isnan(q.P[0]))
		{
			dbg return;
		}

		// 1-2-2. save bifur
		_outputG->bifurcationPoints.push_back(q);
		_outputG->bifurcationPointToCycle[q] = _outputG->currentCycleNo;

		// 1-2-3. save voronoiEdge
		for (int i = 0; i < cycle.size(); i++)
		{
			// i. find v-edge to insert
			VoronoiEdge v;
			auto iNext = i + 1;
			{
				if (iNext == cycle.size()) iNext = 0;
				auto pn = getTouchingDiskCenter(cycle[i].x[1], cycle[iNext].x[0], cycle[i].n[1]);

				v.v0 = pn;
				v.v1 = q;
				v.idx[0] = cycle[i].originalIndex;
				v.idx[1] = cycle[iNext].originalIndex;
			}

			// ii. find place to insert (localIdx);
			std::pair<int, int> key; // key should be (smallerNo, largerNo) 
			int localIdx;
			{
				// ii-1. make key
				if (cycleIdx[i] < cycleIdx[iNext])
				{
					key.first = cycleIdx[i];
					key.second = cycleIdx[iNext];
				}
				else
				{
					key.first = cycleIdx[iNext];
					key.second = cycleIdx[i];
				}

				// ii-2. search for key
				auto& searchRes = _outputG->arcPairToLocalIdx.find(key);
				if (searchRes == _outputG->arcPairToLocalIdx.end())
				{
					// update map if key didn't exist;
					localIdx = _outputG->segmentCurves[_outputG->currentCycleNo].size();
					_outputG->arcPairToLocalIdx[key] = localIdx;
					_outputG->segmentCurves[_outputG->currentCycleNo].push_back(vector<VoronoiEdge>());
				}
				else
				{
					localIdx = searchRes->second;
				}
			}

			// iii.do insertion
			_outputG->segmentCurves[_outputG->currentCycleNo][localIdx].push_back(v);

		}

		return;
	}
	
	if (cycle.size() == 2)
	{
		if (_EpsArc(cycle[0]) || _EpsArc(cycle[1]))
			return;

		// 1-1. sample some points on input curve.
		auto p00 = cycle[0].x[0]; 
		auto p01 = cycle[0].x[1];
		auto p10 = cycle[1].x[0];
		auto p11 = cycle[1].x[1];

		// 1-2. endpoints of bisector
		auto p = getTouchingDiskCenter(cycle[0].x[1], cycle[1].x[0], cycle[0].n[1]);
		auto q = getTouchingDiskCenter(cycle[0].x[0], cycle[1].x[1], cycle[0].n[0]);

		// 1-3. check terminal condition
		bool terminal = false;
		{
		if (sqlength(p00-p11)< rfbTerminationEps) {
			terminal = true;
		}
		else if (sqlength(p01 - p10)<rfbTerminationEps) {
			terminal = true;
		}
		else if (sqlength(p00 - p01)<rfbTerminationEps)
			terminal = true;
		else if (sqlength(p10 - p11)<rfbTerminationEps)
			terminal = true;
		}

		// 1-4. if (terminal && vEdge length is short enough) return it
		if (sqlength(p-q)<rfbTerminationEps2 || terminal) {
			// i. find v-edge to insert
			VoronoiEdge v;
			int i = 0, iNext = 1;
			{
				v.v0 = p;
				v.v1 = q;
				v.idx[0] = cycle[0].originalIndex;
				v.idx[1] = cycle[1].originalIndex;
			}

			// ii. find place to insert (localIdx);
			std::pair<int, int> key; // key should be (smallerNo, largerNo) 
			int localIdx;
			{
				// ii-1. make key
				if (cycleIdx[i] < cycleIdx[iNext])
				{
					key.first = cycleIdx[i];
					key.second = cycleIdx[iNext];
				}
				else
				{
					key.first = cycleIdx[iNext];
					key.second = cycleIdx[i];
				}

				// ii-2. search for key
				auto& searchRes = _outputG->arcPairToLocalIdx.find(key);
				if (searchRes == _outputG->arcPairToLocalIdx.end())
				{
					// update map if key didn't exist;
					localIdx = _outputG->segmentCurves[_outputG->currentCycleNo].size();
					_outputG->arcPairToLocalIdx[key] = localIdx;
					_outputG->segmentCurves[_outputG->currentCycleNo].push_back(vector<VoronoiEdge>());
				}
				else
				{
					localIdx = searchRes->second;
				}
			}

			// iii.do insertion
			_outputG->segmentCurves[_outputG->currentCycleNo][localIdx].push_back(v);
			return;
		}

		// 1-5. if the endpoints of a curve are too close, return. No more subdiv
		if (_EpsArc(cycle[0]) || _EpsArc(cycle[1]))
			return;
	}

	// 2. if not returned above, subdiv
	if (true)
	{
		auto foot = findMaximalDiskSharingPoint(cycle, poc{ 0 , 0.5 }, 0, 0);

		auto first  = subdivCircularArc(cycle[0]	 , 0.5);	// subdiv curve[0] at t = 0.5
		auto second = subdivCircularArc(cycle[foot.c], foot.t); // subdiv some other curve, where it shares a disk with curve[0] at t = 0.5

		vector<CircularArc> sub[2];
		vector<int> subCI[2];
		if (cycle.size() == 2)
		{
			// build [0] :
			{
				sub[0].push_back(first.second);			//curve[0]'s right
				sub[0].push_back(second.first);			//curve[other]'s left
				subCI[0].push_back(cycleIdx[0]);
				subCI[0].push_back(cycleIdx[foot.c]);
			}
			// build [1] :
			{
				sub[1].push_back(first.first);	//curve[0]'s left
				sub[1].push_back(second.second);//curve[other]'s right
				subCI[1].push_back(cycleIdx[0]);
				subCI[1].push_back(cycleIdx[foot.c]);
			}
		}
		else 
		{
			// build [0] : from right segment of curve[0] ~ left seg of other curve
			{
				for (int i = 1; i < foot.c; i++) {
					sub[0].push_back(cycle[i]);
					subCI[0].push_back(cycleIdx[i]);
				}
				sub[0].push_back(second.first);			//curve[other]'s left
				subCI[0].push_back(cycleIdx[foot.c]);
				sub[0].push_back(first.second);			//curve[0]'s right
				subCI[0].push_back(cycleIdx[0]);
			}
			// build [1] : leftovers from [0]
			{
				for (size_t i = foot.c + 1; i < cycle.size(); i++) {
					sub[1].push_back(cycle[i]);
					subCI[1].push_back(cycleIdx[i]);
				}
				sub[1].push_back(first.first);	//curve[0]'s left
				subCI[1].push_back(cycleIdx[0]);
				sub[1].push_back(second.second);//curve[other]'s right
				subCI[1].push_back(cycleIdx[foot.c]);
			}
		}

		buildCalculateGSegCurves(sub[0], subCI[0], depth + 1);
		buildCalculateGSegCurves(sub[1], subCI[1], depth + 1);

		return ;

	}
}

/*
Def:
	Register Input vector to calculator
*/
void
	voronoiCalculator::setInput( std::vector<CircularArc>& arcs, std::vector<int>& leftArcIdx, std::vector<double>& loopIdx)
{
	_arcs	 = &(arcs);
	_leftArc = &(leftArcIdx);
	_loopIdx = &(loopIdx);


	if (_arcs	 != nullptr &&
		_leftArc != nullptr &&
		_loopIdx != nullptr)
		_isPtrSet() = true;
}

/*
Def : 
	Register a vector which will receive the output.
*/
void
	voronoiCalculator::setOutput(std::vector<retTypeFBR>& output)
{
	_output = &output;
}

/*
Def : set output for the graph type output case;
*/
void
	voronoiCalculator::setOutputG(voronoiCalculatorResultG& outputG)
{
	_outputG = &outputG;
}


/*
Def : 
	Find a point which shares a maximal touching disk with "p"
Return : 
	The pair-point represented with pointOnCurve
Assume : 
	_isPtrSet() == true;
Parameter : 
	p : point of interest
	left  : index of arc, which shares p.arc.x0()
	right : index of arc, which shares p.arc.x1()
		Or intuitively, you can put arc idx which you don't want to test against p
*/
poc 
	voronoiCalculator::findTransitionPoint(poc p, int left, int right)
{
	// currently just calls findMaximalDiskSharingPoint
	return findMaximalDiskSharingPoint(getArcs(), p, left, right);
}

/*
Def : Build _Transition and _Domains
Desc :
	A Domain contains info about how an arcs t = [0,1] should be divided.
	A Transition tells us at a point (in the domain set built above) where to go, following the cycle. (Hard to explain)
*/
void
	voronoiCalculator::buildTransitions()
{

	// 1. Error check;
	if (!_isPtrSet()) // this only checks input ptr;
	{
		std::cerr << "!!!!!! ERROR : voronoiCalculator : isPtrSet == false !!!!!!" << std::endl;
		return;
	}
	_isDomainsBuilt() = true;
	_isTransitionsBuilt() = true;

	// 2. Alias
	auto& arcs = getArcs();
	auto& left = getLeft();
	auto& loop = getLoopIdx();

	auto& transitions	= getTransitions();
	auto& domains		= getDomains();
	domains.resize(arcs.size());

	// 2-1. build right;
	std::vector<int> right(left.size(), -1);
	{
		for (int i = 0; i< left.size(); i++)
		{
			int l = left[i];
			if (l != -1)
			{
				right[l] = i;
			}
		}
	}

	// 3. Build Domains & Transitions
	// for(each arc)
	for (int i = 0, length = arcs.size(); i < length; i++)
	{
		// 3-1. find paired point (of an arc's left point)
		auto foot = findTransitionPoint(pointOnCurve{ i, 0.0 }, left[i], i);

		// 3-2. build domain (insert * 3)
		domains[i].insert(0);
		domains[i].insert(1);
		if (foot.c >= 0)
			domains[foot.c].insert(foot.t);

		// 3-2-1. set origIdx for later use (since an arc will be sliced into multiple small arcs)
		arcs[i].originalIndex = i;

		// 3-2-2. check errors for debugging
		if (VORONOI_VERBOSE_ERRORS)
		{
			if (foot.c < 0)												  std::cerr << "ERROR : foot.c < 0 while building domain/transition" << std::endl;
			if (transitions.find(poc{ left[i], 1 }) != transitions.end()) std::cerr << "ERROR : while building transition, overwriting happened" << std::endl;
			if (transitions.find(foot)				!= transitions.end()) std::cerr << "ERROR : while building transition, overwriting happened" << std::endl;
		}

		// 3-3. build transition 
		bool EXPERIMENTAL = true;
		if (!EXPERIMENTAL)
		{
			transitions[poc{ left[i], 1 }] = foot;
			transitions[foot] = poc{ i, 0 };
		}
		else
		{
			// right?
			if (foot.c < 0)
				return;

			// non-g1-neighbor-case
			if (foot.c == left[i])
			{
				//transitions[poc{ left[i], 1 }] = foot;
				transitions[foot] = poc{ i, 0 };
			}
			// t = 0 case
			else if (foot.t == 0.0 && left[foot.c] != -1)
			{
				//
				transitions[poc{ left[i], 1 }] = foot;
				transitions[poc{ left[foot.c], 1 }] = poc{ i, 0 };
			}
			// t = 1 case
			else if (foot.t == 1.0 && right[foot.c] != -1)
			{
				transitions[poc{ left[i], 1 }] = poc{ right[foot.c], 0 };
				transitions[foot] = poc{ i, 0 };
			}
			// t in (0, 1) case
			else
			{
				transitions[poc{ left[i], 1 }] = foot;
				transitions[foot] = poc{ i, 0 };
			}
		}
	}
}

/*
Def : 
	using the domain, divide the arcs's domain(which is just t = [0,1]) into smaller pieces, which will be used to form cycles.
*/
void
	voronoiCalculator::buildPieces()
{
	// 0. Error Check
	if (!_isDomainsBuilt())
	{
		std::cerr << "!!!!!! ERROR : voronoiCalculator : isDomainsBuilt() == false !!!!!!" << std::endl;
		return;
	}
	_isPiecesBuilt() = true;

	// 1. Alias 
	auto& arcs = getArcs();
	//auto& left = getLeft();
	//auto& loop = getLoopIdx();

	auto& domains	  = getDomains();
	//auto& transitions = getTransitions();
	auto& pieces	  = getPieces();
	
	// 2. build piece
	for (int i = 0, length = arcs.size(); i < length; i++) {
		auto& d = domains[i];
		for (auto j = d.begin(), k = ++d.begin(); k != d.end(); ++j, ++k) {
			pieces[poc{ i, *j }] = poc{ i, *k };
		}
	}
}

/*
Def :
	build cycles.
	A cycle is : arc - transition - arc - transition - ...
	
*/
void
	voronoiCalculator::buildCycles()
{
	// 0. Error Check
	if (!_isPiecesBuilt())
	{
		std::cerr << "!!!!!! ERROR : voronoiCalculator : isPiecesBuilt() == false !!!!!!" << std::endl;
		return;
	}
	_isCyclesBuilt() = true;

	// 1. turn piece set into cycle set;
	auto pieces = getPieces(); // WARNING: This declaration not being a reference-type is intended. We are gonna manipulate a copy of "_pieces".
	auto& arcs = getArcs();
	auto& transitions = getTransitions();
	auto& cycles = getCycles();
	
	while (!pieces.empty())
	{
		cycles.push_back(std::vector<CircularArc>());
		auto& cycle = cycles.back();
		//std::vector<double> col;

		// 1-1. first element of cycle.
		auto begin = *pieces.begin();
		auto left  = begin.first;
		auto right = begin.second;
		pieces.erase(left);
		cycle.push_back(planning::subdivCircularArc(arcs[left.c], left.t, right.t)); // note that left.first == right.first

		// +) cyclesOriginal
		_cyclesOriginal.push_back({});
		_cyclesOriginal.back().push_back({ left,right });

		// 1-2. repeat for next element of cycle
		while(true)
		{
			//refactor
			/*
			left = transitions[right];
			auto next = pieces.find(left);

			if (next == pieces.end())
				break;
			right = next->second;
			*/

			auto next = pieces.find(transitions[right]);
			if (next == pieces.end())
				break;
			left  = next->first;
			right = next->second;
			//~refactor

			pieces.erase(left);

			cycle.push_back(planning::subdivCircularArc(arcs[left.c], left.t, right.t));

			// +) cyclesOriginal
			_cyclesOriginal.back().push_back({ left,right });
		}


		//recursivelyFindBisector(cycle, col); // usually called 700 times?
	}
}

/*
Def : 
	from the cycles, evaluate the v_edges inside that cycle.
Date:
2021
	0421 debug : "if(cycles[i].size() > 1)" added
*/
void
	voronoiCalculator::buildVoronoiEdges()
{
	// 0. Error Check
	if (!_isCyclesBuilt())
	{
		std::cerr << "!!!!!! ERROR : voronoiCalculator : isCyclesBuilt() == false !!!!!!" << std::endl;
		return;
	}
	_isVoronoiEdgesBuilt() = true;

	// 1. For each cycle, find bisector
	auto& output = getOutput();
	auto& cycles = getCycles();
	output.resize(cycles.size());

	for (size_t i = 0; i < cycles.size(); i++)
	{
		if(cycles[i].size() > 1)
			output[i] = findBisectorRecursively(cycles[i]);
	}
}


/* 
Def : 
	Returns a list of voronoiEdges for that cycle 
	Do this in a recursive manner, so that if the cycle is too big, cut it and then evaluate it.
*/
retTypeFBR 
	voronoiCalculator::findBisectorRecursively(std::vector<CircularArc>& cycle, int depth)
{
	// need to erase these.
	using namespace planning;
	using namespace std;


	// 1. check cases and return if necessary

	// 1-1. recursive depth enough
	if (depth >= 20) return retTypeFBR();

	// 1-2. triplet case && dpeth enough
	if (cycle.size() == 3 && depth >= 15)
	{
		if (_EpsArc(cycle[0]) || _EpsArc(cycle[1]) || _EpsArc(cycle[2]))
			return {};

		// 1-2-1. find q (bifurcation point)
		auto p0 = getTouchingDiskCenter(cycle[0].x[1], cycle[1].x[0], cycle[0].n[1]);
		auto p1 = getTouchingDiskCenter(cycle[1].x[1], cycle[2].x[0], cycle[1].n[1]);
		auto p2 = getTouchingDiskCenter(cycle[2].x[1], cycle[0].x[0], cycle[2].n[1]);

		auto mid0 = cycle[0](0.5);
		auto mid1 = cycle[1](0.5);
		auto mid2 = cycle[2](0.5);
		
		auto l0 = getLineEquationFromPointVector((mid0 + mid1)*0.5, rotate_p90(mid0 - mid1));
		auto l1 = getLineEquationFromPointVector((mid2 + mid1)*0.5, rotate_p90(mid2 - mid1));
		
		auto q = getLineIntersectionPoint(l0, l1);
		if (isnan(q.P[0]))
		{
			dbg return retTypeFBR();
			q = p0;
		}

		// 1-2-2. save to ret (3 times);
		retTypeFBR edges(3);
		{
			VoronoiEdge& v = edges[0];
			v.v0 = p0;
			v.v1 = q;
			v.idx[0] = cycle[0].originalIndex;
			v.idx[1] = cycle[1].originalIndex;
		}
		{
			VoronoiEdge& v = edges[1];
			v.v0 = p1;
			v.v1 = q;
			v.idx[0] = cycle[1].originalIndex;
			v.idx[1] = cycle[2].originalIndex;
		}
		{
			VoronoiEdge& v = edges[2];
			v.v0 = p2;
			v.v1 = q;
			v.idx[0] = cycle[2].originalIndex;
			v.idx[1] = cycle[0].originalIndex;
		}
		
		return edges;
	}
	
	if (cycle.size() == 2)
	{
		if (_EpsArc(cycle[0]) || _EpsArc(cycle[1]))
			return retTypeFBR();

		// 1-1. sample some points on input curve.
		auto p00 = cycle[0].x[0]; 
		auto p01 = cycle[0].x[1];
		auto p10 = cycle[1].x[0];
		auto p11 = cycle[1].x[1];

		// 1-2. endpoints of bisector
		auto p = getTouchingDiskCenter(cycle[0].x[1], cycle[1].x[0], cycle[0].n[1]);
		auto q = getTouchingDiskCenter(cycle[0].x[0], cycle[1].x[1], cycle[0].n[0]);

		// 1-3. check terminal condition
		bool terminal = false;
		{
		if (sqlength(p00-p11)< rfbTerminationEps) {
			//tag oscrfb
			/*
			Point normal;
			if (cycle[0].ccw)
				p = cycle[0].c.c;
			else
				p = cycle[0].x[0] + cycle[0].c.r * cycle[0].n[0];
			swap(p, q);*/
			//draw(p);
			terminal = true;
		}
		else if (sqlength(p01 - p10)<rfbTerminationEps) {
			//tag oscrfb
			/*Point normal;
			if (cycle[0].ccw)
				q = cycle[0].c.c;
			else
				q = cycle[0].x[1] + cycle[0].c.r * cycle[0].n[1];*/
			//draw(q);
			terminal = true;
		}
		else if (sqlength(p00 - p01)<rfbTerminationEps)
			terminal = true;
		else if (sqlength(p10 - p11)<rfbTerminationEps)
			terminal = true;
		}

		// 1-4. if (terminal && vEdge length is short enough) return it
		if (sqlength(p-q)<rfbTerminationEps2 || terminal) {
			retTypeFBR edges(1);
			{
				auto& v = edges[0];
				v.v0 = p;
				v.v1 = q;
				v.idx[0] = cycle[0].originalIndex;
				v.idx[1] = cycle[1].originalIndex;
			}
			return edges;
		}

		// 1-5. if the endpoints of a curve are too close, return. No more subdiv
		if (_EpsArc(cycle[0]) || _EpsArc(cycle[1]))
			return retTypeFBR();
	}

	// 2. if not returned above, subdiv
	if (true)
	{
		auto foot = findMaximalDiskSharingPoint(cycle, poc{ 0 , 0.5 }, 0, 0);

		auto first  = subdivCircularArc(cycle[0]	 , 0.5);	// subdiv curve[0] at t = 0.5
		auto second = subdivCircularArc(cycle[foot.c], foot.t); // subdiv some other curve, where it shares a disk with curve[0] at t = 0.5

		vector<CircularArc> sub[2];
		if (cycle.size() == 2)
		{
			// build [0] :
			{
				sub[0].push_back(first.second);			//curve[0]'s right
				sub[0].push_back(second.first);			//curve[other]'s left
			}
			// build [1] :
			{
				sub[1].push_back(first.first);	//curve[0]'s left
				sub[1].push_back(second.second);//curve[other]'s right
			}
		}
		else
		{
			// build [0] : from right segment of curve[0] ~ left seg of other curve
			{
				for (int i = 1; i < foot.c; i++) {
					sub[0].push_back(cycle[i]);
				}
				sub[0].push_back(second.first);			//curve[other]'s left
				sub[0].push_back(first.second);			//curve[0]'s right
			}
			// build [1] : leftovers from [0]
			{
				for (size_t i = foot.c + 1; i < cycle.size(); i++) {
					sub[1].push_back(cycle[i]);
				}
				sub[1].push_back(first.first);	//curve[0]'s left
				sub[1].push_back(second.second);//curve[other]'s right
			}
		}
		auto ret0 = findBisectorRecursively(sub[0], depth + 1);
		auto ret1 = findBisectorRecursively(sub[1], depth + 1);

		ret0.insert(ret0.end(), ret1.begin(), ret1.end());
		return ret0;

	}
}

/*
Def: evaluate a cycle to get its bifur-pt

Note:
	Just a simpler version of findBisectorRecursively. (Erased some parts not related to bifurcation)
*/
std::vector<Point>
	voronoiCalculator::findBifurcationPointRecursively(std::vector<CircularArc>& cycle, int depth)
{
	// need to erase these.
	using namespace planning;
	using namespace std;

	// 1. check cases and return if necessary

	// 1-1. recursive depth enough
	if (depth >= 20) return {};

	// 1-2. triplet case && dpeth enough
	if (cycle.size() >= 3 && depth >= 15)
	{
		if (_EpsArc(cycle[0]) || _EpsArc(cycle[1]) || _EpsArc(cycle[2]))
			return {};

		// 1-2-1. find q (bifurcation point)
		auto p0 = getTouchingDiskCenter(cycle[0].x[1], cycle[1].x[0], cycle[0].n[1]);
		auto p1 = getTouchingDiskCenter(cycle[1].x[1], cycle[2].x[0], cycle[1].n[1]);
		Point p2;
		if(cycle.size() == 3)
			p2 = getTouchingDiskCenter(cycle[2].x[1], cycle[0].x[0], cycle[2].n[1]);
		else
			p2 = getTouchingDiskCenter(cycle[2].x[1], cycle[3].x[0], cycle[2].n[1]);

		auto mid0 = cycle[0](0.5);
		auto mid1 = cycle[1](0.5);
		auto mid2 = cycle[2](0.5);
		
		auto l0 = getLineEquationFromPointVector((mid0 + mid1)*0.5, rotate_p90(mid0 - mid1));
		auto l1 = getLineEquationFromPointVector((mid2 + mid1)*0.5, rotate_p90(mid2 - mid1));
		
		auto q = getLineIntersectionPoint(l0, l1);
		if (isnan(q.P[0]))
		{
			dbg return {};
			q = p0;
		}
		
		return { q };
	}
	
	if (cycle.size() <= 2)
	{
		return {};
	}

	// 2. if not returned above, subdiv
	if (true)
	{
		auto foot = findMaximalDiskSharingPoint(cycle, poc{ 0 , 0.5 }, 0, 0);

		auto first  = subdivCircularArc(cycle[0]	 , 0.5);	// subdiv curve[0] at t = 0.5
		auto second = subdivCircularArc(cycle[foot.c], foot.t); // subdiv some other curve, where it shares a disk with curve[0] at t = 0.5

		vector<CircularArc> sub[2];
		// build [0] : from right segment of curve[0] ~ left seg of other curve
		{
			for (int i = 1; i < foot.c; i++) {
				sub[0].push_back(cycle[i]);
			}
			sub[0].push_back(second.first);			//curve[other]'s left
			sub[0].push_back(first.second);			//curve[0]'s right
		}
		// build [1] : leftovers from [0]
		{
			for (size_t i = foot.c + 1; i < cycle.size(); i++) {
				sub[1].push_back(cycle[i]);
			}
			sub[1].push_back(first.first);	//curve[0]'s left
			sub[1].push_back(second.second);//curve[other]'s right
		}

		auto ret0 = findBifurcationPointRecursively(sub[0], depth + 1);
		auto ret1 = findBifurcationPointRecursively(sub[1], depth + 1);

		ret0.insert(ret0.end(), ret1.begin(), ret1.end());
		return ret0;

	}
}

/*
Def:
	Sorts the given vector to make it g0
	n^2
Assume :
	The input is a soup(non-ordered set), but it could be re-arragned to become g0 somehow
Parameter :
	ve : should be a copy, not ref
*/
std::deque<VoronoiEdge>
	voronoiCalculator::sortVectorVoronoiEdge(std::vector<VoronoiEdge> ve, double sqError)
{
	// 1. init step
	std::deque<VoronoiEdge> temp;
	temp.push_back(ve.back());
	ve.pop_back();

	// 2. search for neighbors and append to temp.
	while (ve.size() != 0)
	{
		Point& p0 = temp.front().v0;
		Point& p1 = temp.back().v1;

		bool found = false;
		int deleteIdx = -1;
		for (int i = ve.size() - 1; i > -1; i--)
		{
			Point& q0 = ve[i].v0;
			Point& q1 = ve[i].v1;

			if ((p0 - q1).length2() < sqError)
			{
				found = true; deleteIdx = i;
				temp.push_front(ve[i]);
				break;
			}
			if ((p1 - q0).length2() < sqError)
			{
				found = true; deleteIdx = i;
				temp.push_back(ve[i]);
				break;
			}
			if ((p1 - q1).length2() < sqError)
			{
				found = true; deleteIdx = i;
				temp.push_back(ve[i].flip());
				break;
			}
			if ((p0 - q0).length2() < sqError)
			{
				found = true; deleteIdx = i;
				temp.push_front(ve[i].flip());
				break;
			}
		}

		if (found)
		{
			ve.erase(ve.begin() + deleteIdx);
		}
		else
		{
			std::cerr << "!!! ERROR : Failed sorting segments in calculateG() !!!" << std::endl;
			break;
		}
	}

	// 3. ret
	return temp;
}

/*
Def : 
	given two voronoi Edges v0, v1 : we want to find some vCurve(vector<VorEdge>) which is the closest pair to a vCurve(vector<VorEdge>) which starts with v0 and ends with v1;
Param :
	v0 : the first vEdge of a vCurve we are interested in.
	v1 : the final vEdge of a vCurve we are interested in.
Return :
	vector of VoronoiEdge, if such an pair exists.
	but, if the size of the array is 0(when two bifurcation points merge in the next slice, forming an X-junction) :
		push a eps-vEdge : {v0 = bifur, v1 = bifur}

*/
std::vector<VoronoiEdge> voronoiCalculatorResultG::findVoronoiCurvePair(VoronoiEdge& v0, VoronoiEdge& v1, double equalPointError)
{
	// 1. project v0 v1 (= find closest bifur)
	auto
		& ep0 = v0.v0,
		& ep1 = v1.v1;
	int eppi0, eppi1; //endpoint projection (index) to next slice
	double d0 = 1e8, d1 = 1e8;
	//d0 = d1 = equalPointError;
	bool found0 = false, found1 = false; // whether there is a corresponding point in this class' data

	// 1-1. find query point's closest neighbor in DB;
	for (size_t i = 0; i < allPoints.size(); i++)
	{
		auto& v = allPoints[i];
		auto D0 = (v - ep0).length2();
		auto D1 = (v - ep1).length2();

		if (D0 < d0)
		{
			eppi0 = i;
			d0 = D0;
			//found0 = true;
		}

		if (D1 < d1)
		{
			eppi1 = i;
			d1 = D1;
			//found1 = true;
		}
	}
	if (d0 < equalPointError)
		found0 = true;
	if (d1 < equalPointError)
		found1 = true;

	// 2. take care of found0 or found 1 being false;
	// e.g., if ep0 was a bifurcation point which disappears at next slice
	// 2-1. find a single vEdge instance which is closest to ep0 or ep1. than use the vCurve(containing that vEdge)'s endpoint in graph search
	std::vector<VoronoiEdge> mergeFirst, mergeLast;
	if (!found0)
	{
		// 2-1-1. alias
		int closestP = eppi0; // closest but not close enough;
		auto& pt = ep0;
		auto& tan = (v0.v1 - v0.v0); //size does not matter. should change between found0 and found 1
		auto& arr = mergeFirst;
		auto& newEppi = eppi0;

		// 2-1-2. find closest ve
		auto& cand = V2E()[closestP];
		int closestI = 0, closestJ = 0;
		double dist = 1e10;
		// for (all vCurves near)
		for (size_t i = 0; i < cand.size(); i++)
		{
			auto edgeIdx = cand[i];
			if (edgeIdx == -1)
				continue;

			auto& vc = E()[edgeIdx];

			// for (all vEdge in vCurve)
			for (size_t j = 1; j < vc.size(); j++) // j = 1 since, j=0's v0 is vert in graph
			{
				auto& v = vc[j];
				auto dTemp = (v.v0 - ep0).length2();
				if (dTemp < dist)
				{
					dist = dTemp;
					closestI = i;
					closestJ = j;
				}

			}
		}
		// closestI, closestJ is found;

		// 2-1-3. find need for flip;
		auto vCurveIdx = cand[closestI];
		auto& vClose = E()[vCurveIdx][closestJ];
		auto tan2 = (vClose.v1 - vClose.v0);
		auto dotP = tan * tan2;
		bool needFlip = dotP < 0; //if(false) insert [j-1, 0] else [j, end]

		// 2-1-4. build mergeFirst;
		if (needFlip)
		{
			auto& temp = E()[vCurveIdx];
			for (int i = closestJ - 1; i >= 0; i--)
			{
				////dbg_out
				//std::cout << " temp.size()   ,  i = " << temp.size() << "    ,   " << i << std::endl;
				arr.push_back(temp[i].flip());
			}
			newEppi = E2V()[vCurveIdx].first;
		}
		else
		{
			auto& temp = E()[vCurveIdx];
			for (size_t i = closestJ; i < temp.size(); i++)
			{
				arr.push_back(temp[i]);
			}
			newEppi = E2V()[vCurveIdx].second;
		}

	}
	if (!found1)
	{
		// 2-1-1. alias
		int closestP = eppi1; // closest but not close enough;
		auto& pt = ep1;
		auto& tan = (v1.v0 - v1.v1); //size does not matter. should change between found0 and found 1
		auto& arr = mergeFirst;
		auto& newEppi = eppi1;

		// 2-1-2. find closest ve
		auto& cand = V2E()[closestP];
		int closestI = 0, closestJ = 0;
		double dist = 1e10;
		// for (all vCurves near)
		for (size_t i = 0; i < cand.size(); i++)
		{
			auto edgeIdx = cand[i];
			if (edgeIdx == -1)
				continue;

			auto& vc = E()[edgeIdx];

			// for (all vEdge in vCurve)
			for (size_t j = 1; j < vc.size(); j++) // j = 1 since, j=0's v0 is vert in graph
			{
				auto& v = vc[j];
				auto dTemp = (v.v0 - ep0).length2();
				if (dTemp < dist)
				{
					dist = dTemp;
					closestI = i;
					closestJ = j;
				}

			}
		}
		// closestI, closestJ is found;

		// 2-1-3. find need for flip;
		auto vCurveIdx = cand[closestI];
		auto& vClose = E()[vCurveIdx][closestJ];
		auto tan2 = (vClose.v1 - vClose.v0);
		auto dotP = tan * tan2;
		bool needFlip = dotP < 0; //if(false) insert [j-1, 0] else [j, end]

		// 2-1-4. build mergeLast;
		if (needFlip)
		{
			auto& temp = E()[vCurveIdx];
			for (int i = temp.size() - 1; i >= closestJ; i--) // HARD ???
			{
				arr.push_back(temp[i].flip());
			}

			newEppi = E2V()[vCurveIdx].second;
		}
		else
		{
			auto& temp = E()[vCurveIdx];
			for (size_t i = 0; i < closestJ; i++)
			{
				arr.push_back(temp[i]);
			}
			newEppi = E2V()[vCurveIdx].first;
		}

	}


	// 3. now we just need to find a path in the graph from eppi0 to eppi1;
	// 3-0. triv case
	// 3-0-1. the first/last points are the same.
	if (eppi0 == eppi1)
	{
		// simply merge.
		mergeFirst.insert(mergeFirst.end(), mergeLast.begin(), mergeLast.end());
		// but don't make it size() == 0;
		if (mergeFirst.size() == 0)
		{
			VoronoiEdge vTemp;
			vTemp.v0 = V()[eppi0];
			vTemp.v1 = V()[eppi0];
			vTemp.idx[0] = -1; // to indicate that this is a fake vEdge;
			vTemp.idx[1] = -1; // to indicate that this is a fake vEdge;
			mergeFirst.push_back(vTemp);
		}
		return mergeFirst;
	}
	// 3-0-2. one line between.
	if (V2E()[eppi0][eppi1] != -1)
	{
		// check direction of curve
		auto vCurveIdx = V2E()[eppi0][eppi1];
		auto leftOfVIdx = E2V()[vCurveIdx].first;
		auto needFlip = eppi0 != leftOfVIdx;
		
		// merge
		if (needFlip)
		{
			for (int i = E()[vCurveIdx].size() - 1; i >= 0; i--)
			{
				auto& ve = E()[vCurveIdx][i];
				mergeFirst.push_back(ve.flip());
			}
		}
		else
		{
			for (size_t i = 0, length = E()[vCurveIdx].size(); i < length; i++)
			{
				auto& ve = E()[vCurveIdx][i];
				mergeFirst.push_back(ve);
			}
		}

		// merge Last
		mergeFirst.insert(mergeFirst.end(), mergeLast.begin(), mergeLast.end());

		return mergeFirst;
	}

	// 3-1. Multiple edges between... => Do BFS; // HARD TODO: better way to do search in graph
	std::vector<bool> visited(V().size(), false);
	std::deque<std::vector<int>> queue;
	// 3-1. queue init
	visited[eppi0] = true;
	std::vector<int> temp;
	temp.push_back(eppi0);
	queue.push_back(temp);
	bool pathExists = false; // HARD TODO : when cs-obstacles merge and destroys vCurve
	
	// 3-2. queue to find path in voronoiGraph
	std::vector<int> qRes;
	while (!queue.empty())
	{
		temp = queue.front();
		queue.pop_front();

		bool canBreak = false;

		auto lastVert = temp.back();
		////dbg_out
		//std::cout << "lastVert : " << lastVert << "      V.size() : " << V().size() << std::endl;
		
		auto& connections = V2E()[lastVert];
		
		for (int neigh = 0; neigh < connections.size(); neigh++)
		//for (auto& neigh : neighCandi)
		{
			if (connections[neigh] == -1) // "neigh" is not connected with lastVert
				continue;

			if (visited[neigh])
				continue;

			if (neigh == eppi1)
			{
				canBreak = true;
				pathExists = true;
				qRes = temp;
				qRes.push_back(neigh);
				break;
			}
			else
			{
				visited[neigh] = true;
				auto copy = temp;
				copy.push_back(neigh);
				queue.push_back(copy);
			}
		}
		
		if (canBreak)
		{
			break;
		}

	}

	// 3-3. after finding a path, merge the lists of vedges, regarding the orders
	if (!pathExists)
		return {};
	else
	{
		// now merge these lists;
		for (size_t i = 0; i < qRes.size() - 1; i++)
		{
			auto in = i + 1;
			auto vCurveIdx  = V2E()[qRes[i]][qRes[in]];
			auto leftOfVIdx = E2V()[vCurveIdx].first; // the left point of the current vCurve examined
			bool needFlip; 
			if(i == 0)
				needFlip = (eppi0 != leftOfVIdx);
			else
			{
				auto& lastP = mergeFirst.back().v1;
				auto& p = E()[vCurveIdx].front().v0;
				auto& q = E()[vCurveIdx].back().v1;

				if ((lastP - q).length2() < (lastP - p).length2())
					needFlip = true;
				else
					needFlip = false;
			}

			// merge
			if (needFlip)
			{
				for (int i = E()[vCurveIdx].size() - 1; i >= 0; i--)
				{
					////dbg_out
					//std::cout << "needFlip : i = " << i << std::endl;
					
					auto& ve = E()[vCurveIdx][i];
					mergeFirst.push_back(ve.flip());
				}
			}
			else
			{
				for (size_t i = 0, length = E()[vCurveIdx].size(); i < length; i++)
				{
					auto& ve = E()[vCurveIdx][i];
					mergeFirst.push_back(ve);
				}
			}
		}


		// merge Last
		//mergeFirst.insert(mergeFirst.begin(), mergeLast.begin(), mergeLast.end()); // bug_fixed
		mergeFirst.insert(mergeFirst.end(), mergeLast.begin(), mergeLast.end());

		return mergeFirst;
	}

}

/*
Def : initialize the thing
	NOT done
*/
void voronoiCalculatorResultG::initialize()
{

}
