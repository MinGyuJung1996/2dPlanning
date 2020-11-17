#include "voronoi.hpp"
#include <filesystem>

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
	double rfbTerminationEps = 1e-18; //originally 1e-18
	double rfbTerminationEps2 = 2.5e-3; // origirnally 2.5e-3;


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

	/* Def : assume that a circular arc is parameterized, find that parameter of nt (btw [0,1])
		assume: the function is called with t values which are [0,1], since we are interested in arc only.
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

	/* Def : return one circular arc corresponding to interval [t0, t1] of c

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
		auto n0 = arcNormalInterpolation(c.n[0], c.n[1], t0);
		auto n1 = arcNormalInterpolation(c.n[0], c.n[1], t1);


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

	/* Def : return two circular arcs [0, t] and [t, 1]
	*/
	pair<CircularArc, CircularArc> subdivCircularArc(CircularArc& c, double t)
	{
		pair<CircularArc, CircularArc> ret;
		ret.first  = c;
		ret.second = c;

		auto n = arcNormalInterpolation(c.n[0], c.n[1], t);

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

	/*	Def: Reduce the difference btw "output of MS" & "input of vrIN"
		Par:
			msOut		: model_Result glo var.
			isBoundary	: it's actually, a bool wheter it is the "outer" boundary (not inner loop)
			vrIn		: just empty VR_IN should be okay.
	*/
	void _Convert_MsOut_To_VrIn(vector<deque<ArcSpline>>& INPUT msOut, vector<bool>& INPUT isBoundary, VR_IN& OUTPUT vrIn)
	{
		// 1. count number of outer boundaries
		int nBoundary = 0;
		{
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
			
			if (!isBoundary[i]) // ignore inner loops, since we are doing motion planning
				continue;
			else
			{

				// 2-1. build vrIn.Arcs & arcsPerLoop
				vector<CircularArc> temp;
				convertMsOutput_Clockwise(msOut[i], temp);
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
			double r = 2;
			CircularArc a(Point(0, 0), r, Point(+1, +0), Point(+0, +1));
			CircularArc b(Point(0, 0), r, Point(+0, +1), Point(-1, +0));
			CircularArc c(Point(0, 0), r, Point(-1, +0), Point(+0, -1));
			CircularArc d(Point(0, 0), r, Point(+0, -1), Point(+1, +0));
			auto s = vrIn.arcs.size();
			vrIn.arcs.push_back(a);
			vrIn.arcs.push_back(b);
			vrIn.arcs.push_back(c);
			vrIn.arcs.push_back(d);
			vrIn.left.push_back(s + 3);	// left(a) = d
			vrIn.left.push_back(s + 0);	// left(b) = a
			vrIn.left.push_back(s + 1);	// left(c) = b
			vrIn.left.push_back(s + 2); // left(d) = c
			vrIn.color.push_back(1);
			vrIn.color.push_back(1);
			vrIn.color.push_back(1);
			vrIn.color.push_back(1);
			vrIn.arcsPerLoop.push_back(4);
		}


		// 4. fix bugs in output
		for (auto& i : vrIn.arcs)
		{
			// 4-1. some with strange ccw
			auto ccw = (i.n[0] ^ i.n[1]) > 0;
			//if (PRINT_ERRORS && ccw != i.ccw) cerr << "ERROR : ccw of arc not correct\n      : n0 n1 arc.ccw" << i.n[0] << i.n[1] << " " << i.ccw << endl;
			i.ccw = ccw;

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
	*/
	pointOnCurve findMaximalDiskSharingPoint(vector<CircularArc>& INPUT arcs, pointOnCurve INPUT poc, int INPUT left, int INPUT right)
	{
		//tag g1inc
		if (left != poc.c)
		{
			if (fabs(arcs[left].n[1] * arcs[poc.c].n[0]) < 1 - 1e-8) //check if not g1
				return { left, 1 };
		}

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
		const double r_init = 1e+10;
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
			if (arcs[j].ccw)
				normalInArc = ((arcs[j].n[0] ^ normal) >= -eps) && ((normal ^ arcs[j].n[1]) >= -eps);
			else
				normalInArc = ((arcs[j].n[0] ^ normal) <= eps) && ((normal ^ arcs[j].n[1]) <= eps);

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
			auto t = arcNormalToParameter(arcs[bestJ].n[0], arcs[bestJ].n[1], normal);
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
				planning::output_to_file::bifur_point p;
				p.p = q;
				for (int i = 0; i < 3; i++)
				{
					p.idx.push_back(cycle[i].originalIndex);
				}
				planning::output_to_file::bifur_points[t2].push_back(p);
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