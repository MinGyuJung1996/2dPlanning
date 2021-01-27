#include "voronoi.hpp"
#include "collision detection.hpp"

//double dbgBlock[100];

namespace ms{

	int RES = 100;

extern int coefBasis[20][20];
extern double basis2[10001][3];
extern double basis3[10001][4];
extern double basis4[10001][5];
extern double basis5[10001][6];
extern double basis6[10001][7];

const bool change7 = true; // change 7 model to scene-with-obstacles (6 + 7)
const bool change0 = true; // change 0 to half size
const bool override1 = true;
const bool override7 = true;
std::vector<CircularArc> Model_vca[8]; // model's circularArc representation (before calling postprocessing)
bool Model_from_arc[8];					// true: read from circular arc file // false : read from bezier				

#define useApproxInSymmetryCollision true
#define useNewerVersionOfG1conveter false

#define DEBUG

/*!
*	\brief 필요한 데이터(B-Spline Model Data, Interior Disk Data)를 Import하고, 8개의 기본 모델에 대하여 Arc Spline으로 근사한다
*/
void initialize()
{
	fopen_s(&f, "time.txt", "w");
	ModelInfo_CurrentModel.first  = 1; // 1
	ModelInfo_CurrentModel.second = 6; // 6
	Models_Imported[0] = import_Crv("impt1.txt");
	Models_Imported[1] = import_Crv("impt2.txt");
	Models_Imported[2] = import_Crv("impt3.txt");
	Models_Imported[3] = import_Crv("impt4.txt");
	Models_Imported[4] = import_Crv("impt5.txt");
	Models_Imported[5] = import_Crv("impt6.txt");
	Models_Imported[6] = import_Crv("impt7.txt");
	Models_Imported[7] = import_Crv("impt8.txt");
	InteriorDisks_Imported[0] = importCircles("refinedCircle1.txt");
	InteriorDisks_Imported[1] = importCircles("refinedCircle2.txt");
	InteriorDisks_Imported[2] = importCircles("refinedCircle3.txt");
	InteriorDisks_Imported[3] = importCircles("refinedCircle4.txt");
	InteriorDisks_Imported[4] = importCircles("refinedCircle5.txt");
	InteriorDisks_Imported[5] = importCircles("refinedCircle6.txt");
	InteriorDisks_Imported[6] = importCircles("refinedCircle7.txt");
	InteriorDisks_Imported[7] = importCircles("refinedCircle8.txt");

	//debug : test multiple loops in one model // conclusion : seems to work

	// lambda def: change a given model in models_imported[modelNo]
	//	used for models that have bezier representation
	auto changeModel = [&](int modelNo, double scale, double rotationRadian, Point translation)
	{
		// to generalize, just change alias
		auto& model = Models_Imported[modelNo];
		auto& disks = InteriorDisks_Imported[modelNo];
		{
			Point a[4];
			// 1. do Bezier
			for (auto& i : model)
			{
				// 1-1. scale
				a[0] = scale * i.P[0];
				a[1] = scale * i.P[1];
				a[2] = scale * i.P[2];
				a[3] = scale * i.P[3];

				// 1-2. rotate
				a[0] = a[0].rotate(rotationRadian);
				a[1] = a[1].rotate(rotationRadian);
				a[2] = a[2].rotate(rotationRadian);
				a[3] = a[3].rotate(rotationRadian);

				// 1-3. translate
				a[0] = a[0] + translation;
				a[1] = a[1] + translation;
				a[2] = a[2] + translation;
				a[3] = a[3] + translation;

				// 1-4. save
				i = BezierCrv(a);
			}

			// 2. do disk
			for (auto& i : disks)
			{
				i.c = (scale * i.c).rotate(rotationRadian) + translation;
				i.r = scale * i.r;
			}

		}
	};

	// lambda def: apply transfrom to models_improted[modelFrom] and append it to models_imported[modelTo]
	//	used for models that have bezier representation
	auto appendModel = [&](int modelFrom, int modelTo, double scale, double rotationRadian, Point translation)
	{
		auto& model		= Models_Imported[modelFrom];
		auto& disks		= InteriorDisks_Imported[modelFrom];
		auto& model2	= Models_Imported[modelTo];
		auto& disks2	= InteriorDisks_Imported[modelTo];
		{
			Point a[4];
			// 1. do Bezier
			for (auto& i : model)
			{
				// 1-1. scale
				a[0] = scale * i.P[0];
				a[1] = scale * i.P[1];
				a[2] = scale * i.P[2];
				a[3] = scale * i.P[3];

				// 1-2. rotate
				a[0] = a[0].rotate(rotationRadian);
				a[1] = a[1].rotate(rotationRadian);
				a[2] = a[2].rotate(rotationRadian);
				a[3] = a[3].rotate(rotationRadian);

				// 1-3. translate
				a[0] = a[0] + translation;
				a[1] = a[1] + translation;
				a[2] = a[2] + translation;
				a[3] = a[3] + translation;

				// 1-4. save
				model2.push_back(BezierCrv(a));
			}

			// 2. do disk
			for (auto& i : disks)
			{
				Circle push;
				push.c = (scale * i.c).rotate(rotationRadian) + translation;
				push.r = scale * i.r;

				disks2.push_back(push);
			}
		}
	};


	if (change7)
	{
		changeModel(   7, 0.3, 0.5, Point(-0.3, 0.0));
		appendModel(6, 7, 0.3, 0.4, Point(0.04, 0.1));
		appendModel(5, 7, 0.3, 0.0, Point(0.07, -0.3));
		appendModel(4, 7, 0.3, 0.0, Point(-0.3, -0.4));

		//Point a[4];
		//auto trans = Point(0.3, 0);
		//auto scale = 0.3f;
		//for (auto& i : Models_Imported[7])
		//{
		//	a[0] = scale * i.P[0] - trans;
		//	a[1] = scale * i.P[1] - trans;
		//	a[2] = scale * i.P[2] - trans;
		//	a[3] = scale * i.P[3] - trans;
		//	i = BezierCrv(a);
		//}
		//for (auto& i : InteriorDisks_Imported[7])
		//{
		//	i = (Circle(scale * i.c - trans, scale * i.r));
		//}
		//
		//for (size_t i = 0; i < Models_Imported[6].size(); i++)
		//{
		//	a[0] = scale * Models_Imported[6][i].P[0] + trans;
		//	a[1] = scale * Models_Imported[6][i].P[1] + trans;
		//	a[2] = scale * Models_Imported[6][i].P[2] + trans;
		//	a[3] = scale * Models_Imported[6][i].P[3] + trans;
		//	Models_Imported[7].push_back(BezierCrv(a));
		//}
		//for (auto& i : InteriorDisks_Imported[6])
		//{
		//	InteriorDisks_Imported[7].push_back(Circle(scale * i.c + trans, scale * i.r));
		//}
	}

	if (change0)
	{
		int ind = 0;
		Point a[4];
		auto trans = Point(0, 0);
		auto scale = 0.4f;
		for (auto& i : Models_Imported[ind])
		{
			a[0] = scale * i.P[0] - trans;
			a[1] = scale * i.P[1] - trans;
			a[2] = scale * i.P[2] - trans;
			a[3] = scale * i.P[3] - trans;
			i = BezierCrv(a);
		}
		for (auto& i : InteriorDisks_Imported[ind])
		{
			i = (Circle(scale * i.c - trans, scale * i.r));
		}
	}

	//~debug


	planning::output_to_file::objSize.resize(8);

	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < (int)Models_Imported[i].size(); j++) {
			Models_Imported[i][j].segmentation(Models[i]);
		}

		for (int j = 0; j < (int)Models[i].size(); j++) {
			auto s = Models[i][j].integrityTest();
			if (s.size() > 0)
				Models[i].insert(Models[i].begin() + j + 1, s.begin(), s.end());
		}

		for (int j = 0; j < (int)Models[i].size(); j++) {
			auto tempSpiral = ArcSpline(Models[i][j]); /* where bez -> arc */
			auto input = tempSpiral.integrityTest();
			Models_Approx[i].insert(Models_Approx[i].end(), input.begin(), input.end());
		}

		//build planning::output_to_file::objSize;
		if (i != 7)
		{
			int cnt = 0;
			for (auto& as : Models_Approx[i])
			{
				cnt += as.Arcs.size();
			}
			planning::output_to_file::objSize[i].push_back(cnt);
		}
		else
		{
			int cnt = 0;
			int a0 = Models_Imported[7].size() - Models_Imported[6].size();
			for (size_t j = 0; j < a0; j++)
			{
				cnt += Models_Approx[i][j].Arcs.size();
			}
			planning::output_to_file::objSize[i].push_back(cnt);

			cnt = 0;
			for (size_t j = a0; j < Models_Imported[7].size(); j++)
			{
				cnt += Models_Approx[i][j].Arcs.size();
			}
			planning::output_to_file::objSize[i].push_back(cnt);
		}
		//end objSize
	}

	// tried to merge models_approx after... but seems to break
	if (false)
	{
		Point a[4];
		auto trans = Point(0.3, 0);
		auto scale = 0.3f;
		for (auto& i : Models_Approx[7])
		{
			for (auto& j : i.Arcs)
			{
				j.c.c  = scale * j.c.c - trans;
				j.c.r  = scale * j.c.r;
				j.x[0] = scale * j.x[0] - trans;
				j.x[1] = scale * j.x[1] - trans;

			}
		}
		for (auto& i : InteriorDisks_Imported[7])
		{
			i = (Circle(scale * i.c - trans, scale * i.r));
		}
		/*for (size_t i = 0; i < Models_Approx[6].size(); i++)
		{
			ArcSpline as = Models_Approx[6][i];
			for (auto& j : as.Arcs)
			{
				j.c.c  = scale * j.c.c  + trans;
				j.x[0] = scale * j.x[0] + trans;
				j.x[1] = scale * j.x[1] + trans;
			}

			Models_Approx[7].push_back(as);
		}
		for (auto& i : InteriorDisks_Imported[6])
		{
			InteriorDisks_Imported[7].push_back(Circle(scale * i.c + trans, scale * i.r));
		}*/
	}

	if (override1)
	{
		using namespace std;

		// init step

		int overridedIndex = 1;
		double scaleFactor = 0.21;// 0.2;
		ifstream ar ("arcModel.txt");
		ifstream arr("arcModelRSV.txt");
		ifstream crr("circModelRSV.txt");

		// read size of input

		int
			size0,
			size1,
			size2;

		ar  >> size0;
		arr >> size1;
		crr >> size2;


		// do circles;

		vector<Circle> RSV_Circles;

		// 1. read
		for (int i = 0; i < size2; i++)
		{
			double cx, cy, cr;
			crr >> cx >> cy >> cr;

			RSV_Circles.push_back(Circle(Point(cx, cy), cr));
		}

		// 2. scale
		for (auto& circ : RSV_Circles)
		{
			circ.c = circ.c * scaleFactor;
			circ.r = circ.r * scaleFactor;
		}

		// 3. override data
		InteriorDisks_Imported[overridedIndex] = RSV_Circles;

		// do RSV

		// 1. Read from file
		vector<CircularArc> fileRead;
		for (int i = 0; i < size1; i++)
		{
			double cx, cy, cr, t0, t1, ccw;
			arr >> cx >> cy >> cr >> t0 >> t1 >> ccw;

			if (ccw)
				while (t1 < t0)
					t1 += PI * 2;
			else
				while (t1 > t0)
					t1 -= PI * 2;
			fileRead.push_back(cd::constructArc(Point(cx, cy), cr, t0, t1));
		}

		// 1.5 scaling
		for (auto& arc : fileRead)
		{
			arc.c.c = arc.c.c * scaleFactor;
			arc.c.r = arc.c.r * scaleFactor;
			arc.x0() = arc.x0() * scaleFactor;
			arc.x1() = arc.x1() * scaleFactor;
		}

		// 2. make it G0
		vector<CircularArc> g0;
		planning::_Convert_VectorCircularArc_G0(fileRead, g0, 1);

		//dbg_out
		cout << "override1_data endpoints " << endl;
		for (auto& arc : g0)
			cout << arc.x0() << ",,, " << arc.x1() << endl;
		//dbg

		// 3. make it G1
		vector<CircularArc> g1;
		if(useNewerVersionOfG1conveter)
			planning::_Convert_VectorCircularArc_G1_new(g0, g1);
		else
			planning::_Convert_VectorCircularArc_G1(g0, g1);
		Model_vca[overridedIndex] = g1;
		Model_from_arc[overridedIndex] = true;


		// 4. convert to AS
		vector<ArcSpline> asRSV;
		planning::_Convert_VectorCircularArc_To_MsInput(g1, asRSV);

		// 5. override
		Models_Approx[overridedIndex] = asRSV;

	}

	// if(make object scene from files read)
	if (override7)
	{
		/*
		What is done:
		1. set flag : Model_from_arc
		2. set model data : Model_vca (will be used in post processing... optional since this is obstacles, not robot...)
		3. convert model and save : Models_approx
		4. also set disks : 

		Notice that there can be up to 2~3 copies of the same model with different representation
			=> Model_vca(vector<CircularArc>, original, read from file)
			=> Models_approx(vector<ArcSpline>, modified(segmented) to be an input of minkowski sum)
			=> Models_approx_rotated(rotate Model_vca and then segment)
		*/

		// 1. alias

		int ovIdx = 7;

		Model_from_arc[ovIdx] = true;
		auto& sceneOriginal		= Model_vca[ovIdx];
		auto& sceneProcessed	= Models_Approx[ovIdx];
		auto& sceneCircles		= InteriorDisks_Imported[ovIdx];

		sceneOriginal.resize(0);
		sceneProcessed.resize(0);
		sceneCircles.resize(0);

		using namespace std;

		// 2. read files

		// 2-1. LAMBDA to read files and process it
		/*
		LAMBDA
		Def: do
		1. read from given input file names
		2. make model g0
		3. make model g1

		After calling this func, we can make instances of these models with append model
		*/
		auto readArcModelAndProcessIt = [](const char* arcIn, const char* circIn, vector<CircularArc>& arcOut, vector<Circle>& circOut) 
		{
			// 1. fstream

			ifstream ais(arcIn);
			ifstream cis(circIn);

			size_t asz, csz;
			ais >> asz;
			cis >> csz;

			
			// 2. read circles;
			for (int i = 0; i < csz; i++)
			{
				double cx, cy, cr;
				cis >> cx >> cy >> cr;

				circOut.push_back(Circle(Point(cx, cy), cr));
			}

			// 3. read arcs
			vector<CircularArc> temp;
			set<int> case_c;
			char type;
			double cx, cy, cr, t0, t1, ccw;
			for (int i = 0; i < asz; i++)
			{
				ais >> type;
				switch (type)
				{
				case 'a':
					// the text line contains info : circle, theta0, theta1, ccw_info 
					{
						ais >> cx >> cy >> cr >> t0 >> t1 >> ccw;

						if (ccw)
							while (t1 < t0)
								t1 += PI * 2;
						else
							while (t1 > t0)
								t1 -= PI * 2;
						temp.push_back(cd::constructArc(Point(cx, cy), cr, t0, t1));
					}
					break;
				case 'c':
					// given : radius
					// connect prev.x1() and next.x0(), with a striaght line (approximated radius cr)
					ais >> cr >> ccw;
					temp.push_back(CircularArc());
					temp[i].c.r = cr;
					temp[i].ccw = ccw;
					case_c.insert(i); // process after other arcs are done.
					break;
				case 'l':
					// given : two points on the arc, radius, and which side of the chord the center lies
					//	devised to easily approximate straight lines
					//		connect Point x0(cx,cy) and Point x1(t0,t1) to form a line (approximated with radius cr)
					// its center lies in the left (view-direction = tangent at cx,cy) if (ccw == true)
					// cr : should be at least bigger than half of distance btw x0 and x1
					//		but no upper-bound of cr (but too big cr leads to numerical errors in mink/voronoi)
					// code is kinda messy as it reuses variables for case 'a'
					{
						ais >> cx >> cy >> t0 >> t1 >> cr >> ccw;
						Point x0(cx, cy);
						Point x1(t0, t1);

						//// get center with pythagoras
						//Point dx = x1 - x0;
						//double bsquare = (dx.length2()) / 4;
						//double asquare = cr * cr - bsquare;
						//double a = sqrt(asquare); // dist from circle_center to chord

						//// get normal direction using ccw
						//Point normal;
						//if (ccw)
						//	normal = cd::rotatePoint(dx, 90).normalize();
						//else
						//	normal = cd::rotatePoint(dx, -90).normalize();

						//Point center = (x1 + x0) * 0.5 + normal * a;

						//cd::constructArc(center, x0, x1, ccw);

						temp.push_back(cd::constructArc(x0, x1, cr, ccw));
					}
					break;
				default:
					break;
				}
			}

			// 3-2. take care of case c
			for (auto idx : case_c)
			{
				auto prev = (idx - 1) % asz;
				auto next = (idx + 1) % asz;
				Point x0 = temp[prev].x1();
				Point x1 = temp[next].x0();
				temp[idx] = cd::constructArc(x0, x1, temp[idx].c.r, temp[idx].ccw);
			}

			// processing of temp is done at this point.


			// 4. re-order temp (make it G0-continuous)
			vector<CircularArc> g0;
			planning::_Convert_VectorCircularArc_G0(temp, g0, 1);

			// 5. make g0 G1-continuous by adding eps-arcs
			vector<CircularArc>& g1 = arcOut;
			if (useNewerVersionOfG1conveter)
				planning::_Convert_VectorCircularArc_G1_new(g0, g1);
			else
				planning::_Convert_VectorCircularArc_G1(g0, g1);

		};

		// 2-2. Load L-shaped model
		vector<CircularArc> arcObjectL;
		vector<Circle> circObjectL;
		readArcModelAndProcessIt("Objects/L-shape/arc.txt", "Objects/L-shape/circ.txt", arcObjectL, circObjectL);

		// 2-3. Load L-shaped model 2
		vector<CircularArc> arcObjectL2;
		vector<Circle> circObjectL2;
		readArcModelAndProcessIt("Objects/L-shape2/arc.txt", "Objects/L-shape2/circ.txt", arcObjectL2, circObjectL2);

		// 2-4. Load L-shaped model 3
		vector<CircularArc> arcObjectL3;
		vector<Circle> circObjectL3;
		readArcModelAndProcessIt("Objects/L-shape3/arc.txt", "Objects/L-shape3/circ.txt", arcObjectL3, circObjectL3);

		// 2-5. Load square
		vector<CircularArc> arcObjectSq;
		vector<Circle> circObjectSq;
		readArcModelAndProcessIt("Objects/Square-shape/arc.txt", "Objects/Square-shape/circ.txt", arcObjectSq, circObjectSq);


		// 3. instantiate loaded models to build sceneOriginal/sceneCircles

		// 3-1. LAMBDA which instantiate vectors from section 2.
		auto appendModelToScene = [&sceneOriginal, &sceneCircles](vector<CircularArc>& arcs, vector<Circle>& circs, double scale, double rotationDegree, Point translation)
		{
			for (auto& arc : arcs)
				sceneOriginal.push_back(cd::translateArc(cd::rotateArc(cd::scaleArc(arc, scale), rotationDegree), translation));

			for (auto& circ : circs)
			{
				auto rad = scale * circ.r;
				auto center = cd::rotatePoint(scale * circ.c, rotationDegree) + translation;
				sceneCircles.push_back(Circle(center, rad));
			}
		};

		Point uniformTranslation = Point(0.15, 0.05);

		//appendModelToScene(arcObjectL, circObjectL, 0.3, 0, Point(-0.3, -0.3));
		//appendModelToScene(arcObjectL, circObjectL, 0.4, 180, Point(0.3, 0.3));
		appendModelToScene(arcObjectL3, circObjectL3, 1.3, -90, uniformTranslation + Point(-0.1, +0.1));
		appendModelToScene(arcObjectL2, circObjectL2, 0.4, -90, uniformTranslation + Point(-0.1, +0.1));
		appendModelToScene(arcObjectL3, circObjectL3, 1.3, +90, uniformTranslation + Point(-0.5, +0.03));
		appendModelToScene(arcObjectL2, circObjectL2, 0.4, +90, uniformTranslation + Point(-0.5, +0.03));
		appendModelToScene(arcObjectSq, circObjectSq, 0.8,  +0, uniformTranslation + Point(+0.07, - 0.3));
		appendModelToScene(arcObjectSq, circObjectSq, 0.4, +55, uniformTranslation + Point(-0.53, 0.3));



		// 4. build vec<AS> sceneProcessed(input to MinkSum)
		planning::_Convert_VectorCircularArc_To_MsInput(sceneOriginal, sceneProcessed);

	}

	// set voronoiBoundary;
	// should be properly ordered(G0-continuous)
	{
		double r = 2;
		CircularArc a(Point(0, 0), r, Point(+1, +0), Point(+0, +1));
		CircularArc b(Point(0, 0), r, Point(+0, +1), Point(-1, +0));
		CircularArc c(Point(0, 0), r, Point(-1, +0), Point(+0, -1));
		CircularArc d(Point(0, 0), r, Point(+0, -1), Point(+1, +0));

		// test
		double l = 1.3;
		double eps = 0.01;
		Point
			q1(+l, +l),
			q2(-l, +l),
			q3(-l, -l),
			q4(+l, -l);
		a = cd::constructArc(q1, q2, Point(-1, eps).normalize());
		b = cd::constructArc(q2, q3, Point(-eps, -1).normalize());
		c = cd::constructArc(q3, q4, Point(+1, -eps).normalize());
		d = cd::constructArc(q4, q1, Point(+eps, +1).normalize());

		//planning::voronoiBoundary.push_back(a);
		//planning::voronoiBoundary.push_back(b);
		//planning::voronoiBoundary.push_back(c);
		//planning::voronoiBoundary.push_back(d);
		
		
		/* deprecated as it breaks voronoi near boundary*/
		// make it smaller -> better for BVH
		std::vector<CircularArc> temp;
		temp.push_back(a);
		temp.push_back(b);
		temp.push_back(c);
		temp.push_back(d);
		
		int nPiece = 2;
		for (auto& arc : temp)
		{
			auto par = cd::getParameterOfArc(arc);
			double& rad0 = par.first;
			double& rad1 = par.second;
			double drad = rad1 - rad0;
			for (size_t i = 0; i < nPiece; i++)
			{
				double start = double(i) / nPiece;
				double end = double(i+1) / nPiece;
		
				start = start * drad + rad0;
				end   = end   * drad + rad0;
		
				planning::voronoiBoundary.push_back(cd::constructArc(arc, start, end));
			}
		}
		
	}

	postProcess(ModelInfo_CurrentModel.first, ModelInfo_CurrentModel.second);


	//debug : test if models_APPROX is continuous
	//CircularArc temp;
	//for (int i = 0; i < 8; i++)
	//{
	//	bool first = true;
	//	auto& model = Models_Approx[i]; // model : ith model
	//	for (size_t j = 0; j < model.size(); j++)
	//	{
	//		auto& arcs = model[j]; 
	//		for (size_t k = 0; k < arcs.Arcs.size(); k++)
	//		{
	//			CircularArc t = arcs.Arcs[k];
	//			if (first)
	//			{
	//				first = false;
	//				temp = t;
	//			}
	//			else
	//			{
	//				if (temp.x[1] == t.x[0])
	//				{ }
	//				else
	//					std::cout << "ERR : models_approx not continuous" << std::endl;
	//				temp = t;
	//			}
	//		}
	//	}
	//}
	//std::cout << "DONE TEST MODEL_APPROX" << std::endl;
	//~debug
	
}

/*!
*	\brief 입력받은 두 모델에 대하여 Arc Spline 근사하고, 각각의 Interior Disk를 회전하여 저장한다
*
*	\param FirstModel 첫 번째 모델의 Index
*	\param SecondModel 두 번째 모델의 Index
*/
void postProcess(int FirstModel, int SecondModel)
{
	// 두 모델이 같은 모델인지 판단한다
	// 모델이 같은 경우 예외처리를 하여 알고리즘을 가속화하기 때문이다
	if (DEBUG false /*FirstModel == SecondModel*/)
		ModelInfo_Identical = true;
	else
		ModelInfo_Identical = false;

	// Second Model의 Index가 FirstModel보다 큰 경우, 모델을 그대로 사용한다
	if (DEBUG true /*(SecondModel >= FirstModel)*/) {
		// 각각의 Frame에 대해서 계산한다
		for (int i = 0; i < numofframe; i++) {
			
			// 1. Erase Former data

			// 전 모델에서 계산되었던 데이터를 지워준다
			if (!Models_Rotated[i].empty())
				Models_Rotated[i].clear();

			if (!Models_Rotated_Approx[i].empty())
				Models_Rotated_Approx[i].clear();

			if (!InteriorDisks_Rotated[i].empty())
				InteriorDisks_Rotated[i].clear();

			// 2. make InteriorDisks_Rotated

			// 내부 Disks를 회전하여 저장한다
			// 반지름은 그대로 두고 중심만 회전한다
			InteriorDisks_Rotated[i] = InteriorDisks_Imported[FirstModel];

			for (int j = 0; j < (int)InteriorDisks_Rotated[i].size(); j++)
				InteriorDisks_Rotated[i][j].c = InteriorDisks_Imported[FirstModel][j].c.rotate(2 * PI * i / numofframe);

			// 3. build M_R and M_R_A
			
			// if (this is the case where original bezier curve does not exists)
			if (Model_from_arc[FirstModel])
			{
				planning::_Convert_VectorCircularArc_To_MsInput(Model_vca[FirstModel], Models_Rotated_Approx[i], i);
			}
			else
			{
				// Models_Imported[FirstModel]의 각 BezierCrv의 Control Point를 회전하여 회전한 모델의 정보를 저장한다
				// tempF : 회전한 모델 (아마 temp Figure라 tempF라고 했던 것 같아요)
				auto tempF = Models_Imported[FirstModel];
				for (int j = 0; j < (int)Models_Imported[FirstModel].size(); j++)
					for (int k = 0; k <= (int)Models_Imported[FirstModel][j].Deg; k++)
						tempF[j].P[k] = tempF[j].P[k].rotate(2 * PI * i / numofframe);

				// 회전한 모델을 x, y - extreme, inflection, curvature extreme point에서 쪼갠 뒤 이를 Models_Rotated[i]에 저장한다
				for (int j = 0; j < (int)tempF.size(); j++)
					tempF[j].segmentation(Models_Rotated[i]);

				// 모델이 완벽한지 테스트한다
				for (int j = 0; j < (int)Models_Rotated[i].size(); j++) {
					auto s = Models_Rotated[i][j].integrityTest();
					if (s.size() > 0)
						Models_Rotated[i].insert(Models_Rotated[i].begin() + j + 1, s.begin(), s.end());
				}

				// 이 모델을 Arc Spline으로 근사한다. 위에서 Curvature Monotone으로 잘랐기 때문에 Spiral임이 보장된다
				for (int j = 0; j < (int)Models_Rotated[i].size(); j++) {
					auto tempSpiral = ArcSpline(Models_Rotated[i][j]);
					// 마찬가지로 완전성을 테스트한다
					auto input = tempSpiral.integrityTest();
					Models_Rotated_Approx[i].insert(Models_Rotated_Approx[i].end(), input.begin(), input.end());
				}
			}
		}
	}

	// 첫 번째 모델의 Index가 더 큰 경우, 두 번째 모델을 y축으로 반전한다
	// Minkowski Sum은 교환법칙이 성립하기 때문에,
	// m + n과 n + m이 같은 결과를 나타낸다. 따라서 m > n인 경우 모델을 반전하여 그려본다
	else {
		for (int i = 0; i < numofframe; i++) {
			if (!Models_Rotated[i].empty())
				Models_Rotated[i].clear();

			if (!Models_Rotated_Approx[i].empty())
				Models_Rotated_Approx[i].clear();

			if (!InteriorDisks_Rotated[i].empty())
				InteriorDisks_Rotated[i].clear();

			InteriorDisks_Rotated[i] = InteriorDisks_Imported[FirstModel];

			for (int j = 0; j < (int)InteriorDisks_Rotated[i].size(); j++) {
				InteriorDisks_Rotated[i][j].c.P[0] = -InteriorDisks_Rotated[i][j].c.P[0];
				InteriorDisks_Rotated[i][j].c = InteriorDisks_Rotated[i][j].c.rotate(2 * PI * i / numofframe);
			}


			auto tempF = Models_Imported[FirstModel];

			// Control Point를 반전하고 x좌표를 뒤집는다
			// Control Point를 반전하는 이유는, y축 대칭시 모델의 회전 방향이 바뀌기 때문이다
			// (모델의 회전 방향이 반시계 방향이어야만 모델의 내부 외부 방향이 판단되기 때문이다)
			for (int i = 0; i < (int)Models_Imported[FirstModel].size(); i++) {
				for (int j = 0; j < 4; j++) {
					tempF[i].P[j].P[1] = Models_Imported[FirstModel][i].P[3 - j].P[1];
					tempF[i].P[j].P[0] = -Models_Imported[FirstModel][i].P[3 - j].P[0];
				}
			}

			for (int j = 0; j < (int)Models_Imported[FirstModel].size(); j++)
				for (int k = 0; k <= (int)Models_Imported[FirstModel][j].Deg; k++)
					tempF[j].P[k] = tempF[j].P[k].rotate(2 * PI * i / numofframe);

			for (int j = 0; j < (int)tempF.size(); j++)
				tempF[j].segmentation(Models_Rotated[i]);

			for (int j = 0; j < (int)Models_Rotated[i].size(); j++) {
				auto s = Models_Rotated[i][j].integrityTest();
				if (s.size() > 0)
					Models_Rotated[i].insert(Models_Rotated[i].begin() + j + 1, s.begin(), s.end());
			}

			for (int j = 0; j < (int)Models_Rotated[i].size(); j++) {
				auto tempSpiral = ArcSpline(Models_Rotated[i][j]);
				auto input = tempSpiral.integrityTest();
				Models_Rotated_Approx[i].insert(Models_Rotated_Approx[i].end(), input.begin(), input.end());
			}

		}
	}


	//debug : test if models_rotated_APPROX is continuous
	//CircularArc temp;
	//for (int i = 0; i < 360; i++)
	//{
	//	bool first = true;
	//	auto& model = Models_Rotated_Approx[i]; // model : ith model
	//	for (size_t j = 0; j < model.size(); j++)
	//	{
	//		auto& arcs = model[j];
	//		for (size_t k = 0; k < arcs.Arcs.size(); k++)
	//		{
	//			CircularArc t = arcs.Arcs[k];
	//			if (first)
	//			{
	//				first = false;
	//				temp = t;
	//			}
	//			else
	//			{
	//				if (temp.x[1] == t.x[0])
	//				{
	//				}
	//				else
	//					std::cout << "ERR : models_rotated_approx not continuous at " << FirstModel << ", " << SecondModel << std::endl;
	//				temp = t;
	//			}
	//		}
	//	}
	//}
	//std::cout << "DONE TEST MODEL_ROTATED_APPROX" << std::endl;
	//~debug
}


/*!
*	\brief File Descriptor를 닫는다
*/
void save()
{
	fclose(f);
}

/*!
*	\brief 첫 번째 모델의 frame과, 두 번째 모델의 figure2번째 Index를 가진 모델을 Minkowskisum 한다
*
*	\param frame 첫 번째 모델(postProcess 단계에서 미리 근사됨)의 현재 frame	: [0, 360)
*	\param figure2 두 번째 모델(initialize 단계에서 미리 근사됨)				: [0, 8)
*/
void minkowskisum(int frame, int figure2)
{
	// 1. init global & local vars
	{
		// 이전 frame에서 저장된 정보를 초기화
		Model_Result.clear();
		Cache_Trimming.reset();
		InteriorDisks_Convolution.clear();
		for (int i = 0; i < grid; i++)
			for (int j = 0; j < grid; j++) {
				Grid_Trimming.gCircles[i][j].clear();
				Grid_Trimming.cover[i][j] = false;
				Grid_Trimming.coverCircle[i][j] = NULL;
			}
	}

	// time
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();



	// 2. build GRID (used for trimming)
	{
		// 2-1. find minkowski sum of interior circles and sort it. 
			// Interior Disk를 계산
		auto input = InteriorDisks_Rotated[frame] + InteriorDisks_Imported[figure2]; // size of input = size of ID_R * ID_I;
		InteriorDisks_Convolution.insert(InteriorDisks_Convolution.end(), input.begin(), input.end());

			// Interior Disk를 반지름이 작은것부터 큰것 순서로 Sorting
		std::sort(InteriorDisks_Convolution.begin(), InteriorDisks_Convolution.end());

		// 2-2. Build GRID with 2-1
			//Grid Data Structure를 만듬
		for (int i = 0; i < (int)InteriorDisks_Convolution.size(); i++)
			Grid_Trimming.insert(&InteriorDisks_Convolution[i]);

		// 2-3. extract cache from 2-2.
			// 가장 큰 Disk를 뽑아 Cache에 미리 넣어놓는다
			// (Cache 초기화)
		for (int i = (int)InteriorDisks_Convolution.size() - 1; i > (int)InteriorDisks_Convolution.size() - cacheSize - 1; i--)
			Cache_Trimming.cache[InteriorDisks_Convolution.size() - 1 - i] = &InteriorDisks_Convolution[i];
	}



	// 3. build ConvolutionArcs
	// convolutionArcs will contain a superset of mink boundary.
	std::vector<ArcSpline> ConvolutionArcs;
	{
		// Conv Arc could be built(:= have same normals somewhere) in 2 cases.
		for (int i = 0; i < (int)Models_Rotated_Approx[frame].size(); i++)
			for (int j = 0; j < (int)Models_Approx[figure2].size(); j++) {
				if (		// case i.  same quadrant & same clockwise 
					(Models_Rotated_Approx[frame][i].xQuardrants == Models_Approx[figure2][j].xQuardrants) &&
					(Models_Rotated_Approx[frame][i].yQuardrants == Models_Approx[figure2][j].yQuardrants) &&
					(Models_Rotated_Approx[frame][i].ccw) && (Models_Approx[figure2][j].ccw))
					overlapTest(ConvolutionArcs, Models_Rotated_Approx[frame][i], Models_Approx[figure2][j]);
				else if (	// case ii. diff quadrant & diff clockwise 
					(Models_Rotated_Approx[frame][i].xQuardrants != Models_Approx[figure2][j].xQuardrants) &&
					(Models_Rotated_Approx[frame][i].yQuardrants != Models_Approx[figure2][j].yQuardrants) &&
					(Models_Rotated_Approx[frame][i].ccw != Models_Approx[figure2][j].ccw))
					overlapTestR(ConvolutionArcs, Models_Rotated_Approx[frame][i], Models_Approx[figure2][j]);
			}
	}

	// dbg_out
	if (false)
	{
		using namespace std;
		int i = 5, j = 4;
		cout << "case 1 1 257 5 4 result : " << 
			((Models_Rotated_Approx[frame][i].xQuardrants == Models_Approx[figure2][j].xQuardrants) &&
			(Models_Rotated_Approx[frame][i].yQuardrants == Models_Approx[figure2][j].yQuardrants) &&
			(Models_Rotated_Approx[frame][i].ccw) && (Models_Approx[figure2][j].ccw)) << endl;

		// into overlapTest
		std::vector<ArcSpline>& source = ConvolutionArcs;
		ArcSpline& lhs = Models_Rotated_Approx[frame][i];
		ArcSpline& rhs = Models_Approx[figure2][j];

			// 1. build ls, rs and call convolution_ArcSpline
	
			// 왼쪽 모델의 첫 Index와 끝 Index : ls[0], ls[1]
			// 오른쪽 모델의 첫 Index와 끝 Index : rs[0], rs[1]
			int ls[2], rs[2];
			// 어느 Topology로 두 Arc가 겹쳐있는지 확인한다
			short overlap = overlapCase(lhs, rhs);
			// 각 경우에 대하여 인자를 맞추어 Con

			cout << "case 1 1 257 5 4 overlap : " << overlap << endl;
			switch (overlap) {
				/*
					visually
						clockwise---------->>>
				lhs			 ---	----	 ----	-----	-----	  ---
				rhs			-----	 ----	----	 ---	  ---	-----
				return		  1		  3		  4		  6		  7?	  5?
				(notice that this is used on original models, and they are ccw. so n[0] to n[1] is ccw. So on left is n[1], while on right is n[0])
				*/
				// ls : lhs[idx] of idx ~ [ls[0], ls[1]] may have potential overlap... rs is for rhs
			case 1:
				ls[0] = 0;
				ls[1] = (int)lhs.Arcs.size() - 1;
				rs[0] = rhs.findIdx(lhs.n[0]);
				rs[1] = rhs.findIdx(lhs.n[1]);
				Convolution_ArcSpline(source, lhs, rhs, ls, rs, false);
				break;
			case 3:
				ls[0] = 0;
				ls[1] = lhs.findIdx(rhs.n[1]);
				rs[0] = rhs.findIdx(lhs.n[0]);
				rs[1] = (int)rhs.Arcs.size() - 1;
				Convolution_ArcSpline(source, lhs, rhs, ls, rs, false);
				break;
			case 4:
				ls[0] = lhs.findIdx(rhs.n[0]);
				ls[1] = (int)lhs.Arcs.size() - 1;
				rs[0] = 0;
				rs[1] = rhs.findIdx(lhs.n[1]);
				Convolution_ArcSpline(source, lhs, rhs, ls, rs, false);
				break;
			case 6:
				ls[0] = lhs.findIdx(rhs.n[0]);
				ls[1] = lhs.findIdx(rhs.n[1]);
				rs[0] = 0;
				rs[1] = (int)rhs.Arcs.size() - 1;
				Convolution_ArcSpline(source, lhs, rhs, ls, rs, false);
				break;
			case 7:
				ls[0] = 0; //this may be ineff?
				ls[1] = (int)lhs.Arcs.size() - 1;
				rs[0] = 0;
				rs[1] = (int)rhs.Arcs.size() - 1;
				Convolution_ArcSpline(source, lhs, rhs, ls, rs, false);
			default:
				break;
			}
	}

	// dbg_out
	if (planning::keyboardflag['1'])
	{
		using namespace std;
		cout << "drawing all convolution arcs" << endl;
		Model_Result.push_back(deque<ArcSpline>(ConvolutionArcs.begin(), ConvolutionArcs.end()));
		return;
	}


	/* 4. Local Trimming - Self Intersection */
	// 각 Convolution Arc Spline의 충돌여부를 조사
	for (int i = 0; i < (int)ConvolutionArcs.size() - 1; i++)
		for (int j = i + 1; j < (int)ConvolutionArcs.size(); j++)
			// connected 과정에서 인접한 Arc Spline Segment끼리 서로 연결함
			if (!connected(ConvolutionArcs[i], ConvolutionArcs[j]) && aabbtest(ConvolutionArcs[i], ConvolutionArcs[j]))
				selfIntersectionPts(ConvolutionArcs[i], ConvolutionArcs[j]);

	// Convolution한 이후의 각 Arc Spline Segment에 대하여 Self-Intersection에 의한 Trimming
	// Self Intersection이 없는 경우 : skip
	// 1개 있는 경우 : 잘리는 방향으로 따라가며 Arc Spline Segment를 제거
	// 2개 이상 있는 경우 : 더 자세히 Arc Spline의 상황을 조사하며 Trimming 
	std::vector<ArcSpline> temp;
	for (int i = 0; i < (int)ConvolutionArcs.size(); i++)
		if ((!ConvolutionArcs[i].referenced))
			switch ((int)ConvolutionArcs[i].intersections.size()) {
			case 0:
				break;
			case 1:
				ConvolutionArcs[i].finalTrimming_Simple();
				break;
			default:
				auto input = ConvolutionArcs[i].finalTrimming_Complex();
				// 2개 이상의 교점을 가진 Arc Spline Segment는 쪼갠 뒤(split = true) temp에 따로 담아놓음
				temp.insert(temp.end(), input.begin(), input.end());
				break;
			}
	// 이후 temp를 Convolution Arc 뒤에 추가
	ConvolutionArcs.insert(ConvolutionArcs.end(), temp.begin(), temp.end());

	// 위의 상황에서 2개 이상의 교점을 가져 쪼개진 Arc Spline은(split = true) temp와 중복되므로 여기서 지워줌
	// index를 뒤에서부터 접근 -> vector의 erase시 발생하는 복사를 최소화하기 위함
	// 이 외의 경우 인접관계를 초기화함
	for (int i = (int)ConvolutionArcs.size() - 1; i >= 0; i--) {
		if ((ConvolutionArcs[i].Arcs.size() == 0) || ConvolutionArcs[i].splited)
			ConvolutionArcs.erase(ConvolutionArcs.begin() + i);
		else {
			ConvolutionArcs[i].neighborDist[0] = ConvolutionArcs[i].neighborDist[1] = DBL_MAX;
			ConvolutionArcs[i].neighbor[0] = ConvolutionArcs[i].neighbor[1] = NULL;
			ConvolutionArcs[i]._neighbor[0] = ConvolutionArcs[i]._neighbor[1] = false;
			ConvolutionArcs[i].referenced = false;
		}
	}

	// 인접관계를 다시 계산
	for (int i = 0; i < (int)ConvolutionArcs.size() - 1; i++)
		for (int j = i + 1; j < (int)ConvolutionArcs.size(); j++) {
			connected(ConvolutionArcs[i], ConvolutionArcs[j]);
		}

	// 인접관계를 묶어 각각을 Deque 자료구조로 정리 (Loop를 만든다)
	for (int i = 0; i < (int)ConvolutionArcs.size(); i++) {
		if (!ConvolutionArcs[i].referenced) {
			ConvolutionArcs[i].referenced = true;
			std::deque<ArcSpline> input;
			input.push_back(ConvolutionArcs[i]);
			bool a = ConvolutionArcs[i]._neighbor[0];
			ArcSpline* b = ConvolutionArcs[i].neighbor[0];
			while (a) {
				input.push_back(*b);
				b->referenced = true;
				if (b->_neighbor[0] && b->neighbor[1]) {
					a = true;
					if (!b->neighbor[0]->referenced) {
						b = b->neighbor[0];
					}
					else if (!b->neighbor[1]->referenced) {
						b = b->neighbor[1];
					}
					else {
						a = false;
					}
				}
				else {
					a = false;
				}
			}
			a = ConvolutionArcs[i]._neighbor[1];
			b = ConvolutionArcs[i].neighbor[1];
			while (a) {
				input.push_front(*b);
				b->referenced = true;
				if (b->_neighbor[0] && b->neighbor[1]) {
					if (!b->neighbor[0]->referenced) {
						b = b->neighbor[0];
					}
					else if (!b->neighbor[1]->referenced) {
						b = b->neighbor[1];
					}
					else {
						a = false;
					}
				}
				else {
					a = false;
				}
			}
			Model_Result.push_back(input);
		}
	}


	/* 5. Local Trimming - Collision Detection */
	// 각 Loop의 Arc들 중 x, y의 min / max를 구한다.
	// min이나 max의 Arc를 포함하는 경우, 반드시 그 Loop는 Boundary임이 보장된다
	// min이나 max값을 갖지 않는 Loop의 경우, Inner Loop이거나,(남겨야 하는 Loop) 위의 Trimming 단계에서 사라지지 못한 Segment(Trimming 해야 하는 Loop)이다
	double xmin = DBL_MAX, ymin = DBL_MAX, xmax = -DBL_MAX, ymax = -DBL_MAX;
	int xminIdx, yminIdx, xmaxIdx, ymaxIdx;
	// x, y min / max인 Arc를 포함하는 Loop를 찾는다
	for (int i = 0; i < (int)Model_Result.size(); i++)
		for (int j = 0; j < (int)Model_Result[i].size(); j++)
			for (int k = 0; k < (int)Model_Result[i][j].Arcs.size(); k++) {
				if (Model_Result[i][j].Arcs[k].x[0].P[0] > xmax) {
					xmax = Model_Result[i][j].Arcs[k].x[0].P[0];
					xmaxIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[0] > xmax) {
					xmax = Model_Result[i][j].Arcs[k].x[1].P[0];
					xmaxIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[0].P[0] < xmin) {
					xmin = Model_Result[i][j].Arcs[k].x[0].P[0];
					xminIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[0] < xmin) {
					xmin = Model_Result[i][j].Arcs[k].x[1].P[0];
					xminIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[0].P[1] > ymax) {
					ymax = Model_Result[i][j].Arcs[k].x[0].P[1];
					ymaxIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[1] > ymax) {
					ymax = Model_Result[i][j].Arcs[k].x[1].P[1];
					ymaxIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[0].P[1] < ymin) {
					ymin = Model_Result[i][j].Arcs[k].x[0].P[1];
					yminIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[1] < ymin) {
					ymin = Model_Result[i][j].Arcs[k].x[1].P[1];
					yminIdx = i;
				}
			}

	// 그 Loop를 제외한 Loop의 임의의 한 점에 대해 Collision Detection Test를 진행한다
	// Collision Detection이 발생하면 Trimming되어야만 하는 내부의 Segment이므로, 지워준다
#if useApproxInSymmetryCollision
	for (int i = 0; i < (int)Model_Result.size(); i++)
		if (!(i == xmaxIdx || i == xminIdx || i == yminIdx || i == ymaxIdx))
			if (Model_Result[i].front().Arcs.size() != 0)
				if (collision(Models_Rotated_Approx[frame], Models_Approx[figure2], Model_Result[i].front().mid()))
					Model_Result[i].clear();
#else

	for (int i = 0; i < (int)Model_Result.size(); i++)
		if (!(i == xmaxIdx || i == xminIdx || i == yminIdx || i == ymaxIdx))
			if (Model_Result[i].front().Arcs.size() != 0)
				if (collision(Models_Rotated[frame], Models[figure2], Model_Result[i].front().mid()))
					Model_Result[i].clear();
#endif

	/* 6. Local Trimming - Loop */
	// Loop를 이루지 못하는 경우 지워준다

	// For cases where loop is of 0~2 arcsplines.
	for (int i = (int)Model_Result.size() - 1; i >= 0; i--) {
		if (Model_Result[i].size() == 0)
			Model_Result.erase(Model_Result.begin() + i);
		// ArcSpline 개수가 매우 작은 경우 예외처리
		else if (Model_Result[i].size() < 3) {
			Point init1;
			Point end1;
			if (Model_Result[i].size() != 1) {
				if (!Model_Result[i].front()._neighbor[0])
					init1 = Model_Result[i].front().init();
				else
					init1 = Model_Result[i].front().end();
				if (!Model_Result[i].back()._neighbor[1])
					end1 = Model_Result[i].back().end();
				else
					end1 = Model_Result[i].back().init();
			}
			else {
				init1 = Model_Result[i][0].init();
				end1 = Model_Result[i][0].end();
			}

			if (distance(init1, end1) < EPSILON * EPSILON)	// if (size = 1 or 2 && arcSpline is too short???) erase
				Model_Result.erase(Model_Result.begin() + i);
		}
	}
	// Inner Loop가 있는 경우 Loop Test를 진행
	if (Model_Result.size() != 1) {
		for (int i = 0; i < (int)Model_Result.size() - 1; i++) {
			bool recheck = false;
			for (int j = i + 1; j < (int)Model_Result.size(); j++) {
				if ((Model_Result[i].size() != 0) && (Model_Result[j].size() != 0)) {
					Point init1;
					Point init2;
					Point end1;
					Point end2;
					if (Model_Result[i].size() != 1) {
						if (!Model_Result[i].front()._neighbor[0])
							init1 = Model_Result[i].front().init();
						else
							init1 = Model_Result[i].front().end();
						if (!Model_Result[i].back()._neighbor[1])
							end1 = Model_Result[i].back().end();
						else
							end1 = Model_Result[i].back().init();
					}
					else {
						init1 = Model_Result[i][0].init();
						end1 = Model_Result[i][0].end();
					}
					if (Model_Result[j].size() != 1) {
						if (!Model_Result[j].front()._neighbor[0])
							init2 = Model_Result[j].front().init();
						else
							init2 = Model_Result[j].front().end();
						if (!Model_Result[j].back()._neighbor[1])
							end2 = Model_Result[j].back().end();
						else
							end2 = Model_Result[j].back().init();
					}
					else {
						init2 = Model_Result[j][0].init();
						end2 = Model_Result[j][0].end();
					}

					// Loop를 이루지 못하면 Erase
					if (distance(init1, init2) < N_PRESCISION) {
						for (int k = 0; k < (int)Model_Result[j].size(); k++)
							Model_Result[i].push_front(Model_Result[j][k]);
						Model_Result.erase(Model_Result.begin() + j);
						j--;
						recheck = true;
					}

					else if (distance(init1, end2) < N_PRESCISION) {
						for (int k = (int)Model_Result[j].size() - 1; k >= 0; k--)
							Model_Result[i].push_front(Model_Result[j][k]);
						Model_Result.erase(Model_Result.begin() + j);
						j--;
						recheck = true;
					}

					else if (distance(end1, init2) < N_PRESCISION) {
						for (int k = 0; k < (int)Model_Result[j].size(); k++)
							Model_Result[i].push_back(Model_Result[j][k]);
						Model_Result.erase(Model_Result.begin() + j);
						j--;
						recheck = true;
					}

					else if (distance(end1, end2) < N_PRESCISION) {
						for (int k = (int)Model_Result[j].size() - 1; k >= 0; k--)
							Model_Result[i].push_back(Model_Result[j][k]);
						Model_Result.erase(Model_Result.begin() + j);
						j--;
						recheck = true;
					}
				}
			}
			if (recheck) {
				i--;
			}
		}
	}

	for (int i = (int)Model_Result.size() - 1; i >= 0; i--) {
		if (Model_Result[i].size() < 3)
			Model_Result.erase(Model_Result.begin() + i);
	}

	// 각 Loop에서 x, y의 min / max값을 계산한 뒤, Boundary를 이루는지 Inner Loop을 이루는지를 판단
	std::vector<std::vector<CircularArc*>> arc; // For each loops, find 4 indcies of arcs which each is xmax, xmin, ymax, ymin int that loop.
	arc.resize(4);
	for (int i = 0; i < 4; i++)
		arc[i].resize(Model_Result.size());
	ModelInfo_Boundary.resize(Model_Result.size());
	xmin = DBL_MAX, ymin = DBL_MAX, xmax = -DBL_MAX, ymax = -DBL_MAX;
	for (int i = 0; i < (int)Model_Result.size(); i++) {
		for (int j = 0; j < (int)Model_Result[i].size(); j++)
			for (int k = 0; k < (int)Model_Result[i][j].Arcs.size(); k++) {
				if (Model_Result[i][j].Arcs[k].x[0].P[0] > xmax) {
					xmax = Model_Result[i][j].Arcs[k].x[0].P[0];
					arc[0][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[0] > xmax) {
					xmax = Model_Result[i][j].Arcs[k].x[1].P[0];
					arc[0][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[0].P[0] < xmin) {
					xmin = Model_Result[i][j].Arcs[k].x[0].P[0];
					arc[1][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[0] < xmin) {
					xmin = Model_Result[i][j].Arcs[k].x[1].P[0];
					arc[1][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[0].P[1] > ymax) {
					ymax = Model_Result[i][j].Arcs[k].x[0].P[1];
					arc[2][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[1] > ymax) {
					ymax = Model_Result[i][j].Arcs[k].x[1].P[1];
					arc[2][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[0].P[1] < ymin) {
					ymin = Model_Result[i][j].Arcs[k].x[0].P[1];
					arc[3][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[1] < ymin) {
					ymin = Model_Result[i][j].Arcs[k].x[1].P[1];
					arc[3][i] = &Model_Result[i][j].Arcs[k];
				}
			}
		xmin = DBL_MAX, ymin = DBL_MAX, xmax = -DBL_MAX, ymax = -DBL_MAX;
	}

	for (int i = 0; i < (int)Model_Result.size(); i++) {
		ModelInfo_Boundary[i] = arc[0][i]->isOuterBoundary(0) && arc[1][i]->isOuterBoundary(1) && arc[2][i]->isOuterBoundary(2) && arc[3][i]->isOuterBoundary(3);
	}

	std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
	std::chrono::microseconds micro = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

	//------------------ Trimming Process 종료 ------------------//

	/* 7. Export Data */

	fprintf(f, "%d\t%d\t%d\t%lf\n", ModelInfo_CurrentModel.first, ModelInfo_CurrentModel.second, ModelInfo_CurrentFrame, (micro.count() / 1000.0));

	/* 8. Memory Management */
	for (int i = 0; i < (int)Models_Rotated[frame].size(); i++) {
		if (Models_Rotated[frame][i].child[0] != NULL) {
			//debug
//			Models_Rotated[frame][i].child[0]->freeMemory();
//			Models_Rotated[frame][i].child[1]->freeMemory();
//			delete Models_Rotated[frame][i].child[0];
//			delete Models_Rotated[frame][i].child[1];
		}
	}

	//// 9. some code to make it work with MAT (only test left now)
	//for (size_t i = 0; i < Model_Result.size(); i++)
	//{
	//	for (size_t j = 0;  j < Model_Result[i].size();  j++)
	//	{
	//		for (size_t k = 0; k < Model_Result[i][j].Arcs.size() - 1; k++)
	//		{
	//			float t = Model_Result[i][j].Arcs[k].x[1].P[0] - Model_Result[i][j].Arcs[k + 1].x[0].P[0];
	//			if (fabs(t) > 1e-4) std::cout << "k not continuous ...? " << t << " " << i << " " << j << " " << k << std::endl;
	//		}
	//	}
	//}
}


void minkowskisum_id(int frame, int figure2)
{
	Model_Result.clear();
	Cache_Trimming.reset();
	InteriorDisks_Convolution.clear();
	for (int i = 0; i < grid; i++)
		for (int j = 0; j < grid; j++) {
			Grid_Trimming.gCircles[i][j].clear();
			Grid_Trimming.cover[i][j] = false;
			Grid_Trimming.coverCircle[i][j] = NULL;
		}

	std::vector<ArcSpline> cTrimmed;
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
	auto input = InteriorDisks_Rotated[frame] + InteriorDisks_Imported[figure2];
	InteriorDisks_Convolution.insert(InteriorDisks_Convolution.end(), input.begin(), input.end());

	std::sort(InteriorDisks_Convolution.begin(), InteriorDisks_Convolution.end());

	for (int i = 0; i < (int)InteriorDisks_Convolution.size(); i++)
		Grid_Trimming.insert(&InteriorDisks_Convolution[i]);

	for (int i = (int)InteriorDisks_Convolution.size() - 1; i > (int)InteriorDisks_Convolution.size() - cacheSize - 1; i--)
		Cache_Trimming.cache[InteriorDisks_Convolution.size() - 1 - i] = &InteriorDisks_Convolution[i];

	for (int i = 0; i < (int)Models_Approx[figure2].size(); i++)
		for (int j = i + 1; j < (int)Models_Approx[figure2].size(); j++) {
			if ((Models_Approx[figure2][i].xQuardrants == Models_Approx[figure2][j].xQuardrants) && (Models_Approx[figure2][i].yQuardrants == Models_Approx[figure2][j].yQuardrants) && (Models_Approx[figure2][i].ccw) && (Models_Approx[figure2][j].ccw))
				overlapTest(cTrimmed, Models_Approx[figure2][i], Models_Approx[figure2][j]);
			else if ((Models_Approx[figure2][i].xQuardrants != Models_Approx[figure2][j].xQuardrants) && (Models_Approx[figure2][i].yQuardrants != Models_Approx[figure2][j].yQuardrants) && (Models_Approx[figure2][i].ccw != Models_Approx[figure2][j].ccw))
				overlapTestR(cTrimmed, Models_Approx[figure2][i], Models_Approx[figure2][j]);

		}
	for(int i = 0; i < (int)Models_Approx[figure2].size(); i++)
		if (Models_Approx[figure2][i].ccw)
			overlapTestIden(cTrimmed, Models_Approx[figure2][i]);


	for (int i = 0; i < (int)cTrimmed.size(); i++)
		cTrimmed[i].referenced = false;

	for (int i = 0; i < (int)cTrimmed.size() - 1; i++)
		for (int j = i + 1; j < (int)cTrimmed.size(); j++)
			if (!connected(cTrimmed[i], cTrimmed[j]) && aabbtest(cTrimmed[i], cTrimmed[j]))
				selfIntersectionPts(cTrimmed[i], cTrimmed[j]);



	std::vector<ArcSpline> temp;
	for (int i = 0; i < (int)cTrimmed.size(); i++)
		if ((!cTrimmed[i].referenced))
			switch ((int)cTrimmed[i].intersections.size()) {
			case 0:
				break;
			case 1:
				cTrimmed[i].finalTrimming_Simple();
				break;
			default:
				auto input = cTrimmed[i].finalTrimming_Complex();
				temp.insert(temp.end(), input.begin(), input.end());
				break;
			}
	cTrimmed.insert(cTrimmed.end(), temp.begin(), temp.end());

	for (int i = (int)cTrimmed.size() - 1; i >= 0; i--) {
		if ((cTrimmed[i].Arcs.size() == 0) || cTrimmed[i].splited)
			cTrimmed.erase(cTrimmed.begin() + i);
		else {
			cTrimmed[i].neighborDist[0] = cTrimmed[i].neighborDist[1] = DBL_MAX;
			cTrimmed[i].neighbor[0] = cTrimmed[i].neighbor[1] = NULL;
			cTrimmed[i]._neighbor[0] = cTrimmed[i]._neighbor[1] = false;
			cTrimmed[i].referenced = false;
		}
	}

	for (int i = 0; i < (int)cTrimmed.size() - 1; i++)
		for (int j = i + 1; j < (int)cTrimmed.size(); j++) {
			connected(cTrimmed[i], cTrimmed[j]);
		}


	for (int i = 0; i < (int)cTrimmed.size(); i++) {
		if (!cTrimmed[i].referenced) {
			cTrimmed[i].referenced = true;
			std::deque<ArcSpline> input;
			input.push_back(cTrimmed[i]);
			bool a = cTrimmed[i]._neighbor[0];
			ArcSpline* b = cTrimmed[i].neighbor[0];
			while (a) {
				input.push_back(*b);
				b->referenced = true;
				if (b->_neighbor[0] && b->neighbor[1]) {
					a = true;
					if (!b->neighbor[0]->referenced) {
						b = b->neighbor[0];
					}
					else if (!b->neighbor[1]->referenced) {
						b = b->neighbor[1];
					}
					else {
						a = false;
					}
				}
				else {
					a = false;
				}
			}
			a = cTrimmed[i]._neighbor[1];
			b = cTrimmed[i].neighbor[1];
			while (a) {
				input.push_front(*b);
				b->referenced = true;
				if (b->_neighbor[0] && b->neighbor[1]) {
					if (!b->neighbor[0]->referenced) {
						b = b->neighbor[0];
					}
					else if (!b->neighbor[1]->referenced) {
						b = b->neighbor[1];
					}
					else {
						a = false;
					}
				}
				else {
					a = false;
				}
			}
			Model_Result.push_back(input);
		}
	}

	double xmin = DBL_MAX, ymin = DBL_MAX, xmax = -DBL_MAX, ymax = -DBL_MAX;
	int xminIdx, yminIdx, xmaxIdx, ymaxIdx;


	for (int i = 0; i < (int)Model_Result.size(); i++)
		for (int j = 0; j < (int)Model_Result[i].size(); j++)
			for (int k = 0; k < (int)Model_Result[i][j].Arcs.size(); k++) {
				if (Model_Result[i][j].Arcs[k].x[0].P[0] > xmax) {
					xmax = Model_Result[i][j].Arcs[k].x[0].P[0];
					xmaxIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[0] > xmax) {
					xmax = Model_Result[i][j].Arcs[k].x[1].P[0];
					xmaxIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[0].P[0] < xmin) {
					xmin = Model_Result[i][j].Arcs[k].x[0].P[0];
					xminIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[0] < xmin) {
					xmin = Model_Result[i][j].Arcs[k].x[1].P[0];
					xminIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[0].P[1] > ymax) {
					ymax = Model_Result[i][j].Arcs[k].x[0].P[1];
					ymaxIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[1] > ymax) {
					ymax = Model_Result[i][j].Arcs[k].x[1].P[1];
					ymaxIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[0].P[1] < ymin) {
					ymin = Model_Result[i][j].Arcs[k].x[0].P[1];
					yminIdx = i;
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[1] < ymin) {
					ymin = Model_Result[i][j].Arcs[k].x[1].P[1];
					yminIdx = i;
				}
			}


	for (int i = 0; i < (int)Model_Result.size(); i++)
		if (!(i == xmaxIdx || i == xminIdx || i == yminIdx || i == ymaxIdx))
			if (Model_Result[i].front().Arcs.size() != 0)
				if (collision(Models[figure2], Models[figure2], Model_Result[i].front().mid()))
					Model_Result[i].clear();

	// for robustness
	for (int i = (int)Model_Result.size() - 1; i >= 0; i--) {
		if (Model_Result[i].size() == 0)
			Model_Result.erase(Model_Result.begin() + i);
		else if (Model_Result[i].size() < 3) {
			Point init1;
			Point end1;
			if (Model_Result[i].size() != 1) {
				if (!Model_Result[i].front()._neighbor[0])
					init1 = Model_Result[i].front().init();
				else
					init1 = Model_Result[i].front().end();
				if (!Model_Result[i].back()._neighbor[1])
					end1 = Model_Result[i].back().end();
				else
					end1 = Model_Result[i].back().init();
			}
			else {
				init1 = Model_Result[i][0].init();
				end1 = Model_Result[i][0].end();
			}

			if (distance(init1, end1) < EPSILON * EPSILON)
				Model_Result.erase(Model_Result.begin() + i);
		}
	}

	if (Model_Result.size() != 1) {
		for (int i = 0; i < (int)Model_Result.size() - 1; i++) {
			bool recheck = false;
			for (int j = i + 1; j < (int)Model_Result.size(); j++) {
				if ((Model_Result[i].size() != 0) && (Model_Result[j].size() != 0)) {
					Point init1;
					Point init2;
					Point end1;
					Point end2;
					if (Model_Result[i].size() != 1) {
						if (!Model_Result[i].front()._neighbor[0])
							init1 = Model_Result[i].front().init();
						else
							init1 = Model_Result[i].front().end();
						if (!Model_Result[i].back()._neighbor[1])
							end1 = Model_Result[i].back().end();
						else
							end1 = Model_Result[i].back().init();
					}
					else {
						init1 = Model_Result[i][0].init();
						end1 = Model_Result[i][0].end();
					}
					if (Model_Result[j].size() != 1) {
						if (!Model_Result[j].front()._neighbor[0])
							init2 = Model_Result[j].front().init();
						else
							init2 = Model_Result[j].front().end();
						if (!Model_Result[j].back()._neighbor[1])
							end2 = Model_Result[j].back().end();
						else
							end2 = Model_Result[j].back().init();
					}
					else {
						init2 = Model_Result[j][0].init();
						end2 = Model_Result[j][0].end();
					}

					if (distance(init1, init2) < N_PRESCISION) {
						for (int k = 0; k < (int)Model_Result[j].size(); k++)
							Model_Result[i].push_front(Model_Result[j][k]);
						Model_Result.erase(Model_Result.begin() + j);
						j--;
						recheck = true;
					}

					else if (distance(init1, end2) < N_PRESCISION) {
						for (int k = (int)Model_Result[j].size() - 1; k >= 0; k--)
							Model_Result[i].push_front(Model_Result[j][k]);
						Model_Result.erase(Model_Result.begin() + j);
						j--;
						recheck = true;
					}

					else if (distance(end1, init2) < N_PRESCISION) {
						for (int k = 0; k < (int)Model_Result[j].size(); k++)
							Model_Result[i].push_back(Model_Result[j][k]);
						Model_Result.erase(Model_Result.begin() + j);
						j--;
						recheck = true;
					}

					else if (distance(end1, end2) < N_PRESCISION) {
						for (int k = (int)Model_Result[j].size() - 1; k >= 0; k--)
							Model_Result[i].push_back(Model_Result[j][k]);
						Model_Result.erase(Model_Result.begin() + j);
						j--;
						recheck = true;
					}
				}
			}
			if (recheck) {
				i--;
			}
		}
	}

	for (int i = (int)Model_Result.size() - 1; i >= 0; i--) {
		if (Model_Result[i].size() < 3)
			Model_Result.erase(Model_Result.begin() + i);
	}


	std::vector<std::vector<CircularArc*>> arc;
	arc.resize(4);
	for (int i = 0; i < 4; i++)
		arc[i].resize(Model_Result.size());
	ModelInfo_Boundary.resize(Model_Result.size());
	xmin = DBL_MAX, ymin = DBL_MAX, xmax = -DBL_MAX, ymax = -DBL_MAX;
	for (int i = 0; i < (int)Model_Result.size(); i++) {
		for (int j = 0; j < (int)Model_Result[i].size(); j++)
			for (int k = 0; k < (int)Model_Result[i][j].Arcs.size(); k++) {
				if (Model_Result[i][j].Arcs[k].x[0].P[0] > xmax) {
					xmax = Model_Result[i][j].Arcs[k].x[0].P[0];
					arc[0][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[0] > xmax) {
					xmax = Model_Result[i][j].Arcs[k].x[1].P[0];
					arc[0][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[0].P[0] < xmin) {
					xmin = Model_Result[i][j].Arcs[k].x[0].P[0];
					arc[1][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[0] < xmin) {
					xmin = Model_Result[i][j].Arcs[k].x[1].P[0];
					arc[1][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[0].P[1] > ymax) {
					ymax = Model_Result[i][j].Arcs[k].x[0].P[1];
					arc[2][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[1] > ymax) {
					ymax = Model_Result[i][j].Arcs[k].x[1].P[1];
					arc[2][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[0].P[1] < ymin) {
					ymin = Model_Result[i][j].Arcs[k].x[0].P[1];
					arc[3][i] = &Model_Result[i][j].Arcs[k];
				}
				if (Model_Result[i][j].Arcs[k].x[1].P[1] < ymin) {
					ymin = Model_Result[i][j].Arcs[k].x[1].P[1];
					arc[3][i] = &Model_Result[i][j].Arcs[k];
				}
			}
		xmin = DBL_MAX, ymin = DBL_MAX, xmax = -DBL_MAX, ymax = -DBL_MAX;
	}

	for (int i = 0; i < (int)Model_Result.size(); i++) {
		ModelInfo_Boundary[i] = arc[0][i]->isOuterBoundary(0) && arc[1][i]->isOuterBoundary(1) && arc[2][i]->isOuterBoundary(2) && arc[3][i]->isOuterBoundary(3);
	}

	std::chrono::system_clock::time_point end = std::chrono::system_clock::now();

	std::chrono::microseconds micro = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

	fprintf(f, "%d\t%d\t%d\t%lf\n", ModelInfo_CurrentModel.first, ModelInfo_CurrentModel.second, ModelInfo_CurrentFrame, (micro.count() / 1000.0));
}

#pragma region operators & simple funcs

/*!
*	\brief 생성자
*
*	\param x x-좌표
*	\param y y-좌표
*/
Point::Point(double x, double y)
{
	P[0] = x;
	P[1] = y;
}

/*!
*	\brief 복사 생성자
*
*	\param cpy 복사될 객체
*/
Point::Point(const Point &cpy)
{
	P[0] = cpy.P[0];
	P[1] = cpy.P[1];
}

/*!
*	\brief 두 직선의 교점
*
*	\param l 첫번째 직선
*	\param m 두번재 직선
*/
Point::Point(Line & l, Line & m)
{
	// temp 외적 연산 후 homogeneous coor.에서 세번째 좌표(이 값으로 나누어 1로 맞춰주어야 한다.)
	double temp = l.L[0] * m.L[1] - l.L[1] * m.L[0];
	P[0] = (l.L[1] * m.L[2] - l.L[2] * m.L[1]) / temp;
	P[1] = (l.L[2] * m.L[0] - l.L[0] * m.L[2]) / temp;
}

/*!
*	\brief CircularArc를 삼각형으로 Bounding한 Bounding Volume의 세 번째 점 (Circular Arc의 양 끝점에서 만든 두 접선의 교점)
*
*	\param c Bounding Volume을 만드는 Circular Arc
*/
Point::Point(CircularArc & c)
{
	(*this) = Point(Line(c.x[0], c.x[0] + (c.x[0] - c.c.c).rotate()), Line(c.x[1], c.x[1] + (c.x[1] - c.c.c).rotate()));
}

/*!
*	\brief 소멸자
*/
Point::~Point()
{
}

/*!
*	\brief 합에 대한 역원
*
*	\return 합에 대한 역원을 반환한다.
*/
Point Point::operator-() const
{
	return Point(-P[0], -P[1]);
}

/*!
*	\brief 대입 연산자
*
*	\param rhs 대입될 객체
*
*	\return 대입된 자신을 반환한다.
*/
Point & Point::operator=(const Point &rhs)
{
	P[0] = rhs.P[0];
	P[1] = rhs.P[1];

	return *this;
}


/*!
*	\brief 두 점이 같은지 판단한다.
*
*	\param rhs 비교하는 객체
*
*	\return 같으면 true, 다르면 false를 반환한다.
*/
bool Point::operator==(Point & rhs)
{
	return (abs(P[0] - rhs.P[0]) < N_PRESCISION) && (abs(P[1] - rhs.P[1]) < N_PRESCISION);
}

/*!
*	\brief 두 벡터의 크기를 비교
*
*	\param rhs 비교하는 객체
*
*	\return rhs가 더 크면 true, 작으면 false를 반환한다.
*/
bool Point::operator<(Point & rhs)
{
	return (*this * *this < rhs * rhs);
}

/*!
*	\brief 두 벡터의 크기를 비교
*
*	\param rhs 비교하는 객체
*
*	\return rhs가 더 작으면 true, 크면 false를 반환한다.
*/
bool Point::operator>(Point & rhs)
{
	return (*this * *this > rhs*rhs);
}

/*!
*	\brief 두 점이 가까운지를 판단
*
*	\param rhs 비교하는 객체
*
*	\return 거리가 특정 범위보다 작으면 true, 크면 false를 반환한다.
*/
bool Point::close(Point & rhs)
{
	return (abs(P[0] - rhs.P[0]) < N_HIGH_PRESCISION) && (abs(P[1] - rhs.P[1]) < N_HIGH_PRESCISION);
}

/*!
*	\brief 두 점이 일치하는지 판단
*
*	\param rhs 비교하는 객체
*
*	\return 두 점이 완벽히 일치하면 true를 반환한다.
*/
bool Point::exact(Point & rhs)
{
	return (abs(P[0] - rhs.P[0]) < 1e-10) && (abs(P[1] - rhs.P[1]) < 1e-10);
}

/*!
*	\brief 벡터의 길이의 제곱
*
*	\return 벡터의 길이의 제곱을 반환한다.
*/
double Point::length()
{
	return *this * *this;
}

/*!
*	\brief 단위벡터를 만든다
*
*	\return 방향이 같고 크기가 1인 벡터를 반환한다.
*/
Point &Point::normalize()
{
	double len = sqrt(length());
	P[0] /= len;
	P[1] /= len;

	return *this;
}

/*!
*	\brief 벡터를 회전한다
*
*	\return 반시계방향으로 90도 회전시킨 벡터를 반환한다.
*/
Point Point::rotate()
{
	return Point(-P[1], P[0]);
}

Point Point::rotate(double angle)
{
	Point reval;
	reval.P[0] = (*this).P[0] * cos(angle) - (*this).P[1] * sin(angle);
	reval.P[1] = (*this).P[0] * sin(angle) + (*this).P[1] * cos(angle);
	return reval;
}

/*!
*	\brief 벡터 p부터 q까지의 방향을 판단
*
*	\param p 첫번째 점
*	\param q 두번째 점
*
*	\return 시계방향이면 true, 반시계방향이면 false를 반환한다.
*/
bool counterclockwise(Point & p, Point & q)
{
	return ((p^q) >= 0.0);
}


/*!
*	\brief 두 점 사이의 거리
*
*	\param p 첫번째 점
*	\param q 두번째 점
*
*	\return \a p와 \a q의 거리의 제곱을 반환한다.
*/
double distance(Point &p, Point & q)
{
	return (p - q).length();
}

/*!
*	\brief 두 점의 내적을 구하여 반환한다.
*
*	\param lhs 첫번째 점
*	\param rhs 두번째 점
*
*	\return \a lhs와 \a rhs의 내적을 반환한다.
*/
double operator*(Point &lhs, Point &rhs)
{
	return lhs.P[0] * rhs.P[0] + lhs.P[1] * rhs.P[1];
}

/*!
*	\brief 두 점의 외적을 구하여 반환한다.
*
*	\param lhs 첫번째 점
*	\param rhs 두번째 점
*
*	\return \a lhs와 \a rhs의 외적을 반환한다.
*/
double operator^(Point & lhs, Point & rhs)
{
	return (lhs.P[0] * rhs.P[1] - lhs.P[1] * rhs.P[0]);
}

/*!
*	\brief 벡터의 실수배를 반환한다
*
*	\param s 실수배 해주는 값의 크기
*	\param rhs 실수배 되는 벡터 객체
*
*	\return \a rhs의 각 성분이 \a s배의 값을 갖는 벡터을 반환한다.
*/
Point operator*(double s, Point & rhs)
{
	return Point(s * rhs.P[0], s * rhs.P[1]);
}

/*!
*	\brief 벡터의 실수배
*
*	\param s 실수배 해주는 값의 크기
*	\param lhs 실수배 되는 벡터 객체
*
*	\return \a lhs의 각 성분이 \a s배의 값을 성분으로 갖는 벡터을 반환한다.
*/
Point operator*(Point & lhs, double s)
{
	return Point(lhs.P[0] * s, lhs.P[1] * s);
}

/*!
*	\brief 벡터를 실수로 나눈 값
*
*	\param s 실수배 해주는 값의 크기
*	\param lhs 실수배 되는 벡터 객체
*
*	\return \a lhs의 각 성분을 \a s로 나눈값을 성분으로 갖는 벡터을 반환한다.
*/
Point operator/(Point & lhs, double s)
{
	return lhs * (1 / s);
}

/*!
*	\brief 벡터의 합을 반환한다
*
*	\param lhs 첫번째 벡터
*	\param rhs 두번째 벡터
*
*	\return \a lhs와 \a rhs의 각 성분을 합한 벡터를 반환한다.
*/
Point operator+(const Point & lhs, const Point & rhs)
{
	return Point(lhs.P[0] + rhs.P[0], lhs.P[1] + rhs.P[1]);
}

/*!
*	\brief 두 벡터의 차를 반환한다
*
*	\param lhs 첫번째 벡터
*	\param rhs 두번째 벡터
*
*	\return \a lhs와 \a rhs의 각 성분의 차로 만든 벡터를 반환한다.
*/
Point operator-(const Point & lhs, const Point & rhs)
{
	return Point(lhs.P[0] - rhs.P[0], lhs.P[1] - rhs.P[1]);
}

/*!
*	\brief 점의 좌표를 출력한다
*
*	\param os cout
*	\param p 출력할 객체
*
*	\return p의 x좌표와 y좌표를 순서대로 반환한다
*/
std::ostream & operator<<(std::ostream &os, const Point &p)
{
	os << "(" << p.P[0] << ", " << p.P[1] << ")";
	return os;
}

/*!
*	\brief 점과 직선 사이의 거리
*
*	\param p 점
*	\param l 직선
*
*	\return p와 l 사이의 최단거리를 반환한다. 부호는 homogeneous 좌표에서의 직선의 normal과, 직선으로 부터 점까지의 방향이 같으면 양수, 다르면 음수로 나타난다.
*/
double distance(Line & l, Point & p)
{
	return (l.L[0] * p[0] + l.L[1] * p[1] + l.L[2]) / sqrt(l.L[0] * l.L[0] + l.L[1] * l.L[1]);
}

/*!
*	\brief 원와 원 사이의 교점
*
*	\param localUnitX 원과 원 사이를 잇는 직선을 새로운 기저의 방향이라 할 때, 그 기저의 단위벡터를 global coordinate에서 표현한 것
*	\param localUnitY 원과 원 사이를 잇는 직선에 수직인 직선을 새로운 기저의 방향이라 할 때, 그 기저의 단위벡터를 global coordinate에서 표현한 것
*	\param localX 원의 중심에서 교점을 표현했을 때, local coordinate의 X방향으로의 벡터
*	\param localX 원의 중심에서 교점을 표현했을 때, local coordinate의 Y방향으로의 벡터
*
*	\return 교점의 개수에 따라 서로 다른 tuple을 반환. 첫 번째와 두 번째의 Point 타입은 교점, 세 번째 int 타입은 교점의 개수를 의미한다.
*/
std::tuple<Point, Point, int> intersection_collision(Circle & lhs, Circle & rhs)
{
	std::tuple<Point, Point, int> result;

	// 원의 중심 사이의 거리
	double d = sqrt(distance(lhs.c, rhs.c));
	if (d > lhs.r + rhs.r - N_HIGH_PRESCISION)
		result = std::make_tuple(NULL, NULL, 0);
	else if (d < lhs.r + rhs.r + N_HIGH_PRESCISION) {
		if (d + std::min(lhs.r, rhs.r) < std::max(lhs.r, rhs.r) + N_HIGH_PRESCISION)
			result = std::make_tuple(NULL, NULL, 0);
		else
		{
			Point localUnitX = (rhs.c - lhs.c).normalize();
			Point localUnitY = localUnitX.rotate();
			Point localX = localUnitX*((d*d + lhs.r*lhs.r - rhs.r*rhs.r) / (2 * d));
			Point localY = localUnitY*(sqrt(lhs.r*lhs.r - localX.length()));
			result = std::make_tuple(lhs.c + localX + localY, lhs.c + localX - localY, 2);
		}
	}
	else
		result = std::make_tuple(lhs.projection(rhs.c), NULL, 1);

	return result;
}

/*!
*	\brief 원와 원 사이의 교점
*
*	\param localUnitX 원과 원 사이를 잇는 직선을 새로운 기저의 방향이라 할 때, 그 기저의 단위벡터를 global coordinate에서 표현한 것
*	\param localUnitY 원과 원 사이를 잇는 직선에 수직인 직선을 새로운 기저의 방향이라 할 때, 그 기저의 단위벡터를 global coordinate에서 표현한 것
*	\param localX 원의 중심에서 교점을 표현했을 때, local coordinate의 X방향으로의 벡터
*	\param localX 원의 중심에서 교점을 표현했을 때, local coordinate의 Y방향으로의 벡터
*
*	\return 교점의 개수에 따라 서로 다른 tuple을 반환. 첫 번째와 두 번째의 Point 타입은 교점, 세 번째 int 타입은 교점의 개수를 의미한다. 조금 더 보수적으로 확인한다.

Def : Get circle(not arc) intersection by : cosine law on triangle(center-center line , radius1, radius2)

localX : has direction connecting two circle centers
localY : perpendicular to localX
*/
std::tuple<Point, Point, int> intersection_self(Circle & lhs, Circle & rhs)
{
	std::tuple<Point, Point, int> result;

	// 원의 중심 사이의 거리
	double d = distance(lhs.c, rhs.c);
	if (d > (lhs.r + rhs.r + N_HIGH_PRESCISION) * (lhs.r + rhs.r + N_HIGH_PRESCISION))
		result = std::make_tuple(NULL, NULL, 0);
	else if (d < (lhs.r + rhs.r - N_HIGH_PRESCISION) * (lhs.r + rhs.r - N_HIGH_PRESCISION)) {
		if (std::sqrt(d) + std::min(lhs.r, rhs.r) < std::max(lhs.r, rhs.r) + N_HIGH_PRESCISION)
			result = std::make_tuple(NULL, NULL, 0);
		else
		{
			Point localUnitX = (rhs.c - lhs.c).normalize();
			Point localUnitY = localUnitX.rotate();
			Point localX = localUnitX * ((d + lhs.r*lhs.r - rhs.r*rhs.r) / (2 * std::sqrt(d)));
			Point localY = localUnitY * (sqrt(lhs.r*lhs.r - localX.length()));
			result = std::make_tuple(lhs.c + localX + localY, lhs.c + localX - localY, 2);
		}
	}
	else
		result = std::make_tuple(lhs.projection(rhs.c), NULL, 1);

	return result;
}

/*!
*	\brief Interior Disks를 Import
*
*	\param filename 파일의 이름
*
*	\return Import한 Disk를 Circle 클래스의 vector 형태로 반환한다.
*/
std::vector<Circle> importCircles(std::string filename)
{
	std::vector<Circle> import;
	FILE* f1;
	if (fopen_s(&f1, filename.c_str(), "r") != 0)
		return import;

	fseek(f1, 0, SEEK_END);
	int size = ftell(f1);
	rewind(f1);

	char* dump = new char[size];
	fread_s(dump, size, size, 1, f1);
	fclose(f1);

	int curr = 0;
	int num = atoi(dump);

	for (int i = 0; i < num; i++) {
		Point a;
		double r;
		while (dump[curr] != ' ' && dump[curr] != '\n' && dump[curr] != '\t')
			curr++;
		while (dump[curr] == ' ' || dump[curr] == '\n' || dump[curr] == '\t')
			curr++;
		a[0] = atof(&dump[curr]);
		while (dump[curr] != ' ' && dump[curr] != '\n'&& dump[curr] != '\t')
			curr++;
		while (dump[curr] == ' ' || dump[curr] == '\n' || dump[curr] == '\t')
			curr++;
		a[1] = atof(&dump[curr]);
		while (dump[curr] != ' ' && dump[curr] != '\n'&& dump[curr] != '\t')
			curr++;
		while (dump[curr] == ' ' || dump[curr] == '\n' || dump[curr] == '\t')
			curr++;
		r = atof(&dump[curr]);

		import.push_back(Circle(a, r));
	}
	return import;
}

/*!
*	\brief 두 Circle 클래스의 배열을 minkowski sum한 결과를 반환
*
*	\param lhs 첫번째 원들
*	\param rhs 두번째 원들
*
*	\return lhs와 rhs의 원을 minkowski sum한 결과를 반환한다.
*/
std::vector<Circle> operator+(std::vector<Circle>& lhs, std::vector<Circle>& rhs)
{
	std::vector<Circle> reval;

	for (int i = 0; i < (int)lhs.size(); i++)
		for (int j = 0; j < (int)rhs.size(); j++)
			reval.push_back(Circle(lhs[i].c + rhs[j].c, lhs[i].r + rhs[j].r));

	return reval;
}

/*!
*	\brief 원의 정보 출력
*
*	\param os cout
*	\param p 출력할 원
*
*	\return 원의 중심과 반지름을 순서대로 반환한다
*/
std::ostream & operator<<(std::ostream & os, const Circle & p)
{
	os << "Center: " << p.c << "\n" << "Radius: " << p.r;
	return os;
}

/*!
*	\brief 두 Arc의 교점
*
*	\param s Arc를 포함하는 두 원의 교점
*	\param check 원의 교점이 각 Arc의 범위에 포함되는지를 확인
*
*	\return Arc의 교점의 개수에 따라 서로 다른 tuple을 반환, 세 번째 int는 교점의 개수
*/
std::tuple<Point, Point, int> intersection_CircularArc(CircularArc & lhs, CircularArc & rhs)
{
	//auto s = intersect(lhs.c, rhs.c);
	std::tuple<Point, Point, int> s = intersection_self(lhs.c, rhs.c);


	switch (std::get<2>(s)) {
	case 0:
		return s;
	case 1:
		if (lhs.contain(std::get<0>(s)) && rhs.contain(std::get<0>(s)))
			return s;
		else
			return s = { NULL, NULL, 0 };
	case 2:
		bool check[2];
		check[0] = (lhs.contain(std::get<0>(s)) && rhs.contain(std::get<0>(s)));
		check[1] = (lhs.contain(std::get<1>(s)) && rhs.contain(std::get<1>(s)));
		if (!check[0] && !check[1])
			return s = { NULL, NULL, 0 };
		if (check[0] && !check[1]) {
			std::get<1>(s) = NULL; std::get<2>(s) = 1;
			return s;
		}
		if (!check[0] && check[1]) {
			std::get<0>(s) = std::get<1>(s); std::get<1>(s) = NULL, std::get<2>(s) = 1;
			return s;
		}
		return s;
	default:
		return s;
	}
}

/*!
*	\brief 두 Arc의 충돌 여부
*
*	\param s Arc를 포함하는 두 원의 교점
*	\param check 원의 교점이 각 Arc의 범위에 포함되는지를 확인
*
*	\return Arc의 충돌 여부를 판단하여 교점이 존재하면 true를 반환한다.
*/
bool intersection_bool(CircularArc & lhs, CircularArc & rhs, Point &p)
{
	CircularArc reverse(rhs, p);
	auto s = intersection_collision(lhs.c, reverse.c);
	switch (std::get<2>(s)) {
	case 0:
		return false;
	case 1:
		if (lhs.contain(std::get<0>(s)) && reverse.contain(std::get<0>(s)))
			return true;
		else
			return false;
	case 2:
		bool check[2];
		check[0] = (lhs.contain(std::get<0>(s)) && reverse.contain(std::get<0>(s)));
		check[1] = (lhs.contain(std::get<1>(s)) && reverse.contain(std::get<1>(s)));
		if (check[0] || check[1])
			return true;
		else
			return false;
	default:
		return false;
	}
}



/*!
*	\brief Circular Arc의 정보를 출력
*
*	\param os cout
*	\param p 출력하는 Circular Arc
*
*	\return Circular Arc의 시작점과 끝점의 좌표를 출력한다.
*/
std::ostream & operator<<(std::ostream & os, const CircularArc & p)
{
	os << p.c << "\n" << "init Point: " << p.x[0] << "\n" << "end Point: " << p.x[1];
	return os;
}

/*!
*	\brief 두 Bezier Function의 합
*
*	\param lhs 첫번쨰 함수
*	\param rhs 두번째 함수
*
*	\return 두 Bezier Function의 각 계수를 합한 Bezier Function을 반환한다.
*/
BezierCrv operator+(const BezierCrv & lhs, const BezierCrv & rhs)
{
	if (lhs.PType != rhs.PType) {
		std::cout << "error: Dimension of Bezier function is different!" << std::endl;//for debuging
		return lhs;
	}
	if (lhs.Deg != rhs.Deg) {
		std::cout << "error: Degree of BEzier function is different!" << std::endl;//for debuging
		return lhs;
	}
	auto sum = lhs;
	switch (lhs.PType) {
	case 0:
		for (int i = 0; i <= lhs.Deg; i++)
			sum.P[i].P[0] += rhs.P[i].P[0];
		break;
	case 1:
		for (int i = 0; i <= lhs.Deg; i++)
			sum.P[i] = sum.P[i] + rhs.P[i];
		break;
	default:
		std::cout << "error: PType of Crv has problem" << std::endl;
		break;
	}
	return sum;
}

/*!
*	\brief 두 Bezier Function의 차
*
*	\param lhs 첫번쨰 함수
*	\param rhs 두번째 함수
*
*	\return 두 Bezier Function의 각 계수의 차를 갖는 Bezier Function을 반환한다.
*/
BezierCrv operator-(const BezierCrv & lhs, const BezierCrv & rhs)
{
	if (lhs.PType != rhs.PType) {
		std::cout << "error: Dimension of Bezier function is different!" << std::endl;//for debuging
		return lhs;
	}
	if (lhs.Deg != rhs.Deg) {
		std::cout << "error: Degree of BEzier function is different!" << std::endl;//for debuging
		return lhs;
	}
	auto sum = lhs;
	switch (lhs.PType) {
	case 0:
		for (int i = 0; i <= lhs.Deg; i++)
			sum.P[i].P[0] -= rhs.P[i].P[0];
		break;
	case 1:
		for (int i = 0; i <= lhs.Deg; i++)
			sum.P[i] = sum.P[i] - rhs.P[i];
		break;
	default:
		std::cout << "error: PType of Crv has problem" << std::endl;
		break;
	}
	return sum;
}


/*!
*	\brief 두 Bezier Function의 스칼라곱
*
*	\param lhs 첫번쨰 함수
*	\param rhs 두번째 함수
*
*	\return 두 Bezier Function을 내적한 Bezier Function을 반환한다.
*/
BezierCrv operator*(const BezierCrv & lhs, const BezierCrv & rhs)
{
	if (lhs.PType != rhs.PType) {
		std::cout << "error: Dimension of Bezier function is different!" << std::endl; return lhs;
	}
	BezierCrv reval(lhs.Deg + rhs.Deg, CTRL_PT_E1);
	switch (reval.PType) {
	case 0:
		for (int i = 0; i <= lhs.Deg; i++)
			for (int j = 0; j <= rhs.Deg; j++)
				reval.P[i + j].P[0] += lhs.P[i].P[0] * coefBasis[lhs.Deg][i] * rhs.P[j].P[0] * coefBasis[rhs.Deg][j];
		for (int i = 0; i <= reval.Deg; i++)
			reval.P[i].P[0] /= coefBasis[reval.Deg][i];
		break;
	case 1:
		for (int i = 0; i <= lhs.Deg; i++)
			for (int j = 0; j <= rhs.Deg; j++) {
				reval.P[i + j].P[0] += lhs.P[i].P[0] * coefBasis[lhs.Deg][i] * rhs.P[j].P[0] * coefBasis[rhs.Deg][j];
				reval.P[i + j].P[0] += lhs.P[i].P[1] * coefBasis[lhs.Deg][i] * rhs.P[j].P[1] * coefBasis[rhs.Deg][j];
			}
		for (int i = 0; i <= reval.Deg; i++) {
			reval.P[i].P[0] /= coefBasis[reval.Deg][i];
		}
		break;
	default:
		std::cout << "error: PType of Crv has problem" << std::endl;
		break;

	}
	return reval;
}

/*!
*	\brief 두 Bezier Function의 벡터곱
*
*	\param lhs 첫번째 함수
*	\param rhs 두번째 함수
*
*	\return 두 Bezier Function을 외적한 Bezier Function을 반환한다.
*/
BezierCrv operator^(const BezierCrv & lhs, const BezierCrv & rhs)
{
	if ((lhs.PType == 0) || (rhs.PType == 0)) {
		std::cout << "error: 1 Dimensional Bezier Function cannot define cross product!" << std::endl;
		return lhs;
	}
	BezierCrv reval = BezierCrv(lhs.Deg + rhs.Deg, CTRL_PT_E1);
	for (int i = 0; i <= lhs.Deg; i++)
		for (int j = 0; j <= rhs.Deg; j++) {
			reval.P[i + j].P[0] += lhs.P[i].P[0] * coefBasis[lhs.Deg][i] * rhs.P[j].P[1] * coefBasis[rhs.Deg][j];
			reval.P[i + j].P[0] -= lhs.P[i].P[1] * coefBasis[lhs.Deg][i] * rhs.P[j].P[0] * coefBasis[rhs.Deg][j];
		}
	for (int i = 0; i <= reval.Deg; i++) {
		reval.P[i].P[0] /= coefBasis[reval.Deg][i];
	}
	return reval;
}



/*!
*	\brief Bezier Function의 미분
*
*	\param Crv Bezier Function
*
*	\return Bezier Function를 미분한 함수를 Bezier Function의 Form으로 반환한다.
*/
BezierCrv diff(const BezierCrv & Crv)
{
	if (Crv.Deg == 0) {
		std::cout << "error: cannot differentiate Bezier Function" << std::endl;
		return BezierCrv();
	}
	BezierCrv reval(Crv.Deg - 1, Crv.PType);
	switch (reval.PType) {
	case 0:
		for (int i = 0; i <= reval.Deg; i++)
			reval.P[i].P[0] = Crv.Deg * (Crv.P[i + 1].P[0] - Crv.P[i].P[0]);
		break;
	case 1:
		for (int i = 0; i <= reval.Deg; i++) {
			reval.P[i].P[0] = Crv.Deg * (Crv.P[i + 1].P[0] - Crv.P[i].P[0]);
			reval.P[i].P[1] = Crv.Deg * (Crv.P[i + 1].P[1] - Crv.P[i].P[1]);
		}
		break;
	default:
		std::cout << "error: PType of Crv has problem" << std::endl;
		break;
	}
	return reval;
}

/*!
*	\brief Bezier Function의 정보를 출력
*
*	\param os cout
*	\param c 정보를 출력하는 함수
*
*	\return Bezier Function과 각 Control Point를 출력한다.
*/
std::ostream & operator<<(std::ostream & os, const BezierCrv & c)
{
	os << "Bezier Curve" << std::endl << "Dimension: " << (c.PType + 1) << std::endl << "x: { ";
	switch (c.PType) {
	case 0:
		for (int i = 0; i < c.Deg; i++)
			std::cout << (c.P[i].P[0] * coefBasis[c.Deg][i]) << "(1 - t)^" << (c.Deg - i) << " * t^" << i << "\t + ";
		std::cout << (c.P[c.Deg].P[0] * coefBasis[c.Deg][c.Deg]) << "(1 - t)^" << 0 << " * t^" << c.Deg << " }" << std::endl;

		break;

	case 1:
		for (int i = 0; i < c.Deg; i++)
			std::cout << (c.P[i].P[0] * coefBasis[c.Deg][i]) << "(1 - t)^" << (c.Deg - i) << " * t^" << i << "\t + ";
		std::cout << (c.P[c.Deg].P[0] * coefBasis[c.Deg][c.Deg]) << "(1 - t)^" << 0 << " * t^" << c.Deg << " }" << std::endl << "y: { ";

		for (int i = 0; i < c.Deg; i++)
			std::cout << (c.P[i].P[1] * coefBasis[c.Deg][i]) << "(1 - t)^" << (c.Deg - i) << " * t^" << i << "\t + ";
		std::cout << (c.P[c.Deg].P[1] * coefBasis[c.Deg][c.Deg]) << "(1 - t)^" << 0 << " * t^" << c.Deg << " }" << std::endl;

		std::cout << "Control Points: ";
		for (int i = 0; i <= c.Deg; i++)
			std::cout << c.P[i] << " ";
		std::cout << std::endl;

		break;
	}
	return os;
}

/*!
*	\brief Bezier Curve를 Import
*
*	\param filename 파일의 이름
*
*	\return Import한 Bezier Curve들을 vector의 자료형으로 출력한다.
*/
std::vector<BezierCrv> import_Crv(std::string filename)
{
	std::vector<BezierCrv> input;
	FILE* f1;
	if (fopen_s(&f1, filename.c_str(), "r") != 0)
		return input;

	fseek(f1, 0, SEEK_END);
	int size = ftell(f1);
	rewind(f1);

	char* dump = new char[size];
	fread_s(dump, size, size, 1, f1);
	fclose(f1);

	int curr = 0;
	int num = atoi(dump);

	for (int i = 0; i < num; i++) {
		Point a[4];
		for (int j = 0; j < 4; j++) {
			while (dump[curr] != ' ' && dump[curr] != '\n' && dump[curr] != '\t')
				curr++;
			while (dump[curr] == ' ' || dump[curr] == '\n' || dump[curr] == '\t')
				curr++;
			a[j][0] = atof(&dump[curr]);
			while (dump[curr] != ' ' && dump[curr] != '\n'&& dump[curr] != '\t')
				curr++;
			while (dump[curr] == ' ' || dump[curr] == '\n' || dump[curr] == '\t')
				curr++;
			a[j][1] = atof(&dump[curr]);
		}
		if (a[1].exact(a[2])) {
			auto su = BezierCrv(a).subDiv();
			input.push_back(su.first);
			input.push_back(su.second);
		}
		else {
			input.push_back(BezierCrv(a));
		}
	}
	return input;
}

/*!
*	\brief 두 모델의 충돌여부 판단
*
*	\param lhs 첫번째 모델
*	\param rhs 두번째 모델
*	\param p 대칭점
*
*	\return 모델 lhs와, 모델 rhs를 p에 대해 점대칭이동한 점의 Collision을 판단한다.
*/
bool collision(std::vector<BezierCrv>& lhs, std::vector<BezierCrv> &rhs, Point &p)
{
	std::vector<std::pair<BezierCrv*, BezierCrv*>> q;
	for(int i = 0; i < (int)lhs.size(); i++)
		for (int j = 0; j < (int)rhs.size(); j++) {
			if (lhs[i].aabbtest(rhs[j],p))//lhs[i].aabbtest(BezierCrv(rhs[j],p)))
				q.push_back(std::make_pair(&(lhs[i]), &(rhs[j])));
		}
	while (!q.empty()) {
		auto x = q.back();
		q.pop_back();
		if ((x.first->child[0] == NULL) && (x.second->child[0] == NULL)) {
			if (intersection_bool(x.first->Arcs.first, x.second->Arcs.first, p) || intersection_bool(x.first->Arcs.second, x.second->Arcs.first, p) || intersection_bool(x.first->Arcs.first, x.second->Arcs.second, p) || intersection_bool(x.first->Arcs.second, x.second->Arcs.second, p)) {
				return true;
			}
		}
		else if (x.first->child[0] == NULL) {
			if (x.first->aabbtest(*x.second->child[0], p))
				q.push_back(std::make_pair(x.first, x.second->child[0]));
			if (x.first->aabbtest(*x.second->child[1], p))
				q.push_back(std::make_pair(x.first, x.second->child[1]));
		}
		else if (x.second->child[0] == NULL) {
			if (x.first->child[0]->aabbtest(*x.second, p))
				q.push_back(std::make_pair(x.first->child[0], x.second));
			if (x.first->child[1]->aabbtest(*x.second, p))
				q.push_back(std::make_pair(x.first->child[1], x.second));
		}
		else {
			if (x.first->child[0]->aabbtest(*x.second->child[0], p))
				q.push_back(std::make_pair(x.first->child[0], x.second->child[0]));
			if (x.first->child[1]->aabbtest(*x.second->child[0], p))
				q.push_back(std::make_pair(x.first->child[1], x.second->child[0]));
			if (x.first->child[0]->aabbtest(*x.second->child[1], p))
				q.push_back(std::make_pair(x.first->child[0], x.second->child[1]));
			if (x.first->child[1]->aabbtest(*x.second->child[1], p))
				q.push_back(std::make_pair(x.first->child[1], x.second->child[1]));
		}

	}
	return false;
}

/*
Def : collision test among circularArcs inside (lhs) and (rhs flipped(symmetric-transformed) on p)
Currently a bit brute force... maybe exploit aabb test? => needs to add symmetric transform somewhere
*/
bool collision(std::vector<ArcSpline>& lhs, std::vector<ArcSpline>& rhs, Point& p)
{
	std::vector<std::pair<BezierCrv*, BezierCrv*>> q;
	for (int i = 0; i < (int)lhs.size(); i++)
	for (int j = 0; j < (int)rhs.size(); j++) 
	{
		auto& las = lhs[i];
		auto& ras = rhs[j];
		if (aabbtest(las,ras,p)) // TODO: AABB test of ArcSpline later?
		{
			for (size_t ii = 0; ii < las.Arcs.size(); ii++)
			for (size_t jj = 0; jj < ras.Arcs.size(); jj++)
			{
				if (intersection_bool(las.Arcs[ii], ras.Arcs[jj], p))
					return true;
			}
		}
	}
	return false;
}


/*!
*	\brief 대소비교
*
*	\param lhs 첫 번째 segment
*	\param rhs 두 번째 segment
*
*	\return 두 segment를 비교한다.
*/
bool operator<(const Segment & lhs, const Segment & rhs)
{
	return (lhs.value < rhs.value);
}

/*!
*	\brief Arc Spline의 인접관계 생성
*
*	\param lhs 첫 번째 Arc Spline Segment
*	\param rhs 두 번째 Arc Spline Segment
*
*	\return 두 Arc Spline Segment이 연결되어있으면 true를 반환한다. 두 Arc Spline이 연결된 경우, 각각의 인접정보를 업데이트 시켜준다.

Def: If the endpoints of two arcsplines are very close (&& closer to their past-best-neighbors), update info and return true
Ret: true, if proximity was updated(pair is best, by far)

1. Div to 4 case (lhs.0 or lhs.1) x (rhs.0 or rhs.1)
2. For each case, check if dist is below some bound
3. if under some bound, check if the pair is best(closest) for both of the arcSplines. (no-update when one of the arcsplines has a better neighbor)
4. if best for both => erase past-neighbors proximity info, and update lhs & rhs.
*/
bool connected(ArcSpline & lhs, ArcSpline & rhs)
{
	bool reval = false;
	// 두 ArcSpline의 시작점과 끝점 각각에 대하여 판단하여야 하므로,
	// 총 네 가지 경우에 대하여 판단한다. 
	// 만약 두 점이 Numerical Error 부근에서 비슷한 값을 가지면, 두 점이 연결될 수 있는 후보가 된다.
	if (lhs.init() == rhs.init()) {
		// 기존의 연결관계가 있는 경우, 그 연결관계의 거리와 새로운 거리를 비교한다
		// 기존의 거리가 더 가까운 경우, 업데이트 하지 않는다
		if (lhs._neighbor[0] && (distance(lhs.init(), rhs.init()) > lhs.neighborDist[0])) {
			if (!reval)
				reval = false;
		}
		else if (rhs._neighbor[0] && (distance(lhs.init(), rhs.init()) > rhs.neighborDist[0])) {
			if (!reval)
				reval = false;
		}
		// 새로운 거리가 기존의 거리보다 가까운 경우 이를 업데이트 시켜준다
		else {
			//refresh
			// 기존의 연결관계의 정보를 초기화시켜준다.
			if (lhs._neighbor[0]) {
				lhs.neighbor[0]->neighbor[lhs.relativePosition[0]] = NULL;
				lhs.neighbor[0]->_neighbor[lhs.relativePosition[0]] = false;
				lhs.neighbor[0]->neighborDist[lhs.relativePosition[0]] = DBL_MAX;
			}
			if (rhs._neighbor[0]) {
				rhs.neighbor[0]->neighbor[rhs.relativePosition[0]] = NULL;
				rhs.neighbor[0]->_neighbor[rhs.relativePosition[0]] = false;
				rhs.neighbor[0]->neighborDist[rhs.relativePosition[0]] = DBL_MAX;
			}
			// 새로운 연결관계의 값을 넣어준다.
			lhs.neighbor[0] = &rhs;
			lhs._neighbor[0] = true;
			lhs.relativePosition[0] = false;
			rhs.neighbor[0] = &lhs;
			rhs._neighbor[0] = true;
			rhs.relativePosition[0] = false;
			rhs.neighborDist[0] = lhs.neighborDist[0] = distance(lhs.init(), rhs.init());
			reval = true;
		}
	}
	if (lhs.end() == rhs.init()) {
		if (lhs._neighbor[1] && (distance(lhs.end(), rhs.init()) > lhs.neighborDist[1])) {
			if (!reval)
				reval = false;
		}
		else if (rhs._neighbor[0] && (distance(lhs.end(), rhs.init()) > rhs.neighborDist[0])) {
			if (!reval)
				reval = false;
		}
		else {
			if (lhs._neighbor[1]) {
				lhs.neighbor[1]->neighbor[lhs.relativePosition[1]] = NULL;
				lhs.neighbor[1]->_neighbor[lhs.relativePosition[1]] = false;
				lhs.neighbor[1]->neighborDist[lhs.relativePosition[1]] = DBL_MAX;
			}
			if (rhs._neighbor[0]) {
				rhs.neighbor[0]->neighbor[rhs.relativePosition[0]] = NULL;
				rhs.neighbor[0]->_neighbor[rhs.relativePosition[0]] = false;
				rhs.neighbor[0]->neighborDist[rhs.relativePosition[0]] = DBL_MAX;
			}
			lhs.neighbor[1] = &rhs;
			lhs._neighbor[1] = true;
			lhs.relativePosition[1] = false;
			rhs.neighbor[0] = &lhs;
			rhs._neighbor[0] = true;
			rhs.relativePosition[0] = true;
			rhs.neighborDist[0] = lhs.neighborDist[1] = distance(lhs.end(), rhs.init());
			reval = true;
		}
	}
	if (lhs.init() == rhs.end()) {
		if (lhs._neighbor[0] && (distance(lhs.init(), rhs.end()) > lhs.neighborDist[0])) {
			if (!reval)
				reval = false;
		}
		else if (rhs._neighbor[1] && (distance(lhs.init(), rhs.end()) > rhs.neighborDist[1])) {
			if (!reval)
				reval = false;
		}
		else {
			if (lhs._neighbor[0]) {
				lhs.neighbor[0]->neighbor[lhs.relativePosition[0]] = NULL;
				lhs.neighbor[0]->_neighbor[lhs.relativePosition[0]] = false;
				lhs.neighbor[0]->neighborDist[lhs.relativePosition[0]] = DBL_MAX;
			}
			if (rhs._neighbor[1]) {
				rhs.neighbor[1]->neighbor[rhs.relativePosition[1]] = NULL;
				rhs.neighbor[1]->_neighbor[rhs.relativePosition[1]] = false;
				rhs.neighbor[1]->neighborDist[rhs.relativePosition[1]] = DBL_MAX;
			}
			lhs.neighbor[0] = &rhs;
			lhs._neighbor[0] = true;
			lhs.relativePosition[0] = true;
			rhs.neighbor[1] = &lhs;
			rhs._neighbor[1] = true;
			rhs.relativePosition[1] = false;
			rhs.neighborDist[1] = lhs.neighborDist[0] = distance(lhs.init(), rhs.end());
			reval = true;
		}
	}
	if (lhs.end() == rhs.end()) {
		if (lhs._neighbor[1] && (distance(lhs.end(), rhs.end()) > lhs.neighborDist[1])) {
			if (!reval)
				reval = false;
		}
		else if (rhs._neighbor[1] && (distance(lhs.end(), rhs.end()) > rhs.neighborDist[1])) {
			if (!reval)
				reval = false;
		}
		else {
			if (lhs._neighbor[1]) {
				lhs.neighbor[1]->neighbor[lhs.relativePosition[1]] = NULL;
				lhs.neighbor[1]->_neighbor[lhs.relativePosition[1]] = false;
				lhs.neighbor[1]->neighborDist[lhs.relativePosition[1]] = DBL_MAX;
			}
			if (rhs._neighbor[1]) {
				rhs.neighbor[1]->neighbor[rhs.relativePosition[1]] = NULL;
				rhs.neighbor[1]->_neighbor[rhs.relativePosition[1]] = false;
				rhs.neighbor[1]->neighborDist[rhs.relativePosition[1]] = DBL_MAX;
			}
			lhs.neighbor[1] = &rhs;
			lhs._neighbor[1] = true;
			lhs.relativePosition[1] = true;
			rhs.neighbor[1] = &lhs;
			rhs._neighbor[1] = true;
			rhs.relativePosition[1] = true;
			rhs.neighborDist[1] = lhs.neighborDist[1] = distance(lhs.end(), rhs.end());
			reval = true;
		}
	}
	return reval;
}

/*!
*	\brief Arc Spline의 교점 찾기
*
*	\param lhs 첫 번째 Arc Spline Segment
*	\param rhs 두 번째 Arc Spline Segment
*
*	\return 두 Arc Spline Segment이 만나는지를 확인한 뒤, 만나면 각각의 교점 정보를 업데이트한다.

1. (BVH for lhs) vs (BVH for rhs)
2. if (leaf_VS_leaf case reached) compute if arcs intersect w/ intersection_CircularArc
*/
void selfIntersectionPts(ArcSpline &lhs, ArcSpline &rhs)
{
	/* 모든 Arc는 반시계방향!!! */
	// Arc Spline의 Circular Arc들의 Index 범위를 점점 반으로 쪼개가며 AABB Bounding Volume을 이용하여 충돌을 확인한다.
	// ex. A.Arcs[3-8], B.Arcs[8-13]이 서로 충돌 -> A.Arcs[3-5] / A.Arcs[6-8], B.Arcs[8-10] / B.Arcs[11-13] 각각 총 4 가지 경우에 대하여 충돌을 확인한다
	std::queue<std::pair<int, int>> ql;
	std::queue<std::pair<int, int>> qr;
	ql.push(std::make_pair(0, (int)lhs.Arcs.size() - 1));
	qr.push(std::make_pair(0, (int)rhs.Arcs.size() - 1));
	while (!ql.empty()) {

		// 1. recursively go down BVH

		auto lIdx = ql.front(), rIdx = qr.front();
		ql.pop();
		qr.pop();
		bool t1 = (lIdx.first == lIdx.second), t2 = (rIdx.first == rIdx.second); // t1: reached leaf node for left-BVH, t2: t1, but for right-BVH
		if (!t1 && !t2) {
			if (aabbtest(lhs,rhs,lIdx,rIdx)){
				// 순서가 중요!
				// 왜그랬는지 기억이 안나요 ㅋㅋ
				ql.push(std::make_pair(lIdx.first, (lIdx.first + lIdx.second - 1) / 2));
				ql.push(std::make_pair((lIdx.first + lIdx.second + 1) / 2, lIdx.second));
				ql.push(std::make_pair(lIdx.first, (lIdx.first + lIdx.second - 1) / 2));
				ql.push(std::make_pair((lIdx.first + lIdx.second + 1) / 2, lIdx.second));
				qr.push(std::make_pair(rIdx.first, (rIdx.first + rIdx.second - 1) / 2));
				qr.push(std::make_pair(rIdx.first, (rIdx.first + rIdx.second - 1) / 2));
				qr.push(std::make_pair((rIdx.first + rIdx.second + 1) / 2, rIdx.second));
				qr.push(std::make_pair((rIdx.first + rIdx.second + 1) / 2, rIdx.second));
			}
		}
		else if (t1 && !t2) {
			if (aabbtest(lhs.Arcs[lIdx.first],rhs,rIdx)){
				ql.push(std::make_pair(lIdx.first, lIdx.second));
				ql.push(std::make_pair(lIdx.first, lIdx.second));
				qr.push(std::make_pair(rIdx.first, (rIdx.first + rIdx.second - 1) / 2));
				qr.push(std::make_pair((rIdx.first + rIdx.second + 1) / 2, rIdx.second));
			}
		}

		else if (!t1 && t2) {
			if (aabbtest(rhs.Arcs[rIdx.first], lhs, lIdx)){
				ql.push(std::make_pair(lIdx.first, (lIdx.first + lIdx.second - 1) / 2));
				ql.push(std::make_pair((lIdx.first + lIdx.second + 1) / 2, lIdx.second));
				qr.push(std::make_pair(rIdx.first, rIdx.second));
				qr.push(std::make_pair(rIdx.first, rIdx.second));
			}
		}
		// Leap Node (Single Circular Arc)까지 도달 -> 그 Arc의 교점의 개수를 가지고 경우를 나누자
		else {

			// 2. reached leaf x leaf case.

			// 2-1. find intersection
			auto reval = intersection_CircularArc(lhs.Arcs[lIdx.first], rhs.Arcs[rIdx.first]);

			// 2-2. switch (number_of_intersections)
			switch (std::get<2>(reval)) {
			// 하나의 교점인 경우, Convolution 과정에서 저장해놓은 boundary를 이용하여
			// 지워지는 방향을 dividedPts의 마지막 인자로 표시한다.
			// 교점을 지난 후의 부분이 지워지면 true, 교점 이전이 부분이 지워지면 false를 넣어준다.
			case 1:
				if (counterclockwise(std::get<0>(reval) - lhs.Arcs[lIdx.first].c.c, std::get<0>(reval) - rhs.Arcs[rIdx.first].c.c)) {
					if (lhs.Arcs[lIdx.first].boundary && rhs.Arcs[rIdx.first].boundary) {
						lhs.intersections.push_back(dividePts(lIdx.first, std::get<0>(reval), false));
						rhs.intersections.push_back(dividePts(rIdx.first, std::get<0>(reval), true));
					}
					else if (lhs.Arcs[lIdx.first].boundary && !rhs.Arcs[rIdx.first].boundary) {
						lhs.intersections.push_back(dividePts(lIdx.first, std::get<0>(reval), true));
						rhs.intersections.push_back(dividePts(rIdx.first, std::get<0>(reval), true));
					}
					else if (!lhs.Arcs[lIdx.first].boundary && rhs.Arcs[rIdx.first].boundary) {
						lhs.intersections.push_back(dividePts(lIdx.first, std::get<0>(reval), false));
						rhs.intersections.push_back(dividePts(rIdx.first, std::get<0>(reval), false));
					}
					else {
						lhs.intersections.push_back(dividePts(lIdx.first, std::get<0>(reval), true));
						rhs.intersections.push_back(dividePts(rIdx.first, std::get<0>(reval), false));
					}
				}
				else {
					if (lhs.Arcs[lIdx.first].boundary && rhs.Arcs[rIdx.first].boundary) {
						lhs.intersections.push_back(dividePts(lIdx.first, std::get<0>(reval), true));
						rhs.intersections.push_back(dividePts(rIdx.first, std::get<0>(reval), false));
					}
					else if (lhs.Arcs[lIdx.first].boundary && !rhs.Arcs[rIdx.first].boundary) {
						lhs.intersections.push_back(dividePts(lIdx.first, std::get<0>(reval), false));
						rhs.intersections.push_back(dividePts(rIdx.first, std::get<0>(reval), false));
					}
					else if (!lhs.Arcs[lIdx.first].boundary && rhs.Arcs[rIdx.first].boundary) {
						lhs.intersections.push_back(dividePts(lIdx.first, std::get<0>(reval), true));
						rhs.intersections.push_back(dividePts(rIdx.first, std::get<0>(reval), true));
					}
					else {
						lhs.intersections.push_back(dividePts(lIdx.first, std::get<0>(reval), false));
						rhs.intersections.push_back(dividePts(rIdx.first, std::get<0>(reval), true));
					}
				}
				(lhs.intersections.end() - 1)->initPt = lhs.Arcs[lIdx.first].x[0];
				(rhs.intersections.end() - 1)->initPt = rhs.Arcs[rIdx.first].x[0];
				break;
			// 교점이 두 개인 경우 총 16가지 경우가 나오나, 이를 모두 정리해보면 아래와 같은 네 가지로 줄어든다
			case 2: {
				bool lhsRIBTrhs = (lhs.Arcs[lIdx.first].c.r > rhs.Arcs[rIdx.first].c.r);
				bool sameDirection = ((std::get<0>(reval) - lhs.Arcs[lIdx.first].c.c) * (std::get<0>(reval) - rhs.Arcs[rIdx.first].c.c) > 0.0);
				if (lhs.Arcs[lIdx.first].boundary && rhs.Arcs[rIdx.first].boundary) {
					lhs.intersections.push_back(dividePts(lIdx.first, std::get<0>(reval), false));
					lhs.intersections.push_back(dividePts(lIdx.first, std::get<1>(reval), true));
					rhs.intersections.push_back(dividePts(rIdx.first, std::get<0>(reval), true));
					rhs.intersections.push_back(dividePts(rIdx.first, std::get<1>(reval), false));
				}
				else if (lhs.Arcs[lIdx.first].boundary && !rhs.Arcs[rIdx.first].boundary) {
					lhs.intersections.push_back(dividePts(lIdx.first, std::get<0>(reval), true));
					lhs.intersections.push_back(dividePts(lIdx.first, std::get<1>(reval), false));
					rhs.intersections.push_back(dividePts(rIdx.first, std::get<0>(reval), true));
					rhs.intersections.push_back(dividePts(rIdx.first, std::get<1>(reval), false));
				}
				else if (!lhs.Arcs[lIdx.first].boundary && rhs.Arcs[rIdx.first].boundary) {
					lhs.intersections.push_back(dividePts(lIdx.first, std::get<0>(reval), false));
					lhs.intersections.push_back(dividePts(lIdx.first, std::get<1>(reval), true));
					rhs.intersections.push_back(dividePts(rIdx.first, std::get<0>(reval), false));
					rhs.intersections.push_back(dividePts(rIdx.first, std::get<1>(reval), true));
				}
				else {
					lhs.intersections.push_back(dividePts(lIdx.first, std::get<0>(reval), true));
					lhs.intersections.push_back(dividePts(lIdx.first, std::get<1>(reval), false));
					rhs.intersections.push_back(dividePts(rIdx.first, std::get<0>(reval), false));
					rhs.intersections.push_back(dividePts(rIdx.first, std::get<1>(reval), true));
				}
				(lhs.intersections.end() - 1)->initPt = lhs.Arcs[lIdx.first].x[0];
				(lhs.intersections.end() - 2)->initPt = lhs.Arcs[lIdx.first].x[0];
				(rhs.intersections.end() - 1)->initPt = rhs.Arcs[rIdx.first].x[0];
				(rhs.intersections.end() - 2)->initPt = rhs.Arcs[rIdx.first].x[0];
				break;
			}
			default:
				break;
			}
		}
	}

}



/*!
*	\brief 들어온 두 Arc spline 모델을 확인하여, 두 ArcSpline의 각도가 겹친 범위의 Arc의 Index를 각각 ls, rs에 저장한 뒤 Convolution 함수로 넘겨준다
*
*	\param lhs 첫 번째 모델
*	\param rhs 두 번째 모델
*	\param source Convolution 결과를 저장
*/
void overlapTest(std::vector<ArcSpline> &source, ArcSpline & lhs, ArcSpline & rhs)
{
	// 1. build ls, rs and call convolution_ArcSpline

	// 왼쪽 모델의 첫 Index와 끝 Index : ls[0], ls[1]
	// 오른쪽 모델의 첫 Index와 끝 Index : rs[0], rs[1]
	int ls[2], rs[2];
	// 어느 Topology로 두 Arc가 겹쳐있는지 확인한다
	short overlap = overlapCase(lhs, rhs);
	// 각 경우에 대하여 인자를 맞추어 Con
	switch (overlap) {
	/*
		visually
			clockwise---------->>>
	lhs			 ---	----	 ----	-----	-----	  ---
	rhs			-----	 ----	----	 ---	  ---	-----
	return		  1		  3		  4		  6		  7?	  5?
	(notice that this is used on original models, and they are ccw. so n[0] to n[1] is ccw. So on left is n[1], while on right is n[0])
	*/
	// ls : lhs[idx] of idx ~ [ls[0], ls[1]] may have potential overlap... rs is for rhs
	case 1:
		ls[0] = 0 ;
		ls[1] = (int)lhs.Arcs.size() - 1;
		rs[0] = rhs.findIdx(lhs.n[0]);
		rs[1] = rhs.findIdx(lhs.n[1]);
		Convolution_ArcSpline(source, lhs, rhs, ls, rs, false);
		break;
	case 3:
		ls[0] = 0;
		ls[1] = lhs.findIdx(rhs.n[1]);
		rs[0] = rhs.findIdx(lhs.n[0]);
		rs[1] = (int)rhs.Arcs.size() - 1;
		Convolution_ArcSpline(source, lhs, rhs, ls, rs, false);
		break;
	case 4:
		ls[0] = lhs.findIdx(rhs.n[0]);
		ls[1] = (int)lhs.Arcs.size() - 1;
		rs[0] = 0;
		rs[1] = rhs.findIdx(lhs.n[1]);
		Convolution_ArcSpline(source, lhs, rhs, ls, rs, false);
		break;
	case 6:
		ls[0] = lhs.findIdx(rhs.n[0]);
		ls[1] = lhs.findIdx(rhs.n[1]);
		rs[0] = 0;
		rs[1] = (int)rhs.Arcs.size() - 1;
		Convolution_ArcSpline(source, lhs, rhs, ls, rs, false);
		break;
	case 5: // tag overlap
	case 7:
		ls[0] = 0; //this may be ineff?
		ls[1] = (int)lhs.Arcs.size() - 1;
		rs[0] = 0;
		rs[1] = (int)rhs.Arcs.size() - 1;
		Convolution_ArcSpline(source, lhs, rhs, ls, rs, false);
	default:
		break;
	}
}

/*!
*	\brief 들어온 두 Arc spline 모델을 확인하여, 두 ArcSpline의 각도가 겹친 범위의 Arc의 Index를 각각 ls, rs에 저장한 뒤 Convolution 함수로 넘겨준다
*	(단, 서로 다른 사분면에 속한 경우에 대한 함수)
*
*	\param lhs 첫 번째 모델
*	\param rhs 두 번째 모델
*	\param source Convolution 결과를 저장
*/
void overlapTestR(std::vector<ArcSpline> &source, ArcSpline & lhs, ArcSpline & rhs)
{
	int ls[2], rs[2];
	short overlap = overlapCaseR(lhs, rhs);
	switch (overlap) {
	case 1:
		ls[0] = lhs.ccw ? 0 : (int)lhs.Arcs.size() - 1;
		ls[1] = lhs.ccw ? (int)lhs.Arcs.size() - 1 : 0;
		rs[0] = rhs.findIdx(-lhs.n[0]);
		rs[1] = rhs.findIdx(-lhs.n[1]);
		Convolution_ArcSpline(source, lhs, rhs, ls, rs, true);
		break;
	case 3:
		ls[0] = lhs.ccw ? 0 : (int)lhs.Arcs.size() - 1;
		ls[1] = lhs.findIdx(-rhs.n[1]);
		rs[0] = rhs.findIdx(-lhs.n[0]);
		rs[1] = rhs.ccw ? (int)rhs.Arcs.size() - 1 : 0;
		Convolution_ArcSpline(source, lhs, rhs, ls, rs, true);
		break;
	case 4:
		ls[0] = lhs.findIdx(-rhs.n[0]);
		ls[1] = lhs.ccw ? (int)lhs.Arcs.size() - 1 : 0;
		rs[0] = rhs.ccw ? 0 : (int)rhs.Arcs.size() - 1;
		rs[1] = rhs.findIdx(-lhs.n[1]);
		Convolution_ArcSpline(source, lhs, rhs, ls, rs, true);
		break;
	case 6:
		ls[0] = lhs.findIdx(-rhs.n[0]);
		ls[1] = lhs.findIdx(-rhs.n[1]);
		rs[0] = rhs.ccw ? 0 : (int)rhs.Arcs.size() - 1;
		rs[1] = rhs.ccw ? (int)rhs.Arcs.size() - 1 : 0;
		Convolution_ArcSpline(source, lhs, rhs, ls, rs, true);
		break;
	case 5:
	case 7:
		// dbg_out
		std::cout << "overlapTestR : case reached : " << overlap << std::endl;
		// ~ dbg
	default:
		break;
	}
}

/*!
*	\brief 들어온 하나의 Arc spline 모델을 확인하여, 자기 자신과 Convolution한다
*	(단, 서로 같은 모델에 대한 함수)
*
*	\param lhs 첫 번째 모델
*	\param source Convolution 결과를 저장
*/
void overlapTestIden(std::vector<ArcSpline>& source, ArcSpline & lhs)
{
	std::vector<ArcSpline> reval;
	ArcSpline input;
	bool check = false;
	for (int i = 0; i < (int)lhs.Arcs.size(); i++) {
		check = false;
		auto c = CircularArc(2.0 * lhs.Arcs[i].c.c, 2.0 * lhs.Arcs[i].c.r, lhs.Arcs[i].n[0], lhs.Arcs[i].n[1]);
		if (c.trimmingTest()) {
			check = true;
			c.boundary = true;
			input.Arcs.push_back(c);
		}
		else if (input.Arcs.size() != 0) {
			input.ccw = true;
			input.referenced = false;
			reval.push_back(input);
			input.Arcs.clear();
		}
	}

	if ((input.Arcs.size() != 0)) {
		input.ccw = true;
		input.referenced = false;
		reval.push_back(input);
		input.Arcs.clear();
	}


	if (reval.size() != 0)
		source.insert(source.end(), reval.begin(), reval.end());
}

/*!
*	\brief 들어온 두 Arc spline 모델을 확인하여, 두 ArcSpline의 위상이 서로 어떻게 놓여있는지 판단한다
*
*	\param lhs 첫 번째 모델
*	\param rhs 두 번째 모델
*
*	\return 각각의 상황에 따라 1, 3, 4, 6의 값을 반환한다

in overlaptest(caller) 1,3,4,6,7 is used...
i guess...

visually
		clockwise---------->>>
lhs			 ---	----	 ----	-----	-----	  ---
rhs			-----	 ----	----	 ---	  ---	-----
return		  1		  3		  4		  6		  7?	  5?
(notice that this is used on original models, and they are ccw. so n[0] to n[1] is ccw. So on left is n[1], while on right is n[0])
0 can be returned => no overlap
2 can't
5 seems possible.
but 5,7 can be handled with other case?

*/
short overlapCase(ArcSpline & lhs, ArcSpline & rhs)
{
	short reval = 0;
	if (lhs.contain(rhs.n[0]))
		reval++;
	reval <<= 1;
	if (lhs.contain(rhs.n[1]))
		reval++;
	reval <<= 1;
	if (rhs.contain(lhs.n[0]))
		reval++;
	return reval;
}

/*!
*	\brief 들어온 두 Arc spline 모델을 확인하여, 두 ArcSpline의 위상이 서로 어떻게 놓여있는지 판단한다 (서로 다른 방향에 관한 함수)
*
*	\param lhs 첫 번째 모델
*	\param rhs 두 번째 모델
*
*	\return 각각의 상황에 따라 1, 3, 4, 6의 값을 반환한다

visually (actually they should be in one quadrant...)
      <--ccw--          <--ccw--          <--ccw--          <--ccw--        
lhs	    -----      ||     -----      ||     -----      ||     -----      || 
      --     --    ||   --     --    ||	  --     --    ||   --     --    ||
                   || --             ||	           --  || --         --  ||
				   || 			     ||				   || 			     ||
rhs	--  	   --  || 			 --  ||	--   		   || 			     ||
      --     --	   ||   --     --    ||	  --     --	   ||   --     --	 ||
        -----	   ||     -----	     ||	    -----	   ||     -----	     ||
	  <--cw--		    <--cw--			  <--cw--		    <--cw--
	      1			        3			      4			        6
*/
short overlapCaseR(ArcSpline & lhs, ArcSpline & rhs)
{
	short reval = 0;
	if (lhs.contain(-rhs.n[0]))
		reval++;
	reval <<= 1;
	if (lhs.contain(-rhs.n[1]))
		reval++;
	reval <<= 1;
	if (rhs.contain(-lhs.n[0]))
		reval++;
	return reval;
}

/* Def: in "reverse" case of convolution_arcspline, if rhs.rad > lhs.rad, ccw gets changed
*/
void flipArcWhenRHSDominates(CircularArc& c, CircularArc& lhs, CircularArc& rhs)
{
	// Arcspline's elements are ordered in a way following lhs's ccw
	// but in "reverse case" & "rhs.r > lhs.r" order get's swapped in global view
	if (rhs.c.r > lhs.c.r)
	{
		// swapping this before forming loop causes problem in finding loop...
		/*auto x = c.x[0];
		c.x[0] = c.x[1];
		c.x[1] = x;
		auto n = c.n[0];
		c.n[0] = c.n[1];
		c.n[1] = n;
		c.ccw  = !c.ccw;*/

		// so just set bool
		c.globalccw = !lhs.ccw;

	}
	else
		c.globalccw = lhs.ccw;

	//debug
	c.lhs = &lhs;
	c.rhs = &rhs;
}


/*!
*	\brief lhs, rhs의 각 Arcspline 모델을 Convolution 한 뒤, 이를 source에 저장한다
*
*	\param source 최종 결과를 저장한다
*	\param lhs 첫 번째 모델
*	\param lhs 두 번째 모델
*	\param ls 첫 번째 모델의 겹치는 Circular Arc의 Index
*	\param ls 두 번째 모델의 겹치는 Circular Arc의 Index
*	\param reverse Arc의 방향이 서로 같은지를 판단

TODO : the code should be fixed, so that all circular arcs are clockwise when they form the boundary loop
	or unify it ccw? and flip it?
*/
void Convolution_ArcSpline(std::vector<ArcSpline> &source, ArcSpline & lhs, ArcSpline & rhs, int ls[2], int rs[2], bool reverse)
{

	std::vector<ArcSpline> reval;
	ArcSpline input;
	bool check = false;

	// Arc가 같은 방향으로 볼록하다 (같은 사분면에 포함되어있다)
	// case1 : same quadrant
	if (!reverse) {
		// Arc의 반지름이 바뀌는 Normal을 저장
		// All resulting arcs are represented with normal interval [cut, next_cut]. where next_cut is the closest normal which there is a change in arc. After inserting arc, cut = next_cut.
		Vector cut;
		cut = counterclockwise(lhs.Arcs[ls[0]].n[0], rhs.Arcs[rs[0]].n[0]) ? rhs.Arcs[rs[0]].n[0] : lhs.Arcs[ls[0]].n[0];

		// 1-1. simulataneously move i & j to find overlapping normals. 
		for (int i = ls[0], j = rs[0]; (i != ls[1]) || (j != rs[1]);) 
		{
			check = false;
			if (i == ls[1]) {
				auto c = CircularArc(lhs.Arcs[i].c.c + rhs.Arcs[j].c.c, lhs.Arcs[i].c.r + rhs.Arcs[j].c.r, cut, rhs.Arcs[j].n[1]);
				if (c.trimmingTest()) {
					check = true;
					c.boundary = true;
					input.Arcs.push_back(c);
				}

				cut = rhs.Arcs[j].n[1];
				j++;
			}
			else if (j == rs[1]) {
				auto c = CircularArc(lhs.Arcs[i].c.c + rhs.Arcs[j].c.c, lhs.Arcs[i].c.r + rhs.Arcs[j].c.r, cut, lhs.Arcs[i].n[1]);
				if (c.trimmingTest()) {
					check = true;
					c.boundary = true;
					input.Arcs.push_back(c);
				}

				cut = lhs.Arcs[i].n[1];
				i++;
			}
			else if (counterclockwise(lhs.Arcs[i].n[1], rhs.Arcs[j].n[1])) {
				auto c = CircularArc(lhs.Arcs[i].c.c + rhs.Arcs[j].c.c, lhs.Arcs[i].c.r + rhs.Arcs[j].c.r, cut, lhs.Arcs[i].n[1]);
				if (c.trimmingTest()) {
					check = true;
					c.boundary = true;
					input.Arcs.push_back(c);
				}

				cut = lhs.Arcs[i].n[1];
				i++;
			}
			else {
				auto c = CircularArc(lhs.Arcs[i].c.c + rhs.Arcs[j].c.c, lhs.Arcs[i].c.r + rhs.Arcs[j].c.r, cut, rhs.Arcs[j].n[1]);
				if (c.trimmingTest()) {
					check = true;
					c.boundary = true;
					input.Arcs.push_back(c);
				}

				cut = rhs.Arcs[j].n[1];
				j++;
			}

			if (!check && (input.Arcs.size() != 0)) {
				input.ccw = true;
				input.referenced = false;
				reval.push_back(input);
				input.Arcs.clear();
			}
		} //~ for(;;)

		// 1-2. after for() ends, take care of the last arc(i=ls[1], j=rs[1])
		if (counterclockwise(lhs.Arcs[ls[1]].n[1], rhs.Arcs[rs[1]].n[1])) {
			auto c = CircularArc(lhs.Arcs[ls[1]].c.c + rhs.Arcs[rs[1]].c.c, lhs.Arcs[ls[1]].c.r + rhs.Arcs[rs[1]].c.r, cut, lhs.Arcs[ls[1]].n[1]);
			if (c.trimmingTest()) {
				c.boundary = true;
				input.Arcs.push_back(c);
			}

		}
		else {
			auto c = CircularArc(lhs.Arcs[ls[1]].c.c + rhs.Arcs[rs[1]].c.c, lhs.Arcs[ls[1]].c.r + rhs.Arcs[rs[1]].c.r, cut, rhs.Arcs[rs[1]].n[1]);
			if (c.trimmingTest()) {
				c.boundary = true;
				input.Arcs.push_back(c);
			}

		}

		if (input.Arcs.size() != 0) {	
			// ccw를 다른 정보를 담는 것으로 재활용!
			// if ccw = true, convex
			// else, concave
			// in this case, (not reversed) ccw of lhs and rhs is same!
			input.referenced = false;
			input.ccw = true;
			reval.push_back(input);
			input.Arcs.clear();
		}


	} //~ if (!reverse)
	else {
		//Arc가 다른 방향! rhs의 방향을 뒤집어서 생각하자
		// case2 : opposite quadrant
		// To form topology, change the code
		// 1. so that new arc follows cw/ccw of its 2 inputs
		// + and arcspline as a whole also follows ccw
		// 2. continuous arcs? ( 1 may flip arcs... )
		Vector cut;
		int ld = lhs.ccw ? 1 : -1;
		int rd = rhs.ccw ? 1 : -1;
		cut = counterclockwise(lhs.Arcs[ls[0]].n[0], -rhs.Arcs[rs[0]].n[0]) ? -rhs.Arcs[rs[0]].n[0] : lhs.Arcs[ls[0]].n[0];
		for (int i = ls[0], j = rs[0]; (i != ls[1]) || (j != rs[1]);) 
		{
			check = false;
			if (i == ls[1]) {
				auto c = CircularArc(lhs.Arcs[i].c.c + rhs.Arcs[j].c.c, lhs.Arcs[i].c.r - rhs.Arcs[j].c.r, cut, -rhs.Arcs[j].n[1]);
				if (c.trimmingTest() && ((lhs.ccw && (lhs.Arcs[i].c.r - rhs.Arcs[j].c.r < 0.0)) || (rhs.ccw && (lhs.Arcs[i].c.r - rhs.Arcs[j].c.r > 0.0)))) {
					check = true;
					c.boundary = c.ccw ^ rhs.ccw;
					flipArcWhenRHSDominates(c, lhs.Arcs[i], rhs.Arcs[j]);
					input.Arcs.push_back(c);
				}
				cut = -rhs.Arcs[j].n[1];
				j += rd;
			}
			else if (j == rs[1]) {
				auto c = CircularArc(lhs.Arcs[i].c.c + rhs.Arcs[j].c.c, lhs.Arcs[i].c.r - rhs.Arcs[j].c.r, cut, lhs.Arcs[i].n[1]);
				if (c.trimmingTest() && ((lhs.ccw && (lhs.Arcs[i].c.r - rhs.Arcs[j].c.r < 0.0)) || (rhs.ccw && (lhs.Arcs[i].c.r - rhs.Arcs[j].c.r > 0.0)))) {
					check = true;
					c.boundary = c.ccw ^ rhs.ccw;
					flipArcWhenRHSDominates(c, lhs.Arcs[i], rhs.Arcs[j]);
					input.Arcs.push_back(c);
				}

				cut = lhs.Arcs[i].n[1];
				i += ld;
			}
			else if (counterclockwise(lhs.Arcs[i].n[1], -rhs.Arcs[j].n[1])) {
				auto c = CircularArc(lhs.Arcs[i].c.c + rhs.Arcs[j].c.c, lhs.Arcs[i].c.r - rhs.Arcs[j].c.r, cut, lhs.Arcs[i].n[1]);
				if (c.trimmingTest() && ((lhs.ccw && (lhs.Arcs[i].c.r - rhs.Arcs[j].c.r < 0.0)) || (rhs.ccw && (lhs.Arcs[i].c.r - rhs.Arcs[j].c.r > 0.0)))) {
					check = true;
					c.boundary = c.ccw ^ rhs.ccw;
					flipArcWhenRHSDominates(c, lhs.Arcs[i], rhs.Arcs[j]);
					input.Arcs.push_back(c);
				}

				cut = lhs.Arcs[i].n[1];
				i += ld;
			}
			else {
				auto c = CircularArc(lhs.Arcs[i].c.c + rhs.Arcs[j].c.c, lhs.Arcs[i].c.r - rhs.Arcs[j].c.r, cut, -rhs.Arcs[j].n[1]);
				if (c.trimmingTest() && ((lhs.ccw && (lhs.Arcs[i].c.r - rhs.Arcs[j].c.r < 0.0)) || (rhs.ccw && (lhs.Arcs[i].c.r - rhs.Arcs[j].c.r > 0.0)))) {
					check = true;
					c.boundary = c.ccw ^ rhs.ccw;
					flipArcWhenRHSDominates(c, lhs.Arcs[i], rhs.Arcs[j]);
					input.Arcs.push_back(c);
				}

				cut = -rhs.Arcs[j].n[1];
				j += rd;
			}

			if (!check && (input.Arcs.size() != 0)) {
				input.ccw = rhs.ccw ^ input.Arcs[0].ccw;
				input.referenced = false;
				reval.push_back(input);
				input.Arcs.clear();
			}
		} // ~for

		if (counterclockwise(lhs.Arcs[ls[1]].n[1], -rhs.Arcs[rs[1]].n[1])) {
			auto c = CircularArc(lhs.Arcs[ls[1]].c.c + rhs.Arcs[rs[1]].c.c, lhs.Arcs[ls[1]].c.r - rhs.Arcs[rs[1]].c.r, cut, lhs.Arcs[ls[1]].n[1]);
			if (c.trimmingTest() && ((lhs.ccw && (lhs.Arcs[ls[1]].c.r - rhs.Arcs[rs[1]].c.r < 0.0)) || (rhs.ccw && (lhs.Arcs[ls[1]].c.r - rhs.Arcs[rs[1]].c.r > 0.0)))) {
				c.boundary = c.ccw ^ rhs.ccw;
				flipArcWhenRHSDominates(c, lhs.Arcs[ls[1]], rhs.Arcs[rs[1]]);
				input.Arcs.push_back(c);
			}

		}
		else {
			auto c = CircularArc(lhs.Arcs[ls[1]].c.c + rhs.Arcs[rs[1]].c.c, lhs.Arcs[ls[1]].c.r - rhs.Arcs[rs[1]].c.r, cut, -rhs.Arcs[rs[1]].n[1]);
			if (c.trimmingTest() && ((lhs.ccw && (lhs.Arcs[ls[1]].c.r - rhs.Arcs[rs[1]].c.r < 0.0)) || (rhs.ccw && (lhs.Arcs[ls[1]].c.r - rhs.Arcs[rs[1]].c.r > 0.0)))) {
				c.boundary = c.ccw ^ rhs.ccw;
				flipArcWhenRHSDominates(c, lhs.Arcs[ls[1]], rhs.Arcs[rs[1]]);
				input.Arcs.push_back(c);
			}

		}

		//ccw = ((lhs.ccw) && Models_Rotated_Approx[0].ccw) ^ !((rhs.ccw) && (!Models_Rotated_Approx[0].ccw));
		if (input.Arcs.size() != 0) {
			input.ccw = rhs.ccw ^ input.Arcs[0].ccw;
			input.referenced = false;
			reval.push_back(input);
			input.Arcs.clear();
		}
		

	}

	if (reval.size() != 0)
		source.insert(source.end(), reval.begin(), reval.end());
}

/*!
*	\brief 두 Arc Spline의 aabb 테스트를 한다
*
*	\param lhs 첫 번째 모델
*	\param rhs 두 번째 모델
*
*	\return 두 aabb box가 충돌하면 true, 아니면 false를 반환한다

Def: using the fact that arcSplines are x,y-monotone, get its AABB by endpoints. Then check its overlap.

Q: is this better than interval test? => we do not know wheter arcSpline.0 < arcSpline.1, so maybe?
*/
bool aabbtest(ArcSpline & lhs, ArcSpline & rhs)
{
	double width = abs(lhs.init().P[0] - lhs.end().P[0]) + abs(rhs.init().P[0] - rhs.end().P[0]);
	double wDiff = abs((lhs.init().P[0] + lhs.end().P[0]) - (rhs.init().P[0] + rhs.end().P[0]));
	if (width < wDiff)
		return false;
	double height = abs(lhs.init().P[1] - lhs.end().P[1]) + abs(rhs.init().P[1] - rhs.end().P[1]);
	double hDiff = abs((lhs.init().P[1] + lhs.end().P[1]) - (rhs.init().P[1] + rhs.end().P[1]));
	if (height < hDiff)
		return false;
	return true;
}

/*
Def : aabbtest, but assume that rhs is flipped over a point p

Change : 
	Actually, The other part's comment said it flips with the center of symmtery of P.
	But what its code did was : 1. flip with origin 2. translate with p.
	So this part does it like that
*/
bool aabbtest(ArcSpline& lhs, ArcSpline& rhs, Point& p)
{
	// Naming convention [left/right][init/end][x/y]
	double
		lix = lhs.init().P[0],
		liy = lhs.init().P[1],
		lex = lhs. end().P[0],
		ley = lhs. end().P[1];
	double
		rix = /* 2 * */ p.P[0] - rhs.init().P[0],
		riy = /* 2 * */ p.P[1] - rhs.init().P[1],
		rex = /* 2 * */ p.P[0] - rhs. end().P[0],
		rey = /* 2 * */ p.P[1] - rhs. end().P[1];

	double width  = abs( lix - lex) + abs(rix - rex);
	double wDiff  = abs((lix + lex) -    (rix + rex));
	if (width < wDiff)
		return false;
	double height = abs( liy - ley) + abs(riy - rey);
	double hDiff  = abs((liy + ley) -    (riy + rey));
	if (height < hDiff)
		return false;
	return true;
}

/*!
*	\brief 두 Arc Spline의 aabb 테스트를 한다
*
*	\param lhs 첫 번째 Arc Spline Segment
*	\param rhs 두 번째 Arc Spline Segment
*	\param left 첫 번째 모델의 Arc Index의 범위
*	\param right 두 번째 모델의 Arc Index의 범위
*
*	\return lhs의 left, rhs의 right 범위에 있는 Arc로 생성한 두 aabb box가 충돌하면 true, 아니면 false를 반환한다
*/
bool aabbtest(ArcSpline & lhs, ArcSpline & rhs, std::pair<int, int>& left, std::pair<int, int>& right)
{
	double width = abs(lhs.Arcs[left.first].x[0].P[0] - lhs.Arcs[left.second].x[1].P[0]) + abs(rhs.Arcs[right.first].x[0].P[0] - rhs.Arcs[right.second].x[1].P[0]);
	double wDiff = abs((lhs.Arcs[left.first].x[0].P[0] + lhs.Arcs[left.second].x[1].P[0]) - (rhs.Arcs[right.first].x[0].P[0] + rhs.Arcs[right.second].x[1].P[0]));
	if (width < wDiff)
		return false;
	double height = abs(lhs.Arcs[left.first].x[0].P[1] - lhs.Arcs[left.second].x[1].P[1]) + abs(rhs.Arcs[right.first].x[0].P[1] - rhs.Arcs[right.second].x[1].P[1]);
	double hDiff = abs((lhs.Arcs[left.first].x[0].P[1] + lhs.Arcs[left.second].x[1].P[1]) - (rhs.Arcs[right.first].x[0].P[1] + rhs.Arcs[right.second].x[1].P[1]));
	if (height < hDiff)
		return false;
	return true;
}

/*!
*	\brief 두 Arc Spline의 aabb 테스트를 한다
*
*	\param lhs 첫 번째 Circular Arc
*	\param rhs 두 번째 Arc Spline Segment
*	\param right 두 번째 모델의 Arc Index의 범위
*
*	\return lhs와 rhs의 right 범위에 있는 Arc로 생성한 두 aabb box가 충돌하면 true, 아니면 false를 반환한다
*/
bool aabbtest(CircularArc & lhs, ArcSpline & rhs, std::pair<int, int>& right)
{
	double width = abs(lhs.x[0].P[0] - lhs.x[1].P[0]) + abs(rhs.Arcs[right.first].x[0].P[0] - rhs.Arcs[right.second].x[1].P[0]);
	double wDiff = abs((lhs.x[0].P[0] + lhs.x[1].P[0]) - (rhs.Arcs[right.first].x[0].P[0] + rhs.Arcs[right.second].x[1].P[0]));
	if (width < wDiff)
		return false;
	double height = abs(lhs.x[0].P[1] - lhs.x[1].P[1]) + abs(rhs.Arcs[right.first].x[0].P[1] - rhs.Arcs[right.second].x[1].P[1]);
	double hDiff = abs((lhs.x[0].P[1] + lhs.x[1].P[1]) - (rhs.Arcs[right.first].x[0].P[1] + rhs.Arcs[right.second].x[1].P[1]));
	if (height < hDiff)
		return false;
	return true;
}

/*!
*	\brief Arc Spline의 정보 출력
*
*	\param os cout
*	\param p Arc Spline
*
*	\return Arc Spline를 이루는 각 Circular Arc를 순서대로 출력한다
*/
std::ostream & operator<<(std::ostream & os, const ArcSpline & p)
{
	std::cout << "ArcSpline: \n";
	for (int i = 0; i < (int)p.Arcs.size(); i++) {
		std::cout << p.Arcs[i] << std::endl;
	}
	std::cout << std::endl;
	return os;
}

/*!
*	\brief input으로 들어온 Arc Spline Segment들을 Trimming - 실제로는 각 Circular Arc를 Convolution할 때 동시에 Trimming 하지만, Statistic Data를 뽑아낼 때 Trimming Time을 따로 측정하기 위해 따로 함수를 구현하였음.
*
*	\param input trimming하기 전 Arc Spline Segments
*	\param trimmed trimming을 한 뒤의 Arc Spline Segments
*/
void circleTrimming(std::vector<ArcSpline>& input, std::vector<ArcSpline>& trimmed)
{
	for (int i = 0; i < (int)input.size(); i++) {
		ArcSpline temp;
		for (int j = 0; j < (int)input[i].Arcs.size(); j++) {
			if (input[i].Arcs[j].trimmingTest()) {
				temp.Arcs.push_back(input[i].Arcs[j]);
			}
		}
		if (temp.Arcs.size() != 0) {
			temp.ccw = input[i].ccw;
			temp.n[0] = input[i].n[0];
			temp.n[1] = input[i].n[1];
			temp.xQuardrants = input[i].xQuardrants;
			temp.yQuardrants = input[i].yQuardrants;
			trimmed.push_back(temp);
		}
	}
}

/*!
*	\brief 두 BCA의 충돌 여부를 판단
*
*	\param lhs 첫 번째 BCA
*	\param rhs 두 번째 BCA
*/
bool Collision_BCA(BCA & lhs, BCA & rhs)
{
	if (std::get<2>(intersection_CircularArc(lhs.outer, rhs.outer)) != 0)
		return true;
	if (std::get<2>(intersection_CircularArc(lhs.inner, rhs.outer)) != 0)
		return true;
	if (std::get<2>(intersection_CircularArc(lhs.outer, rhs.inner)) != 0)
		return true;
	if (lhs.contain(rhs.x[0]))
		return true;
	if (lhs.contain(rhs.x[1]))
		return true;
	if (rhs.contain(lhs.x[0]))
		return true;
	if (rhs.contain(lhs.x[1]))
		return true;
	return false;
}

/*!
*	\brief Circular Arc와 BCA의 충돌 여부를 판단
*
*	\param lhs Circular Arc
*	\param rhs BCA
*/
bool Collision_BCA(CircularArc & lhs, BCA & rhs)
{
	if (std::get<2>(intersection_CircularArc(lhs, rhs.inner)))
		return true;
	if (std::get<2>(intersection_CircularArc(lhs, rhs.outer)))
		return true;
	if (rhs.contain(lhs.x[0]))
		return true;
	if (rhs.contain(lhs.x[1]))
		return true;
	return false;
}

/*!
*	\brief 두 dividePts의 대소를 비교
*
*	\param lhs 첫 번째 dividePts
*	\param rhs 두 번째 dividePts
*
*	\return 두 dividePts의 대소관계를 판단하여 반환한다. 해당 Arc Spline Segment의 시작점과 가까울 수록 작은 값이다.
*/
bool operator<(dividePts & lhs, dividePts & rhs)
{
	if (lhs.idx != rhs.idx)
		return lhs.idx < rhs.idx;
	else
		return distance(lhs.initPt, lhs.dividePt) < distance(lhs.initPt, rhs.dividePt);
}


/*!
*	\brief 기본 생성자
*/
Line::Line() {
	P[0] = Point();
	P[1] = Point();
	L[0] = 0.0;
	L[1] = 0.0;
	L[2] = 0.0;
}

/*!
*	\brief 두 점을 지나는 직선을 구한다
*
*	\param p 첫번쨰 점
*	\param q 두번째 점
*
*	\return 두 점을 지나는 직선의 방정식의 계수를 반환한다
*/
Line::Line(Point & p, Point & q) {
	L[0] = p[1] - q[1];
	L[1] = q[0] - p[0];
	L[2] = p[0] * q[1] - p[1] * q[0];
	P[0] = p;
	P[1] = q;
}

/*!
*	\brief 소멸자
*/
Line::~Line()
{
}


/*!
*	\brief 두 Line의 대소비교
*
*	\param rhs 비교하는 Line
*
*	\return 두 Line의 길이를 비교하여 그 대소관계를 반환한다.
*/
bool Line::operator<(const Line & rhs)
{
	return counterclockwise((P[1] - P[0]), (rhs.P[1] - rhs.P[0]));
}


/*!
*	\brief Line을 뒤집는다
*
*	\return Line의 시작점과 끝점을 서로 바꾸어 반환한다.
*/
Line Line::operator-()
{
	return Line(P[1], P[0]);
}

/*!
*	\brief Line 위의 내/외분점을 반환
*
*	\param t 두 점 사이의 비율
*
*	\return Line 위의 t : 1-t 내분점을 반환한다. (t < 0 or 1-t < 0인 경우 외분점)
*/
Point Line::operator()(double t)
{
	return (1 - t) * P[0] + t * P[1];
}

/*!
*	\brief p를 지나고 dir와 평행한 직선
*
*	\param p 첫번쨰 점
*	\param dir 방향벡터
*
*	\return p를 지나고 dir와 평행한 직선을 반환한다.
*/
Line shoot(Point & p, Point & dir)
{
	return Line(p, p + dir);
}

/*!
*	\brief p와 q의 수직이등분선
*
*	\param p 첫번쨰 점
*	\param q 두번째 점
*
*	\return p와 q의 수직이등분선을 반환한다.
*/
Line bisector(Point & p, Point & q)
{
	return shoot((p + q) / 2, (q - p).rotate());
}

/*!
*	\brief 중심이 _c, 반지름이 _r인 원
*
*	\param _c 원의 중심
*	\param _r 원의 반지름
*/
Circle::Circle(Point & _c, double _r)
{
	c = _c, r = _r;
}

/*!
*	\brief 중심이 _c, x를 지나는 원
*
*	\param _c 원의 중심
*	\param x 원이 지나는 점
*/
Circle::Circle(Point & _c, Point & p)
{
	c = _c, r = sqrt(distance(_c, p));
}

/*!
*	\brief p, q를 지나고 p에서의 접선벡터가 v인 원
*
*	\param p 첫번째 점
*	\param v 첫번째 점에서의 접선벡터
*	\param q 두번째 점
*/
Circle::Circle(Point & p, Point & q, Point & v)
	:c(shoot(p, v.rotate()), bisector(p, q)), r(sqrt(distance(c, p))) {}

Circle::~Circle()
{
}

/*!
*	\brief 원 위에있는 점 중 angle 각도를 가진 점
*
*	\param p 원의 local polar coordinate에서의 angle값
*
*	\return 원위에 있는 angle 각도의 점을 반환한다.
*/
Point Circle::operator()(double angle)
{
	return Point(cos(angle), sin(angle)) * r + c;
}

/*!
*	\brief 원의 대소관계를 반환
*
*	\param rhs 비교하는 원
*
*	\return 나 자신과 rhs의 반지름의 크기를 비교하여 그 대소관계를 반환한다.
*/
bool Circle::operator<(const Circle & rhs)
{
	return (r < rhs.r);
}

/*!
*	\brief 원의 대소관계를 반환
*
*	\param rhs 비교하는 원
*
*	\return 나 자신과 rhs의 반지름의 크기를 비교하여 그 대소관계를 반환한다.
*/
bool Circle::operator>(const Circle & rhs)
{
	return (r > rhs.r);
}

/*!
*	\brief 점 p가 원에 포함되는지를 판단
*
*	\param p 포함되는지 판단하려는 객체
*
*	\return 점 p가 원에 포함되면 true, 그렇지 않으면 false를 반환한다.
*/
bool Circle::contain(Point & p)
{
	return distance(c, p)  < r * r;
}

/*!
*	\brief 점 p가 원에 포함되는지를 판단
*
*	\param p 포함되는지 판단하려는 객체
*
*	\return 점 p가 원에 포함되면 true, 그렇지 않으면 false를 반환한다. Trimming을 위한 함수이므로, Interior Disk의 오차를 감안하여 조금 더 보수적으로 true를 반환한다.
*/
bool Circle::contain_trimming(Point & p)
{
	return distance(c, p) < r * r * 0.95;
}

/*!
*	\brief 점 p와 Circular Arc의 양 끝점이 원에 포함되는지를 판단
*
*	\param Arc 포함되는지 판단하려는 객체
*	\param p 포함되는지 판단하려는 객체
*
*	\return 점 p와 Circular Arc가 모두 원에 포함되면 true, 그렇지 않으면 false를 반환한다.
*/
bool Circle::contain(CircularArc & Arc, Point & p)
{
	return contain_trimming(Arc.x[0]) && contain_trimming(Arc.x[1]) && (contain_trimming(p));
}



/*!
*	\brief 원의 중심으로부터 점 p까지의 반직선과 원의 교점
*
*	\param p projection하려는 객체
*
*	\return 원의 중심으로부터 점 p까지의 반직선과 원의 교점을 반환한다.
*/
Point Circle::projection(Point & p)
{
	return (p - c).normalize() * r + c;
}

/*!
*	\brief 원의 중심에서 점 p까지의 단위벡터
*
*	\param p 원 위의 점
*
*	\return 원의 중심에서 바라본 p방향의 단위벡터를 반환한다
*/
Point Circle::localDirection(Point & p)
{
	return (p - c).normalize();
}

/*!
*	\brief 원 위의 점을 CRES개 만큼 Sampling하여 그림
*/
void Circle::draw()
{
	glBegin(GL_POLYGON);
	for (int i = 0; i <= CRES; i++) {
		double angle = 2 * PI*(double)i / (double)CRES;
		Point unit = { cos(angle), sin(angle) };
		Point p = c + r * 1.0 * unit;
		glVertex2dv(p.P);
	}
	glEnd();
}

/*!
*	\brief Circular Arc를 p에 대하여 점대칭
*
*	\param arc 대칭시키는 객체
*	\param p 대칭점
*
*	\return arc를 p에 대하여 대칭시킨 Circular Arc를 반환한다.
*/
CircularArc::CircularArc(CircularArc & arc, Point & p)
{
	c.c = -arc.c.c + p;
	c.r = arc.c.r;
	x[0] = -arc.x[0] + p;
	x[1] = -arc.x[1] + p;
	n[0] = -arc.n[0];
	n[1] = -arc.n[1];
}

/*!
*	\brief i부터 e까지, i에서의 접선이 t인 Arc 생성
*
*	\param i 시작점
*	\param e 끝점
*	\param t 시작점의 접선방향
*/
CircularArc::CircularArc(Point & i, Point & e, Point & t)
	:c(i, e, t), x{ i, e }, n{ c.localDirection(i), c.localDirection(e) }
{
	if (!counterclockwise(n[0], n[1])) {
		ccw = false;
		std::swap(n[0], n[1]); //왜인지는 모르지만 일단
	}
	else
		ccw = true;
	boundary = false;
}

/*!
*	\brief 원의 중심이 c, 반지름이 r, Circular Arc의 두 끝점의 수직방향이 각각 normal1, normal2인 Circular Arc를 생성
*
*	\param _c 원의 중심
*	\param _r 원의 반지름
*	\param normal1 시작점의 수직 방향
*	\param normal2 끝점의 수직 방향
*/
CircularArc::CircularArc(Point & _c, double _r, Vector normal1, Vector normal2)
{
	if (_r > 0.0) {
		c.c = _c;
		c.r = _r;
		x[0] = normal1 * _r + _c;
		x[1] = normal2 * _r + _c;
		n[0] = normal1;
		n[1] = normal2;
		ccw = true;	//in this case, purpose of ccw is to determine the sign of radius. => No... This also includes info about globalCCW
	}
	else {
		c.c = _c;
		c.r = -_r;
		x[0] = normal1 * _r + _c;
		x[1] = normal2 * _r + _c;
		n[0] = -normal1;
		n[1] = -normal2;
		ccw = false;
	}
}

/*!
*	\brief 소멸자
*/
CircularArc::~CircularArc()
{
}

/*!
*	\brief i와 e 사이를 angle에 대해 0부터 1사이의 t로 매개화했을때, t의 값에 따른 점의 위치
*
*	\return i와 e 사이를 angle에 대해 0부터 1사이의 t로 매개화했을때, t의 값에 따른 점의 위치를 반환한다.
*/
Point CircularArc::operator()(double t)
{
	if (t > 1.000 + N_PRESCISION || t < -N_PRESCISION)
		std::cout << "error: t is out of coverage" << std::endl;//for debuging
	return c.projection(x[0] + t * (x[1] - x[0]));
}

/*!
*	\brief 호의 방향을 바꾸어주는 연산자
*/
CircularArc CircularArc::operator-()
{
	auto s = *this;
	std::swap(s.x[0], s.x[1]);
	return s;
}


/*!
*	\brief Circular Arc의 반지름 비교
*
*	\param rhs 비교하는 객체
*
*	\return 두 Circular Arc의 반지름의 대소관계를 반환.
*/
bool CircularArc::operator<(CircularArc & rhs)
{
	return c < rhs.c;
}


/*!
*	\brief Circular Arc의 x, y extreme부근의 normal의 값을 보정 x / y축의 방향과 방향이 비슷하면 이를 정확히 x / y축의 방향으로 바꾸어준다
*/
void CircularArc::refineNormal()
{
	if (n[0].P[0] > 1.0 - N_HIGH_PRESCISION) {
		n[0].P[0] = 1.0;
		n[0].P[1] = 0.0;
		if (ccw)
			x[0] = c.c + c.r * n[0];
		else
			x[1] = c.c + c.r * n[0];
	}
	if (n[1].P[1] > 1.0 - N_HIGH_PRESCISION) {
		n[1].P[1] = 1.0;
		n[1].P[0] = 0.0;
		if (ccw)
			x[1] = c.c + c.r * n[1];
		else
			x[0] = c.c + c.r * n[1];
	}
	if (n[0].P[0] < -1.0 + N_HIGH_PRESCISION) {
		n[0].P[0] = -1.0;
		n[0].P[1] = 0.0;
		if (ccw)
			x[0] = c.c + c.r * n[0];
		else
			x[1] = c.c + c.r * n[0];
	}
	if (n[1].P[1] < -1.0 - N_HIGH_PRESCISION) {
		n[1].P[1] = -1.0;
		n[1].P[0] = 0.0;
		if (ccw)
			x[1] = c.c + c.r * n[1];
		else
			x[0] = c.c + c.r * n[1];
	}

}


/*!
*	\brief Circular Arc가 Interior Disks에 포함되는지를 확인
*
*	\param p Circular Arc의 양 끝점에서의 접선이 만나는 점. Circular Arc의 양 끝점과 p로 삼각형을 만들면, Circular Arc가 Bounding된다.
*	\param init 조사하는 Cache의 시작 Index
*	\param Cache_Trimming Caching된 Interior Disk의 주소가 담겨있는 클래스
*	\param Grid_Trimming 각 Grid에 포함된 Interior Disk 주소가 저장되어있는 클래스
*	\param FindPosition Circular Arc가 포함된 Grid의 위치를 반환
*	\param SingleGrid Circular Arc가 하나의 Grid에만 포함되는지를 확인
*
*	\return Circular Arc가 Trimming되지 않는 경우 True를 반환한다.
*/
bool CircularArc::trimmingTest()
{
	if (x[0].exact(x[1]))
		return false;
	
	Point p = Point(*this);

	/* Trimming Using Cache */
	int init = Cache_Trimming.idx;
	do {
		if (Cache_Trimming.cache[Cache_Trimming.idx]->contain(*this, p)) {
			// Trimming (Cache Hit)
			// check : 최근에 해당 Disk가 Trimming을 성공했음을 Check함
			Cache_Trimming.check[Cache_Trimming.idx] = true;
			return false;
		}
		if (Cache_Trimming.idx == cacheSize - 1)
			Cache_Trimming.idx = 0;
		else
			Cache_Trimming.idx++;
	} while (init != Cache_Trimming.idx);

	// FP[0][1][2][3] -> {xmin, xmax, ymin, ymax}의 position
	std::vector<int> FindPosition;
	try
	{
		FindPosition = Grid_Trimming.find(p, x[0], x[1]);
	}
	catch (...)
	{
		//dbg_out
		std::cout << "catched" << std::endl;
		return false;
	}

	bool SingleGrid = (FindPosition[0] == FindPosition[1]) && (FindPosition[2] == FindPosition[3]);

	for(int i = FindPosition[0]; i <= FindPosition[1]; i++)
		for (int j = FindPosition[2]; j <= FindPosition[3]; j++) {
			auto result = Grid_Trimming.trimming(p, x[0], x[1], i, j, SingleGrid);
			// if covered
			if ((result.first == NULL) && result.second) {
				// Cache update
				// Second Chance Algorithm으로 Cache에서 지울 Index를 찾아냄
				while (Cache_Trimming.check[Cache_Trimming.idx]) {
					Cache_Trimming.check[Cache_Trimming.idx++] = false;
					if (Cache_Trimming.idx == cacheSize)
						Cache_Trimming.idx = 0;
				}
				Cache_Trimming.cache[Cache_Trimming.idx] = Grid_Trimming.coverCircle[i][j];
				Cache_Trimming.check[Cache_Trimming.idx] = true;
				return false;
			}
			// Cache Update
			else if (result.first != NULL) {
				while (Cache_Trimming.check[Cache_Trimming.idx]) {
					Cache_Trimming.check[Cache_Trimming.idx++] = false;
					if (Cache_Trimming.idx == cacheSize)
						Cache_Trimming.idx = 0;
				}
				Cache_Trimming.cache[Cache_Trimming.idx] = result.first;
				Cache_Trimming.check[Cache_Trimming.idx] = true;
				return false;
			}
		}
	//fail
	return true;
}

/*!
*	\brief 점이 호 위에 있는지 판단
*
*	\param p 판단하는 점
*
*	\return 점이 호 위에 있으면 true, 없으면 false를 반환한다.
*/
bool CircularArc::contain(Point & p)
{
	Point q = c.localDirection(p);
	return counterclockwise(n[0], q) && counterclockwise(q, n[1]);
}

/*!
*	\brief Circular Arc가 반시계방향인지 판단
*
*	\return Circular Arc가 반시계방향인 경우 true를 반환한다.
*/
bool CircularArc::isCCW()
{
	return counterclockwise(x[0] - c.c, x[1] - c.c);
}

/*!
*	\brief Circular Arc가 x축의 양의방향으로 Convex인지 판단
*
*	\return Circular Arc가 x축의 양의방향으로 Convex인 경우 true를 반환한다.
*/
bool CircularArc::isXQuardrants()
{
	Vector test = n[0] + n[1];
	return (test.P[0] > 0);
}


/*!
*	\brief Circular Arc가 y축의 양의방향으로 Convex인지 판단
*
*	\return Circular Arc가 y축의 양의방향으로 Convex인 경우 true를 반환한다.
*/
bool CircularArc::isYQuardrants()
{
	Vector test = n[0] + n[1];
	return (test.P[1] > 0);
}


/*!
*	\brief Circular Arc의 Convex 부분이 외부인지를 판단
*
*	\return Circular Arc의 Convex 바깥쪽 부분이 Model의 외부인 경우 true를 반환한다.
*/
bool CircularArc::isOuterBoundary(int _case)
{
	switch (_case) {
	case 0:
		return boundary && isXQuardrants();
	case 1:
		return boundary && !isXQuardrants();
	case 2:
		return boundary && isYQuardrants();
	case 3:
		return boundary && !isYQuardrants();
	default:
		return false;
	}
}



/*!
*	\brief 호를 두개로 나눔
*
*	\param t 호를 0부터 1까지의 실수로 매개화 했을 때 호 위의 위치를 나타내는 변수
*
*	\return 호를 매개화된 t로 나누어 두 호의 쌍을 출력한다.
*/
std::pair<CircularArc, CircularArc> CircularArc::subDiv(double t)
{
	if (t > 1 || t < 0)
		std::cout << "error: t is out of coverage" << std::endl;//for debuging
	auto p = c.projection(x[0] + t*(x[1] - x[0]));
	return std::make_pair(CircularArc(x[0], p, c.localDirection(x[0]).rotate()), CircularArc(p, x[1], c.localDirection(p).rotate()));
}

/*!
*	\brief RES개의 점에서 Sampling하여 Circular Arc를 그림
*/
void CircularArc::draw()
{
	glBegin(GL_POINTS);
	glVertex2dv(x[0].P);
	glVertex2dv(x[1].P);
	glEnd();
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i <= RES; i++) {
		//glColor3f((float)i / (float)RES, 1.0f - (float)i / (float)RES, 0.5f); //for testing direction
		glVertex2dv((*this)((double)i / RES).P);
	}
	glEnd();
}

/*!
*	Def: Circular drawing function for arcs larger than 180 degrees
*/
void CircularArc::draw2(float z)
{
	double theta0 = atan2(this->n0().y(), this->n0().x());
	double theta1 = atan2(this->n1().y(), this->n1().x());
	if (this->ccw)
		while (theta1 < theta0) theta1 += 2 * PI;
	else
		while (theta1 > theta0) theta1 -= 2 * PI;
	


	//glBegin(GL_POINTS);
	//glVertex2dv(x[0].P);
	//glVertex2dv(x[1].P);
	//glEnd();
	if (z == 0.0f)
	{
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i <= RES; i++) {
			auto theta = theta0 + (theta1 - theta0) * i / RES;
			glVertex2dv((this->c.c + this->c.r * Point(cos(theta), sin(theta))).P);
		}
		glEnd();
	}
	else
	{
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i <= RES; i++) {
			auto theta = theta0 + (theta1 - theta0) * i / RES;
			auto temp = (this->c.c + this->c.r * Point(cos(theta), sin(theta)));
			glVertex3d(temp.x(), temp.y(), z);
		}
		glEnd();

	}
}


/*!
*	\brief Bezier Curve의 t점에서의 Geometry를 생성하는 생성자
*/
Geometry::Geometry(BezierCrv & Crv, double t)
{
	if (Crv.PType == 0)
		std::cout << "error: Geometry cannot generated as dimension of Crv is less than 2" << std::endl;
	else {
		x = Crv(t);
		v = diff(Crv)(t);
		a = diff(diff(Crv))(t);
		n = v.rotate().normalize();
		r = 1 / Crv.curvature(t);
		e = x + n * r;
	}
}

/*!
*	\brief Circular Arc의 t점에서의 Geometry를 생성하는 생성자
*/
Geometry::Geometry(CircularArc & a, double t)
{
	x = a(t);
	e = a.c.c;
	//곡률의 중심을 향하는 방향
	n = (e - x).normalize();
	r = a.c.r;
	if (!counterclockwise(a.c.c[0] - e, a.c.c[1] - e)) {
		r = -r;
		n = -n;
	}
	v = -n.rotate();
}

/*!
*	\brief 소멸자
*/
Geometry::~Geometry()
{
}

/*!
*	\brief Geometry에서의 Osculating Circle을 만듬
*
*	\return Geometry 점에서의 Osculating Circle을 반환
*/
Circle Geometry::osculatingCircle()
{
	return Circle(e, abs(r));
}

/*!
*	\brief 생성자
*
*	\param deg 함수의 차수
*	\param ptype 함수의 차원 (0인 경우 1차원, 1인 경우 2차원)
*/
BezierCrv::BezierCrv(int deg, CtrlPtType ptype)
{
	Deg = deg;
	PType = ptype;
	P.assign(deg + 1, Point());
	endType = defaultPt;
	ccw = false;
}

/*!
*	\brief 복사 생성자
*/
BezierCrv::BezierCrv(const BezierCrv & cpy)
{
	Deg = cpy.Deg;
	PType = cpy.PType;
	P = cpy.P;
	endType = cpy.endType;
	ccw = cpy.ccw;
}

/*!
*	\brief a의 Control Point로 이루어진 Cubic Bezier Curve를 생성하는 생성자
*
*	\param a Control Points
*/
BezierCrv::BezierCrv(Point a[])
{
	Deg = 3;
	PType = CTRL_PT_E2;
	for (int i = 0; i < 4; i++)
		P.push_back(a[i]);
	endType = defaultPt;
	ccw = false;
}

/*!
*	\brief Bezier Curve를 점 p에 대하여 대칭
*
*	\param Crv 점대칭시키는 객체
*	\param Crv 점대칭점
*
*	\return Crv를 p에 대하여 대칭시킨 Bezier Curve를 반환한다.
*/
BezierCrv::BezierCrv(const BezierCrv & Crv, const Point & p)
{
	(*this) = Crv;
	for (int i = 0; i <= Deg; i++)
		P[i] = -P[i] + p;
}

/*!
*	\brief 소멸자
*/
BezierCrv::~BezierCrv()
{
}

/*!
*	\brief 대입 연산자
*
*	\param rhs 대입될 객체
*
*	\return 대입된 자신을 반환한다.
*/
BezierCrv & BezierCrv::operator=(const BezierCrv & rhs)
{
	Deg = rhs.Deg;
	PType = rhs.PType;
	P = rhs.P;
	endType = rhs.endType;
	ccw = rhs.ccw;
	return *this;
}

/*!
*	\brief Bezier Function의 값
*
*	\param t 대입하는 값
*
*	\return Bezier Function에 t를 대입한 값을 반환한다.
*/
Point BezierCrv::operator()(double t)
{
	Point pt;
	int idx = (int)(t * 10000);
	double ratio = t * 10000 - (double)idx;

	switch (Deg)
	{
	case 0:
		pt = P[0];
		break;
	case 1:
		pt = P[0] * (1 - t) + P[1] * t;
		break;

	case 2:
		pt = (1 - ratio) * (P[0] * basis2[idx][0] + P[1] * basis2[idx][1] + P[2] * basis2[idx][2]) + ratio * (P[0] * basis2[idx + 1][0] + P[1] * basis2[idx + 1][1] + P[2] * basis2[idx + 1][2]);
		break;

	case 3:
		pt = (1 - ratio) * (P[0] * basis3[idx][0] + P[1] * basis3[idx][1] + P[2] * basis3[idx][2] + P[3] * basis3[idx][3]) + ratio * (P[0] * basis3[idx + 1][0] + P[1] * basis3[idx + 1][1] + P[2] * basis3[idx + 1][2] + P[3] * basis3[idx + 1][3]);
		break;

	case 4:
		pt = (1 - ratio) * (P[0] * basis4[idx][0] + P[1] * basis4[idx][1] + P[2] * basis4[idx][2] + P[3] * basis4[idx][3] + P[4] * basis4[idx][4]) + ratio * (P[0] * basis4[idx + 1][0] + P[1] * basis4[idx + 1][1] + P[2] * basis4[idx + 1][2] + P[3] * basis4[idx + 1][3] + P[4] * basis4[idx + 1][4]);
		break;

	case 5:
		pt = (1 - ratio) * (P[0] * basis5[idx][0] + P[1] * basis5[idx][1] + P[2] * basis5[idx][2] + P[3] * basis5[idx][3] + P[4] * basis5[idx][4] + P[5] * basis5[idx][5]) + ratio * (P[0] * basis5[idx + 1][0] + P[1] * basis5[idx + 1][1] + P[2] * basis5[idx + 1][2] + P[3] * basis5[idx + 1][3] + P[4] * basis5[idx + 1][4] + P[5] * basis5[idx + 1][5]);
		break;

	case 6:
		pt = (1 - ratio) * (P[0] * basis6[idx][0] + P[1] * basis6[idx][1] + P[2] * basis6[idx][2] + P[3] * basis6[idx][3] + P[4] * basis6[idx][4] + P[5] * basis6[idx][5] + P[6] * basis6[idx][6]) + ratio * (P[0] * basis6[idx][0] + P[1] * basis6[idx + 1][1] + P[2] * basis6[idx + 1][2] + P[3] * basis6[idx + 1][3] + P[4] * basis6[idx + 1][4] + P[5] * basis6[idx + 1][5] + P[6] * basis6[idx + 1][6]);
		break;

	default:
		// deCasteljau 알고리즘으로 구하기.
		break;
	}
	return pt;
}

/*!
*	\brief Bezier Function에 실수를 곱함
*
*	\param rhs 실수배하는 값
*
*	\return Bezier Function에 rhs를 곱한 Bezier Function을 반환한다.
*/
BezierCrv BezierCrv::operator*(double rhs)
{
	BezierCrv lhs = *this;
	for (int i = 0; i <= lhs.Deg; i++)
		lhs.P[i] = lhs.P[i] * rhs;
	return lhs;
}

/*!
*	\brief Bezier Curve의 시작점에서의 접선방향 벡터
*
*	\return Bezier Curve의 시작점에서의 접선방향 벡터를 반환한다.
*/
Point BezierCrv::tangentialVector_sp()
{
	return (double)Deg * (P[1]-P[0]);
}

/*!
*	\brief Bezier Curve의 끝점에서의 접선방향 벡터
*
*	\return Bezier Curve의 끝점에서의 접선방향 벡터를 반환한다.
*/
Point BezierCrv::tangentialVector_ep()
{
	return (double)Deg * (P[Deg] - P[Deg - 1]);
}

/*!
*	\brief Bezier Function을 두 개로 나눔
*
*	\param t 나누는 값
*
*	\return 하나의 Bezier Function을 t를 기준으로 두 개의 Bezier Function으로 나눈다.
*/
std::pair<BezierCrv, BezierCrv> BezierCrv::subDiv(double t)
{
	std::pair<BezierCrv, BezierCrv> reval = { BezierCrv(Deg, PType) , BezierCrv(Deg, PType) };
	auto temp = *this;
	//처음 i개의 Point만으로 이루어진 Bezier Function에 t를 대입한 값: 나누어진 Function의 i번째 Control Point
	for (int i = 0; i <= Deg; i++) {
		temp.Deg = i;
		reval.first.P[i] = temp(t);
	}

	temp.reverse();
	//끝으로부터 i개의 Point만으로 이루어진 Bezier Function에 t를 대입한 값: 나누어진 Function의 i번째 Control Point
	for (int i = 0; i <= Deg; i++) {
		temp.Deg = i;
		reval.second.P[Deg - i] = temp(1 - t);
	}

	if (Deg > 2)
		reval.first.P[Deg] = reval.second.P[0] = reval.first.P[Deg - 1] * (1 - t) + reval.second.P[1] * t;

	reval.first.ccw = reval.second.ccw = ccw;

	return reval;
}

/*!
*	\brief Bezier Curve의 곡률
*
*	\param t 곡률을 구하는 점의 위치
*
*	\return Bezier Curve의 t에 해당하는 점에서의 곡률 값을 반환한다.
*/
double BezierCrv::curvature(double t)
{
	Point v = diff(*this)(t);
	Point a = diff(diff(*this))(t);
	return v ^ a / (v.length() * sqrt(v.length()));
}

/*!
*	\brief Bezier Curve를 Bi Circular Arc로 근사했을 때 Error Bound EPSILON / 2.0를 만족하는지를 확인
*
*	\param bs 
*
*	\return Bezier Function에 rhs를 곱한 Bezier Function을 반환한다.
*/
bool BezierCrv::isSatisfyingErrorBound()
{
	Line bs = bisector(P.front(), P.back());
	Line initNormal = shoot(P.front(), (P[1] - P[0]).rotate());
	Line endNormal = shoot(P.back(), (P[Deg] - P[Deg - 1]).rotate());
	Point initCircleCenter(initNormal, bs);
	Point endCircleCenter(endNormal, bs);
	double initRadius = sqrt(distance(initCircleCenter, P.front()));
	double endRadius = sqrt(distance(endCircleCenter, P.back()));
	double centerdiff = sqrt(distance(initCircleCenter, endCircleCenter));
	if ((abs(initRadius - endRadius + centerdiff) < EPSILON / 2.0) || (abs(endRadius - initRadius + centerdiff) < EPSILON / 2.0))
		return true;
	else if ((abs(initRadius - endRadius + centerdiff) < EPSILON / 1.4) || (abs(endRadius - initRadius + centerdiff) < EPSILON / 1.4)) {
		auto x = BiArc();
		bool a1 = x.first.c.r < x.second.c.r;
		bool a2 = initRadius < endRadius;
		if (a1 && a2) {
			if (abs(x.first.c.r + sqrt(distance(endCircleCenter, x.first.c.c)) - endRadius < EPSILON / 2.0) && abs(-x.second.c.r + sqrt(distance(initCircleCenter, x.second.c.c)) + initRadius < EPSILON / 2.0))
				return true;
		}
		else if (a1 && (!a2)) {
			if (abs(x.first.c.r + sqrt(distance(initCircleCenter, x.first.c.c)) - initRadius < EPSILON / 2.0) && abs(-x.second.c.r + sqrt(distance(endCircleCenter, x.second.c.c)) + endRadius < EPSILON / 2.0))
				return true;
		}
		else if ((!a1) && a2) {
			if (abs(x.second.c.r + sqrt(distance(endCircleCenter, x.second.c.c)) - endRadius < EPSILON / 2.0) && abs(-x.first.c.r + sqrt(distance(initCircleCenter, x.first.c.c)) + initRadius < EPSILON / 2.0))
				return true;
		}
		else {
			if (abs(-x.first.c.r + sqrt(distance(endCircleCenter, x.first.c.c)) + endRadius < EPSILON / 2.0) && abs(x.second.c.r + sqrt(distance(initCircleCenter, x.second.c.c)) - initRadius < EPSILON / 2.0))
				return true;
		}
		return false;
	}
	else if (isSatisfyingErrorBoundBilens()){
		return true;
	}
	else
		return false;

}

bool BezierCrv::isSatisfyingErrorBoundBilens()
{
	Point jc(bisector(P[0], P[3]), bisector(P[0] + (P[1] - P[0]).normalize(), P[3] + (P[3] - P[2]).normalize()));
	Circle J(jc, sqrt(distance(jc, P[0])));

	Geometry g0(*this, 0.0);
	Geometry g1(*this, 1.0);

	auto ic = intersection_self(J, g0.osculatingCircle());
	auto oc = intersection_self(J, g1.osculatingCircle());

	Point its1, its2;
	its1 = (distance(std::get<0>(ic), g0.x) < distance(std::get<1>(ic), g0.x)) ? std::get<1>(ic) : std::get<0>(ic);
	its2 = (distance(std::get<0>(oc), g1.x) < distance(std::get<1>(oc), g1.x)) ? std::get<1>(oc) : std::get<0>(oc);

	CircularArc c12(g1.x, its1, g1.v);
	CircularArc c21(g0.x, its2, g0.v);

	auto ba = this->BiArc();

	double error1 = (ba.first.c.r > c12.c.r) ? c12.c.r + sqrt(distance(c12.c.c, ba.first.c.c)) - ba.first.c.r : ba.first.c.r + sqrt(distance(c12.c.c, ba.first.c.c)) - c12.c.r;
	double error2 = (ba.second.c.r > c21.c.r) ? c21.c.r + sqrt(distance(c21.c.c, ba.second.c.c)) - ba.second.c.r : ba.second.c.r + sqrt(distance(c21.c.c, ba.second.c.c)) - c21.c.r;

	return (error1 < (EPSILON / 2.0)) && (error2 < (EPSILON / 2.0));
}



/*!
*	\brief Bezier Function의 Control Point의 순서를 뒤집음 (0 부터 1 까지의 t로 매개화 된 것을 1 부터 0 까지로 뒤집음)
*/
BezierCrv &BezierCrv::reverse()
{
	for (int i = 0; i <= Deg / 2; i++)
		std::swap(P[i], P[Deg - i]);
	if (ccw)
		ccw = false;
	else
		ccw = true;
	return *this;
}

/*!
*	\brief Bezier Function의 차수를 Reduction
*/
BezierCrv & BezierCrv::reduceControlPt()
{
	if (Deg < 2) {
		std::cout << "error: cannot reduce Control Points as the number of Control Points are not enough" << std::endl;
		return *this;
	}
	switch (PType) {
	case 0:
		for (int i = 0; i < Deg; i++)
			P[i].P[0] = P[i + 1].P[0] * (double)i / (double)(Deg - 1) + P[i].P[0] * (1 - (double)i / (double)(Deg - 1));
		Deg--;
		P.pop_back();
		break;
	case 1:
		for (int i = 0; i < Deg; i++)
			P[i] = P[i + 1] * (double)i / (double)(Deg - 1) + P[i] * (1 - (double)i / (double)(Deg - 1));
		Deg--;
		P.pop_back();
		break;
	}
	return *this;
}

/*!
*	\brief Bezier Function의 Control Point의 순서를 뒤집음 (0 부터 1 까지의 t로 매개화 된 것을 1 부터 0 까지로 뒤집음)
*/
std::vector<double> BezierCrv::solve(double i, double e)
{
	if (PType == 1) {
		std::cout << "error: 2 Dimension Bezier Function cannot be solved" << std::endl;
		return{};
	}

	bool temp = false;
	for (int i = 0; (i < Deg) && !temp; i++) {
		if (((P[i].P[0] > 0) && (P[i + 1].P[0] < 0)) || ((P[i].P[0] < 0) && (P[i + 1].P[0] > 0)))
			temp = true;
	}

	if (temp) {
		if (((e - i) < N_PRESCISION)) {
			double k = -P[0].P[0] / (P[Deg].P[0] - P[0].P[0]);
			if (!isfinite(k) || k < 0 || 1 < k)
				k = .5;
			return std::vector<double>(1, e * k + (1 - k) * i);
		}

		auto c = subDiv();
		double m = (i + e) / 2;
		std::vector<double> s1 = c.first.solve(i, m);
		std::vector<double> s2 = c.second.solve(m, e);
		s1.insert(s1.end(), s2.begin(), s2.end());
		return s1;
	}
	return{};
}


/*!
*	\brief Bezier Curve위의 특정 기울기를 가진 점을 반환
*
*	\param normal 구하는 기울기 벡터와 수직인 벡터
*
*	\return normal과 수직인 Curve 위의 점을 Curve를 매개화한 값(0과 1 사이)을 기준으로 반환한다.
*/
std::vector<double> BezierCrv::solvePararell(Vector & normal)
{
	auto v = diff(*this);
	v.PType = CTRL_PT_E1;
	for (int i = 0; i <= v.Deg; i++) {
		v.P[i].P[0] = v.P[i].P[0] * normal[0] + v.P[i].P[1] * normal[1];
	}

	return v.solve();
}

/*!
*	\brief Bezier Curve의 x방향 Bezier Function을 반환
*/
BezierCrv BezierCrv::getX()
{
	if (Deg == 0) {
		std::cout << "error: Bezier Function is 1 Dimensinal Function" << std::endl;
		return *this;
	}
	BezierCrv reval(Deg, CTRL_PT_E1);
	for (int i = 0; i <= Deg; i++)
		reval.P[i].P[0] = P[i].P[0];
	return reval;
}

/*!
*	\brief Bezier Curve의 y방향 Bezier Function을 반환
*/
BezierCrv BezierCrv::getY()
{
	if (Deg == 0) {
		std::cout << "error: Bezier Function is 1 Dimensinal Function" << std::endl;
		return *this;
	}
	BezierCrv reval(Deg, CTRL_PT_E1);
	for (int i = 0; i <= Deg; i++)
		reval.P[i].P[0] = P[i].P[1];
	return reval;
}


/*!
*	\brief 두 Bezier Curve의 충돌 조사
*
*	\param rhs 충돌 조사하는 객체
*
*	\return 자신과 rhs의 aabbtest를 하여 rhs와 충돌하면 true를 반환한다.
*/
bool BezierCrv::aabbtest(BezierCrv & rhs)
{
	double width = abs(P[0].P[0] - P[Deg].P[0]) + abs(rhs.P[0].P[0] - rhs.P[Deg].P[0]);
	double wDiff = abs((P[0].P[0] + P[Deg].P[0]) - (rhs.P[0].P[0] + rhs.P[Deg].P[0]));
	if (width + N_PRESCISION < wDiff)
		return false;
	double height = abs(P[0].P[1] - P[Deg].P[1]) + abs(rhs.P[0].P[1] - rhs.P[Deg].P[1]);
	double hDiff = abs((P[0].P[1] + P[Deg].P[1]) - (rhs.P[0].P[1] + rhs.P[Deg].P[1]));
	if (height + N_PRESCISION < hDiff)
		return false;
	return true;
}

/*!
*	\brief rhs를 p에 대하여 점대칭한 Bezier Curve와의 충돌을 조사
*
*	\param rhs 점대칭하고 충돌여부를 판단하는 Bezier Curve
*	\param p 점대칭점
*
*	\return 자신과 rhs를 p에 대하여 점대칭한 Bezier Curve를 aabbtest를 한 뒤 충돌하면 true를 반환한다.
*/
bool BezierCrv::aabbtest(BezierCrv & rhs, Point & p)
{
	double width = abs(P[0].P[0] - P[Deg].P[0]) + abs(rhs.P[0].P[0] - rhs.P[Deg].P[0]);
	double wDiff = abs((P[0].P[0] + P[Deg].P[0]) - (2 * p.P[0] - rhs.P[0].P[0] - rhs.P[Deg].P[0]));
	if (width + N_PRESCISION < wDiff)
		return false;
	double height = abs(P[0].P[1] - P[Deg].P[1]) + abs(rhs.P[0].P[1] - rhs.P[Deg].P[1]);
	double hDiff = abs((P[0].P[1] + P[Deg].P[1]) - (2 * p.P[1] - rhs.P[0].P[1] - rhs.P[Deg].P[1]));
	if (height + N_PRESCISION < hDiff)
		return false;
	return true;
}

/*!
*	\brief Spiral Bezier Curve를 두 개의 Circular Arc로 근사
*
*	\return Bezier Curve를 두 개의 Circular Arc로 근사한 뒤 이를 pair로 묶어 반환한다.
*/
std::pair<CircularArc, CircularArc> BezierCrv::BiArc()
{
	auto tangent = diff(*this);
	auto initTangent = tangent(0.0), endTangent = -tangent(1.0);
	auto init = (*this)(0.0), end = (*this)(1.0);
	Point x(shoot(init, initTangent), shoot(end, endTangent)); // intersection of tangent lines @ endpoints

	double chord = sqrt(distance((*this)(0.0), (*this)(1.0))); //length chord
	double a = sqrt(distance(init, x)), b = sqrt(distance(end, x)); //x-endpoint dist
	
	double w = a / (a + b);
	Point temp = w * end + (1 - w) * init; // some point on chord
	
	double t = (a + b) / (a + b + chord);
	Point ip = temp * t + x * (1 - t); // some point on x-temp

	return std::make_pair(CircularArc(init, ip, initTangent), CircularArc(ip, end, P[Deg] - P[0]));
}

/*!
*	\brief Bezier Curve를 20개의 Line Segment로 그림
*/
void BezierCrv::draw()
{
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i <= 20; i++) {
		glVertex2dv((*this)(i / 20.0).P);
	}
	glEnd();
}

/*!
*	\brief Bezier Curve를 x, y, curvature extreme point와 inflection point에서 나눔
*
*	\param a 나누어진 Bezier Curve Segment를 저장
*
*	\return Bezier Curve를 나누어 각각을 a에 저장한다.
*/
void BezierCrv::segmentation(std::vector<BezierCrv> &a)
{
	std::vector<Segment> divPts;

	BezierCrv e = diff(*this), ee = diff(e), eee = diff(ee);
	divPts.push_back(Segment(0.0));

	//x, y extreame
	std::vector<double> xev = e.getX().solve(), yev = e.getY().solve();
	for (int i = 0; i < (int)xev.size(); i++)
		if ((xev[i] > N_PRESCISION) && (xev[i] < (1.0 - N_PRESCISION)))
			divPts.push_back(Segment(xev[i], xyExtremePt));
	for (int i = 0; i < (int)yev.size(); i++)
		if ((yev[i] > N_PRESCISION) && (yev[i] < (1.0 - N_PRESCISION)))
			divPts.push_back(Segment(yev[i], xyExtremePt));

	//inflection
	std::vector<double> ifp = (e ^ ee).solve();
	std::vector<double> ifpv;
	for (int i = 0; i < (int)ifp.size(); i++)
		if ((ifp[i] > N_PRESCISION) && (ifp[i] < (1.0 - N_PRESCISION))) {
			divPts.push_back(Segment(ifp[i], inflectionPt));
		}

	//curvature extreame
	std::vector<double> cxp = ((e * e) * (e ^ eee) - (e ^ ee) * (e * ee) * 3.0).solve();
	std::vector<double> cxpv;
	for (int i = 0; i < (int)cxp.size(); i++)
	if ((cxp[i] > N_PRESCISION) && (cxp[i] < (1.0 - N_PRESCISION))) {
	divPts.push_back(Segment(cxp[i],curvatureExtremePt));
	}

	//sorting values
	std::sort(divPts.begin() + 1, divPts.end());
	for (int i = (int)divPts.size() - 1; i > 0; i--)
		if (divPts[i].value - divPts[i - 1].value < N_HIGH_PRESCISION)
			divPts.erase(divPts.begin() + i);
	if (1.0 - divPts.back().value < N_HIGH_PRESCISION)
		divPts.pop_back();
	divPts.push_back(Segment(1.0));

	BezierCrv temp = *this;
	for (int i = 1; i < (int)divPts.size(); i++) {
		double repara = (divPts[i].value - divPts[i - 1].value) / (1.0 - divPts[i - 1].value);
		auto s = temp.subDiv(repara);
		s.first.endType = divPts[i].St;
		a.push_back(s.first);
		temp = s.second;
	}
	
}

/*!
*	\brief Bezier Curve를 split할 때 발생할 수 있는 Inflection Point의 오차를 해결하기 위해 다시 나눔
*
*	\return Inflection Point가 Bezier Curve 내부에 포함되어 있는 경우, 이를 다시 split하여 반환한다.
*/
std::vector<BezierCrv> BezierCrv::integrityTest()
{
	if (PType == 0) {
		std::cout << "error: Integrity test is only for 2 Dimension Bezier Function" << std::endl;
		return {};
	}
	else {
		if (counterclockwise(P[1] - P[0], P[Deg] - P[0]) == counterclockwise(P[Deg] - P[Deg - 1], P[Deg] - P[0]))
		{
			BezierCrv e = diff(*this), ee = diff(e), eee = diff(ee);
			std::vector<double> ifp = (e ^ ee).solve();
			for (int i = (int)ifp.size() - 1; i >= 0; i--)
				if ((ifp[i] < N_PRESCISION) || (ifp[i] > (1.0 - N_PRESCISION)))
					ifp.erase(ifp.begin() + i);
			if (ifp.size() != 0) {
				std::vector<BezierCrv> reval;
				ifp.push_back(0.0);
				std::sort(ifp.begin(), ifp.end());
				auto temp = (*this);
				for (int i = 1; i < (int)ifp.size(); i++) {
					double repara = (ifp[i] - ifp[i - 1]) / (1.0 - ifp[i - 1]);
					auto s = temp.subDiv(repara);
					if (i != 1)
						reval.push_back(s.first);
					else
						(*this) = s.first;
					temp = s.second;
				}
				reval.push_back(temp);
				return reval;
			}
			else
				return {};
		}
		else
			return{};
	}
}

/*!
*	\brief Bezier Curve의 BVH에 의하여 Heap에 동적으로 할당되어있는 Memory를 해제
*/
void BezierCrv::freeMemory()
{
	if (child[0] != NULL) {
		child[0]->freeMemory();
		delete child[0];
		child[1]->freeMemory();
		delete child[1];
	}
}

/*!
*	\brief Bezier Curve를 split하는 매개화된 위치를 생성하는 생성자
*/
Segment::Segment(double _value)
{
	value = _value;
	St = defaultPt;

}

/*!
*	\brief Bezier Curve를 split하는 매개화된 위치를 생성하는 생성자
*/
Segment::Segment(double _value, SegmentTypes _St)
{
	value = _value;
	St = _St;
}

/*!
*	\brief 소멸자
*/
Segment::~Segment()
{
}


/*!
*	\brief 기본 생성자
*/
ArcSpline::ArcSpline()
{
	neighborDist[0] = DBL_MAX;
	neighborDist[1] = DBL_MAX;
}


/*!
*	\brief Bezier Curve를 EPSILON / 2.0의 오차를 갖는 Arc Spline Segment로 근사하여 Arc Spline을 생성하는 생성자
*/
ArcSpline::ArcSpline(BezierCrv & Crv)
{
	ccw = counterclockwise(Crv.P[1] - Crv.P[0], Crv.P[3] - Crv.P[2]);

	std::vector<BezierCrv*> q;
	q.push_back(&Crv);

	//subdiv until it reaches error-bound, so one's who reach EB first is inserted first...? but this is a stack... so order is fine...
	while (!q.empty()) {
		auto x = q.back();
		q.pop_back();
		if (x->isSatisfyingErrorBound() ||(x->P[0] == x->P[3])) {
			x->child[0] = NULL;
			x->child[1] = NULL;
			auto input = x->BiArc();
			Arcs.push_back(input.first);
			x->Arcs.first = input.first;
			Arcs.push_back(input.second);
			x->Arcs.second = input.second;

		}
		else {
			x->child[0] = new BezierCrv;
			x->child[1] = new BezierCrv;
			std::pair<BezierCrv, BezierCrv> input = x->subDiv();
			*(x->child[0]) = input.first;
			*(x->child[1]) = input.second;
			q.push_back(x->child[1]);
			q.push_back(x->child[0]);
		}
	}

	// this functions is called after monotone-seg... so it will be on a quadrant. decide it by normal's direction
	Vector test = Arcs.front().n[0] + Arcs.back().n[1];
	xQuardrants = (test.P[0] > 0);
	yQuardrants = (test.P[1] > 0);
	n[0] = ccw ? Arcs.front().n[0] : Arcs.back().n[0];
	n[1] = ccw ? Arcs.back().n[1] : Arcs.front().n[1];
}


/*!
*	\brief Arc Spline의 앞뒤를 잘라냄
*
*	\param originCrv 자르기 전의 Arc Spline Segment
*	\param begin 자르는 점
*	\param end 자르는 점
*
*	\return originCrv에서 begin과 end 사이만 남기고 나머지는 지운 Arc Spline을 반환한다.
*/
ArcSpline::ArcSpline(ArcSpline & originCrv, dividePts & begin, dividePts & end)
{
	Arcs.insert(this->Arcs.end(), originCrv.Arcs.begin() + begin.idx, originCrv.Arcs.begin() + end.idx + 1);
	Arcs.front().x[0] = begin.dividePt;
	Arcs.back().x[1] = end.dividePt;
	splited = false;
}


/*!
*	\brief 소멸자
*/
ArcSpline::~ArcSpline()
{
}

/*!
*	\brief Arc Spline Segment가 특정 Normal을 포함하는지를 판단
*
*	\param norm 비교하는 벡터의 방향
*
*	\return norm을 Arc Spline Segment가 포함하면 true를 반환한다.

True is returned when norm is n0 or n1
*/
bool ArcSpline::contain(Vector &norm)
{
	return counterclockwise(n[0], norm) && counterclockwise(norm, n[1]);
}

/*!
*	\brief Arc Spline의 시작점을 반환
*/
Point& ArcSpline::init()
{
	return Arcs.front().x[0];
}

/*!
*	\brief Arc Spline의 끝점을 반환
*/
Point& ArcSpline::end()
{
	return Arcs.back().x[1];
}

/*!
*	\brief Arc Spline의 중간점을 반환
*/
Point & ArcSpline::mid()
{
	int idx = (int)Arcs.size() / 2;
	return Arcs[idx].x[0];
}

/*!
*	\brief Arc Spline의 각 Circular Arc를 그림
*/
void ArcSpline::draw()
{
	for (int i = 0; i < (int)Arcs.size(); i++)
		Arcs[i].draw();
}

/*!
*	\brief Arc Spline Segment의 Circular Arc 중 Normal을 포함하는 Circular Arc의 Index를 반환
*
*	\param norm 주어진 Normal의 방향벡터
*
*	\return Arc Spline Segment의 Circular Arc 중 Normal을 포함하는 Circular Arc의 Index를 반환한다.
*/
int ArcSpline::findIdx(Vector &norm)
{
	int i = ccw ? 0 : (int)Arcs.size();
	int e = ccw ? (int)Arcs.size() : 0;
	int m = (i + e) / 2;
	while ((i != m)&&(e != m)) {
		if (counterclockwise(Arcs[m].n[0], norm) && (counterclockwise(norm, Arcs[m].n[1])))
			return m;
		bool test = (counterclockwise(Arcs[m].n[1], norm));
		if (test) {
			i = m;
			m = (m + e) / 2;
		}
		else {
			e = m;
			m = (i + m) / 2;
		}
	}
	return m;
}

/*!
*	\brief Arc Spline Segment가 Model의 Self Intersection 을 가질 때, 내부로 파고드는 부분을 Trimming
*
*	\return Arc Spline Segment가 Model의 Self Intersection 을 가질 때, 내부로 파고드는 부분을 Trimming한다.
*/
void ArcSpline::finalTrimming_Simple()
{
	referenced = true;
	splited = false;

	if (intersections[0].cutAfterPt) {
		Arcs[intersections[0].idx].x[1] = intersections[0].dividePt;
		Arcs.erase(Arcs.begin() + intersections[0].idx + 1, Arcs.end());
	}

	else {
		Arcs[intersections[0].idx].x[0] = intersections[0].dividePt;
		Arcs.erase(Arcs.begin(), Arcs.begin() + intersections[0].idx);
	}

	if (_neighbor[0])
		(*neighbor[0]).finalTrimmingSuccessive(relativePosition[0], !intersections[0].cutAfterPt);
	if (_neighbor[1])
		(*neighbor[1]).finalTrimmingSuccessive(relativePosition[1], intersections[0].cutAfterPt);
}

/*!
*	\brief 내부에 포함되어 Boundary를 형성하지 않는 것이 확실한 Arc Spline Segment를 지워나감
*
*	\param relPos 지워나가는 방향
*	\param cut true
*
*	\return 내부로 파고드는 Arc Spline Segment를 지운다

Def : recursively trim neighbors without intersections
*/
void ArcSpline::finalTrimmingSuccessive(bool relPos, bool cut)
{
	if (cut) {
		if (intersections.size() == 0) {
			referenced = true;
			Arcs.clear();
			if (_neighbor[!relPos])
				(*neighbor[!relPos]).finalTrimmingSuccessive(relativePosition[!relPos], true);
		}
	}
}

/*!
*	\brief 하나의 Arc Spline Segment가 한 개 이상의 Self Intersection Point를 갖는 경우의 Trimming
*
*	\return 여러 개의 Self Intersection Point가 존재하는 경우, 내부로 파고들어 지워지는 것이 확실한 부분들을 지워나간다.

Def : given intersections, trim useless arcs
*/
std::vector<ArcSpline> ArcSpline::finalTrimming_Complex()
{
	referenced = true;
	
	std::sort(intersections.begin(), intersections.end());
	std::vector<ArcSpline> reval;

	// 1. simplify cutAfterPts... => if two consecutive intersections share the same cutAfterPt, erase one.
	for (int i = (int)intersections.size() - 1; i > 0; i--) {
		if (intersections[i].cutAfterPt && intersections[i - 1].cutAfterPt)
			intersections.erase(intersections.begin() + i);
		else if ((!intersections[i].cutAfterPt) && (!intersections[i - 1].cutAfterPt))
			intersections.erase(intersections.begin() + i - 1);
	}

	// 2. evaluate simplified_intersections
	// 2-1. simple case (only one intersection) => simply trim half of the arc
	if (intersections.size() == 1) {

		// 2-1-1. trim half.
		if (intersections[0].cutAfterPt) {
			Arcs[intersections[0].idx].x[1] = intersections[0].dividePt;
			Arcs.erase(Arcs.begin() + intersections[0].idx + 1, Arcs.end());
		}
		else {
			Arcs[intersections[0].idx].x[0] = intersections[0].dividePt;
			Arcs.erase(Arcs.begin(), Arcs.begin() + intersections[0].idx);
		}

		// 2-1-2. recursively destroy neighbors without intersections. (if nearest cut is trimmed)
		if (_neighbor[0])
			(*neighbor[0]).finalTrimmingSuccessive(relativePosition[0], !intersections[0].cutAfterPt);
		if (_neighbor[1])
			(*neighbor[1]).finalTrimmingSuccessive(relativePosition[1], intersections[0].cutAfterPt);
		reval.push_back(*this);
		splited = true;
		return reval;
	}
	// 2-2. complex case (inter >= 2)
	else {
		
		// 2-2-1. recursively destroy neighbors without intersections. (if nearest cut is trimmed)
		if (_neighbor[0])
			(*neighbor[0]).finalTrimmingSuccessive(relativePosition[0], !intersections.front().cutAfterPt);
		if (_neighbor[1])
			(*neighbor[1]).finalTrimmingSuccessive(relativePosition[1], intersections.back().cutAfterPt);

		// 2-2-2. Judge (start, intersection0)
		if (intersections.front().cutAfterPt) {
			auto x = *this;
			x.Arcs[intersections.front().idx].x[1] = intersections.front().dividePt;
			x.Arcs.erase(x.Arcs.begin() + intersections.front().idx + 1, x.Arcs.end());
			reval.push_back(x);
		}

		// 2-2-3. Judge (intersection_n, intersection_n+1)
		for (int i = 0; i < (int)intersections.size() - 1; i++)
			if (!intersections[i].cutAfterPt) {
				reval.push_back(ArcSpline(*this, intersections[i], intersections[i + 1]));
			}

		// 2-2-4. Judge (intersection_last, end)
		if (!intersections.back().cutAfterPt) {
			auto x = *this;
			x.Arcs[intersections.back().idx].x[0] = intersections.back().dividePt;
			x.Arcs.erase(x.Arcs.begin(), x.Arcs.begin() + intersections.back().idx);
			reval.push_back(x);
		}
		splited = true;
		return reval;
	}

}

/*!
*	\brief Numerical Error에 의하여 Arc Spline Segment에 발생한 오차를 보정
*
*	\return Arc Spline Segment의 Normal을 다듬고, Circular Arc의 방향이 뒤집힌 경우 Arc Spline Segment를 나누어 반환한다.
*/
std::vector<ArcSpline> ArcSpline::integrityTest()
{
	std::vector<ArcSpline> reval;
	if (Arcs.size() == 0)
		return std::vector<ArcSpline>();

	if (Arcs.front().n[1] * Arcs.back().n[0] < 0) {
		reval.resize(2);
		bool flip = false; // from the point turned true, arcs go inside reval[1]

		for (int i = 0; i < (int)Arcs.size(); i++) { // i guess, the differnce in normal's quadrant (assuming arc is in one quad) mean's diff in cw/ccw?
			Arcs[i].refineNormal(); // just changes normals that are : very close to (1,0) or (0,1)
			if (Arcs[i].x[0].close(Arcs[i].x[1])) { // arcs with epsilon length
				Arcs.erase(Arcs.begin() + i);
				i--;
			}
			else if (!flip) {
				reval[0].Arcs.push_back(Arcs[i]);
				if (i + 1 != (int)Arcs.size() && (Arcs[i].n[1] * Arcs[i + 1].n[0] < 0))
					flip = true;
			}
			else {
				reval[1].Arcs.push_back(Arcs[i]);
			}
		}

		if (reval[1].Arcs.size() != 0) {
			Vector test = reval[1].Arcs.front().n[0] + reval[1].Arcs.back().n[1];
			reval[1].xQuardrants = (test.P[0] > 0);
			reval[1].yQuardrants = (test.P[1] > 0);
			reval[1].ccw = reval[1].Arcs.front().ccw;
			reval[1].n[0] = reval[1].ccw ? reval[1].Arcs.front().n[0] : reval[1].Arcs.back().n[0];
			reval[1].n[1] = reval[1].ccw ? reval[1].Arcs.back().n[1] : reval[1].Arcs.front().n[1];
		}
		else
			reval.pop_back();

		if (reval[0].Arcs.size() != 0) {
			Vector test = reval[0].Arcs.front().n[0] + reval[0].Arcs.back().n[1];
			reval[0].xQuardrants = (test.P[0] > 0);
			reval[0].yQuardrants = (test.P[1] > 0);
			reval[0].ccw = reval[0].Arcs.front().ccw;
			reval[0].n[0] = reval[0].ccw ? reval[0].Arcs.front().n[0] : reval[0].Arcs.back().n[0];
			reval[0].n[1] = reval[0].ccw ? reval[0].Arcs.back().n[1] : reval[0].Arcs.front().n[1];
		}
		else
			reval.erase(reval.begin());
	}
	else
		reval.push_back(*this);
	return reval;
}


/*!
*	\brief Arc Spline Segment를 감싸는 BCA Bounding Volume를 생성하는 생성자
*
*	\param s 감싸는 Arc Spline Segment - Spiral 가정이 필요하다
*/
BCA::BCA(ArcSpline & s)
{
	x[0] = s.Arcs.front().x[0];
	x[1] = s.Arcs.back().x[1];
	Point t[2];
	t[0] = (s.Arcs.front().x[0] - s.Arcs.front().c.c).rotate();
	t[1] = -(s.Arcs.back().x[1] - s.Arcs.back().c.c).rotate();
	outer = CircularArc(x[0], x[1], t[0]);
	inner = -CircularArc(x[1], x[0], t[1]);
	if (inner < outer)
		std::swap(inner, outer);
}

/*!
*	\brief Arc Spline Segment의 일부를 감싸는 BCA Bounding Volume를 생성하는 생성자
*
*	\param s 감싸는 Arc Spline Segment - Spiral 가정이 필요하다
*	\param Idx Arc Spline Segment중 BCA를 만드는 Arc의 Index 범위
*/
BCA::BCA(ArcSpline & s, std::pair<int, int>& Idx)
{
	x[0] = s.Arcs[Idx.first].x[0];
	x[1] = s.Arcs[Idx.second].x[1];
	Point t[2];
	t[0] = (s.Arcs[Idx.first].x[0] - s.Arcs[Idx.first].c.c).rotate();
	t[1] = -(s.Arcs[Idx.second].x[1] - s.Arcs[Idx.second].c.c).rotate();
	outer = CircularArc(x[0], x[1], t[0]);
	inner = -CircularArc(x[1], x[0], t[1]);
	if (inner < outer)
		std::swap(inner, outer);
}

/*!
*	\brief Bezier Curve를 감싸는 BCA Bounding Volume를 생성하는 생성자
*
*	\param Crv Bounding하는 Bezier Curve
*/
BCA::BCA(BezierCrv & Crv)
{
	x[0] = Crv(0), x[1] = Crv(1);
	outer = CircularArc(Crv(0), Crv(1), Crv.tangentialVector_sp());
	inner = -CircularArc(Crv(1), Crv(0), -Crv.tangentialVector_ep());
	if (inner.c.r < outer.c.r)
		std::swap(outer, inner);
}

/*!
*	\brief 소멸자
*/
BCA::~BCA()
{
}

/*!
*	\brief BCA가 p를 포함하는지를 판단
*
*	\param P 포함여부를 판단하는 점
*
*	\return BCA가 점 P를 포함하면 true를 반환한다.
*/
bool BCA::contain(Point & P)
{
	return ((outer.c.contain(P)) && (!inner.c.contain(P)));
}

/*!
*	\brief BCA의 두께
*
*	\return BCA의 두께를 반환한다.
*/
double BCA::thickness()
{
	return sqrt(distance(inner.c.c, outer.c.c)) - abs(inner.c.r - outer.c.r);
}

/*!
*	\brief dividePts를 생성하는 생성자
*
*	\param _idx dividePts를 포함하는 Arc Spline의 Index
*	\param _dividePt 교점
*	\param _cutAfterPt _dividePt의 뒤쪽이 Boundary 내부를 파고들어 지워지는 부분인 경우 true, 앞쪽이 파고드는 부분인 경우 false
*/
dividePts::dividePts(int _idx, Point & _dividePt, bool _cutAfterPt)
{
	idx = _idx;
	dividePt = _dividePt;
	cutAfterPt = _cutAfterPt;
}


/*!
*	\brief Cache의 정보를 초기화
*/
void CacheCircles::reset()
{
	idx = 0;
	for (int i = 0; i < cacheSize; i++) {
		check[i] = false;
		cache[i] = NULL;
	}
}

/*!
*	\brief Grid의 생성자
*/
Grid::Grid()
{
	double diff_x = (grid_max_x - grid_min_x) / double(grid);
	double diff_y = (grid_max_y - grid_min_y) / double(grid);

	for (int i = 0; i <= grid; i++) {
		x[i] = grid_min_x + i * diff_x;
		y[i] = grid_min_y + i * diff_y;
	}

	for (int i = 0; i <= grid; i++)
		for (int j = 0; j <= grid; j++) {
			nodes[i][j] = Point(x[i], y[j]);
		}
	for (int i = 0; i < grid; i++)
		for (int j = 0; j < grid; j++) {
			cover[i][j] = false;
			coverCircle[i][j] = NULL;
		}
}

/*!
*	\brief Circle과 만나는 Grid를 찾아 Grid에 해당 Circle을 추가하여 업데이트
*
*	\param input Circle의 주소
*/
void Grid::insert(Circle *input)
{
	for (int i = 0; i < grid; i++)
		for (int j = 0; j < grid; j++) {
			short Case = 0;
			bool inserted = false;
			if (input->c.P[1] > y[j + 1]) {
				Case++;
			}
			Case <<= 1;
			if (input->c.P[1] > y[j]) {
				Case++;
			}
			Case <<= 1;
			if (input->c.P[0] > x[i + 1]) {
				Case++;
			}
			Case <<= 1;
			if (input->c.P[0] > x[i]) {
				Case++;
			}
			switch (Case) {
			case 0:
				if (distance(nodes[i][j], input->c) < input->r * input->r) {
					gCircles[i][j].push_back(input);
					inserted = true;
				}
				break;
			case 1:
				if (y[j] - input->c.P[1] < input->r) {
					gCircles[i][j].push_back(input);
					inserted = true;
				}
				break;
			case 3:
				if (distance(nodes[i + 1][j], input->c) < input->r * input->r) {
					gCircles[i][j].push_back(input);
					inserted = true;
				}
				break;
			case 4:
				if (x[i] - input->c.P[0] < input->r) {
					gCircles[i][j].push_back(input);
					inserted = true;
				}
				break;
			case 5:
				gCircles[i][j].push_back(input);
				inserted = true;
				break;
			case 7:
				if (input->c.P[0] - x[i + 1] < input->r) {
					gCircles[i][j].push_back(input);
					inserted = true;
				}
				break;
			case 12:
				if (distance(nodes[i][j + 1], input->c) < input->r * input->r) {
					gCircles[i][j].push_back(input);
					inserted = true;
				}
				break;
			case 13:
				if (input->c.P[1] - y[j + 1] < input->r) {
					gCircles[i][j].push_back(input);
					inserted = true;
				}
				break;
			case 15:
				if (distance(nodes[i + 1][j + 1], input->c) < input->r * input->r) {
					gCircles[i][j].push_back(input);
					inserted = true;
				}
				break;
			default:
				std::cout << "grid input error!" << std::endl;
			}
			if (inserted && input->contain(nodes[i][j]) && input->contain(nodes[i + 1][j]) && input->contain(nodes[i][j + 1]) && input->contain(nodes[i + 1][j + 1])) {
				cover[i][j] = true;
				coverCircle[i][j] = input;
			}
		}

}

/*!
*	\brief Circular Arc가 만나는 Grid의 Index
*
*	\param p1 첫 번째 점
*	\param p2 두 번째 점
*	\param p3 세 번째 점
*
*	\return Grid의 x, y방향 Index 네 개를 반환한다.
*/
/* reval:
 * first value: min x idx
 * second value: max x idx
 * third value: min y idx
 * forth value: max y idx
 */
std::vector<int> Grid::find(const Point & p1, const Point & p2, const Point & p3)
{
	std::vector<int> reval;
	double xmax = (p1.P[0] > p2.P[0]) ? p1.P[0] : p2.P[0];
	double xmin = (p1.P[0] < p2.P[0]) ? p1.P[0] : p2.P[0];
	double ymax = (p1.P[1] > p2.P[1]) ? p1.P[1] : p2.P[1];
	double ymin = (p1.P[1] < p2.P[1]) ? p1.P[1] : p2.P[1];
	xmax = (xmax > p3.P[0]) ? xmax : p3.P[0];
	xmin = (xmin < p3.P[0]) ? xmin : p3.P[0];
	ymax = (ymax > p3.P[1]) ? ymax : p3.P[1];
	ymin = (ymin < p3.P[1]) ? ymin : p3.P[1];

	for (int i = 0; i < grid; i++) {
		if (xmin < x[i + 1]) {
			reval.push_back(i);
			break;
		}
	}
	if (reval.size() == 0 || reval[0] < 0 || reval[0] >= grid)
	{
		throw 0;
	}
	for (int i = reval[0]; i < grid; i++) {
		if (xmax < x[i + 1]) {
			reval.push_back(i);
			break;
		}
	}
	for (int i = 0; i < grid; i++) {
		if (ymin < y[i + 1]) {
			reval.push_back(i);
			break;
		}
	}
	if (reval.size() < 3 || reval[2] < 0 || reval[2] >= grid)
	{
		std::cout << "throwing" << std::endl;
		throw 0;
	}
	for (int i = reval[2]; i < grid; i++) {
		if (ymax < y[i + 1]) {
			reval.push_back(i);
			break;
		}
	}

	if (reval.size() != 4)
		std::cout << "error occurs! - grid/find" << std::endl;

	return reval;
}

/*!
*	\brief Grid에 속한 Circle이 p1, p2, p3를 포함하는지를 판단
*
*	\param p1 첫 번째 점
*	\param p2 두 번째 점
*	\param p3 세 번째 점
*	\param x Grid의 x방향 Index
*	\param y Grid의 y방향 Index
*	\param singleGrid Circular Arc가 단 하나의 Grid에 포함되는지를 판단
*
*	\return Circular Arc를 Trimming하는 Circle의 주소와, Trimming 여부를 pair로 반환. (pair의 두 번째 값이 false인 경우 Circle*은 NULL일 수밖에 없음)
*/
std::pair<Circle*, bool> Grid::trimming(Point& p1, Point& p2, Point& p3, int x, int y, bool singleGrid)
{
	// Single Grid인 경우에만 Trimming이 가능함을 확신할 수 있음
	if ((covercheck = cover[x][y]) && singleGrid)
		return std::make_pair((Circle*)NULL, true);
	int s = (int)gCircles[x][y].size();

	
	for (int i = 0; i < s; i++)
		// 순서가 뒤집힌 이유: Sorting 할 때 반지름이 작은 것 부터 큰 것 순서로 Sorting 되어 있음!
		// Grid의 Circle 중 반지름이 큰 Circle과 먼저 비교해보기 위함!
		if (gCircles[x][y][s - i - 1]->contain_trimming(p1) && gCircles[x][y][s - i - 1]->contain_trimming(p2) && gCircles[x][y][s - i - 1]->contain_trimming(p3))
			return std::make_pair(gCircles[x][y][s - i - 1], true);
	return std::make_pair((Circle*)NULL, false);
}

}