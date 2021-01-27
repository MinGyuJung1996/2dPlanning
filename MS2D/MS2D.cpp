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
*	\brief �ʿ��� ������(B-Spline Model Data, Interior Disk Data)�� Import�ϰ�, 8���� �⺻ �𵨿� ���Ͽ� Arc Spline���� �ٻ��Ѵ�
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
*	\brief �Է¹��� �� �𵨿� ���Ͽ� Arc Spline �ٻ��ϰ�, ������ Interior Disk�� ȸ���Ͽ� �����Ѵ�
*
*	\param FirstModel ù ��° ���� Index
*	\param SecondModel �� ��° ���� Index
*/
void postProcess(int FirstModel, int SecondModel)
{
	// �� ���� ���� ������ �Ǵ��Ѵ�
	// ���� ���� ��� ����ó���� �Ͽ� �˰����� ����ȭ�ϱ� �����̴�
	if (DEBUG false /*FirstModel == SecondModel*/)
		ModelInfo_Identical = true;
	else
		ModelInfo_Identical = false;

	// Second Model�� Index�� FirstModel���� ū ���, ���� �״�� ����Ѵ�
	if (DEBUG true /*(SecondModel >= FirstModel)*/) {
		// ������ Frame�� ���ؼ� ����Ѵ�
		for (int i = 0; i < numofframe; i++) {
			
			// 1. Erase Former data

			// �� �𵨿��� ���Ǿ��� �����͸� �����ش�
			if (!Models_Rotated[i].empty())
				Models_Rotated[i].clear();

			if (!Models_Rotated_Approx[i].empty())
				Models_Rotated_Approx[i].clear();

			if (!InteriorDisks_Rotated[i].empty())
				InteriorDisks_Rotated[i].clear();

			// 2. make InteriorDisks_Rotated

			// ���� Disks�� ȸ���Ͽ� �����Ѵ�
			// �������� �״�� �ΰ� �߽ɸ� ȸ���Ѵ�
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
				// Models_Imported[FirstModel]�� �� BezierCrv�� Control Point�� ȸ���Ͽ� ȸ���� ���� ������ �����Ѵ�
				// tempF : ȸ���� �� (�Ƹ� temp Figure�� tempF��� �ߴ� �� ���ƿ�)
				auto tempF = Models_Imported[FirstModel];
				for (int j = 0; j < (int)Models_Imported[FirstModel].size(); j++)
					for (int k = 0; k <= (int)Models_Imported[FirstModel][j].Deg; k++)
						tempF[j].P[k] = tempF[j].P[k].rotate(2 * PI * i / numofframe);

				// ȸ���� ���� x, y - extreme, inflection, curvature extreme point���� �ɰ� �� �̸� Models_Rotated[i]�� �����Ѵ�
				for (int j = 0; j < (int)tempF.size(); j++)
					tempF[j].segmentation(Models_Rotated[i]);

				// ���� �Ϻ����� �׽�Ʈ�Ѵ�
				for (int j = 0; j < (int)Models_Rotated[i].size(); j++) {
					auto s = Models_Rotated[i][j].integrityTest();
					if (s.size() > 0)
						Models_Rotated[i].insert(Models_Rotated[i].begin() + j + 1, s.begin(), s.end());
				}

				// �� ���� Arc Spline���� �ٻ��Ѵ�. ������ Curvature Monotone���� �߶��� ������ Spiral���� ����ȴ�
				for (int j = 0; j < (int)Models_Rotated[i].size(); j++) {
					auto tempSpiral = ArcSpline(Models_Rotated[i][j]);
					// ���������� �������� �׽�Ʈ�Ѵ�
					auto input = tempSpiral.integrityTest();
					Models_Rotated_Approx[i].insert(Models_Rotated_Approx[i].end(), input.begin(), input.end());
				}
			}
		}
	}

	// ù ��° ���� Index�� �� ū ���, �� ��° ���� y������ �����Ѵ�
	// Minkowski Sum�� ��ȯ��Ģ�� �����ϱ� ������,
	// m + n�� n + m�� ���� ����� ��Ÿ����. ���� m > n�� ��� ���� �����Ͽ� �׷�����
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

			// Control Point�� �����ϰ� x��ǥ�� �����´�
			// Control Point�� �����ϴ� ������, y�� ��Ī�� ���� ȸ�� ������ �ٲ�� �����̴�
			// (���� ȸ�� ������ �ݽð� �����̾�߸� ���� ���� �ܺ� ������ �ǴܵǱ� �����̴�)
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
*	\brief File Descriptor�� �ݴ´�
*/
void save()
{
	fclose(f);
}

/*!
*	\brief ù ��° ���� frame��, �� ��° ���� figure2��° Index�� ���� ���� Minkowskisum �Ѵ�
*
*	\param frame ù ��° ��(postProcess �ܰ迡�� �̸� �ٻ��)�� ���� frame	: [0, 360)
*	\param figure2 �� ��° ��(initialize �ܰ迡�� �̸� �ٻ��)				: [0, 8)
*/
void minkowskisum(int frame, int figure2)
{
	// 1. init global & local vars
	{
		// ���� frame���� ����� ������ �ʱ�ȭ
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
			// Interior Disk�� ���
		auto input = InteriorDisks_Rotated[frame] + InteriorDisks_Imported[figure2]; // size of input = size of ID_R * ID_I;
		InteriorDisks_Convolution.insert(InteriorDisks_Convolution.end(), input.begin(), input.end());

			// Interior Disk�� �������� �����ͺ��� ū�� ������ Sorting
		std::sort(InteriorDisks_Convolution.begin(), InteriorDisks_Convolution.end());

		// 2-2. Build GRID with 2-1
			//Grid Data Structure�� ����
		for (int i = 0; i < (int)InteriorDisks_Convolution.size(); i++)
			Grid_Trimming.insert(&InteriorDisks_Convolution[i]);

		// 2-3. extract cache from 2-2.
			// ���� ū Disk�� �̾� Cache�� �̸� �־���´�
			// (Cache �ʱ�ȭ)
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
	
			// ���� ���� ù Index�� �� Index : ls[0], ls[1]
			// ������ ���� ù Index�� �� Index : rs[0], rs[1]
			int ls[2], rs[2];
			// ��� Topology�� �� Arc�� �����ִ��� Ȯ���Ѵ�
			short overlap = overlapCase(lhs, rhs);
			// �� ��쿡 ���Ͽ� ���ڸ� ���߾� Con

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
	// �� Convolution Arc Spline�� �浹���θ� ����
	for (int i = 0; i < (int)ConvolutionArcs.size() - 1; i++)
		for (int j = i + 1; j < (int)ConvolutionArcs.size(); j++)
			// connected �������� ������ Arc Spline Segment���� ���� ������
			if (!connected(ConvolutionArcs[i], ConvolutionArcs[j]) && aabbtest(ConvolutionArcs[i], ConvolutionArcs[j]))
				selfIntersectionPts(ConvolutionArcs[i], ConvolutionArcs[j]);

	// Convolution�� ������ �� Arc Spline Segment�� ���Ͽ� Self-Intersection�� ���� Trimming
	// Self Intersection�� ���� ��� : skip
	// 1�� �ִ� ��� : �߸��� �������� ���󰡸� Arc Spline Segment�� ����
	// 2�� �̻� �ִ� ��� : �� �ڼ��� Arc Spline�� ��Ȳ�� �����ϸ� Trimming 
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
				// 2�� �̻��� ������ ���� Arc Spline Segment�� �ɰ� ��(split = true) temp�� ���� ��Ƴ���
				temp.insert(temp.end(), input.begin(), input.end());
				break;
			}
	// ���� temp�� Convolution Arc �ڿ� �߰�
	ConvolutionArcs.insert(ConvolutionArcs.end(), temp.begin(), temp.end());

	// ���� ��Ȳ���� 2�� �̻��� ������ ���� �ɰ��� Arc Spline��(split = true) temp�� �ߺ��ǹǷ� ���⼭ ������
	// index�� �ڿ������� ���� -> vector�� erase�� �߻��ϴ� ���縦 �ּ�ȭ�ϱ� ����
	// �� ���� ��� �������踦 �ʱ�ȭ��
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

	// �������踦 �ٽ� ���
	for (int i = 0; i < (int)ConvolutionArcs.size() - 1; i++)
		for (int j = i + 1; j < (int)ConvolutionArcs.size(); j++) {
			connected(ConvolutionArcs[i], ConvolutionArcs[j]);
		}

	// �������踦 ���� ������ Deque �ڷᱸ���� ���� (Loop�� �����)
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
	// �� Loop�� Arc�� �� x, y�� min / max�� ���Ѵ�.
	// min�̳� max�� Arc�� �����ϴ� ���, �ݵ�� �� Loop�� Boundary���� ����ȴ�
	// min�̳� max���� ���� �ʴ� Loop�� ���, Inner Loop�̰ų�,(���ܾ� �ϴ� Loop) ���� Trimming �ܰ迡�� ������� ���� Segment(Trimming �ؾ� �ϴ� Loop)�̴�
	double xmin = DBL_MAX, ymin = DBL_MAX, xmax = -DBL_MAX, ymax = -DBL_MAX;
	int xminIdx, yminIdx, xmaxIdx, ymaxIdx;
	// x, y min / max�� Arc�� �����ϴ� Loop�� ã�´�
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

	// �� Loop�� ������ Loop�� ������ �� ���� ���� Collision Detection Test�� �����Ѵ�
	// Collision Detection�� �߻��ϸ� Trimming�Ǿ�߸� �ϴ� ������ Segment�̹Ƿ�, �����ش�
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
	// Loop�� �̷��� ���ϴ� ��� �����ش�

	// For cases where loop is of 0~2 arcsplines.
	for (int i = (int)Model_Result.size() - 1; i >= 0; i--) {
		if (Model_Result[i].size() == 0)
			Model_Result.erase(Model_Result.begin() + i);
		// ArcSpline ������ �ſ� ���� ��� ����ó��
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
	// Inner Loop�� �ִ� ��� Loop Test�� ����
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

					// Loop�� �̷��� ���ϸ� Erase
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

	// �� Loop���� x, y�� min / max���� ����� ��, Boundary�� �̷���� Inner Loop�� �̷������ �Ǵ�
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

	//------------------ Trimming Process ���� ------------------//

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
*	\brief ������
*
*	\param x x-��ǥ
*	\param y y-��ǥ
*/
Point::Point(double x, double y)
{
	P[0] = x;
	P[1] = y;
}

/*!
*	\brief ���� ������
*
*	\param cpy ����� ��ü
*/
Point::Point(const Point &cpy)
{
	P[0] = cpy.P[0];
	P[1] = cpy.P[1];
}

/*!
*	\brief �� ������ ����
*
*	\param l ù��° ����
*	\param m �ι��� ����
*/
Point::Point(Line & l, Line & m)
{
	// temp ���� ���� �� homogeneous coor.���� ����° ��ǥ(�� ������ ������ 1�� �����־�� �Ѵ�.)
	double temp = l.L[0] * m.L[1] - l.L[1] * m.L[0];
	P[0] = (l.L[1] * m.L[2] - l.L[2] * m.L[1]) / temp;
	P[1] = (l.L[2] * m.L[0] - l.L[0] * m.L[2]) / temp;
}

/*!
*	\brief CircularArc�� �ﰢ������ Bounding�� Bounding Volume�� �� ��° �� (Circular Arc�� �� �������� ���� �� ������ ����)
*
*	\param c Bounding Volume�� ����� Circular Arc
*/
Point::Point(CircularArc & c)
{
	(*this) = Point(Line(c.x[0], c.x[0] + (c.x[0] - c.c.c).rotate()), Line(c.x[1], c.x[1] + (c.x[1] - c.c.c).rotate()));
}

/*!
*	\brief �Ҹ���
*/
Point::~Point()
{
}

/*!
*	\brief �տ� ���� ����
*
*	\return �տ� ���� ������ ��ȯ�Ѵ�.
*/
Point Point::operator-() const
{
	return Point(-P[0], -P[1]);
}

/*!
*	\brief ���� ������
*
*	\param rhs ���Ե� ��ü
*
*	\return ���Ե� �ڽ��� ��ȯ�Ѵ�.
*/
Point & Point::operator=(const Point &rhs)
{
	P[0] = rhs.P[0];
	P[1] = rhs.P[1];

	return *this;
}


/*!
*	\brief �� ���� ������ �Ǵ��Ѵ�.
*
*	\param rhs ���ϴ� ��ü
*
*	\return ������ true, �ٸ��� false�� ��ȯ�Ѵ�.
*/
bool Point::operator==(Point & rhs)
{
	return (abs(P[0] - rhs.P[0]) < N_PRESCISION) && (abs(P[1] - rhs.P[1]) < N_PRESCISION);
}

/*!
*	\brief �� ������ ũ�⸦ ��
*
*	\param rhs ���ϴ� ��ü
*
*	\return rhs�� �� ũ�� true, ������ false�� ��ȯ�Ѵ�.
*/
bool Point::operator<(Point & rhs)
{
	return (*this * *this < rhs * rhs);
}

/*!
*	\brief �� ������ ũ�⸦ ��
*
*	\param rhs ���ϴ� ��ü
*
*	\return rhs�� �� ������ true, ũ�� false�� ��ȯ�Ѵ�.
*/
bool Point::operator>(Point & rhs)
{
	return (*this * *this > rhs*rhs);
}

/*!
*	\brief �� ���� ��������� �Ǵ�
*
*	\param rhs ���ϴ� ��ü
*
*	\return �Ÿ��� Ư�� �������� ������ true, ũ�� false�� ��ȯ�Ѵ�.
*/
bool Point::close(Point & rhs)
{
	return (abs(P[0] - rhs.P[0]) < N_HIGH_PRESCISION) && (abs(P[1] - rhs.P[1]) < N_HIGH_PRESCISION);
}

/*!
*	\brief �� ���� ��ġ�ϴ��� �Ǵ�
*
*	\param rhs ���ϴ� ��ü
*
*	\return �� ���� �Ϻ��� ��ġ�ϸ� true�� ��ȯ�Ѵ�.
*/
bool Point::exact(Point & rhs)
{
	return (abs(P[0] - rhs.P[0]) < 1e-10) && (abs(P[1] - rhs.P[1]) < 1e-10);
}

/*!
*	\brief ������ ������ ����
*
*	\return ������ ������ ������ ��ȯ�Ѵ�.
*/
double Point::length()
{
	return *this * *this;
}

/*!
*	\brief �������͸� �����
*
*	\return ������ ���� ũ�Ⱑ 1�� ���͸� ��ȯ�Ѵ�.
*/
Point &Point::normalize()
{
	double len = sqrt(length());
	P[0] /= len;
	P[1] /= len;

	return *this;
}

/*!
*	\brief ���͸� ȸ���Ѵ�
*
*	\return �ݽð�������� 90�� ȸ����Ų ���͸� ��ȯ�Ѵ�.
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
*	\brief ���� p���� q������ ������ �Ǵ�
*
*	\param p ù��° ��
*	\param q �ι�° ��
*
*	\return �ð�����̸� true, �ݽð�����̸� false�� ��ȯ�Ѵ�.
*/
bool counterclockwise(Point & p, Point & q)
{
	return ((p^q) >= 0.0);
}


/*!
*	\brief �� �� ������ �Ÿ�
*
*	\param p ù��° ��
*	\param q �ι�° ��
*
*	\return \a p�� \a q�� �Ÿ��� ������ ��ȯ�Ѵ�.
*/
double distance(Point &p, Point & q)
{
	return (p - q).length();
}

/*!
*	\brief �� ���� ������ ���Ͽ� ��ȯ�Ѵ�.
*
*	\param lhs ù��° ��
*	\param rhs �ι�° ��
*
*	\return \a lhs�� \a rhs�� ������ ��ȯ�Ѵ�.
*/
double operator*(Point &lhs, Point &rhs)
{
	return lhs.P[0] * rhs.P[0] + lhs.P[1] * rhs.P[1];
}

/*!
*	\brief �� ���� ������ ���Ͽ� ��ȯ�Ѵ�.
*
*	\param lhs ù��° ��
*	\param rhs �ι�° ��
*
*	\return \a lhs�� \a rhs�� ������ ��ȯ�Ѵ�.
*/
double operator^(Point & lhs, Point & rhs)
{
	return (lhs.P[0] * rhs.P[1] - lhs.P[1] * rhs.P[0]);
}

/*!
*	\brief ������ �Ǽ��踦 ��ȯ�Ѵ�
*
*	\param s �Ǽ��� ���ִ� ���� ũ��
*	\param rhs �Ǽ��� �Ǵ� ���� ��ü
*
*	\return \a rhs�� �� ������ \a s���� ���� ���� ������ ��ȯ�Ѵ�.
*/
Point operator*(double s, Point & rhs)
{
	return Point(s * rhs.P[0], s * rhs.P[1]);
}

/*!
*	\brief ������ �Ǽ���
*
*	\param s �Ǽ��� ���ִ� ���� ũ��
*	\param lhs �Ǽ��� �Ǵ� ���� ��ü
*
*	\return \a lhs�� �� ������ \a s���� ���� �������� ���� ������ ��ȯ�Ѵ�.
*/
Point operator*(Point & lhs, double s)
{
	return Point(lhs.P[0] * s, lhs.P[1] * s);
}

/*!
*	\brief ���͸� �Ǽ��� ���� ��
*
*	\param s �Ǽ��� ���ִ� ���� ũ��
*	\param lhs �Ǽ��� �Ǵ� ���� ��ü
*
*	\return \a lhs�� �� ������ \a s�� �������� �������� ���� ������ ��ȯ�Ѵ�.
*/
Point operator/(Point & lhs, double s)
{
	return lhs * (1 / s);
}

/*!
*	\brief ������ ���� ��ȯ�Ѵ�
*
*	\param lhs ù��° ����
*	\param rhs �ι�° ����
*
*	\return \a lhs�� \a rhs�� �� ������ ���� ���͸� ��ȯ�Ѵ�.
*/
Point operator+(const Point & lhs, const Point & rhs)
{
	return Point(lhs.P[0] + rhs.P[0], lhs.P[1] + rhs.P[1]);
}

/*!
*	\brief �� ������ ���� ��ȯ�Ѵ�
*
*	\param lhs ù��° ����
*	\param rhs �ι�° ����
*
*	\return \a lhs�� \a rhs�� �� ������ ���� ���� ���͸� ��ȯ�Ѵ�.
*/
Point operator-(const Point & lhs, const Point & rhs)
{
	return Point(lhs.P[0] - rhs.P[0], lhs.P[1] - rhs.P[1]);
}

/*!
*	\brief ���� ��ǥ�� ����Ѵ�
*
*	\param os cout
*	\param p ����� ��ü
*
*	\return p�� x��ǥ�� y��ǥ�� ������� ��ȯ�Ѵ�
*/
std::ostream & operator<<(std::ostream &os, const Point &p)
{
	os << "(" << p.P[0] << ", " << p.P[1] << ")";
	return os;
}

/*!
*	\brief ���� ���� ������ �Ÿ�
*
*	\param p ��
*	\param l ����
*
*	\return p�� l ������ �ִܰŸ��� ��ȯ�Ѵ�. ��ȣ�� homogeneous ��ǥ������ ������ normal��, �������� ���� �������� ������ ������ ���, �ٸ��� ������ ��Ÿ����.
*/
double distance(Line & l, Point & p)
{
	return (l.L[0] * p[0] + l.L[1] * p[1] + l.L[2]) / sqrt(l.L[0] * l.L[0] + l.L[1] * l.L[1]);
}

/*!
*	\brief ���� �� ������ ����
*
*	\param localUnitX ���� �� ���̸� �մ� ������ ���ο� ������ �����̶� �� ��, �� ������ �������͸� global coordinate���� ǥ���� ��
*	\param localUnitY ���� �� ���̸� �մ� ������ ������ ������ ���ο� ������ �����̶� �� ��, �� ������ �������͸� global coordinate���� ǥ���� ��
*	\param localX ���� �߽ɿ��� ������ ǥ������ ��, local coordinate�� X���������� ����
*	\param localX ���� �߽ɿ��� ������ ǥ������ ��, local coordinate�� Y���������� ����
*
*	\return ������ ������ ���� ���� �ٸ� tuple�� ��ȯ. ù ��°�� �� ��°�� Point Ÿ���� ����, �� ��° int Ÿ���� ������ ������ �ǹ��Ѵ�.
*/
std::tuple<Point, Point, int> intersection_collision(Circle & lhs, Circle & rhs)
{
	std::tuple<Point, Point, int> result;

	// ���� �߽� ������ �Ÿ�
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
*	\brief ���� �� ������ ����
*
*	\param localUnitX ���� �� ���̸� �մ� ������ ���ο� ������ �����̶� �� ��, �� ������ �������͸� global coordinate���� ǥ���� ��
*	\param localUnitY ���� �� ���̸� �մ� ������ ������ ������ ���ο� ������ �����̶� �� ��, �� ������ �������͸� global coordinate���� ǥ���� ��
*	\param localX ���� �߽ɿ��� ������ ǥ������ ��, local coordinate�� X���������� ����
*	\param localX ���� �߽ɿ��� ������ ǥ������ ��, local coordinate�� Y���������� ����
*
*	\return ������ ������ ���� ���� �ٸ� tuple�� ��ȯ. ù ��°�� �� ��°�� Point Ÿ���� ����, �� ��° int Ÿ���� ������ ������ �ǹ��Ѵ�. ���� �� ���������� Ȯ���Ѵ�.

Def : Get circle(not arc) intersection by : cosine law on triangle(center-center line , radius1, radius2)

localX : has direction connecting two circle centers
localY : perpendicular to localX
*/
std::tuple<Point, Point, int> intersection_self(Circle & lhs, Circle & rhs)
{
	std::tuple<Point, Point, int> result;

	// ���� �߽� ������ �Ÿ�
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
*	\brief Interior Disks�� Import
*
*	\param filename ������ �̸�
*
*	\return Import�� Disk�� Circle Ŭ������ vector ���·� ��ȯ�Ѵ�.
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
*	\brief �� Circle Ŭ������ �迭�� minkowski sum�� ����� ��ȯ
*
*	\param lhs ù��° ����
*	\param rhs �ι�° ����
*
*	\return lhs�� rhs�� ���� minkowski sum�� ����� ��ȯ�Ѵ�.
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
*	\brief ���� ���� ���
*
*	\param os cout
*	\param p ����� ��
*
*	\return ���� �߽ɰ� �������� ������� ��ȯ�Ѵ�
*/
std::ostream & operator<<(std::ostream & os, const Circle & p)
{
	os << "Center: " << p.c << "\n" << "Radius: " << p.r;
	return os;
}

/*!
*	\brief �� Arc�� ����
*
*	\param s Arc�� �����ϴ� �� ���� ����
*	\param check ���� ������ �� Arc�� ������ ���ԵǴ����� Ȯ��
*
*	\return Arc�� ������ ������ ���� ���� �ٸ� tuple�� ��ȯ, �� ��° int�� ������ ����
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
*	\brief �� Arc�� �浹 ����
*
*	\param s Arc�� �����ϴ� �� ���� ����
*	\param check ���� ������ �� Arc�� ������ ���ԵǴ����� Ȯ��
*
*	\return Arc�� �浹 ���θ� �Ǵ��Ͽ� ������ �����ϸ� true�� ��ȯ�Ѵ�.
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
*	\brief Circular Arc�� ������ ���
*
*	\param os cout
*	\param p ����ϴ� Circular Arc
*
*	\return Circular Arc�� �������� ������ ��ǥ�� ����Ѵ�.
*/
std::ostream & operator<<(std::ostream & os, const CircularArc & p)
{
	os << p.c << "\n" << "init Point: " << p.x[0] << "\n" << "end Point: " << p.x[1];
	return os;
}

/*!
*	\brief �� Bezier Function�� ��
*
*	\param lhs ù���� �Լ�
*	\param rhs �ι�° �Լ�
*
*	\return �� Bezier Function�� �� ����� ���� Bezier Function�� ��ȯ�Ѵ�.
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
*	\brief �� Bezier Function�� ��
*
*	\param lhs ù���� �Լ�
*	\param rhs �ι�° �Լ�
*
*	\return �� Bezier Function�� �� ����� ���� ���� Bezier Function�� ��ȯ�Ѵ�.
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
*	\brief �� Bezier Function�� ��Į���
*
*	\param lhs ù���� �Լ�
*	\param rhs �ι�° �Լ�
*
*	\return �� Bezier Function�� ������ Bezier Function�� ��ȯ�Ѵ�.
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
*	\brief �� Bezier Function�� ���Ͱ�
*
*	\param lhs ù��° �Լ�
*	\param rhs �ι�° �Լ�
*
*	\return �� Bezier Function�� ������ Bezier Function�� ��ȯ�Ѵ�.
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
*	\brief Bezier Function�� �̺�
*
*	\param Crv Bezier Function
*
*	\return Bezier Function�� �̺��� �Լ��� Bezier Function�� Form���� ��ȯ�Ѵ�.
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
*	\brief Bezier Function�� ������ ���
*
*	\param os cout
*	\param c ������ ����ϴ� �Լ�
*
*	\return Bezier Function�� �� Control Point�� ����Ѵ�.
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
*	\brief Bezier Curve�� Import
*
*	\param filename ������ �̸�
*
*	\return Import�� Bezier Curve���� vector�� �ڷ������� ����Ѵ�.
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
*	\brief �� ���� �浹���� �Ǵ�
*
*	\param lhs ù��° ��
*	\param rhs �ι�° ��
*	\param p ��Ī��
*
*	\return �� lhs��, �� rhs�� p�� ���� ����Ī�̵��� ���� Collision�� �Ǵ��Ѵ�.
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
*	\brief ��Һ�
*
*	\param lhs ù ��° segment
*	\param rhs �� ��° segment
*
*	\return �� segment�� ���Ѵ�.
*/
bool operator<(const Segment & lhs, const Segment & rhs)
{
	return (lhs.value < rhs.value);
}

/*!
*	\brief Arc Spline�� �������� ����
*
*	\param lhs ù ��° Arc Spline Segment
*	\param rhs �� ��° Arc Spline Segment
*
*	\return �� Arc Spline Segment�� ����Ǿ������� true�� ��ȯ�Ѵ�. �� Arc Spline�� ����� ���, ������ ���������� ������Ʈ �����ش�.

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
	// �� ArcSpline�� �������� ���� ������ ���Ͽ� �Ǵ��Ͽ��� �ϹǷ�,
	// �� �� ���� ��쿡 ���Ͽ� �Ǵ��Ѵ�. 
	// ���� �� ���� Numerical Error �αٿ��� ����� ���� ������, �� ���� ����� �� �ִ� �ĺ��� �ȴ�.
	if (lhs.init() == rhs.init()) {
		// ������ ������谡 �ִ� ���, �� ��������� �Ÿ��� ���ο� �Ÿ��� ���Ѵ�
		// ������ �Ÿ��� �� ����� ���, ������Ʈ ���� �ʴ´�
		if (lhs._neighbor[0] && (distance(lhs.init(), rhs.init()) > lhs.neighborDist[0])) {
			if (!reval)
				reval = false;
		}
		else if (rhs._neighbor[0] && (distance(lhs.init(), rhs.init()) > rhs.neighborDist[0])) {
			if (!reval)
				reval = false;
		}
		// ���ο� �Ÿ��� ������ �Ÿ����� ����� ��� �̸� ������Ʈ �����ش�
		else {
			//refresh
			// ������ ��������� ������ �ʱ�ȭ�����ش�.
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
			// ���ο� ��������� ���� �־��ش�.
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
*	\brief Arc Spline�� ���� ã��
*
*	\param lhs ù ��° Arc Spline Segment
*	\param rhs �� ��° Arc Spline Segment
*
*	\return �� Arc Spline Segment�� ���������� Ȯ���� ��, ������ ������ ���� ������ ������Ʈ�Ѵ�.

1. (BVH for lhs) vs (BVH for rhs)
2. if (leaf_VS_leaf case reached) compute if arcs intersect w/ intersection_CircularArc
*/
void selfIntersectionPts(ArcSpline &lhs, ArcSpline &rhs)
{
	/* ��� Arc�� �ݽð����!!! */
	// Arc Spline�� Circular Arc���� Index ������ ���� ������ �ɰ����� AABB Bounding Volume�� �̿��Ͽ� �浹�� Ȯ���Ѵ�.
	// ex. A.Arcs[3-8], B.Arcs[8-13]�� ���� �浹 -> A.Arcs[3-5] / A.Arcs[6-8], B.Arcs[8-10] / B.Arcs[11-13] ���� �� 4 ���� ��쿡 ���Ͽ� �浹�� Ȯ���Ѵ�
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
				// ������ �߿�!
				// �ֱ׷����� ����� �ȳ��� ����
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
		// Leap Node (Single Circular Arc)���� ���� -> �� Arc�� ������ ������ ������ ��츦 ������
		else {

			// 2. reached leaf x leaf case.

			// 2-1. find intersection
			auto reval = intersection_CircularArc(lhs.Arcs[lIdx.first], rhs.Arcs[rIdx.first]);

			// 2-2. switch (number_of_intersections)
			switch (std::get<2>(reval)) {
			// �ϳ��� ������ ���, Convolution �������� �����س��� boundary�� �̿��Ͽ�
			// �������� ������ dividedPts�� ������ ���ڷ� ǥ���Ѵ�.
			// ������ ���� ���� �κ��� �������� true, ���� ������ �κ��� �������� false�� �־��ش�.
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
			// ������ �� ���� ��� �� 16���� ��찡 ������, �̸� ��� �����غ��� �Ʒ��� ���� �� ������ �پ���
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
*	\brief ���� �� Arc spline ���� Ȯ���Ͽ�, �� ArcSpline�� ������ ��ģ ������ Arc�� Index�� ���� ls, rs�� ������ �� Convolution �Լ��� �Ѱ��ش�
*
*	\param lhs ù ��° ��
*	\param rhs �� ��° ��
*	\param source Convolution ����� ����
*/
void overlapTest(std::vector<ArcSpline> &source, ArcSpline & lhs, ArcSpline & rhs)
{
	// 1. build ls, rs and call convolution_ArcSpline

	// ���� ���� ù Index�� �� Index : ls[0], ls[1]
	// ������ ���� ù Index�� �� Index : rs[0], rs[1]
	int ls[2], rs[2];
	// ��� Topology�� �� Arc�� �����ִ��� Ȯ���Ѵ�
	short overlap = overlapCase(lhs, rhs);
	// �� ��쿡 ���Ͽ� ���ڸ� ���߾� Con
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
*	\brief ���� �� Arc spline ���� Ȯ���Ͽ�, �� ArcSpline�� ������ ��ģ ������ Arc�� Index�� ���� ls, rs�� ������ �� Convolution �Լ��� �Ѱ��ش�
*	(��, ���� �ٸ� ��и鿡 ���� ��쿡 ���� �Լ�)
*
*	\param lhs ù ��° ��
*	\param rhs �� ��° ��
*	\param source Convolution ����� ����
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
*	\brief ���� �ϳ��� Arc spline ���� Ȯ���Ͽ�, �ڱ� �ڽŰ� Convolution�Ѵ�
*	(��, ���� ���� �𵨿� ���� �Լ�)
*
*	\param lhs ù ��° ��
*	\param source Convolution ����� ����
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
*	\brief ���� �� Arc spline ���� Ȯ���Ͽ�, �� ArcSpline�� ������ ���� ��� �����ִ��� �Ǵ��Ѵ�
*
*	\param lhs ù ��° ��
*	\param rhs �� ��° ��
*
*	\return ������ ��Ȳ�� ���� 1, 3, 4, 6�� ���� ��ȯ�Ѵ�

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
*	\brief ���� �� Arc spline ���� Ȯ���Ͽ�, �� ArcSpline�� ������ ���� ��� �����ִ��� �Ǵ��Ѵ� (���� �ٸ� ���⿡ ���� �Լ�)
*
*	\param lhs ù ��° ��
*	\param rhs �� ��° ��
*
*	\return ������ ��Ȳ�� ���� 1, 3, 4, 6�� ���� ��ȯ�Ѵ�

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
*	\brief lhs, rhs�� �� Arcspline ���� Convolution �� ��, �̸� source�� �����Ѵ�
*
*	\param source ���� ����� �����Ѵ�
*	\param lhs ù ��° ��
*	\param lhs �� ��° ��
*	\param ls ù ��° ���� ��ġ�� Circular Arc�� Index
*	\param ls �� ��° ���� ��ġ�� Circular Arc�� Index
*	\param reverse Arc�� ������ ���� �������� �Ǵ�

TODO : the code should be fixed, so that all circular arcs are clockwise when they form the boundary loop
	or unify it ccw? and flip it?
*/
void Convolution_ArcSpline(std::vector<ArcSpline> &source, ArcSpline & lhs, ArcSpline & rhs, int ls[2], int rs[2], bool reverse)
{

	std::vector<ArcSpline> reval;
	ArcSpline input;
	bool check = false;

	// Arc�� ���� �������� �����ϴ� (���� ��и鿡 ���ԵǾ��ִ�)
	// case1 : same quadrant
	if (!reverse) {
		// Arc�� �������� �ٲ�� Normal�� ����
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
			// ccw�� �ٸ� ������ ��� ������ ��Ȱ��!
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
		//Arc�� �ٸ� ����! rhs�� ������ ����� ��������
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
*	\brief �� Arc Spline�� aabb �׽�Ʈ�� �Ѵ�
*
*	\param lhs ù ��° ��
*	\param rhs �� ��° ��
*
*	\return �� aabb box�� �浹�ϸ� true, �ƴϸ� false�� ��ȯ�Ѵ�

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
*	\brief �� Arc Spline�� aabb �׽�Ʈ�� �Ѵ�
*
*	\param lhs ù ��° Arc Spline Segment
*	\param rhs �� ��° Arc Spline Segment
*	\param left ù ��° ���� Arc Index�� ����
*	\param right �� ��° ���� Arc Index�� ����
*
*	\return lhs�� left, rhs�� right ������ �ִ� Arc�� ������ �� aabb box�� �浹�ϸ� true, �ƴϸ� false�� ��ȯ�Ѵ�
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
*	\brief �� Arc Spline�� aabb �׽�Ʈ�� �Ѵ�
*
*	\param lhs ù ��° Circular Arc
*	\param rhs �� ��° Arc Spline Segment
*	\param right �� ��° ���� Arc Index�� ����
*
*	\return lhs�� rhs�� right ������ �ִ� Arc�� ������ �� aabb box�� �浹�ϸ� true, �ƴϸ� false�� ��ȯ�Ѵ�
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
*	\brief Arc Spline�� ���� ���
*
*	\param os cout
*	\param p Arc Spline
*
*	\return Arc Spline�� �̷�� �� Circular Arc�� ������� ����Ѵ�
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
*	\brief input���� ���� Arc Spline Segment���� Trimming - �����δ� �� Circular Arc�� Convolution�� �� ���ÿ� Trimming ������, Statistic Data�� �̾Ƴ� �� Trimming Time�� ���� �����ϱ� ���� ���� �Լ��� �����Ͽ���.
*
*	\param input trimming�ϱ� �� Arc Spline Segments
*	\param trimmed trimming�� �� ���� Arc Spline Segments
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
*	\brief �� BCA�� �浹 ���θ� �Ǵ�
*
*	\param lhs ù ��° BCA
*	\param rhs �� ��° BCA
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
*	\brief Circular Arc�� BCA�� �浹 ���θ� �Ǵ�
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
*	\brief �� dividePts�� ��Ҹ� ��
*
*	\param lhs ù ��° dividePts
*	\param rhs �� ��° dividePts
*
*	\return �� dividePts�� ��Ұ��踦 �Ǵ��Ͽ� ��ȯ�Ѵ�. �ش� Arc Spline Segment�� �������� ����� ���� ���� ���̴�.
*/
bool operator<(dividePts & lhs, dividePts & rhs)
{
	if (lhs.idx != rhs.idx)
		return lhs.idx < rhs.idx;
	else
		return distance(lhs.initPt, lhs.dividePt) < distance(lhs.initPt, rhs.dividePt);
}


/*!
*	\brief �⺻ ������
*/
Line::Line() {
	P[0] = Point();
	P[1] = Point();
	L[0] = 0.0;
	L[1] = 0.0;
	L[2] = 0.0;
}

/*!
*	\brief �� ���� ������ ������ ���Ѵ�
*
*	\param p ù���� ��
*	\param q �ι�° ��
*
*	\return �� ���� ������ ������ �������� ����� ��ȯ�Ѵ�
*/
Line::Line(Point & p, Point & q) {
	L[0] = p[1] - q[1];
	L[1] = q[0] - p[0];
	L[2] = p[0] * q[1] - p[1] * q[0];
	P[0] = p;
	P[1] = q;
}

/*!
*	\brief �Ҹ���
*/
Line::~Line()
{
}


/*!
*	\brief �� Line�� ��Һ�
*
*	\param rhs ���ϴ� Line
*
*	\return �� Line�� ���̸� ���Ͽ� �� ��Ұ��踦 ��ȯ�Ѵ�.
*/
bool Line::operator<(const Line & rhs)
{
	return counterclockwise((P[1] - P[0]), (rhs.P[1] - rhs.P[0]));
}


/*!
*	\brief Line�� �����´�
*
*	\return Line�� �������� ������ ���� �ٲپ� ��ȯ�Ѵ�.
*/
Line Line::operator-()
{
	return Line(P[1], P[0]);
}

/*!
*	\brief Line ���� ��/�ܺ����� ��ȯ
*
*	\param t �� �� ������ ����
*
*	\return Line ���� t : 1-t �������� ��ȯ�Ѵ�. (t < 0 or 1-t < 0�� ��� �ܺ���)
*/
Point Line::operator()(double t)
{
	return (1 - t) * P[0] + t * P[1];
}

/*!
*	\brief p�� ������ dir�� ������ ����
*
*	\param p ù���� ��
*	\param dir ���⺤��
*
*	\return p�� ������ dir�� ������ ������ ��ȯ�Ѵ�.
*/
Line shoot(Point & p, Point & dir)
{
	return Line(p, p + dir);
}

/*!
*	\brief p�� q�� �����̵�м�
*
*	\param p ù���� ��
*	\param q �ι�° ��
*
*	\return p�� q�� �����̵�м��� ��ȯ�Ѵ�.
*/
Line bisector(Point & p, Point & q)
{
	return shoot((p + q) / 2, (q - p).rotate());
}

/*!
*	\brief �߽��� _c, �������� _r�� ��
*
*	\param _c ���� �߽�
*	\param _r ���� ������
*/
Circle::Circle(Point & _c, double _r)
{
	c = _c, r = _r;
}

/*!
*	\brief �߽��� _c, x�� ������ ��
*
*	\param _c ���� �߽�
*	\param x ���� ������ ��
*/
Circle::Circle(Point & _c, Point & p)
{
	c = _c, r = sqrt(distance(_c, p));
}

/*!
*	\brief p, q�� ������ p������ �������Ͱ� v�� ��
*
*	\param p ù��° ��
*	\param v ù��° �������� ��������
*	\param q �ι�° ��
*/
Circle::Circle(Point & p, Point & q, Point & v)
	:c(shoot(p, v.rotate()), bisector(p, q)), r(sqrt(distance(c, p))) {}

Circle::~Circle()
{
}

/*!
*	\brief �� �����ִ� �� �� angle ������ ���� ��
*
*	\param p ���� local polar coordinate������ angle��
*
*	\return ������ �ִ� angle ������ ���� ��ȯ�Ѵ�.
*/
Point Circle::operator()(double angle)
{
	return Point(cos(angle), sin(angle)) * r + c;
}

/*!
*	\brief ���� ��Ұ��踦 ��ȯ
*
*	\param rhs ���ϴ� ��
*
*	\return �� �ڽŰ� rhs�� �������� ũ�⸦ ���Ͽ� �� ��Ұ��踦 ��ȯ�Ѵ�.
*/
bool Circle::operator<(const Circle & rhs)
{
	return (r < rhs.r);
}

/*!
*	\brief ���� ��Ұ��踦 ��ȯ
*
*	\param rhs ���ϴ� ��
*
*	\return �� �ڽŰ� rhs�� �������� ũ�⸦ ���Ͽ� �� ��Ұ��踦 ��ȯ�Ѵ�.
*/
bool Circle::operator>(const Circle & rhs)
{
	return (r > rhs.r);
}

/*!
*	\brief �� p�� ���� ���ԵǴ����� �Ǵ�
*
*	\param p ���ԵǴ��� �Ǵ��Ϸ��� ��ü
*
*	\return �� p�� ���� ���ԵǸ� true, �׷��� ������ false�� ��ȯ�Ѵ�.
*/
bool Circle::contain(Point & p)
{
	return distance(c, p)  < r * r;
}

/*!
*	\brief �� p�� ���� ���ԵǴ����� �Ǵ�
*
*	\param p ���ԵǴ��� �Ǵ��Ϸ��� ��ü
*
*	\return �� p�� ���� ���ԵǸ� true, �׷��� ������ false�� ��ȯ�Ѵ�. Trimming�� ���� �Լ��̹Ƿ�, Interior Disk�� ������ �����Ͽ� ���� �� ���������� true�� ��ȯ�Ѵ�.
*/
bool Circle::contain_trimming(Point & p)
{
	return distance(c, p) < r * r * 0.95;
}

/*!
*	\brief �� p�� Circular Arc�� �� ������ ���� ���ԵǴ����� �Ǵ�
*
*	\param Arc ���ԵǴ��� �Ǵ��Ϸ��� ��ü
*	\param p ���ԵǴ��� �Ǵ��Ϸ��� ��ü
*
*	\return �� p�� Circular Arc�� ��� ���� ���ԵǸ� true, �׷��� ������ false�� ��ȯ�Ѵ�.
*/
bool Circle::contain(CircularArc & Arc, Point & p)
{
	return contain_trimming(Arc.x[0]) && contain_trimming(Arc.x[1]) && (contain_trimming(p));
}



/*!
*	\brief ���� �߽����κ��� �� p������ �������� ���� ����
*
*	\param p projection�Ϸ��� ��ü
*
*	\return ���� �߽����κ��� �� p������ �������� ���� ������ ��ȯ�Ѵ�.
*/
Point Circle::projection(Point & p)
{
	return (p - c).normalize() * r + c;
}

/*!
*	\brief ���� �߽ɿ��� �� p������ ��������
*
*	\param p �� ���� ��
*
*	\return ���� �߽ɿ��� �ٶ� p������ �������͸� ��ȯ�Ѵ�
*/
Point Circle::localDirection(Point & p)
{
	return (p - c).normalize();
}

/*!
*	\brief �� ���� ���� CRES�� ��ŭ Sampling�Ͽ� �׸�
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
*	\brief Circular Arc�� p�� ���Ͽ� ����Ī
*
*	\param arc ��Ī��Ű�� ��ü
*	\param p ��Ī��
*
*	\return arc�� p�� ���Ͽ� ��Ī��Ų Circular Arc�� ��ȯ�Ѵ�.
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
*	\brief i���� e����, i������ ������ t�� Arc ����
*
*	\param i ������
*	\param e ����
*	\param t �������� ��������
*/
CircularArc::CircularArc(Point & i, Point & e, Point & t)
	:c(i, e, t), x{ i, e }, n{ c.localDirection(i), c.localDirection(e) }
{
	if (!counterclockwise(n[0], n[1])) {
		ccw = false;
		std::swap(n[0], n[1]); //�������� ������ �ϴ�
	}
	else
		ccw = true;
	boundary = false;
}

/*!
*	\brief ���� �߽��� c, �������� r, Circular Arc�� �� ������ ���������� ���� normal1, normal2�� Circular Arc�� ����
*
*	\param _c ���� �߽�
*	\param _r ���� ������
*	\param normal1 �������� ���� ����
*	\param normal2 ������ ���� ����
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
*	\brief �Ҹ���
*/
CircularArc::~CircularArc()
{
}

/*!
*	\brief i�� e ���̸� angle�� ���� 0���� 1������ t�� �Ű�ȭ������, t�� ���� ���� ���� ��ġ
*
*	\return i�� e ���̸� angle�� ���� 0���� 1������ t�� �Ű�ȭ������, t�� ���� ���� ���� ��ġ�� ��ȯ�Ѵ�.
*/
Point CircularArc::operator()(double t)
{
	if (t > 1.000 + N_PRESCISION || t < -N_PRESCISION)
		std::cout << "error: t is out of coverage" << std::endl;//for debuging
	return c.projection(x[0] + t * (x[1] - x[0]));
}

/*!
*	\brief ȣ�� ������ �ٲپ��ִ� ������
*/
CircularArc CircularArc::operator-()
{
	auto s = *this;
	std::swap(s.x[0], s.x[1]);
	return s;
}


/*!
*	\brief Circular Arc�� ������ ��
*
*	\param rhs ���ϴ� ��ü
*
*	\return �� Circular Arc�� �������� ��Ұ��踦 ��ȯ.
*/
bool CircularArc::operator<(CircularArc & rhs)
{
	return c < rhs.c;
}


/*!
*	\brief Circular Arc�� x, y extreme�α��� normal�� ���� ���� x / y���� ����� ������ ����ϸ� �̸� ��Ȯ�� x / y���� �������� �ٲپ��ش�
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
*	\brief Circular Arc�� Interior Disks�� ���ԵǴ����� Ȯ��
*
*	\param p Circular Arc�� �� ���������� ������ ������ ��. Circular Arc�� �� ������ p�� �ﰢ���� �����, Circular Arc�� Bounding�ȴ�.
*	\param init �����ϴ� Cache�� ���� Index
*	\param Cache_Trimming Caching�� Interior Disk�� �ּҰ� ����ִ� Ŭ����
*	\param Grid_Trimming �� Grid�� ���Ե� Interior Disk �ּҰ� ����Ǿ��ִ� Ŭ����
*	\param FindPosition Circular Arc�� ���Ե� Grid�� ��ġ�� ��ȯ
*	\param SingleGrid Circular Arc�� �ϳ��� Grid���� ���ԵǴ����� Ȯ��
*
*	\return Circular Arc�� Trimming���� �ʴ� ��� True�� ��ȯ�Ѵ�.
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
			// check : �ֱٿ� �ش� Disk�� Trimming�� ���������� Check��
			Cache_Trimming.check[Cache_Trimming.idx] = true;
			return false;
		}
		if (Cache_Trimming.idx == cacheSize - 1)
			Cache_Trimming.idx = 0;
		else
			Cache_Trimming.idx++;
	} while (init != Cache_Trimming.idx);

	// FP[0][1][2][3] -> {xmin, xmax, ymin, ymax}�� position
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
				// Second Chance Algorithm���� Cache���� ���� Index�� ã�Ƴ�
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
*	\brief ���� ȣ ���� �ִ��� �Ǵ�
*
*	\param p �Ǵ��ϴ� ��
*
*	\return ���� ȣ ���� ������ true, ������ false�� ��ȯ�Ѵ�.
*/
bool CircularArc::contain(Point & p)
{
	Point q = c.localDirection(p);
	return counterclockwise(n[0], q) && counterclockwise(q, n[1]);
}

/*!
*	\brief Circular Arc�� �ݽð�������� �Ǵ�
*
*	\return Circular Arc�� �ݽð������ ��� true�� ��ȯ�Ѵ�.
*/
bool CircularArc::isCCW()
{
	return counterclockwise(x[0] - c.c, x[1] - c.c);
}

/*!
*	\brief Circular Arc�� x���� ���ǹ������� Convex���� �Ǵ�
*
*	\return Circular Arc�� x���� ���ǹ������� Convex�� ��� true�� ��ȯ�Ѵ�.
*/
bool CircularArc::isXQuardrants()
{
	Vector test = n[0] + n[1];
	return (test.P[0] > 0);
}


/*!
*	\brief Circular Arc�� y���� ���ǹ������� Convex���� �Ǵ�
*
*	\return Circular Arc�� y���� ���ǹ������� Convex�� ��� true�� ��ȯ�Ѵ�.
*/
bool CircularArc::isYQuardrants()
{
	Vector test = n[0] + n[1];
	return (test.P[1] > 0);
}


/*!
*	\brief Circular Arc�� Convex �κ��� �ܺ������� �Ǵ�
*
*	\return Circular Arc�� Convex �ٱ��� �κ��� Model�� �ܺ��� ��� true�� ��ȯ�Ѵ�.
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
*	\brief ȣ�� �ΰ��� ����
*
*	\param t ȣ�� 0���� 1������ �Ǽ��� �Ű�ȭ ���� �� ȣ ���� ��ġ�� ��Ÿ���� ����
*
*	\return ȣ�� �Ű�ȭ�� t�� ������ �� ȣ�� ���� ����Ѵ�.
*/
std::pair<CircularArc, CircularArc> CircularArc::subDiv(double t)
{
	if (t > 1 || t < 0)
		std::cout << "error: t is out of coverage" << std::endl;//for debuging
	auto p = c.projection(x[0] + t*(x[1] - x[0]));
	return std::make_pair(CircularArc(x[0], p, c.localDirection(x[0]).rotate()), CircularArc(p, x[1], c.localDirection(p).rotate()));
}

/*!
*	\brief RES���� ������ Sampling�Ͽ� Circular Arc�� �׸�
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
*	\brief Bezier Curve�� t�������� Geometry�� �����ϴ� ������
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
*	\brief Circular Arc�� t�������� Geometry�� �����ϴ� ������
*/
Geometry::Geometry(CircularArc & a, double t)
{
	x = a(t);
	e = a.c.c;
	//����� �߽��� ���ϴ� ����
	n = (e - x).normalize();
	r = a.c.r;
	if (!counterclockwise(a.c.c[0] - e, a.c.c[1] - e)) {
		r = -r;
		n = -n;
	}
	v = -n.rotate();
}

/*!
*	\brief �Ҹ���
*/
Geometry::~Geometry()
{
}

/*!
*	\brief Geometry������ Osculating Circle�� ����
*
*	\return Geometry �������� Osculating Circle�� ��ȯ
*/
Circle Geometry::osculatingCircle()
{
	return Circle(e, abs(r));
}

/*!
*	\brief ������
*
*	\param deg �Լ��� ����
*	\param ptype �Լ��� ���� (0�� ��� 1����, 1�� ��� 2����)
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
*	\brief ���� ������
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
*	\brief a�� Control Point�� �̷���� Cubic Bezier Curve�� �����ϴ� ������
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
*	\brief Bezier Curve�� �� p�� ���Ͽ� ��Ī
*
*	\param Crv ����Ī��Ű�� ��ü
*	\param Crv ����Ī��
*
*	\return Crv�� p�� ���Ͽ� ��Ī��Ų Bezier Curve�� ��ȯ�Ѵ�.
*/
BezierCrv::BezierCrv(const BezierCrv & Crv, const Point & p)
{
	(*this) = Crv;
	for (int i = 0; i <= Deg; i++)
		P[i] = -P[i] + p;
}

/*!
*	\brief �Ҹ���
*/
BezierCrv::~BezierCrv()
{
}

/*!
*	\brief ���� ������
*
*	\param rhs ���Ե� ��ü
*
*	\return ���Ե� �ڽ��� ��ȯ�Ѵ�.
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
*	\brief Bezier Function�� ��
*
*	\param t �����ϴ� ��
*
*	\return Bezier Function�� t�� ������ ���� ��ȯ�Ѵ�.
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
		// deCasteljau �˰������� ���ϱ�.
		break;
	}
	return pt;
}

/*!
*	\brief Bezier Function�� �Ǽ��� ����
*
*	\param rhs �Ǽ����ϴ� ��
*
*	\return Bezier Function�� rhs�� ���� Bezier Function�� ��ȯ�Ѵ�.
*/
BezierCrv BezierCrv::operator*(double rhs)
{
	BezierCrv lhs = *this;
	for (int i = 0; i <= lhs.Deg; i++)
		lhs.P[i] = lhs.P[i] * rhs;
	return lhs;
}

/*!
*	\brief Bezier Curve�� ������������ �������� ����
*
*	\return Bezier Curve�� ������������ �������� ���͸� ��ȯ�Ѵ�.
*/
Point BezierCrv::tangentialVector_sp()
{
	return (double)Deg * (P[1]-P[0]);
}

/*!
*	\brief Bezier Curve�� ���������� �������� ����
*
*	\return Bezier Curve�� ���������� �������� ���͸� ��ȯ�Ѵ�.
*/
Point BezierCrv::tangentialVector_ep()
{
	return (double)Deg * (P[Deg] - P[Deg - 1]);
}

/*!
*	\brief Bezier Function�� �� ���� ����
*
*	\param t ������ ��
*
*	\return �ϳ��� Bezier Function�� t�� �������� �� ���� Bezier Function���� ������.
*/
std::pair<BezierCrv, BezierCrv> BezierCrv::subDiv(double t)
{
	std::pair<BezierCrv, BezierCrv> reval = { BezierCrv(Deg, PType) , BezierCrv(Deg, PType) };
	auto temp = *this;
	//ó�� i���� Point������ �̷���� Bezier Function�� t�� ������ ��: �������� Function�� i��° Control Point
	for (int i = 0; i <= Deg; i++) {
		temp.Deg = i;
		reval.first.P[i] = temp(t);
	}

	temp.reverse();
	//�����κ��� i���� Point������ �̷���� Bezier Function�� t�� ������ ��: �������� Function�� i��° Control Point
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
*	\brief Bezier Curve�� ���
*
*	\param t ����� ���ϴ� ���� ��ġ
*
*	\return Bezier Curve�� t�� �ش��ϴ� �������� ��� ���� ��ȯ�Ѵ�.
*/
double BezierCrv::curvature(double t)
{
	Point v = diff(*this)(t);
	Point a = diff(diff(*this))(t);
	return v ^ a / (v.length() * sqrt(v.length()));
}

/*!
*	\brief Bezier Curve�� Bi Circular Arc�� �ٻ����� �� Error Bound EPSILON / 2.0�� �����ϴ����� Ȯ��
*
*	\param bs 
*
*	\return Bezier Function�� rhs�� ���� Bezier Function�� ��ȯ�Ѵ�.
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
*	\brief Bezier Function�� Control Point�� ������ ������ (0 ���� 1 ������ t�� �Ű�ȭ �� ���� 1 ���� 0 ������ ������)
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
*	\brief Bezier Function�� ������ Reduction
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
*	\brief Bezier Function�� Control Point�� ������ ������ (0 ���� 1 ������ t�� �Ű�ȭ �� ���� 1 ���� 0 ������ ������)
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
*	\brief Bezier Curve���� Ư�� ���⸦ ���� ���� ��ȯ
*
*	\param normal ���ϴ� ���� ���Ϳ� ������ ����
*
*	\return normal�� ������ Curve ���� ���� Curve�� �Ű�ȭ�� ��(0�� 1 ����)�� �������� ��ȯ�Ѵ�.
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
*	\brief Bezier Curve�� x���� Bezier Function�� ��ȯ
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
*	\brief Bezier Curve�� y���� Bezier Function�� ��ȯ
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
*	\brief �� Bezier Curve�� �浹 ����
*
*	\param rhs �浹 �����ϴ� ��ü
*
*	\return �ڽŰ� rhs�� aabbtest�� �Ͽ� rhs�� �浹�ϸ� true�� ��ȯ�Ѵ�.
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
*	\brief rhs�� p�� ���Ͽ� ����Ī�� Bezier Curve���� �浹�� ����
*
*	\param rhs ����Ī�ϰ� �浹���θ� �Ǵ��ϴ� Bezier Curve
*	\param p ����Ī��
*
*	\return �ڽŰ� rhs�� p�� ���Ͽ� ����Ī�� Bezier Curve�� aabbtest�� �� �� �浹�ϸ� true�� ��ȯ�Ѵ�.
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
*	\brief Spiral Bezier Curve�� �� ���� Circular Arc�� �ٻ�
*
*	\return Bezier Curve�� �� ���� Circular Arc�� �ٻ��� �� �̸� pair�� ���� ��ȯ�Ѵ�.
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
*	\brief Bezier Curve�� 20���� Line Segment�� �׸�
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
*	\brief Bezier Curve�� x, y, curvature extreme point�� inflection point���� ����
*
*	\param a �������� Bezier Curve Segment�� ����
*
*	\return Bezier Curve�� ������ ������ a�� �����Ѵ�.
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
*	\brief Bezier Curve�� split�� �� �߻��� �� �ִ� Inflection Point�� ������ �ذ��ϱ� ���� �ٽ� ����
*
*	\return Inflection Point�� Bezier Curve ���ο� ���ԵǾ� �ִ� ���, �̸� �ٽ� split�Ͽ� ��ȯ�Ѵ�.
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
*	\brief Bezier Curve�� BVH�� ���Ͽ� Heap�� �������� �Ҵ�Ǿ��ִ� Memory�� ����
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
*	\brief Bezier Curve�� split�ϴ� �Ű�ȭ�� ��ġ�� �����ϴ� ������
*/
Segment::Segment(double _value)
{
	value = _value;
	St = defaultPt;

}

/*!
*	\brief Bezier Curve�� split�ϴ� �Ű�ȭ�� ��ġ�� �����ϴ� ������
*/
Segment::Segment(double _value, SegmentTypes _St)
{
	value = _value;
	St = _St;
}

/*!
*	\brief �Ҹ���
*/
Segment::~Segment()
{
}


/*!
*	\brief �⺻ ������
*/
ArcSpline::ArcSpline()
{
	neighborDist[0] = DBL_MAX;
	neighborDist[1] = DBL_MAX;
}


/*!
*	\brief Bezier Curve�� EPSILON / 2.0�� ������ ���� Arc Spline Segment�� �ٻ��Ͽ� Arc Spline�� �����ϴ� ������
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
*	\brief Arc Spline�� �յڸ� �߶�
*
*	\param originCrv �ڸ��� ���� Arc Spline Segment
*	\param begin �ڸ��� ��
*	\param end �ڸ��� ��
*
*	\return originCrv���� begin�� end ���̸� ����� �������� ���� Arc Spline�� ��ȯ�Ѵ�.
*/
ArcSpline::ArcSpline(ArcSpline & originCrv, dividePts & begin, dividePts & end)
{
	Arcs.insert(this->Arcs.end(), originCrv.Arcs.begin() + begin.idx, originCrv.Arcs.begin() + end.idx + 1);
	Arcs.front().x[0] = begin.dividePt;
	Arcs.back().x[1] = end.dividePt;
	splited = false;
}


/*!
*	\brief �Ҹ���
*/
ArcSpline::~ArcSpline()
{
}

/*!
*	\brief Arc Spline Segment�� Ư�� Normal�� �����ϴ����� �Ǵ�
*
*	\param norm ���ϴ� ������ ����
*
*	\return norm�� Arc Spline Segment�� �����ϸ� true�� ��ȯ�Ѵ�.

True is returned when norm is n0 or n1
*/
bool ArcSpline::contain(Vector &norm)
{
	return counterclockwise(n[0], norm) && counterclockwise(norm, n[1]);
}

/*!
*	\brief Arc Spline�� �������� ��ȯ
*/
Point& ArcSpline::init()
{
	return Arcs.front().x[0];
}

/*!
*	\brief Arc Spline�� ������ ��ȯ
*/
Point& ArcSpline::end()
{
	return Arcs.back().x[1];
}

/*!
*	\brief Arc Spline�� �߰����� ��ȯ
*/
Point & ArcSpline::mid()
{
	int idx = (int)Arcs.size() / 2;
	return Arcs[idx].x[0];
}

/*!
*	\brief Arc Spline�� �� Circular Arc�� �׸�
*/
void ArcSpline::draw()
{
	for (int i = 0; i < (int)Arcs.size(); i++)
		Arcs[i].draw();
}

/*!
*	\brief Arc Spline Segment�� Circular Arc �� Normal�� �����ϴ� Circular Arc�� Index�� ��ȯ
*
*	\param norm �־��� Normal�� ���⺤��
*
*	\return Arc Spline Segment�� Circular Arc �� Normal�� �����ϴ� Circular Arc�� Index�� ��ȯ�Ѵ�.
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
*	\brief Arc Spline Segment�� Model�� Self Intersection �� ���� ��, ���η� �İ��� �κ��� Trimming
*
*	\return Arc Spline Segment�� Model�� Self Intersection �� ���� ��, ���η� �İ��� �κ��� Trimming�Ѵ�.
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
*	\brief ���ο� ���ԵǾ� Boundary�� �������� �ʴ� ���� Ȯ���� Arc Spline Segment�� ��������
*
*	\param relPos ���������� ����
*	\param cut true
*
*	\return ���η� �İ��� Arc Spline Segment�� �����

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
*	\brief �ϳ��� Arc Spline Segment�� �� �� �̻��� Self Intersection Point�� ���� ����� Trimming
*
*	\return ���� ���� Self Intersection Point�� �����ϴ� ���, ���η� �İ��� �������� ���� Ȯ���� �κе��� ����������.

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
*	\brief Numerical Error�� ���Ͽ� Arc Spline Segment�� �߻��� ������ ����
*
*	\return Arc Spline Segment�� Normal�� �ٵ��, Circular Arc�� ������ ������ ��� Arc Spline Segment�� ������ ��ȯ�Ѵ�.
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
*	\brief Arc Spline Segment�� ���δ� BCA Bounding Volume�� �����ϴ� ������
*
*	\param s ���δ� Arc Spline Segment - Spiral ������ �ʿ��ϴ�
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
*	\brief Arc Spline Segment�� �Ϻθ� ���δ� BCA Bounding Volume�� �����ϴ� ������
*
*	\param s ���δ� Arc Spline Segment - Spiral ������ �ʿ��ϴ�
*	\param Idx Arc Spline Segment�� BCA�� ����� Arc�� Index ����
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
*	\brief Bezier Curve�� ���δ� BCA Bounding Volume�� �����ϴ� ������
*
*	\param Crv Bounding�ϴ� Bezier Curve
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
*	\brief �Ҹ���
*/
BCA::~BCA()
{
}

/*!
*	\brief BCA�� p�� �����ϴ����� �Ǵ�
*
*	\param P ���Կ��θ� �Ǵ��ϴ� ��
*
*	\return BCA�� �� P�� �����ϸ� true�� ��ȯ�Ѵ�.
*/
bool BCA::contain(Point & P)
{
	return ((outer.c.contain(P)) && (!inner.c.contain(P)));
}

/*!
*	\brief BCA�� �β�
*
*	\return BCA�� �β��� ��ȯ�Ѵ�.
*/
double BCA::thickness()
{
	return sqrt(distance(inner.c.c, outer.c.c)) - abs(inner.c.r - outer.c.r);
}

/*!
*	\brief dividePts�� �����ϴ� ������
*
*	\param _idx dividePts�� �����ϴ� Arc Spline�� Index
*	\param _dividePt ����
*	\param _cutAfterPt _dividePt�� ������ Boundary ���θ� �İ��� �������� �κ��� ��� true, ������ �İ��� �κ��� ��� false
*/
dividePts::dividePts(int _idx, Point & _dividePt, bool _cutAfterPt)
{
	idx = _idx;
	dividePt = _dividePt;
	cutAfterPt = _cutAfterPt;
}


/*!
*	\brief Cache�� ������ �ʱ�ȭ
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
*	\brief Grid�� ������
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
*	\brief Circle�� ������ Grid�� ã�� Grid�� �ش� Circle�� �߰��Ͽ� ������Ʈ
*
*	\param input Circle�� �ּ�
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
*	\brief Circular Arc�� ������ Grid�� Index
*
*	\param p1 ù ��° ��
*	\param p2 �� ��° ��
*	\param p3 �� ��° ��
*
*	\return Grid�� x, y���� Index �� ���� ��ȯ�Ѵ�.
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
*	\brief Grid�� ���� Circle�� p1, p2, p3�� �����ϴ����� �Ǵ�
*
*	\param p1 ù ��° ��
*	\param p2 �� ��° ��
*	\param p3 �� ��° ��
*	\param x Grid�� x���� Index
*	\param y Grid�� y���� Index
*	\param singleGrid Circular Arc�� �� �ϳ��� Grid�� ���ԵǴ����� �Ǵ�
*
*	\return Circular Arc�� Trimming�ϴ� Circle�� �ּҿ�, Trimming ���θ� pair�� ��ȯ. (pair�� �� ��° ���� false�� ��� Circle*�� NULL�� ���ۿ� ����)
*/
std::pair<Circle*, bool> Grid::trimming(Point& p1, Point& p2, Point& p3, int x, int y, bool singleGrid)
{
	// Single Grid�� ��쿡�� Trimming�� �������� Ȯ���� �� ����
	if ((covercheck = cover[x][y]) && singleGrid)
		return std::make_pair((Circle*)NULL, true);
	int s = (int)gCircles[x][y].size();

	
	for (int i = 0; i < s; i++)
		// ������ ������ ����: Sorting �� �� �������� ���� �� ���� ū �� ������ Sorting �Ǿ� ����!
		// Grid�� Circle �� �������� ū Circle�� ���� ���غ��� ����!
		if (gCircles[x][y][s - i - 1]->contain_trimming(p1) && gCircles[x][y][s - i - 1]->contain_trimming(p2) && gCircles[x][y][s - i - 1]->contain_trimming(p3))
			return std::make_pair(gCircles[x][y][s - i - 1], true);
	return std::make_pair((Circle*)NULL, false);
}

}