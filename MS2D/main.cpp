#include "MS2D.h"
#include <iostream>
#include <iomanip>
#include "voronoi.hpp"
#include "collision detection.hpp"


planning::coneVoronoi coneVor;

namespace ms {

	/* OpenGL Parameters */
	GLsizei wd = 900, ht = 600;
	double zoom = 1.4;
	double tx, ty;
	using namespace std;

	/* Comments - MS2D.h 참고 */

#pragma region global variables related to time.
	int ModelInfo_CurrentFrame = 357; // 355;//355; //148; // 44;		// 7,7,261 case
	int ModelInfo_CurrentFrameOld; // due to the program structure... when we process frame i, modelInfo_CurrentFrame = i + 1...(due to pre increment in some func...) => save val.
	//int ModelInfo_CurrentFrame = 93;			// [0, 360)
	pair<int, int> ModelInfo_CurrentModel;	// [0, 8) x [0, 8)

	int
		&t0 = ModelInfo_CurrentModel.first,
		&t1 = ModelInfo_CurrentModel.second,
		&t2 = ModelInfo_CurrentFrame;	

	int numoftest = 0;						// actually a bool, if 1 -> end test.
#pragma endregion

#pragma region global variables containing data (~~ input for minkowskiSum)
// Models are 1. imported. 2. segmented. 3. rotated.
	vector<BezierCrv>	Models_Imported[8];					// def: result of read_file.							// set@ void initialize()
	vector<Circle>		InteriorDisks_Imported[8];			// def: result of read_file.							// set@ void initialize()

	vector<BezierCrv>	Models[8];							// def: segmented version of Models_Imported.			// set@ initialize	// used@ minkowskiSum
	vector<ArcSpline>	Models_Approx[8];					// def: arc approximated verison of above				// set@ initialize	// used@ minkowskiSum
	vector<BezierCrv>	Models_Rotated[numofframe];			// def: this[i] = i degree rotated First model segments // set@ postProcess // used@ minkowskiSum
	vector<ArcSpline>	Models_Rotated_Approx[numofframe];	// def: arc approximated verison of above				// set@ postProcess // used@ minkowskiSum

	vector<Circle>		InteriorDisks_Rotated[numofframe];	// def: this[i] = i degree rotated disks of First model // set@ postProcess // used@ minkowskiSum
#pragma endregion

#pragma region global variables used during minkowskiSum (~~ intermediate)
	vector<Circle> InteriorDisks_Convolution;				// def: ???												// set&used@ minkowskiSum (and minkowskisum_id)
	Grid Grid_Trimming;										// def: ???												// set@ minkowski // used@ trimmingTest (minkowski->overlapTest->convolution_ArcSplin->)
	CacheCircles Cache_Trimming;							// def: ???												// set@ minkowski // used@ trimmingTest
#pragma endregion

#pragma region global variables storing result (~~ ouput of minkowskiSum)
	vector<deque<ArcSpline>> Model_Result;					// def: resultant minkowskiSum
	vector<bool> ModelInfo_Boundary;						// def: whether each element in model_result is boundary??? (_Result.size == _Boundary)
															//	-> true when outer boundary, not inner loop (i guess...)
#pragma endregion

#pragma region global varibles (misc)
	FILE *f;					// "time.txt"
	bool curves = false;		// not used var.
	bool ModelInfo_Identical;	// flag set true when mod1=mod2 && rotation = 0. -> calls minkowski_id instead of original.
	vector<BezierCrv> Crvr;		// not used.
	clock_t now, last;			// only used in animate_func
#pragma endregion

#define dbg
	dbg int dbgcnt = 0;

	void animate_func()
	{
		now = clock();
		clock_t seconds = now - last;

		// if last_drawn's rotation = 0, show it for 1second.
		if (ModelInfo_CurrentFrame == 1) {
			while (clock() - last < 1000);
		}

		//calculate minkowski sum.
		if ((ModelInfo_CurrentFrame == 0) && (ModelInfo_Identical))
		{
			minkowskisum_id(ModelInfo_CurrentFrame, ModelInfo_CurrentModel.second);
			cout << "minkowski_id\n";
		}
		else
			minkowskisum(ModelInfo_CurrentFrame, ModelInfo_CurrentModel.second);
		
		if(planning::forwardTime)
			ModelInfo_CurrentFrame++;
		dbgcnt = 0;
		//debug 

		//~debug

		
		last = now;

		glutPostRedisplay();
	}


	void reshape_callback(GLint w, GLint h)
	{
		if (w * 2 < h * 3) {
			wd = w;
			ht = w * 2 / 3;
			glViewport(wd * 1 / 3, 0, wd * 2 / 3, ht);
		}
		else {
			wd = h * 3 / 2;
			ht = h;
			glViewport(wd * 1 / 3, 0, wd * 2 / 3, ht);
		}
	}


	void setup_viewvolume()
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(-zoom + tx, zoom + tx, -zoom + ty, zoom + ty);
	}
	void setup_transform()
	{
	}

	void display_callback(void)
	{
		// if (rotated 360 degree) change model not rotating (=model2)
		if (ModelInfo_CurrentFrame == numofframe) {
			ModelInfo_CurrentFrame = 0;
			ModelInfo_CurrentModel.second++;

			// if (traversed all model2) change model rotating (=model1)
			if (ModelInfo_CurrentModel.second == 8) {
				ModelInfo_CurrentModel.second = 0;
				ModelInfo_CurrentModel.first++;
			}

			// if (traversed all model1) pause screen.
			if (ModelInfo_CurrentModel.first == 8) {
				numoftest++;
				if (numoftest == 1) {
					save();
					system("pause");
					exit(0);
				}
				ModelInfo_CurrentModel.first = ModelInfo_CurrentModel.second = 0;
			}
			postProcess(ModelInfo_CurrentModel.first, ModelInfo_CurrentModel.second);
		}

		glClearColor(1.0, 1.0, 1.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		setup_viewvolume();
		setup_transform();


		// Rendering code....
		glLineWidth(2.0f);
		glPointSize(2.8f);
		glColor3f(0.0f, 0.0f, 0.0f);


		glViewport(wd * 1 / 3, 0, wd * 2 / 3, ht);
		//glViewport(wd * 1 / 3, 0, 128, 128);

		/* Boundary 위에 Model을 올려놓은 그림을 그리는 코드 */
		//if (curves) {
		//	for (int u = 0; u < (int)Crv[ModelInfo_CurrentModel.second].size(); u++)
		//		Crv[ModelInfo_CurrentModel.second][u].draw();
		//}

		//if (curves) {
		//	glColor3f(1.0f, 0.0f, 0.0f);
		//	for (int i = 0; i < (int)Model_Result.size(); i++)
		//		for (int j = 0; j < (int)Model_Result[i].size(); j++)
		//			for (int u = 0; u < (int)Models_Rotated[ModelInfo_CurrentFrame].size(); u++)
		//				Models_Rotated[ModelInfo_CurrentFrame][u].draw(Model_Result[i][j].Models_Rotated_Approx.front().x[0]);

		//}

		glColor3f(0.0f, 0.0f, 0.0f);

		////////////////////////////////////////////////////////////////////////
		// plannning::output_to_file
		if(OUTPUT_TO_FILE && t2 == 1)
			planning::output_to_file::start();

		vector<vector<CircularArc>>& pushed = planning::output_to_file::ms_obj[t2];
		if(planning::output_to_file::flag)
		for (size_t i = 0, length = Model_Result.size(); i < length; i++)
		{
			if (ModelInfo_Boundary[i])
			{
				vector<CircularArc> v;
				for (auto& a : Model_Result[i])
				{
					for (auto&b : a.Arcs)
						v.push_back(b);
				}
				pushed.push_back(v);
			}
			else continue;
		}

		////////////////////////////////////////////////////////////////////////


		//debug : see order
		static int cnt = 11110;
		cnt++;
		int c = 0;

		// debug : to see if results are connected in order : conclusion : somewhat connected & not?
		if (0 && ModelInfo_CurrentFrame == 62)
		{
			if (Model_Result.size() > 0)
			{
				for (size_t i = 0; i < Model_Result.size(); i++)
				{
					for (size_t j = 0; j < Model_Result[i].size(); j++)
					{
						for (size_t k = 0; k < Model_Result[i][j].Arcs.size(); k++)
						{
							auto& a = Model_Result[i][j].Arcs[k];
							cout << i << ", " << j << ", " << k << " : " << Model_Result[i][j].Arcs[k].x[0].P[0] << ", " << Model_Result[i][j].Arcs[k].x[1].P[0] << ",, " << a.globalccw << " " << a.ccw;
							if (a.lhs)
							{
								cout
									<< ", lhs: r " << a.lhs->c.r << " quad " << a.lhs->n[0] << " ccw " << a.lhs->ccw
									<< ", rhs: r " << a.rhs->c.r << " quad " << a.rhs->n[0] << " ccw " << a.rhs->ccw;
							}
							cout << endl;

						}
						//cout << i << ", " << j << " : " << Model_Result[i][j].Arcs[0].x[0].P[0] << ", " << Model_Result[i][j].Arcs[Model_Result[i][j].Arcs.size()-1].x[1].P[0] << endl;
					}
				}
			}
		}
		//~debug
		
		// DRAW MINK
		if(planning::drawMinkowski)
		for (int i = 0; i < (int)Model_Result.size(); i++) {
			if (ModelInfo_Boundary[i])
				glColor3d(0.0, 0.0, 0.0);
			//glColor3d(i % 2, i / 2 % 2, 0.0);
			else
			{
				glColor3d(0.0, 0.0, 1.0);
			}

			for (int j = 0; j < (int)Model_Result[i].size(); j++)
			{
				for (int k = 0; k < (int)Model_Result[i][j].Arcs.size(); k++)
				{
					dbg if (c > cnt) break;
					dbg c++;
					Model_Result[i][j].Arcs[k].draw();
				}
				dbg if (c > cnt) break;
			}

			////debug
			//vector<CircularArc> ordered;
			//planning::convertMsOutput_Clockwise(Model_Result[i], ordered);
			//for (size_t i = 0, length = ordered.size(); i < length; i++)
			//{
			//	if (c > cnt) break;
			//	c++;

			//	/*if (i == 97)
			//		glColor3f(0.5f, 0.5f, 1.0f);
			//	if (i == 99)
			//		glColor3f(0, 0, 0);*/
			//	ordered[i].draw();
			//}
		}

		// TEST
		/*glBegin(GL_TRIANGLES);
		glVertex3f(1, 1, -1);
		glVertex3f(0, 1, 0);
		glVertex3f(1, 0, -1);
		glEnd();
*/
		//TEST : conc internal pixel format of opengl can be thought of as unsigned byte(1byte per component, 4byte(rgba) per pixel).
		//	data isn't change if glColor/glReadPixels are done with 1byte-components.
		//	unsigned short changes (255,256,257, 280 are all changed to 257... but why 257?)
		/*{
			static int temp = -3;
			temp+=3;

			glColor3ub(temp, temp+1, temp+2);
			glBegin(GL_TRIANGLES);
			glVertex3d(-100, 100, 0.1);
			glVertex3d(+100, 100, 0.1);
			glVertex3d(0, -300, 0.1);
			glEnd();

			unsigned char b[24];
			glReadPixels(300, 300, 2, 3, GL_RGBA, GL_UNSIGNED_BYTE, b);
			for (auto i : b)
				cerr << int(i) << " ";
			cerr << endl;
			if (b[0] != temp) cerr << "eeeeeeee: " << b[0] << " " << temp << endl;
			if (b[1] != temp+1) cerr << "eeeeeeee: " << b[1] << " " << temp+ 1 << endl;
			if (b[2] != temp + 2) cerr << "eeeeeeee: " << b[2] << " " << temp + 2 << endl;
		}*/

		/////////////////////////////////////////////////////////////////////////
		// DRAW BOUND

		if(planning::drawBoundary)
		{
			auto temp = RES;
			RES = 100;
			double r = 2;
			CircularArc a(Point(0, 0), r, Point(+1, +0), Point(+0, +1)); a.draw();
			CircularArc b(Point(0, 0), r, Point(+0, +1), Point(-1, +0)); b.draw();
			CircularArc c(Point(0, 0), r, Point(-1, +0), Point(+0, -1)); c.draw();
			CircularArc d(Point(0, 0), r, Point(+0, -1), Point(+1, +0)); d.draw();
			RES = temp;
		}

		glColor3f(0.0f, 0.7f, 0.0f);
		static int vr_cnt = 0;
		vr_cnt++;
		
		/*if(vr_cnt > 2)
		{*/
			planning::VR_IN vrin;
			planning::_Convert_MsOut_To_VrIn(Model_Result, ModelInfo_Boundary, vrin);
			planning::_Medial_Axis_Transformation(vrin);
			cout << "---------------------------------------------number of circular arcs ("<< ModelInfo_CurrentModel.first << ", " <<ModelInfo_CurrentModel.second << ", " << ModelInfo_CurrentFrame << ") : " << vrin.arcs.size() << endl;
			cout << "---------------------------------------------number of voronoi line segments : " << planning::lineSegCnt << endl;

			/////////////////////////////////////////////////////////////////////////
			//output_to_file
			if (planning::output_to_file::flag)
				planning::output_to_file::vrIn[t2] = vrin;

			/////////////////////////////////////////////////////////////////////////
			//draw coneVoronoi
			coneVor.colorType = 1;
			if (planning::keyboardflag['q'])
				coneVor.drawVoronoi(vrin);
			/////////////////////////////////////////////////////////////////////////

			// dbg
			if (1)
			{
				// tag0544
				for (auto a : vrin.arcs)
				{
					Point p(0.595017, -0.75676);
					double eps = 1e-8;
					if ((a.x[0] - p).length() < eps || (a.x[1] - p).length() < eps)
					{
						cerr << "normals " << a.n[0] << "       " << a.n[1] << endl;
						a.draw();
					}
				}
			}
		/*}*/
		glColor3f(0, 0, 0);
		/////////////////////////////////////////////////////////////////////////

		//DRAW circlesToDraw;
		if(planning::keyboardflag['e'])
		for (auto a : planning::circlesToDraw)
			a.draw();

		/////////////////////////////////////////////////////////////////////////

		// debug : to see if results are connected in order : conclusion : somewhat connected & not?
		if (0 && ModelInfo_CurrentFrame == 1)
		{
			if (Model_Result.size() > 0)
			{
				for (size_t i = 0; i < Model_Result.size(); i++)
				{
					for (size_t j = 0; j < Model_Result[i].size(); j++)
					{
						for (size_t k = 0; k < Model_Result[i][j].Arcs.size(); k++)
						{
							auto& a = Model_Result[i][j].Arcs[k];
							cout << i << ", " << j << ", " << k << " : " << Model_Result[i][j].Arcs[k].x[0].P[0] << ", " << Model_Result[i][j].Arcs[k].x[1].P[0] << ",, " << a.globalccw << " " << a.ccw;
							if (a.lhs)
							{
								cout
									<< ", lhs: r " << a.lhs->c.r << " quad " << a.lhs->n[0] << " ccw " << a.lhs->ccw
									<< ", rhs: r " << a.rhs->c.r << " quad " << a.rhs->n[0] << " ccw " << a.rhs->ccw;
							}
							cout << endl;
						}
						//cout << i << ", " << j << " : " << Model_Result[i][j].Arcs[0].x[0].P[0] << ", " << Model_Result[i][j].Arcs[Model_Result[i][j].Arcs.size()-1].x[1].P[0] << endl;
					}
				}
			}
		}
		//~debug

		////////////////////////////////////////////////////////////////////////
		// plannning::output_to_file
		if (planning::output_to_file::flag && t2 == 0)
			planning::output_to_file::end();

		////////////////////////////////////////////////////////////////////////
		//minksum approx
		if (planning::keyboardflag['c'])
		{
			glClearColor(1.0, 1.0, 1.0, 1.0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			setup_viewvolume();
			setup_transform();

			// Rendering code....
			glLineWidth(2.0f);
			glPointSize(2.8f);


			glColor3f(0.0f, 0.0f, 0.0f);
			for (auto& as : Models_Approx[t1])
			{
				auto translation = as.init();
				for(auto& as : Models_Rotated_Approx[t2])
					for (auto& arc : as.Arcs)
					{
						cd::translateArc(arc, translation).draw();
					}
			}

			glColor3f(0.0f, 0.0f, 1.0f);
			for (auto& as : Models_Approx[t1])
				as.draw();
		}

		////////////////////////////////////////////////////////////////////////

		//interest arcspline
		//debug
		static int interest = 0, interest2 = 0, cpressed = 0, vpressed = 0;
		{
			if (planning::keyboardflag['c'] && !cpressed)
			{
				interest++;
				cpressed = 1;
			}
			if (!planning::keyboardflag['c']) cpressed = 0;
			if (planning::keyboardflag['v'] && !vpressed)
			{
				interest2++;
				vpressed = 1;
			}
			if (!planning::keyboardflag['v']) vpressed = 0;
			interest = interest % Models_Rotated_Approx[ModelInfo_CurrentFrame].size();
			interest2 = interest2 % Models_Approx[ModelInfo_CurrentModel.second].size();
		}

		//dbg_out 
		cout << "interest ArcSpline index 1 & 2 : " << interest << "   " << interest2 << endl;

		// debug time(1,1,257), interest (5, 4)

		////////////////////////////////////////////////////////////////////////
		/* Left View ports */
		////////////////////////////////////////////////////////////////////////
		glColor3f(0.0f, 0.0f, 0.0f);

		if (false) // true: draw original, false : draw Approx version
		{
			glViewport(0, 0, wd * 1 / 3, ht / 2);
			for (int i = 0; i < (int)Models_Rotated[ModelInfo_CurrentFrame].size() && i < cnt; i++)
				Models_Rotated[ModelInfo_CurrentFrame][i].draw();

			glViewport(0, ht / 2, wd * 1 / 3, ht / 2);
			for (int i = 0; i < (int)Models_Imported[ModelInfo_CurrentModel.second].size() && i < cnt; i++) {
				Models_Imported[ModelInfo_CurrentModel.second][i].draw();
			}
		}
		else
		{
			glViewport(0, 0, wd * 1 / 3, ht / 2);
			if (planning::keyboardflag['x'])
			{
				for (auto& as : Models_Approx[ModelInfo_CurrentModel.second])
					for (auto& arc : as.Arcs)
					{
						auto a2 = arc;
						a2.ccw = (arc.n[0] ^ arc.n[1]) > 0; // tag maccw
						a2.draw2();
					}
			}
			else
			{
				for (int i = 0; i < (int)Models_Rotated_Approx[ModelInfo_CurrentFrame].size() && i < cnt; i++)
				{
					if (i == interest)
						glColor3f(0, 1, 1);
					else
						glColor3f(0, 0, 0);
					Models_Rotated_Approx[ModelInfo_CurrentFrame][i].draw();

				}
				if (planning::keyboardflag['z'])
					for (auto& idr : InteriorDisks_Rotated[t2])
						idr.draw();
			}
			glViewport(0, ht / 2, wd * 1 / 3, ht / 2);
			if (planning::keyboardflag['x']) // if (x) draw sweep
			{
				vector<CircularArc> temp;
				for (auto& as : Models_Approx[ModelInfo_CurrentModel.second])
					for (auto& arc : as.Arcs)
					{
						// arc.
						auto a2 = arc;
						a2.n0() = (a2.x0() - a2.c.c).normalize();
						a2.n1() = (a2.x1() - a2.c.c).normalize();
						a2.ccw = (a2.n[0] ^ a2.n[1]) > 0;
						a2.convex = a2.ccw;

						temp.push_back(a2);


						////dbg_out (tag maccw)
						//bool ccwEqual = ((arc.n[0] ^ arc.n[1]) > 0 == arc.ccw);
						//if (!ccwEqual) cout << "ccw not equal, cross, ccw :" << asin(arc.n[0] ^ arc.n[1])*180/3.1415 << " " << arc.ccw << endl;
					}
				
				////test
				////result : arc[n].x1 = arc[n+1].x0 in model_approx
				//double accumulate = 0;
				//for (size_t i = 0, length = temp.size(); i < length; i++)
				//{
				//	auto arc0 = temp[i];
				//	auto arc1 = temp[i + 1];
				//	if (i != length - 1)
				//	{
				//		auto len = (arc0.x1() - arc1.x0()).length2();
				//		//dbg_out 
				//		cout << arc0.x1() << " " << arc1.x0() << endl;
				//		accumulate += len;
				//	}
				//	temp[i].convex = !temp[i].ccw;
				//}
				////dbg_out
				//cout << "accumulate : " << accumulate << endl;

				// dbg
				/*temp.resize(0);
				double r = 0.5;*/
				//CircularArc a(Point(0, 1), r, Point(+1, +0), Point(+0, +1)); temp.push_back(a);
				////CircularArc b(Point(-0.5, 1.5), -r, Point(+0, +1), Point(-1, +0)); b.ccw = true; temp.push_back(b);
				//CircularArc b(Point(0, 1), r, Point(+0, +1), Point(-1, +0)); temp.push_back(b);
				//CircularArc c(Point(0, 1), r, Point(-1, +0), Point(-0, -1)); temp.push_back(c);
				//CircularArc d(Point(0, 1), r, Point(+0, -1), Point(+1, +0)); temp.push_back(d);

				//// convex case
				//CircularArc a(Point(0, 0.4) + Point( 0.5,  0.5), -r, Point(+1, +0), Point(+0, +1)); a.ccw = true; a.convex = false; temp.push_back(a);
				//CircularArc b(Point(0, 0.4) + Point(-0.5,  0.5), -r, Point(+0, +1), Point(-1, +0)); b.ccw = true; b.convex = false; temp.push_back(b);
				//CircularArc c(Point(0, 0.4) + Point(-0.5, -0.5), -r, Point(-1, +0), Point(-0, -1)); c.ccw = true; c.convex = false; temp.push_back(c);
				//CircularArc d(Point(0, 0.4) + Point( 0.5, -0.5), -r, Point(+0, -1), Point(+1, +0)); d.ccw = true; d.convex = false; temp.push_back(d);

				//// dbg
				//temp.resize(0);
				//double r = 0.5;
				//CircularArc a(Point(0, 0.6) + Point(+0.4, 0), r, Point(-0.8, +0.6), Point(-0.8, -0.6)); a.ccw = true; a.convex = true; temp.push_back(a);
				//CircularArc b(Point(0, 0.6) + Point(-0.4, 0), r, Point(+0.8, -0.6), Point(+0.8, +0.6)); b.ccw = true; b.convex = true; temp.push_back(b);

				auto sweep = cd::getRotationBoundary(temp, double(ms::t2));
				//dbg

				// dbg_out
				cout << sweep.size() << " " << temp.size() << endl;
				for (auto i : sweep)
					i.draw2();
			}
			else //else draw original
			{
				for (int i = 0; i < (int)Models_Approx[ModelInfo_CurrentModel.second].size() && i < cnt; i++) 
				{
					if (i == interest2)
						glColor3f(0, 1, 1);
					else
						glColor3f(0, 0, 0);
				
					Models_Approx[ModelInfo_CurrentModel.second][i].draw();
				}
			if (planning::keyboardflag['z']) //if(z) draw internal disk
				for (auto& id : InteriorDisks_Imported[ModelInfo_CurrentModel.second])
					id.draw();
			}
		}
		

		glutSwapBuffers();
	}

	void hit_index(int x, int y)
	{
	}

	void mouse_callback(int button, int action, int x, int y)
	{

		if (button == 3)
			zoom *= 1.05;

		if (button == 4)
			zoom *= 0.95;

		if (button == GLUT_LEFT_BUTTON)
		{
			auto fx = float(x - 2 * wd / 3) / (wd/3);
			auto fy = float(-y + ht / 2) / (ht / 2);
			fx = zoom * fx + tx;
			fy = zoom * fy + ty;
			cerr
				<< "***************************************************************" << endl
				<< "clicked point : " << fx << ", " << fy << "   ( zoom = " << zoom << " ) " << endl
				<< "***************************************************************" << endl;
		}

		static int grb = 0;
		static Point c;
		if (button == GLUT_RIGHT_BUTTON)
		{
			auto fx = float(x - 2 * wd / 3) / (wd / 3);
			auto fy = float(-y + ht / 2) / (ht / 2);
			fx = zoom * fx + tx;
			fy = zoom * fy + ty;

			if (grb % 2 == 0)
			{
				c = Point(fx, fy);
			}
			else
			{
				planning::circlesToDraw.push_back(Circle(c, Point(fx, fy)));
			}

			grb++;
		}

		glutPostRedisplay();
	}

	void keyboard_callback(unsigned char a, int b, int c) {
		/*
		currently used :
		w
		a s d f g
		z x c v b n m
		*/

		using namespace planning;
		if (a == 'f')
			forwardTime = (forwardTime + 1) % 2;
		if (a == 'v')
			drawVoronoiSingleBranch = (drawVoronoiSingleBranch + 1) % 2;
		if (a == 'b')
			drawBifurCircle = (drawBifurCircle + 1) % 2;
		if (a == 'm')
			drawMinkowski = (drawMinkowski + 1) % 2;
		if (a == 'n')
			drawBoundary = (drawBoundary + 1) % 2;
		if (a == 'g')
			drawTransition = (drawTransition + 1) % 2;

		keyboardflag[a] = !keyboardflag[a];

		if (a == 'r')
		{
			rfbTerminationEps *= 10.0;
			cerr << "rfbTerminationEps = " << rfbTerminationEps << endl;
		}
		if (a == 't')
		{
			rfbTerminationEps /= 10;
			cerr << "rfbTerminationEps = " << rfbTerminationEps << endl;
		}


		if (a == 'a')
			tx += 0.05 * zoom;
		if (a == 'd')
			tx -= 0.05 * zoom;
		if (a == 's')
			ty += 0.05 * zoom;
		if (a == 'w')
			ty -= 0.05 * zoom;
		//if (a == 'm')
		//	curves ^= true;
		glutPostRedisplay();
	}

	int main(int argc, char *argv[])
	{
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
		glutInitWindowSize(wd, ht);
		glutCreateWindow("Minkowski Sum");
		initialize();
		glutReshapeFunc(reshape_callback);
		glutDisplayFunc(display_callback);
		glutMouseFunc(mouse_callback);
		glutKeyboardFunc(keyboard_callback);
		glutIdleFunc(animate_func);

		glEnable(GL_DEPTH_TEST);

		glutMainLoop();
		return 0;
	}


	/*
	main func for testing RSV
	*/
	int main2(int argc, char* argv[])
	{
		wd = 1200, ht = 600;

		auto reshapeFunc = [](GLint w, GLint h)->void
		{
			if (w  < h * 2) {
				wd = w;
				ht = w / 2;
				//glViewport(wd * 1 / 3, 0, wd * 2 / 3, ht);
			}
			else {
				wd = h * 2;
				ht = h;
				//glViewport(wd * 1 / 3, 0, wd * 2 / 3, ht);
			}
		};
		auto displayFunc = [](void) -> void
		{
			
			glClearColor(1.0, 1.0, 1.0, 1.0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			setup_viewvolume();
			setup_transform();

			// Rendering code....
			glLineWidth(2.0f);
			glPointSize(2.8f);
			glColor3f(0.0f, 0.0f, 0.0f);

			// build temp
			vector<CircularArc> temp;
			{
				for (auto& as : Models_Approx[ModelInfo_CurrentModel.second])
					for (auto& arc : as.Arcs)
					{
						// arc.
						auto a2 = arc;
						a2.n0() = (a2.x0() - a2.c.c).normalize();
						a2.n1() = (a2.x1() - a2.c.c).normalize();
						a2.ccw = (a2.n[0] ^ a2.n[1]) > 0;
						a2.convex = a2.ccw;

						temp.push_back(a2);


						////dbg_out (tag maccw)
						//bool ccwEqual = ((arc.n[0] ^ arc.n[1]) > 0 == arc.ccw);
						//if (!ccwEqual) cout << "ccw not equal, cross, ccw :" << asin(arc.n[0] ^ arc.n[1])*180/3.1415 << " " << arc.ccw << endl;

						
					}

				////test
				////result : arc[n].x1 = arc[n+1].x0 in model_approx
				//double accumulate = 0;
				//for (size_t i = 0, length = temp.size(); i < length; i++)
				//{
				//	auto arc0 = temp[i];
				//	auto arc1 = temp[i + 1];
				//	if (i != length - 1)
				//	{
				//		auto len = (arc0.x1() - arc1.x0()).length2();
				//		//dbg_out 
				//		cout << arc0.x1() << " " << arc1.x0() << endl;
				//		accumulate += len;
				//	}
				//	temp[i].convex = !temp[i].ccw;
				//}
				////dbg_out
				//cout << "accumulate : " << accumulate << endl;

				// dbg
				/*temp.resize(0);
				double r = 0.5;*/
				//CircularArc a(Point(0, 1), r, Point(+1, +0), Point(+0, +1)); temp.push_back(a);
				////CircularArc b(Point(-0.5, 1.5), -r, Point(+0, +1), Point(-1, +0)); b.ccw = true; temp.push_back(b);
				//CircularArc b(Point(0, 1), r, Point(+0, +1), Point(-1, +0)); temp.push_back(b);
				//CircularArc c(Point(0, 1), r, Point(-1, +0), Point(-0, -1)); temp.push_back(c);
				//CircularArc d(Point(0, 1), r, Point(+0, -1), Point(+1, +0)); temp.push_back(d);

				//// convex case
				//CircularArc a(Point(0, 0.4) + Point( 0.5,  0.5), -r, Point(+1, +0), Point(+0, +1)); a.ccw = true; a.convex = false; temp.push_back(a);
				//CircularArc b(Point(0, 0.4) + Point(-0.5,  0.5), -r, Point(+0, +1), Point(-1, +0)); b.ccw = true; b.convex = false; temp.push_back(b);
				//CircularArc c(Point(0, 0.4) + Point(-0.5, -0.5), -r, Point(-1, +0), Point(-0, -1)); c.ccw = true; c.convex = false; temp.push_back(c);
				//CircularArc d(Point(0, 0.4) + Point( 0.5, -0.5), -r, Point(+0, -1), Point(+1, +0)); d.ccw = true; d.convex = false; temp.push_back(d);

				//// dbg
				//temp.resize(0);
				//double r = 0.5;
				//CircularArc a(Point(0, 0.6) + Point(+0.4, 0), r, Point(-0.8, +0.6), Point(-0.8, -0.6)); a.ccw = true; a.convex = true; temp.push_back(a);
				//CircularArc b(Point(0, 0.6) + Point(-0.4, 0), r, Point(+0.8, -0.6), Point(+0.8, +0.6)); b.ccw = true; b.convex = true; temp.push_back(b);
			}

			// build internal circles
			double r = 0.2 - 1e-8;
			Point
				center0(0, 0),
				center1(0, 0.3),
				center2(0, -0.3);
			Circle
				circle0(center0, r),
				circle1(center1, r),
				circle2(center2, r);

			//dbg_out : n of conv/conc
			int conv = 0, conc = 0;
			for (auto& a : temp)
				if (a.convex)
					conv++;
				else
					conc++;
			cout << " temp: No of conv/conv : " << conv << "   " << conc << endl;
			// ~dbg_out

			// dbg : test info
			ms::t2 = 1;
			cout << "****** Model : " << ModelInfo_CurrentModel.second << "   /   Rotation(degree) :  " << ms::t2 << endl;
			// ~dbg

			// dbg : write rectangle to file
			static bool write = false;
			ofstream aR ;
			ofstream aRR;
			ofstream cRR;

			if (write)
			{
				aR.open ("arcsRect.txt");
				aRR.open("arcsRectRSV.txt");
				cRR.open("circRectRSV.txt");


				auto writeArc = [&](CircularArc& c)	//write arc to file
				{
					aR << scientific << setprecision(20);
					aR << c.c.c.P[0] << " " << c.c.c.P[1] << " " << c.c.r << " " << atan2(c.n[0].P[1], c.n[0].P[0]) << " " << atan2(c.n[1].P[1], c.n[1].P[0]) << " " << c.ccw << endl;
				};

				aR << temp.size() << endl;
				for (auto arc : temp)
				{
					writeArc(arc);
				}
			}
			if (write)
			{
				auto writeCircle = [&](Circle& c)	//write arc to file
				{
					cRR << fixed << setprecision(20);
					cRR << c.c.P[0] << " " << c.c.P[1] << " " << c.r << endl;
				};

				cRR << 3 << endl;
				writeCircle(circle0);
				writeCircle(circle1);
				writeCircle(circle2);
			}
			// ~dbg

			// VIEWPORT 0
			glViewport(0, 0, wd / 2, ht);
			{
				auto sweep = cd::getRotationBoundary(temp, double(ms::t2), 1);
				for (size_t i = 0, length = sweep.size(); i < length; i++)
				{
					//debug
					if (sweep[i].convex)
						glColor3f(0, 0, 0);
					else
						glColor3f(0, 1, 0);

					if (i == 0)
					{
						glColor3f(0, 1, 0);
						cout << "interest arc endpoint : " << sweep[i].x0() << "    " << sweep[i].x1() << endl;
					}
					else
						glColor3f(0, 0, 0);
					sweep[i].draw2(0.5);

				}

				//dbg_out : draw circ
				circle0.draw();
				circle1.draw();
				circle2.draw();

				// dbg_out
				cout << "before trimming size : " << sweep.size() << endl;
			}

			// VIEWPORT 1
			glViewport(wd / 2, 0, wd / 2, ht);
			{
				auto sweep2 = cd::getRotationBoundary(temp, double(ms::t2), 2);
				// dbg_out
				cout << "divArc size : " << sweep2.size() << endl;
				glColor3f(0, 0, 0);
				//debug
				static int interest = 0, cpressed = 0, vpressed = 0;
				{
					if (planning::keyboardflag['c'] && !cpressed)
					{
						interest++;
						cpressed = 1;
					}
					if (!planning::keyboardflag['c']) cpressed = 0;
					if (planning::keyboardflag['v'] && !vpressed)
					{
						interest--;
						vpressed = 1;
					}
					if (!planning::keyboardflag['v']) vpressed = 0;
					interest = interest % sweep2.size();
				}
				for (size_t i = 0, length = sweep2.size(); i < length; i++)
				{
					////debug
					//if (sweep2[i].convex)
					//	glColor3f(0, 0, 0);
					//else
					//	glColor3f(0, 1, 0);

					// debug
					{

						if (i == interest)
						{
							glColor3f(0, 1, 0);
							cout << "interest " << interest << " : interest arc endpoint : " << sweep2[i].x0() << "    " << sweep2[i].x1() << endl;
						}
						else
							glColor3f(0, 0, 0);
					}
					sweep2[i].draw2(0.5);
				}

				if (planning::keyboardflag['x'])
				{
					glColor3f(0, 0, 1);
					for (auto as : Models_Approx[ModelInfo_CurrentModel.second])
						for (auto a : as.Arcs)
							a.draw();
				}

				// dbg : write rectangle
				if (write)
				{
					auto writeArc = [&](CircularArc& c)	//write arc to file
					{
						aRR << fixed << setprecision(20);
						aRR << c.c.c.P[0] << " " << c.c.c.P[1] << " " << c.c.r << " " << atan2(c.n[0].P[1], c.n[0].P[0]) << " " << atan2(c.n[1].P[1], c.n[1].P[0]) << " " << c.ccw << endl;
					};

					// build list of indices of arcs to use.
					set<int> untrimmed;
				
					untrimmed.insert(1);
					untrimmed.insert(3);
					untrimmed.insert(6);
					untrimmed.insert(8);
					untrimmed.insert(11);
					untrimmed.insert(13);
					untrimmed.insert(16);
					untrimmed.insert(18);

					for (int i = 20; i < 32; i++)
						untrimmed.insert(i);

					// fout
					aRR << untrimmed.size() << endl;
					for (int i = 0; i < 32; i++)
						if (untrimmed.find(i) != untrimmed.end())
							writeArc(sweep2[i]);
				}
				// ~dbg
			}

			// dbg
			if (write)
			{
				write = false; // this is last write;
				aR.close();
				aRR.close();
				cRR.close();
			}

			// ~dbg

			glutSwapBuffers();
		};
		auto    idleFunc = []() -> void
		{
			clock_t now = clock();

			// if last_drawn's rotation = 0, show it for 1second.
			if (ModelInfo_CurrentFrame == numofframe - 1) {
				while (clock() - now < 500);
			}

			if (planning::forwardTime)
				ModelInfo_CurrentFrame++;
			if (ModelInfo_CurrentFrame == numofframe)
			{
				ModelInfo_CurrentModel.second = (ModelInfo_CurrentModel.second + 1) % 8;
				ModelInfo_CurrentFrame = 0;
			}

			// debug
			CircularArc test;
			test.convex = false;
			CircularArc B(test);
			CircularArc C;
			C = test;
			cout << "convexity test " << test.convex << B.convex << C.convex << endl;
			// ~debug

			glutPostRedisplay();
		};


		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
		glutInitWindowSize(wd, ht);
		glutCreateWindow("RSV");
		initialize();
		t2 = t1 = t0 = 0;
		//override model0
		{
			double
				r0 = 100,
				r1 = 0.1;

			Point // from 1st quadrant, in ccw dir
				p0( 0.2,  0.5),
				p1(-0.2,  0.5),
				p2(-0.2, -0.5),
				p3( 0.2, -0.5);
			Point
				c0(0, -r0),
				c1(+r0, 0),
				c2(0, +r0),
				c3(-r0, 0);
			
			vector<CircularArc> 
				vec;

			// semi-straight arcs
			vec.push_back(cd::constructArc(c0, p0 - Point(r1, 0), p1 + Point(r1, 0), true));
			vec.push_back(cd::constructArc(c1, p1 - Point(0, r1), p2 + Point(0, r1), true));
			vec.push_back(cd::constructArc(c2, p2 + Point(r1, 0), p3 - Point(r1, 0), true));
			vec.push_back(cd::constructArc(c3, p3 + Point(0, r1), p0 - Point(0, r1), true));

			// corner-arcs
			vec.push_back(cd::constructArc(p0 + Point(-r1, -r1), r1, PI * 0.0, PI * 0.5));
			vec.push_back(cd::constructArc(p1 + Point(+r1, -r1), r1, PI * 0.5, PI * 1.0));
			vec.push_back(cd::constructArc(p2 + Point(+r1, +r1), r1, PI * 1.0, PI * 1.5));
			vec.push_back(cd::constructArc(p3 + Point(-r1, +r1), r1, PI * 1.5, PI * 2.0));

			for (auto& a : vec)
				a.convex = true;

			ArcSpline as;
			as.Arcs = vec;
			vector<ArcSpline> asvec; 
			asvec.push_back(as);

			Models_Approx[0] = asvec;

			//debug
			auto
				t0 = p2 - c2,
				t2 = p3 - c2,
				t1 = Point(0, 0) - c2;
			auto
				th0 = atan2(t0.y(), t0.x()),
				th1 = atan2(t1.y(), t1.x()),
				th2 = atan2(t2.y(), t2.x());

			cout << "Thetas in square " << th0 << " " << th1 << " " << th2 << endl;
		}

		glutReshapeFunc(reshapeFunc);
		glutDisplayFunc(displayFunc);
		glutMouseFunc(mouse_callback);
		glutKeyboardFunc(keyboard_callback);
		glutIdleFunc(idleFunc);

		glEnable(GL_DEPTH_TEST);

		glutMainLoop();
		return 0;
	}

}