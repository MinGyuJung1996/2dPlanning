#include "MS2D.h"
#include <iostream>
#include "voronoi.hpp"

namespace ms {

	/* OpenGL Parameters */
	GLsizei wd = 900, ht = 600;
	double zoom = 1.4;
	double tx, ty;
	using namespace std;

	/* Comments - MS2D.h 참고 */

#pragma region global variables related to time.
	int ModelInfo_CurrentFrame = 44;			// 7,7,261 case
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
		glClear(GL_COLOR_BUFFER_BIT);
		setup_viewvolume();
		setup_transform();


		// Rendering code....
		glLineWidth(2.0f);
		glPointSize(2.8f);
		glColor3f(0.0f, 0.0f, 0.0f);


		glViewport(wd * 1 / 3, 0, wd * 2 / 3, ht);

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
		
		if(planning::drawMinkowski)
		for (int i = 0; i < (int)Model_Result.size(); i++) {
			if (ModelInfo_Boundary[i])
				glColor3d(0.0, 0.0, 0.0);
			//glColor3d(i % 2, i / 2 % 2, 0.0);
			else
			{
				glColor3d(0.0, 0.0, 1.0);
			}

			/*for (int j = 0; j < (int)Model_Result[i].size(); j++)
			{
				for (int k = 0; k < (int)Model_Result[i][j].Arcs.size(); k++)
				{
					dbg if (c > cnt) break;
					dbg c++;
					Model_Result[i][j].Arcs[k].draw();
				}
				dbg if (c > cnt) break;
			}*/

			//debug
			vector<CircularArc> ordered;
			planning::convertMsOutput_Clockwise(Model_Result[i], ordered);
			for (size_t i = 0, length = ordered.size(); i < length; i++)
			{
				if (c > cnt) break;
				c++;

				/*if (i == 97)
					glColor3f(0.5f, 0.5f, 1.0f);
				if (i == 99)
					glColor3f(0, 0, 0);*/
				ordered[i].draw();
			}
		}


		/////////////////////////////////////////////////////////////////////////

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
		
		if(vr_cnt > 2)
		{
			planning::VR_IN vrin;
			planning::_Convert_MsOut_To_VrIn(Model_Result, ModelInfo_Boundary, vrin);
			cout << "---------------------------------------------number of circular arcs ("<< ModelInfo_CurrentModel.first << ", " <<ModelInfo_CurrentModel.second << ", " << ModelInfo_CurrentFrame << ") : " << vrin.arcs.size() << endl;
			planning::_Medial_Axis_Transformation(vrin);
		}
		glColor3f(0, 0, 0);
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

		glColor3f(0.0f, 0.0f, 0.0f);

		if (true) // true: draw original, false : draw Approx version
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
			for (int i = 0; i < (int)Models_Rotated_Approx[ModelInfo_CurrentFrame].size() && i < cnt; i++)
				Models_Rotated_Approx[ModelInfo_CurrentFrame][i].draw();

			glViewport(0, ht / 2, wd * 1 / 3, ht / 2);
			for (int i = 0; i < (int)Models_Approx[ModelInfo_CurrentModel.second].size() && i < cnt; i++) {
				Models_Approx[ModelInfo_CurrentModel.second][i].draw();
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

		glutPostRedisplay();
	}

	void keyboard_callback(unsigned char a, int b, int c) {
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
		glutMainLoop();
		return 0;
	}

}