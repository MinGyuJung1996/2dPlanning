#include "MS2D.h"
#include <iostream>
#include <iomanip>
#include "voronoi.hpp"
#include "collision detection.hpp"
#include "xyt-cs.hpp"

namespace rendering3D
{
	using namespace std;
	using CircularArc = ms::CircularArc;
	using Circle = ms::Circle;
	using Point = ms::Point;
	int ht;
	int wd;
	


	/********************************************************************************************************************
	**																												   **
	**																												   **
	**																												   **
	**																												   **
	********************************************************************************************************************/

	void setup_veiwvolume2()
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		double
			fovyInDegrees = 30.0,
			aspectRatio = 1.0, // x/y
			znear = 0.001,
			zfar = 100.0;
		//gluPerspective(fovy, aspect, znear, zfar); // this breaks the code which turns bezier to arcs -> this seems like a v ery complicated bug... kind of circumvent it for now

		double matrix[16];

		// calculate perspective by myself // kind of from https://www.khronos.org/opengl/wiki/GluPerspective_code
		{
			// 1.
			double ymax, xmax;
			ymax = znear * tan(fovyInDegrees * PI / 360.0);
			// ymin = -ymax;
			// xmin = -ymax * aspectRatio;
			xmax = ymax * aspectRatio;

			// 2.
			double
				left = -xmax,
				right = xmax,
				bottom = -ymax,
				top = ymax;

			double temp, temp2, temp3, temp4;
			temp = 2.0 * znear;
			temp2 = right - left;
			temp3 = top - bottom;
			temp4 = zfar - znear;
			matrix[0] = temp / temp2;
			matrix[1] = 0.0;
			matrix[2] = 0.0;
			matrix[3] = 0.0;
			matrix[4] = 0.0;
			matrix[5] = temp / temp3;
			matrix[6] = 0.0;
			matrix[7] = 0.0;
			matrix[8] = (right + left) / temp2;
			matrix[9] = (top + bottom) / temp3;
			matrix[10] = (-zfar - znear) / temp4;
			matrix[11] = -1.0;
			matrix[12] = 0.0;
			matrix[13] = 0.0;
			matrix[14] = (-temp * zfar) / temp4;
			matrix[15] = 0.0;

		}
		glLoadMatrixd(matrix);
	
	}

	class cam3d
	{
	public:
		double camera[9]; // pos, forward, up(forward X left)
		double leftDir[3];
		double elevation;
		bool *key, *key2;

		/* adjustables */
		double mult		;
		double dtheta	;
		double dphi		;
		double cdtheta	;
		double sdtheta	;
		double phimax	;
		double phimin	;

	public:

		/* 3d math funcs */
		inline void multAdd(double* x, double* y, double m)
		{
			x[0] += m * y[0];
			x[1] += m * y[1];
			x[2] += m * y[2];
		};
		inline void cross(double* lhs, double* rhs, double* result)
		{
			result[0] = lhs[1] * rhs[2] - lhs[2] * rhs[1];
			result[1] = lhs[2] * rhs[0] - lhs[0] * rhs[2];
			result[2] = lhs[0] * rhs[1] - lhs[1] * rhs[0];
		}

		/* alias */
		inline double* pos() { return camera; }
		inline double* forward() { return camera + 3; }
		inline double* left() { return leftDir; }
		inline double* up() { return camera + 3; }

		/* con/destructor */
		cam3d() = default;
		inline cam3d(bool keyNow[], bool keyLast[])
		{
			initialize(keyNow, keyLast);
		}
		~cam3d() = default;

		/* funcs */
		void initialize(bool keyNow[], bool keyLast[])
		{

			key = keyNow;
			key2 = keyLast;
			camera[0] = 0.0;
			camera[1] = 0.0;
			camera[2] = 0.0;

			camera[3] = 0.0;
			camera[4] = 1.0;
			camera[5] = 0.0;

			camera[6] = 0.0;
			camera[7] = 0.0;
			camera[8] = 1.0;

			leftDir[0] = 0.0;
			leftDir[1] = -1.0;
			leftDir[2] = 0.0;

			elevation = 0.0;

			
			mult	= 0.5;
			dphi	= 6.0 * PI / 360.0;
			dtheta	= 6.0 * PI / 360.0;
			cdtheta	= cos(dtheta);
			sdtheta	= sin(dtheta);
			phimax	= +150.0 * PI / 360.0;
			phimin	= -150.0 * PI / 360.0;
		}

		void update()
		{

			
			char temp;

			// 1. tranlsate camera(wasd rf)
			temp = 'r';
			if (key[temp] != key2[temp])
				multAdd(camera, camera + 3, mult);
			temp = 'f';
			if (key[temp] != key2[temp])
				multAdd(camera, camera + 3, -mult);
			temp = 'a';
			if (key[temp] != key2[temp])
				multAdd(camera, leftDir, mult);
			temp = 'd';
			if (key[temp] != key2[temp])
				multAdd(camera, leftDir, -mult);
			temp = 'w';
			if (key[temp] != key2[temp])
				multAdd(camera, camera + 6, mult);
			temp = 's';
			if (key[temp] != key2[temp])
				multAdd(camera, camera + 6, -mult);

			// 2. rotate camera(ijkl)
			temp = 'j';
			if (key[temp] != key2[temp])
			{
				double x = cdtheta * camera[3] - sdtheta * camera[4];
				double y = sdtheta * camera[3] + cdtheta * camera[4];
				camera[3] = x;
				camera[4] = y;
			}

			temp = 'l';
			if (key[temp] != key2[temp])
			{
				double x = +cdtheta * camera[3] + sdtheta * camera[4];
				double y = -sdtheta * camera[3] + cdtheta * camera[4];
				camera[3] = x;
				camera[4] = y;
			}

			temp = 'i';
			if (key[temp] != key2[temp])
			{
				elevation += dphi;
				if (elevation > phimax)
					elevation = phimax;
			}

			temp = 'k';
			if (key[temp] != key2[temp])
			{
				elevation -= dphi;
				if (elevation < phimin)
					elevation = phimin;
			}

			// 3. camera speed;
			temp = 'z';
			if (key[temp] != key2[temp])
			{
				mult = mult * 2;
			}
			temp = 'x';
			if (key[temp] != key2[temp])
			{
				mult = mult * 0.5;
			}

			//set cam-view-dir-with elevation (also called forward dir)
			{
				double
					x = camera[3],
					y = camera[4],
					z = sin(elevation);
				double
					l = sqrt(x * x + y * y);
				x = cos(elevation) * x / l;
				y = cos(elevation) * y / l;
				camera[3] = x;
				camera[4] = y;
				camera[5] = z;
			}

			// set cam-left-dir
			{
				leftDir[0] = -camera[4];
				leftDir[1] = +camera[3];
				leftDir[2] = 0.0;
				double ldl =
					leftDir[0] * leftDir[0] +
					leftDir[1] * leftDir[1] +
					leftDir[2] * leftDir[2];
				ldl = sqrt(ldl);
				leftDir[0] /= ldl;
				leftDir[1] /= ldl;
			}

			// set cam-up-dir
			{
				cross(camera + 3, leftDir, camera + 6);
			}

			cout << "camera : " << camera[0] << " " << camera[1] << " " << camera[2] << " // "
				<< camera[3] << " " << camera[4] << " " << camera[5] << " // "
				<< camera[6] << " " << camera[7] << " " << camera[8] << " " << endl;
		}

		void setViewMat()
		{
			// gllookat doesn't work...:
			// referenced https://www.khronos.org/opengl/wiki/GluLookAt_code

			double* pos		= camera;
			double* forward = camera + 3;
			double* side	= leftDir;
			double* up		= camera + 6;

			double matrix[16];
			
			matrix[0]  = -side[0];
			matrix[4]  = -side[1];
			matrix[8]  = -side[2];
			matrix[12] = 0.0;
			// -------------------
			matrix[1]  = up[0];
			matrix[5]  = up[1];
			matrix[9]  = up[2];
			matrix[13] = 0.0;
			// -------------------
			matrix[2]  = -forward[0];
			matrix[6]  = -forward[1];
			matrix[10] = -forward[2];
			matrix[14] = 0.0;
			// -------------------
			matrix[3]  = 0.0;
			matrix[7]  = 0.0;
			matrix[11] = 0.0;
			matrix[15] = 1.0;
			// -------------------
			matrix[12] = -(matrix[0] * pos[0] + matrix[4] * pos[1] + matrix[8]  * pos[2]);
			matrix[13] = -(matrix[1] * pos[0] + matrix[5] * pos[1] + matrix[9]  * pos[2]);
			matrix[14] = -(matrix[2] * pos[0] + matrix[6] * pos[1] + matrix[10] * pos[2]);

			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			glLoadMatrixd(matrix);
		}
	};


	/********************************************************************************************************************
	**																												   **
	**																												   **
	**																												   **
	**																												   **
	********************************************************************************************************************/

	/*
	Def : render loop to see configuration space obstacles in 3d.
	*/
	namespace renderCSObjectGlobal
	{
		std::vector<decltype(ms::Model_Result)> mink;
		std::vector<decltype(ms::ModelInfo_Boundary)> isBoundary;
		double camera[9]; // center-xyz, view-dir-xyz, up-xyz // notice that this is not a direct input to gluLookat
		double elevation = 0;
		cam3d camera3d;

	}
	void renderCSObject(
		int argc,
		char* argv[],
		std::vector<decltype(ms::Model_Result)>& MRs,
		std::vector<decltype(ms::ModelInfo_Boundary)>& MIBs
		)
	{
		// 0. init stuff
		using namespace renderCSObjectGlobal;
		mink = MRs;
		isBoundary = MIBs;
		wd = ht = 800;
		double xtoy = 1.0;
		{
			camera[0] = 0.0;
			camera[1] = 0.0;
			camera[2] = 0.0;
		
			camera[3] = 0.0;
			camera[4] = 1.0;
			camera[5] = 0.0;
			
			camera[6] = 0.0;
			camera[7] = 0.0;
			camera[8] = 1.0;
		}

		cam3d cam(planning::keyboardflag, planning::keyboardflag_last);
		camera3d = cam;

		// 0-1. init gl context
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
		glutInitWindowSize(wd, ht);
		glutCreateWindow("CS Obstacles");

		//cam = cam3d(planning::keyboardflag, planning::keyboardflag_last);
		
		// 0-2 make lambdas.
		auto reshapeFunc = [](GLint w, GLint h)
		{
			// later
		};
		auto idleFunc = []()
		{
			// later


			
			glutPostRedisplay();
		};
		auto mouseFunc = [](int button, int action, int x, int y)
		{
		
		};
		auto keyboardFunc = [](unsigned char a, int b, int c)
		{
			planning::keyboardflag[a] = !planning::keyboardflag[a];
			// pressing a once will flip keybaord flag btw 0 and 1
			// pressing it for a long time will make it oscillate 010101
		};
		auto displayFunc = []()
		{
			// 1. clear
			glClearColor(0.0, 1.0, 1.0, 1.0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glColor3f(0.0f, 0.0f, 0.0f);
			
			// 2. set matrix
			// 2-1. perspective;
			setup_veiwvolume2();
			
			// 2-2. modelview
			
			//glMatrixMode(GL_MODELVIEW);
			//glLoadIdentity();
			//gluLookAt(
			//	camera[0], camera[1], camera[2],
			//	camera[0] + camera[3], camera[1] + camera[4], camera[2] + camera[5],
			//	camera[6], camera[7], camera[8]);

			camera3d.setViewMat();
			
			// 3. set viewport and draw
			//glViewport(wd * 1 / 3, 0, wd * 2 / 3, ht);
			
			// 3-1. VIEWPORT 0
			glViewport(0, 0, wd, ht);

			// 3-1-1. light
			glEnable(GL_LIGHTING);
			float ambLight[4] = { 0.1, 0.1, 0.1, 1.0 };
			float difLight[4] = { 0.8, 0.8, 0.8, 1.0 };
			float posLight[4] = { camera[0], camera[1], camera[2], 1.0 };
			glLightfv(GL_LIGHT0, GL_AMBIENT, ambLight);
			glLightfv(GL_LIGHT0, GL_DIFFUSE, difLight);
			glLightfv(GL_LIGHT0, GL_POSITION, posLight);
			glEnable(GL_LIGHT0);
			float lightModel2Side = 1;

			float red[4] = { 1,0,0,1 };
			float red2[4] = { .4, .5, .2, 1.0 };
			glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
			glMaterialfv(GL_BACK , GL_AMBIENT_AND_DIFFUSE, red2);

			// 3-1-2. real draw
			//glColor3f(1, 0, 0);

			//glNormal3f(0, -1, 0);
			//glVertex3f(0, 1, 0.1);
			//glVertex3f(0, 1, 0);
			//glVertex3f(0.1, 1, 0);

			double dz = 1.0 / 360.f;
			double z;
			int count = 0;
			int RES = 3;
			auto drawArcToQuads = [&](CircularArc& c)
			{
				for (int i = 0; i < RES; i++) {
					Point p = c((double)i / RES);
					Point q = c((double)(i + 1) / RES);

					Point d = p - q;
					d.normalize();

					float normal[3] = { -d.y(), d.x(), 0.0 };

					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(normal);
					glVertex3f(p.x(), p.y(), z);
					glVertex3f(p.x(), p.y(), z + dz);
					glVertex3f(q.x(), q.y(), z);
					glVertex3f(q.x(), q.y(), z + dz);
					count++;
					count++;
					glEnd();

				}
			};


			for (int i = 0; i <1 /* ms::numofframe*/; i++)
			{
				z = i / 360.0f;
				for (auto& loops : mink)
				{
					for (auto& loop : loops)
					{
						for (auto& as : loop)
							for (auto& arc : as.Arcs)
							{
								drawArcToQuads(arc);
							}
					}
				}
			}
			cout << "tri count : " << count << endl;




			
			// 4. swap
			glutSwapBuffers();

			// 10. camera 
			if(false)
			{
				// 1. take care of camera
				auto& key = planning::keyboardflag;
				auto& key2 = planning::keyboardflag_last;

				// 1-1. left
				double leftDir[3];
				leftDir[0] = -camera[4];
				leftDir[1] = camera[3];
				leftDir[2] = 0.0;
				double ldl =
					leftDir[0] * leftDir[0] +
					leftDir[1] * leftDir[1] +
					leftDir[2] * leftDir[2];
				ldl = sqrt(ldl);
				leftDir[0] /= ldl;
				leftDir[1] /= ldl;

				// 1-2. simple vec mult add for 3-arr-double
				// x = x + y * m;
				auto multAdd = [&](double* x, double* y, double m)
				{
					x[0] += m * y[0];
					x[1] += m * y[1];
					x[2] += m * y[2];
				};

				// param
				double mult = 0.03;
				double dtheta = 6.0 * PI / 360.0;
				double dphi   = 6.0 * PI / 360.0;
				double cdtheta = cos(dtheta);
				double sdtheta = sin(dtheta);
				double phimax = +150.0 * PI / 360.0;
				double phimin = -150.0 * PI / 360.0;

				char temp;

				temp = 'w';
				if (key[temp] != key2[temp])
					multAdd(camera, camera + 3, mult);
				temp = 's';
				if (key[temp] != key2[temp])
					multAdd(camera, camera + 3, -mult);
				temp = 'a';
				if (key[temp] != key2[temp])
					multAdd(camera, leftDir, mult);
				temp = 'd';
				if (key[temp] != key2[temp])
					multAdd(camera, leftDir, -mult);
				temp = 'r';
				if (key[temp] != key2[temp])
					multAdd(camera, camera + 6, mult);
				temp = 'f';
				if (key[temp] != key2[temp])
					multAdd(camera, camera + 6, -mult);

				temp = 'j';
				if (key[temp] != key2[temp])
				{
					double x = cdtheta * camera[3] - sdtheta * camera[4];
					double y = sdtheta * camera[3] + cdtheta * camera[4];
					camera[3] = x;
					camera[4] = y;
				}

				temp = 'l';
				if (key[temp] != key2[temp])
				{
					double x = +cdtheta * camera[3] + sdtheta * camera[4];
					double y = -sdtheta * camera[3] + cdtheta * camera[4];
					camera[3] = x;
					camera[4] = y;
				}

				temp = 'i';
				if (key[temp] != key2[temp])
				{
					elevation += dphi;
					if (elevation > phimax)
						elevation = phimax;
				}

				temp = 'k';
				if (key[temp] != key2[temp])
				{
					elevation -= dphi;
					if (elevation < phimin)
						elevation = phimin;
				}

				//set cam-view-dir-with elevation
				{
					double
						x = camera[3],
						y = camera[4],
						z = sin(elevation);
					double
						l = sqrt(x * x + y * y);
					x = cos(elevation) * x / l;
					y = cos(elevation) * y / l;
					camera[3] = x;
					camera[4] = y;
					camera[5] = z;
				}



				cout << "camera : " << camera[0] << " " << camera[1] << " " << camera[2] << " // "
					<< camera[3] << " " << camera[4] << " " << camera[5] << " // "
					<< camera[6] << " " << camera[7] << " " << camera[8] << " " << endl;
			}

			camera3d.update();

			// 99. set keyboardFlagOld
			for (int i = 0; i < 256; i++)
				planning::keyboardflag_last[i] = planning::keyboardflag[i];
		};
		
		// 0-3. set labmdas
		glutReshapeFunc(reshapeFunc);
		glutDisplayFunc(displayFunc);
		glutMouseFunc(mouseFunc);
		glutKeyboardFunc(keyboardFunc);
		glutIdleFunc(idleFunc);
		
		glEnable(GL_DEPTH_TEST);
		
		glutMainLoop();
	}




	/********************************************************************************************************************
	**																												   **
	**													~~Main 3~~													   **
	**																												   **
	**																												   **
	********************************************************************************************************************/

	/*
	Def : main3 := make 3d configuration space obstacle using there equations.
	*/
	namespace main3
	{
		cam3d cam;
		vector<csSurf> out;
		vector<vector<triNormal>*> drawingSets;
		int t2 = 0;
		float colors[300];
		vector<voronoiCalculator> vcs;
		vector<vector<Point>> bifurPts;
		vector<xyt> bifurDrawingSetTemp;
		vector<voronoiCalculatorResultG> topoVorRes;

		vector<xyt> drawingSetVorVert;
		vector<xyt> drawingSetVorNorm;
		vector<std::pair<int, int>> drawingSetVorIdx;
		vector<int> drawingSetVorIdxFrom; // lesser SliceNo that was used to form the corresponding drawingSetVorIdx;


		void tessellateVoronoiCurvePair(int in_nSamples, vector<VoronoiEdge>& in_vec0, vector<VoronoiEdge>& in_vec1, double in_z0, double in_z1, vector<xyt>& out_vert, vector<xyt>& out_norm);


		void reshapeFunc(GLint w, GLint h)
		{
			// later
		};
		void idleFunc()
		{
			// later

			glutPostRedisplay();
		};
		void mouseFunc(int button, int action, int x, int y)
		{

		};
		void keyboardFunc(unsigned char a, int b, int c)
		{
			planning::keyboardflag[a] = !planning::keyboardflag[a];
			// pressing a once will flip keybaord flag btw 0 and 1
			// pressing it for a long time will make it oscillate 010101
		};
		void displayFunc()
		{
			// 1. clear
			glClearColor(0.0, 1.0, 1.0, 1.0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glColor3f(0.0f, 0.0f, 0.0f);

			// 1-2. time management
			if (planning::keyboardflag['o'] != planning::keyboardflag_last['o'])
				t2++;
			if (planning::keyboardflag['p'] != planning::keyboardflag_last['p'])
				t2--;
			if (t2 >= ms::numofframe)
				t2 = t2 % ms::numofframe;
			if (t2 < 0)
				t2 += ms::numofframe;

			cout << " !!! : current rotation Degree : " << t2 << endl;

			// 2. VIEWPORT 0
			glViewport(0, 0, wd / 2, ht);
			glScissor(0, 0, wd / 2, ht);
			
			// 2-1. matrix
			setup_veiwvolume2();
			cam.setViewMat();

			// 2-2. light
			glEnable(GL_LIGHTING);
			float ambLight[4] = { 0.1, 0.1, 0.1, 1.0 };
			float difLight[4] = { 0.8, 0.8, 0.8, 1.0 };
			float posLight[4] = { cam.pos()[0], cam.pos()[1], cam.pos()[2], 1.0 };
			glLightfv(GL_LIGHT0, GL_AMBIENT, ambLight);
			glLightfv(GL_LIGHT0, GL_DIFFUSE, difLight);
			glLightfv(GL_LIGHT0, GL_POSITION, posLight);
			glEnable(GL_LIGHT0);

			float red[4] = { 1,0,0,1 };
			float red2[4] = { .7, .3, .7, 1.0 };
			glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, red);
			glMaterialfv(GL_BACK , GL_AMBIENT_AND_DIFFUSE, red);


			// 2-3. draw tri;

			//// case, evaluated until tess
			//for (auto& surf : out)
			//{
			//	auto& s = surf.surface();
			//	auto& n = surf.normals();

			//	int I = s.size();
			//	int J = s[0].size();

			//	for (int i = 0; i < (I - 1); i++)
			//	{
			//		glBegin(GL_TRIANGLE_STRIP);
			//		for (int j = 0; j < J; j++)
			//		{
			//			glNormal3dv(n[i+0][j].v3d());
			//			glVertex3dv(s[i+0][j].v3d());
			//			glNormal3dv(n[i+1][j].v3d());
			//			glVertex3dv(s[i+1][j].v3d());
			//		}
			//		glEnd();
			//	}

			//}

			// case, evaluated until drawingSet.
			if (!planning::keyboardflag['b'])
			{
				glBegin(GL_TRIANGLES);
				for (auto& ds : drawingSets)
				{
					for (auto& tn : *ds)
					{
						glNormal3dv(tn.n0());
						glVertex3dv(tn.x0());
						glNormal3dv(tn.n1());
						glVertex3dv(tn.x1());
						glNormal3dv(tn.n2());
						glVertex3dv(tn.x2());
					}
				}
				glEnd();
			}

			// 2-4. draw -pi/pi plane

			if (planning::keyboardflag['c'])
			{
				glBegin(GL_TRIANGLES);
				glVertex3d(-100, -100, +3.14159297);
				glVertex3d(100 , 0   , +3.14159297);
				glVertex3d(0   , 100 , +3.14159297);
				glVertex3d(-100, -100, -3.14159297);
				glVertex3d(100 , 0   , -3.14159297);
				glVertex3d(0   , 100 , -3.14159297);
				glEnd();
			}

			if (planning::keyboardflag['v'])
			{
				glBegin(GL_TRIANGLES);
				if (t2 > 180)
				{
					glVertex3d(-100, -100, (double(t2) - 360) / 360 * PI2);
					glVertex3d(100,		0, (double(t2) - 360) / 360 * PI2);
					glVertex3d(0,	  100, (double(t2) - 360) / 360 * PI2);
				}
				else
				{
					glVertex3d(-100, -100, (double(t2)) / 360 * PI2);
					glVertex3d(100,     0, (double(t2)) / 360 * PI2);
					glVertex3d(0,     100, (double(t2)) / 360 * PI2);
				}
				glEnd();
			}


			// 5. drawVoronoi 3d
			xyt camForward;
			camForward.x() = cam.forward()[0];
			camForward.y() = cam.forward()[1];
			camForward.t() = cam.forward()[2];
			if (planning::keyboardflag['n'])
			{
				float g[4] = { 0,1,0,1 };
				float g2[4] = { .7, .7, .2, 1.0 };
				glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, g);
				glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, g);
				for (auto idx : drawingSetVorIdx)
				{
					glBegin(GL_TRIANGLE_STRIP);
					for (size_t i = idx.first; i < idx.second; i++)
					{
						glNormal3dv(drawingSetVorNorm[i].v3d());
						glVertex3dv(drawingSetVorVert[i].v3d());

						/*{
							auto& norm = drawingSetVorNorm[i];
							auto dot = cam.forward()
						}*/
					}
					glEnd();
				}
			}
			else if (planning::keyboardflag['m'])
			{
				float g[4] = { 0,1,0,1 };
				float g2[4] = { .7, .7, .2, 1.0 };
				glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, g);
				glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, g);
				for (int i = 0; i<drawingSetVorIdx.size(); i++)
				{
					if (drawingSetVorIdxFrom[i] == t2)
					{
						auto idx = drawingSetVorIdx[i];
						glBegin(GL_TRIANGLE_STRIP);
						for (size_t i = idx.first; i < idx.second; i++)
						{
							glNormal3dv(drawingSetVorNorm[i].v3d());
							glVertex3dv(drawingSetVorVert[i].v3d());
						}
						glEnd();
					}
				}
			}


			// 2-5. bifur
			if (!planning::keyboardflag[','])
			{
				glDisable(GL_LIGHTING);
				glColor3f(1, 0, 1);
				glBegin(GL_LINES);
				for (int i = 0; i < bifurDrawingSetTemp.size(); i += 2)
				{
					glVertex3dv(bifurDrawingSetTemp[i + 0].v3d());
					glVertex3dv(bifurDrawingSetTemp[i + 1].v3d());
				}
				glEnd();
			}

			// 3. VIEWPORT 1
			glViewport(wd/2, 0, wd/2, ht);
			glScissor(wd / 2, 0, wd / 2, ht);
			glClearColor(0.5, 1.0, 1.0, 1.0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			
			glLineWidth(2.5);
			glDisable(GL_LIGHTING);
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glMatrixMode(GL_MODELVIEW);
			static int viewLength = 4.5;
			glLoadIdentity(); 
			gluOrtho2D(-viewLength, viewLength, -viewLength, viewLength);

			//glClearColor(1.0, 1.0, 1.0, 1.0);
			//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			//glColor3f(0.0f, 0.0f, 0.0f);
			
			// 3-2. do mink
			

			ms::minkowskisum(t2, 1);

			// 3-3. draw mink
			//dbg
			int cnt = 0;
			for (auto& loop : ms::Model_Result)
				for (auto& as : loop)
				{
					as.draw();
					cnt++;
				}
			cout << "AS count : " << cnt << endl;

			// 4. find voronoi
			glColor3f(1, 0, 1);
			planning::VR_IN vrin;
			planning::_Convert_MsOut_To_VrIn(ms::Model_Result, ms::ModelInfo_Boundary, vrin);

			voronoiCalculator vc;
			vc.initialize();
			vc.setInput(vrin.arcs, vrin.left, vrin.color);
			//dbg_out
			cout << "input arcs.size() : " << vrin.arcs.size() << endl;
			
			const int vorOption = 0;

			// i. orig
			if (vorOption == 0)
				planning::_Medial_Axis_Transformation(vrin);
			// ii. calcg()
			if (vorOption == 1)
			{
				//voronoiCalculator vc;
				vc.initialize();
				vc.setInput(vrin.arcs, vrin.left, vrin.color);
				//dbg_out
				cout << "input arcs.size() : " << vrin.arcs.size() << endl;
				voronoiCalculatorResultG vcg;
				vc.setOutputG(vcg);
				vc.calculateG();

				glBegin(GL_LINES);
				for (size_t i = 0; i < vcg.E().size(); i++)
				{
					glColor3fv(colors + 3 * i);
					for (auto& ve : vcg.E()[i])
					{
						glVertex2dv(ve.v0.P);
						glVertex2dv(ve.v1.P);
					}
				}
				glEnd();

				glPointSize(6.0);
				glBegin(GL_POINTS);
				for(int i = 0; i <vcg.V().size(); i++)
				{
					glVertex2dv(vcg.V()[i].P);
				}
				glEnd();
				glPointSize(1.0);

				//// dbg_out
				//cout << "ALL Points : " << vcg.V().size() << endl;
				//for (auto& a : vcg.V())
				//{
				//	cout << a << endl;
				//}
				//for (auto& a : vcg.E2V())
				//{
				//	cout << a.first << " , " << a.second << endl;
				//}
				//// ~dbg
			}

			// iii. calc()
			if (vorOption == 2)
			{
				//voronoiCalculator vc;
				vc.initialize();
				vc.setInput(vrin.arcs, vrin.left, vrin.color);
				//dbg_out
				cout << "input arcs.size() : " << vrin.arcs.size() << endl;
				vector<deque<VoronoiEdge>> v_res;
				vc.setOutput(v_res);
				vc.calculate();

				// 5. draw voronoi;
				int colInd = 0;
				int vecount = 0;
				glBegin(GL_LINES);
				for (auto& cycle : v_res)
				{
					glColor3fv(colors + 3 * colInd);
					for (auto& edge : cycle)
					{
						glVertex2dv(edge.v0.P);
						glVertex2dv(edge.v1.P);
						vecount++;
					}
					colInd++;
					if (colInd >= sizeof(colors) / 3)
						colInd = 0;
				}
				glEnd();

				cout << "Number of domain : " << vc.getDomains().size() << endl;
				cout << "Number of trans  : " << vc.getTransitions().size() << endl;
				cout << "Number of pieces : " << vc.getPieces().size() << endl;
				cout << "Number of cycles : " << vc.getCycles().size() << endl;
				cout << "vecount : " << vecount << endl;

				int bifurCount = 0;
				for (auto& a : vc.getCycles())
				{
					if (a.size() > 2)
						bifurCount++;
				}
				cout << "bifurCount : " << bifurCount << " / " << vc.getCycles().size() << endl;

				// 6.test
				{
					if (vc.getCycles()[0].size() == 2)
					{
						for (auto& a : vc.getOutput()[0])
						{
							cout << a.v0 << " ... " << a.v1 << endl;
						}
					}
				}
			}

			// 6. TEST input
			if (false)
			{
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				for (auto& as : ms::Models_Approx[1])
				{
					as.draw();
				}
			}

			// 7. DEBUG : draw cycle
			static int cycIdx = -1, cycIdx2 = -1;
			if (planning::keyboardflag['['] != planning::keyboardflag_last['['])
				cycIdx++;
			if (planning::keyboardflag[']'] != planning::keyboardflag_last[']'])
				cycIdx--;
			if (planning::keyboardflag['{'] != planning::keyboardflag_last['{'])
				cycIdx2++;
			if (planning::keyboardflag['}'] != planning::keyboardflag_last['}'])
				cycIdx2--;

			//dbg_out : cycle out
			{
				cout << "cycIdx : " << cycIdx << endl;
				cout << "cycIdx2 : " << cycIdx2 << endl;
				glColor3f(0, 1, 0);
				if (cycIdx >= 0 && cycIdx < vc.getCycles().size())
				{
					// alias & size output
					auto& cyc = vc.getCycles()[cycIdx];
					cout << "cycSize : " << cyc.size() << endl;

					// do the drawing
					double len = 0.1;
					glBegin(GL_LINES);
					for (int j = 0; j < cyc.size(); j++)
					{
						if (cycIdx2 < 0 || (cycIdx2 % cyc.size()) == j)
						{
							auto& arc = cyc[j];
							auto p0 = arc.x0();
							auto p1 = arc.x1();
							auto p00 = p0 + len * arc.n0();
							auto p01 = p0 - len * arc.n0();
							auto p10 = p1 + len * arc.n1();
							auto p11 = p1 - len * arc.n1();

							glVertex2dv(p00.P);
							glVertex2dv(p01.P);
							glVertex2dv(p10.P);
							glVertex2dv(p11.P);
						}
					}
					glEnd();
				}
				// dbg_out
				auto& cycOriginal = vc.getCyclesOri();
				for (int i = 0; i < cycOriginal.size(); i++)
				{
					if (i != cycIdx)
						continue;
					cout << "cycOri : " << i << endl;
					for (auto pc : cycOriginal[i])
					{
						cout << " pc : " << pc.first.c << ", " << pc.first.t << " --> " << pc.second.c << ", " << pc.second.t << endl;
						cout << "   -arc : " << vrin.arcs[pc.first.c].x0() << vrin.arcs[pc.first.c].x1() << vrin.arcs[pc.first.c].ccw << " ... circ : " << vrin.arcs[pc.first.c].c.c << vrin.arcs[pc.first.c].c.r << endl;
					}
				}
			}

			// 97. swap
			glScissor(0, 0, wd, ht);
			glutSwapBuffers();

			// 98. set camera;
			cam.update();
			/*
			Current key:
			wasd rf :	move cam pos
			jkli :		move cam ori
			zx :		change cam speed
			cv :		draw planes on left
			op :		+- time
			b :			draw 3d mink
			n :			draw 3d vor
			m :			draw 3d vor (current time, time + 1) only
			
			*/

			// 99. set keyboardFlagOld
			for (int i = 0; i < 256; i++)
				planning::keyboardflag_last[i] = planning::keyboardflag[i];
		};

		int main3(int argc, char* argv[])
		{
			const int
				_10_bifur = false,
				_11_bifur_pair = false,
				_12_vor2d = false,
				_13_vor3d = false;

			// 1. basic stuff
			wd = 1600;
			ht = 800;
			cam.initialize(planning::keyboardflag, planning::keyboardflag_last);
			glutInit(&argc, argv);
			glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
			glutInitWindowSize(wd, ht);
			glutCreateWindow("CS Obstacles");
			glEnable(GL_SCISSOR_TEST);
			glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

			//planning::_h_fmdsp_g1 = 1e-12;

			// 2. make scene
			vector<CircularArc> robot;
			vector<Circle> robotC;

			vector<CircularArc> obs;
			vector<Circle> obsC;

			{
				robot.push_back(CircularArc(Point(1, 1), 0.5, Point( 1, 0), Point( 0,  1)));
				robot.push_back(CircularArc(Point(1, 1), 0.5, Point( 0, 1), Point(-1,  0)));
				robot.push_back(CircularArc(Point(1, 1), 0.5, Point(-1, 0), Point( 0, -1)));
				robot.push_back(CircularArc(Point(1, 1), 0.5, Point( 0,-1), Point( 1,  0)));
				robotC.push_back(    Circle(Point(1, 1), 0.35));
				robotC.push_back(    Circle(Point(1, 1), 0.4));
				robotC.push_back(    Circle(Point(1, 1), 0.3));

				obs = robot;
				obsC = robotC;

				vector<CircularArc> arcObjectSq;
				vector<Circle>	   circObjectSq;
				readArcModel("Objects/Square-shape-g1/arc.txt", "Objects/Square-shape-g1/circ.txt", arcObjectSq, circObjectSq);

				vector<CircularArc> arcObjectTri;
				vector<Circle>     circObjectTri;
				readArcModel("Objects/Hex-shape-g1/arc.txt", "Objects/Hex-shape-g1/circ.txt", arcObjectTri, circObjectTri);

				vector<CircularArc> arcObject8;
				vector<Circle>     circObject8;
				readArcModel("Objects/8/arc.txt", "Objects/8/circ.txt", arcObject8, circObject8);

				vector<CircularArc> arcASC45;
				vector<Circle>     circASC45;
				readArcModelAndProcessIt("Objects/ASC20/arc.txt", "Objects/ASC20/circ.txt", arcASC45, circASC45);

				appendArcModel(obs, obsC, arcObjectSq, circObjectSq, 1, 0.5, Point(-1.2, -1.3));
				//appendArcModel(obs, obsC, arcObjectSq, circObjectSq, 1, 0.5, Point(+1.2, -1.5));
				//appendArcModel(obs, obsC, arcObjectTri, circObjectTri, 1, -0.3, Point(+1.1, -1.5));
				appendArcModel(obs, obsC, arcASC45, circASC45, 1, -0.3, Point(+1.1, -1.5));

				//change robot to tri
				robot.clear();
				robotC.clear();
				appendArcModel(robot, robotC, arcObject8, circObject8, 1, 0, Point(0.1, 0.1));
				std::cout << "r.size " << robot.size() << std::endl;
			}

			// 3. use calc
			configSpaceCalculator csc;
			csc.setRobot(robot);
			csc.setObstacle(obs);
			csc.setOutput(out);

			csc.isInputProper();
			csc.calculate();

			// 4. tess
			int tessSize = 7;
			for (auto& surf : out)
				surf.tessellation(tessSize,tessSize);

			// 5. cut tris and offset it. So that theta is in some range.
			for (auto& surf : out)
				drawingSets.push_back(surf.drawingSet());

			// 6. Get Ready for minksum_first_func
			// 6-1. register (init() step of minkSum original program)
			//planning::_Convert_VectorCircularArc_To_MsInput(robot,	ms::Models_Approx[0]);
			ms::Model_vca[0] = robot;
			ms::InteriorDisks_Imported[0] = robotC;
			ms::Model_from_arc[0] = true;
		
			planning::_Convert_VectorCircularArc_To_MsInput(obs, ms::Models_Approx[1]);
			//ms::Model_vca[1] = obs;
			ms::InteriorDisks_Imported[1] = obsC;
			ms::Model_from_arc[1] = true;

			// 6-2. postProcess
			ms::postProcess(0, 1);

			// 7 do mink
			// ms::minkowskisum(180, 1);

			// 8. set voronoiBoundary
			{

				double l = 4.0;
				double eps = 0.01;
				Point
					q1(+l, +l),
					q2(-l, +l),
					q3(-l, -l),
					q4(+l, -l);
				auto a = cd::constructArc(q1, q2, Point(-1, +eps).normalize());
				auto b = cd::constructArc(q2, q3, Point(-eps, -1).normalize());
				auto c = cd::constructArc(q3, q4, Point(+1, -eps).normalize());
				auto d = cd::constructArc(q4, q1, Point(+eps, +1).normalize());
				planning::voronoiBoundary.push_back(a);
				planning::voronoiBoundary.push_back(b);
				planning::voronoiBoundary.push_back(c);
				planning::voronoiBoundary.push_back(d);

			}

			// 9. set colors randomly;
			float f = 1.234567;
			float f2 = 1.741562;
			float f3 = 1.55555;
			for (int i = 0; i < 100; i++)
			{
				float r = f*f2 - int(f*f2);
				float b = 1.0 - r;
				float g = (f2 * f2 / f) - int(f2 * f2 / f);
				colors[3 * i] = r;
				colors[3 * i + 1] = g;
				colors[3 * i + 2] = b;

				f = f * f;
				f2 = f2 * f2 / f3;
			}

			// 10. voronoi bifur points
			vcs.resize(ms::numofframe);
			bifurPts.resize(ms::numofframe);

			if(_10_bifur)
			for (int i = 0; i < ms::numofframe; i++)
			{
				ms::minkowskisum(i, 1);
				planning::VR_IN vrin;
				planning::_Convert_MsOut_To_VrIn(ms::Model_Result, ms::ModelInfo_Boundary, vrin);

				auto& vc = vcs[i];
				vc.initialize();
				vc.setInput(vrin.arcs, vrin.left, vrin.color);
				vector<deque<VoronoiEdge>> v_res;
				vc.setOutput(v_res);
				vc.buildTransitions();
				vc.buildPieces();
				vc.buildCycles();

				auto& cycles = vc.getCycles();
				for (auto& cyc : cycles)
				{
					auto temp = vc.findBifurcationPointRecursively(cyc);
					bifurPts[i].insert(bifurPts[i].end(), temp.begin(), temp.end());
				}
			}

			// 11. find pair between bifur pts
			// need some way to make all the points be included in the drawingset
			if(_11_bifur_pair)
			{
				std::vector<Point> smaller, bigger;
				for (int i = 0; i < ms::numofframe; i++)
				{
					int iNext = (i + 1) % ms::numofframe;
					double smallerZ, biggerZ;


					// compare size
					if (bifurPts[i].size() > bifurPts[iNext].size())
					{
						smaller = bifurPts[iNext];
						bigger = bifurPts[i];

						if (i > 180)
						{
							biggerZ = double(i - 360) / 360.0 * PI2;
							smallerZ = double(i - 359) / 360.0 * PI2;
						}
						else
						{
							biggerZ = double(i) / 360.0 * PI2;
							smallerZ = double(i +1) / 360.0 * PI2;
						}
					}
					else
					{
						smaller = bifurPts[i];
						bigger = bifurPts[iNext];

						if (i > 180)
						{
							biggerZ = double(i - 359) / 360.0 * PI2;
							smallerZ = double(i - 360) / 360.0 * PI2;
						}
						else
						{
							biggerZ = double(i + 1) / 360.0 * PI2;
							smallerZ = double(i ) / 360.0 * PI2;
						}
					}

					// for each point in bigger set get closest
					for (size_t j = 0; j < bigger.size(); j++)
					{
						int minIdx = 0;
						double minDist = 1e8;
						for (size_t k = 0; k < smaller.size(); k++)
						{
							double sqDist = (bigger[j] - smaller[k]).length2();
							if (sqDist < minDist)
							{
								minIdx = k;
								minDist = sqDist;
							}
						}

						xyt big(bigger[j].x(), bigger[j].y(), biggerZ);
						xyt sml(smaller[minIdx].x(), smaller[minIdx].y(), smallerZ);

						if (big.t() < sml.t())
						{
							bifurDrawingSetTemp.push_back(big);
							bifurDrawingSetTemp.push_back(sml);
						}
						else
						{
							bifurDrawingSetTemp.push_back(sml);
							bifurDrawingSetTemp.push_back(big);
						}


					}

				}
			}


			// 12. build v-diagram with topology
			if(_12_vor2d)
			{
				// 12-1. resize;
				vcs.resize(ms::numofframe);
				topoVorRes.resize(ms::numofframe);

				for (int i = 0; i < ms::numofframe; i++)
				{
					// 12-1. alias
					voronoiCalculator& vc = vcs[i];
					voronoiCalculatorResultG& vcg = topoVorRes[i];
					//voronoiCalculator vc;
					//voronoiCalculatorResultG vcg;

					// 12-2. mink
					ms::minkowskisum(i, 1);
					planning::VR_IN vrin;
					planning::_Convert_MsOut_To_VrIn(ms::Model_Result, ms::ModelInfo_Boundary, vrin);

					// 12-3. vor with topo
					//dbg_out
					cout << "calculating voronoi : " << i << endl;
					cout << "input arcs.size() : " << vrin.arcs.size() << endl;
					vc.initialize();
					vc.setInput(vrin.arcs, vrin.left, vrin.color);
					vc.setOutputG(vcg);
					vc.calculateG();
				}
			} 

			// 13. find drawingSet for vor-diag
			double interSlicePointError = 1e-1; //squared val : interslice point connection further than sqrt(thisValue) is considered to be a false connection.
			int nSamples = 10;
			if(_13_vor3d)
			{
				// for each vCurve endpoint p0,p1 (do bifur point first, then danglingPt => to handle case where bifur pt disappears btw slice)
				//	project p0, p1 to slice_n+1, and find there nearest point in slice_n+1 (each q0, q1)
				//		q0 should be in some ball of p0
				//		first look for bifur points, dangling points, then point on curve(since biufr will probably be bifur next slice)

				// 13-1. compare near slice & find connection
				for (int i = 0; i < ms::numofframe; i++)
				{
					// 13-2. alias
					auto inext = i + 1;
					if (inext == ms::numofframe)
						inext = 0;

					auto& vcgr0 = topoVorRes[i];
					auto& edges = vcgr0.E();
					auto& verts = vcgr0.V();
					auto& e2v   = vcgr0.E2V();

					auto& vcgr1 = topoVorRes[inext];
					auto& vertsNext = vcgr1.V();

					// debug_var
					double max2 = -1;

					// 13-3. find an edges relationshipp btw slices
					// for (each edge)
					for (int j = 0; j < edges.size(); j++)
					{
						auto& edge = edges[j];
						auto& epIndices = e2v[j];
						auto& ep0 = verts[epIndices.first]; //edge's EndPoint
						auto& ep1 = verts[epIndices.second];
						
						{
						dbg //dbg_out
							double max = -1;
							for (size_t i = 0; i < edge.size() - 1; i++)
							{
								// just in case : note that i has two scopes
								auto& p0 = edge[i].v1;
								auto& p1 = edge[i + 1].v0;
								auto len = (p0 - p1).length2();
								if (len > max)
									max = len;
							}
							if (max > max2)
								max2 = max;
							//cout << "max p.v0-q.v1 diff : " << sqrt(max) << endl;
						_dbg
						}

						// 13-3-1. examine edge's endpoint's projection (find pair)
						auto pair = vcgr1.findVoronoiCurvePair(edge.front(), edge.back(), interSlicePointError);
						
						// 13-3-2. form samples; // Just pick vEdge endpoints for now.
						int ns = nSamples;
						std::vector<int> vs0, vs1; // array to hold sample idx
						{
							if (pair.size() < ns)
								ns = pair.size();
							if (edge.size() < ns)
								ns = edge.size();

							for (int i = 0; i < ns; i++)
							{
								vs0.push_back(i * edge.size() / ns);
								vs1.push_back(i * pair.size() / ns);
							}
						}

						// 13-3-3 find out z coord
						double z0, z1;
						{
							if (i < 180)
							{
								z0 = i;
								z1 = i + 1;
								z0 = z0 * PI / 180;
								z1 = z1 * PI / 180;
							}
							else
							{
								z0 = i - 360;
								z1 = i + 1 - 360;
								z0 = z0 * PI / 180;
								z1 = z1 * PI / 180;
							}
						}

						// 13-3-4. find drawing set
						{
							vector<VoronoiEdge> EDGE;
							EDGE.insert(EDGE.end(), edge.begin(), edge.end());

							int start = drawingSetVorVert.size();
							tessellateVoronoiCurvePair(nSamples, EDGE, pair, z0, z1, drawingSetVorVert, drawingSetVorNorm);
							int end = drawingSetVorVert.size();
							drawingSetVorIdx.push_back(std::pair<int,int>(start,end));
							drawingSetVorIdxFrom.push_back(i);
						}

						//// 13-3-4 build drawing set
						//std::pair<int, int> drawIdx;
						//drawIdx.first = drawingSetVorVert.size();
						//for (int j = 0; j < ns; j++)
						//{
						//	// drawingSetPoint
						//	auto vert0 = xyt(edge[vs0[j]].v0.x(), edge[vs0[j]].v0.y(), z0);
						//	auto vert1 = xyt(pair[vs1[j]].v0.x(), pair[vs1[j]].v0.y(), z1);
						//	drawingSetVorVert.push_back(vert0);
						//	drawingSetVorVert.push_back(vert1);
						//
						//	// normal => approximate dS/dTheta and dS/dPhi
						//	auto dTheta = vert1 - vert0;
						//	auto dPhi0 = xyt(edge[vs0[j]].v1.x() - edge[vs0[j]].v0.x(), edge[vs0[j]].v1.y() - edge[vs0[j]].v0.y(), 0.0);
						//	auto dPhi1 = xyt(pair[vs1[j]].v1.x() - pair[vs1[j]].v0.x(), pair[vs1[j]].v1.y() - pair[vs1[j]].v0.y(), 0.0);
						//	auto norm0 = -(dPhi0.cross(dTheta)).normalize();
						//	auto norm1 = -(dPhi1.cross(dTheta)).normalize();
						//	drawingSetVorNorm.push_back(norm0);
						//	drawingSetVorNorm.push_back(norm1);
						//}
						//// 13-3-4-2. j == ns : this case is special
						//// TODO: case where ns = 0 (like when two bifur merges...) need more robust sampling
						//if(ns != 0) 
						//{
						//
						//	// drawingSetPoint
						//	auto vert0 = xyt(edge.back().v1.x(), edge.back().v1.y(), z0);
						//	auto vert1 = xyt(pair.back().v1.x(), pair.back().v1.y(), z1);
						//	drawingSetVorVert.push_back(vert0);
						//	drawingSetVorVert.push_back(vert1);
						//
						//	// normal => approximate dS/dTheta and dS/dPhi
						//	auto dTheta = vert1 - vert0;
						//	auto dPhi0 = xyt(edge.back().v1.x() - edge.back().v0.x(), edge.back().v1.y() - edge.back().v0.y(), 0.0);
						//	auto dPhi1 = xyt(pair.back().v1.x() - pair.back().v0.x(), pair.back().v1.y() - pair.back().v0.y(), 0.0);
						//	auto norm0 = -(dPhi0.cross(dTheta)).normalize();
						//	auto norm1 = -(dPhi1.cross(dTheta)).normalize();
						//	drawingSetVorNorm.push_back(norm0);
						//	drawingSetVorNorm.push_back(norm1);
						//}
						//drawIdx.second = drawingSetVorVert.size();
						//drawingSetVorIdx.push_back(drawIdx);
						//drawingSetVorIdxFrom.push_back(i);

						// 13-3-5 Take care of pts in slice iNext, which was not paired to slice i's some point
						// HARD TODO
					}
					
					// debug out;
					std::cout << "Maximum v_edge diff at slice " << i << " : " << max2 << std::endl;
				}
			}


			// 99. register Func and start loop
			glutReshapeFunc(reshapeFunc);
			glutDisplayFunc(displayFunc);
			glutMouseFunc(mouseFunc);
			glutKeyboardFunc(keyboardFunc);
			glutIdleFunc(idleFunc);

			glEnable(GL_DEPTH_TEST);

			glutMainLoop();

			return 0;
		}

		/*
		Def:
			given two vCurves(vec0, vec1) from different slice(z0 and z1), Make connections and tessellate them into triangles.
		Param : 
			nSamples := the number of triangles will be 2*nSamples 

		
		*/
		void tessellateVoronoiCurvePair(int in_nSamples, vector<VoronoiEdge>& in_vec0, vector<VoronoiEdge>& in_vec1, double in_z0, double in_z1, vector<xyt>& out_vert, vector<xyt>& out_norm)
		{
			// 1. Alias
			int& ns = in_nSamples;
			vector<VoronoiEdge>& v0 = in_vec0;
			vector<VoronoiEdge>& v1 = in_vec1;
			double z0 = in_z0;
			double z1 = in_z1; 
			vector<xyt>& vert = out_vert;
			vector<xyt>& norm = out_norm;

			// 2. sample ns-times from vCurves and form tri-strip
			double sz0 = v0.size();
			double sz1 = v1.size();
			double rcp = 1.0 / ns; // reciprocal of ns;
			for (int i = 0; i < ns; i++)
			{
				// 2-1. find samplePoint's param := integer + t (where t in [0,1))
				double t0 = sz0 * i * rcp;
				double t1 = sz1 * i * rcp;

				// 2-2. decompose to : integer + [0,1)
				int idx0 = t0; // floor
				int idx1 = t1; // fllor

				t0 = t0 - idx0;
				t1 = t1 - idx1;

				// 2-3. ready for interpolation
				Point
					& p00 = v0[idx0].v0,
					& p01 = v0[idx0].v1,
					& p10 = v1[idx1].v0,
					& p11 = v1[idx1].v1;

				// 2-4. do interpolation
				Point
					p0 = (1-t0)*p00 + (t0)*p01,
					p1 = (1-t1)*p10 + (t1)*p11;

				// 2-5. insert vert;
				vert.emplace_back(p0.x(), p0.y(), z0);
				vert.emplace_back(p1.x(), p1.y(), z1);

				// 2-6. find dS/dPhi, dS/dTheta;
				auto
					dTheta = p1 - p0,
					dPhi0  = p01 - p00,
					dPhi1  = p11 - p10;

				// 2-7. change to 3d coord
				xyt
					dt(dTheta.x(), dTheta.y(), 1),
					dp0(dPhi0.x(), dPhi0.y(), 0),
					dp1(dPhi1.x(), dPhi1.y(), 0);

				// 2-8. insert
				norm.push_back(-dp0.cross(dt).normalize());
				norm.push_back(-dp1.cross(dt).normalize());

			}

			// 3. when i == ns case (should be done a bit differently)
			// similar to section 2.
			{
				// 3-1. ready for interpolation (idx corresponds to last element)
				Point
					& p00 = v0.back().v0,
					& p01 = v0.back().v1,
					& p10 = v1.back().v0,
					& p11 = v1.back().v1;

				// 3-2. do interpolation (t0 and t1 is just 1.0)
				Point
					p0 = p01,
					p1 = p11;

				// 3-3. insert vert;
				vert.emplace_back(p0.x(), p0.y(), z0);
				vert.emplace_back(p1.x(), p1.y(), z1);

				// 2-6. find dS/dPhi, dS/dTheta;
				auto
					dTheta = p1 - p0,
					dPhi0 = p01 - p00,
					dPhi1 = p11 - p10;

				// 2-7. change to 3d coord
				xyt
					dt(dTheta.x(), dTheta.y(), 1),
					dp0(dPhi0.x(), dPhi0.y(), 0),
					dp1(dPhi1.x(), dPhi1.y(), 0);

				// 2-8. insert
				norm.push_back(-dp0.cross(dt).normalize());
				norm.push_back(-dp1.cross(dt).normalize());
			}

		}
	}

}


/********************************************************************************************************************
**																												   **
**													~~Main 4~~													   **
**																												   **
**																												   **
********************************************************************************************************************/

/*
Def : main3 := make 3d configuration space obstacle using there equations.
*/
namespace sceneEditor
{
	/*
	q w : swap model original idx
	e	: add model original's instance

	a s : swap instance idx
	d	: delete instance
	ijkl: translate current instance
	u o : rotate current instance
	p ; : scale current instance

	n m : modify multiplier(speed)

	c v : rotate mink/vor degree

	z	: output to file
	
	*/

	int wd, ht;
	vector<vector<CircularArc>> modelsArc;
	vector<vector<Circle>>		modelsCir;
	vector<string>				modelsName;
	vector<vector<double>>		instances; // each instance : idx, scale, rotation(degree), transX, transY

	int modelIdx = 0;
	int instIdx = 0;

	double speed = 1.0;
	double robotDegree = 0.0;


	void reshapeFunc(GLint w, GLint h)
	{
		// later
	};
	void idleFunc()
	{
		// later

		glutPostRedisplay();
	};
	void mouseFunc(int button, int action, int x, int y)
	{

	};
	void keyboardFunc(unsigned char a, int b, int c)
	{
		planning::keyboardflag[a] = !planning::keyboardflag[a];
		// pressing a once will flip keybaord flag btw 0 and 1
		// pressing it for a long time will make it oscillate 010101
	};
	void displayFunc()
	{
		// 0. Take care of inputs
		{
			auto press = [&](char c) ->bool
			{
				return planning::keyboardflag[c] != planning::keyboardflag_last[c];
			};

			if (press('w'))
			{
				modelIdx++;
				if (modelIdx >= modelsArc.size())
					modelIdx = modelsArc.size() - 1;
			}
			if (press('q'))
			{
				modelIdx--;
				if (modelIdx < 0)
					modelIdx = 0;
			}
			if (press('e'))
			{
				vector<double> temp({double(modelIdx), 1.0, 0.0, 0.0, 0.0});
				instances.push_back(temp);
				instIdx = instances.size() - 1;
			}

			if (press('s'))
				instIdx++;
			if (press('a'))
				instIdx--;

			if (instIdx >= 0 && instIdx < instances.size())
			{
				if (press('d'))
				{
					auto it = instances.begin() + instIdx;
					instances.erase(it);
				}

				if (press('j'))
				{
					instances[instIdx][3] -= 0.1 * speed;
				}
				if (press('l'))
				{
					instances[instIdx][3] += 0.1 * speed;
				}
				if (press('k'))
				{
					instances[instIdx][4] -= 0.1 * speed;
				}
				if (press('i'))
				{
					instances[instIdx][4] += 0.1 * speed;
				}

				if (press('u'))
				{
					instances[instIdx][2] += 1 * speed;
				}
				if (press('o'))
				{
					instances[instIdx][2] -= 1 * speed;
				}

				if (press('p'))
				{
					instances[instIdx][1] += 0.1 * speed;
				}
				if (press(';'))
				{
					instances[instIdx][1] -= 0.1 * speed;
				}
			}

			if (press('n'))
			{
				speed *= 2.0;
			}
			if (press('m'))
			{
				speed *= 0.5;
			}

			if (press('c'))
			{
				robotDegree += 1.0;
				if (int(robotDegree) >= ms::numofframe)
					robotDegree = ms::numofframe - 1;
			}
			if (press('v'))
			{
				robotDegree -= 1.0;
				if (robotDegree < 0)
					robotDegree = 0;
			}

			if (press('z'))
			{
				char str[100];
				sprintf_s(str, 100, "scene %d.txt", time(NULL));
				ofstream fout(str);
				for (auto& t : instances)
				{
					fout << modelsName[int(t[0])] << " " << t[1] << " " << t[2] << " " << t[3] << " " << t[4] << endl;
				}
			}

			cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
			cout << "Model : " << modelIdx << " :  " << modelsName[modelIdx] << endl;
			cout << "Inst  : " << instIdx <<  endl;
			cout << "Robot andgle : " << robotDegree << endl;
		}

		// 1. clear
		glClearColor(0.0, 1.0, 1.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glColor3f(0.0f, 0.0f, 0.0f);

		glDisable(GL_LIGHTING);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		static double viewLength = 1.5;
		glLoadIdentity();
		gluOrtho2D(-viewLength, viewLength, -viewLength, viewLength);

		// 2. VIEWPORT 0
		glViewport(0, 0, wd / 2, ht);
		glScissor(0, 0, wd / 2, ht);

		cout << "instancse size : " << instances.size() << endl;

		// 2-1. build model ready for mink
		{
			int obsIdx = 7;
			ms::Model_from_arc[obsIdx] = true;
			auto& sceneOriginal		= ms::Model_vca[obsIdx];
			auto& sceneProcessed	= ms::Models_Approx[obsIdx];
			auto& sceneCircles		= ms::InteriorDisks_Imported[obsIdx];

			sceneOriginal.resize(0);
			sceneCircles.resize(0);
			sceneProcessed.resize(0);

			// do appending
			for (auto& inst : instances)
			{
				appendArcModel(sceneOriginal, sceneCircles, modelsArc[int(inst[0])], modelsCir[int(inst[0])], inst[1], inst[2], Point(inst[3], inst[4]));
			}

			planning::_Convert_VectorCircularArc_To_MsInput(sceneOriginal, sceneProcessed);
		}

		// 2-2. do drawing
		for (int i = 0; i < instances.size(); i++)
		{
			auto& inst = instances[i];
			if (i == instIdx)
				glColor3f(0, 0, 1);
			else
				glColor3f(0, 0, 0);

			vector<CircularArc> temp;
			vector<Circle> temp2;
			appendArcModel(temp, temp2, modelsArc[int(inst[0])], modelsCir[int(inst[0])], inst[1], inst[2], Point(inst[3], inst[4]));

			for (auto& arc : temp)
				arc.draw2();
		}

		// 2-3. draw bound
		glColor3f(0, 0, 0);
		for (auto& arc : planning::voronoiBoundary)
			arc.draw2();
		// 2-4. draw circ
		if (planning::keyboardflag['3'])
		{
			glColor3f(0, 0, 0);
			for (auto& circ : ms::InteriorDisks_Imported[7])
				circ.draw();
		}

		// 3. VIEWPORT 1
		glViewport(wd / 2, 0, wd / 2, ht);
		glScissor(wd / 2, 0, wd / 2, ht);
		glClearColor(0.5, 1.0, 1.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glLineWidth(2.5);

		if(planning::keyboardflag['1'])
		{
			// draw mink
			ms::ModelInfo_CurrentModel = std::make_pair(1, 7);
			ms::postProcess(ms::ModelInfo_CurrentModel.first, ms::ModelInfo_CurrentModel.second);
			ms::t2 = int(robotDegree);
			ms::minkowskisum(ms::t2, 7);
			glColor3f(0, 0, 0);
			for (auto& loop : ms::Model_Result)
				for(auto& as : loop)
					as.draw();
			cout << "Mink res size :" <<  ms::Model_Result.size() << " " << ms::ModelInfo_Boundary.size() << endl;

			// draw Vor
			glColor3f(1, 0, 1);
			if (planning::keyboardflag['2'])
			{
				planning::VR_IN vrin;
				planning::_Convert_MsOut_To_VrIn(ms::Model_Result, ms::ModelInfo_Boundary, vrin);
				voronoiCalculator vc;
				vector<deque<VoronoiEdge>> v_res;
				vc.initialize();
				vc.setInput(vrin.arcs, vrin.left, vrin.color);
				vc.setOutput(v_res);
				vc.calculate();

				glBegin(GL_LINES);
				for (auto& a : v_res)
					for (auto& b : a)
					{
						glVertex2dv(b.v0.P);
						glVertex2dv(b.v1.P);
					}
				glEnd();
			}
		}

		// 2-3. draw bound
		glColor3f(0, 0, 0);
		for (auto& arc : planning::voronoiBoundary)
			arc.draw2();
		
		// 97. swap
		glScissor(0, 0, wd, ht);
		glutSwapBuffers();

		// 99. set keyboardFlagOld
		for (int i = 0; i < 256; i++)
			planning::keyboardflag_last[i] = planning::keyboardflag[i];
	};

	int main(int argc, char* argv[])
	{
		

		// 1. basic stuff
		wd = 1600;
		ht = 800;
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
		glutInitWindowSize(wd, ht);
		glutCreateWindow("SceneEditor");
		glEnable(GL_SCISSOR_TEST);

		// 2. read Models
		initializeRobotObstacles(0, 1);// Just to set the robots; model_approx[0] and model_approx[1]
		
		int i = 0;
		modelsArc.resize(0);
		modelsCir.resize(0);
		auto fillModels = [&](const char* file) -> void
		{
			string name = file;
			modelsName.push_back(name);
			modelsArc.push_back({});
			modelsCir.push_back({});
			auto& arcs = modelsArc[i];
			auto& cirs = modelsCir[i];
			readArcModelAndProcessIt(("Objects/" + name + "/arc.txt").c_str(), ("Objects/" + name + "/circ.txt").c_str(), arcs, cirs);
			i++;
		};
		{
			fillModels("Tri-shape-g1");
			fillModels("Square-shape-g1");
			fillModels("pent-shape-g1");
			fillModels("hex-shape-g1");
			fillModels("L-shape");
			fillModels("L-shape3");
			fillModels("ASC30");
			fillModels("ASC10");
			fillModels("ASC3");
			fillModels("ASC1");
		}


		// 99. register Func and start loop
		glutReshapeFunc(reshapeFunc);
		glutDisplayFunc(displayFunc);
		glutMouseFunc(mouseFunc);
		glutKeyboardFunc(keyboardFunc);
		glutIdleFunc(idleFunc);

		glEnable(GL_DEPTH_TEST);

		glutMainLoop();

		return 0;
	}
}