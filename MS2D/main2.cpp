#include "MS2D.h"
#include <iostream>
#include <iomanip>
#include "voronoi.hpp"
#include "collision detection.hpp"

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
			znear = 0.01,
			zfar = 10.0;
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

			
			mult	= 0.03;
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

			float red[4] = { 1,0,0,1 };
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, red);

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
	**																												   **
	**																												   **
	**																												   **
	********************************************************************************************************************/

	/*
	Def : main3 := make 3d configuration space obstacle using there equations.
	*/
	namespace main3
	{
		cam3d cam;

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

			// 2. VIEWPORT 0
			glViewport(0, 0, wd, ht);
			
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
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, red);


			// 4. swap
			glutSwapBuffers();

			// 98. set camera;
			cam.update();

			// 99. set keyboardFlagOld
			for (int i = 0; i < 256; i++)
				planning::keyboardflag_last[i] = planning::keyboardflag[i];
		};



		int main3(int argc, char* argv[])
		{
			// 1. basic stuff
			wd = ht = 800;
			cam.initialize(planning::keyboardflag, planning::keyboardflag_last);
			glutInit(&argc, argv);
			glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
			glutInitWindowSize(wd, ht);
			glutCreateWindow("CS Obstacles");

			// 2. make scene
			vector<CircularArc> robot;
			vector<Circle> robotC;

			vector<CircularArc> obs;
			vector<Circle> obsC;

			{

			}

			// 3.

		}
	}

}