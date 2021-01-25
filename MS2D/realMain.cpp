#include <iostream>
#include <fstream>
#include "voronoi.hpp"
#include "AStarOnVorDiag.h"

using namespace std;

namespace ms
{
	int main(int argc, char *argv[]);
	int main2(int argc, char* argv[]);
	void renderMinkVoronoi(
		int argc,
		char* argv[],
		std::vector<decltype(ms::Model_Result)>& MRs,
		std::vector<decltype(ms::ModelInfo_Boundary)>& MIBs,
		std::vector<std::vector<planning::output_to_file::v_edge>>& v_edges,
		decltype(planning::voronoiBoundary)& voronoiBoundary
		);
	void renderPath(
		int argc,
		char* argv[],
		std::vector<double>& path
		);
	void renderRefinementCollisionTest(
		int argc,
		char* argv[],
		std::vector<decltype(ms::Model_Result)>& MRs,
		std::vector<decltype(ms::ModelInfo_Boundary)>& MIBs,
		std::vector<std::vector<planning::output_to_file::v_edge>>& v_edges,
		decltype(planning::voronoiBoundary)& voronoiBoundary,
		std::vector<planning::VR_IN> VRINs
		);
}

namespace graphSearch
{

	int main(int argc, char** argv);
	void searchTest();
	int main2(int argc, char* argv[]);
}

int main(int argc, char *argv[]) {

	//Test stuff:
	if (false)
	{
		using namespace ms;
		vector<CircularArc> cycle;

		CircularArc c0, c1, c2;
		/*
		c0.ccw = false;
		c1.ccw = false;
		c2.ccw = false;
		c0.c.c = Point( 2 + 0, 0 + 0);
		c0.c.r = 1;
		c1.c.c = Point(-2 + 0, 0 + 0);
		c1.c.r = 1;
		c2.c.c = Point( 0 + 0, 2 + 0);
		c2.c.r = 1;*/
		
		/*c0.c.c = Point(0, 4);
		c0.c.r = 1;
		c1.c.c = Point(7, 7);
		c1.c.r = 3 * sqrt(2) - 3;
		c2.c.c = Point(4, -1);
		c2.c.r = 2;
		cycle.push_back(c0);
		cycle.push_back(c1);
		cycle.push_back(c2);*/
		/*
		c0.ccw = false;
		c1.ccw = false;
		c2.ccw = true;
		c0.c.c = Point(0.564474, -0.103322);
		c0.c.r = 0.246477;
		c1.c.c = Point(0.549305, -0.529539);
		c1.c.r = 0.0423848;
		c2.c.c = Point(0, 0);
		c2.c.r = 2;*/

		c0.ccw = false;
		c1.ccw = false;
		c2.ccw = true;
		c0.c.c = Point(0.5, -0.1);
		c0.c.r = 0.2;
		c1.c.c = Point(0.5, -0.5);
		c1.c.r = 0.1;
		c2.c.c = Point(0, 0);
		c2.c.r = 2;


		cycle.push_back(c0);
		cycle.push_back(c1);
		cycle.push_back(c2);


		Point bifur(0, 0);
		double R;
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
			// plugging x, y in (6) to (1)~(3) will give two solutions of R... it will give one pos & one negative i guess...? -> since solution for (s0,s1,s2) case r0,r1 has duality with solution for (-s0,-s1,-s2)'s -r0,-r1 


			// 1-1. first check if three circle centers are colinear.
			bool isColinear;
			{
				double det = (cycle[0].c.c - cycle[1].c.c).normalize() ^ (cycle[1].c.c - cycle[2].c.c).normalize();
				isColinear = fabs(det) < 1e-10;
			}

			if (true) // TODO? when colinear
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

				if (fabs(a) < 1e-100) R = -c / b;
				else R = min(((-b + sqrt(b*b - 4 * a*c)) / a) * 0.5, ((-b - sqrt(b*b - 4 * a*c)) / a) * 0.5);

				// 1-4. compute point of bifurcation
				d0 += a0;
				d1 += b0;

				bifur = Point(c0*R + d0, c1*R + d1);

				cout << a << " " << b << " " << c << endl;

				
			}
			cout << "dist0 " << sqrt((c0.c.c - bifur).length()) << endl;
			cout << "dist1 " << sqrt((c1.c.c - bifur).length()) << endl;
			cout << "dist2 " << sqrt((c2.c.c - bifur).length()) << endl;

		}

		cout << "bifur " << bifur << endl;
		cout << "R " << R << endl;
	}

	//cout << "fake func" << endl;

	//graphSearch::searchTest();
	graphSearch::main2(argc, argv);
	
	//graphSearch::main(0, NULL);
	
	ms::main2(argc, argv);

	system("pause");
}

namespace graphSearch
{
	using v_edge = planning::output_to_file::v_edge;

	/*
	Def:
		Computes Voronoi Edges for each slice, and store them in a vector<vector<v_edge>> v_edges.
		Each Voronoi Edge can be referenced by v_edges[Slice_Number][Edge_Number_In_That_Slice];
	*/
	int main2(int argc, char* argv[])
	{
		// 1. build v_edges

		ms::initialize();	// read data from files.
		ms::ModelInfo_CurrentModel = std::make_pair(1, 7);
		ms::postProcess(ms::ModelInfo_CurrentModel.first, ms::ModelInfo_CurrentModel.second); // process arcs to satisfy conditions.
		planning::output_to_file::flag = true; // flag to enable : gathering data inside global var, during v-diagram construction.
		planning::output_to_file::v_edges.resize(0); //empty
		planning::output_to_file::v_edges.resize(ms::numofframe); //pre-allocate
		planning::output_to_file::bifur_points.resize(0); // dummy
		planning::output_to_file::bifur_points.resize(ms::numofframe); //dummy
		planning::drawVoronoiSingleBranch = false; //disable drawing for now.
		std::vector<decltype(ms::Model_Result)> MRs(ms::numofframe);		// data collected for checking
		std::vector<decltype(ms::ModelInfo_Boundary)> MIBs(ms::numofframe); // data collected for checking
		std::vector<planning::VR_IN> VRINs(ms::numofframe);
		for (size_t i = 0, length = ms::numofframe /* = 360*/; i < length; i++)
		{
			ms::t2 = i;
			ms::minkowskisum(i, 7);

			planning::VR_IN& vrin = VRINs[i];
			planning::_Convert_MsOut_To_VrIn(ms::Model_Result, ms::ModelInfo_Boundary, vrin);
			planning::_Medial_Axis_Transformation(vrin);

			// save data for checking
			MRs[i] = ms::Model_Result;
			MIBs[i] = ms::ModelInfo_Boundary;

			//// dbg_out
			//std::cout << "voronoi " << i << " "
			//	<< planning::output_to_file::v_edges[i].size() << std::endl;
		}
		planning::output_to_file::flag = false;

		///* test if result is same : print all v-edges to file and compare it*/
		//std::ofstream fout("ve_out.txt");
		//for (size_t i = 0, length = ms::numofframe; i < length; i++)
		//{
		//	fout << "voronoi " << i << std::endl;
		//	fout << planning::output_to_file::v_edges[i].size() << std::endl;
		//	for (auto e : planning::output_to_file::v_edges[i])
		//		fout << e.v0 << "\t" << e.v1 << std::endl;

		//}
		//std::cout << "END OF TEST" << std::endl;

		// 2. Alias for long names

		//using v_edge = planning::output_to_file::v_edge;
		std::vector<std::vector<v_edge>>&
			v_edges = planning::output_to_file::v_edges;

		//Graph theGr = create_VorGraph(v_edges);
		//std::vector<v_edge> path = invoke_AStar(theGr);
		std::vector<std::vector<v_edge>> v_edges_sparced;
		std::vector<std::vector<v_edge>>::iterator iterEdgesLayer = v_edges.begin();
		for ( size_t i = 0; i < v_edges.size(); ++i, ++iterEdgesLayer)
			//if (0 == (i % 10)) // filter out intermediate layers
				v_edges_sparced.push_back(*iterEdgesLayer);

		vector<Vertex> vecVertices;
		map<Vertex, int, VertexLessFn> mapLookup;
		Graph theGr = create_VorGraph(v_edges_sparced, vecVertices, mapLookup);
		Vertex ptnSrc(-0.70296182284311681, -0.30610712038352472, 0.0);
		Vertex ptnDst(0.76932775901415118, 0.36524457774288216, 320.0);
		std::vector<Vertex> path = invoke_AStar(theGr, vecVertices, mapLookup, ptnSrc, ptnDst);


		// 3. call functions from namespace graphSearch (AStarOnVorDiag.cpp)
		 // triplet of (path[3n+0], path[3n+1], path[3n+2]) represents a vertice in path. 
		// vertices = ...

		// 4~. do sth with the path....

		// 4-1. Just to check whether mink/vor was constructed properly.
		// uncomment below to begin renderLoop for mink/voronoi calculated above.
		//ms::renderMinkVoronoi(argc, argv, MRs, MIBs, v_edges, planning::voronoiBoundary);
		ms::renderRefinementCollisionTest(argc, argv, MRs, MIBs, v_edges, planning::voronoiBoundary, VRINs);

		// 4-2. render robot's path
		// Path found on step 3 should be used instead of dummy_path
		std::vector<double> renderedPath;
		{
			// just a fake path to check program pipeline.
			//for (int i = 0; i < 100; i++)
			for(auto v : path)
			{
				//double x = -0.7 + 1.4 / 100.0 * i; // x-coord of Robot center
				//double y = -0.5 + (i / 100.0) * (i / 100.0); // y-coord of Robot center
				//double z = log10(i + 1) * 180;	// should be rotation in degrees

				renderedPath.push_back(v.x);
				renderedPath.push_back(v.y);
				renderedPath.push_back(v.z);
			}
		}
		ms::renderPath(argc, argv, renderedPath);

		return 0;
	}

	

	void searchTest()
	{
		double coords2d[9][2] = { {0.,0.}, {1., 0.}, {10., 0.},
								 {0.,1.}, {1., 1.}, {10., 1.},
								 {0.,10.}, {1., 10.}, {10., 10.} };
		int layerEdgeIndeces[12][2] = { {0,1}, {1,2}, {3,4}, {4,5}, {6,7}, {7,8},
										{0,3}, {1,4}, {2,5}, {3,6}, {4,7}, {5,8} };
		vector<v_edge> layer0;
		vector<v_edge> layer1;
		for (int i = 0; i < 12; ++i)
		{
			int idxP = layerEdgeIndeces[i][0];
			int idxQ = layerEdgeIndeces[i][1];
			ms::Point p(coords2d[idxP][0], coords2d[idxP][1]);
			ms::Point q(coords2d[idxQ][0], coords2d[idxQ][1]);
			v_edge currEdge;
			currEdge.v0 = p;
			currEdge.v1 = q;
			currEdge.idx[0] = -1;
			currEdge.idx[1] = -1;
			layer0.push_back(currEdge);
			layer1.push_back(currEdge);
		}
		vector<vector<v_edge>> v_edges;
		v_edges.push_back(layer0);
		v_edges.push_back(layer1);

		vector<Vertex> vecVertices;
		map<Vertex, int, VertexLessFn> mapLookup;
		Graph theGr = create_VorGraph(v_edges, vecVertices, mapLookup);
		Vertex ptnSrc(0, 0, 0);
		Vertex ptnDst(10, 10, 180.);
		std::vector<Vertex> path = invoke_AStar(theGr, vecVertices, mapLookup, ptnSrc, ptnDst);
		if (path.empty())
		{
			cout << "Path not found" << endl;
		}
		else
		{
			for (auto v : path)
			{
				cout << v << endl;
			}
		}

	}
}