//=======================================================================
//
// Inspired and based on 
// https://www.boost.org/doc/libs/1_43_0/libs/graph/example/astar-cities.cpp
//
//=======================================================================

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <time.h>
#include <vector>
#include <list>
#include <set>
#include <iostream>
#include <fstream>
#include <unordered_map> 
#include <math.h>    // for sqrt
#include <algorithm>    // std::min
#include "voronoi.hpp"
#include "AStarOnVorDiag.h"

double EPS = 0.00001;

//#define EXCHANGE_FILE_FULL_NAME "C:\\SNU\\MSData\\exchange.txt" 
//#define EXCHANGE_FILE_FULL_NAME "exchange.txt" 


//-----------------------------------------------------------------------------
bool Vertex::operator ==(const Vertex& other) const
{
	VertexLessFn cmp;
	return !cmp(*this, other) && !cmp(other, *this);
}

Vertex Vertex::operator +(const Vertex& other) const
{
	return Vertex(x + other.x, y + other.y, z + other.z);
}

Vertex Vertex::operator -(const Vertex& other) const
{
	return *this + (other*(-1));
}

Vertex Vertex::operator *(double w) const
{
	return Vertex(x * w, y * w, z * w);
}

double Vertex::dot(const Vertex& other) const
{
	return x * other.x + y * other.y + z * other.z;
}

Vertex Vertex::cross(const Vertex& other) const
{
	double s1 = y * other.z - z * other.y;
	double s2 = z * other.x - x * other.z;
	double s3 = x * other.y - y * other.x;
	return Vertex(s1, s2, s3);
}

double Vertex::norm() const
{
	return ::sqrt(x * x + y * y + z * z);
}

void Vertex::normalize()
{
	double nrm = norm();
	if (nrm < EPS)
		return;
	nrm = 1. / nrm;
	x = x * nrm;
	y = y * nrm;
	z = z * nrm;
}

double Vertex::get_angle_between(const Vertex& other) const
{
	//assumption: vectors are normalized
	double cos_gamma = ::min(this->dot(other), 1.);
	double gamma = acos(cos_gamma);
	return gamma;
}

Vertex Vertex::geodesic_avg(const Vertex& other, double w) const
{
	//assumption: v1, v2 normalized
	if (w < EPS)
		return Vertex(*this);
	else if (EPS > (1. - w))
		return Vertex(other);

	double gamma = get_angle_between(other);
	gamma *= (1. - w);

	Vertex nr = cross(other);
	nr.normalize();
	double cos_gamma = cos(gamma);
	double sin_gamma = sin(gamma);
	Vertex res_vec = *this * cos_gamma
		+ nr.cross(*this) * sin_gamma
		+ nr * (1. - cos_gamma) * nr.dot(*this);
	res_vec.normalize();
	return res_vec;
}

ostream& operator << (ostream& os, const Vertex& obj)
{
	os << "(" << obj.x << ", " << obj.y << ", " << obj.z << ")";
	return os;
}

int register_vertex(const Vertex&                   v,
					vector<Vertex>&                 vecVertices,
					map<Vertex, int, VertexLessFn>& mapLookup)
{
	auto vi = mapLookup.find(v);
	int idx = -1;
	if (vi == mapLookup.end())
	{
		vecVertices.push_back(v);
		idx = (int)vecVertices.size() - 1;
		mapLookup.insert({ v, idx });
	}
	else
	{
		idx = vi->second;
	}
	return idx;
}

void read_slice(vector<v_edge>&     vecVorEdges,
	double                          zSlice,
	vector<Vertex>&                 vecVertices,
	vector<Edge>&                   vecEdges,
	vector<Cost>&                   vecWeights,
	map<Vertex, int, VertexLessFn>& mapLookup,
	set<int>&                       setCurrSliceIdx)
{

	for (auto vorEdge : vecVorEdges)
	{
		Vertex vrtxP(vorEdge.v0.x(), vorEdge.v0.y(), zSlice);
		Vertex vrtxQ(vorEdge.v1.x(), vorEdge.v1.y(), zSlice);
		int idxP = register_vertex(vrtxP, vecVertices, mapLookup);
		int idxQ = register_vertex(vrtxQ, vecVertices, mapLookup);
		setCurrSliceIdx.insert(idxP);
		setCurrSliceIdx.insert(idxQ);
		vecEdges.push_back({ idxP, idxQ });
		vecWeights.push_back(vrtxP.dist(vrtxQ));
	}
}
//-----------------------------------------------------------------------------
void connect_slices(const vector<Vertex>&  vecVertices,
	const set<int>&        setPrevSliceIdx,
	const set<int>&        setCurrSliceIdx,
	vector<Edge>&          vecEdges,
	vector<Cost>&          vecWeights)
{
	for (auto prevSliceIter : setPrevSliceIdx)
	{
		Vertex v1 = vecVertices[prevSliceIter];
		double minDist = 1000.0;
		int minCurrSliceIdx = -1;
		//int nStartIdx = std::max(0, i - 10);
		//int nEndIdx = std::min(i + 10, (int) setCurrSliceIdx.size());
		for (auto currSliceIter : setCurrSliceIdx)
		{
			Vertex v2 = vecVertices[currSliceIter];
			double currDist = v1.dist(v2);
			if (minDist > currDist)
			{
				minDist = currDist;
				minCurrSliceIdx = currSliceIter;
			}
		}
		if (minDist < 0.1)
		{
			vecEdges.push_back({ prevSliceIter, minCurrSliceIdx });
			vecWeights.push_back(minDist + 0.01);
		}
	}
}
//-----------------------------------------------------------------------------

Graph create_VorGraph(	vector<vector<v_edge>>& vorGr, 
						vector<Vertex>& vecVertices,
						map<Vertex, int, VertexLessFn>& mapLookup)
{
	vector<Edge> vecEdges; // edge is a pair of indexes in vecVertices 
	vector<Cost> vecWeights; // weight of an edge is its length

	double zStep = 360./vorGr.size(); 
	double currZ = 0.;
	set<int> setPrevSliceIdx;
	set<int> setCurrSliceIdx;
	for(auto vorGrLayer : vorGr)
	{
		setCurrSliceIdx.clear();
		read_slice(	vorGrLayer, currZ,
					vecVertices, vecEdges, vecWeights,
					mapLookup, setCurrSliceIdx);
		currZ += zStep;
		if (setPrevSliceIdx.size() != 0)
			connect_slices(vecVertices, setPrevSliceIdx, setCurrSliceIdx, vecEdges, vecWeights);
		setCurrSliceIdx.swap(setPrevSliceIdx);
	}
	// create graph
	Graph theGraph(vecVertices.size());
	WeightMap weightmap = get(edge_weight, theGraph);
	for (std::size_t j = 0; j < vecEdges.size(); ++j)
	{
		EdgeDescr e;
		bool inserted;
		boost::tie(e, inserted) = add_edge(vecEdges[j].first, vecEdges[j].second, theGraph);
		weightmap[e] = vecWeights[j];
	}
	return theGraph;
}

vector<Vertex> invoke_AStar(Graph& theGr, 
							const vector<Vertex>& vecVertices, 
							const map<Vertex, int, VertexLessFn>& mapLookup,
							const Vertex& ptnSrc,
							const Vertex& ptnDst)
{
	VertexDescr start = mapLookup.find(ptnSrc)->second;
	VertexDescr goal = mapLookup.find(ptnDst)->second;


	//cout << "Start vertex: " << start << endl;
	//cout << "Goal vertex: " << goal << endl;


	vector<Graph::vertex_descriptor> p(num_vertices(theGr));
	vector<Cost> d(num_vertices(theGr));
	vector<Vertex> res;
	try {
		// call astar named parameter interface
		astar_search
		(theGr, start,
			distance_heuristic<Graph, Cost, vector<Vertex>>
			(vecVertices, goal),
			predecessor_map(&p[0]).distance_map(&d[0]).
			visitor(astar_goal_visitor<VertexDescr>(goal)));
	}
	catch (found_goal)
	{
		// found a path to the goal
		list<VertexDescr> shortest_path;
		for (VertexDescr v = goal;; v = p[v]) {
			shortest_path.push_front(v);
			if (p[v] == v)
				break;
		}

		//cout << "Shortest path from " << start << " to "
		//	<< goal << ": "<< endl;
		list<VertexDescr>::iterator spi = shortest_path.begin();
		for (; spi != shortest_path.end(); ++spi)
			res.push_back(vecVertices[*spi]);
	}
	return res;
}
//============================ END OF FILE ===================================