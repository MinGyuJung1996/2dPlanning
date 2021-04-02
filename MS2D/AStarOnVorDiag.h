#pragma once

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

using namespace std;
using namespace boost;


typedef pair<int, int>											Edge;
typedef double                                                  Cost;
typedef adjacency_list<listS, vecS, undirectedS, no_property,
	property<edge_weight_t, Cost>>								Graph;
typedef property_map<Graph, edge_weight_t>::type                WeightMap;
typedef Graph::edge_descriptor                                  EdgeDescr;
typedef Graph::vertex_descriptor                                VertexDescr;

extern double EPS;

//-----------------------------------------------------------------------------
// auxiliary types
struct Vertex
{
	Vertex(double x, double y, double z = 0.) :x(x), y(y), z(z) {}
	Cost dist(const Vertex& other) const
	{
		Cost dx = x - other.x;
		Cost dy = y - other.y;
		Cost dz = z - other.z;
		return ::sqrt(dx * dx + dy * dy /*+ dz * dz*/);

	}
	double x, y, z;
	friend ostream& operator << (ostream& os, const Vertex& obj);
};

class VertexLessFn
{
public:
	bool operator() (const Vertex& v0, const Vertex& v1) const
	{
		if (abs(v0.x - v1.x) < EPS)
			if (abs(v0.y - v1.y) < EPS)
				if (abs(v0.z - v1.z) < EPS)
					return false;
				else
					return v0.z < v1.z;
			else
				return v0.y < v1.y;
		else
			return v0.x < v1.x;
	}
};
//-----------------------------------------------------------------------------
// euclidean distance heuristic
template <class Graph, class CostType, class LocMap>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
	typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
	distance_heuristic(LocMap l, Vertex goal)
		: m_location(l), m_goal(goal) {}
	CostType operator()(Vertex u)
	{
		return m_location[m_goal].dist(m_location[u]);
	}
private:
	LocMap m_location;
	Vertex m_goal;
};


//-----------------------------------------------------------------------------
// exception for termination
struct found_goal {};

//-----------------------------------------------------------------------------
// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
	astar_goal_visitor(Vertex goal) : m_goal(goal) {}
	template <class Graph>
	void examine_vertex(Vertex u, Graph& g) {
		if (u == m_goal)
			throw found_goal();
	}
private:
	Vertex m_goal;
};

using v_edge = planning::output_to_file::v_edge;
Graph create_VorGraph(vector<vector<v_edge>>& vorGr,
					  vector<Vertex>& vecVertices,
				      map<Vertex, int, VertexLessFn>& mapLookup);
vector<Vertex> invoke_AStar(Graph& theGr,
							const vector<Vertex>& vecVertices,
							const map<Vertex, int, VertexLessFn>& mapLookup,
							const Vertex& ptnSrc,
							const Vertex& ptnDst);
