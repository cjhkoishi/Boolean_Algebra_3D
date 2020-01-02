#pragma once
#include<vector>
#include<list>
#include<string>
#include<fstream>
#include<sstream>
#include"Point.h"

using namespace std;
struct Vertex;
struct HalfEdge;
struct Face;

struct Vertex {
	P2D p;
	int id = 0;
	HalfEdge* inc_edge = 0;
	Vertex(P2D p) :p(p) {};
};

struct HalfEdge {
	Vertex* start_point = 0;
	HalfEdge* next = 0, * prev = 0, * twin = 0;
	Face* inc_face = 0;
	bool GetSign();
	bool isSameBoundary(HalfEdge* obj);
};

struct Face {
	HalfEdge* outer_edge = 0;
	vector<HalfEdge*> inner_edge;
	bool isInside(P2D p);
};

class DoublyConnectedEdgeList
{
public:
	list<Vertex*> vertex_list;
	list<HalfEdge*> edge_list;
	list<Face*> face_list;

	Face* GetPostion(P2D p);

	void GetEdgesByVertex(Vertex* v, vector<HalfEdge*>& edges);
	void ConvertFromPlanarGraph(vector<P2D> points, vector<Segment> edges);
	void AddEdge(Vertex* v1, Vertex* v2);
	Vertex* AddVertex(P2D p);
	void Trianglate();
	bool isTrianglated();

	DoublyConnectedEdgeList();
	~DoublyConnectedEdgeList();
};

typedef DoublyConnectedEdgeList DCEL;

