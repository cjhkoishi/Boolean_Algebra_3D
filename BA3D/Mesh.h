#pragma once
#include"DoublyConnectedEdgeList.h"
#include <map>

class SegmentP {
public:
	P3D vert[2];

	double Dist(P3D p);

	SegmentP();
	SegmentP(P3D P, P3D Q);
};

class TriangleP {
public:
	P3D vert[3];

	int intersect(SegmentP ray, P3D& intersection, double& r, double& s, double& t);

	TriangleP();
	TriangleP(P3D P, P3D Q, P3D R);
};

class Surface
{
public:
	struct FaceInfo {
		list<Triangle>::iterator face;
		map<int, pair<P3D, P2D>> uv_corr;
		list<Segment> connections;
	};

	map<int, P3D> vertices;
	list<Triangle> faces;


	void TriangletionForCell(vector<FaceInfo> info);
	void LoadFromFile(string filename);
	void WriteToFile(string filename);
	void ElimitateUnusedPoint();
	int Postion(P3D p);

	Surface();
	~Surface();
};

class MeshWithCell :
	public Surface 
{
public:
	vector<Segment> cell;

	void Cutting(vector<Surface>& result);
};