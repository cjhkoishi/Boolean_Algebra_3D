#pragma once
#include"DoublyConnectedEdgeList.h"
#include <map>

class SegmentP {
public:
	P3D vert[2];

	double Dist(P3D p);
	bool OnDetect(P3D p);

	SegmentP();
	SegmentP(P3D P, P3D Q);
};

class TriangleP {
public:
	P3D vert[3];
	

	P3D Norm();
	double Dist(P3D p);
	bool OnDetect(P3D p);
	/*
	r s-parameter of point in triangle
	t-parameter of point in segment
	0=0  1=1  2=2  3=01  4=12  5=20  6=012 -1=otherwise
	0=0  1=1 2=01 -1=otherwise
	*/
	bool intersect(SegmentP ray, P3D& intersection, double& r, double& s, double& t,int& tri_pos_code,int& seg_pos_code);
	bool intersect(TriangleP sub,SegmentP intersection,double& u0,double& v0,double& u1,double& v1,int& code1,int& code2);

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