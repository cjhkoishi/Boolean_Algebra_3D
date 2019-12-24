#pragma once
#include"DoublyConnectedEdgeList.h"
#include <map>

class SegmentP {
public:
	P3D vert[2];

	double Dist(P3D p);
	int OnDetect(P3D p);

	SegmentP();
	SegmentP(P3D P, P3D Q);
};

class TriangleP {
public:
	P3D vert[3];


	P3D Norm();
	double Dist(P3D p);
	int OnDetect(P3D p);
	/*
	r s-parameter of point in triangle
	t-parameter of point in segment
	0=0  1=1  2=2  3=01  4=12  5=20  6=012 -1=otherwise
	0=0  1=1 2=01 -1=otherwise
	*/
	bool intersect(SegmentP ray, P3D& intersection, double& r, double& s, double& t, int& tri_pos_code, int& seg_pos_code);
	bool intersect(TriangleP sub, SegmentP& intersection, P2D uv[4], int code[4]);
	P2D AffineCoor(P3D p);

	TriangleP();
	TriangleP(P3D P, P3D Q, P3D R);
};


class Seg_Comp {
public:
	bool operator()(const SegmentP& s1, const SegmentP& s2)const {
		return LESS(s1.vert[0], s2.vert[0]) || s1.vert[0] == s2.vert[0] && LESS(s1.vert[1], s2.vert[1]);
	}
};
struct SOF {
	P2D uv[2];
	int code[2];
	SOF(P2D uv0, P2D uv1, int code0, int code1)
	{
		this->uv[0] = uv0;
		this->uv[1] = uv1;
		this->code[0] = code0;
		this->code[1] = code1;
	};
	SOF(){}
};
typedef map<SegmentP, map<int,SOF>, Seg_Comp> SegInfo;
void FindIntersection(vector<TriangleP> Ts, SegInfo& intersections);

class Surface
{
public:
	map<int, P3D> vertices;
	list<Triangle> faces;

	void Cutting(vector<Segment> segs, vector<Surface>& result);
	void Pasting(vector<Surface> pieces);
	void LoadFromFile(string filename);
	void WriteToFile(string filename);
	void ElimitateUnusedPoint();
	int Postion(P3D p);
	int Postion(TriangleP T);
	double Vol();
	bool Orientation();
	int MaxIndex();
	void Intersect(Surface sub,SegInfo& SI1,SegInfo& SI2);

	Surface meet(Surface sub);
	Surface join(Surface sub);
	Surface inverse();

	Surface();
	~Surface();
};

//表示三角网格表面上的图，用于存储交线信息
class Path
{
public:
	struct PointInfo {
		struct Label {
			Triangle tri;
			P2D uv;
			int pos_code = -1;//0-2 vertex 3-5 edge 6-inside
			int belong_ID = -65535;

			bool operator==(const Label& obj)const{
				return tri == obj.tri && pos_code == obj.pos_code && belong_ID == obj.belong_ID;
			}
			Label(Triangle tri, P2D uv, int pos_code, int belong_ID) :
				tri(tri),
				uv(uv),
				pos_code(pos_code),
				belong_ID(belong_ID)
			{}
			Label() {}
		};
		P3D point;
		int pos_code;//0=face 1=edge 2=vertex
		vector<Label> labels;
	};

	Surface* S;//目标曲面
	map<int, PointInfo> new_points;//新点（包括所在三角面，位置标示码）索引对应于原顶点不重复的顶点ID
	vector<Segment> segs;//线段

	void Triangulate();

	Path() {};
	Path(Surface S,SegInfo SI);
};