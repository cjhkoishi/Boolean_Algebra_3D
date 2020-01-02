#pragma once
#include"Mesh.h"
class Yin
{
public:
	bool status;
	vector<Surface> boundarys;
	vector<vector<int>> atoms;//

	void GenerateHasseDigram();
	int Postion(P3D p);
	int Postion(TriangleP T);

	void Intersect(Yin& obj,vector<SegInfo>& intersection0, vector<SegInfo>& intersection1);

	void Cutting(vector<vector<Segment>> trace,vector<Surface>& result);
	void Pasting(vector<Surface> pieces);
	void split(Surface& S);

	Yin meet(Yin& obj);
	Yin complement();
	Yin join(Yin& obj);

	void OutPut(string name,int config=0);
};

