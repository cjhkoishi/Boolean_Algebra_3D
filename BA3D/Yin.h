#pragma once
#include"Mesh.h"
class Yin
{
public:
	vector<Surface> boundarys;
	vector<vector<int>> atoms;//

	void GenerateHasseDigram();
	int Postion(P3D p);

	void Intersect(Yin obj,vector<SegInfo>& intersection0, vector<SegInfo>& intersection1);

	void Cutting(vector<vector<Segment>> trace,vector<Surface>& result);
	void Pasting(vector<Surface> pieces);
};

