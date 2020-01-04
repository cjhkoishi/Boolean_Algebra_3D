#pragma once
#include"Mesh.h"
class Yin
{
public:
	bool status;
	vector<Mesh> boundarys;
	vector<vector<int>> atoms;//
	vector<int> Eular;

	void GenerateHasseDigram();
	int Postion(P3D p);
	int Postion(TriangleP T);

	void Intersect(Yin& obj,vector<SegInfo>& intersection0, vector<SegInfo>& intersection1);

	void Cutting(vector<vector<Segment>> trace,vector<Mesh>& result);
	void Pasting(vector<Mesh> pieces);
	void split(Mesh& S);

	Yin meet(Yin& obj);
	Yin complement();
	Yin join(Yin& obj);

	void InPut(string filename);
	void OutPut(string name,int config=0);
};

