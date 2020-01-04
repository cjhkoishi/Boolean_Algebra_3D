#include "Yin.h"
#include <iostream>
#include "UnionFind.h"

void Yin::GenerateHasseDigram()
{

	atoms.clear();
	if (boundarys.size() == 0)
		return;
	auto copy = boundarys;
	list<Simplex<2>> hasse;
	vector<bool> signs;
	signs.resize(copy.size());

	for (int i = 0; i < copy.size(); i++) {
		signs[i] = copy[i].Orientation();
		if (!signs[i])
			copy[i] = copy[i].inverse();
	}

	for (int i = 0; i < copy.size(); i++)
		for (int j = 0; j < copy.size(); j++) {
			if (i == j)
				continue;
			Triangle t = *copy[i].faces.begin();
			TriangleP test_T(copy[i].vertices[t[0]], copy[i].vertices[t[1]], copy[i].vertices[t[2]]);
			int pos = copy[j].Postion(test_T);
			if (pos > 1) {
				cout << "mistakes in here" << endl;
				return;
			}
			Simplex<2> order;
			if (pos == 1) {
				order[0] = i;
				order[1] = j;
				hasse.push_back(order);
			}

		}

	for (auto is = hasse.begin(); is != hasse.end(); is++) {
		Simplex<2> s = *is;
		int u = s[0];
		int v = s[1];
		int k = v;
		int n = copy.size();
		bool* T = new bool[n];
		for (int i = 0; i < n; i++)
			T[i] = false;
		T[v] = true;
		while (true) {
			bool flag = false;
			int prev = -1;
			for (auto i = hasse.begin(); i != hasse.end(); i++) {
				if ((*i)[0] == k && i != is && !T[(*i)[1]]) {
					T[(*i)[1]] = flag = true;
					k = (*i)[1];
					//DFS mission
					Simplex<2> rem;
					rem[0] = u;
					rem[1] = k;
					hasse.remove(rem);

					break;
				}
				if ((*i)[1] == k && T[(*i)[0]]) {
					prev = (*i)[0];
				}
			}
			if (!flag) {
				k = prev;
				if (k == -1)
					break;
			}
		}
		delete[] T;
	}

	//构造ATOMS
	map<int, int> forest;
	bool sign_flag = true;
	bool root_sign = true;
	vector<int> depths;
	depths.resize(copy.size());
	for_each(hasse.begin(), hasse.end(), [&](Simplex<2>& s) {
		forest[s[0]] = s[1];
		});
	for (int i = 0; i < copy.size(); i++) {
		int depth = 0;
		int k = i;

		while (forest.find(k) != forest.end()) {
			k = forest[k];
			depth++;
		}
		if (depth == 0 && sign_flag) {
			root_sign = signs[i];
			sign_flag = false;
		}
		depths[i] = depth;
	}
	for (int i = 0; i < copy.size(); i++) {
		if (root_sign) {
			depths[i] = depths[i] % 2;
		}
		else {
			depths[i] = (depths[i] + 1) % 2;
		}
	}

	UnionFind A(copy.size());
	vector<bool> connected;
	connected.resize(copy.size());
	for (int i = 0; i < connected.size(); i++) {
		connected[i] = false;
	}
	for_each(hasse.begin(), hasse.end(), [&](Simplex<2>& s) {
		if (depths[s[0]] == 1 && depths[s[1]] == 0) {
			connected[s[0]] = connected[s[1]] = true;
			A.Union(s[0], s[1]);
		}
		});
	//处理孤立逆向曲面
	if (!root_sign) {
		int f = -1;
		for (int i = 0; i < connected.size(); i++) {
			if (!connected[i]) {
				if (f == -1)
					f = i;
				A.Union(i, f);
			}
		}
	}

	status = !root_sign;

	atoms.resize(A.ComponentsNum());
	map<int, int> index_map;
	int l = 0;
	for (int i = 0; i < copy.size(); i++) {
		if (index_map.find(A.Find(i)) == index_map.end())
			index_map[A.Find(i)] = l++;
		atoms[index_map[A.Find(i)]].push_back(i);
	}

}

int Yin::Postion(P3D p)
{
	if (boundarys.size() == 0)
		return status ? 1 : 0;
	bool res = false;
	for (int i = 0; i < atoms.size(); i++) {
		bool component_res = true;
		for (int j = 0; j < atoms[i].size(); j++) {
			int pos = boundarys[atoms[i][j]].Postion(p);
			if (pos == 2)
				return 2;
			component_res &= (pos == 1);
		}
		res |= component_res;
	}
	return res ? 1 : 0;
}

int Yin::Postion(TriangleP T)
{
	if (boundarys.size() == 0)
		return status ? 1 : 0;
	bool res = false;
	for (int i = 0; i < atoms.size(); i++) {
		bool component_res = true;
		for (int j = 0; j < atoms[i].size(); j++) {
			int pos = boundarys[atoms[i][j]].Postion(T);
			if (pos == 2 || pos == 3)
				return pos;
			component_res &= (pos == 1);
		}
		res |= component_res;
	}
	return res ? 1 : 0;
}

void Yin::Intersect(Yin& obj, vector<SegInfo>& intersection0, vector<SegInfo>& intersection1)
{
	vector<TriangleP> tris;
	vector<int> offsets;
	map<int, int> offset_map;
	map<int, int> group_map;
	int N = 0;
	for (int i = 0; i < boundarys.size(); i++) {
		N += boundarys[i].faces.size();
	}
	for (int i = 0; i < obj.boundarys.size(); i++) {
		N += obj.boundarys[i].faces.size();
	}
	tris.resize(N);
	offsets.resize(boundarys.size() + obj.boundarys.size());

	int m = 0;
	int l = 0;
	int K = 0;
	for (int i = 0; i < boundarys.size(); i++) {
		for (auto j = boundarys[i].faces.begin(); j != boundarys[i].faces.end(); j++) {
			Triangle T = *j;
			tris[m] = boundarys[i].GetGeoTriangle(T);
			offset_map[m] = l;
			group_map[m] = i;
			m++;
		}
		l = m;
	}
	K = l;
	int obj_offset = boundarys.size();
	for (int i = 0; i < obj.boundarys.size(); i++) {
		for (auto j = obj.boundarys[i].faces.begin(); j != obj.boundarys[i].faces.end(); j++) {
			Triangle T = *j;
			tris[m] = obj.boundarys[i].GetGeoTriangle(T);
			offset_map[m] = l;
			group_map[m] = i;
			m++;
		}
		l = m;
	}

	SegInfo SI_all;
	FindIntersection(tris, SI_all);


	intersection0.clear();
	intersection0.resize(boundarys.size());
	intersection1.clear();
	intersection1.resize(obj.boundarys.size());
	for (auto i = SI_all.begin(); i != SI_all.end(); i++) {
		SegmentP sp = i->first;
		auto f = i->second;
		//去除无效交线
		set<P3D, comparator> pool;
		for (auto it = f.begin(); it != f.end(); it++) {
			auto sof = *it;
			SOF S = sof.second;
			TriangleP T = tris[sof.first];
			P3D V = T.Norm();
			V = (1 / V.norm()) * V;
			bool invalid = S.code[0] < 3 && (
				S.code[1] < 3 ||
				S.code[1] == S.code[0] + 3 ||
				S.code[1] == (S.code[0] + 2) % 3 + 3
				);
			invalid |= S.code[0] >= 3 && S.code[0] < 6 && (
				S.code[1] == S.code[0] ||
				S.code[1] == S.code[0] - 3 ||
				S.code[1] == (S.code[0] - 2) % 3
				);

			if (invalid) {
				int s = S.code[0];
				if (s < 3) {
					s = S.code[1];
					if (s < 3) {
						switch (S.code[0] + S.code[1])
						{
						case 1:
							s = 3;
							break;
						case 3:
							s = 4;
							break;
						case 2:
							s = 5;
							break;
						}
					}
				}
				s -= 3;
				SegmentP side(T.vert[s], T.vert[(s + 1) % 3]);
				if ((side.vert[1] - side.vert[0]) * (sp.vert[1] - sp.vert[0]) > 0)
					pool.insert(V);
				else
					pool.insert(-1 * V);
			}
			else {
				pool.insert(V);
				pool.insert(-1 * V);
			}
		}
		if (pool.size() <= 2)
			continue;

		for (auto j = f.begin(); j != f.end(); j++) {
			if (j->first < K)
				intersection0[group_map[j->first]][sp][j->first - offset_map[j->first]] = j->second;
			else
				intersection1[group_map[j->first]][sp][j->first - offset_map[j->first]] = j->second;
		}
	}
}

void Yin::Cutting(vector<vector<Segment>> trace, vector<Mesh>& result)
{
	if (trace.size() != boundarys.size()) {
		cout << "unavalidable traces" << endl;
		return;
	}
	for (int i = 0; i < boundarys.size(); i++) {
		vector<Mesh> h;
		boundarys[i].Cutting(trace[i], h);
		result.insert(result.begin(), h.begin(), h.end());
	}
}

void Yin::Pasting(vector<Mesh> pieces)
{
	Mesh M;
	vector<int> offsets;
	offsets.resize(pieces.size());
	offsets[0] = 0;
	for (int i = 1; i < offsets.size(); i++) {
		offsets[i] = pieces[i - 1].MaxIndex() + 1;
	}
	for (int i = 0; i < offsets.size() - 1; i++) {
		offsets[i + 1] = offsets[i] + offsets[i + 1];
	}
	map<int, int> index_map;
	map<P3D, int, comparator> pool;

	int o = 0;
	for (auto i = pieces.begin(); i != pieces.end(); i++) {
		for (auto j = i->vertices.begin(); j != i->vertices.end(); j++) {
			if (pool.find(j->second) != pool.end())
				index_map[j->first + offsets[o]] = pool[j->second];
			else {
				index_map[j->first + offsets[o]] = j->first + offsets[o];
				pool[j->second] = j->first + offsets[o];
				M.vertices[j->first + offsets[o]] = j->second;
			}
		}
		for (auto j = i->faces.begin(); j != i->faces.end(); j++) {
			(*j)[0] = index_map[(*j)[0] + offsets[o]];
			(*j)[1] = index_map[(*j)[1] + offsets[o]];
			(*j)[2] = index_map[(*j)[2] + offsets[o]];
			M.faces.push_back(*j);
		}
		o++;
	}

	split(M);
}

void Yin::split(Mesh& S)
{
	boundarys.clear();
	atoms.clear();
	if (S.faces.size() == 0)
		return;

	map<int, vector<int>> st;//星形算子
	vector<Triangle> vfaces(S.faces.begin(), S.faces.end());
	for (int i = 0; i < vfaces.size(); i++) {
		Triangle& T = vfaces[i];
		st[T[0]].push_back(i);
		st[T[1]].push_back(i);
		st[T[2]].push_back(i);
	}

	int m = S.MaxIndex() + 1;
	auto target = S.vertices;
	for (auto i = target.begin(); i != target.end(); i++) {
		UnionFind com;
		int ID = i->first;
		vector<int> tris = st[ID];
		map<int, map<int, bool>> radii;//v of [ID,v] => Triangle IDs => ori
		//构造radii领域
		for (int j = 0; j < tris.size(); j++) {
			com.Add(tris[j]);
			Triangle T = vfaces[tris[j]];
			for (int v = 0; v < 3; v++)
				if (T[v] != ID) {
					bool ori = T[(v + 1) % 3] != ID;
					radii[T[v]][tris[j]] = ori;
				}
		}
		//检查good pair连通集
		for (auto j = radii.begin(); j != radii.end(); j++) {
			int r = j->first;
			map<int, bool>& sectors = j->second;
			if (sectors.size() % 2 != 0) {
				cout << "Not border of an Yin Set!" << endl;
			}
			if (sectors.size() == 2) {
				com.Union(sectors.begin()->first, sectors.rbegin()->first);
				continue;
			}
			//选择比较基准面
			P3D base;
			P3D axis = S.vertices[r] - S.vertices[ID];
			vector<int> circul;//tri ID
			bool flag = true;
			for (auto f = sectors.begin(); f != sectors.end(); f++) {
				circul.push_back(f->first);
				if (flag) {
					Triangle T = vfaces[f->first];
					TriangleP BT = S.GetGeoTriangle(T);
					base = BT.Norm();
					flag = false;
				}
			}
			P3D Z = base.cross(axis);
			TriangleP arc_plane(P3D(0, 0, 0), base, Z);
			sort(circul.begin(), circul.end(), [&](int a, int b)->bool {
				Triangle TA = vfaces[a];
				Triangle TB = vfaces[b];
				TriangleP AP = S.GetGeoTriangle(TA);
				TriangleP BP = S.GetGeoTriangle(TB);
				P3D NA = AP.Norm();
				P3D NB = BP.Norm();
				if (!sectors[a])
					NA = -1 * NA;
				if (!sectors[b])
					NB = -1 * NB;
				P2D PA = arc_plane.AffineCoor(NA);
				P2D PB = arc_plane.AffineCoor(NB);
				return atan2(PA.coor[1], PA.coor[0]) < atan2(PB.coor[1], PB.coor[0]);
				});
			if (sectors[circul[0]])
				for (int d = 0; d < circul.size() / 2; d++)
					com.Union(circul[2 * d], circul[2 * d + 1]);
			else
				for (int d = 0; d < circul.size() / 2; d++)
					com.Union(circul[2 * d + 1], circul[(2 * d + 2) % circul.size()]);

		}
		//split "ID" point
		st[ID].clear();
		int count = com.ComponentsNum();
		map<int, int> group2order;
		int l = 0;
		for (int t = 0; t < tris.size(); t++) {
			int g = com.Find(tris[t]);
			if (group2order.find(g) == group2order.end())
				group2order[g] = l++;
			int new_index = group2order[g] != 0 ? m + group2order[g] - 1 : ID;
			for (int v = 0; v < 3; v++)
				if (vfaces[tris[t]][v] == ID)
					vfaces[tris[t]][v] = new_index;
			st[new_index].push_back(tris[t]);

		}
		for (int c = 1; c < count; c++) {
			S.vertices[m++] = S.vertices[ID];
		}
	}
	//分离独立曲面
	UnionFind com_s;
	map<Segment, vector<int>> mirror;
	for (int i = 0; i < vfaces.size(); i++) {
		Triangle& T = vfaces[i];
		Segment edge;
		for (int v = 0; v < 3; v++) {
			edge[0] = T[v];
			edge[1] = T[(v + 1) % 3];
			edge.sort();
			mirror[edge].push_back(i);
		}
		com_s.Add(i);
	}
	for (auto i = mirror.begin(); i != mirror.end(); i++) {
		auto con = i->second;
		for (int c = 0; c < con.size(); c++) {
			com_s.Union(con[0], con[c]);
		}
	}
	boundarys.resize(com_s.ComponentsNum());
	for (int i = 0; i < boundarys.size(); i++) {
		boundarys[i].vertices = S.vertices;
	}
	map<int, int> g2o;
	int l = 0;
	for (int i = 0; i < vfaces.size(); i++) {
		int g = com_s.Find(i);
		if (g2o.find(g) == g2o.end())
			g2o[g] = l++;
		boundarys[g2o[g]].faces.push_back(vfaces[i]);
	}

	for (int i = 0; i < boundarys.size(); i++) {
		boundarys[i].AdjustVerticesID();
	}

	GenerateHasseDigram();

	Eular.resize(boundarys.size());
	for (int i = 0; i < Eular.size(); i++) {
		int V = boundarys[i].vertices.size();
		int F = boundarys[i].faces.size();
		Eular[i] = V - F / 2;
	}
}

Yin Yin::meet(Yin& obj)
{
	vector<SegInfo> SI[2];
	Intersect(obj, SI[0], SI[1]);
	vector<Path> P[2];
	vector<vector<Segment>> traces;
	vector<Mesh> pieces[2], res;

	P[0].resize(boundarys.size());
	P[1].resize(obj.boundarys.size());
	traces.resize(boundarys.size());
	for (int i = 0; i < SI[0].size(); i++) {
		P[0][i].Construct(boundarys[i], SI[0][i]);
		P[0][i].Triangulate();
		traces[i] = P[0][i].segs;
		boundarys[i] = *P[0][i].target;
	}
	Cutting(traces, pieces[0]);

	traces.clear();
	traces.resize(obj.boundarys.size());
	for (int i = 0; i < SI[1].size(); i++) {
		P[1][i].Construct(obj.boundarys[i], SI[1][i]);
		P[1][i].Triangulate();
		traces[i] = P[1][i].segs;
		obj.boundarys[i] = *P[1][i].target;
	}
	obj.Cutting(traces, pieces[1]);

	for (int i = 0; i < pieces[0].size(); i++) {
		TriangleP T = pieces[0][i].GetGeoTriangle(*pieces[0][i].faces.begin());
		int pos = obj.Postion(T);
		if (pos == 1 || pos == 2)
			res.push_back(pieces[0][i]);
	}
	for (int i = 0; i < pieces[1].size(); i++) {
		TriangleP T = pieces[1][i].GetGeoTriangle(*pieces[1][i].faces.begin());
		int pos = Postion(T);
		if (pos == 1)
			res.push_back(pieces[1][i]);
	}

	Yin fin;
	fin.Pasting(res);

	fin.status = status && obj.status;

	return fin;
}

Yin Yin::complement()
{
	Yin fin;

	Yin Null;
	vector<SegInfo> SI[2];
	Intersect(Null, SI[0], SI[1]);
	vector<Path> P;
	vector<vector<Segment>> traces;
	vector<Mesh> pieces;

	P.resize(boundarys.size());
	traces.resize(boundarys.size());
	for (int i = 0; i < SI[0].size(); i++) {
		P[i].Construct(boundarys[i], SI[0][i]);
		P[i].Triangulate();
		traces[i] = P[i].segs;
		boundarys[i] = *P[i].target;
	}
	Cutting(traces, pieces);

	for (int i = 0; i < pieces.size(); i++) {
		for (auto j = pieces[i].faces.begin(); j != pieces[i].faces.end(); j++) {
			Triangle& T = *j;
			swap(T[1], T[2]);
		}
	}
	fin.Pasting(pieces);

	fin.status = !status;

	return fin;
}

Yin Yin::join(Yin& obj)
{
	Yin inv0 = complement();
	Yin inv1 = obj.complement();
	Yin m = inv0.meet(inv1);
	Yin res = m.complement();
	return res;
}

void Yin::InPut(string filename)
{
	Mesh S;
	S.LoadFromFile(filename);
	split(S);
}

void Yin::OutPut(string name, int config)
{
	stringstream ss;
	if (config == 0) {
		for (int i = 0; i < boundarys.size(); i++) {
			ss.str("");
			ss << name << "_" << i << ".obj";
			boundarys[i].WriteToFile(ss.str());
		}
	}
	else if (config == 1) {
		ss << name << ".obj";
		Mesh all;
		all.Pasting(boundarys);
		all.WriteToFile(ss.str());
	}
	else if (config == 2) {
	}
}
