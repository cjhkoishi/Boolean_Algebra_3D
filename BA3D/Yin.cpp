#include "Yin.h"
#include <iostream>
#include "UnionFind.h"

void Yin::GenerateHasseDigram()
{
	atoms.clear();
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
		if (s[0] == 1 && s[1] == 0){
			connected[s[0]] = connected[s[1]] = true;
			A.Union(s[0], s[1]);
		}
		});
	//处理无界yin集的根节点
	if (!root_sign) {
		int f = -1;
		for (int i = 0; i < connected.size(); i++) {
			if (connected[i]) {
				if (f == -1)
					f = i;
				else
					A.Union(i, f);
			}
		}
	}

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

void Yin::Cutting(vector<vector<Segment>> trace, vector<Surface>& result)
{
	if (trace.size() != boundarys.size()){
		cout << "unavalidable traces" << endl;
		return;
	}
	for (int i = 0; i < boundarys.size(); i++) {
		vector<Surface> h;
		boundarys[i].Cutting(trace[i], h);
		result.insert(result.begin(),h.begin(), h.end());
	}
}

void Yin::Pasting(vector<Surface> pieces)
{

}
