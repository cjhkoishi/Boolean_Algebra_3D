#pragma once
#include<map>
#include<set>
using namespace std;
class UnionFind
{
private:
	map<int, int> group;
public:
	int Find(int i) {
		if (group.find(i) != group.end())
			return group[i];
		else
			return -1;
	};

	void Union(int i, int j) {
		if (group[i] != group[j]) {
			int k = group[i];
			int l = group[j];
			for (auto it = group.begin(); it != group.end(); it++) {
				if (it->second == l)
					it->second = k;
			}
		}
	};

	void Add(int i) {
		if (group.find(i) == group.end())
			group[i] = i;
	};

	int ComponentsNum() {
		set<int> coms;
		for (auto it = group.begin(); it != group.end(); it++) {
			coms.insert(it->second);
		}
		return coms.size();
	};

	UnionFind() {};
	UnionFind(int n) {
		for (int i = 0; i < n; i++)
			Add(i);
	};
};

