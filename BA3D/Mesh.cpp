#include "Mesh.h"
#include"UnionFind.h"

void Surface::TriangletionForCell(vector<FaceInfo> info)
{
}

void Surface::LoadFromFile(string filename)
{
	fstream fin(filename, ios::in);
	string str;


	int v = 0;
	while (getline(fin, str)) {
		stringstream ss(str);
		char c;
		ss >> c;
		if (c == 'v') {
			double x, y, z;
			ss >> x >> y >> z;
			vertices[v] = P3D(x, y, z);
			v++;
		}
		if (c == 'f') {
			Triangle t;
			ss >> t[0] >> t[1] >> t[2];
			t[0]--;
			t[1]--;
			t[2]--;
			faces.push_back(t);
		}
	}
	fin.close();
}

void Surface::WriteToFile(string filename)
{
	fstream fout(filename, ios::out);
	map<int, int> index;
	int k = 0;
	for (auto i = vertices.begin(); i != vertices.end(); i++, k++) {
		index[i->first] = k + 1;
		fout << "v " << i->second.coor[0] << ' ' << i->second.coor[1] << ' ' << i->second.coor[2] << endl;
	}
	for (auto i = faces.begin(); i != faces.end(); i++) {
		Triangle T = *i;
		fout << "f " << index[T[0]] << ' ' << index[T[1]] << ' ' << index[T[2]] << endl;
	}
	fout.close();
}

void Surface::ElimitateUnusedPoint()
{
	set<int> buffer;
	for_each(faces.begin(), faces.end(), [&](Triangle T) {
		buffer.insert(T[0]);
		buffer.insert(T[1]);
		buffer.insert(T[2]);
		});
	for (auto i = vertices.begin(); i != vertices.end(); ) {
		if (buffer.find(i->first) == buffer.end()) {
			int bad_ind = i->first;
			i++;
			vertices.erase(bad_ind);
		}
		else {
			i++;
		}
	}
}

int Surface::Postion(P3D p)
{
	return 0;
}

Surface::Surface()
{
}

Surface::~Surface()
{
}

double SegmentP::Dist(P3D p)
{

	return 0.0;
}

SegmentP::SegmentP()
{
}

SegmentP::SegmentP(P3D P, P3D Q)
{
	vert[0] = P;
	vert[1] = Q;
}

int TriangleP::intersect(SegmentP ray, P3D& intersection, double& r, double& s, double& t)
{
	P3D X = vert[1] - vert[0];
	P3D Y = vert[2] - vert[0];
	P3D P = ray.vert[1] - ray.vert[0];
	P3D D = vert[0] - ray.vert[0];
	P3D n = X.cross(Y);
	double t0 = P * n;
	if (t0 == 0)
		return false;
	double t1 = D * n;
	t = t1 / t0;
	intersection = ray.vert[0] + t * P;
	P3D E1 = D.cross(Y);
	P3D E2 = X.cross(D);
	r = E1 * P / -t0;
	s = E2 * P / -t0;
	if (r >= 0 && s >= 0 && r + s <= 1)
		return true;
	return false;
}

TriangleP::TriangleP()
{
}

TriangleP::TriangleP(P3D P, P3D Q, P3D R)
{
	vert[0] = P;
	vert[1] = Q;
	vert[2] = R;
}

void MeshWithCell::Cutting(vector<Surface>& result)
{
	UnionFind com;
	vector<Triangle> vec_faces;
	map<int, vector<int>> neighbor;
	for (int i = 0; i < faces.size(); i++) {
		com.Add(i);
	}

	for_each(cell.begin(), cell.end(), [&](Segment& s) {s.sort(); });

	vec_faces.assign(faces.begin(), faces.end());

	for (int i = 0; i < vec_faces.size(); i++) {
		set<Segment> origin_detector;
		Triangle T = vec_faces[i];
		Segment S[3];
		S[0][0] = S[2][1] = T[0];
		S[1][0] = S[0][1] = T[1];
		S[2][0] = S[1][1] = T[2];
		S[0].sort();
		S[1].sort();
		S[2].sort();
		origin_detector.insert(S[0]);
		origin_detector.insert(S[1]);
		origin_detector.insert(S[2]);
		for (int j = 0; j < vec_faces.size(); j++) {
			if (i == j)
				continue;
			auto detector = origin_detector;
			T = vec_faces[j];
			S[0][0] = S[2][1] = T[0];
			S[1][0] = S[0][1] = T[1];
			S[2][0] = S[1][1] = T[2];
			S[0].sort();
			S[1].sort();
			S[2].sort();
			for (int c = 0; c < 3; c++) {
				if (!detector.insert(S[c]).second) {
					bool isCut = find(cell.begin(), cell.end(), S[c]) != cell.end();
					if (isCut)
						break;
					com.Union(i, j);
				}
			}
		}
	}
	result.resize(com.ComponentsNum());
	for_each(result.begin(), result.end(), [&](Surface& M) {M.vertices = vertices; M.faces.clear(); });
	map<int, int> index;
	int k = 0;
	for (int i = 0; i < vec_faces.size(); i++) {
		if (index.find(com.Find(i)) == index.end()) {
			index[com.Find(i)] = k;
			k++;
		}
		Surface& M = result[index[com.Find(i)]];
		M.faces.push_back(vec_faces[i]);
	}
	for_each(result.begin(), result.end(), [&](Surface& M) {M.ElimitateUnusedPoint(); });
}
