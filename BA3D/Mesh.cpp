#include "Mesh.h"
#include"UnionFind.h"
#include <iostream>



void Surface::Cutting(vector<Segment> segs, vector<Surface>& result)
{
	UnionFind com;
	vector<Triangle> vec_faces;
	map<int, vector<int>> neighbor;
	for (int i = 0; i < faces.size(); i++) {
		com.Add(i);
	}

	for_each(segs.begin(), segs.end(), [&](Segment& s) {s.sort(); });

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
					bool isCut = find(segs.begin(), segs.end(), S[c]) != segs.end();
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

void Surface::Pasting(vector<Surface>& pieces)
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
	int random_factor = 0;
	while (true) {
		SegmentP ray(p, p + P3D(
			cos(random_factor) * cos(random_factor),
			cos(random_factor) * sin(random_factor),
			sin(random_factor)
		));
		bool flag = true;
		int wind_num = 0;
		for (auto i = faces.begin(); i != faces.end(); i++) {
			P3D PIN;
			double r, s, t;
			int tc, sc;
			TriangleP T(vertices[(*i)[0]], vertices[(*i)[1]], vertices[(*i)[2]]);
			bool nontrivial = T.intersect(ray, PIN, r, s, t, tc, sc);
			if (sc == 0 && T.OnDetect(p)) {
				return 2;
			}
			if (!nontrivial || tc != 6 && tc != -1) {
				flag = false;
				break;
			}
			if (tc == 6 && t > 0 && sc != 0) {
				wind_num++;
			}
		}
		if (flag) {
			return wind_num % 2;
		}
		random_factor++;
	}
}

double Surface::Vol()
{
	double res = 0;
	for_each(faces.begin(), faces.end(), [&](Triangle T) {
		P3D v[3];
		v[0] = vertices[T[0]];
		v[1] = vertices[T[1]];
		v[2] = vertices[T[2]];
		double vol = det(v[0], v[1], v[2]);
		res += vol;
		});
	return res;
}

bool Surface::Orientation()
{
	return Vol() > 0;
}

Surface::Surface()
{
}

Surface::~Surface()
{
}

double SegmentP::Dist(P3D p)
{
	P3D C = vert[1] - vert[0];
	P3D D = p - vert[0];
	double line_dist = C.cross(D).norm() / C.norm();
	return line_dist;
}

bool SegmentP::OnDetect(P3D p)
{
	if (p == vert[0] || p == vert[1])
		return true;
	P3D p0 = p - vert[0];
	P3D p1 = p - vert[1];
	P3D R = vert[1] - vert[0];
	bool test1 = p0 * R > 0;
	bool test2 = p1 * R < 0;
	if (Dist(p) < EPSILON && test1 && test2) {
		return true;
	}
	return false;
}

SegmentP::SegmentP()
{
}

SegmentP::SegmentP(P3D P, P3D Q)
{
	vert[0] = P;
	vert[1] = Q;
}

P3D TriangleP::Norm()
{
	P3D C1 = vert[1] - vert[0];
	P3D C2 = vert[2] - vert[0];
	return C1.cross(C2);
}

double TriangleP::Dist(P3D p)
{
	P3D D = p - vert[0];
	P3D C1 = vert[1] - vert[0];
	P3D C2 = vert[2] - vert[0];
	double vol = det(D, C1, C2);
	double factor = C1.cross(C2).norm();
	double res = vol / factor;
	return res;
}

int TriangleP::OnDetect(P3D p)
{
	if (Dist(p) >= EPSILON)
		return -1;
	P3D n = Norm();
	bool flag = true;
	for (int i = 0; i < 3; i++) {
		P3D B = vert[(i + 1) % 3] - vert[i];
		P3D D = p - vert[i];
		if (D.cross(B) * n > 0) {
			flag = false;
			break;
		}
	}
	if (flag)
		return true;

	for (int i = 0; i < 3; i++) {
		SegmentP S(vert[i], vert[(i + 1) % 3]);
		if (S.OnDetect(p))
			return true;
	}

	return false;
}

bool TriangleP::intersect(
	SegmentP ray,
	P3D& intersection,
	double& r,
	double& s,
	double& t,
	int& tri_pos_code,
	int& seg_pos_code)
{
	tri_pos_code = -1;
	seg_pos_code = -1;

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

	bool flag = true;
	for (int i = 0; i < 3; i++)
		if (flag && vert[i] == intersection)
		{
			tri_pos_code = i;
			flag = false;
		}

	for (int i = 0; i < 3; i++)
		if (flag && SegmentP(vert[i], vert[(i + 1) % 3]).OnDetect(intersection))
		{
			tri_pos_code = i + 3;
			flag = false;
		}
	if (flag && r >= 0 && s >= 0 && r + s <= 1)
		tri_pos_code = 6;

	flag = true;
	for (int i = 0; i < 2; i++)
		if (flag && ray.vert[i] == intersection)
		{
			seg_pos_code = i;
			flag = false;
		}

	if (flag && t >= 0 && t <= 1)
		seg_pos_code = 2;

	return true;
}

bool TriangleP::intersect(
	TriangleP sub,
	SegmentP& intersection, 
	P2D uv[4], 
	int code[4])
{
	double r, s, t;
	int tri_pos_code, seg_pos_code;
	int k = 0;
	//检测顶点是否在三角面上
	for (int i = 0; i < 3; i++) {
		
	}
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

void Path::Triangulate()
{
	//add new points into the mesh except those which is on the old vertex
	map<Triangle, vector<PointInfo::Label> > labels;//每个面上的所属点标签，包含二维uv坐标
	for_each(new_points.begin(), new_points.end(), [&](pair<int, PointInfo> element) {
		if (element.second.pos_code >= 1)
			S->vertices[element.first] = element.second.point;
		else {
			cout << "there is a point with code 0." << endl;
		}
		auto labs = element.second.labels;
		for_each(labs.begin(), labs.end(), [&](PointInfo::Label& lab) {
			labels[lab.tri].push_back(lab);
			});
		});
	//handle each triangle.



	for_each(labels.begin(), labels.end(), [&](pair<Triangle, vector<PointInfo::Label> > element) {
		cout << element.first[0] << element.first[1] << element.first[2] << endl;
		DCEL G;
		map<int, Vertex*> ID;//用于存储DCEL顶点ID
		map<Vertex*, int> inv_ID;
		vector<int> side_point[3];//边界点ID
		side_point[0].push_back(element.first[0]);
		side_point[0].push_back(element.first[1]);
		side_point[1].push_back(element.first[1]);
		side_point[1].push_back(element.first[2]);
		side_point[2].push_back(element.first[2]);
		side_point[2].push_back(element.first[0]);
		Vertex* IV;
		IV = G.AddVertex(P2D(0, 0));
		ID[element.first[0]] = IV;
		inv_ID[IV] = element.first[0];
		IV = G.AddVertex(P2D(1, 0));
		ID[element.first[1]] = IV;
		inv_ID[IV] = element.first[1];
		IV = G.AddVertex(P2D(0, 1));
		ID[element.first[2]] = IV;
		inv_ID[IV] = element.first[2];

		map<int, PointInfo::Label> lab_ID;

		//在G中添加点，分为边界点与内点，边界点需排序处理
		for_each(element.second.begin(), element.second.end(), [&](PointInfo::Label& lab) {
			if (lab.pos_code < 3) {
				lab_ID[lab.belong_ID] = lab;
				return;
			}
			if (lab.pos_code < 6 && lab.pos_code >= 3) {
				side_point[lab.pos_code - 3].push_back(lab.belong_ID);
			}
			Vertex* V = G.AddVertex(lab.uv);
			ID[lab.belong_ID] = V;
			inv_ID[V] = lab.belong_ID;
			lab_ID[lab.belong_ID] = lab;
			});
		//为边界点排序
		P2D base;
		auto comp = [&](int p1, int p2)->bool {
			P2D t1 = ID[p1]->p - base;
			P2D t2 = ID[p2]->p - base;
			return abs(t1.coor[0]) + abs(t1.coor[1]) < abs(t2.coor[0]) + abs(t2.coor[1]);
		};
		base = P2D(0, 0);
		sort(side_point[0].begin(), side_point[0].end(), comp);
		base = P2D(1, 0);
		sort(side_point[1].begin(), side_point[1].end(), comp);
		base = P2D(0, 1);
		sort(side_point[2].begin(), side_point[2].end(), comp);
		//连接边界
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < side_point[i].size() - 1; j++) {
				G.AddEdge(ID[side_point[i][j]], ID[side_point[i][j + 1]]);
			}
		}
		//添内部边
		for_each(segs.begin(), segs.end(), [&](Segment seg) {
			PointInfo::Label end_pt[2];
			for (int i = 0; i < 2; i++)
				if (lab_ID.find(seg[i]) != lab_ID.end())
					end_pt[i] = lab_ID[seg[i]];
				else
					return;

			int code[2] = { end_pt[0].pos_code ,end_pt[1].pos_code };
			bool invalid = code[0] < 3 && (code[1] == code[0] || code[1] == code[0] + 3 || code[1] == (code[0] + 2) % 3 + 3);
			invalid |= code[0] >= 3 && code[0] < 6 && (code[1] == code[0] || code[1] == code[0] - 3 || code[1] == (code[0] - 2) % 3);

			if (code[0] == 6 || code[1] == 6 || !invalid/*判断是否为内部边（incomplete）*/) {
				G.AddEdge(ID[seg[0]], ID[seg[1]]);
			}
			});
		//三角化
		G.Trianglate();
		//删除原面
		S->faces.remove(element.first);
		//添加新面
		for_each(G.face_list.begin(), G.face_list.end(), [&](Face* f) {
			if (f->inner_edge.size() != 0)
				return;
			Triangle new_T;
			new_T[0] = inv_ID[f->outer_edge->start_point];
			new_T[1] = inv_ID[f->outer_edge->next->start_point];
			new_T[2] = inv_ID[f->outer_edge->next->next->start_point];

			cout << new_T[0] << new_T[1] << new_T[2] << endl;

			S->faces.push_back(new_T);
			});
		});

}

void FindIntersection(vector<TriangleP> Ts, list<SegInfo>& intersections)
{
	intersections.clear();
}
