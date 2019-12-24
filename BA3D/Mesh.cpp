#include "Mesh.h"
#include"UnionFind.h"
#include <iostream>



void Surface::Cutting(vector<Segment> segs, vector<Surface>& result)
{
	result.clear();
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

void Surface::Pasting(vector<Surface> pieces)
{
	faces.clear();
	vertices.clear();
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
				vertices[j->first + offsets[o]] = j->second;
			}
		}
		for (auto j = i->faces.begin(); j != i->faces.end(); j++) {
			(*j)[0] = index_map[(*j)[0] + offsets[o]];
			(*j)[1] = index_map[(*j)[1] + offsets[o]];
			(*j)[2] = index_map[(*j)[2] + offsets[o]];
			faces.push_back(*j);
		}
		o++;
	}
}

void Surface::LoadFromFile(string filename)
{
	fstream fin(filename, ios::in);
	string str;


	int v = 0;
	while (getline(fin, str)) {
		stringstream ss(str);
		char c = 0;
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
	bool sign = Orientation();
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
			if ((sc == 0 || !nontrivial) && T.OnDetect(p) != -1) {
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
			return ((wind_num % 2 == 1) ^ (!sign)) ? 1 : 0;
		}
		random_factor++;
	}
}

int Surface::Postion(TriangleP T_)
{
	P3D p = (1.0 / 3) * (T_.vert[0] + T_.vert[1] + T_.vert[2]);

	int random_factor = 0;
	bool sign = Orientation();
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
			if ((sc == 0 || !nontrivial) && T.OnDetect(p) != -1) {
				P3D n1 = T.Norm();
				P3D n2 = T_.Norm();
				return n1 * n2 > 0 ? 2 : 3;
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
			return ((wind_num % 2 == 1) ^ (!sign)) ? 1 : 0;
		}
		random_factor++;
	}
	return 0;
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
	return res / 6;
}

bool Surface::Orientation()
{
	return Vol() > 0;
}

int Surface::MaxIndex()
{
	int m = 0;
	for_each(vertices.begin(), vertices.end(), [&](auto& element) {
		if (element.first > m)
			m = element.first;
		});
	return m;
}

void Surface::Intersect(Surface sub, SegInfo& SI1, SegInfo& SI2)
{
	SegInfo SI;
	vector<TriangleP> Tris;
	int offset = faces.size();
	for_each(faces.begin(), faces.end(), [&](Triangle& element) {
		TriangleP TP;
		TP.vert[0] = vertices[element[0]];
		TP.vert[1] = vertices[element[1]];
		TP.vert[2] = vertices[element[2]];
		Tris.push_back(TP);
		});
	for_each(sub.faces.begin(), sub.faces.end(), [&](Triangle& element) {
		TriangleP TP;
		TP.vert[0] = sub.vertices[element[0]];
		TP.vert[1] = sub.vertices[element[1]];
		TP.vert[2] = sub.vertices[element[2]];

		Tris.push_back(TP);
		});

	FindIntersection(Tris, SI);

	for_each(SI.begin(), SI.end(), [&](pair<const SegmentP, map<int, SOF>>& element) {
		//排除平凡交线
		if (element.second.size() <= 2)
		{
			bool flag = true;
			for_each(element.second.begin(), element.second.end(), [&](pair<const int, SOF>& sof) {
				SOF S = sof.second;
				if (S.code[0] >= 3 || S.code[1] >= 3)
					flag = false;
				});
			if (flag)
				return;
		}
		//分类
		for_each(element.second.begin(), element.second.end(), [&](pair<const int, SOF>& sof) {
			SOF S = sof.second;
			int index = sof.first;
			if (index < offset)
				SI1[element.first][index] = S;
			else
				SI2[element.first][index - offset] = S;
			});
		});
}

Surface Surface::meet(Surface sub)
{
	Surface res;
	vector<Surface> pieces[3];
	SegInfo SI[2];

	Intersect(sub, SI[0], SI[1]);
	Path P[2] = { Path(*this,SI[0]),Path(sub,SI[1]) };
	P[0].Triangulate();
	//P[0].S->WriteToFile("res_0.obj");
	P[1].Triangulate();
	//P[1].S->WriteToFile("res_1.obj");

	P[0].S->Cutting(P[0].segs, pieces[0]);
	P[1].S->Cutting(P[1].segs, pieces[1]);

	///初步选择
	for (auto i = pieces[1].begin(); i != pieces[1].end(); i++) {
		Triangle T0 = *i->faces.begin();
		TriangleP TP0(i->vertices[T0[0]], i->vertices[T0[1]], i->vertices[T0[2]]);
		int pos = Postion(TP0);
		if (pos == 1 || pos == 2)
			pieces[2].push_back(*i);
	};
	for (auto i = pieces[0].begin(); i != pieces[0].end(); i++) {
		Triangle T0 = *i->faces.begin();
		TriangleP TP0(i->vertices[T0[0]], i->vertices[T0[1]], i->vertices[T0[2]]);
		if (sub.Postion(TP0) == 1)
			pieces[2].push_back(*i);
	};

	res.Pasting(pieces[2]);
	return res;
}

Surface Surface::join(Surface sub)
{
	Surface S1 = inverse();
	Surface S2 = sub.inverse();
	Surface res = S1.meet(S2);
	return res.inverse();
}

Surface Surface::inverse()
{
	Surface res(*this);
	for_each(res.faces.begin(), res.faces.end(), [&](Triangle& T) {
		swap(T[1], T[2]);
		});
	return res;
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

int SegmentP::OnDetect(P3D p)
{
	for (int i = 0; i < 2; i++)
		if (p == vert[i])
			return i;
	P3D p0 = p - vert[0];
	P3D p1 = p - vert[1];
	P3D R = vert[1] - vert[0];
	bool test1 = p0 * R > 0;
	bool test2 = p1 * R < 0;
	if (Dist(p) < EPSILON && test1 && test2) {
		return 2;
	}
	return -1;
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
	return abs(res);
}

int TriangleP::OnDetect(P3D p)
{
	if (Dist(p) >= EPSILON)
		return -1;

	for (int i = 0; i < 3; i++) {
		SegmentP S(vert[i], vert[(i + 1) % 3]);
		int r = S.OnDetect(p);
		if (r == -1)
			continue;
		else if (r == 0 || r == 1)
			return (i + r) % 3;
		else if (r == 2)
			return 3 + i;
	}

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
		return 6;
	else
		return -1;
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


	int code = OnDetect(intersection);
	bool flag = code == -1 || code == 6;
	if (flag && r >= 0 && s >= 0 && r + s <= 1)
		tri_pos_code = 6;
	else
		tri_pos_code = code;


	code = ray.OnDetect(intersection);
	flag = code == -1 || code == 2;
	if (flag && t >= 0 && t <= 1)
		seg_pos_code = 2;
	else
		seg_pos_code = code;

	return true;
}

bool TriangleP::intersect(
	TriangleP sub,
	SegmentP& intersection,
	P2D uv[4],
	int code[4])
{
	P2D rs[2];
	double t;
	int tri_pos_code[2], seg_pos_code;
	int k = 0;
	auto ADD = [&](P3D& p) {



		if (k == 0) {
			intersection.vert[0] = p;
			uv[0] = rs[0];
			uv[1] = rs[1];
			code[0] = tri_pos_code[0];
			code[1] = tri_pos_code[1];
			k++;
		}
		else if (k == 1) {
			if (intersection.vert[0] == p)
				return;
			intersection.vert[1] = p;
			uv[2] = rs[0];
			uv[3] = rs[1];
			code[2] = tri_pos_code[0];
			code[3] = tri_pos_code[1];
			k++;
		}
		else if (!(intersection.vert[0] == p) && !(intersection.vert[1] == p)) {
			cout << "error occurs in TRIANGLE_INTERSECT because three or more endpoints are detected" << endl;
		}
	};

	//判断平行
	bool parallel_check = false;
	P3D T = sub.vert[1] - sub.vert[0] + vert[0];
	P3D W = sub.vert[2] - sub.vert[0] + vert[0];
	if (Dist(T) < EPSILON && Dist(W) < EPSILON)
		//parallel_check = true;
		return false;

	//检测顶点
	P2D vert_uv[3] = { P2D(0,0),P2D(1,0),P2D(0,1) };
	for (int i = 0; i < 3; i++) {
		int d_code = OnDetect(sub.vert[i]);
		if (d_code != -1) {
			P2D res_uv = AffineCoor(sub.vert[i]);
			rs[0] = res_uv;
			rs[1] = vert_uv[i];
			tri_pos_code[0] = d_code;
			tri_pos_code[1] = i;
			ADD(sub.vert[i]);
		}
	}
	for (int i = 0; i < 3; i++) {
		int d_code = sub.OnDetect(vert[i]);
		if (d_code != -1) {
			P2D res_uv = sub.AffineCoor(vert[i]);
			rs[1] = res_uv;
			rs[0] = vert_uv[i];
			tri_pos_code[1] = d_code;
			tri_pos_code[0] = i;
			ADD(vert[i]);
		}
	}

	//if (parallel_check)//若平行，根据已找出的endpoint数量确定结果
		//return k == 2;

	//检测边
	for (int i = 0; i < 3; i++) {
		SegmentP line(sub.vert[i], sub.vert[(i + 1) % 3]);
		P3D pt;
		bool res = intersect(line, pt, rs[0].coor[0], rs[0].coor[1], t, tri_pos_code[0], seg_pos_code);
		if (tri_pos_code[0] == -1 || seg_pos_code != 2)
			continue;
		P2D t_uv = t * vert_uv[(i + 1) % 3] + (1 - t) * vert_uv[i];
		rs[1] = t_uv;
		tri_pos_code[1] = 3 + i;
		ADD(pt);
	}
	for (int i = 0; i < 3; i++) {
		SegmentP line(vert[i], vert[(i + 1) % 3]);
		P3D pt;
		bool res = sub.intersect(line, pt, rs[1].coor[0], rs[1].coor[1], t, tri_pos_code[1], seg_pos_code);
		if (tri_pos_code[1] == -1 || seg_pos_code != 2)
			continue;
		P2D t_uv = t * vert_uv[(i + 1) % 3] + (1 - t) * vert_uv[i];
		rs[0] = t_uv;
		tri_pos_code[0] = 3 + i;
		ADD(pt);
	}

	//规范交线的方向
	if (!LESS(intersection.vert[0], intersection.vert[1])) {
		swap(intersection.vert[0], intersection.vert[1]);
		swap(uv[0], uv[2]);
		swap(uv[1], uv[3]);
		swap(code[0], code[2]);
		swap(code[1], code[3]);
	}

	return k == 2;
}

P2D TriangleP::AffineCoor(P3D p)
{
	P3D X = vert[1] - vert[0];
	P3D Y = vert[2] - vert[0];
	double xx = X * X;
	double xy = X * Y;
	double yy = Y * Y;
	double xp = X * (p - vert[0]);
	double yp = Y * (p - vert[0]);
	double cm = xx * yy - xy * xy;
	double um = xp * yy - yp * xy;
	double vm = xx * yp - xp * xy;
	return P2D(um / cm, vm / cm);
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
	map<int, int> index_map;
	for_each(new_points.begin(), new_points.end(), [&](pair<int, PointInfo> element) {
		if (element.second.pos_code >= 1)
			S->vertices[element.first] = element.second.point;
		/*else {
			index_map[element.first] = element.second.labels.begin()->belong_ID;
		}*/
		auto labs = element.second.labels;
		for_each(labs.begin(), labs.end(), [&](PointInfo::Label& lab) {
			labels[lab.tri].push_back(lab);
			});
		});
	/*for_each(segs.begin(), segs.end(), [&](Segment& line) {
		if (index_map.find(line[0]) != index_map.end())
			line[0] = index_map[line[0]];
		if (index_map.find(line[1]) != index_map.end())
			line[1] = index_map[line[1]];
		});*/
		//handle each triangle.



	for_each(labels.begin(), labels.end(), [&](pair<Triangle, vector<PointInfo::Label> > element) {
		cout << element.first[0] << " " << element.first[1] << " " << element.first[2] << endl;


		if (element.first[0] == 0 && element.first[1] == 5 && element.first[2] == 4)
			cout << "?" << endl;


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
			bool invalid = code[0] < 3 && (code[1] < 3 || code[1] == code[0] + 3 || code[1] == (code[0] + 2) % 3 + 3);
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

			S->faces.push_back(new_T);
			});
		});

}

Path::Path(Surface S, SegInfo SI)
{
	this->S = new Surface(S);
	int m = S.MaxIndex() + 1;
	map<P3D, int, comparator> pool;
	for_each(S.vertices.begin(), S.vertices.end(), [&](pair<int, P3D> element) {
		pool[element.second] = element.first;
		});
	for_each(SI.begin(), SI.end(), [&](pair<const SegmentP, map<int, SOF>>& element) {
		if (pool.find(element.first.vert[0]) == pool.end())
			pool[element.first.vert[0]] = m++;
		if (pool.find(element.first.vert[1]) == pool.end())
			pool[element.first.vert[1]] = m++;
		new_points[pool[element.first.vert[0]]].point = element.first.vert[0];
		new_points[pool[element.first.vert[1]]].point = element.first.vert[1];

		for_each(element.second.begin(), element.second.end(), [&](pair<const int, SOF>& sof) {
			PointInfo::Label lab;

			auto d = S.faces.begin();
			for (int i = 0; i < sof.first; i++)
				d++;
			lab.tri = *d;

			lab.belong_ID = pool[element.first.vert[0]];
			lab.pos_code = sof.second.code[0];
			lab.uv = sof.second.uv[0];
			auto& L0 = new_points[pool[element.first.vert[0]]].labels;
			if (find(L0.begin(), L0.end(), lab) == L0.end())
				new_points[pool[element.first.vert[0]]].labels.push_back(lab);
			lab.belong_ID = pool[element.first.vert[1]];
			lab.pos_code = sof.second.code[1];
			lab.uv = sof.second.uv[1];
			auto& L1 = new_points[pool[element.first.vert[1]]].labels;
			if (find(L1.begin(), L1.end(), lab) == L1.end())
				new_points[pool[element.first.vert[1]]].labels.push_back(lab);

			});

		new_points[pool[element.first.vert[0]]].pos_code = new_points[pool[element.first.vert[0]]].labels.begin()->pos_code / 3;
		new_points[pool[element.first.vert[1]]].pos_code = new_points[pool[element.first.vert[1]]].labels.begin()->pos_code / 3;

		Segment line;
		line[0] = pool[element.first.vert[0]];
		line[1] = pool[element.first.vert[1]];
		segs.push_back(line);
		});

	return;
}

void FindIntersection(vector<TriangleP> Ts, SegInfo& intersections)
{
	intersections.clear();
	for (int i = 0; i < Ts.size(); i++)
		for (int j = i + 1; j < Ts.size(); j++) {
			P2D uv[4];
			SegmentP seg;
			int code[4];
			bool res = Ts[i].intersect(Ts[j], seg, uv, code);
			if (res) {
				intersections[seg][i] = SOF(uv[0], uv[2], code[0], code[2]);
				intersections[seg][j] = SOF(uv[1], uv[3], code[1], code[3]);
			}
		}


	cout << "Graphics3D[{";
	bool _1 = false;
	for_each(Ts.begin(), Ts.end(), [&](TriangleP& element) {
		if (_1) {
			cout << ',';
		}
		cout << "{Opacity[0.5],EdgeForm[],Triangle[{{"
			<< element.vert[0].coor[0] << ","
			<< element.vert[0].coor[1] << ","
			<< element.vert[0].coor[2] << "},{"
			<< element.vert[1].coor[0] << ","
			<< element.vert[1].coor[1] << ","
			<< element.vert[1].coor[2] << "},{"
			<< element.vert[2].coor[0] << ","
			<< element.vert[2].coor[1] << ","
			<< element.vert[2].coor[2] << "}}]}";

		_1 = true;
		});
	for_each(intersections.begin(), intersections.end(), [&](auto& element) {
		if (element.second.size() == 2)
		{
			bool flag = true;
			for_each(element.second.begin(), element.second.end(), [&](pair<const int, SOF>& sof) {
				SOF S = sof.second;
				if (S.code[0] >= 3 || S.code[1] >= 3)
					flag = false;
				});
			if (flag)
				return;
		}
		/*else {
			int k = 0;
			set<P3D, comparator> pool;
			for_each(element.second.begin(), element.second.end(), [&](pair<const int, SOF>& sof) {
				SOF S = sof.second;
				if (S.code[0] < 3 && S.code[1] < 3) {
					TriangleP T;
					T.vert[0] = element.first.vert[0];
					T.vert[1] = element.first.vert[1];
					T.vert[2] = Ts[sof.first].vert[(S.code[1] + 1) % 3 == S.code[0] ? (S.code[1] + 2) % 3 : (S.code[1] + 1) % 3];
					P3D n = (1 / T.Norm().norm()) * T.Norm();
					k++;
					pool.insert(n);
				}
				});
			int num = element.second.size() - k + pool.size();
			if (num <= 2)
				return;
		}*/
		cout << ',';
		cout << "{Dashed,Thick,Line[{{";
		cout << element.first.vert[0].coor[0] << ","
			<< element.first.vert[0].coor[1] << ","
			<< element.first.vert[0].coor[2] << "},{"
			<< element.first.vert[1].coor[0] << ","
			<< element.first.vert[1].coor[1] << ","
			<< element.first.vert[1].coor[2] << "}";
		cout << "}]}";
		});
	cout << "}]" << endl;
}
