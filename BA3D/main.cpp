#include<iostream>
#include<fstream>
#include<sstream>
#include"Mesh.h"
#include<opencv2/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
using namespace std;

cv::Mat img = cv::Mat::zeros(600, 800, CV_8UC3);

void drawG(DCEL& G);
void readG(string filename, DCEL& G);

int main() {
	int mode;
	cin >> mode;
	srand(556124464);
	if (mode == 0) {

		while (true) {
			DCEL G;
			//readG("data.txt",G);
			int n_cube = 9;
			double p = 0.5;
			vector<P2D> ps;
			vector<Segment> ls;
			ps.resize(n_cube * n_cube);
			for (int i = 0; i < n_cube * n_cube; i++) {
				ps[i] = P2D(i % n_cube + 0.5, i / n_cube + 0.5);
			}
			for (int i = 0; i < n_cube * (n_cube - 1); i++) {
				double ptest = rand() * 1.0 / RAND_MAX;
				if (ptest < p /*|| i / (n_cube - 1) == 0 || i / (n_cube - 1) == n_cube - 1*/) {
					Segment s;
					s[0] = i % (n_cube - 1) + n_cube * (i / (n_cube - 1));
					s[1] = i % (n_cube - 1) + n_cube * (i / (n_cube - 1)) + 1;
					ls.push_back(s);
				}
			}
			for (int i = 0; i < n_cube * (n_cube - 1); i++) {
				double ptest = rand() * 1.0 / RAND_MAX;
				if (ptest < p /*|| i % n_cube == 0 || i % n_cube == n_cube - 1*/) {
					Segment s;
					s[0] = i;
					s[1] = i + n_cube;
					ls.push_back(s);
				}
			}
			G.ConvertFromPlanarGraph(ps, ls);
			drawG(G);
			//cout << G.isTrianglated() << endl;
			cv::imshow("test", img);
			cv::waitKey(0);
			G.Trianglate();
			img = cv::Mat::zeros(600, 800, CV_8UC3);
			/*static int kk = 0;
			for (auto i = G.vertex_list.begin(); i != G.vertex_list.end(); i++) {
				double arg = kk * 0.618 * 2 * PI;
				(*i)->p = (*i)->p + 0.1 * P2D(cos(arg), sin(arg));
				kk++;
			}*/
			drawG(G);
			cout << G.isTrianglated() << endl;
			cv::imshow("test", img);
			cv::waitKey(0);
			img = cv::Mat::zeros(600, 800, CV_8UC3);
		}
		return 0;
	}
	if (mode == 1) {
		DCEL G;
		readG("data.txt", G);
		drawG(G);
		cv::imshow("test", img);
		cv::waitKey(0);
		G.Trianglate();
		img = cv::Mat::zeros(600, 800, CV_8UC3);
		drawG(G);
		cv::imshow("test", img);
		cv::waitKey(0);
	}
	if (mode == 2) {
		TriangleP T(P3D(0, 0, 0), P3D(2, 1, 1), P3D(0, 2, 2));

		SegmentP L(P3D(1, 0.5, 0.5), P3D(1, 1, -2));
		P3D I(0, 0, 0);
		double r, s, t;
		int tc, sc;
		T.intersect(L, I, r, s, t, tc, sc);
		cout << r << " " << s << " " << t << " " << tc << " " << sc << " " << endl;
		SegmentP ray(P3D(1, 2, 3), P3D(2, 0, 5));
		cout << ray.OnDetect(P3D(0, 4, 1)) << endl;

	}
	if (mode == 3) {
		TriangleP T1(P3D(0, 0, 0), P3D(2, 1, 1), P3D(0, 2, 2));
		TriangleP T2(P3D(0, 0, 1), P3D(2, 1, 1), P3D(0, 2, 1));
		//TriangleP T2(P3D(1,0, 4), P3D(1, 0, -1), P3D(1, 2, 1));
		//TriangleP T1(P3D(0.585859, 0.112148, 0.341697), P3D(0.592001, 0.49589, 0.989501), P3D(0.186895, 0.687677, 0.445077));
		//TriangleP T2(P3D(0.674663, 0.00481958, 0.745864), P3D(0.185482, 0.0560274, 0.711849), P3D(0.568723, 0.723776, 0.559884));
		SegmentP intersection;
		P2D uv[4];
		int code[4];

		T1.intersect(T2, intersection, uv, code);


		return 0;
	}
	if (mode == 4) {
		TriangleP T1(P3D(0, 0, 0), P3D(2, 0, 0), P3D(1, 1, 1));
		TriangleP T2(P3D(0, 0.5, 0.5), P3D(2, -1, 0), P3D(1, 1, -1));
		TriangleP T3(P3D(0, 0, 0), P3D(2, 0, 0), P3D(1, -1, 1));
		TriangleP T4(P3D(0, 0, 0), P3D(2, 0, 1), P3D(1, -1, -1));

		vector<TriangleP> Tris;
		Tris.push_back(T1);
		Tris.push_back(T2);
		Tris.push_back(T3);
		Tris.push_back(T4);

		SegInfo segs;
		FindIntersection(Tris, segs);

		return 0;
	}
	if (mode == 5) {
		Surface S;
		S.LoadFromFile("ar3k.obj");
		for (int i = 0; i < 21; i++) {
			for (int j = 0; j < 21; j++) {
				double x = i * 5 - 50;
				double y = j * 5 - 50;
				cout << S.Postion(P3D(x, y, -50)) << " ";
			}
			cout << endl;
		}
		/*cout << S.Postion(P3D(1, 0.6, 0.6)) << " ";*/
	}
	if (mode == 6) {
		Surface S;
		S.LoadFromFile("ar3k.obj");
		cout << S.Vol() << endl;
	}
	if (mode == 7) {
		Surface S;
		S.LoadFromFile("data.obj");
		Path P;
		P.S = &S;
		Path::PointInfo pi;
		Path::PointInfo::Label lab(Triangle(), P2D(0, 0), 0, 0);

		pi.point = P3D(0, 1, 0);
		pi.pos_code = 2;
		lab.tri[0] = 1;
		lab.tri[1] = 0;
		lab.tri[2] = 2;
		lab.belong_ID = 4;
		lab.uv = P2D(0.25, 0.25);
		lab.pos_code = 6;
		pi.labels.push_back(lab);
		P.new_points[4] = pi;
		pi.labels.clear();

		pi.point = P3D(0, 0.5, 0);
		pi.pos_code = 2;
		lab.tri[0] = 1;
		lab.tri[1] = 0;
		lab.tri[2] = 2;
		lab.belong_ID = 5;
		lab.uv = P2D(0.375, 0.375);
		lab.pos_code = 6;
		pi.labels.push_back(lab);
		P.new_points[5] = pi;
		pi.labels.clear();

		pi.point = P3D(0.5, 0.5, 0);
		pi.pos_code = 2;
		lab.tri[0] = 1;
		lab.tri[1] = 0;
		lab.tri[2] = 2;
		lab.belong_ID = 6;
		lab.uv = P2D(0.625, 0.125);
		lab.pos_code = 6;
		pi.labels.push_back(lab);
		P.new_points[6] = pi;
		pi.labels.clear();

		pi.point = P3D(0.5, 1, 0);
		pi.pos_code = 1;
		lab.tri[0] = 1;
		lab.tri[1] = 0;
		lab.tri[2] = 2;
		lab.belong_ID = 7;
		lab.uv = P2D(0.5, 0);
		lab.pos_code = 3;
		pi.labels.push_back(lab);
		lab.tri[0] = 3;
		lab.tri[1] = 0;
		lab.tri[2] = 1;
		lab.belong_ID = 7;
		lab.uv = P2D(0.5, 0.5);
		lab.pos_code = 4;
		pi.labels.push_back(lab);
		P.new_points[7] = pi;
		pi.labels.clear();

		pi.point = P3D(0.25, 1.5, 0);
		pi.pos_code = 1;
		lab.tri[0] = 1;
		lab.tri[1] = 0;
		lab.tri[2] = 2;
		lab.belong_ID = 8;
		lab.uv = P2D(0.25, 0);
		lab.pos_code = 3;
		pi.labels.push_back(lab);
		lab.tri[0] = 3;
		lab.tri[1] = 0;
		lab.tri[2] = 1;
		lab.belong_ID = 8;
		lab.uv = P2D(0.25, 0.75);
		lab.pos_code = 4;
		pi.labels.push_back(lab);
		P.new_points[8] = pi;
		pi.labels.clear();

		pi.point = P3D(0.75, 0.5, 0);
		pi.pos_code = 1;
		lab.tri[0] = 1;
		lab.tri[1] = 0;
		lab.tri[2] = 2;
		lab.belong_ID = 9;
		lab.uv = P2D(0.75, 0);
		lab.pos_code = 3;
		pi.labels.push_back(lab);
		lab.tri[0] = 3;
		lab.tri[1] = 0;
		lab.tri[2] = 1;
		lab.belong_ID = 9;
		lab.uv = P2D(0.75, 0.25);
		lab.pos_code = 4;
		pi.labels.push_back(lab);
		P.new_points[9] = pi;
		pi.labels.clear();

		pi.point = P3D(-0.5, 1, 0);
		pi.pos_code = 1;
		lab.tri[0] = 1;
		lab.tri[1] = 0;
		lab.tri[2] = 2;
		lab.belong_ID = 10;
		lab.uv = P2D(0, 0.5);
		lab.pos_code = 5;
		pi.labels.push_back(lab);
		lab.tri[0] = 3;
		lab.tri[1] = 1;
		lab.tri[2] = 2;
		lab.belong_ID = 10;
		lab.uv = P2D(0.5, 0.5);
		lab.pos_code = 4;
		pi.labels.push_back(lab);
		P.new_points[10] = pi;
		pi.labels.clear();

		Segment seg;
		seg[0] = 4;
		seg[1] = 5;
		P.segs.push_back(seg);
		seg[0] = 5;
		seg[1] = 6;
		P.segs.push_back(seg);
		seg[0] = 7;
		seg[1] = 6;
		P.segs.push_back(seg);
		seg[0] = 7;
		seg[1] = 8;
		P.segs.push_back(seg);
		seg[0] = 7;
		seg[1] = 9;
		P.segs.push_back(seg);
		seg[0] = 10;
		seg[1] = 8;
		P.segs.push_back(seg);
		seg[0] = 10;
		seg[1] = 4;
		P.segs.push_back(seg);

		P.Triangulate();

		S.WriteToFile("res.obj");

		vector<Surface> res;
		S.Cutting(P.segs, res);

		res[0].WriteToFile("res0.obj");
		res[1].WriteToFile("res1.obj");
	}
	if (mode == 111) {
		DCEL G;
		Vertex* V[5];
		V[0] = G.AddVertex(P2D(0, 0));
		V[1] = G.AddVertex(P2D(1, 0));
		V[2] = G.AddVertex(P2D(0, 1));
		V[3] = G.AddVertex(P2D(0.25, 0.25));
		V[4] = G.AddVertex(P2D(0.375, 0.375));
		G.AddEdge(V[0], V[1]);
		G.AddEdge(V[1], V[2]);
		G.AddEdge(V[2], V[0]);
		G.AddEdge(V[3], V[4]);
		drawG(G);
		cv::imshow("test", img);
		cv::waitKey(0);
		img = cv::Mat::zeros(600, 800, CV_8UC3);
		G.Trianglate();
		drawG(G);
		cv::imshow("test", img);
		cv::waitKey(0);
	}
}

void drawG(DCEL& G)
{
	double scale = 50;
	int limit = 1;
	for (auto i = G.face_list.begin(); i != G.face_list.end(); i++) {
		if ((*i)->outer_edge == 0)
			continue;
		int num_b = (*i)->inner_edge.size() + 1;
		vector<HalfEdge*> polys = (*i)->inner_edge;
		polys.push_back((*i)->outer_edge);

		cv::Point** pts = new cv::Point * [num_b];
		int* nts = new int[num_b];
		for (int b = 0; b < num_b; b++) {
			int num_v = 0;
			HalfEdge* l = polys[b];
			do {
				num_v++;
				l = l->next;
			} while (l != polys[b]);
			pts[b] = new cv::Point[num_v];
			nts[b] = num_v;
			int v = 0;
			do {
				pts[b][v].x = l->start_point->p.coor[0] * scale;
				pts[b][v].y = l->start_point->p.coor[1] * scale;
				v++;
				l = l->next;
			} while (l != polys[b]);

		}
		cv::Scalar color = cv::Scalar(rand() % 156 + 100, rand() % 156 + 100, rand() % 156 + 100);
		cv::fillPoly((const cv::InputOutputArray)img,
			(const cv::Point**)pts,
			(const int*)nts,
			num_b,
			color);
		cv::imshow("test", img);
		cv::waitKey(0);
		//limit--;
		if (limit == 0)
			break;
	}
	for (auto i = G.edge_list.begin(); i != G.edge_list.end(); i++) {
		cv::line(img,
			cv::Point(
			(int)((*i)->start_point->p.coor[0] * scale),
				(int)((*i)->start_point->p.coor[1] * scale)),
			cv::Point(
			(int)((*i)->next->start_point->p.coor[0] * scale),
				(int)((*i)->next->start_point->p.coor[1] * scale)),
			cv::Scalar(255, 255, 255),
			1,
			16);
	}
}

void readG(string filename, DCEL& G)
{
	fstream fin(filename, ios::in);
	string str;

	vector<P2D> points;
	vector<Segment> edges;
	while (getline(fin, str)) {
		stringstream ss(str);
		char c;
		ss >> c;
		if (c == 'v') {
			double x, y;
			ss >> x >> y;
			points.push_back(P2D(x, y));
		}
		if (c == 'l') {
			int s1, s2;
			ss >> s1 >> s2;
			Segment s;
			s[0] = s1;
			s[1] = s2;
			edges.push_back(s);
		}
	}
	G.ConvertFromPlanarGraph(points, edges);
	fin.close();
}
