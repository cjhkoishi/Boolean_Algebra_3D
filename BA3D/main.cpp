#include<iostream>
#include<fstream>
#include<sstream>
#include"Mesh.h"
#include<opencv2/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include "Yin.h"
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
				ps[i] = P2D(i % n_cube + 0.5 + 0.2 * rand() * 1.0 / RAND_MAX, i / n_cube + 0.5 + 0.2 * rand() * 1.0 / RAND_MAX);
			}
			for (int i = 0; i < n_cube * (n_cube - 1); i++) {
				double ptest = rand() * 1.0 / RAND_MAX;
				if (ptest < p || i / (n_cube - 1) == 0 || i / (n_cube - 1) == n_cube - 1/**/) {
					Segment s;
					s[0] = i % (n_cube - 1) + n_cube * (i / (n_cube - 1));
					s[1] = i % (n_cube - 1) + n_cube * (i / (n_cube - 1)) + 1;
					ls.push_back(s);
				}
			}
			for (int i = 0; i < n_cube * (n_cube - 1); i++) {
				double ptest = rand() * 1.0 / RAND_MAX;
				if (ptest < p || i % n_cube == 0 || i % n_cube == n_cube - 1/**/) {
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
		TriangleP T(P3D(0.1675, 0.8975, 0.2245), P3D(2.7843, 1.1145, 1.4268), P3D(0, 2.1246, 2.7586));

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
		TriangleP T1(P3D(0, 0, 0), P3D(2, 0, 0), P3D(1, 2, 0));
		TriangleP T2(P3D(0, 0, 0), P3D(2, 0, 0), P3D(1, 1, 2));
		TriangleP T3(P3D(1, 2, 0), P3D(2, 0, 0), P3D(1, 1, 2));
		TriangleP T4(P3D(0, 0, 0), P3D(1, 2, 0), P3D(1, 1, 2));
		TriangleP T5(P3D(0, 0, 1), P3D(0, 2, 1), P3D(2, 0, 1));
		TriangleP T6(P3D(2, 2, 1), P3D(0, 2, 1), P3D(2, 0, 1));

		vector<TriangleP> Tris;
		Tris.push_back(T1);
		Tris.push_back(T2);
		Tris.push_back(T3);
		Tris.push_back(T4);
		Tris.push_back(T5);
		Tris.push_back(T6);
		SegInfo segs;
		FindIntersection(Tris, segs);

		return 0;
	}
	if (mode == 10) {
		Surface S1, S2;
		vector<TriangleP> Tris;
		SegInfo segs1, segs2;
		S1.LoadFromFile("sphere.obj");
		S2.LoadFromFile("model_.obj");

		/*for_each(S1.faces.begin(), S1.faces.end(), [&](Triangle& T) {
			TriangleP TP;
			double sca = 2.2;
			TP.vert[0] = sca * S1.vertices[T[0]];
			TP.vert[1] = sca * S1.vertices[T[1]];
			TP.vert[2] = sca * S1.vertices[T[2]];
			Tris.push_back(TP);
			});
		for_each(S2.faces.begin(), S2.faces.end(), [&](Triangle& T) {
			TriangleP TP;
			TP.vert[0] = S2.vertices[T[0]];
			TP.vert[1] = S2.vertices[T[1]];
			TP.vert[2] = S2.vertices[T[2]];
			Tris.push_back(TP);
			});
		FindIntersection(Tris, segs);
		S1.Intersect(S2,segs1,segs2);
		Path P1(S2,segs2);
		P1.Triangulate();
		P1.S->WriteToFile("res.obj");
		vector<Surface> res;
		P1.S->Cutting(P1.segs, res);

		int index=0;
		for_each(res.begin(), res.end(), [&](Surface& element) {
			stringstream ss("res");
			ss << index++ << ".obj";
			element.WriteToFile(ss.str());
			});*/
		Surface S3 = S1.join(S2);
		S3.WriteToFile("res.obj");
		cout << S1.OCScheck() << endl;
		cout << S2.OCScheck() << endl;
		cout << S3.OCScheck() << endl;
		//S1.Intersect(S2, segs1, segs2);

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
		P3D p1(1e-10, 1e-10, 1e-10);
		P3D p2(0, 0, 0);
		cout << (p1 == p2) << endl;
	}
	if (mode == 7) {
		Surface bag;
		bag.LoadFromFile("double.obj");
		Yin Y;
		Y.split(bag);
		Y.OutPut("W");
		return 0;
	}
	if (mode == 111) {
		Yin Y;
		Surface cube[4];
		cube[0].LoadFromFile("Yin/CubeT/cube0.obj");
		cube[1].LoadFromFile("Yin/CubeT/cube1.obj");
		cube[2].LoadFromFile("Yin/CubeT/cube2.obj");
		cube[3].LoadFromFile("Yin/CubeT/cube3.obj");

		Y.boundarys.push_back(cube[0]);
		Y.boundarys.push_back(cube[1]);
		Y.boundarys.push_back(cube[2]);
		Y.boundarys.push_back(cube[3]);

		Y.GenerateHasseDigram();
		Y.OutPut("C", 1);
		return 0;
	}
	if (mode == 222) {
		Yin Y[2];
		Surface S[2];
		S[0].LoadFromFile("tor.obj");
		S[1].LoadFromFile("2xICO.obj");
		Y[0].split(S[0]);
		Y[1].split(S[1]);

		Yin res=Y[0].join(Y[1]);
		cout << res.boundarys.size() << endl;
		res.OutPut("W",1);

		return 0;
	}
	if (mode == 333) {
		Surface adj;
		adj.LoadFromFile("ar3k_.obj");
		for (auto i = adj.vertices.begin(); i != adj.vertices.end(); i++) {
			i->second = 2 * (i->second);
		}
		adj.WriteToFile("ar3k_.obj");
		return 0;
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
		//cv::imshow("test", img);
		//cv::waitKey(0);
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
