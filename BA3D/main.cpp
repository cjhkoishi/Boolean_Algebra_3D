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
	int mode = cin.get() - '0';
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
		SegmentP L(P3D(0, 2, 0), P3D(0, 2, -2));
		P3D I(0, 0, 0);
		double r, s, t;
		int tc, sc;
		T.intersect(L, I, r, s, t, tc, sc);
		cout << r << " " << s << " " << t << " " << tc << " " << sc << " " << endl;
		SegmentP ray(P3D(1, 2, 3), P3D(2, 0, 5));
		cout << ray.OnDetect(P3D(0, 4, 1)) << endl;

	}
	if (mode == 3) {
		MeshWithCell MC;
		MC.LoadFromFile("data.obj");
		Segment s;
		s[0] = 0;
		s[1] = 1;
		MC.cell.push_back(s);
		s[0] = 1;
		s[1] = 2;
		MC.cell.push_back(s);
		s[0] = 2;
		s[1] = 0;
		MC.cell.push_back(s);
		vector<Surface> res;
		MC.Cutting(res);
		res[0].WriteToFile("data0.obj");
		res[1].WriteToFile("data1.obj");
		return 0;
	}
	if (mode == 4) {
		Surface S;
		S.LoadFromFile("data.obj");
		for (int i = 0; i < 21; i++) {
			for (int j = 0; j < 21; j++) {
				double x = i * 0.1;
				double y = j * 0.1;
				cout << S.Postion(P3D(x, y, 0)) << " ";
			}
			cout << endl;
		}
		/*cout << S.Postion(P3D(1, 0.6, 0.6)) << " ";*/
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
