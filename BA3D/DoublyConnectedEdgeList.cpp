#include "DoublyConnectedEdgeList.h"
#include"Polygon.h"
#include<set>
#include <iostream>
#include <map>
#include <stack>

Face* DoublyConnectedEdgeList::GetPostion(P2D p)
{
	Face* res = 0;
	for (auto i = face_list.begin(); i != face_list.end(); i++) {
		bool detect_res = (*i)->isInside(p);
		if (detect_res) {
			res = *i;
			break;
		}
	}
	return res;
}

void DoublyConnectedEdgeList::GetEdgesByVertex(Vertex* v, vector<HalfEdge*>& edges)
{
	edges.clear();
	HalfEdge* l = v->inc_edge;
	if (l == 0)
		return;
	do {
		edges.push_back(l);
		l = l->prev->twin;
	} while (l != v->inc_edge);
}

void DoublyConnectedEdgeList::ConvertFromPlanarGraph(vector<P2D> points, vector<Segment> edges)
{
	vector<list<Vertex*>::iterator> index;
	for (int i = 0; i < points.size(); i++) {
		Vertex* new_p = new Vertex(points[i]);
		new_p->id = i;
		vertex_list.push_back(new_p);
		index.push_back(--vertex_list.end());
	}
	for (int i = 0; i < edges.size(); i++) {
		AddEdge(*index[edges[i][0]], *index[edges[i][1]]);
	}
}

void DoublyConnectedEdgeList::AddEdge(Vertex* v1, Vertex* v2)
{
	HalfEdge* test = v1->inc_edge;
	if (test != 0)
		do {
			if (test->next->start_point == v2) {
				cout << "the edge had been added." << endl;
				return;
			}
			test = test->prev->twin;
		} while (test != v1->inc_edge);


		HalfEdge* e1 = new HalfEdge;
		HalfEdge* e2 = new HalfEdge;
		edge_list.push_back(e1);
		edge_list.push_back(e2);
		Face* target;
		P2D p1 = v1->p;
		P2D p2 = v2->p;
		P2D mid = 0.5 * (p1 + p2);
		target = GetPostion(mid);
		///non-intersection test///

		///select the neib edge///
		HalfEdge* in1, * in2, * out1, * out2;
		vector<HalfEdge*> sort_pool;
		P2D base = p2 - p1;
		auto preb = [&](HalfEdge* l1, HalfEdge* l2)->bool {
			P2D h1 = l1->next->start_point->p - l1->start_point->p;
			P2D h2 = l2->next->start_point->p - l2->start_point->p;
			return base.Angle(h1) < base.Angle(h2);
		};

		if (v1->inc_edge == 0) {
			in1 = e2;
			out1 = e1;
		}
		else {
			GetEdgesByVertex(v1, sort_pool);
			base = p2 - p1;
			sort(sort_pool.begin(), sort_pool.end(), preb);
			in1 = sort_pool[0]->twin;
			out1 = sort_pool[sort_pool.size() - 1];
		}
		if (v2->inc_edge == 0) {
			in2 = e1;
			out2 = e2;
		}
		else {
			GetEdgesByVertex(v2, sort_pool);
			base = p1 - p2;
			sort(sort_pool.begin(), sort_pool.end(), preb);
			in2 = sort_pool[0]->twin;
			out2 = sort_pool[sort_pool.size() - 1];
		}

		///set vetex and edge info///
		e1->twin = e2;
		e1->next = out2;
		e1->prev = in1;
		e1->start_point = v1;

		e2->twin = e1;
		e2->next = out1;
		e2->prev = in2;
		e2->start_point = v2;

		in1->next = e1;
		out1->prev = e2;
		in2->next = e2;
		out2->prev = e1;

		v1->inc_edge = e1;
		v2->inc_edge = e2;
		///update face info///
		vector<HalfEdge*> original_boundary = target->inner_edge;
		if (target->outer_edge != 0)
			original_boundary.push_back(target->outer_edge);

		if (v1->id != v2->id) {
			///e+1///
			e1->inc_face = target;
			e2->inc_face = target;
			int remove_index = 0;
			bool trigger = false;
			for (int i = 0; i < original_boundary.size(); i++) {
				if (original_boundary[i]->isSameBoundary(e1))
				{
					original_boundary.erase(original_boundary.begin() + i);
					i--;
				}
			}
			original_boundary.push_back(e1);
			target->outer_edge = 0;
			target->inner_edge.clear();


			for (int i = 0; i < original_boundary.size(); i++) {
				if (original_boundary[i]->GetSign()) {
					target->outer_edge = original_boundary[i];
				}
				else {
					target->inner_edge.push_back(original_boundary[i]);
				}
			}

			int id1 = v1->id, id2 = v2->id;
			for (auto i = vertex_list.begin(); i != vertex_list.end(); i++)
				if ((*i)->id == id2)
					(*i)->id = id1;
		}
		else {
			///e+1 f+1///
			int coincide_index = 0;
			HalfEdge* coincide = 0;
			for (int i = 0; i < original_boundary.size(); i++) {
				if (original_boundary[i]->isSameBoundary(e1)) {
					coincide = e1;
					coincide_index = i;
					break;
				}
				if (original_boundary[i]->isSameBoundary(e2)) {
					coincide = e2;
					coincide_index = i;
					break;
				}
			}
			original_boundary[coincide_index] = coincide;
			original_boundary.push_back(coincide == e1 ? e2 : e1);
			face_list.remove(target);
			delete target;
			Face* f1 = new Face;
			Face* f2 = new Face;
			face_list.push_back(f1);
			face_list.push_back(f2);
			bool trigger = false;

			auto updatef = [&](HalfEdge* H, Face* F) {
				HalfEdge* l = H;
				do {
					l->inc_face = F;
					l = l->next;
				} while (l != H);
			};

			for (int i = 0; i < original_boundary.size(); i++) {
				if (original_boundary[i]->GetSign()) {
					(trigger ? f2 : f1)->outer_edge = original_boundary[i];
					updatef(original_boundary[i], (trigger ? f2 : f1));
					trigger = true;
				}
			}
			for (int i = 0; i < original_boundary.size(); i++) {
				if (!original_boundary[i]->GetSign()) {
					P2D b_mid = 0.5 * (original_boundary[i]->start_point->p + original_boundary[i]->next->start_point->p);
					bool isAdjacent = original_boundary[i]->twin->isSameBoundary(f1->outer_edge);
					if (f1->isInside(b_mid) && !isAdjacent) {
						f1->inner_edge.push_back(original_boundary[i]);
						updatef(original_boundary[i], f1);
					}
					else {
						f2->inner_edge.push_back(original_boundary[i]);
						updatef(original_boundary[i], f2);
					}
				}
			}
		}
}

void DoublyConnectedEdgeList::Trianglate()
{
	struct Trapezoid {
		HalfEdge* e[2];
		HalfEdge* top;
	};
	class HE_Comp {
	public:
		comparator less;
		bool operator()(HalfEdge* const e1, HalfEdge* const e2) const {
			bool l = less(e1->start_point->p, e2->start_point->p);
			bool g = less(e2->start_point->p, e1->start_point->p);
			return g || !l && !g && less(e1->next->start_point->p, e2->next->start_point->p);
		}
	};
	list<pair<HalfEdge*, HalfEdge*>> new_edges;
	comparator less;
	auto type_func = [&](HalfEdge* l)->int {
		P2D v1 = l->prev->start_point->p;
		P2D v2 = l->start_point->p;
		P2D v3 = l->next->start_point->p;
		P2D d1 = v2 - v1;
		P2D d2 = v3 - v2;
		if (!(less(v1, v2) ^ less(v2, v3)))
			return 0;
		if (less(v1, v2)) {
			if (d1.cross(d2) > 0)
				return 1;
			else
				return 3;
		}
		else {
			if (d1.cross(d2) > 0)
				return 2;
			else
				return 4;
		}
	};//calculate type of vertex


	for (auto fi = face_list.begin(); fi != face_list.end(); fi++) {
		Face* f = *fi;
		map<HalfEdge*, int, HE_Comp> Q;//0=normal 1=start 2=end 3=split 4=merge 5=dealed
		list<Trapezoid> T;


		vector<HalfEdge*> all_boundary = f->inner_edge;
		if (f->outer_edge != 0)
			all_boundary.push_back(f->outer_edge);
		else
			continue;

		for (auto i = all_boundary.begin(); i != all_boundary.end(); i++) {
			HalfEdge* l = *i;
			do {
				Q[l] = type_func(l);
				l = l->next;
			} while (l != *i);
		}

		for (auto ev = Q.begin(); ev != Q.end(); ev++) {
			HalfEdge* current_p = ev->first;

			if (ev->second == 0) {
				P2D d = current_p->next->start_point->p - current_p->start_point->p;
				bool dir = less(d, P2D::O);

				for (auto t = T.begin(); t != T.end(); t++) {
					HalfEdge* origin_p = t->top;
					bool flag = false;
					if (t->e[0] == current_p->prev && dir) {
						t->e[0] = current_p;
						t->top = current_p;
						flag = true;
					}
					else if (t->e[1] == current_p && !dir) {
						t->e[1] = current_p->prev;
						t->top = current_p;
						flag = true;
					}

					if (flag && Q[origin_p] == 4) {
						new_edges.push_back(pair<HalfEdge*, HalfEdge*>(origin_p, current_p));
						Q[origin_p] = 5;
					}
					if (flag)
						break;
				}
			}

			if (ev->second == 1) {
				Trapezoid new_tr;
				new_tr.e[0] = current_p;
				new_tr.e[1] = current_p->prev;
				new_tr.top = current_p;
				T.push_back(new_tr);
			}

			if (ev->second == 2) {
				for (auto t = T.begin(); t != T.end(); t++) {
					if (t->e[1] == current_p) {
						if (Q[t->top] == 4) {
							new_edges.push_back(pair<HalfEdge*, HalfEdge*>(t->top, current_p));
							Q[t->top] = 5;
						}
						T.erase(t);
						break;
					}
				}
			}

			if (ev->second == 3) {
				for (auto t = T.begin(); t != T.end(); t++) {
					P2D r = current_p->start_point->p;
					P2D r0 = r - t->e[0]->start_point->p;
					P2D d0 = t->e[0]->next->start_point->p - t->e[0]->start_point->p;
					P2D r1 = r - t->e[1]->start_point->p;
					P2D d1 = t->e[1]->next->start_point->p - t->e[1]->start_point->p;
					if (d0.cross(r0) > 0 && d1.cross(r1) > 0) {

						new_edges.push_back(pair<HalfEdge*, HalfEdge*>(t->top, current_p));

						Trapezoid new_tr;
						new_tr.e[0] = current_p;
						new_tr.e[1] = t->e[1];
						new_tr.top = current_p;
						t->e[1] = current_p->prev;
						t->top = current_p;
						T.push_back(new_tr);

						break;
					}
				}
			}

			if (ev->second == 4) {
				list<Trapezoid>::iterator t0, t1;
				for (auto t = T.begin(); t != T.end(); t++)
					if (t->e[1] == current_p) {
						if (Q[t->top] == 4) {
							new_edges.push_back(pair<HalfEdge*, HalfEdge*>(t->top, current_p));
							Q[t->top] = 5;
						}
						t0 = t;
						break;
					}
				for (auto t = T.begin(); t != T.end(); t++)
					if (t->e[0] == current_p->prev) {
						if (Q[t->top] == 4) {
							new_edges.push_back(pair<HalfEdge*, HalfEdge*>(t->top, current_p));
							Q[t->top] = 5;
						}
						t1 = t;
						break;
					}
				Trapezoid new_tr;
				new_tr.e[0] = t0->e[0];
				new_tr.e[1] = t1->e[1];
				new_tr.top = current_p;
				T.erase(t0);
				T.erase(t1);
				T.push_back(new_tr);

			}
		}
	}
	for (auto i = new_edges.begin(); i != new_edges.end(); i++) {
		AddEdge(i->first->start_point, i->second->start_point);
	}

	new_edges.clear();

	for (auto fi = face_list.begin(); fi != face_list.end(); fi++) {
		Face* f = *fi;
		if (f->outer_edge != 0 && f->inner_edge.size() != 0) {
			cout << "mistake occurs in simple polygons";
			return;
		}
		if (f->outer_edge == 0)
			continue;

		map<HalfEdge*, int, HE_Comp> Q;//0=left 1=right

		HalfEdge* l = f->outer_edge;
		do {
			int type = less(l->next->start_point->p, l->start_point->p) ? 0 : 1;
			Q[l] = type;
			l = l->next;
		} while (l != f->outer_edge);
		Q.rbegin()->second = 2;
		vector<pair<HalfEdge*, int>> V;
		stack<int> S;
		for (auto m = Q.begin(); m != Q.end(); m++)
			V.push_back(*m);

		for (int v = 0; v < V.size(); v++) {
			if (S.size() < 2)
				S.push(v);
			else if (V[v].second == V[S.top()].second) {
				while (S.size()>=2) {
					HalfEdge* t0 = V[v].first;
					HalfEdge* t1 = V[S.top()].first;
					int buf = S.top();
					S.pop();
					HalfEdge* t2 = V[S.top()].first;
					P2D d0 = t0->start_point->p - t1->start_point->p;
					P2D d1 = t1->start_point->p - t2->start_point->p;
					double exproduct = d1.cross(d0);
					if (exproduct > 0&&V[buf].second==0|| exproduct < 0 && V[buf].second == 1)
						new_edges.push_back(pair<HalfEdge*, HalfEdge*>(t0, t2));
					else {
						S.push(buf);
						break;
					}
				}
				S.push(v);
			}
			else {
				int t = S.top();
				if (v == V.size() - 1)
					S.pop();
				while (S.size() != 1) {
					new_edges.push_back(pair<HalfEdge*, HalfEdge*>(V[v].first, V[S.top()].first));
					S.pop();
				}
				S.pop();
				S.push(t);
				S.push(v);
			}
		}
	}
	for (auto i = new_edges.begin(); i != new_edges.end(); i++) {
		AddEdge(i->first->start_point, i->second->start_point);
	}/**/

}

bool DoublyConnectedEdgeList::isTrianglated()
{
	for (auto i = face_list.begin(); i != face_list.end(); i++) {
		if ((*i)->outer_edge == 0)
			continue;
		if ((*i)->inner_edge.size() != 0)
			return false;
		HalfEdge* l = (*i)->outer_edge;
		int num_edges=0;
		do {
			num_edges++;
			l = l->next;
		} while (l != (*i)->outer_edge);
		if (num_edges != 3)
			return false;
	}
	return true;
}

void DoublyConnectedEdgeList::ExportMCode()
{
	for (auto i = face_list.begin(); i != face_list.end(); i++) {
		cout << "{ { ";
		HalfEdge* l = (*i)->outer_edge;
		if (l != 0) {
			do {
				cout << " {" << l->start_point->p.coor[0] << "," << l->start_point->p.coor[1] << "}";
				l = l->next;
				if (l != (*i)->outer_edge)
					cout << ",";
			} while (l != (*i)->outer_edge);
		}
		cout << "}";
		for (auto j = (*i)->inner_edge.begin(); j != (*i)->inner_edge.end(); j++) {
			l = (*j);
			cout << ", {";
			do {
				cout << " {" << l->start_point->p.coor[0] << "," << l->start_point->p.coor[1] << "}";
				l = l->next;
				if (l != *j)
					cout << ",";
			} while (l != *j);
			cout << " }";
		}
		cout << " }" << endl;
	}
}

DoublyConnectedEdgeList::DoublyConnectedEdgeList()
{
	face_list.push_back(new Face);
}

DoublyConnectedEdgeList::~DoublyConnectedEdgeList()
{
	for (auto i = vertex_list.begin(); i != vertex_list.end(); i++)
		delete* i;
	for (auto i = edge_list.begin(); i != edge_list.end(); i++)
		delete* i;
	for (auto i = face_list.begin(); i != face_list.end(); i++)
		delete* i;
}

bool Face::isInside(P2D p)
{
	bool detect_res = true;
	Polygon pl;
	HalfEdge* l = outer_edge;
	if (l != 0) {
		do {
			pl.append(l->start_point->p);
			l = l->next;
		} while (l != outer_edge);
		detect_res &= pl.postion(p);
	}
	for (auto f = inner_edge.begin(); f != inner_edge.end(); f++) {
		l = (*f);
		pl.clear();
		do {
			pl.append(l->start_point->p);
			l = l->next;
		} while (l != (*f));
		detect_res &= !pl.postion(p);
	}
	return detect_res;
}

bool HalfEdge::GetSign()
{
	Polygon pl;
	HalfEdge* l = this;
	do {
		pl.append(l->start_point->p);
		l = l->next;
	} while (l != this);
	return pl.orientation();
}

bool HalfEdge::isSameBoundary(HalfEdge* obj)
{
	HalfEdge* l = this;
	do {
		if (l == obj) {
			return true;
		}
		l = l->next;
	} while (l != this);
	return false;
}
