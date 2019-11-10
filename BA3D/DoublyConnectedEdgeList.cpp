#include "DoublyConnectedEdgeList.h"
#include"Polygon.h"
#include<set>
#include <iostream>

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
	vector<HalfEdge*> comparator;
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
		GetEdgesByVertex(v1, comparator);
		base = p2 - p1;
		sort(comparator.begin(), comparator.end(), preb);
		in1 = comparator[0]->twin;
		out1 = comparator[comparator.size() - 1];
	}
	if (v2->inc_edge == 0) {
		in2 = e1;
		out2 = e2;
	}
	else {
		GetEdgesByVertex(v2, comparator);
		base = p1 - p2;
		sort(comparator.begin(), comparator.end(), preb);
		in2 = comparator[0]->twin;
		out2 = comparator[comparator.size() - 1];
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
