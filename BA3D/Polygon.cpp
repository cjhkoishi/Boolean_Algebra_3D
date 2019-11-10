#include "Polygon.h"
#include <vector>
using namespace std;

void Polygon::append(P2D p)
{
	Node* nx, * pr, * new_p;
	if (head == 0) {
		head = new_p = new Node(p);
		nx = head;
		pr = head;
	}
	else {
		new_p = new Node(p);
		nx = head;
		pr = head->prev;
	}
	new_p->next = nx;
	new_p->prev = pr;
	nx->prev = new_p;
	pr->next = new_p;
}

bool Polygon::orientation()
{
	Node* i = head;
	Node* m = head;
	vector<Node*> ms;
	ms.push_back(i);
	do {
		if (m->p.coor[0] == i->p.coor[0] && m->p.coor[1] == i->p.coor[1])
		{
			ms.push_back(i);
		}		
		else if (m->p.coor[0] < i->p.coor[0] || m->p.coor[0] == i->p.coor[0] && m->p.coor[1] < i->p.coor[1]) {
			m = i;
			ms.clear();
			ms.push_back(m);
		}
		i = i->next;
	} while (i != head);
	bool res=true;
	for (int j = 0; j < ms.size(); j++) {
		P2D p1 = ms[j]->p - ms[j]->prev->p;
		P2D p2 = ms[j]->next->p - ms[j]->p;
		double c = p1.cross(p2);
		if ( c<= 0) {
			res = false;
			break;
		}
	}
	return res;
}

bool Polygon::postion(P2D p)
{
	Node* i = head;
	int cn = 0;
	do {
		P2D v1 = i->p - p;
		P2D v2 = i->next->p - p;
		P2D w = v2 - v1;
		if (v1.coor[1] * v2.coor[1] > 0) {
			i = i->next;
			continue;
		}
		if (w.coor[1] != 0)
		{
			bool dir = w.coor[1] > 0;
			double s = -v1.cross(w);
			double t = -v1.coor[1];
			bool uptest = dir && s > 0 && t >= 0 && t < w.coor[1];
			bool downtest = !dir && s < 0 && t < 0 && t >= w.coor[1];
			if (uptest || downtest)
				cn++;
		}
		i = i->next;
	} while (i != head);
	return abs(cn) % 2;
}

void Polygon::clear()
{
	Node* i = head;
	if (i == 0)
		return;
	do {
		Node* j = i->next;
		delete i;
		i = j;
	} while (i != head);
	head = 0;
}

Polygon::Polygon(const Polygon& obj)
{
	clear();
	if (obj.head != 0) {
		head = new Node(obj.head->p);
		Node* i = obj.head->next;
		Node* k = head;
		while (obj.head != i) {
			Node* j = new Node(i->p);
			k->next = j;
			j->prev = k;
			k = j;
			i = i->next;
		}
		k->next = head;
		head->prev = k;
	}
}

Polygon::~Polygon()
{
	clear();
}
