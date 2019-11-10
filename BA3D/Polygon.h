#pragma once
#include"Point.h"
#include<math.h>
class Polygon
{
public:
	struct Node {
		P2D p;
		Node* next;
		Node* prev;
		Node(P2D p):p(p),next(0),prev(0) {
		};
	};

	Node* head=0;

	void append(P2D p);
	bool orientation();
	bool postion(P2D p);
	void clear();

	Polygon() {};
	Polygon(const Polygon& obj);
	~Polygon();
};

