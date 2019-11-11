#pragma once
#include<math.h>
#include<algorithm>
#define PI 3.141592653589793116
#define EPSILON 1e-10

template<int n>
class Point
{
public:
	double coor[n];

	Point<n> operator+(Point<n> rhs)
	{
		Point<n> res;
		for (int i = 0; i < n; i++) {
			res.coor[i] = coor[i] + rhs.coor[i];
		}
		return res;
	}
	Point<n> operator-(Point<n> rhs)
	{
		Point<n> res;
		for (int i = 0; i < n; i++) {
			res.coor[i] = coor[i] - rhs.coor[i];
		}
		return res;
	}
	double operator*(Point<n> rhs) {
		double res = 0;
		for (int i = 0; i < n; i++) {
			res += coor[i] * rhs.coor[i];
		}
		return res;
	}

	friend Point<n> operator*(double t, Point<n> p) {
		Point<n> res;
		for (int i = 0; i < n; i++) {
			res.coor[i] = t * p.coor[i];
		}
		return res;
	}

	bool operator==(const Point<n>& rhs)const {
		for (int i = 0; i < n; i++) {
			if (abs(coor[i] - rhs.coor[i]) > EPSILON)
				return false;
		}
		return true;
	}

	Point()
	{
		for (int i = 0; i < n; i++) {
			coor[i] = 0;
		}
	}

};


class P2D :public Point<2> {
public:
	static const P2D O;

	double cross(P2D rhs) {
		return coor[0] * rhs.coor[1] - coor[1] * rhs.coor[0];
	}

	double Angle(P2D p) {
		double a1 = atan2(coor[1], coor[0]);
		double a2 = atan2(p.coor[1], p.coor[0]);
		double res = a2 - a1;
		while (!(res < 2 * PI && res >= 0))
			res += res < 0 ? 2 * PI : -2 * PI;
		return res;
	};

	bool operator==(const P2D& rhs)const {
		return (const Point<2>)(*this) == (const Point<2>)rhs;
	}

	P2D() {

	}

	P2D(double x, double y) {
		coor[0] = x;
		coor[1] = y;
	}

	P2D(const Point<2>& obj) :Point<2>(obj) {
	}
};

class P3D :public Point<3>
{
public:
	P3D cross(P3D rhs) {
		return P3D(
			coor[1] * rhs.coor[2] - coor[2] * rhs.coor[1],
			coor[2] * rhs.coor[0] - coor[0] * rhs.coor[2],
			coor[0] * rhs.coor[1] - coor[1] * rhs.coor[0]);
	}

	friend double det(P3D p1, P3D p2, P3D p3) {
		return  p1 * (p2.cross(p3));
	}

	P3D(double x, double y, double z) {
		coor[0] = x;
		coor[1] = y;
		coor[2] = z;
	}

	P3D(const Point<3>& obj) :Point<3>(obj) {

	}
};

class comparator {
public:
	bool operator()(const P2D& p1, const P2D& p2)const {
		return (p1.coor[1] < p2.coor[1] || p1.coor[1] == p2.coor[1] && p1.coor[0] < p2.coor[0]) && !(p1 == p2);
	}
};

template<int n>
class Simplex {
public:
	int _array[n];

	int& operator[](int index) {
		return _array[index];
	}
};
typedef Simplex<2> Segment;