#pragma once
#include <QtWidgets>

class Edge {
private:
	double xz, yz, xk, yk; //zaciatocny a koncovy bod
	double x = 0, dy =0; // x na algo
	int count = 0;

public:
	Edge() { xz = 0; yz = 0; xk = 0; yk = 0; }
	Edge(double x1, double y1, double x2, double y2) { xz = x1; yz = y1; xk = x2; yk = y2;};
	Edge(QPoint a, QPoint b) { xz = a.x(); yz = a.y(); xk = b.x(); yk = b.y(); }

	void setXZ(double x) { xz = x; }
	void setYZ(double y) { yz = y; }
	void setXK(double x) { xk = x; }
	void setYK(double y) { yk = y; }
	void setX(double x1) { x = x1; }
	void setDY() { dy = yk - yz; }
	void setDY(double dy1) { dy = dy1; }
	void setCount(int c) { count = c; }

	double getXZ() const { return xz; }
	double getYZ() const { return yz; }
	double getXK() { return xk; }
	double getYK() { return yk; }
	double getX() { return x; }
	double getDY() { return dy; }
	int getCount() { return count; }

	double getM() { double m = (yk - yz) / (xk - xz); return m; }
	double getW() { double m = (yk - yz) / (xk - xz); double w = 1 / m; return w; }
	
	void printEdge() { qDebug() << "xZ=" << xz << "yZ=" << yz << "xK=" << xk << "yk" << yk << "dy=" << dy << "x=" << x << "w=" << getW(); }

	bool operator<(const Edge& edge) { 
		if (edge.count == 0) 
			{ return yz < edge.yz; } 
		if (edge.count == 1)
		{ return x < edge.x; }
	}
};

