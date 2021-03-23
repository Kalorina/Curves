#pragma once
#include <QtWidgets>
#include "edgeClass.h"

class ViewerWidget :public QWidget {
	Q_OBJECT
private:
	QString name = "";
	QSize areaSize = QSize(0, 0);
	QImage* img = nullptr;
	QRgb* data = nullptr;
	QPainter* painter = nullptr;


public:
	ViewerWidget(QString viewerName, QSize imgSize, QWidget* parent = Q_NULLPTR);
	~ViewerWidget();
	void resizeWidget(QSize size);

	//Image functions
	bool setImage(const QImage& inputImg);
	QImage* getImage() { return img; };
	bool isEmpty();

	//Data functions
	QRgb* getData() { return data; }
	void setPixel(int x, int y, const QColor& color);
	void setPixel(int x, int y, unsigned char r, unsigned char g, unsigned char b);
	bool isInside(int x, int y) { return (x >= 0 && y >= 0 && x < img->width() && y < img->height()) ? true : false; }

	//Get/Set functions
	QString getName() { return name; }
	void setName(QString newName) { name = newName; }

	void setPainter() { painter = new QPainter(img); }
	void setDataPtr() { data = reinterpret_cast<QRgb*>(img->bits()); }

	int getImgWidth() { return img->width(); };
	int getImgHeight() { return img->height(); };

	void drawLineDDA(QPoint startPoint, QPoint endPoint, QColor color);
	void drawCircleDDA(QPoint originPoint, QPoint radiusPoint, QColor color);

	void drawLineBresen(QPoint startPoint, QPoint endPoint, QColor color);
	void drawCircleBresen(QPoint originPoint, QPoint radiusPoint, QColor color);

	void draw(QVector<QPoint> points, QColor color, QString algorithm, QString interpolation, bool fillOn);

	//orezavanie 
	QVector<QPoint> cyrusBeck(QPoint a, QPoint b); //usecka
	QVector<QPoint> sutherlandHodgman(QVector<QPoint> points, int minX); //polygone

	//Vyplnanie farieb
	void scanLine(QVector<QPoint> points, QColor color);
	QVector<Edge> redirectEdgesByY(QVector<Edge> edges);
	QVector<QPoint> sortByYThenByX(QVector<QPoint> points);

	void scanLineTriangle(QVector<QPoint> points, QColor color, QString interpolation);
	QColor nearestNeighbor(QVector<QPoint> points, QPoint p, QColor color, QColor c1, QColor c2, QColor c3);
	QColor Barycentric(QVector<QPoint> points, QPoint p, QColor color, QColor c1, QColor c2, QColor c3);
	QColor interpolationPixel(QString interpolation, QVector<QPoint> points, QPoint p, QColor color, QColor c1, QColor c2, QColor c3);

	//Curves

	void drawPoints(QVector<QPoint> points);
	void hermiteCurve(QVector<QPoint> points, int numberOfpoints, double degree);
	QVector<QPoint> findTangent(QVector<QPoint> points);

	void bezierCurve(QVector<QPoint> points);

	void coonsCurve(QVector<QPoint> points);

	void clear(QColor color = Qt::white);

public slots:
	void paintEvent(QPaintEvent* event) Q_DECL_OVERRIDE;
};