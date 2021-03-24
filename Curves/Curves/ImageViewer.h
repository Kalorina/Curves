#pragma once

#include <QtWidgets/QMainWindow>
#include <QtWidgets>
#include "ui_ImageViewer.h"
#include "ViewerWidget.h"
#include "NewImageDialog.h"

class ImageViewer : public QMainWindow
{
	Q_OBJECT

public:
	ImageViewer(QWidget* parent = Q_NULLPTR);

private:
	Ui::ImageViewerClass* ui;
	NewImageDialog* newImgDialog;

	QSettings settings;
	QMessageBox msgBox;

	QPoint lastMovePoint = QPoint(0, 0);
	QPoint endPoint = QPoint(0, 0);
	QColor color = QColor("black");

	bool polygoneMode = false;
	bool drawingActive = false;
	bool objectDrawn = false;
	bool movingObject = false;

	bool fillaction = false;

	bool hermiteCurveDraw = false;
	bool bezierCurveDraw = false;
	bool coonsCurveDraw = false;

	QVector<QPoint> points;

	//ViewerWidget functions
	ViewerWidget* getViewerWidget(int tabId);
	ViewerWidget* getCurrentViewerWidget();

	//Event filters
	bool eventFilter(QObject* obj, QEvent* event);

	//ViewerWidget Events
	bool ViewerWidgetEventFilter(QObject* obj, QEvent* event);
	void ViewerWidgetMouseButtonPress(ViewerWidget* w, QEvent* event);
	void ViewerWidgetMouseButtonRelease(ViewerWidget* w, QEvent* event);
	void ViewerWidgetMouseMove(ViewerWidget* w, QEvent* event);
	void ViewerWidgetLeave(ViewerWidget* w, QEvent* event);
	void ViewerWidgetEnter(ViewerWidget* w, QEvent* event);
	void ViewerWidgetWheel(ViewerWidget* w, QEvent* event);

	//ImageViewer Events
	void closeEvent(QCloseEvent* event);

	//Image functions
	void openNewTabForImg(ViewerWidget* vW);
	bool openImage(QString filename);
	bool saveImage(QString filename);
	void clearImage();
	void setBackgroundColor(QColor color);

	//Inline functions
	inline bool isImgOpened() { return ui->tabWidget->count() == 0 ? false : true; }

private slots:
	//Tabs slots
	void on_tabWidget_tabCloseRequested(int tabId);
	void on_actionRename_triggered();

	//Image slots
	void on_actionNew_triggered();
	void newImageAccepted();
	void on_actionOpen_triggered();
	void on_actionSave_as_triggered();
	void on_actionClear_triggered();
	void on_actionSet_background_color_triggered();

	void on_pushButtonDraw_clicked();
	void on_pushButtonColorPalette_clicked();

	//transformations

	void on_pushButtonPolygone_clicked();
	void on_pushButtonClear_clicked();
	void on_pushButtonRotate_clicked(); 
	void on_pushButtonScale_clicked(); 
	void on_pushButtonShear_clicked(); 
	void on_pushButtonSymetry_clicked();

	//Curves

	void on_pushButtonHermite_clicked();
	void on_pushButtonBezier_clicked();
	void on_pushButtonCoons_clicked();

};
