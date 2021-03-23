#include "ImageViewer.h"

ImageViewer::ImageViewer(QWidget* parent)
	: QMainWindow(parent), ui(new Ui::ImageViewerClass)
{
	ui->setupUi(this);

	ui->menuBar->setDisabled(true);

	int width = 500;
	int height = 500;
	QString name = "Scatch";
	openNewTabForImg(new ViewerWidget(name, QSize(width, height)));
	ui->tabWidget->setCurrentIndex(ui->tabWidget->count() - 1);

	ui->groupBoxSetUp->setDisabled(true);

	//ui->groupBoxCurves->setDisabled(true);

}

//ViewerWidget functions
ViewerWidget* ImageViewer::getViewerWidget(int tabId)
{
	QScrollArea* s = static_cast<QScrollArea*>(ui->tabWidget->widget(tabId));
	if (s) {
		ViewerWidget* vW = static_cast<ViewerWidget*>(s->widget());
		return vW;
	}
	return nullptr;
}
ViewerWidget* ImageViewer::getCurrentViewerWidget()
{
	return getViewerWidget(ui->tabWidget->currentIndex());
}

// Event filters
bool ImageViewer::eventFilter(QObject* obj, QEvent* event)
{
	if (obj->objectName() == "ViewerWidget") {
		return ViewerWidgetEventFilter(obj, event);
	}
	return false;
}

//ViewerWidget Events
bool ImageViewer::ViewerWidgetEventFilter(QObject* obj, QEvent* event)
{
	ViewerWidget* w = static_cast<ViewerWidget*>(obj);

	if (!w) {
		return false;
	}

	if (event->type() == QEvent::MouseButtonPress) {
		ViewerWidgetMouseButtonPress(w, event);
	}
	else if (event->type() == QEvent::MouseButtonRelease) {
		ViewerWidgetMouseButtonRelease(w, event);
	}
	else if (event->type() == QEvent::MouseMove) {
		ViewerWidgetMouseMove(w, event);
	}
	else if (event->type() == QEvent::Leave) {
		ViewerWidgetLeave(w, event);
	}
	else if (event->type() == QEvent::Enter) {
		ViewerWidgetEnter(w, event);
	}
	else if (event->type() == QEvent::Wheel) {
		ViewerWidgetWheel(w, event);
	}

	return QObject::eventFilter(obj, event);
}
void ImageViewer::ViewerWidgetMouseButtonPress(ViewerWidget* w, QEvent* event)
{
	QMouseEvent* e = static_cast<QMouseEvent*>(event);
	
	if (e->button() == Qt::LeftButton)
	{
		points.append(e->pos());
	}

	w->drawPoints(points);

	/*if (drawingActive)
	{
		if (e->button() == Qt::LeftButton)
		{
			points.append(e->pos());
			//w->setPixel(e->pos().x(), e->pos().y(), color);
			//qDebug() << points;

			if (!polygoneMode && points.size() == 2)
			{
				w->draw(points, color, ui->comboBoxAlg->currentText(), ui->comboBoxInterpolation->currentText(), ui->checkBoxFill->isChecked());
				drawingActive = false;
				objectDrawn = true;
			}
		}
		if (e->button() == Qt::RightButton && polygoneMode)
		{
			w->draw(points, color, ui->comboBoxAlg->currentText(), ui->comboBoxInterpolation->currentText(), ui->checkBoxFill->isChecked());
			drawingActive = false;
			objectDrawn = true;
		}
	}
	else
	{
		if (e->button() == Qt::LeftButton)
		{
			if (objectDrawn)
			{
				movingObject = true;
				lastMovePoint = e->pos();
			}
		}
	}*/
}
void ImageViewer::ViewerWidgetMouseButtonRelease(ViewerWidget* w, QEvent* event)
{
	QMouseEvent* e = static_cast<QMouseEvent*>(event);

	if (e->button() == Qt::LeftButton)
	{
		movingObject = false;
	}
}
void ImageViewer::ViewerWidgetMouseMove(ViewerWidget* w, QEvent* event)
{
	QMouseEvent* e = static_cast<QMouseEvent*>(event);
	
	QPoint vector = QPoint(0, 0);

	if (movingObject)
	{
		clearImage();
		vector.setX(e->pos().x() - lastMovePoint.x());
		vector.setY(e->pos().y() - lastMovePoint.y());

		for (int i = 0; i < points.size(); i++)
		{
			points[i] = QPoint(points[i].x() + vector.x(), points[i].y() + vector.y());
			
		}
		w->draw(points, color, ui->comboBoxAlg->currentText(), ui->comboBoxInterpolation->currentText(), ui->checkBoxFill->isChecked());
		lastMovePoint = e->pos();
	}
}
void ImageViewer::ViewerWidgetLeave(ViewerWidget* w, QEvent* event)
{
}
void ImageViewer::ViewerWidgetEnter(ViewerWidget* w, QEvent* event)
{
}
void ImageViewer::ViewerWidgetWheel(ViewerWidget* w, QEvent* event)
{
	QWheelEvent* wheelEvent = static_cast<QWheelEvent*>(event);

	double factorDown = 0.75;
	double factorUp = 1.25;

	clearImage();

	if (wheelEvent->angleDelta().y() > 0)
	{
		for (int i = 1; i < points.size(); i++)
		{
			points[i] = QPoint(factorUp * (points[i].x() - points[0].x()) + points[0].x(), factorUp * (points[i].y() - points[0].y()) + points[0].y());

		}
	}

	if (wheelEvent->angleDelta().y() < 0)
	{
		for (int i = 1; i < points.size(); i++)
		{
			points[i] = QPoint(factorDown * (points[i].x() - points[0].x()) + points[0].x(), factorDown * (points[i].y() - points[0].y()) + points[0].y());

		}
	}

	w->draw(points, color, ui->comboBoxAlg->currentText(), ui->comboBoxInterpolation->currentText(), ui->checkBoxFill->isChecked());
}

//ImageViewer Events
void ImageViewer::closeEvent(QCloseEvent* event)
{
	if (QMessageBox::Yes == QMessageBox::question(this, "Close Confirmation", "Are you sure you want to exit?", QMessageBox::Yes | QMessageBox::No))
	{
		event->accept();
	}
	else {
		event->ignore();
	}
}

//Image functions
void ImageViewer::openNewTabForImg(ViewerWidget* vW)
{
	QScrollArea* scrollArea = new QScrollArea;
	scrollArea->setWidget(vW);

	scrollArea->setBackgroundRole(QPalette::Dark);
	scrollArea->setWidgetResizable(true);
	scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
	scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

	vW->setObjectName("ViewerWidget");
	vW->installEventFilter(this);

	QString name = vW->getName();

	ui->tabWidget->addTab(scrollArea, name);
}
bool ImageViewer::openImage(QString filename)
{
	QFileInfo fi(filename);

	QString name = fi.baseName();
	openNewTabForImg(new ViewerWidget(name, QSize(0, 0)));
	ui->tabWidget->setCurrentIndex(ui->tabWidget->count() - 1);

	ViewerWidget* w = getCurrentViewerWidget();

	QImage loadedImg(filename);
	return w->setImage(loadedImg);
}
bool ImageViewer::saveImage(QString filename)
{
	QFileInfo fi(filename);
	QString extension = fi.completeSuffix();
	ViewerWidget* w = getCurrentViewerWidget();

	QImage* img = w->getImage();
	return img->save(filename, extension.toStdString().c_str());
}
void ImageViewer::clearImage()
{
	ViewerWidget* w = getCurrentViewerWidget();
	w->clear();
}
void ImageViewer::setBackgroundColor(QColor color)
{
	ViewerWidget* w = getCurrentViewerWidget();
	w->clear(color);
}

//Slots

//Tabs slots
void ImageViewer::on_tabWidget_tabCloseRequested(int tabId)
{
	ViewerWidget* vW = getViewerWidget(tabId);
	delete vW; //vW->~ViewerWidget();
	ui->tabWidget->removeTab(tabId);
}
void ImageViewer::drawPoints()
{
}
void ImageViewer::on_actionRename_triggered()
{
	if (!isImgOpened()) {
		msgBox.setText("No image is opened.");
		msgBox.setIcon(QMessageBox::Information);
		msgBox.exec();
		return;
	}
	ViewerWidget* w = getCurrentViewerWidget();
	bool ok;
	QString text = QInputDialog::getText(this, QString("Rename image"), tr("Image name:"), QLineEdit::Normal, w->getName(), &ok);
	if (ok && !text.trimmed().isEmpty())
	{
		w->setName(text);
		ui->tabWidget->setTabText(ui->tabWidget->currentIndex(), text);
	}
}

//Image slots
void ImageViewer::on_actionNew_triggered()
{
	newImgDialog = new NewImageDialog(this);
	connect(newImgDialog, SIGNAL(accepted()), this, SLOT(newImageAccepted()));
	newImgDialog->exec();
}
void ImageViewer::newImageAccepted()
{
	NewImageDialog* newImgDialog = static_cast<NewImageDialog*>(sender());

	int width = newImgDialog->getWidth();
	int height = newImgDialog->getHeight();
	QString name = newImgDialog->getName();
	openNewTabForImg(new ViewerWidget(name, QSize(width, height)));
	ui->tabWidget->setCurrentIndex(ui->tabWidget->count() - 1);

	ui->groupBoxSetUp->setEnabled(true);
}
void ImageViewer::on_actionOpen_triggered()
{
	QString folder = settings.value("folder_img_load_path", "").toString();

	QString fileFilter = "Image data (*.bmp *.gif *.jpg *.jpeg *.png *.pbm *.pgm *.ppm .*xbm .* xpm);;All files (*)";
	QString fileName = QFileDialog::getOpenFileName(this, "Load image", folder, fileFilter);
	if (fileName.isEmpty()) { return; }

	QFileInfo fi(fileName);
	settings.setValue("folder_img_load_path", fi.absoluteDir().absolutePath());

	if (!openImage(fileName)) {
		msgBox.setText("Unable to open image.");
		msgBox.setIcon(QMessageBox::Warning);
		msgBox.exec();
	}
}
void ImageViewer::on_actionSave_as_triggered()
{
	if (!isImgOpened()) {
		msgBox.setText("No image to save.");
		msgBox.setIcon(QMessageBox::Information);
		msgBox.exec();
		return;
	}
	QString folder = settings.value("folder_img_save_path", "").toString();

	ViewerWidget* w = getCurrentViewerWidget();

	QString fileFilter = "Image data (*.bmp *.gif *.jpg *.jpeg *.png *.pbm *.pgm *.ppm .*xbm .* xpm);;All files (*)";
	QString fileName = QFileDialog::getSaveFileName(this, "Save image", folder + "/" + w->getName(), fileFilter);
	if (fileName.isEmpty()) { return; }

	QFileInfo fi(fileName);
	settings.setValue("folder_img_save_path", fi.absoluteDir().absolutePath());

	if (!saveImage(fileName)) {
		msgBox.setText("Unable to save image.");
		msgBox.setIcon(QMessageBox::Warning);
		msgBox.exec();
	}
	else {
		msgBox.setText(QString("File %1 saved.").arg(fileName));
		msgBox.setIcon(QMessageBox::Information);
		msgBox.exec();
	}
}
void ImageViewer::on_actionClear_triggered()
{
	if (!isImgOpened()) {
		msgBox.setText("No image is opened.");
		msgBox.setIcon(QMessageBox::Information);
		msgBox.exec();
		return;
	}
	clearImage();
}
void ImageViewer::on_actionSet_background_color_triggered()
{
	QColor backgroundColor = QColorDialog::getColor(Qt::white, this, "Select color of background");
	if (backgroundColor.isValid()) {
		setBackgroundColor(backgroundColor);
	}
}


void ImageViewer::on_pushButtonColorPalette_clicked()
{
	QColor color1 = QColorDialog::getColor(Qt::white, this, "Color Palette");
	
	if (color1.isValid())
	{
		color = color1;
	}
}
void ImageViewer::on_pushButtonDraw_clicked()
{
	drawingActive = true;
	//ui->groupBoxTransformations->setEnabled(true);
	ui->pushButtonDraw->setDisabled(true);
	ui->groupBoxCurves->setEnabled(true);

}

//Transformations

void ImageViewer::on_pushButtonPolygone_clicked()
{
	polygoneMode = true;
	ui->pushButtonPolygone->setDisabled(true);
}
void ImageViewer::on_pushButtonClear_clicked()
{
	clearImage();

	//polygoneMode = false;
	//drawingActive = false;
	//objectDrawn = false;
	//ui->pushButtonPolygone->setEnabled(true);
	//ui->groupBoxTransformations->setDisabled(true);
	//ui->pushButtonDraw->setEnabled(true);

	ui->pushButtonHermite->setEnabled(true);
	ui->pushButtonBezier->setEnabled(true);
	ui->pushButtonCoons->setEnabled(true);

	bezierCurve = false;
	hermiteCurve = false;
	coonsCurve = false;

	points.clear();
}
void ImageViewer::on_pushButtonRotate_clicked()
{
	ViewerWidget* w = getCurrentViewerWidget();
	double degree = ui->spinBoxRotate->value();
	QPoint zero = QPoint(points[0].x(), points[0].y());
	int a, b;

	clearImage();

	if (degree >= 0)
	{
		for (int i = 1; i < points.size(); i++)
		{
			a = points[i].x() - zero.x();
			b = points[i].y() - zero.y();
			points[i].setX(a * qCos(-degree * M_PI / 180) - b * qSin(-degree * M_PI / 180) + zero.x());
			points[i].setY(a * qSin(-degree * M_PI / 180) + b * qCos(-degree * M_PI / 180) + zero.y());
		}
	}
	if (degree < 0)
	{
		for (int i = 1; i < points.size(); i++)
		{
			a = points[i].x() - zero.x();
			b = points[i].y() - zero.y();
			points[i].setX(a * qCos(degree * M_PI / 180) + b * qSin(degree * M_PI / 180) + zero.x());
			points[i].setY(-a * qSin(degree * M_PI / 180) + b * qCos(degree * M_PI / 180) + zero.y());
		}
	}

	//qDebug() << points;

	w->draw(points, color, ui->comboBoxAlg->currentText(), ui->comboBoxInterpolation->currentText(), ui->checkBoxFill->isChecked());
}
void ImageViewer::on_pushButtonScale_clicked()
{
	ViewerWidget* w = getCurrentViewerWidget();

	double factorX = ui->spinBoxScale_1->value();
	double factorY = ui->spinBoxScale_2->value();

	clearImage();

	if (factorX != 0 && factorY != 0)
	{
		for (int i = 1; i < points.size(); i++)
		{
			points[i] = QPoint(factorX * (points[i].x() - points[0].x()) + points[0].x(), factorY * (points[i].y() - points[0].y()) + points[0].y());

		}
	}
	
	w->draw(points, color, ui->comboBoxAlg->currentText(), ui->comboBoxInterpolation->currentText(), ui->checkBoxFill->isChecked());
}
void ImageViewer::on_pushButtonShear_clicked()
{
	ViewerWidget* w = getCurrentViewerWidget();

	double factorX = ui->spinBoxShear->value();

	clearImage();

	for (int i = 0; i < points.size()-1; i++)
	{
		points[i] = QPoint(points[i].x() + factorX * points[i].y(), points[i].y());
	}

	w->draw(points, color, ui->comboBoxAlg->currentText(), ui->comboBoxInterpolation->currentText(), ui->checkBoxFill->isChecked());
}
void ImageViewer::on_pushButtonSymetry_clicked()
{
	ViewerWidget* w = getCurrentViewerWidget();

	QPoint vector = QPoint(points[1].x() - points[0].x(), points[1].y() - points[0].y());
	QPoint normal = QPoint(vector.y(), -vector.x());

	int a = normal.x(), b = normal.y();
	int c = -a * points[0].x() - b * points[0].y();
	double d = 0;

	clearImage();

	for (int i = 0; i < points.size(); i++)
	{
		d = (a * points[i].x() + b * points[i].y() + c) / (pow(a, 2) + pow(b, 2));
		points[i] = QPoint(points[i].x() - 2 * a * d, points[i].y() - 2 * b * d);
	}

	w->draw(points, color, ui->comboBoxAlg->currentText(), ui->comboBoxInterpolation->currentText(), ui->checkBoxFill->isChecked());
}

//Curves

void ImageViewer::on_pushButtonHermite_clicked()
{
	ViewerWidget* w = getCurrentViewerWidget();

	ui->pushButtonHermite->setDisabled(true);
	ui->pushButtonBezier->setDisabled(true);
	ui->pushButtonCoons->setDisabled(true);

	//w->hermiteCurve(points);
}

void ImageViewer::on_pushButtonBezier_clicked()
{
	ViewerWidget* w = getCurrentViewerWidget();

	ui->pushButtonHermite->setDisabled(true);
	ui->pushButtonBezier->setDisabled(true);
	ui->pushButtonCoons->setDisabled(true);

	w->bezierCurve(points);
}

void ImageViewer::on_pushButtonCoons_clicked()
{
	ViewerWidget* w = getCurrentViewerWidget();

	ui->pushButtonHermite->setDisabled(true);
	ui->pushButtonBezier->setDisabled(true);
	ui->pushButtonCoons->setDisabled(true);

	w->coonsCurve(points);
}

