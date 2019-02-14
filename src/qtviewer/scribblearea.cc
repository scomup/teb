
#include <QtWidgets>
#include <iostream>

#include "scribblearea.h"

ScribbleArea::ScribbleArea(QWidget *parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_StaticContents);
    scribbling = false;
}


void ScribbleArea::clearImage()
{
    image_.fill(qRgb(255, 255, 255));
    update();
}

void ScribbleArea::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        clearImage();
        drawAllPoints();
        lastPoint = event->pos();
        scribbling = true;
    }
    if (event->button() == Qt::RightButton )
    {
        points_.push_back(event->pos());
        drawAllPoints();
    }

}


void ScribbleArea::mouseMoveEvent(QMouseEvent *event)
{
    if ((event->buttons() & Qt::LeftButton) && scribbling)
        drawLineTo(event->pos());
}

void ScribbleArea::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && scribbling) {
        drawLineTo(event->pos());
        scribbling = false;
    }
}

void ScribbleArea::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    QRect dirtyRect = event->rect();
    painter.drawImage(dirtyRect, image_, dirtyRect);
}

void ScribbleArea::resizeEvent(QResizeEvent *event)
{
    if (width() > image_.width() || height() > image_.height()) {
        int newWidth = qMax(width() + 128, image_.width());
        int newHeight = qMax(height() + 128, image_.height());
        resizeImage(&image_, QSize(newWidth, newHeight));
        update();
    }
    QWidget::resizeEvent(event);
}

void ScribbleArea::drawLineTo(const QPoint &endPoint)
{
    QPainter painter(&image_);
    painter.setPen(QPen(Qt::red, 1, Qt::SolidLine, Qt::RoundCap,
                        Qt::RoundJoin));
    painter.drawLine(lastPoint, endPoint);

    int rad = 3;
    update(QRect(lastPoint, endPoint).normalized()
                                     .adjusted(-rad, -rad, +rad, +rad));
    lastPoint = endPoint;
}

void ScribbleArea::drawAllPoints()
{
    QPainter painter(&image_);
    painter.setPen(QPen(Qt::blue, 1, Qt::SolidLine, Qt::RoundCap,
                        Qt::RoundJoin));
    for(QPoint& p : points_){
        painter.drawEllipse(p, 5, 5);
    }
    update();
}

void ScribbleArea::resizeImage(QImage *image_, const QSize &newSize)
{
    if (image_->size() == newSize)
        return;

    QImage newImage(newSize, QImage::Format_RGB32);
    newImage.fill(qRgb(255, 255, 255));
    QPainter painter(&newImage);
    painter.drawImage(QPoint(0, 0), *image_);
    *image_ = newImage;
}
