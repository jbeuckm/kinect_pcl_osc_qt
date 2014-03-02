#include "BlobRenderer.h"
#include <QVector>
#include <QMouseEvent>

BlobRenderer::BlobRenderer(QWidget *parent)
    : QWidget(parent)
    , polygons()
    , brush(QColor(0,0,255,50))
    , pen(QColor(0,255,0))
{
    antialiased = false;

    setBackgroundRole(QPalette::Base);
    setAutoFillBackground(true);
}

void BlobRenderer::updateBackgroundImage(QImage image)
{
    backgroundPixmap = QPixmap::fromImage(image);

    scaleX = (float)this->width()/(float)image.width();
    scaleY = (float)this->height()/(float)image.height();
}

void BlobRenderer::resetPolygons()
{
    polygons.resize(0);
}

void BlobRenderer::addContour(std::vector<cv::Point> contour)
{
    QVector<QPoint> points;

    for (int j=0; j<contour.size(); j++) {
        cv::Point p = contour.at(j);
        QPoint qp(p.x, p.y);
        points.append(qp);
    }

    QPolygon poly(points);

    polygons.append(poly);
}

void BlobRenderer::paintEvent(QPaintEvent * /* event */)
{
    QPainter painter(this);

    painter.scale(scaleX, scaleY);

    painter.setPen(pen);
    painter.setBrush(brush);

    if (antialiased)
        painter.setRenderHint(QPainter::Antialiasing, true);

    painter.drawPixmap(0, 0, backgroundPixmap);

    for (int i=0; i<polygons.size(); i++) {

        painter.drawPolygon(polygons[i]);

    }

}

void BlobRenderer::mousePressEvent ( QMouseEvent * e )
{
    // store click position
    m_lastPoint = e->pos();
    // set the flag meaning "click begin"
    m_mouseClick = true;
}
void BlobRenderer::mouseReleaseEvent ( QMouseEvent * e )
{

    std::vector<cv::Point> contour;

    // check if cursor not moved since click beginning
    if ((m_mouseClick) && (e->pos() == m_lastPoint))
    {
        // do something: for example emit Click signal
        emit contourSelected(contour);
    }
}

QSize BlobRenderer::sizeHint() const
{
    return QSize(400, 200);
}

QSize BlobRenderer::minimumSizeHint() const
{
    return QSize(100, 100);
}


void BlobRenderer::setPen(const QPen &pen)
{
    this->pen = pen;
    update();
}

void BlobRenderer::setBrush(const QBrush &brush)
{
    this->brush = brush;
    update();
}
void BlobRenderer::setAntialiased(bool antialiased)
{
    this->antialiased = antialiased;
    update();
}
