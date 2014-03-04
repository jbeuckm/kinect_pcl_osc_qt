#include "BlobRenderer.h"
#include <QVector>
#include <QMouseEvent>

BlobRenderer::BlobRenderer(QWidget *parent)
    : QWidget(parent)
    , paths()
    , brush(QColor(0,0,255,50))
    , hilightBrush(QColor(0,0,255,255))
    , pen(QColor(0,255,0))
{
    antialiased = false;

    setMouseTracking(true);

    setBackgroundRole(QPalette::Base);
    setAutoFillBackground(true);
}

void BlobRenderer::updateBackgroundImage(QImage image)
{
    backgroundPixmap = QPixmap::fromImage(image);

    scaleX = (float)this->width()/(float)image.width();
    scaleY = (float)this->height()/(float)image.height();
}

void BlobRenderer::resetPaths()
{
    paths.resize(0);
}

void BlobRenderer::addPath(std::vector<cv::Point> contour)
{
    QPainterPath path(QPoint(contour.at(0).x, contour.at(0).y));

    for (int j=0; j<contour.size(); j++) {
        cv::Point p = contour.at(j);
        QPoint qp(p.x, p.y);

        path.lineTo(qp);
    }

    path.closeSubpath();
    paths.append(path);
}

void BlobRenderer::paintEvent(QPaintEvent *e /* event */)
{
    QPainter painter(this);

    painter.scale(scaleX, scaleY);

    painter.setPen(pen);

    if (antialiased) {
        painter.setRenderHint(QPainter::Antialiasing, true);
    }

    painter.drawPixmap(0, 0, backgroundPixmap);


    for (int i=0; i<paths.size(); i++) {

        QPainterPath poly = paths.at(i);

        if (poly.contains(mousePos)) {
            painter.setBrush(hilightBrush);
        }
        else {
            painter.setBrush(brush);
        }
        painter.drawPath(poly);

    }

}

void BlobRenderer::mousePressEvent ( QMouseEvent * e )
{
    m_lastPoint = e->pos();
    m_mouseClick = true;
}
void BlobRenderer::mouseReleaseEvent ( QMouseEvent * e )
{
    std::cout << (float)(e->x()) << std::endl;

    // check if cursor not moved since click beginning
    if ((m_mouseClick) && (e->pos() == m_lastPoint))
    {
        for (int i=0; i<paths.size(); i++) {

            QPainterPath poly = paths.at(i);

            if (poly.contains(mousePos)) {
                emit contourSelected(poly);
                break;
            }
        }
    }
}
void BlobRenderer::mouseMoveEvent(QMouseEvent *e)
{
    mousePos.setX(e->x() / scaleX);
    mousePos.setY(e->y() / scaleY);
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
