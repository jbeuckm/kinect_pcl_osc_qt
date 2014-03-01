#include "BlobRenderer.h"
#include <QVector>


BlobRenderer::BlobRenderer(QWidget *parent)
    : QWidget(parent)
    , polygons()
{
    shape = Polygon;
    antialiased = false;
    transformed = false;
//    pixmap.load(":/images/qt-logo.png");

    setBackgroundRole(QPalette::Base);
    setAutoFillBackground(true);

}

void BlobRenderer::updateBackgroundImage(QImage image)
{
    backgroundPixmap = QPixmap::fromImage(image);

    scaleX = (float)this->width()/(float)image.width();
    scaleY = (float)this->height()/(float)image.height();
}

void BlobRenderer::addContour(std::vector<cv::Point> contour)
{
    std::cout << "addContour" << std::endl;
    QVector<QPoint> points;

    for (int j=0; j<contour.size(); j++) {
        cv::Point p = contour.at(j);
        QPoint qp(p.x, p.y);
        points.append(qp);
        std::cout << p.x << "," << p.y << std::endl;
    }

    QPolygon poly(points);
//    polygons.resize(0);
//    polygons.append(poly);
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
/*
    for (int i=0; i<contours.size(); i++) {
        std::vector<cv::Point> contour = contours[i];
        QPoint points[contour.size()];

        for (int j=0; j<contour.size(); j++) {
            cv::Point p = contour[j];
            points[j] = QPoint(p.x, p.y);
        }

        painter.drawPolyline(points, contour.size());

    }
*/


    /*
    static const QPoint points[4] = {
        QPoint(10, 80),
        QPoint(20, 10),
        QPoint(80, 30),
        QPoint(90, 70)
    };

    QRect rect(10, 20, 80, 60);

    QPainterPath path;
    path.moveTo(20, 80);
    path.lineTo(20, 30);
    path.cubicTo(80, 0, 50, 50, 80, 80);

    int startAngle = 20 * 16;
    int arcLength = 120 * 16;


    for (int x = 0; x < width(); x += 100) {
        for (int y = 0; y < height(); y += 100) {
            painter.save();
            painter.translate(x, y);

            if (transformed) {
                painter.translate(50, 50);
                painter.rotate(60.0);
                painter.scale(0.6, 0.9);
                painter.translate(-50, -50);
            }
            switch (shape) {
            case Line:
                painter.drawLine(rect.bottomLeft(), rect.topRight());
                break;
            case Points:
                painter.drawPoints(points, 4);
                break;
            case Polyline:
                painter.drawPolyline(points, 4);
                break;
            case Polygon:
                painter.drawPolygon(points, 4);
                break;
            case Rect:
                painter.drawRect(rect);
                break;
            case RoundedRect:
                painter.drawRoundedRect(rect, 25, 25, Qt::RelativeSize);
                break;
            case Ellipse:
                painter.drawEllipse(rect);
                break;
            case Arc:
                painter.drawArc(rect, startAngle, arcLength);
                break;
            case Chord:
                painter.drawChord(rect, startAngle, arcLength);
                break;
            case Pie:
                painter.drawPie(rect, startAngle, arcLength);
                break;
            case Path:
                painter.drawPath(path);
                break;
            case Text:
                painter.drawText(rect, Qt::AlignCenter, tr("Qt by\nNokia"));
                break;
            case Pixmap:
            }

            painter.restore();
        }
    }

    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.setPen(palette().dark().color());
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(QRect(0, 0, width() - 1, height() - 1));
    */
}


QSize BlobRenderer::sizeHint() const
{
    return QSize(400, 200);
}

QSize BlobRenderer::minimumSizeHint() const
{
    return QSize(100, 100);
}

void BlobRenderer::setShape(Shape shape)
{
    this->shape = shape;
    update();
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

void BlobRenderer::setTransformed(bool transformed)
{
    this->transformed = transformed;
    update();
}
