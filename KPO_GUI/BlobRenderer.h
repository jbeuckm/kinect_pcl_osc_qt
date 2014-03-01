#ifndef BLOBRENDERER_H
#define BLOBRENDERER_H

#include <QWidget>
#include <QPainter>
#include <QPen>
#include <QImage>

#include <opencv/cv.h>

class BlobRenderer : public QWidget
{
    Q_OBJECT

public:
    enum Shape { Line, Points, Polyline, Polygon, Rect, RoundedRect, Ellipse, Arc,
                 Chord, Pie, Path, Text, Pixmap };

    BlobRenderer(QWidget *parent = 0);

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

    void updateBackgroundImage(QImage image);

    void resetPolygons();
    void addContour(std::vector<cv::Point>);
    QVector<QPolygon> polygons;

public slots:
    void setShape(Shape shape);
    void setPen(const QPen &pen);
    void setBrush(const QBrush &brush);
    void setAntialiased(bool antialiased);
    void setTransformed(bool transformed);

protected:
    void paintEvent(QPaintEvent *event);

private:
    Shape shape;
    QPen pen;
    QBrush brush;
    bool antialiased;
    bool transformed;
    QPixmap backgroundPixmap;


    float scaleX;
    float scaleY;
};
    


#endif // BLOBRENDERER_H
