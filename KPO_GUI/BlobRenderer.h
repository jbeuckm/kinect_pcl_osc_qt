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

    BlobRenderer(QWidget *parent = 0);

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

    void updateBackgroundImage(QImage image);

    void resetPaths();
    void addPath(std::vector<cv::Point>);
    QVector<QPainterPath> paths;

public slots:
    void setPen(const QPen &pen);
    void setBrush(const QBrush &brush);
    void setAntialiased(bool antialiased);

protected:
    void paintEvent(QPaintEvent *event);

    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);

signals:
    void contourSelected(QPainterPath contour);

private:
    QPen pen;
    QBrush brush;
    QBrush hilightBrush;
    bool antialiased;

    QPixmap backgroundPixmap;

    float scaleX;
    float scaleY;

    QPoint m_lastPoint;
    bool m_mouseClick;

    QPoint mousePos;
};
    


#endif // BLOBRENDERER_H
