#include <QtCore/QCoreApplication>
#include <QSocketNotifier>
#include <QBasicTimer>
#include <QObject>

#include "kpoAppCurses.h"
#include <ncurses.h>


class Worker : public QObject
{
    Q_OBJECT
    QBasicTimer m_timer;
    Q_SLOT void readyRead() {
        // It's OK to call this with no data available to be read.
        int c;
        while ((c = getch()) != ERR) {
            printw("%c", (char)(c <= 255 ? c : '?'));
        }
    }
    void timerEvent(QTimerEvent * ev) {
        if (ev->timerId() != m_timer.timerId()) return;
        printw("*");
        refresh();
    }
public:
    Worker(QObject * parent = 0) : QObject(parent) {
        connect(new QSocketNotifier(0, QSocketNotifier::Read, this),
                SIGNAL(activated(int)), SLOT(readyRead()));
        readyRead(); // data might be already available without notification
        m_timer.start(1000, this);
    }
};




int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    
    Worker w;
    WINDOW * win = initscr();
    clear();
    cbreak(); // all input is available immediately
    noecho(); // no echo
    keypad(win, true); // special keys are interpreted and returned as single int from getch()
    nodelay(win, true); // getch() is a non-blocking call


    // Open the first available camera
    pcl::OpenNIGrabber grabber ("#1");
    // Check if an RGB stream is provided
    if (!grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud> ())
    {
        PCL_ERROR ("Device #1 does not provide an RGB stream!\n");
        return (-1);
    }

    kpoAppCurses v (grabber);

    int rv = app.exec ();

    endwin();
    return (rv);
}

#include "main.moc"


