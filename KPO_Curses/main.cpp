#include <QtCore/QCoreApplication>
#include <QSocketNotifier>
#include <QBasicTimer>
#include <QObject>

#include "kpoAppCurses.h"
#include <ncurses.h>
#include <csignal>
#include <stdio.h>
#include <signal.h>
#include <linux/cdk.h>


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



#include <streambuf>

class buffer
    : public std::streambuf
{
    std::ostream&   os;
    std::streambuf* buf;

public:
    buffer(std::ostream& os) : os(os), buf(os.rdbuf())
    { }

    ~buffer()
    {
        os.rdbuf(buf);
    }
};


kpoAppCurses *v;


struct CleanExit{
    CleanExit() {

        signal(SIGINT, &CleanExit::exitQt);
        signal(SIGKILL, &CleanExit::exitQt);
        signal(SIGSTOP, &CleanExit::exitQt);
        signal(SIGTERM, &CleanExit::exitQt);
//        signal(SIGSEGV, &CleanExit::exitQt);

#ifdef SIGBREAK
    signal(SIGBREAK, &CleanExit::exitQt);
#endif
#ifdef SIGHUP
    signal(SIGHUP, &CleanExit::exitQt);
#endif
#ifdef SIGQUIT
    signal(SIGQUIT, &CleanExit::exitQt);
#endif
    }

    static void exitQt(int sig) {
        v->saveSettings();
        QCoreApplication::exit(0);
    }
};


int main(int argc, char *argv[])
{
    buffer b(std::cout);
    std::ofstream file("file.txt");

    std::cout.rdbuf(file.rdbuf());

    CleanExit cleanExit;
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

    v = new kpoAppCurses(grabber);

    int rv = app.exec ();

    delete v;

    endwin();
    return (rv);
}

#include "main.moc"


