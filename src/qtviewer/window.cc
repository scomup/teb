#include <QtWidgets>

#include "window.h"
#include "scribblearea.h"

MainWindow::MainWindow()
{
    scribbleArea = new ScribbleArea;
    setCentralWidget(scribbleArea);
    setWindowTitle(tr("Scribble"));
    resize(500, 500);
}

