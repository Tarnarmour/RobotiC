#include "vizscene.h"
#include <QApplication>


int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  VizScene viz;
  viz.show();

  return a.exec();
}
