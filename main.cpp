#include "vizscene.h"
#include <QApplication>
#include "kinematics.h"
#include <vector>


int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  std::vector<std::vector<double>> dh{{0, 0, 1, 0},
                                      {0, 0, 1, 0},
                                      {0, 0, 1, 0},
                                      {0, 0, 1, 0}};

  SerialArm arm(dh);

  VizScene viz(nullptr, arm);
  viz.show();

  return a.exec();
}
