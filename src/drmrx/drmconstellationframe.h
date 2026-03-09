#ifndef DRMCONSTELLATIONFRAME_H
#define DRMCONSTELLATIONFRAME_H

#include <QFrame>

enum econstellation { FAC, MSC };

namespace Ui {
class drmConstellationFrame;
}

class drmConstellationFrame : public QFrame {
  Q_OBJECT

 public:
  explicit drmConstellationFrame(QWidget* parent = nullptr);
  ~drmConstellationFrame() override;
  void setConstellation(econstellation constellation);
  void clearConstellation();

 private:
  Ui::drmConstellationFrame* ui;
  void paintEvent(QPaintEvent*) override;
  float ConstellationArray[2 * 2959];
  int lmsc;
};

#endif  // DRMCONSTELLATIONFRAME_H
