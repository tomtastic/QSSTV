#ifndef DRMPSDFRAME_H
#define DRMPSDFRAME_H

#include <QFrame>
#define PSDSPAN 512

namespace Ui {
class drmPSDFrame;
}

class drmPSDFrame : public QFrame {
  Q_OBJECT

 public:
  explicit drmPSDFrame(QWidget* parent = nullptr);
  ~drmPSDFrame() override;
  void setPSD();

 private:
  Ui::drmPSDFrame* ui;
  void paintEvent(QPaintEvent*) override;
  float psdArray[PSDSPAN];
  float psdCArray[PSDSPAN / 4];
};

#endif  // DRMPSDFRAME_H
