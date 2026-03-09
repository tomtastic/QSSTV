#ifndef DRMSEGMENTSVIEW_H
#define DRMSEGMENTSVIEW_H

#include <QLabel>


class drmSegmentsView : public QLabel {
  Q_OBJECT

 public:
  explicit drmSegmentsView(QWidget* parent = nullptr);
  ~drmSegmentsView() override;
  void setColorFail(QColor color);
  void setColorOK(QColor color);
  void setMaxBlocks(int mb) {
    if (mb == 0)
      maxBlocks = 1;
    else
      maxBlocks = mb;
  }
  void setBlocks(QList<short unsigned int> blkList);

 protected:
  void paintEvent(QPaintEvent* e) override;

 private:
  QColor colFail;
  QColor colOK;
  QList<short unsigned int> blockList;
  short int maxBlocks;
  int blockListCount;
};

#endif  // DRMSEGMENTSVIEW_H
