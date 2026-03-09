#ifndef GRAPHICITEMS_H
#define GRAPHICITEMS_H


#include "basegraphicitem.h"


class itemRectangle : public graphItemBase {
 public:
  itemRectangle(QMenu* cntxtMenu);
  void drawItem(QPainter* painter) override;
  int type() const override { return RECTANGLE; }
};

class itemEllipse : public graphItemBase {
 public:
  itemEllipse(QMenu* cntxtMenu);
  void drawItem(QPainter* painter) override;
  int type() const override { return ELLIPSE; }
};


class itemLine : public graphItemBase {
 public:
  itemLine(QMenu* cntxtMenu);
  void drawItem(QPainter* painter) override;
  int type() const override { return LINE; }
  //  QPainterPath shape() const;
};

class itemImage : public graphItemBase {
 public:
  itemImage(QMenu* cntxtMenu);
  void drawItem(QPainter* painter) override;
  int type() const override { return IMAGE; }
};

class itemText : public graphItemBase {
 public:
  itemText(QMenu* cntxtMenu);
  void drawItem(QPainter* painter) override;
  int type() const override { return TEXT; }
  QPainterPath shape() const override;
  void setText(const QString& t) override;
  void setFont(QFont f) override;
  QRectF boundingRct;
};

class itemReplayImage : public graphItemBase {
 public:
  itemReplayImage(QMenu* cntxtMenu);
  void drawItem(QPainter* painter) override;
  int type() const override { return REPLAY; }
};

class itemBorder : public graphItemBase {
 public:
  itemBorder(QMenu* cntxtMenu);
  void drawItem(QPainter* painter) override;
  int type() const override { return SBORDER; }
};


#endif  // GRAPHICITEMS_H
