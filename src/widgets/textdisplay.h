#ifndef TEXTDISPLAY_H
#define TEXTDISPLAY_H

#include <QDialog>

namespace Ui {
class textDisplay;
}

class textDisplay : public QDialog {
  Q_OBJECT

 public:
  explicit textDisplay(QWidget* parent = nullptr);
  ~textDisplay();
  void clear();
  void append(const QString& t);

 private:
  Ui::textDisplay* ui;
};

#endif  // TEXTDISPLAY_H
