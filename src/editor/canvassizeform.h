#ifndef CANVASSIZEFORM_H
#define CANVASSIZEFORM_H

#include <QDialog>

namespace Ui {
class canvasSizeForm;
}

class canvasSizeForm : public QDialog {
  Q_OBJECT

 public:
  explicit canvasSizeForm(QWidget* parent = nullptr);
  ~canvasSizeForm() override;
  QRect getSize();
  void setSize(int x, int y);

 private:
  Ui::canvasSizeForm* ui;
};

#endif  // CANVASSIZEFORM_H
