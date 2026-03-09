#ifndef XYZPLAINTEXTEDIT_H
#define XYZPLAINTEXTEDIT_H

#include <QPlainTextEdit>
#include <QFocusEvent>

class xyzPlainTextEdit : public QPlainTextEdit {
  Q_OBJECT
 public:
  explicit xyzPlainTextEdit(QWidget* parent = nullptr);
  void focusOutEvent(QFocusEvent* event) override;

 signals:
  void editingFinished();

 public slots:
};

#endif  // XYZPLAINTEXTEDIT_H
