#ifndef CWCONFIG_H
#define CWCONFIG_H

#include "baseconfig.h"

extern QString cwText;
extern int cwTone;
extern int cwWPM;
// extern bool enableCW;


namespace Ui {
class cwConfig;
}

class cwConfig : public baseConfig {
  Q_OBJECT

 public:
  explicit cwConfig(QWidget* parent = nullptr);
  ~cwConfig() override;
  void readSettings() override;
  void writeSettings() override;
  void getParams() override;
  void setParams() override;

 private:
  Ui::cwConfig* ui;
};

#endif  // CWCONFIG_H
