#ifndef OPERATORCONFIG_H
#define OPERATORCONFIG_H
#include "baseconfig.h"

extern QString myCallsign;
extern QString myQth;
extern QString myLocator;
extern QString myLastname;
extern QString myFirstname;
extern QString lastReceivedCall;

extern bool onlineStatusEnabled;
extern QString onlineStatusText;


namespace Ui {
class operatorConfig;
}

class operatorConfig : public baseConfig {
  Q_OBJECT

 public:
  explicit operatorConfig(QWidget* parent = nullptr);
  ~operatorConfig() override;
  void readSettings() override;
  void writeSettings() override;
  void getParams() override;
  void setParams() override;

 private:
  Ui::operatorConfig* ui;
};

#endif  // OPERATORCONFIG_H
