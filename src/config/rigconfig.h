#ifndef RIGCONFIG_H
#define RIGCONFIG_H

#include "baseconfig.h"

class rigControl;
struct scatParams;


namespace Ui {
class rigConfig;
}

class rigConfig : public baseConfig {
  Q_OBJECT

 public:
  explicit rigConfig(QWidget* parent = nullptr);
  ~rigConfig() override;
  void attachRigController(rigControl* rigCtrl);
  void readSettings() override;
  void writeSettings() override;
  void getParams() override;
  void setParams() override;

 public slots:
  void slotEnableCAT();
  void slotEnablePTT();
  void slotEnableXMLRPC();
  void slotEnableHamlibNet();
  void slotRestart();
  void slotCheckPTT0();
  void slotCheckPTT1();
  void slotCheckPTT2();
  void slotCheckPTT3();

 private:
  Ui::rigConfig* ui;
  scatParams* cp;
  rigControl* rigController;
  void checkPTT(int p, bool b);
};

#endif  // RIGCONFIG_H
