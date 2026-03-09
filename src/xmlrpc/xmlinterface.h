#ifndef XMLINTERFACE_H
#define XMLINTERFACE_H

#include <QObject>
#include "maiaXmlRpcServer.h"
#include "maiaXmlRpcServerConnection.h"

struct sxmlInfo {
  sxmlInfo() { frequency = -1.; }
  QString rigName;
  QString bandWidth;
  double frequency;
  QString mode;
  QString trxState;
  int notch;
};

class xmlInterface : public QObject {
  Q_OBJECT
 public:
  explicit xmlInterface(QObject* parent = 0);
  void activatePTT(bool b);


 public slots:
  void takeControl();
  void setName(const QString& t);
  void setModes(QVariantList t);
  void setBandwidths(QVariantList t);
  void setBandwidth(const QString& t);
  void setWfSideband(const QString& t);
  void setMode(const QString& t);
  void setFrequency(double d);
  QVariantList systemMulticall(QVariantList s);
  double getFrequency();
  QString getMode();
  QString getBandwidth();
  QString getTrxState();
  int getNotch(int t);


 private:
  MaiaXmlRpcServer* rpcServer;
  void log(const QString& cmd, const QString& t);
  void log(const QString& cmd, QVariantList t);
  sxmlInfo rigInfo;
};

#endif  // XMLINTERFACE_H
