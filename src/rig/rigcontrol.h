#ifndef RIGCONTROL_H
#define RIGCONTROL_H

#include <hamlib/rig.h>
#include "xmlinterface.h"

#include <QObject>
#include <QComboBox>
#include <QTcpSocket>

extern "C" int write_block(hamlib_port_t* p, const char* txbuffer, size_t count);
extern "C" int read_block(hamlib_port_t* p, char* rxbuffer, size_t count);

#define RIGCMDTRIES 4


bool model_Sort(const rig_caps* caps1, const rig_caps* caps2);

struct scatParams {
  QString configLabel;
  QString serialPort; /**<  serial port device*/
  QString radioModel;
  int radioModelNumber;
  QString civAddress;
  int baudrate; /**<  serial port baudrate*/
  QString parity;
  int stopbits;
  int databits;
  QString handshake;
  bool enableCAT;
  bool enableSerialPTT;
  QString pttSerialPort;
  bool activeRTS;
  bool activeDTR;
  bool nactiveRTS;
  bool nactiveDTR;
  ptt_type_t pttType;
  bool enableXMLRPC;
  int XMLRPCPort;
  double txOnDelay;

  // Network control settings
  bool enableHamlibNetworkControl;
  QString hamlibHost;
  int hamlibPort;
};

class rigControl : public QObject {
  Q_OBJECT
 public:
  rigControl(int radioIndex);
  ~rigControl() override;
  bool init();
  bool enabled() { return rigControlEnabled; }
  bool getFrequency(double& frequency);
  bool setFrequency(double frequency);
  bool getMode(QString& mode);
  bool setMode(QString mode, const QString& passBand);
  int getModelNumber(int idx);
  int getRadioModelIndex();
  bool getRadioList(QComboBox* cb);
  void disable();
  scatParams* params() { return &catParams; }
  void activatePTT(bool b);
  double getTxDelay() { return catParams.txOnDelay; }
  int rawCommand(const QByteArray& ba);
  QString initError;

 private:
  RIG* my_rig;    // handle to rig (nstance)
  freq_t freq;    // frequency
  rmode_t rmode;  // radio mode of operation
  pbwidth_t width;
  vfo_t vfo;     // vfo selection
  int strength;  // S-Meter level
  int retcode;   // generic return code from functions
  rig_model_t myrig_model;
  bool rigControlEnabled;
  void errorMessage(int errorCode, const QString& command);
  void getRadioList();
  scatParams catParams;
  int serialP;
  bool setPTT(bool On);
  double lastFrequency;
  QStringList xmlModes;
  bool canSetFreq;
  bool canGetFreq;
  bool canSetMode;
  bool canGetMode;
  bool canSetPTT;
  bool canGetPTT;

  // Hamlib network control members
  QTcpSocket* hamlibSocket;

  bool initHamlibNetwork();
  bool sendHamlibCommand(const QString& command);
  QString readHamlibResponse();
};


#endif
