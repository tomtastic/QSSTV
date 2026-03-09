#ifndef FTPFUNCTIONS_H
#define FTPFUNCTIONS_H

#include <QString>
#include <QObject>
#include <QMessageBox>
#include <QTemporaryFile>
#include "ftpthread.h"

class ftpThread;

class ftpFunctions : public QObject {
  Q_OBJECT
 public:
  ftpFunctions();
  ~ftpFunctions() override;
  bool test(const QString& name, const QString& tHost, int tPort, const QString& tUser, const QString& tPasswd,
            const QString& tDirectory, bool doSetup);
  void uploadFile(const QString& source, const QString& destination, bool wait, bool closeWhenDone);
  void uploadData(const QByteArray& ba, const QString& destination, bool wait, bool closeWhenDone);
  void downloadFile(const QString& source, const QString& destination, bool wait, bool closeWhenDone);
  void remove(const QString& source, bool wait, bool closeWhenDone);
  void rename(const QString& tSource, const QString& tDestination, bool wait, bool closeDone);
  void setupFtp(const QString& name, const QString& tHost, int tPort, const QString& tUser, const QString& tPasswd,
                const QString& tDirectory);
  void listFiles(const QString& mask, bool closeWhenDone);
  void mremove(const QString& mask, bool wait, bool closeWhenDone);
  void disconnectFtp();
  eftpError getLastError() { return lastError; }
  eftpError getLastErrorStr(QString& lastErrorString) {
    lastErrorString = lastErrorStr;
    return lastError;
  }
  QList<QUrlInfo> getListing();
  bool isBusy() { return busy; }
  void changePath(const QString& source, bool wait);
  void changeThreadName(const QString& tidName);


 private slots:
  void slotCommandsDone(int err, QString errStr);
  void slotThreadFinished();
  void slotDownloadFinished(bool err, const QString& filename);
  void slotListingFinished(bool err);


 signals:
  void downloadDone(bool, QString);
  void listingDone(bool err);

 private:
  void setupFtp();

  bool checkUpload(const QString& fn, const QString& rfn);
  bool checkRemove(const QString& rfn);
  bool checkStart();
  void checkWait(bool wait);

  ftpThread* ftpThr;
  QString idName;
  QString host;
  int port;
  QString user;
  QString passwd;
  QString directory;

  QMessageBox mb;
  bool endOfCommands;
  eftpError lastError;
  QString lastErrorStr;
  QThread* thread;
  QTemporaryFile ftmp;
  int commandID;
  bool busy;
  bool mremoveCmd;
};

#endif  // FTPFUNCTIONS_H
