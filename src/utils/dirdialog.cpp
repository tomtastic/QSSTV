#include "dirdialog.h"
#include <QDir>
#include <QFileDialog>

static QString lastPath;

dirDialog::dirDialog(QWidget * parent, QString title)
{
  parentPtr = parent;
  dialogTitle = title;
}

dirDialog::~dirDialog()
{
}

QString dirDialog::openFileName(const QString &path, const QString &filter)
{
  QString fn;
  if (path.isEmpty() && lastPath.isEmpty())
  {
    lastPath = QDir::homePath();
  }
  else if (!path.isEmpty())
  {
    lastPath = path;
  }

  QFileDialog dialog(parentPtr, dialogTitle, lastPath, filter);
  dialog.setOption(QFileDialog::DontUseNativeDialog, true);

  
  if (dialog.exec() == QDialog::Accepted)
  {
    fn = dialog.selectedFiles().first();
    if (!fn.isEmpty())
    {
      QFileInfo fi(fn);
      lastPath = fi.absolutePath();
    }
  }
  return fn;
}

QStringList dirDialog::openFileNames(const QString &path, const QString &filter)
{
  QStringList fl;
  if (path.isEmpty() && lastPath.isEmpty())
  {
    lastPath = QDir::homePath();
  }
  else if (!path.isEmpty())
  {
    lastPath = path;
  }

  QFileDialog dialog(parentPtr, dialogTitle, lastPath, filter);
  dialog.setOption(QFileDialog::DontUseNativeDialog, true);


  if (dialog.exec() == QDialog::Accepted)
  {
    fl = dialog.selectedFiles();
    if (!fl.isEmpty())
    {
      QFileInfo fi(fl.first());
      lastPath = fi.absolutePath();
    }
  }
  return fl;
}

QString dirDialog::openDirName(const QString &path)
{
  QString fn;
  if (path.isEmpty() && lastPath.isEmpty())
  {
    lastPath = QDir::homePath();
  }
  else if (!path.isEmpty())
  {
    lastPath = path;
  }

  QFileDialog dialog(parentPtr, dialogTitle, lastPath);
  dialog.setFileMode(QFileDialog::Directory);
  dialog.setOption(QFileDialog::DontUseNativeDialog, true);


  if (dialog.exec() == QDialog::Accepted)
  {
    fn = dialog.selectedFiles().first();
    if (!fn.isEmpty())
    {
      lastPath = fn;
    }
  }
  return fn;
}

QString dirDialog::saveFileName(const QString &path, const QString &filter, QString extension)
{
  QString fn;
  if (path.isEmpty() && lastPath.isEmpty())
  {
    lastPath = QDir::currentPath();
  }
  else if (!path.isEmpty())
  {
    lastPath = path;
  }

  QFileDialog dialog(parentPtr, dialogTitle, lastPath, filter);
  dialog.setAcceptMode(QFileDialog::AcceptSave);
  dialog.setOption(QFileDialog::DontUseNativeDialog, true);


  if (dialog.exec() == QDialog::Accepted)
  {
    fn = dialog.selectedFiles().first();
    if (!fn.isEmpty())
    {
      QFileInfo fi(fn);
      if (!extension.isEmpty() && fi.suffix().isEmpty())
      {
        fn += "." + extension;
      }
      lastPath = fi.absolutePath();
    }
  }
  return fn;
}
