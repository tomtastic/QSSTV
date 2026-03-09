#ifndef HEXCONVERTOR_H
#define HEXCONVERTOR_H

#include <QString>
#include <QByteArray>


bool hexFromString(const QString& s, QByteArray& ba, bool toHex);
#endif  // HEXCONVERTOR_H
