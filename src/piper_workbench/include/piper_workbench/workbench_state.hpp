#pragma once
#include <QVector>
#include <QJsonObject>
#include <QRegularExpression>
#include <cmath>
namespace WorkbenchState {
inline bool targetChanged(const QVector<double> &previous,const QVector<double> &current){
 if(previous.size()!=current.size())return true;
 for(int i=0;i<current.size();++i)if(!std::isfinite(current[i])||std::abs(previous[i]-current[i])>1e-7)return true;
 return false;
}
inline QJsonObject selection(const QJsonObject &selections,QString kind,QString camera){
 return selections[kind+":"+(kind=="tcp"?QString("d455"):camera)].toObject();
}
inline QString logSignature(QString source,QString message){
 static const QRegularExpression ansi(QStringLiteral("\\x1B\\[[0-9;]*[A-Za-z]"));
 static const QRegularExpression timestamp(QStringLiteral("\\[\\d{6,}(?:\\.\\d+)?\\]"));
 static const QRegularExpression paths(QStringLiteral("(?:package://|file://)?[A-Za-z0-9_./-]+\\.(?:png|svg|jpg|jpeg|yaml|json|py|cpp|hpp)"));
 static const QRegularExpression numbers(QStringLiteral("\\d+(?:\\.\\d+)?"));
 static const QRegularExpression whitespace(QStringLiteral("\\s+"));
 message.remove(ansi);message.remove(timestamp);message.replace(paths,QStringLiteral("<file>")).replace(numbers,QStringLiteral("#"));
 return source+QStringLiteral("|")+message.replace(whitespace,QStringLiteral(" ")).trimmed().left(320);
}
}
