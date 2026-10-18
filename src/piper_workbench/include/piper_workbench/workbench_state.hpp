#pragma once
#include <QVector>
#include <QJsonObject>
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
}
