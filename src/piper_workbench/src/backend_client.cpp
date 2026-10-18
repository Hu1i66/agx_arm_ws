#include "piper_workbench/backend_client.hpp"
#include <QJsonDocument>
#include <QTimer>
BackendClient::BackendClient(QObject *parent):QObject(parent){
  connect(&socket_,&QLocalSocket::connected,this,[this]{emit online(true);});
  connect(&socket_,&QLocalSocket::disconnected,this,[this]{buffer_.clear();pending_.clear();emit online(false);});
  connect(&socket_,&QLocalSocket::readyRead,this,[this]{
    buffer_+=socket_.readAll();int end;
    while((end=buffer_.indexOf('\n'))>=0){
      auto row=QJsonDocument::fromJson(buffer_.left(end)).object();buffer_.remove(0,end+1);
      if(row.contains("event")){emit event(row["event"].toString(),row["data"]);continue;}
      auto id=row["id"].toString();auto reply=pending_.take(id);
      if(!row["ok"].toBool()){emit error(row["error"].toObject()["message"].toString());continue;}
      if(reply)reply(row["result"]);
    }
  });
  auto retry=new QTimer(this);connect(retry,&QTimer::timeout,this,[this]{
    if(socket_.state()==QLocalSocket::UnconnectedState)socket_.connectToServer(QString::fromLocal8Bit(qgetenv("AGX_ARM_WS"))+"/.runtime/workbench.sock");
  });retry->start(1200);QTimer::singleShot(0,this,[this]{socket_.connectToServer(QString::fromLocal8Bit(qgetenv("AGX_ARM_WS"))+"/.runtime/workbench.sock");});
}
QString BackendClient::call(QString op,QJsonObject args,Reply reply){
  if(!connected()){emit error("后台尚未连接，请从桌面入口启动工作台或检查后台日志");return {};}
  auto id=QString::number(++sequence_);if(reply)pending_[id]=reply;
  socket_.write(QJsonDocument(QJsonObject{{"id",id},{"op",op},{"args",args}}).toJson(QJsonDocument::Compact)+'\n');return id;
}

BackendClient::~BackendClient(){
 // QLocalSocket can emit disconnected from its destructor, after buffer_/pending_ are gone.
 QObject::disconnect(&socket_,nullptr,this,nullptr);socket_.abort();
}
