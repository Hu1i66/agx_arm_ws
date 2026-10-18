#pragma once
#include <QObject>
#include <QLocalSocket>
#include <QJsonObject>
#include <QMap>
#include <functional>
class BackendClient : public QObject {
  Q_OBJECT
public:
  using Reply = std::function<void(QJsonValue)>;
  explicit BackendClient(QObject *parent=nullptr);
  ~BackendClient() override;
  QString call(QString op,QJsonObject args={},Reply reply={});
  bool connected() const {return socket_.state()==QLocalSocket::ConnectedState;}
signals:
  void event(QString,QJsonValue);
  void error(QString);
  void online(bool);
private:
  QLocalSocket socket_;QByteArray buffer_;int sequence_=0;
  QMap<QString,Reply> pending_;
};
