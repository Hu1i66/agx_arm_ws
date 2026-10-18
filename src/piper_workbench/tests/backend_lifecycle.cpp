#include "piper_workbench/backend_client.hpp"
#include <QCoreApplication>
#include <QLocalServer>
#include <QDir>
#include <QElapsedTimer>
int main(int argc,char **argv){
 QCoreApplication app(argc,argv);QString root=QDir::currentPath()+"/.runtime/lifecycle-test";
 QDir().mkpath(root+"/.runtime");qputenv("AGX_ARM_WS",root.toUtf8());QLocalServer server;auto path=root+"/.runtime/workbench.sock";QLocalServer::removeServer(path);if(!server.listen(path))return 2;
 int count=0;
 for(int i=0;i<100;++i){
  auto *client=new BackendClient;QElapsedTimer timer;timer.start();while(!client->connected()&&timer.elapsed()<1000)app.processEvents();
  if(!client->connected())return 3;
  QList<QLocalSocket *> accepted;while(server.hasPendingConnections()){auto *socket=server.nextPendingConnection();if(socket)accepted<<socket;}
  client->call("status.get",{},[&](QJsonValue){++count;});
  for(auto *socket:accepted){socket->write(QByteArray(20000,'x'));socket->flush();}
  app.processEvents();delete client;app.processEvents();
  for(auto *socket:accepted)delete socket;
 }
 server.close();QLocalServer::removeServer(path);QDir(root+"/.runtime").rmdir(".");QDir(root).rmdir(".");return 0;
}
