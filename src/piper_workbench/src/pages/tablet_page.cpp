#include "piper_workbench/main_window.hpp"
#include "piper_workbench/backend_client.hpp"
using namespace Ui;

static QString tabletRole(const QString &role){return role=="operator"?"操作者":role=="observer"?"观察者":role;}
static QString tabletAge(qint64 stamp){
 if(stamp<=0)return "—";
 qint64 seconds=qMax<qint64>(0,QDateTime::currentSecsSinceEpoch()-stamp);
 if(seconds<60)return QString("%1 秒前").arg(seconds);
 if(seconds<3600)return QString("%1 分钟前").arg(seconds/60);
 return QString("%1 小时前").arg(seconds/3600);
}

QWidget *MainWindow::makeTabletManagement(){
 auto *page=new QWidget;auto *layout=new QVBoxLayout(page);layout->setContentsMargins(0,12,0,0);layout->setSpacing(12);
 auto *top=card("");auto *topRow=new QWidget;auto *tl=new QHBoxLayout(topRow);tl->setContentsMargins(0,0,0,0);
 tabletSummary_=text("网关状态读取中","notice");tabletSummary_->setWordWrap(true);tabletSummary_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred);tl->addWidget(tabletSummary_,1);
 auto *refreshButton=button("刷新","refresh");auto *start=button("启动服务","play");auto *stop=button("停止服务","stop");for(auto *b:{refreshButton,start,stop}){b->setMinimumWidth(92);b->setMaximumWidth(116);b->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);tl->addWidget(b);}static_cast<QVBoxLayout *>(top->layout())->addWidget(topRow);layout->addWidget(top);
 connect(refreshButton,&QPushButton::clicked,this,[this,refreshButton]{request(refreshButton,"tablet.status.get",{},[this](QJsonValue){refreshTablet();},"正在刷新…","已刷新");});
 connect(start,&QPushButton::clicked,this,[this,start]{request(start,"tablet.start_all",{},[this](QJsonValue){refreshTablet();},"正在启动…","已启动");});
 connect(stop,&QPushButton::clicked,this,[this,stop]{request(stop,"tablet.stop_all",{},[this](QJsonValue){refreshTablet();},"正在停止…","已停止");});
 auto *grid=new QGridLayout;grid->setSpacing(12);layout->addLayout(grid,1);

 auto *pair=card("平板接入与配对");auto *pl=static_cast<QVBoxLayout *>(pair->layout());
 tabletPairQr_=new QLabel("网关启动后显示配对二维码");tabletPairQr_->setAlignment(Qt::AlignCenter);tabletPairQr_->setMinimumSize(220,220);tabletPairQr_->setMaximumHeight(238);tabletPairQr_->setStyleSheet("background:#F8FAFD;border:1px solid #E2E8F0;border-radius:7px;color:#64748B;");pl->addWidget(tabletPairQr_);
 tabletPairCode_=new QLabel("尚未生成");tabletPairCode_->setAlignment(Qt::AlignCenter);tabletPairCode_->setStyleSheet("font:700 26px 'DejaVu Sans Mono';color:#1769FF;");pl->addWidget(tabletPairCode_);
 tabletPairAddress_=new QLabel("—");tabletPairAddress_->setWordWrap(true);tabletPairAddress_->setTextInteractionFlags(Qt::TextSelectableByMouse);tabletPairAddress_->setStyleSheet("color:#42546F;");pl->addWidget(tabletPairAddress_);
 auto *pairForm=new QFormLayout;tabletPairRole_=new QComboBox;tabletPairRole_->addItem("操作者 · 可控制","operator");tabletPairRole_->addItem("观察者 · 只读","observer");pairForm->addRow("默认角色",tabletPairRole_);pl->addLayout(pairForm);
 auto *pairButtons=new QGridLayout;auto *generate=button("生成新配对码",true,"refresh");auto *copy=button("复制地址","save");auto *revokeAll=button("撤销全部设备","stop");revokeAll->setObjectName("danger");pairButtons->addWidget(generate,0,0);pairButtons->addWidget(copy,0,1);pairButtons->addWidget(revokeAll,1,0,1,2);pl->addLayout(pairButtons);
 pl->addWidget(text("配对码有效期 10 分钟，仅可使用一次。二维码由网关生成，C++ 页面不维护第二套配对状态。","muted"));
 connect(generate,&QPushButton::clicked,this,[this,generate]{request(generate,"tablet.pair.create",{{"role",tabletPairRole_->currentData().toString()}},[this](QJsonValue){refreshTablet();},"正在生成…","已生成");});
 connect(copy,&QPushButton::clicked,this,[this]{QString value=tabletPairAddress_->property("pairUrl").toString();if(value.isEmpty()){showError("当前没有可用配对地址");return;}QGuiApplication::clipboard()->setText(value);toast_->setText("配对地址已复制");});
 connect(revokeAll,&QPushButton::clicked,this,[this,revokeAll]{request(revokeAll,"tablet.pair.revoke_all",{},[this](QJsonValue){refreshTablet();},"正在撤销…","已撤销");});
 grid->addWidget(pair,0,0);

 auto *devices=card("在线设备与访问权限");auto *dl=static_cast<QVBoxLayout *>(devices->layout());
 tabletDevices_=new QTableWidget(0,10);tabletDevices_->setHorizontalHeaderLabels({"设备","角色","地址","会话","WS RTT","视频","App","租约","最后心跳","操作"});tabletDevices_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);tabletDevices_->horizontalHeader()->setSectionResizeMode(0,QHeaderView::ResizeToContents);tabletDevices_->setSelectionBehavior(QAbstractItemView::SelectRows);tabletDevices_->setSelectionMode(QAbstractItemView::SingleSelection);tabletDevices_->setEditTriggers(QAbstractItemView::NoEditTriggers);tabletDevices_->verticalHeader()->hide();tabletDevices_->setMinimumHeight(270);dl->addWidget(tabletDevices_,1);
 auto *deviceActions=new QHBoxLayout;auto *takeLease=button("收回控制租约","stop");auto *revoke=button("撤销选中设备","stop");revoke->setObjectName("danger");deviceActions->addWidget(takeLease);deviceActions->addWidget(revoke);deviceActions->addStretch();dl->addLayout(deviceActions);
 connect(takeLease,&QPushButton::clicked,this,[this,takeLease]{request(takeLease,"tablet.lease.revoke",{},[this](QJsonValue){refreshTablet();},"正在收回…","已收回");});
 connect(revoke,&QPushButton::clicked,this,[this,revoke]{auto *item=tabletDevices_->item(tabletDevices_->currentRow(),0);QString id=item?item->data(Qt::UserRole).toString():QString();if(id.isEmpty()){showError("请先选择在线设备");return;}request(revoke,"tablet.device.revoke",{{"device_id",id}},[this](QJsonValue){refreshTablet();},"正在撤销…","已撤销");});
 grid->addWidget(devices,0,1,1,2);

 auto *media=card("视频流健康");auto *ml=static_cast<QVBoxLayout *>(media->layout());tabletMedia_=new QTableWidget(2,7);tabletMedia_->setHorizontalHeaderLabels({"相机","编码","输入 FPS","码率","接收端","帧龄","状态"});tabletMedia_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);tabletMedia_->setEditTriggers(QAbstractItemView::NoEditTriggers);tabletMedia_->verticalHeader()->hide();tabletMedia_->setMinimumHeight(145);ml->addWidget(tabletMedia_);auto *mediaButtons=new QHBoxLayout;for(QString cameraId:{"d455","dabai"}){auto *b=button(cameraId=="d455"?"请求 D455 关键帧":"请求 Dabai 关键帧","refresh");mediaButtons->addWidget(b);connect(b,&QPushButton::clicked,this,[this,cameraId,b]{request(b,"tablet.stream.keyframe",{{"camera_id",cameraId}},[this](QJsonValue){refreshTablet();},"正在请求…","已请求");});}ml->addLayout(mediaButtons);grid->addWidget(media,1,0,1,2);

 auto *services=card("服务与本地证书");auto *sl=static_cast<QVBoxLayout *>(services->layout());auto *gatewayRow=new QHBoxLayout;gatewayRow->addWidget(text("网关"));auto *gStart=button("启动");auto *gStop=button("停止");auto *gRestart=button("重启","refresh");for(auto *b:{gStart,gStop,gRestart})gatewayRow->addWidget(b);sl->addLayout(gatewayRow);
 connect(gStart,&QPushButton::clicked,this,[this,gStart]{request(gStart,"tablet.gateway.start",{},[this](QJsonValue){refreshTablet();},"正在启动…","已启动");});
 connect(gStop,&QPushButton::clicked,this,[this,gStop]{request(gStop,"tablet.gateway.stop",{},[this](QJsonValue){refreshTablet();},"正在停止…","已停止");});
 connect(gRestart,&QPushButton::clicked,this,[this,gRestart]{request(gRestart,"tablet.gateway.restart",{},[this](QJsonValue){refreshTablet();},"正在重启…","已重启");});
 auto *mediaRow=new QHBoxLayout;mediaRow->addWidget(text("媒体"));auto *mStart=button("启动");auto *mStop=button("停止");auto *mRestart=button("重启","refresh");for(auto *b:{mStart,mStop,mRestart})mediaRow->addWidget(b);sl->addLayout(mediaRow);
 connect(mStart,&QPushButton::clicked,this,[this,mStart]{request(mStart,"tablet.media.start",{},[this](QJsonValue){refreshTablet();},"正在启动…","已启动");});
 connect(mStop,&QPushButton::clicked,this,[this,mStop]{request(mStop,"tablet.media.stop",{},[this](QJsonValue){refreshTablet();},"正在停止…","已停止");});
 connect(mRestart,&QPushButton::clicked,this,[this,mRestart]{request(mRestart,"tablet.media.restart",{},[this](QJsonValue){refreshTablet();},"正在重启…","已重启");});
 tabletCertState_=text("证书状态读取中","notice");sl->addWidget(tabletCertState_);
 auto *certButtons=new QHBoxLayout;auto *download=button("打开本地 CA 证书","save");auto *regen=button("重新生成证书","refresh");certButtons->addWidget(download);certButtons->addWidget(regen);sl->addLayout(certButtons);
 connect(download,&QPushButton::clicked,this,[this,download]{request(download,"tablet.certificate.download",{},[this](QJsonValue v){showFile(v.toObject()["path"].toString());},"正在定位…","已打开");});
 connect(regen,&QPushButton::clicked,this,[this,regen]{request(regen,"tablet.certificate.regenerate",{},[this](QJsonValue){refreshTablet();toast_->setText("本地证书已重新生成；重启网关后生效，平板需重新信任新 CA");},"正在生成…","已生成");});
 auto *logButtons=new QHBoxLayout;auto *gatewayLog=button("网关日志","logs");auto *mediaLog=button("媒体日志","logs");logButtons->addWidget(gatewayLog);logButtons->addWidget(mediaLog);sl->addLayout(logButtons);
 connect(gatewayLog,&QPushButton::clicked,this,[this]{showFile(QString::fromLocal8Bit(qgetenv("AGX_ARM_WS"))+"/log/runtime/tablet-gateway.log");});
 connect(mediaLog,&QPushButton::clicked,this,[this]{showFile(QString::fromLocal8Bit(qgetenv("AGX_ARM_WS"))+"/log/runtime/tablet-media.log");});
 sl->addStretch();grid->addWidget(services,1,2);

 auto *events=card("最近平板事件与错误");auto *el=static_cast<QVBoxLayout *>(events->layout());tabletEvents_=new QPlainTextEdit;tabletEvents_->setReadOnly(true);tabletEvents_->setMaximumBlockCount(200);tabletEvents_->setPlaceholderText("配对、重连、租约、媒体建立和网关错误将显示在这里；完整日志进入日志页");el->addWidget(tabletEvents_);layout->addWidget(events);
 grid->setColumnStretch(0,2);grid->setColumnStretch(1,3);grid->setColumnStretch(2,2);
 QTimer::singleShot(0,this,[this]{refreshTablet();});return Ui::scroll(page);
}

void MainWindow::refreshTablet(){
 if(!backend_||!backend_->connected())return;
 backend_->call("tablet.status.get",{},[this](QJsonValue value){
  auto data=value.toObject();auto gateway=data["gateway"].toObject();auto process=data["gateway_process"].toObject();auto mediaProcess=data["media_process"].toObject();
  auto pwa=data["pwa"].toObject();
  bool online=gateway["state"].toString()=="online";auto cert=gateway["certificate"].toObject();QString certState=cert["state"].toString()=="valid"?QString("证书有效 · 剩余 %1 天").arg(cert["days_remaining"].toInt()):"证书未配置或已过期";
  auto devices=gateway["devices"].toArray();int onlineDevices=0;for(auto row:devices)if(row.toObject()["online"].toBool())++onlineDevices;
  auto lease=gateway["lease"].toObject();QString leaseText=lease.isEmpty()?"无控制租约":QString("%1 · 剩余 %2 秒").arg(lease["device_name"].toString()).arg(lease["remaining_s"].toDouble(),0,'f',1);
  auto media=gateway["media"].toObject();
  tabletSummary_->setText(QString("<b>网关</b>  %1　　<b>HTTPS</b>  %2　　<b>媒体</b>  %3　　<b>PWA</b>  v%4 / cache %5\n<b>在线设备</b>  %6　　<b>控制租约</b>  %7")
   .arg(online?"在线":process["state"].toString()=="running"?"进程运行中 · 等待 HTTPS":"未启动",certState,media["state"].toString()=="online"?"在线":"离线")
   .arg(pwa["version"].toString(),pwa["cache_version"].toString()).arg(onlineDevices).arg(leaseText));
  auto pair=gateway["pair"].toObject();QString code=pair["code"].toString();tabletPairExpires_=qint64(pair["expires_at"].toDouble());tabletPairAddress_->setProperty("pairUrl",pair["url"].toString());QStringList addresses;for(auto row:gateway["addresses"].toArray())addresses<<row.toString();tabletPairAddress_->setText(pair["url"].toString().isEmpty()?"网关在线后可生成一次性配对地址；局域网备用地址："+addresses.join("  "):pair["url"].toString());
  if(code.isEmpty()){tabletPairCode_->setText("尚未生成");tabletPairQr_->setText("请生成新的配对码");tabletPairQr_->setPixmap(QPixmap());}
  else{
   qint64 remaining=qMax<qint64>(0,tabletPairExpires_-QDateTime::currentSecsSinceEpoch());
   tabletPairCode_->setText(code+QString("　%1:%2").arg(remaining/60,2,10,QChar('0')).arg(remaining%60,2,10,QChar('0')));
   QByteArray bytes=QByteArray::fromBase64(pair["qr_png_base64"].toString().toLatin1());QPixmap qr;if(qr.loadFromData(bytes,"PNG"))tabletPairQr_->setPixmap(qr.scaled(218,218,Qt::KeepAspectRatio,Qt::SmoothTransformation));else tabletPairQr_->setText("二维码解析失败");
  }
  {QSignalBlocker block(tabletDevices_);tabletDevices_->setRowCount(devices.size());for(int i=0;i<devices.size();++i){
   auto row=devices[i].toObject();QString id=row["id"].toString();auto *name=new QTableWidgetItem(row["online"].toBool()?row["name"].toString():row["name"].toString()+" · 离线");name->setData(Qt::UserRole,id);tabletDevices_->setItem(i,0,name);
   tabletDevices_->setItem(i,1,new QTableWidgetItem(tabletRole(row["role"].toString())));tabletDevices_->setItem(i,2,new QTableWidgetItem(row["ip"].toString("—")));
   qint64 duration=qMax<qint64>(0,qint64(QDateTime::currentSecsSinceEpoch()-row["created_at"].toDouble()));tabletDevices_->setItem(i,3,new QTableWidgetItem(QString("%1 分").arg(duration/60)));
   tabletDevices_->setItem(i,4,new QTableWidgetItem(row["ws_rtt_ms"].isNull()?"—":QString("%1 ms").arg(row["ws_rtt_ms"].toInt())));
   tabletDevices_->setItem(i,5,new QTableWidgetItem(row["online"].toBool()?"可请求":"—"));tabletDevices_->setItem(i,6,new QTableWidgetItem(row["app_version"].toString("—")));
   bool held=row["id"].toString()==lease["device_id"].toString();tabletDevices_->setItem(i,7,new QTableWidgetItem(held?leaseText:"—"));tabletDevices_->setItem(i,8,new QTableWidgetItem(tabletAge(qint64(row["last_seen"].toDouble()))));
   auto *detail=new QTableWidgetItem("查看详情");detail->setToolTip(QString("User-Agent\n%1\n\n页面\n%2\n\n最近重连原因\n%3").arg(row["user_agent"].toString(),row["page"].toString(),row["reconnect_reason"].toString()));tabletDevices_->setItem(i,9,detail);
  }}
  auto cameras=media["cameras"].toObject();for(int i=0;i<2;++i){QString cameraId=i==0?"d455":"dabai";auto row=cameras[cameraId].toObject();tabletMedia_->setItem(i,0,new QTableWidgetItem(cameraId=="d455"?"D455":"Dabai"));tabletMedia_->setItem(i,1,new QTableWidgetItem(row["codec"].toString("等待")));tabletMedia_->setItem(i,2,new QTableWidgetItem(QString::number(row["input_fps"].toDouble(),'f',1)));tabletMedia_->setItem(i,3,new QTableWidgetItem(QString("%1 kbps").arg(row["bitrate_kbps"].toDouble(),0,'f',1)));tabletMedia_->setItem(i,4,new QTableWidgetItem(QString("%1 / WebRTC %2").arg(row["subscribers"].toInt()).arg(row["webrtc_peers"].toInt())));tabletMedia_->setItem(i,5,new QTableWidgetItem(row["frame_age_s"].isNull()?"—":QString("%1 秒").arg(row["frame_age_s"].toDouble(),0,'f',2)));tabletMedia_->setItem(i,6,new QTableWidgetItem(row["state"].toString()=="streaming"?"推流中":"等待图像"));}
  tabletCertState_->setText(certState+QString("\nCA 文件：%1").arg(cert["ca_path"].toString()));
  QStringList events;for(auto row:gateway["events"].toArray()){auto item=row.toObject();events<<QString("%1  %2  %3").arg(item["time"].toString(),item["type"].toString(),QString::fromUtf8(QJsonDocument(item["detail"].toObject()).toJson(QJsonDocument::Compact)));}if(events.isEmpty())events<<"暂无平板事件";tabletEvents_->setPlainText(events.join('\n'));
 });
}
