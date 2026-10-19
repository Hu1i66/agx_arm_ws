#include "piper_workbench/ui_helpers.hpp"
#include <algorithm>
#include <cmath>
namespace {
class AnimatedButton:public QPushButton{
public:
 explicit AnimatedButton(QString text):QPushButton(text){
  animation_.setDuration(110);animation_.setEasingCurve(QEasingCurve::OutCubic);
  connect(&animation_,&QVariantAnimation::valueChanged,this,[this](QVariant v){overlay_=v.toDouble();update();});
 }
 bool event(QEvent *event)override{
  double target=-1;
  switch(event->type()){
   case QEvent::Enter:target=.06;break;case QEvent::Leave:target=0;break;
   case QEvent::MouseButtonPress:target=.16;break;case QEvent::MouseButtonRelease:target=.06;break;default:break;
  }
  if(target>=0){animation_.stop();animation_.setStartValue(overlay_);animation_.setEndValue(target);animation_.start();}
  return QPushButton::event(event);
 }
 void paintEvent(QPaintEvent *event)override{
  QPushButton::paintEvent(event);if(overlay_<=0)return;QPainter p(this);p.setRenderHint(QPainter::Antialiasing);
  p.setPen(Qt::NoPen);p.setBrush(QColor(35,109,255,int(overlay_*255)));p.drawRoundedRect(rect().adjusted(1,1,-1,-1),6,6);
 }
private:QVariantAnimation animation_;double overlay_=0;
};
}
QIcon Ui::icon(QString name){return QIcon(QString::fromLocal8Bit(qgetenv("AGX_ARM_WS"))+"/src/piper_workbench/resources/icons/"+name+".svg");}
QFrame *Ui::card(QString title,QWidget *parent){
 auto *box=new QFrame(parent);box->setObjectName("card");auto *shadow=new QGraphicsDropShadowEffect(box);shadow->setBlurRadius(12);shadow->setOffset(0,2);shadow->setColor(QColor(32,58,92,9));box->setGraphicsEffect(shadow);auto *layout=new QVBoxLayout(box);layout->setContentsMargins(16,14,16,14);layout->setSpacing(12);
 if(!title.isEmpty()){
  auto *head=new QHBoxLayout;QString symbol=title.contains("相机")||title.contains("预览")?"capture":title.contains("标定")||title.contains("TCP")?"calibrate":title.contains("机械臂")||title.contains("运动")?"robot":title.contains("日志")||title.contains("输出")?"logs":title.contains("连接")||title.contains("启动")?"connect":"grab";
  auto *glyph=new QLabel;glyph->setPixmap(icon(symbol).pixmap(22,22));glyph->setFixedWidth(24);head->addWidget(glyph);auto *label=new QLabel(title);label->setObjectName("cardTitle");head->addWidget(label,1);layout->addLayout(head);
 }return box;
}
QPushButton *Ui::button(QString text,bool primary,QString symbol){
 auto *b=new AnimatedButton(text);b->setMinimumHeight(38);b->setCursor(Qt::PointingHandCursor);
 if(primary)b->setObjectName("primary");if(!symbol.isEmpty()){
  auto glyph=icon(symbol).pixmap(px(20),px(20));if(primary){QPainter paint(&glyph);paint.setCompositionMode(QPainter::CompositionMode_SourceIn);paint.fillRect(glyph.rect(),Qt::white);}b->setIcon(QIcon(glyph));
 }
 return b;
}
QLabel *Ui::text(QString value,QString role){auto *l=new QLabel(value);l->setObjectName(role);l->setWordWrap(true);return l;}
QWidget *Ui::scroll(QWidget *child){auto *s=new QScrollArea;s->setWidgetResizable(true);s->setFrameShape(QFrame::NoFrame);s->setWidget(child);return s;}
QRect Ui::ImageView::imageRect()const{auto size=(frame.isNull()?QSize(640,480):frame.size()).scaled(rect().size(),Qt::KeepAspectRatio);return QRect(QPoint((width()-size.width())/2,(height()-size.height())/2),size);}
void Ui::ImageView::paintEvent(QPaintEvent *){
 QPainter p(this);p.setRenderHint(QPainter::Antialiasing);p.fillRect(rect(),QColor("#F0F3F7"));
 auto box=imageRect();if(frame.isNull()){
  p.setPen(QPen(QColor("#CBD5E1"),1));p.drawRoundedRect(rect().adjusted(1,1,-1,-1),7,7);
  icon("capture").paint(&p,QRect(width()/2-px(20),height()/2-px(52),px(40),px(40)));p.setPen(QColor("#64748B"));p.drawText(rect().adjusted(0,20,0,20),Qt::AlignCenter,waiting);return;
 }
 p.drawImage(box,frame);
 if(ageSeconds>2){
  QRect banner(px(8),px(8),width()-px(16),px(32));p.setPen(Qt::NoPen);p.setBrush(QColor(255,247,224,242));p.drawRoundedRect(banner,6,6);
  p.setPen(QColor("#865A13"));p.drawText(banner,Qt::AlignCenter,QString("画面已暂停 · 上次更新 %1 秒前").arg(ageSeconds,0,'f',0));
 }
 if(selected<0||ageSeconds>2)return;
 for(auto row:detection["objects"].toArray()){
  auto o=row.toObject();if(o["track_id"].toInt(-2)!=selected)continue;auto b=o["bbox_pixel"].toObject();
  QRectF r(box.x()+b["x1"].toDouble()*box.width()/frame.width(),box.y()+b["y1"].toDouble()*box.height()/frame.height(),(b["x2"].toDouble()-b["x1"].toDouble())*box.width()/frame.width(),(b["y2"].toDouble()-b["y1"].toDouble())*box.height()/frame.height());
  p.setPen(QPen(QColor("#1769FF"),3));p.drawRoundedRect(r,4,4);
 }
}
void Ui::ImageView::mousePressEvent(QMouseEvent *event){
 auto r=imageRect();if(!r.contains(event->pos()))return;double x=(event->x()-r.x())*(frame.isNull()?640:frame.width())/double(r.width()),y=(event->y()-r.y())*(frame.isNull()?480:frame.height())/double(r.height());
 for(auto row:detection["objects"].toArray()){auto o=row.toObject(),b=o["bbox_pixel"].toObject();if(x>=b["x1"].toDouble()&&x<=b["x2"].toDouble()&&y>=b["y1"].toDouble()&&y<=b["y2"].toDouble()){selected=o["track_id"].toInt();if(picked)picked(selected);update();break;}}
}

Ui::GraspFlow::GraspFlow(QWidget *parent):QWidget(parent){
 setMinimumHeight(88);setMaximumHeight(88);setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
 animation_.setDuration(420);animation_.setEasingCurve(QEasingCurve::OutCubic);
 connect(&animation_,&QVariantAnimation::valueChanged,this,[this](QVariant value){displayPos_=value.toDouble();update();});
 pulseTimer_.setInterval(33);connect(&pulseTimer_,&QTimer::timeout,this,[this]{pulse_=.5+.5*std::sin(pulseClock_.elapsed()/1000.*5.2);update();});
 pulseClock_.start();pulseTimer_.start();
}
void Ui::GraspFlow::setStage(QString stage,QString detail){
 static const QMap<QString,int> order={{"waiting",0},{"target",1},{"approach",2},{"descend",3},{"grasp",4},{"lift",4},{"place",5},{"release",5},{"return",6},{"complete",6},{"error",-1}};
 stage_=stage;if(!detail.isEmpty())detail_=detail;
 int target=order.value(stage,0);if(target>=0&&target!=int(qRound(displayPos_))){animation_.stop();animation_.setStartValue(displayPos_);animation_.setEndValue(qreal(target));animation_.start();}
 update();
}
void Ui::GraspFlow::setActive(bool active){active_=active;update();}
void Ui::GraspFlow::paintEvent(QPaintEvent *){
 static const QStringList labels={"识别","接近","对齐","夹取","搬运","释放","返回"};
 QPainter p(this);p.setRenderHint(QPainter::Antialiasing);
 QRectF panel=rect().adjusted(0,0,-1,-1);p.setPen(QPen(QColor("#E2E8F0"),1));p.setBrush(QColor("#FBFDFF"));p.drawRoundedRect(panel,8,8);
 const qreal left=22.,right=width()-22.,y=25.,step=(right-left)/(labels.size()-1);
 QLinearGradient track(left,0,right,0);track.setColorAt(0,QColor("#8FB8FF"));track.setColorAt(1,QColor("#1769FF"));
 p.setPen(QPen(QColor("#E6ECF4"),3));p.drawLine(QPointF(left,y),QPointF(right,y));
 if(displayPos_>0){QPointF end(left+step*displayPos_,y);p.setPen(QPen(QBrush(track),3));p.drawLine(QPointF(left,y),end);}
 int activeIndex=qRound(displayPos_);QString activeStage=stage_;bool failed=activeStage=="error";
 for(int i=0;i<labels.size();++i){
  QPointF center(left+step*i,y);bool done=i<activeIndex;bool current=i==activeIndex;
  QColor color=done?QColor("#1769FF"):current?(failed?QColor("#D64545"):QColor("#1769FF")):QColor("#D7E0EC");
  if(current&&active_&&!failed){p.setPen(QPen(QColor(23,105,255,45),7+4*pulse_));p.setBrush(Qt::NoBrush);p.drawEllipse(center,10+pulse_*2,10+pulse_*2);}
  p.setPen(Qt::NoPen);p.setBrush(color);p.drawEllipse(center,done||current?8.:6.,done||current?8.:6.);
  if(done){p.setPen(QPen(Qt::white,2,Qt::SolidLine,Qt::RoundCap));p.drawLine(center+QPointF(-3,0),center+QPointF(-.5,2.5));p.drawLine(center+QPointF(-.5,2.5),center+QPointF(3.5,-3));}
  p.setPen(current?QColor("#0F4FC4"):done?QColor("#365A91"):QColor("#8A99AD"));
  QFont f=font();f.setBold(current);
  if(f.pointSizeF()>0)f.setPointSizeF(std::max(8.,f.pointSizeF()*.93));
  else f.setPixelSize(std::max(10,int((f.pixelSize()>0?f.pixelSize():14)*.93)));
  p.setFont(f);p.drawText(QRectF(center.x()-38,y+12,76,26),Qt::AlignHCenter|Qt::AlignTop,labels[i]);
 }
 QFont f=font();if(f.pointSizeF()>0)f.setPointSizeF(std::max(8.,f.pointSizeF()*.88));else f.setPixelSize(std::max(10,int((f.pixelSize()>0?f.pixelSize():14)*.88)));
 p.setFont(f);p.setPen(QColor("#66768C"));p.drawText(QRectF(18,4,width()-36,18),Qt::AlignRight|Qt::AlignVCenter,detail_);
}

Ui::RegionEditor::RegionEditor(QWidget *parent):QWidget(parent){
 setMinimumSize(420,300);setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);setMouseTracking(true);setCursor(Qt::CrossCursor);
}
void Ui::RegionEditor::setFrame(const QImage &frame){frame_=frame;update();}
void Ui::RegionEditor::setLines(int trigger,int stop,int abandon){lines_[0]=trigger;lines_[1]=stop;lines_[2]=abandon;update();}
void Ui::RegionEditor::setPolygon(const QJsonArray &polygon){polygon_=polygon;update();}
void Ui::RegionEditor::setMode(int mode){mode_=mode;dragLine_=dragVertex_=-1;setCursor(mode==2?Qt::CrossCursor:Qt::ArrowCursor);update();}
QRectF Ui::RegionEditor::imageRect()const{
 QSize source=frame_.isNull()?QSize(640,480):frame_.size();QSize scaled=source.scaled(rect().size(),Qt::KeepAspectRatio);
 return QRectF(QPointF((width()-scaled.width())/2.,(height()-scaled.height())/2.),scaled);
}
bool Ui::RegionEditor::imagePoint(QPoint position,QPointF &point)const{
 QRectF box=imageRect();if(!box.contains(position))return false;QSize source=frame_.isNull()?QSize(640,480):frame_.size();
 point=QPointF((position.x()-box.left())*source.width()/box.width(),(position.y()-box.top())*source.height()/box.height());return true;
}
void Ui::RegionEditor::paintEvent(QPaintEvent *){
 QPainter p(this);p.setRenderHint(QPainter::Antialiasing);p.fillRect(rect(),QColor("#F0F3F7"));QRectF box=imageRect();
 if(frame_.isNull()){
  p.setPen(QColor("#64748B"));p.drawText(rect(),Qt::AlignCenter,"等待 D455 原始画面");return;
 }
 p.drawImage(box,frame_);QSize source=frame_.size();
 QPolygonF polygon;
 for(auto value:polygon_){auto point=value.toArray();polygon<<QPointF(box.left()+point[0].toDouble()*box.width()/source.width(),box.top()+point[1].toDouble()*box.height()/source.height());}
 if(polygon.size()>=2){
  QPen beltPen(QColor("#FF9F1A"),2,Qt::DashLine);p.setPen(beltPen);p.setBrush(QColor(255,159,26,28));
  if(polygon.size()>=3)p.drawPolygon(polygon);else p.drawPolyline(polygon);
  p.setBrush(QColor("#FF9F1A"));p.setPen(QPen(Qt::white,1));
  for(auto point:polygon)p.drawEllipse(point,5,5);
 }
 QStringList names={"触发","停带","放弃"};QList<QColor> colors={QColor("#1769FF"),QColor("#E59A00"),QColor("#D64545")};
 for(int i=0;i<3;++i){
  double x=box.left()+lines_[i]*box.width()/source.width();p.setPen(QPen(colors[i],2));p.drawLine(QPointF(x,box.top()),QPointF(x,box.bottom()));
  QRectF label(x+4,box.top()+6+i*22,56,19);p.setPen(Qt::NoPen);p.setBrush(QColor(255,255,255,220));p.drawRoundedRect(label,4,4);p.setPen(colors[i]);QFont f=font();f.setBold(true);f.setPixelSize(11);p.setFont(f);p.drawText(label,Qt::AlignCenter,QString("%1 %2").arg(names[i]).arg(lines_[i]));
 }
 p.setPen(QColor("#475569"));QFont f=font();f.setPixelSize(11);p.setFont(f);p.drawText(box.adjusted(8,box.height()-25,0,0),Qt::AlignLeft,"橙色虚线：D455 传送带区域");
}
void Ui::RegionEditor::mousePressEvent(QMouseEvent *event){
 QPointF image;if(!imagePoint(event->pos(),image))return;QSize source=frame_.isNull()?QSize(640,480):frame_.size();QRectF box=imageRect();
 if(mode_==0){
  int best=-1;double distance=14.;
  for(int i=0;i<3;++i){double x=box.left()+lines_[i]*box.width()/source.width();double d=std::abs(event->x()-x);if(d<distance){best=i;distance=d;}}
  dragLine_=best;return;
 }
 if(mode_==2){
  polygon_.append(QJsonArray{int(std::lround(image.x())),int(std::lround(image.y()))});dragVertex_=polygon_.size()-1;if(polygonMoved)polygonMoved(polygon_);update();return;
 }
 if(mode_==1){
  int nearest=-1;double bestDistance=12.;
  for(int i=0;i<polygon_.size();++i){auto point=polygon_[i].toArray();QPointF d(box.left()+point[0].toDouble()*box.width()/source.width(),box.top()+point[1].toDouble()*box.height()/source.height());double distance=QLineF(event->pos(),d).length();if(distance<bestDistance){nearest=i;bestDistance=distance;}}
  if(nearest>=0){dragVertex_=nearest;return;}
  if(polygon_.size()>=2){
   double edgeDistance=12.;int insertAfter=-1;
   for(int i=0;i<polygon_.size();++i){
    auto a=polygon_[i].toArray(),b=polygon_[(i+1)%polygon_.size()].toArray();
    QPointF da(box.left()+a[0].toDouble()*box.width()/source.width(),box.top()+a[1].toDouble()*box.height()/source.height());
    QPointF db(box.left()+b[0].toDouble()*box.width()/source.width(),box.top()+b[1].toDouble()*box.height()/source.height());
    QPointF ab=db-da;double length2=QPointF::dotProduct(ab,ab);double t=length2>0?std::clamp(QPointF::dotProduct(QPointF(event->pos())-da,ab)/length2,0.,1.):0.;
    double distance=QLineF(event->pos(),da+t*ab).length();if(distance<edgeDistance){edgeDistance=distance;insertAfter=i;}
   }
   if(insertAfter>=0){QJsonArray next;for(int i=0;i<polygon_.size();++i){next.append(polygon_[i]);if(i==insertAfter)next.append(QJsonArray{int(std::lround(image.x())),int(std::lround(image.y()))});}polygon_=next;dragVertex_=insertAfter+1;if(polygonMoved)polygonMoved(polygon_);update();}
  }
 }
}
void Ui::RegionEditor::mouseMoveEvent(QMouseEvent *event){
 if(!(event->buttons()&Qt::LeftButton))return;QPointF image;if(!imagePoint(event->pos(),image))return;
 int x=int(std::lround(image.x())),y=int(std::lround(image.y()));
 if(mode_==0&&dragLine_>=0){
  if(dragLine_==0)lines_[0]=std::clamp(x,1,lines_[1]-1);
  else if(dragLine_==1)lines_[1]=std::clamp(x,lines_[0]+1,lines_[2]);
  else lines_[2]=std::clamp(x,lines_[1],639);
  if(lineMoved)lineMoved(dragLine_,lines_[dragLine_]);update();
 }else if(mode_==1&&dragVertex_>=0&&dragVertex_<polygon_.size()){
  polygon_[dragVertex_]=QJsonArray{std::clamp(x,0,639),std::clamp(y,0,479)};if(polygonMoved)polygonMoved(polygon_);update();
 }
}
void Ui::RegionEditor::mouseReleaseEvent(QMouseEvent *){
 if(mode_==1&&dragVertex_>=0&&polygonMoved)polygonMoved(polygon_);dragLine_=dragVertex_=-1;
}

void Ui::collapsible(QFrame *card){
 auto *layout=static_cast<QVBoxLayout *>(card->layout());if(!layout->count()||!layout->itemAt(0)->layout())return;
 auto *header=static_cast<QHBoxLayout *>(layout->itemAt(0)->layout());auto *toggle=new QToolButton;toggle->setArrowType(Qt::UpArrow);toggle->setCheckable(true);toggle->setChecked(true);toggle->setToolTip("展开 / 收起");header->addWidget(toggle);
 auto *body=new QWidget;auto *content=new QVBoxLayout(body);content->setContentsMargins(0,0,0,0);content->setSpacing(12);
 while(layout->count()>1){auto *item=layout->takeAt(1);if(item->widget()){content->addWidget(item->widget());delete item;}else if(item->layout())content->addLayout(item->layout());}
 layout->addWidget(body);
 QObject::connect(toggle,&QToolButton::toggled,card,[body,toggle](bool expanded){
  int height=body->sizeHint().height();if(expanded)body->show();auto *animation=new QPropertyAnimation(body,"maximumHeight",body);animation->setDuration(210);animation->setEasingCurve(QEasingCurve::InOutCubic);animation->setStartValue(expanded?0:body->height());animation->setEndValue(expanded?height:0);toggle->setArrowType(expanded?Qt::UpArrow:Qt::DownArrow);
  QObject::connect(animation,&QPropertyAnimation::finished,body,[body,expanded]{if(expanded)body->setMaximumHeight(QWIDGETSIZE_MAX);else body->hide();});animation->start(QAbstractAnimation::DeleteWhenStopped);
 });
}

void Ui::TracePlot::paintEvent(QPaintEvent *){
 QPainter painter(this);painter.setRenderHint(QPainter::Antialiasing);painter.fillRect(rect(),QColor("#FAFCFF"));QRectF plot(55,22,width()-80,height()-65);painter.setPen(QPen(QColor("#CBD5E1"),1));painter.drawLine(plot.bottomLeft(),plot.topLeft());painter.drawLine(plot.bottomLeft(),plot.bottomRight());
 QVector<QPointF> actual,target;
 for(auto value:report["feedback"].toArray()){
  auto row=value.toObject();if(!row["trajectory_s"].isDouble())continue;double t=row["trajectory_s"].toDouble();
  if(metric==0&&row["error_mm"].isDouble())actual<<QPointF(t,row["error_mm"].toDouble());
  if(metric==1){auto feedback=row["feedback_tcp_m"].toArray(),wanted=row["target_tcp_m"].toArray();if(feedback.size()==3&&wanted.size()==3){actual<<QPointF(t,feedback[2].toDouble()*1000.);target<<QPointF(t,wanted[2].toDouble()*1000.);}}
  if(metric==2&&row["gripper_width_m"].isDouble())actual<<QPointF(t,row["gripper_width_m"].toDouble()*1000.);
 }
 painter.setPen(QColor("#64748B"));painter.drawText(QRectF(0,0,50,20),"mm");painter.drawText(QRectF(width()-90,height()-30,80,20),"时间 / 秒");
 if(actual.size()<2){painter.drawText(plot,Qt::AlignCenter,"报告未包含此曲线的数据");return;}
 double xmin=actual[0].x(),xmax=actual.back().x(),ymin=actual[0].y(),ymax=ymin;
 for(const auto &list:{actual,target})for(auto point:list){ymin=std::min(ymin,point.y());ymax=std::max(ymax,point.y());}
 if(xmax<=xmin)xmax=xmin+1;if(ymax-ymin<.1)ymax=ymin+.1;double margin=(ymax-ymin)*.12;ymin-=margin;ymax+=margin;
 for(int i=0;i<=4;++i){double y=plot.bottom()-i*plot.height()/4.;painter.setPen(QPen(QColor("#E7EDF5"),1));painter.drawLine(QPointF(plot.left(),y),QPointF(plot.right(),y));painter.setPen(QColor("#64748B"));painter.drawText(QRectF(0,y-10,48,20),Qt::AlignRight,QString::number(ymin+(ymax-ymin)*i/4.,'f',1));}
 auto draw=[&](const QVector<QPointF> &points,QColor color,Qt::PenStyle style){QPainterPath path;bool first=true;for(auto value:points){QPointF point(plot.left()+(value.x()-xmin)/(xmax-xmin)*plot.width(),plot.bottom()-(value.y()-ymin)/(ymax-ymin)*plot.height());if(first){path.moveTo(point);first=false;}else path.lineTo(point);}painter.setPen(QPen(color,2,style));painter.drawPath(path);};
 draw(actual,QColor("#1769FF"),Qt::SolidLine);draw(target,QColor("#7587A3"),Qt::DashLine);painter.setPen(QColor("#64748B"));painter.drawText(QRectF(plot.left(),plot.bottom()+8,80,20),QString::number(xmin,'f',2));painter.drawText(QRectF(plot.right()-55,plot.bottom()+8,55,20),Qt::AlignRight,QString::number(xmax,'f',2));
}

// Humble RViz passes logical sizes to Ogre's native X11 surface. Run Qt at
// native pixels, then scale only our widget metrics to retain desktop readability.
// See ros2/rviz#1052 and #1870; no system Qt/ROS libraries are replaced.
int Ui::px(int value){return qRound(value*qApp->property("workbenchUiScale").toDouble());}
void Ui::scaleNativeUi(QWidget *root){
 const double scale=qApp->property("workbenchUiScale").toDouble();if(qFuzzyCompare(scale,1.))return;
 QString css=qApp->styleSheet();QRegularExpression pixels("(-?\\d+)px");auto matches=pixels.globalMatch(css);QVector<QRegularExpressionMatch> positions;while(matches.hasNext())positions<<matches.next();
 for(auto it=positions.crbegin();it!=positions.crend();++it)css.replace(it->capturedStart(),it->capturedLength(),QString::number(px(it->captured(1).toInt()))+"px");qApp->setStyleSheet(css);
 auto widgets=root->findChildren<QWidget *>();widgets.prepend(root);
 for(auto *widget:widgets){
  auto minimum=widget->minimumSize(),maximum=widget->maximumSize();
  widget->setMinimumSize(px(minimum.width()),px(minimum.height()));
  widget->setMaximumSize(maximum.width()==QWIDGETSIZE_MAX?QWIDGETSIZE_MAX:px(maximum.width()),maximum.height()==QWIDGETSIZE_MAX?QWIDGETSIZE_MAX:px(maximum.height()));
  if(auto *button=qobject_cast<QAbstractButton *>(widget))button->setIconSize(button->iconSize()*scale);
  if(auto *list=qobject_cast<QAbstractItemView *>(widget))list->setIconSize(list->iconSize()*scale);
  if(auto *label=qobject_cast<QLabel *>(widget)){auto pixmap=label->pixmap(Qt::ReturnByValue);if(!pixmap.isNull())label->setPixmap(pixmap.scaled(pixmap.size()*scale,Qt::KeepAspectRatio,Qt::SmoothTransformation));}
  if(auto *table=qobject_cast<QTableWidget *>(widget))table->verticalHeader()->setDefaultSectionSize(px(30));
 }
 for(auto *layout:root->findChildren<QLayout *>()){
  auto m=layout->contentsMargins();layout->setContentsMargins(px(m.left()),px(m.top()),px(m.right()),px(m.bottom()));
  if(auto *grid=qobject_cast<QGridLayout *>(layout)){if(grid->horizontalSpacing()>=0)grid->setHorizontalSpacing(px(grid->horizontalSpacing()));if(grid->verticalSpacing()>=0)grid->setVerticalSpacing(px(grid->verticalSpacing()));}
  else if(auto *form=qobject_cast<QFormLayout *>(layout)){if(form->horizontalSpacing()>=0)form->setHorizontalSpacing(px(form->horizontalSpacing()));if(form->verticalSpacing()>=0)form->setVerticalSpacing(px(form->verticalSpacing()));}
  else if(layout->spacing()>=0)layout->setSpacing(px(layout->spacing()));
  for(int i=0;i<layout->count();++i)if(auto *spacer=layout->itemAt(i)->spacerItem()){auto size=spacer->sizeHint();spacer->changeSize(px(size.width()),px(size.height()),spacer->sizePolicy().horizontalPolicy(),spacer->sizePolicy().verticalPolicy());}
 }
 root->resize(root->size()*scale);
}
void Ui::fadeIn(QWidget *widget,int duration){
 if(!widget)return;
 auto *effect=new QGraphicsOpacityEffect(widget);widget->setGraphicsEffect(effect);
 auto *animation=new QPropertyAnimation(effect,"opacity",widget);
 animation->setDuration(duration);animation->setEasingCurve(QEasingCurve::OutCubic);
 animation->setStartValue(.38);animation->setEndValue(1.);
 animation->start(QAbstractAnimation::DeleteWhenStopped);
 QObject::connect(animation,&QPropertyAnimation::finished,widget,[widget]{widget->setGraphicsEffect(nullptr);});
}
