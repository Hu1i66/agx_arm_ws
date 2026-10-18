#include "piper_workbench/ui_helpers.hpp"
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
  auto glyph=icon(symbol).pixmap(20,20);if(primary){QPainter paint(&glyph);paint.setCompositionMode(QPainter::CompositionMode_SourceIn);paint.fillRect(glyph.rect(),Qt::white);}b->setIcon(QIcon(glyph));
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
  icon("capture").paint(&p,QRect(width()/2-20,height()/2-52,40,40));p.setPen(QColor("#64748B"));p.drawText(rect().adjusted(0,20,0,20),Qt::AlignCenter,waiting);return;
 }
 p.drawImage(box,frame);if(selected<0)return;
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
