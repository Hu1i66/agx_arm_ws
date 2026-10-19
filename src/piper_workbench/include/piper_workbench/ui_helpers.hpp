#pragma once
#include <QtWidgets>
#include <QVariantAnimation>
#include <QJsonArray>
namespace Ui {
int px(int value);
void scaleNativeUi(QWidget *root);
QFrame *card(QString title,QWidget *parent=nullptr);
QPushButton *button(QString text,bool primary=false,QString icon={});
QLabel *text(QString value,QString role="muted");
QWidget *scroll(QWidget *child);
QIcon icon(QString name);
void collapsible(QFrame *card);
void fadeIn(QWidget *widget,int duration=220);
class TracePlot:public QWidget{
public:
 explicit TracePlot(QWidget *parent=nullptr):QWidget(parent){setMinimumHeight(230);}
 QJsonObject report;int metric=0;
 void paintEvent(QPaintEvent *)override;
};
class ImageView:public QWidget{
public:
 explicit ImageView(QWidget *parent=nullptr):QWidget(parent){setMinimumSize(160,120);setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);}
 double ageSeconds=-1;
 QImage frame;QString waiting="等待相机图像";QJsonObject detection;int selected=-1;
 std::function<void(int)> picked;
 void paintEvent(QPaintEvent *)override;
 void mousePressEvent(QMouseEvent *)override;
 QRect imageRect()const;
};
class GraspFlow:public QWidget{
public:
 explicit GraspFlow(QWidget *parent=nullptr);
 void setStage(QString stage,QString detail={});
 void setActive(bool active);
 void paintEvent(QPaintEvent *)override;
private:
 QVariantAnimation animation_;QTimer pulseTimer_;QElapsedTimer pulseClock_;
 QString stage_="waiting",detail_="等待开始";qreal displayPos_=0.,pulse_=0.;bool active_=false;
};
class RegionEditor:public QWidget{
public:
 explicit RegionEditor(QWidget *parent=nullptr);
 void setFrame(const QImage &frame);
 void setLines(int trigger,int stop,int abandon);
 void setPolygon(const QJsonArray &polygon);
 void setMode(int mode);
 QJsonArray polygon()const{return polygon_;}
 std::function<void(int,int)> lineMoved;
 std::function<void(const QJsonArray &)> polygonMoved;
protected:
 void paintEvent(QPaintEvent *)override;
 void mousePressEvent(QMouseEvent *)override;
 void mouseMoveEvent(QMouseEvent *)override;
 void mouseReleaseEvent(QMouseEvent *)override;
private:
 QRectF imageRect()const;bool imagePoint(QPoint position,QPointF &point)const;
 QImage frame_;QJsonArray polygon_;int lines_[3]={130,490,510};int mode_=0,dragLine_=-1,dragVertex_=-1;
};
}
