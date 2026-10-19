#pragma once
#include <QtWidgets>
namespace Ui {
int px(int value);
void scaleNativeUi(QWidget *root);
QFrame *card(QString title,QWidget *parent=nullptr);
QPushButton *button(QString text,bool primary=false,QString icon={});
QLabel *text(QString value,QString role="muted");
QWidget *scroll(QWidget *child);
QIcon icon(QString name);
void collapsible(QFrame *card);
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
}
