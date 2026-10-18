#include <QApplication>
#include <QTimer>
#include <QScreen>
#include <QWindow>
#include "piper_workbench/main_window.hpp"
#include <rclcpp/rclcpp.hpp>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  QApplication app(argc, argv);
  QApplication::setDesktopFileName("piper-workbench");
  QApplication::setWindowIcon(QIcon(QString::fromLocal8Bit(qgetenv("AGX_ARM_WS"))+"/src/piper_workbench/resources/icons/robot.svg"));
  qRegisterMetaType<QVector<double>>("QVector<double>");
  auto window = std::make_unique<MainWindow>(); window->show();
  auto args = app.arguments();
  if(args.contains("--size")){auto size=args.value(args.indexOf("--size")+1).split("x");if(size.size()==2)window->resize(size[0].toInt(),size[1].toInt());}
  if(args.contains("--page"))window->showPage(args.value(args.indexOf("--page")+1).toInt());
  if (args.contains("--screenshot")) {
    QString path = args.value(args.indexOf("--screenshot") + 1);
    QTimer::singleShot(6500, window.get(), [&window, path] {
      window->raise();window->activateWindow();QApplication::processEvents();
      QTimer::singleShot(250,window.get(),[&window,path]{window->windowHandle()->screen()->grabWindow(window->winId()).save(path);
      QFile report(path+".json");if(report.open(QFile::WriteOnly))report.write(QJsonDocument(window->inspect()).toJson(QJsonDocument::Indented));});
    });
    QTimer::singleShot(7500, &app, &QApplication::quit);
  }
  int result = app.exec(); window.reset(); rclcpp::shutdown(); return result;
}
