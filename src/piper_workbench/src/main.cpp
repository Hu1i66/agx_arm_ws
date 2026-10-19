#include <QApplication>
#include <QTimer>
#include <QScreen>
#include <QWindow>
#include "piper_workbench/main_window.hpp"
#include <rclcpp/rclcpp.hpp>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Keep Ogre's X11 drawable and Qt window in the same native pixel units.
  qputenv("QT_ENABLE_HIGHDPI_SCALING","0");qunsetenv("QT_SCALE_FACTOR");qunsetenv("QT_SCREEN_SCALE_FACTORS");
  QApplication::setAttribute(Qt::AA_DisableHighDpiScaling);
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  QApplication app(argc, argv);
  app.setProperty("workbenchUiScale",qBound(1.,app.primaryScreen()->logicalDotsPerInch()/96.,3.));
  QApplication::setDesktopFileName("piper-workbench");
  QApplication::setWindowIcon(QIcon(QString::fromLocal8Bit(qgetenv("AGX_ARM_WS"))+"/src/piper_workbench/resources/icons/robot.svg"));
  qRegisterMetaType<QVector<double>>("QVector<double>");
  auto window = std::make_unique<MainWindow>(); window->show();
  auto args = app.arguments();
  if(args.contains("--size")){auto size=args.value(args.indexOf("--size")+1).split("x");if(size.size()==2)window->resize(Ui::px(size[0].toInt()),Ui::px(size[1].toInt()));}
  if(args.contains("--page"))window->showPage(args.value(args.indexOf("--page")+1).toInt());
  QString capturePath=args.contains("--screenshot")?args.value(args.indexOf("--screenshot")+1):QString::fromLocal8Bit(qgetenv("PIPER_WORKBENCH_SCREENSHOT"));
  if (!capturePath.isEmpty()) {
    QString path=capturePath;
    QTimer::singleShot(6500, window.get(), [&window, path] {
      window->raise();window->activateWindow();QApplication::processEvents();
      QTimer::singleShot(250,window.get(),[&window,path]{window->windowHandle()->screen()->grabWindow(window->winId()).save(path);
      QFile report(path+".json");if(report.open(QFile::WriteOnly))report.write(QJsonDocument(window->inspect()).toJson(QJsonDocument::Indented));});
    });
    QTimer::singleShot(7500, &app, &QApplication::quit);
  }
  QTimer rosShutdown;QObject::connect(&rosShutdown,&QTimer::timeout,&app,[&app]{if(!rclcpp::ok())app.quit();});rosShutdown.start(200);
  int result = app.exec(); window.reset(); rclcpp::shutdown(); return result;
}
