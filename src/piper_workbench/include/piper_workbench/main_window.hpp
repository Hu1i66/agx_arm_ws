#pragma once
#include <QtWidgets>
#include <QJsonObject>
#include "piper_workbench/ui_helpers.hpp"
class RvizPanel;class BackendClient;class RosViewBridge;class MotionController;
class MainWindow:public QMainWindow{
 Q_OBJECT
public:
 explicit MainWindow(QWidget *parent=nullptr);
 ~MainWindow() override;
 void showPage(int index);
 QJsonObject inspect() const;
protected:void closeEvent(QCloseEvent *)override;bool eventFilter(QObject *,QEvent *)override;
private:
 QWidget *makeMain();QWidget *makeDebug();QWidget *makeConnections();QWidget *makeCalibration();QWidget *makeCapture();QWidget *makeLogs();
 QWidget *makeMotion();QWidget *makeParameters();QWidget *makeCameraSettings();QWidget *makeFiles();QWidget *makeTeach();QWidget *makeAdvanced();
 void refresh();void receive(QString event,QJsonValue data);void showError(QString message);void updateTargets();void applyStatus(QJsonObject);
 void refreshFiles();void refreshPlacements();void savePlacements();void setGoal(QVector<double>);void syncGoal();void moveRviz();
 void startCalibration(QString mode={});void sendCalibration(QString action);void refreshParameters();void runTask(bool continuous);
 void logRow(QJsonObject);void filterLogs();void showFile(QString path);QString camera()const;
 BackendClient *backend_;RosViewBridge *bridge_;MotionController *motion_;RvizPanel *rviz_;
 QStackedWidget *pages_;QListWidget *nav_;QTabWidget *debugTabs_;QWidget *mainHost_,*motionHost_;QLabel *toast_,*health_,*jointFeedback_,*taskState_,*motionState_,*connectionState_,*calibrationState_,*captureState_;
 QComboBox *mode_,*plan_,*placementMode_,*calibrationKind_,*calibrationCamera_,*fileKind_,*fileCamera_;
 QTableWidget *targets_,*files_,*sampleTable_,*logTable_,*teachTable_;
 QLabel *resultSummary_,*logMetadata_;
 QComboBox *logTime_;QListWidget *logSources_;
 QPlainTextEdit *connectionLog_,*calibrationLog_,*resultText_,*logDetail_,*diagnosticText_;
 QLineEdit *logSearch_,*planName_;QComboBox *logSource_,*logLevel_,*teachPlan_;QCheckBox *logPause_;
 QDoubleSpinBox *velocity_,*acceleration_,*captureInterval_,*verifyAngle_,*verifySpeed_;QSpinBox *verifySteps_;
 QVector<QCheckBox *> verifyAxes_;
 QCheckBox *captureD455_,*captureDabai_;QListWidget *captureImages_,*diagnostics_;
 QMap<QString,Ui::ImageView *> images_;QVector<QDoubleSpinBox *> joints_,tcp_;
 QJsonObject status_,placements_,teachDraft_;QMap<QString,QJsonObject> detections_;QJsonArray logRows_,calibrationFiles_;QJsonObject parameterGroups_;
 QMap<QString,QMap<QString,QDoubleSpinBox *>> parameterFields_;QMap<QString,QLabel *> componentStates_,healthBadges_;
 bool logsDirty_=false;
 QString calibrationSession_,lastBackup_,lastExport_,selectedSession_;QJsonArray affected_;int selectedTrack_=-1;bool syncing_=false;QVector<double> lastActual_,observedGoal_;
 QTimer *refreshTimer_;
};
