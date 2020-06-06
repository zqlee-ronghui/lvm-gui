#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <chrono>
#include <QDateTime>
#include <QMainWindow>
#include <QMessageBox>
#include <QLabel>
#include <QString>
#include <QSettings>
#include <QTcpSocket>
#include <QTimer>
#include <QDateTime>

#include "mysql/mysql.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/PolygonMesh.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "vtkRenderWindow.h"
#include "proto/message.pb.h"
#include "config.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
 Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

 private slots:
  void TimeUpdate();
  void ReadData();
  void ReadError(QAbstractSocket::SocketError);
//  void on_btnConn_clicked();
  void CheckHeartBeat();
  void VisualSpin();

  void on_actionconfig_triggered();
  void on_actionconnect_triggered();
  void MainWindowReceiveDataFromConfig(QString ip, QString port);

  signals:
  void MainWindowSendDataToConfig(QString, QString);

 private:
  void closeEvent(QCloseEvent *event);
  void initviewer();
  void clearmodel();

  Ui::MainWindow *ui;
  QDialog *config_;
  QTcpSocket *tcpClient;
  QTimer *timer;
  QLabel *currentTimeLabel;
  QLabel *statusLabel;
  QString ip_, port_;

  bool connected;
  QByteArray buffer;

  lrobot::lidarvolumemeas::Message message;

  std::chrono::system_clock::time_point timestamp;

  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model;
  pcl::PolygonMeshPtr mesh;

  std::unique_ptr<MYSQL> db_;
};

#endif // MAINWINDOW_H
