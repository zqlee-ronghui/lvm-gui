#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <chrono>
#include <string>
#include <vector>

#include <QDateTime>
#include <QMainWindow>
#include <QMessageBox>
#include <QLabel>
#include <QString>
#include <QSettings>
#include <QTcpSocket>
#include <QTimer>
#include <QDateTime>
#include <QFileDialog>
#include <QSqlTableModel>
#include <QSqlRecord>

#include "mysql/mysql.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/PolygonMesh.h"
#include "pcl/io/ply_io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "vtkRenderWindow.h"
#include "proto/message.pb.h"
#include "config.h"
#include "databaseui.h"
#include "about.h"

#include "common/base64.h"
#include "nlohmann/json.hpp"

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
  void on_actionscan_view_triggered();
  void on_actionmodel_view_triggered();
  void on_actionabout_triggered();
  void on_actionsave_model_triggered();
  void on_actionopen_triggered();
  void MainWindowReceiveDataFromConfig(QString ip, QString port);

  signals:
  void MainWindowSendDataToConfig(QString, QString);

 private:
  void closeEvent(QCloseEvent *event);
  void initviewer();
  void clearmodel();

  Ui::MainWindow *ui;
  QDialog *config_;
  QDialog *database_ui_;
  QDialog *about_;
  QTcpSocket *tcpClient;
  QTimer *timer;
  QLabel *currentTimeLabel;
  QLabel *statusLabel;
  QString ip_, port_;

  bool connected;
  QByteArray buffer;

  lrobot::lidarvolumemeas::Message message;

  std::vector<std::pair<uint64_t, std::string>> scans_;

  std::chrono::system_clock::time_point timestamp;

  pcl::visualization::PCLVisualizer::Ptr viewer;

  pcl::PointCloud<pcl::PointXYZ>::Ptr model;
  pcl::PolygonMeshPtr mesh;

  std::shared_ptr<QSqlTableModel> mysql_model_;
  int save_index;

  QString dir_;
};

#endif // MAINWINDOW_H
