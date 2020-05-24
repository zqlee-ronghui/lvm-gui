#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <chrono>
#include <QMainWindow>
#include <QTcpSocket>
#include <QTimer>
#include <QDateTime>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/PolygonMesh.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "vtkRenderWindow.h"
#include "proto/message.pb.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
 Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

 private slots:
  void ReadData();
  void ReadError(QAbstractSocket::SocketError);
  void on_btnConn_clicked();
  void CheckHeartBeat();
  void VisualSpin();

 private:
  void initviewer();
  void clearmodel();

  Ui::MainWindow *ui;
  QTcpSocket *tcpClient;
  QTimer *timer;

  bool connected;
  QByteArray buffer;

  lrobot::lidarvolumemeas::Message message;

  std::chrono::system_clock::time_point timestamp;

  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model;
  pcl::PolygonMeshPtr mesh;
};

#endif // MAINWINDOW_H
