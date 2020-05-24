#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    connected(false) {
  ui->setupUi(this);
  this->setWindowTitle ("lvm-gui");
  tcpClient = new QTcpSocket(this);
  tcpClient->abort();
  timer = new QTimer(this);
  ui->btnConn->setEnabled(true);

  timestamp = std::chrono::system_clock::now();

  initviewer();

  connect(tcpClient, SIGNAL(readyRead()), this, SLOT(ReadData()));
  connect(tcpClient, SIGNAL(error(QAbstractSocket::SocketError)), \
            this, SLOT(ReadError(QAbstractSocket::SocketError)));
  connect(timer, SIGNAL(timeout()), this, SLOT(CheckHeartBeat()));
  connect(timer, SIGNAL(timeout()), this, SLOT(VisualSpin()));
}

MainWindow::~MainWindow() {
  delete ui;
}

void MainWindow::on_btnConn_clicked() {
  if (ui->btnConn->text() == "connect") {
    tcpClient->connectToHost(ui->edtIP->text(), ui->edtPort->text().toInt());
    if (tcpClient->waitForConnected(1000)) {
      ui->btnConn->setText("disconnect");
      ui->edtIP->setDisabled(true);
      ui->edtPort->setDisabled(true);
    }
    timer->start(300);
  } else {
    tcpClient->disconnectFromHost();
    buffer.clear();
    if(tcpClient->state() == QAbstractSocket::UnconnectedState || tcpClient->waitForDisconnected(1000)) {
      ui->btnConn->setText("connect");
      ui->edtIP->setEnabled(true);
      ui->edtPort->setEnabled(true);
      ui->temperature->setText("");
      ui->status->setText("");
    }
    timer->stop();
  }
}

void MainWindow::ReadData() {
  QByteArray tmp = tcpClient->readAll();
  if (!tmp.isEmpty()) {
    timestamp = std::chrono::system_clock::now();
    buffer.append(tmp);
    while(buffer.size() >= 4) {
      uint32_t msg_length = (uint8_t)(buffer.at(0)) << 24 | (uint8_t)(buffer.at(1)) << 16 | (uint8_t)(buffer.at(2)) << 8 | (uint8_t)(buffer.at(3));
      if(buffer.size() >= 4 + msg_length) {
        message.Clear();
        message.ParseFromArray(buffer.data() + 4, msg_length);
        if(message.has_info()) {
          ui->temperature->setText(QString::number(message.info().temperature()));
          ui->status->setText(QString(lrobot::lidarvolumemeas::State_Name(message.info().state()).c_str()));
        } else if (message.has_result()) {
          ui->length->setText(QString::number(message.result().statistic().length()));
          ui->width->setText(QString::number(message.result().statistic().width()));
          ui->height->setText(QString::number(message.result().statistic().height()));
          ui->volume->setText(QString::number(message.result().statistic().volume()));
          ui->timestamp->setText(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));

          clearmodel();

          if(message.result().model().points().size() > 0) {
            for(auto p : message.result().model().points()) {
              model->points.emplace_back(pcl::PointXYZ{p.x(), p.y(), p.z()});
            }
          } else {
            model->points.resize(2000);
            for (size_t i = 0; i < 2000; ++i)
            {
              model->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
              model->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
              model->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
            }
          }
          model->height = 1;
          model->width = model->points.size();
          model->is_dense = false;
          viewer->updatePointCloud(model, "model");
          ui->qvtkWidget->update();
        }
        buffer.remove(0, 4 + msg_length);
      } else {
        break;
      }
    }
  }
}

void MainWindow::ReadError(QAbstractSocket::SocketError) {
  tcpClient->disconnectFromHost();
  ui->btnConn->setText(tr("connect"));
}

void MainWindow::CheckHeartBeat() {
  auto tmp = std::chrono::system_clock::now();
  int duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(tmp - timestamp).count();
  if (duration_ms > 2000) {
    ui->temperature->setStyleSheet("QLabel{background:#FF6666;}");
    ui->status->setStyleSheet("QLabel{background:#FF6666;}");
  } else {
    ui->temperature->setStyleSheet("QLabel{background:#009999;}");
    ui->status->setStyleSheet("QLabel{background:#009999;}");
  }
}

void MainWindow::VisualSpin() {
  if (model->points.size() > 0) {
    viewer->updatePointCloud(model, "model");
  }
  if (mesh->polygons.size() > 0) {
    viewer->updatePolygonMesh(*mesh, "mesh");
  }
  ui->qvtkWidget->update();
}

void MainWindow::initviewer() {
  viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
//  viewer->addCoordinateSystem(1.0, "model", 0);

  model.reset(new pcl::PointCloud<pcl::PointXYZ>);
  mesh.reset(new pcl::PolygonMesh);
  model->points.emplace_back(pcl::PointXYZ{0, 0, 0});
  model->points.emplace_back(pcl::PointXYZ{0, 0, 0});
  model->points.emplace_back(pcl::PointXYZ{0, 0, 0});

  pcl::toPCLPointCloud2(*model, mesh->cloud);
  pcl::Vertices v;
  v.vertices.emplace_back(0);
  v.vertices.emplace_back(1);
  v.vertices.emplace_back(2);
  mesh->polygons.emplace_back(v);

  viewer->addPointCloud(model, "model");
  viewer->addPolygonMesh(*mesh, "mesh");
  viewer->resetCamera();
  ui->qvtkWidget->update();
  clearmodel();
}

void MainWindow::clearmodel() {
  model->clear();
  mesh->cloud.width = 0;
  mesh->cloud.height = 1;
  mesh->cloud.data.clear();
  mesh->polygons.clear();
}
