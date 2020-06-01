#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    connected(false) {
  ui->setupUi(this);
  this->setWindowTitle ("lvm-gui");

  config_ = new Config(this);
  config_->setModal(false);

  currentTimeLabel = new QLabel;
  ui->statusbar->addWidget(currentTimeLabel);

  statusLabel = new QLabel;
  ui->statusbar->addPermanentWidget(statusLabel);

  tcpClient = new QTcpSocket(this);
  tcpClient->abort();
  timer = new QTimer(this);
  timer->start(300);
  //ui->btnConn->setEnabled(true);

  timestamp = std::chrono::system_clock::now();

  QSettings setting("./config.ini",QSettings::IniFormat);
  ip_ = setting.value("ip").toString();
  port_ = setting.value("port").toString();

  initviewer();

  connect(tcpClient, SIGNAL(readyRead()), this, SLOT(ReadData()));
  connect(tcpClient, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(ReadError(QAbstractSocket::SocketError)));
  connect(timer, SIGNAL(timeout()), this, SLOT(CheckHeartBeat()));
  connect(timer, SIGNAL(timeout()), this, SLOT(VisualSpin()));
  connect(timer, SIGNAL(timeout()), this, SLOT(TimeUpdate()));
  connect(config_, SIGNAL(ConfigSendDataToMainWindow(QString, QString)), this, SLOT(MainWindowReceiveDataFromConfig(QString, QString)));
  connect(this, SIGNAL(MainWindowSendDataToConfig(QString, QString)), config_, SLOT(ConfigReceiveDataFromMainWindow(QString, QString)));

}

MainWindow::~MainWindow() {
  delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event) {
  QSettings setting("./config.ini",QSettings::IniFormat);
  setting.setValue("ip", ip_);
  setting.setValue("port", port_);
}

void MainWindow::TimeUpdate() {
  QDateTime current_time = QDateTime::currentDateTime();
  QString timestr = current_time.toString( "yyyy/MM/dd hh:mm:ss");
  currentTimeLabel->setText(timestr);
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
        switch(message.data_case()) {
          case lrobot::lidarvolumemeas::Message::kInfo:
            statusLabel->setText(QString(lrobot::lidarvolumemeas::State_Name(message.info().state()).c_str()));
            break;
          case lrobot::lidarvolumemeas::Message::kScan:
            clearmodel();
            if(message.scan().points().size() > 0) {
              for(auto p : message.scan().points()) {
                model->points.emplace_back(pcl::PointXYZ{p.x(), p.y(), p.z()});
              }
            }
            model->height = 1;
            model->width = model->points.size();
            model->is_dense = false;
            viewer->updatePointCloud(model, "model");
            ui->qvtkWidget->update();
            break;
          case lrobot::lidarvolumemeas::Message::kResult:
            ui->length->setText(QString::number(message.result().statistic().length()));
            ui->width->setText(QString::number(message.result().statistic().width()));
            ui->height->setText(QString::number(message.result().statistic().height()));
            ui->volume->setText(QString::number(message.result().statistic().volume()));
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
            break;
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
  //ui->btnConn->setText(tr("connect"));
}

void MainWindow::CheckHeartBeat() {
  if(connected) {
    auto tmp = std::chrono::system_clock::now();
    int duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(tmp - timestamp).count();
    if (duration_ms > 2000) {
      statusLabel->setText("CheckHeartBeat Failed.");
    }
  } else {
    statusLabel->setText("offline");
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

void MainWindow::on_actionconfig_triggered() {
  emit MainWindowSendDataToConfig(ip_, port_);
  config_->show();
}

void MainWindow::on_actionconnect_triggered() {
  if(ip_.isEmpty() || port_.isEmpty()) {
    QMessageBox::warning(this,"Warning","Invalid IP address or port.");
  } else {
    if (ui->actionconnect->text() == "connect") {
      tcpClient->connectToHost(ip_, port_.toInt());
      if (tcpClient->waitForConnected(1000)) {
        ui->actionconnect->setText("disconnect");
        connected = true;
      } else {
        QMessageBox::warning(this,"Warning","connect failed.");
      }
    } else {
      tcpClient->disconnectFromHost();
      buffer.clear();
      if(tcpClient->state() == QAbstractSocket::UnconnectedState || tcpClient->waitForDisconnected(1000)) {
        ui->actionconnect->setText("connect");
      }
      connected = false;
    }
  }
}

void MainWindow::MainWindowReceiveDataFromConfig(QString ip, QString port) {
  ip_ = ip;
  port_ = port;
}
