#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    connected(false) {
  ui->setupUi(this);
  this->setWindowTitle ("lvm-gui");
  save_index = 0;
//  db_.reset(new MYSQL());
//  mysql_init(db_.get());
//  if(mysql_real_connect(db_.get(), "localhost","ronghui","mysql-ronghui","lvm_db",0, NULL, CLIENT_FOUND_ROWS)) {
//    std::cout << "connect mysql success." << std::endl;
//  }

  config_ = new Config(this);
  config_->setModal(false);

  about_ = new About(this);
  about_->setModal(false);

  currentTimeLabel = new QLabel;
  ui->statusbar->addWidget(currentTimeLabel);

  statusLabel = new QLabel;
  ui->statusbar->addPermanentWidget(statusLabel);
  ui->statusbar->setSizeGripEnabled(false);

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
          case lrobot::lidarvolumemeas::Message::kInfo: {
            statusLabel->setText(QString(lrobot::lidarvolumemeas::State_Name(message.info().state()).c_str()));
            break;
          } case lrobot::lidarvolumemeas::Message::kScan: {
            ui->id->setText(QString::number(message.scan().statistic().velocity()));
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
          } case lrobot::lidarvolumemeas::Message::kResult: {
            ui->id->setText(QString::number(message.result().statistic().velocity()));
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
    if (duration_ms > 5000) {
      statusLabel->setText("CheckHeartBeat Failed.");
      tcpClient->disconnectFromHost();
      buffer.clear();
      if(tcpClient->state() == QAbstractSocket::UnconnectedState || tcpClient->waitForDisconnected(1000)) {
        ui->actionconnect->setText("connect");
      }
      connected = false;
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
//  auto pose = viewer->getViewerPose();
//  std::cout << "==================" << std::endl;
//  std::cout << "viewer pose: \n" << pose.matrix() << std::endl;
//
//  pcl::visualization::Camera camera;
//  viewer->getCameraParameters(camera);
//  std::cout << "camera fovy: " << camera.fovy << std::endl;
//  std::cout << "camera wind: " << camera.window_size[0] << ", " << camera.window_size[1] << std::endl;
//  std::cout << "ui size: " << ui->qvtkWidget->size().width() << ", " << ui->qvtkWidget->size().height() << std::endl;

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
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model");
  viewer->addPolygonMesh(*mesh, "mesh");

  for(float y = -3; y <= 3; y += 1.0) {
    viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ(-2, y, 6), pcl::PointXYZ(10, y, 6), "yline" + std::to_string(y));
  }
  for(float x = -2; x <= 10; x += 1.0) {
    viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ(x, -3, 6), pcl::PointXYZ(x, 3, 6), "xline" + std::to_string(x));
  }
//  pcl::ModelCoefficients plane_coeff;
//  plane_coeff.values.resize (4);    // We need 4 values
//  plane_coeff.values[0] = 0;
//  plane_coeff.values[1] = 0;
//  plane_coeff.values[2] = 1;
//  plane_coeff.values[3] = -6;
//  viewer->addPlane(plane_coeff, "ground");

  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 1, 0, 0, 0);
  viewer->setBackgroundColor(164.0 / 255.0, 164.0 / 255.0, 164.0 / 255.0);
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
        timestamp = std::chrono::system_clock::now();
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

void MainWindow::on_actionscan_view_triggered() {
  float fovy = 0.8575f;
  Eigen::Matrix3f intrinsics(Eigen::Matrix3f::Identity());
  intrinsics(0, 2) = ui->qvtkWidget->width() * 0.5f;
  intrinsics(1, 2) = ui->qvtkWidget->height() * 0.5f;
  intrinsics(1, 1) = intrinsics(1, 2) / fovy * 2.0f;

  Eigen::Matrix4f extrinsics(Eigen::Matrix4f::Identity());
  extrinsics << 0.00711892,     0.10875,   -0.994044,     5.21231,
      0.999962, -0.00572975,  0.00653447,   0.0659031,
      -0.004985,   -0.994053,   -0.108786,     4.50848,
      0,           0,           0,           1;
  viewer->setCameraParameters(intrinsics, extrinsics);
}

void MainWindow::on_actionmodel_view_triggered() {
  float fovy = 0.81f;
  Eigen::Matrix3f intrinsics(Eigen::Matrix3f::Identity());
  intrinsics(0, 2) = ui->qvtkWidget->width() * 0.5f;
  intrinsics(1, 2) = ui->qvtkWidget->height() * 0.5f;
  intrinsics(1, 1) = intrinsics(1, 2) / fovy * 2.0f;

  Eigen::Matrix4f extrinsics(Eigen::Matrix4f::Identity());
  extrinsics <<  0.755122, -0.236257, -0.611534,   11.9141,
      0.65472,  0.319672,  0.684947,  -7.88965,
      0.0336673, -0.917601,  0.396073,  0.178935,
      0,         0,         0,         1;
  viewer->setCameraParameters(intrinsics, extrinsics);
}

void MainWindow::on_actionabout_triggered() {
  about_->show();
}

void MainWindow::on_actionsave_model_triggered() {
  if(model->size() > 0) {
    QString filename = QFileDialog::getSaveFileName(this, tr("Open point cloud"), QString("model_%1.pcd").arg(save_index++), tr("Point cloud data (*.pcd *.ply)"));

    if(filename.isEmpty()) {
      return;
    }

    int return_status;
    if(filename.endsWith(".pcd", Qt::CaseInsensitive)) {
      return_status = pcl::io::savePCDFileBinary (filename.toStdString(), *model);
    } else if(filename.endsWith(".ply", Qt::CaseInsensitive)) {
      return_status = pcl::io::savePLYFileBinary (filename.toStdString(), *model);
    } else {
      filename.append(".ply");
      return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *model);
    }

    if (return_status != 0) {
      QMessageBox::warning(this,"Warning","Save model failed.");
      return;
    }
  } else {
    QMessageBox::warning(this,"Warning","model is empty.");
    return;
  }

}

void MainWindow::MainWindowReceiveDataFromConfig(QString ip, QString port) {
  ip_ = ip;
  port_ = port;
}
