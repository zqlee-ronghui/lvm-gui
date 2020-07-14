#include "databaseui.h"
#include "ui_databaseui.h"

DatabaseUI::DatabaseUI(QWidget *parent, QSqlTableModel *model) :
    QDialog(parent),
    ui(new Ui::DatabaseUI) {
  this->setWindowTitle("Database");
  ui->setupUi(this);
  mysql_model_.reset(model);
  mysql_model_->select();
  ui->tableView->setModel(mysql_model_.get());
  ui->tableView->setEditTriggers(QAbstractItemView::NoEditTriggers);

  timer = new QTimer(this);
  timer->start(300);
  connect(timer, SIGNAL(timeout()), this, SLOT(VTKSpin()));

  first_visualization = true;
  viewer.reset(new pcl::visualization::PCLVisualizer("database_pointcloud"));
  viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);
}

DatabaseUI::~DatabaseUI() {
  viewer->close();
  delete ui;
}

void DatabaseUI::on_tableView_doubleClicked(const QModelIndex &index)
{
  if(index.column() == 5) {
    auto content = index.data().toString();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(content.toStdString(), *cloud) == -1) {
      PCL_ERROR("Couldn't read %s\n", content.toStdString().c_str());
      return;
    }
    if(first_visualization) {
      viewer->addPointCloud(cloud, "model");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model");
      first_visualization = false;
    } else {
      viewer->updatePointCloud(cloud, "model");
    }
    return;
  }
}

void DatabaseUI::on_first_one_btn_clicked()
{
  if(mysql_model_->rowCount() > 0) {
    ui->tableView->selectRow(0);
  }
}

void DatabaseUI::on_pre_one_btn_clicked()
{
  if(ui->tableView->currentIndex().row() > 0 && mysql_model_->rowCount() > 1) {
    ui->tableView->selectRow(ui->tableView->currentIndex().row() - 1);
  }
}

void DatabaseUI::on_all_btn_clicked()
{
  mysql_model_->setTable("course");
  mysql_model_->select();
}

void DatabaseUI::on_next_one_btn_clicked()
{
  if(mysql_model_->rowCount() > 1 && mysql_model_->rowCount() - 1 > ui->tableView->currentIndex().row()) {
    ui->tableView->selectRow(ui->tableView->currentIndex().row() + 1);
  }
}

void DatabaseUI::on_last_one_btn_clicked()
{
  if(mysql_model_->rowCount() > 0) {
    ui->tableView->selectRow(mysql_model_->rowCount() - 1);
  }
}

void DatabaseUI::VTKSpin() {
  if(!first_visualization && !viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
