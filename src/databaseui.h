#ifndef DATABASEUI_H
#define DATABASEUI_H
#include <memory>
#include <QDialog>
#include <QSqlTableModel>
#include <QMessageBox>
#include <QSlider>
#include <QTimer>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"

namespace Ui {
class DatabaseUI;
}

class DatabaseUI : public QDialog {
 Q_OBJECT

 public:
  explicit DatabaseUI(QWidget *parent, QSqlTableModel *model);
  ~DatabaseUI();
 private slots:
  void on_tableView_doubleClicked(const QModelIndex &index);
  void on_first_one_btn_clicked();
  void on_pre_one_btn_clicked();
  void on_all_btn_clicked();
  void on_next_one_btn_clicked();
  void on_last_one_btn_clicked();
  void VTKSpin();
 private:
  Ui::DatabaseUI *ui;
  QTimer *timer;
  std::shared_ptr<QSqlTableModel> mysql_model_;
  bool first_visualization;
  pcl::visualization::PCLVisualizer::Ptr viewer;
};

#endif // DATABASEUI_H
