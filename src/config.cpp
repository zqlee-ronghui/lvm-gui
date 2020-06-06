#include "config.h"
#include "ui_config.h"

Config::Config(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Config)
{
    ui->setupUi(this);
}

Config::~Config()
{
    delete ui;
}

void Config::on_Config_accepted()
{
  emit ConfigSendDataToMainWindow(ui->ipEdit->text(), ui->portEdit->text());
}

void Config::ConfigReceiveDataFromMainWindow(QString ip, QString port) {
  ui->ipEdit->setText(ip);
  ui->portEdit->setText(port);
}
