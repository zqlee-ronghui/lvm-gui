#ifndef CONFIG_H
#define CONFIG_H

#include <QDialog>

namespace Ui {
class Config;
}

class Config : public QDialog
{
    Q_OBJECT

public:
    explicit Config(QWidget *parent = nullptr);
    ~Config();

private slots:
    void on_Config_accepted();
    void ConfigReceiveDataFromMainWindow(QString, QString);

signals:
    void ConfigSendDataToMainWindow(QString, QString);

private:
    Ui::Config *ui;
};

#endif // CONFIG_H
