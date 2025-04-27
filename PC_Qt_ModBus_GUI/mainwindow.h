
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QTimer>
#include <QtCharts>
#include <QPushButton>
#include <QTableWidget>
#include <QtCharts/QChartView>
#include <QVBoxLayout>
#include <QHBoxLayout>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

QT_CHARTS_USE_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void sendModbusRequest();
    void readSerialData();
    void sendRelayCommand(bool on);

private:
    Ui::MainWindow *ui;
    QSerialPort *serial;
    QTimer *timer;
    QChart *chart;
    QLineSeries *series;

    QPushButton *btnRelayOn;
    QPushButton *btnRelayOff;
    QTableWidget *tableWidget;
    QChartView *chartView;

    void parseModbusResponse(const QByteArray &data);
    uint16_t calculateCRC(const QByteArray &data);
    void setupChart();
    void setupUi();
};

#endif // MAINWINDOW_H
