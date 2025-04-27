
#include "mainwindow.h"
#include <QMessageBox>
#include <QSerialPortInfo>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setupUi();

    serial = new QSerialPort(this);
    timer = new QTimer(this);

    serial->setPortName("COM5");
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    if (!serial->open(QIODevice::ReadWrite)) {
        QMessageBox::critical(this, "Error", "Serial Port Open Failed!");
    }

    connect(timer, &QTimer::timeout, this, &MainWindow::sendModbusRequest);
    connect(serial, &QSerialPort::readyRead, this, &MainWindow::readSerialData);
    connect(btnRelayOn, &QPushButton::clicked, [=]() { sendRelayCommand(true); });
    connect(btnRelayOff, &QPushButton::clicked, [=]() { sendRelayCommand(false); });

    timer->start(500);
}

MainWindow::~MainWindow()
{
}

void MainWindow::setupUi()
{
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    btnRelayOn = new QPushButton("Relay ON", this);
    btnRelayOff = new QPushButton("Relay OFF", this);

    tableWidget = new QTableWidget(10, 2, this);
    tableWidget->setHorizontalHeaderLabels(QStringList() << "Channel" << "Value");
    for (int i = 0; i < 10; ++i) {
        tableWidget->setItem(i, 0, new QTableWidgetItem(QString("CH%1").arg(i)));
    }

    chart = new QChart();
    series = new QLineSeries();
    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->setTitle("ADC Channel 0 Real-Time Plot");

    chartView = new QChartView(chart, this);
    chartView->setRenderHint(QPainter::Antialiasing);

    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(btnRelayOn);
    buttonLayout->addWidget(btnRelayOff);

    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
    mainLayout->addLayout(buttonLayout);
    mainLayout->addWidget(tableWidget);
    mainLayout->addWidget(chartView);
}

void MainWindow::sendModbusRequest()
{
    QByteArray request;
    request.append(0x01);
    request.append(0x03);
    request.append(0x00);
    request.append(0x00);
    request.append(0x00);
    request.append(0x0A);

    uint16_t crc = calculateCRC(request);
    request.append(crc & 0xFF);
    request.append((crc >> 8) & 0xFF);

    serial->write(request);
}

void MainWindow::sendRelayCommand(bool on)
{
    QByteArray command;
    command.append(0x01);
    command.append(0x06);
    command.append(0x00);
    command.append(0x01);
    if (on) {
        command.append(0x00);
        command.append(0x01);
    } else {
        command.append(0x00);
        command.append(0x00);
    }

    uint16_t crc = calculateCRC(command);
    command.append(crc & 0xFF);
    command.append((crc >> 8) & 0xFF);

    serial->write(command);
}

void MainWindow::readSerialData()
{
    QByteArray data = serial->readAll();
    parseModbusResponse(data);
}

void MainWindow::parseModbusResponse(const QByteArray &data)
{
    if (data.size() < 5) return;

    int byteCount = (uint8_t)data[2];
    if (data.size() < 3 + byteCount + 2) return;

    QVector<QPointF> points;

    for (int i = 0; i < byteCount / 2; ++i) {
        uint16_t value = (uint8_t)data[3 + i*2] << 8 | (uint8_t)data[4 + i*2];
        tableWidget->setItem(i, 1, new QTableWidgetItem(QString::number(value)));
        if (i == 0) {
            points.append(QPointF(series->count(), value));
        }
    }

    if (!points.isEmpty()) {
        series->append(points);
        if (series->count() > 100) {
            series->remove(0);
        }
    }
}

uint16_t MainWindow::calculateCRC(const QByteArray &data)
{
    uint16_t crc = 0xFFFF;
    for (char b : data) {
        crc ^= (uint8_t)b;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
