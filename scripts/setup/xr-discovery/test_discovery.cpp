// Native regression test: broadcasts stay on loopback and TCP stays connected.
#include <QCoreApplication>
#include <QSettings>
#include <QTcpSocket>
#include <QTemporaryDir>
#include <QElapsedTimer>
#include <iostream>
#include "Manage/TCPConnectionManage/tcpserverworker.h"
#include "utils/utils.h"

extern "C" int quest_xr_discovery_protocol();
// ELF interposition limits the unmodified upstream sender to loopback in this test.
QStringList DevConnSDK::Utils::getLocalIPv4AddressList() { return {"127.0.0.1"}; }

static void require(bool ok, const char* text) {
    if (!ok) { std::cerr << text << std::endl; std::exit(1); }
}

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    QTemporaryDir folder;
    require(folder.isValid(), "temporary directory");
    const QString path = folder.path() + "/discovery.ini";
    qputenv("QUEST_XR_DISCOVERY_CONFIG", path.toUtf8());
    require(quest_xr_discovery_protocol() == 1, "patched library must be loaded");
    QUdpSocket receiver;
    require(receiver.bind(QHostAddress::AnyIPv4, 29888, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint), "UDP bind");
    DevConnSDK::TcpServerWorker worker;
    worker.m_broadCastUdp = new QUdpSocket();
    worker.m_tcpServer = new QTcpServer();
    require(worker.m_tcpServer->listen(QHostAddress::LocalHost, 0), "TCP listen");
    QTcpSocket client;
    client.connectToHost(QHostAddress::LocalHost, worker.m_tcpServer->serverPort());
    require(client.waitForConnected(1000), "TCP connect");
    require(worker.m_tcpServer->waitForNewConnection(1000), "TCP accept");
    QTcpSocket* peer = worker.m_tcpServer->nextPendingConnection();
    auto received = [&]() {
        bool found = false;
        QElapsedTimer timer;
        timer.start();
        while (timer.elapsed() < 200) {
            if (!receiver.hasPendingDatagrams()) receiver.waitForReadyRead(20);
            while (receiver.hasPendingDatagrams()) {
                QByteArray data(2048, '\0');
                QHostAddress sender;
                auto size = receiver.readDatagram(data.data(), data.size(), &sender);
                data.resize(size);
                if (sender.isLoopback() && data.contains("127.0.0.1")) found = true;
            }
        }
        return found;
    };
    auto check = [&](bool expected) {
        worker.onBraodCastTCPServer();
        require(received() == expected, "discovery packet mismatch");
        QSettings status(path + ".status", QSettings::IniFormat);
        status.sync();
        require(status.value("TCP/BroadcastEnabled").toBool() == expected, "status mismatch");
        require(client.state() == QAbstractSocket::ConnectedState, "TCP was disconnected");
        client.write("tracking");
        client.waitForBytesWritten(1000);
        require(peer->waitForReadyRead(1000), "TCP data interrupted");
        require(peer->readAll() == "tracking", "TCP payload changed");
    };
    check(false); // No config: default off.
    {
        QSettings config(path, QSettings::IniFormat);
        config.setValue("TCP/BroadcastEnabled", true);
        config.sync();
    }
    check(true);
    {
        QSettings config(path, QSettings::IniFormat);
        config.setValue("TCP/BroadcastEnabled", false);
        config.sync();
    }
    check(false);
    worker.release();
    std::cout << "Native discovery test passed: default off, on/off packets, TCP continuity." << std::endl;
}
