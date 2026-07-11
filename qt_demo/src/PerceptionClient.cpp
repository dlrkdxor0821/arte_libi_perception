#include "PerceptionClient.h"
#include "FrameImageProvider.h"
#include <QImage>
#include <QTimer>
#include <QDebug>

PerceptionClient::PerceptionClient(FrameImageProvider *provider, QObject *parent)
    : QObject(parent), m_provider(provider) {
    connect(&m_sock, &QTcpSocket::readyRead, this, &PerceptionClient::onReadyRead);
    connect(&m_sock, &QTcpSocket::connected, this, &PerceptionClient::onConnected);
    connect(&m_sock, &QTcpSocket::disconnected, this, &PerceptionClient::onDisconnected);
    // server not up yet (connection refused while it loads the model) -> keep retrying
    connect(&m_sock, &QTcpSocket::errorOccurred, this,
            [this](QAbstractSocket::SocketError) {
        if (m_connected) { m_connected = false; emit connectedChanged(); }
        QTimer::singleShot(1000, this, [this]() { connectTo(m_host, m_port); });
    });
}

void PerceptionClient::connectTo(const QString &host, int port) {
    m_host = host; m_port = port;
    m_sock.abort();
    m_sock.connectToHost(host, quint16(port));
}

void PerceptionClient::onConnected() {
    m_connected = true; emit connectedChanged();
    qInfo() << "[viewer] connected to" << m_host << m_port;
}

void PerceptionClient::onDisconnected() {
    m_connected = false; emit connectedChanged();
    qInfo() << "[viewer] disconnected";
    // retry after a short delay
    QTimer::singleShot(1000, this, [this]() { connectTo(m_host, m_port); });
}

void PerceptionClient::onReadyRead() {
    m_buf.append(m_sock.readAll());
    while (m_buf.size() >= 4) {
        const quint32 n = (quint8(m_buf[0]) << 24) | (quint8(m_buf[1]) << 16)
                        | (quint8(m_buf[2]) << 8) | quint8(m_buf[3]);
        if (quint32(m_buf.size()) < 4 + n) break;
        const QByteArray jpeg = m_buf.mid(4, int(n));
        m_buf.remove(0, int(4 + n));
        QImage img;
        if (img.loadFromData(jpeg, "JPEG") && !img.isNull()) {
            m_provider->setImage(img);
            m_counter++;
            if (m_counter == 1 || m_counter % 30 == 0)
                qInfo() << "[viewer] frames received:" << m_counter
                        << "size" << img.width() << "x" << img.height();
            emit frameChanged();
        }
    }
}

void PerceptionClient::sendCommand(const QByteArray &cmd) {
    if (m_sock.state() == QAbstractSocket::ConnectedState) {
        m_sock.write(cmd);
        m_sock.flush();
    }
}

void PerceptionClient::doRegister() { sendCommand("register\n"); }
void PerceptionClient::doReset()    { sendCommand("reset\n"); }
