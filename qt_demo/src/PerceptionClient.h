#pragma once
#include <QObject>
#include <QTcpSocket>
#include <QByteArray>
#include <QString>

class FrameImageProvider;

class PerceptionClient : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool connected READ connected NOTIFY connectedChanged)
    Q_PROPERTY(int frameCounter READ frameCounter NOTIFY frameChanged)
public:
    explicit PerceptionClient(FrameImageProvider *provider, QObject *parent = nullptr);
    bool connected() const { return m_connected; }
    int frameCounter() const { return m_counter; }
    Q_INVOKABLE void connectTo(const QString &host, int port);
    Q_INVOKABLE void doRegister();
    Q_INVOKABLE void doReset();
signals:
    void connectedChanged();
    void frameChanged();
private slots:
    void onReadyRead();
    void onConnected();
    void onDisconnected();
private:
    void sendCommand(const QByteArray &cmd);
    QTcpSocket m_sock;
    FrameImageProvider *m_provider;
    QByteArray m_buf;
    bool m_connected = false;
    int m_counter = 0;
    QString m_host;
    int m_port = 5007;
};
