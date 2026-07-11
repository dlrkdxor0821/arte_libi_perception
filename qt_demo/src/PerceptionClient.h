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
    Q_PROPERTY(int lidarFront READ lidarFront NOTIFY lidarChanged)
    Q_PROPERTY(int lidarBack READ lidarBack NOTIFY lidarChanged)
    Q_PROPERTY(int lidarLeft READ lidarLeft NOTIFY lidarChanged)
    Q_PROPERTY(int lidarRight READ lidarRight NOTIFY lidarChanged)
public:
    explicit PerceptionClient(FrameImageProvider *provider, QObject *parent = nullptr);
    bool connected() const { return m_connected; }
    int frameCounter() const { return m_counter; }
    int lidarFront() const { return m_lF; }
    int lidarBack() const { return m_lB; }
    int lidarLeft() const { return m_lL; }
    int lidarRight() const { return m_lR; }
    Q_INVOKABLE void connectTo(const QString &host, int port);
    Q_INVOKABLE void doRegister();
    Q_INVOKABLE void doReset();
signals:
    void connectedChanged();
    void frameChanged();
    void lidarChanged();
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
    int m_lF = -1, m_lB = -1, m_lL = -1, m_lR = -1;   // LiDAR cm, -1 = no reading
    QString m_host;
    int m_port = 5007;
};
