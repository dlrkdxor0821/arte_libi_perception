#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "FrameImageProvider.h"
#include "PerceptionClient.h"

int main(int argc, char **argv) {
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    auto *provider = new FrameImageProvider();
    engine.addImageProvider("perc", provider);

    auto *client = new PerceptionClient(provider, &app);
    engine.rootContext()->setContextProperty("client", client);

    engine.load(QUrl(QStringLiteral("qrc:/qml/Main.qml")));
    if (engine.rootObjects().isEmpty())
        return -1;

    QString host = "127.0.0.1";
    int port = 5007;
    if (argc >= 2) host = QString::fromUtf8(argv[1]);
    if (argc >= 3) port = QString::fromUtf8(argv[2]).toInt();
    client->connectTo(host, port);

    return app.exec();
}
