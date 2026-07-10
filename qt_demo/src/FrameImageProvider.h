#pragma once
#include <QQuickImageProvider>
#include <QImage>
#include <QMutex>

class FrameImageProvider : public QQuickImageProvider {
public:
    FrameImageProvider() : QQuickImageProvider(QQuickImageProvider::Image) {}
    QImage requestImage(const QString &, QSize *size, const QSize &) override {
        QMutexLocker lock(&m_mutex);
        if (size) *size = m_image.size();
        return m_image;
    }
    void setImage(const QImage &img) { QMutexLocker lock(&m_mutex); m_image = img; }
private:
    QImage m_image;
    QMutex m_mutex;
};
