/****************************************************************************
** Meta object code from reading C++ file 'PerceptionClient.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../src/PerceptionClient.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PerceptionClient.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_PerceptionClient_t {
    QByteArrayData data[23];
    char stringdata0[264];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PerceptionClient_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PerceptionClient_t qt_meta_stringdata_PerceptionClient = {
    {
QT_MOC_LITERAL(0, 0, 16), // "PerceptionClient"
QT_MOC_LITERAL(1, 17, 16), // "connectedChanged"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 12), // "frameChanged"
QT_MOC_LITERAL(4, 48, 12), // "lidarChanged"
QT_MOC_LITERAL(5, 61, 11), // "onReadyRead"
QT_MOC_LITERAL(6, 73, 11), // "onConnected"
QT_MOC_LITERAL(7, 85, 14), // "onDisconnected"
QT_MOC_LITERAL(8, 100, 9), // "connectTo"
QT_MOC_LITERAL(9, 110, 4), // "host"
QT_MOC_LITERAL(10, 115, 4), // "port"
QT_MOC_LITERAL(11, 120, 10), // "doRegister"
QT_MOC_LITERAL(12, 131, 7), // "doReset"
QT_MOC_LITERAL(13, 139, 9), // "connected"
QT_MOC_LITERAL(14, 149, 12), // "frameCounter"
QT_MOC_LITERAL(15, 162, 10), // "lidarFront"
QT_MOC_LITERAL(16, 173, 9), // "lidarBack"
QT_MOC_LITERAL(17, 183, 9), // "lidarLeft"
QT_MOC_LITERAL(18, 193, 10), // "lidarRight"
QT_MOC_LITERAL(19, 204, 14), // "lidarFrontLeft"
QT_MOC_LITERAL(20, 219, 15), // "lidarFrontRight"
QT_MOC_LITERAL(21, 235, 13), // "lidarBackLeft"
QT_MOC_LITERAL(22, 249, 14) // "lidarBackRight"

    },
    "PerceptionClient\0connectedChanged\0\0"
    "frameChanged\0lidarChanged\0onReadyRead\0"
    "onConnected\0onDisconnected\0connectTo\0"
    "host\0port\0doRegister\0doReset\0connected\0"
    "frameCounter\0lidarFront\0lidarBack\0"
    "lidarLeft\0lidarRight\0lidarFrontLeft\0"
    "lidarFrontRight\0lidarBackLeft\0"
    "lidarBackRight"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PerceptionClient[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
      10,   72, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   59,    2, 0x06 /* Public */,
       3,    0,   60,    2, 0x06 /* Public */,
       4,    0,   61,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   62,    2, 0x08 /* Private */,
       6,    0,   63,    2, 0x08 /* Private */,
       7,    0,   64,    2, 0x08 /* Private */,

 // methods: name, argc, parameters, tag, flags
       8,    2,   65,    2, 0x02 /* Public */,
      11,    0,   70,    2, 0x02 /* Public */,
      12,    0,   71,    2, 0x02 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // methods: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::Int,    9,   10,
    QMetaType::Void,
    QMetaType::Void,

 // properties: name, type, flags
      13, QMetaType::Bool, 0x00495001,
      14, QMetaType::Int, 0x00495001,
      15, QMetaType::Int, 0x00495001,
      16, QMetaType::Int, 0x00495001,
      17, QMetaType::Int, 0x00495001,
      18, QMetaType::Int, 0x00495001,
      19, QMetaType::Int, 0x00495001,
      20, QMetaType::Int, 0x00495001,
      21, QMetaType::Int, 0x00495001,
      22, QMetaType::Int, 0x00495001,

 // properties: notify_signal_id
       0,
       1,
       2,
       2,
       2,
       2,
       2,
       2,
       2,
       2,

       0        // eod
};

void PerceptionClient::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<PerceptionClient *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->connectedChanged(); break;
        case 1: _t->frameChanged(); break;
        case 2: _t->lidarChanged(); break;
        case 3: _t->onReadyRead(); break;
        case 4: _t->onConnected(); break;
        case 5: _t->onDisconnected(); break;
        case 6: _t->connectTo((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 7: _t->doRegister(); break;
        case 8: _t->doReset(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (PerceptionClient::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PerceptionClient::connectedChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (PerceptionClient::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PerceptionClient::frameChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (PerceptionClient::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PerceptionClient::lidarChanged)) {
                *result = 2;
                return;
            }
        }
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty) {
        auto *_t = static_cast<PerceptionClient *>(_o);
        (void)_t;
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< bool*>(_v) = _t->connected(); break;
        case 1: *reinterpret_cast< int*>(_v) = _t->frameCounter(); break;
        case 2: *reinterpret_cast< int*>(_v) = _t->lidarFront(); break;
        case 3: *reinterpret_cast< int*>(_v) = _t->lidarBack(); break;
        case 4: *reinterpret_cast< int*>(_v) = _t->lidarLeft(); break;
        case 5: *reinterpret_cast< int*>(_v) = _t->lidarRight(); break;
        case 6: *reinterpret_cast< int*>(_v) = _t->lidarFrontLeft(); break;
        case 7: *reinterpret_cast< int*>(_v) = _t->lidarFrontRight(); break;
        case 8: *reinterpret_cast< int*>(_v) = _t->lidarBackLeft(); break;
        case 9: *reinterpret_cast< int*>(_v) = _t->lidarBackRight(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
}

QT_INIT_METAOBJECT const QMetaObject PerceptionClient::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_PerceptionClient.data,
    qt_meta_data_PerceptionClient,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *PerceptionClient::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PerceptionClient::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_PerceptionClient.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int PerceptionClient::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 10;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 10;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 10;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 10;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 10;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}

// SIGNAL 0
void PerceptionClient::connectedChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void PerceptionClient::frameChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void PerceptionClient::lidarChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
