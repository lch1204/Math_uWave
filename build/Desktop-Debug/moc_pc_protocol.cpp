/****************************************************************************
** Meta object code from reading C++ file 'pc_protocol.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../Connection/pc_protocol.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pc_protocol.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ControlSystem__PC_Protocol_t {
    QByteArrayData data[7];
    char stringdata0[88];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ControlSystem__PC_Protocol_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ControlSystem__PC_Protocol_t qt_meta_stringdata_ControlSystem__PC_Protocol = {
    {
QT_MOC_LITERAL(0, 0, 26), // "ControlSystem::PC_Protocol"
QT_MOC_LITERAL(1, 27, 12), // "dataReceived"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 13), // "startExchange"
QT_MOC_LITERAL(4, 55, 11), // "stopExhange"
QT_MOC_LITERAL(5, 67, 8), // "sendData"
QT_MOC_LITERAL(6, 76, 11) // "receiveData"

    },
    "ControlSystem::PC_Protocol\0dataReceived\0"
    "\0startExchange\0stopExhange\0sendData\0"
    "receiveData"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ControlSystem__PC_Protocol[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   40,    2, 0x0a /* Public */,
       4,    0,   41,    2, 0x0a /* Public */,
       5,    0,   42,    2, 0x0a /* Public */,
       6,    0,   43,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void ControlSystem::PC_Protocol::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<PC_Protocol *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->dataReceived(); break;
        case 1: _t->startExchange(); break;
        case 2: _t->stopExhange(); break;
        case 3: _t->sendData(); break;
        case 4: _t->receiveData(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (PC_Protocol::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PC_Protocol::dataReceived)) {
                *result = 0;
                return;
            }
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject ControlSystem::PC_Protocol::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_ControlSystem__PC_Protocol.data,
    qt_meta_data_ControlSystem__PC_Protocol,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ControlSystem::PC_Protocol::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ControlSystem::PC_Protocol::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ControlSystem__PC_Protocol.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "MetaUdpProtocol"))
        return static_cast< MetaUdpProtocol*>(this);
    return QObject::qt_metacast(_clname);
}

int ControlSystem::PC_Protocol::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void ControlSystem::PC_Protocol::dataReceived()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
struct qt_meta_stringdata_Pult__PC_Protocol_t {
    QByteArrayData data[7];
    char stringdata0[79];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Pult__PC_Protocol_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Pult__PC_Protocol_t qt_meta_stringdata_Pult__PC_Protocol = {
    {
QT_MOC_LITERAL(0, 0, 17), // "Pult::PC_Protocol"
QT_MOC_LITERAL(1, 18, 12), // "dataReceived"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 13), // "startExchange"
QT_MOC_LITERAL(4, 46, 11), // "stopExhange"
QT_MOC_LITERAL(5, 58, 8), // "sendData"
QT_MOC_LITERAL(6, 67, 11) // "receiveData"

    },
    "Pult::PC_Protocol\0dataReceived\0\0"
    "startExchange\0stopExhange\0sendData\0"
    "receiveData"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Pult__PC_Protocol[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   40,    2, 0x0a /* Public */,
       4,    0,   41,    2, 0x0a /* Public */,
       5,    0,   42,    2, 0x0a /* Public */,
       6,    0,   43,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void Pult::PC_Protocol::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<PC_Protocol *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->dataReceived(); break;
        case 1: _t->startExchange(); break;
        case 2: _t->stopExhange(); break;
        case 3: _t->sendData(); break;
        case 4: _t->receiveData(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (PC_Protocol::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PC_Protocol::dataReceived)) {
                *result = 0;
                return;
            }
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject Pult::PC_Protocol::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_Pult__PC_Protocol.data,
    qt_meta_data_Pult__PC_Protocol,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Pult::PC_Protocol::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Pult::PC_Protocol::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Pult__PC_Protocol.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "MetaUdpProtocol"))
        return static_cast< MetaUdpProtocol*>(this);
    return QObject::qt_metacast(_clname);
}

int Pult::PC_Protocol::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void Pult::PC_Protocol::dataReceived()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
