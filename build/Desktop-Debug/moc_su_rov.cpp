/****************************************************************************
** Meta object code from reading C++ file 'su_rov.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../MathAUV/su_rov.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'su_rov.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SU_ROV_t {
    QByteArrayData data[8];
    char stringdata0[64];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SU_ROV_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SU_ROV_t qt_meta_stringdata_SU_ROV = {
    {
QT_MOC_LITERAL(0, 0, 6), // "SU_ROV"
QT_MOC_LITERAL(1, 7, 18), // "updateCoromAUVReal"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 1), // "x"
QT_MOC_LITERAL(4, 29, 1), // "y"
QT_MOC_LITERAL(5, 31, 17), // "updateCoromAUVEkf"
QT_MOC_LITERAL(6, 49, 12), // "updateCircle"
QT_MOC_LITERAL(7, 62, 1) // "r"

    },
    "SU_ROV\0updateCoromAUVReal\0\0x\0y\0"
    "updateCoromAUVEkf\0updateCircle\0r"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SU_ROV[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   29,    2, 0x06 /* Public */,
       5,    2,   34,    2, 0x06 /* Public */,
       6,    1,   39,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double,    7,

       0        // eod
};

void SU_ROV::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SU_ROV *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->updateCoromAUVReal((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 1: _t->updateCoromAUVEkf((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 2: _t->updateCircle((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (SU_ROV::*)(double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SU_ROV::updateCoromAUVReal)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (SU_ROV::*)(double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SU_ROV::updateCoromAUVEkf)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (SU_ROV::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SU_ROV::updateCircle)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject SU_ROV::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_SU_ROV.data,
    qt_meta_data_SU_ROV,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *SU_ROV::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SU_ROV::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SU_ROV.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int SU_ROV::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void SU_ROV::updateCoromAUVReal(double _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SU_ROV::updateCoromAUVEkf(double _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void SU_ROV::updateCircle(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
