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
    QByteArrayData data[28];
    char stringdata0[223];
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
QT_MOC_LITERAL(7, 62, 1), // "r"
QT_MOC_LITERAL(8, 64, 24), // "updateVelocityVector_ekf"
QT_MOC_LITERAL(9, 89, 2), // "vx"
QT_MOC_LITERAL(10, 92, 2), // "vy"
QT_MOC_LITERAL(11, 95, 25), // "updateVelocityVector_real"
QT_MOC_LITERAL(12, 121, 15), // "positionUpdated"
QT_MOC_LITERAL(13, 137, 1), // "z"
QT_MOC_LITERAL(14, 139, 18), // "orientationUpdated"
QT_MOC_LITERAL(15, 158, 3), // "phi"
QT_MOC_LITERAL(16, 162, 5), // "theta"
QT_MOC_LITERAL(17, 168, 3), // "psi"
QT_MOC_LITERAL(18, 172, 7), // "updateX"
QT_MOC_LITERAL(19, 180, 1), // "X"
QT_MOC_LITERAL(20, 182, 4), // "ekfX"
QT_MOC_LITERAL(21, 187, 5), // "t_sim"
QT_MOC_LITERAL(22, 193, 7), // "updateY"
QT_MOC_LITERAL(23, 201, 1), // "Y"
QT_MOC_LITERAL(24, 203, 4), // "ekfY"
QT_MOC_LITERAL(25, 208, 7), // "updateZ"
QT_MOC_LITERAL(26, 216, 1), // "Z"
QT_MOC_LITERAL(27, 218, 4) // "ekfZ"

    },
    "SU_ROV\0updateCoromAUVReal\0\0x\0y\0"
    "updateCoromAUVEkf\0updateCircle\0r\0"
    "updateVelocityVector_ekf\0vx\0vy\0"
    "updateVelocityVector_real\0positionUpdated\0"
    "z\0orientationUpdated\0phi\0theta\0psi\0"
    "updateX\0X\0ekfX\0t_sim\0updateY\0Y\0ekfY\0"
    "updateZ\0Z\0ekfZ"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SU_ROV[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      10,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   64,    2, 0x06 /* Public */,
       5,    2,   69,    2, 0x06 /* Public */,
       6,    1,   74,    2, 0x06 /* Public */,
       8,    2,   77,    2, 0x06 /* Public */,
      11,    2,   82,    2, 0x06 /* Public */,
      12,    3,   87,    2, 0x06 /* Public */,
      14,    3,   94,    2, 0x06 /* Public */,
      18,    3,  101,    2, 0x06 /* Public */,
      22,    3,  108,    2, 0x06 /* Public */,
      25,    3,  115,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Double,    7,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    9,   10,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    9,   10,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,    3,    4,   13,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,   15,   16,   17,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,   19,   20,   21,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,   23,   24,   21,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,   26,   27,   21,

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
        case 3: _t->updateVelocityVector_ekf((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 4: _t->updateVelocityVector_real((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 5: _t->positionUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 6: _t->orientationUpdated((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 7: _t->updateX((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 8: _t->updateY((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 9: _t->updateZ((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
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
        {
            using _t = void (SU_ROV::*)(double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SU_ROV::updateVelocityVector_ekf)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (SU_ROV::*)(double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SU_ROV::updateVelocityVector_real)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (SU_ROV::*)(double , double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SU_ROV::positionUpdated)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (SU_ROV::*)(double , double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SU_ROV::orientationUpdated)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (SU_ROV::*)(double , double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SU_ROV::updateX)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (SU_ROV::*)(double , double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SU_ROV::updateY)) {
                *result = 8;
                return;
            }
        }
        {
            using _t = void (SU_ROV::*)(double , double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SU_ROV::updateZ)) {
                *result = 9;
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
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
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

// SIGNAL 3
void SU_ROV::updateVelocityVector_ekf(double _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void SU_ROV::updateVelocityVector_real(double _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void SU_ROV::positionUpdated(double _t1, double _t2, double _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void SU_ROV::orientationUpdated(double _t1, double _t2, double _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void SU_ROV::updateX(double _t1, double _t2, double _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void SU_ROV::updateY(double _t1, double _t2, double _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void SU_ROV::updateZ(double _t1, double _t2, double _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
