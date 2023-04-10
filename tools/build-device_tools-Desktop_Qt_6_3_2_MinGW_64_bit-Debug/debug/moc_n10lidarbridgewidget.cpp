/****************************************************************************
** Meta object code from reading C++ file 'n10lidarbridgewidget.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.3.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../device_tools/n10lidarbridgewidget.h"
#include <QtGui/qtextcursor.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'n10lidarbridgewidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.3.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_N10LidarBridgeWidget_t {
    uint offsetsAndSizes[16];
    char stringdata0[21];
    char stringdata1[16];
    char stringdata2[1];
    char stringdata3[8];
    char stringdata4[20];
    char stringdata5[20];
    char stringdata6[18];
    char stringdata7[22];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_N10LidarBridgeWidget_t::offsetsAndSizes) + ofs), len 
static const qt_meta_stringdata_N10LidarBridgeWidget_t qt_meta_stringdata_N10LidarBridgeWidget = {
    {
        QT_MOC_LITERAL(0, 20),  // "N10LidarBridgeWidget"
        QT_MOC_LITERAL(21, 15),  // "slot_sp_connect"
        QT_MOC_LITERAL(37, 0),  // ""
        QT_MOC_LITERAL(38, 7),  // "checked"
        QT_MOC_LITERAL(46, 19),  // "slot_socket_connect"
        QT_MOC_LITERAL(66, 19),  // "slot_bridge_connect"
        QT_MOC_LITERAL(86, 17),  // "slot_sp_data_recv"
        QT_MOC_LITERAL(104, 21)   // "slot_socket_data_recv"
    },
    "N10LidarBridgeWidget",
    "slot_sp_connect",
    "",
    "checked",
    "slot_socket_connect",
    "slot_bridge_connect",
    "slot_sp_data_recv",
    "slot_socket_data_recv"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_N10LidarBridgeWidget[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   62,    2, 0x08,    1 /* Private */,
       1,    0,   65,    2, 0x28,    3 /* Private | MethodCloned */,
       4,    1,   66,    2, 0x08,    4 /* Private */,
       4,    0,   69,    2, 0x28,    6 /* Private | MethodCloned */,
       5,    1,   70,    2, 0x08,    7 /* Private */,
       5,    0,   73,    2, 0x28,    9 /* Private | MethodCloned */,
       6,    0,   74,    2, 0x08,   10 /* Private */,
       7,    0,   75,    2, 0x08,   11 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void N10LidarBridgeWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<N10LidarBridgeWidget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->slot_sp_connect((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 1: _t->slot_sp_connect(); break;
        case 2: _t->slot_socket_connect((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 3: _t->slot_socket_connect(); break;
        case 4: _t->slot_bridge_connect((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 5: _t->slot_bridge_connect(); break;
        case 6: _t->slot_sp_data_recv(); break;
        case 7: _t->slot_socket_data_recv(); break;
        default: ;
        }
    }
}

const QMetaObject N10LidarBridgeWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_N10LidarBridgeWidget.offsetsAndSizes,
    qt_meta_data_N10LidarBridgeWidget,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_N10LidarBridgeWidget_t
, QtPrivate::TypeAndForceComplete<N10LidarBridgeWidget, std::true_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>


>,
    nullptr
} };


const QMetaObject *N10LidarBridgeWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *N10LidarBridgeWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_N10LidarBridgeWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int N10LidarBridgeWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 8;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
