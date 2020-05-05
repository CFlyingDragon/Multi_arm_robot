/****************************************************************************
** Meta object code from reading C++ file 'rqt_ethercat_test_plugin_widget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../src/robot_driver/ros_ethercat-master/rqt_ethercat_test_plugin/include/rqt_ethercat_test_plugin/rqt_ethercat_test_plugin_widget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'rqt_ethercat_test_plugin_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rqt_ethercat_test_plugin_widget_t {
    QByteArrayData data[13];
    char stringdata0[258];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rqt_ethercat_test_plugin_widget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rqt_ethercat_test_plugin_widget_t qt_meta_stringdata_rqt_ethercat_test_plugin_widget = {
    {
QT_MOC_LITERAL(0, 0, 31), // "rqt_ethercat_test_plugin_widget"
QT_MOC_LITERAL(1, 32, 42), // "on_info_display_customContext..."
QT_MOC_LITERAL(2, 75, 0), // ""
QT_MOC_LITERAL(3, 76, 3), // "pos"
QT_MOC_LITERAL(4, 80, 18), // "on_connect_clicked"
QT_MOC_LITERAL(5, 99, 21), // "on_disconnect_clicked"
QT_MOC_LITERAL(6, 121, 19), // "on_set_mode_clicked"
QT_MOC_LITERAL(7, 141, 15), // "on_send_clicked"
QT_MOC_LITERAL(8, 157, 15), // "on_stop_clicked"
QT_MOC_LITERAL(9, 173, 18), // "on_readSDO_clicked"
QT_MOC_LITERAL(10, 192, 19), // "on_writeSDO_clicked"
QT_MOC_LITERAL(11, 212, 40), // "on_mode_of_operation_currentI..."
QT_MOC_LITERAL(12, 253, 4) // "arg1"

    },
    "rqt_ethercat_test_plugin_widget\0"
    "on_info_display_customContextMenuRequested\0"
    "\0pos\0on_connect_clicked\0on_disconnect_clicked\0"
    "on_set_mode_clicked\0on_send_clicked\0"
    "on_stop_clicked\0on_readSDO_clicked\0"
    "on_writeSDO_clicked\0"
    "on_mode_of_operation_currentIndexChanged\0"
    "arg1"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rqt_ethercat_test_plugin_widget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   59,    2, 0x08 /* Private */,
       4,    0,   62,    2, 0x08 /* Private */,
       5,    0,   63,    2, 0x08 /* Private */,
       6,    0,   64,    2, 0x08 /* Private */,
       7,    0,   65,    2, 0x08 /* Private */,
       8,    0,   66,    2, 0x08 /* Private */,
       9,    0,   67,    2, 0x08 /* Private */,
      10,    0,   68,    2, 0x08 /* Private */,
      11,    1,   69,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::QPoint,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   12,

       0        // eod
};

void rqt_ethercat_test_plugin_widget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        rqt_ethercat_test_plugin_widget *_t = static_cast<rqt_ethercat_test_plugin_widget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_info_display_customContextMenuRequested((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 1: _t->on_connect_clicked(); break;
        case 2: _t->on_disconnect_clicked(); break;
        case 3: _t->on_set_mode_clicked(); break;
        case 4: _t->on_send_clicked(); break;
        case 5: _t->on_stop_clicked(); break;
        case 6: _t->on_readSDO_clicked(); break;
        case 7: _t->on_writeSDO_clicked(); break;
        case 8: _t->on_mode_of_operation_currentIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject rqt_ethercat_test_plugin_widget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_rqt_ethercat_test_plugin_widget.data,
      qt_meta_data_rqt_ethercat_test_plugin_widget,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rqt_ethercat_test_plugin_widget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rqt_ethercat_test_plugin_widget::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rqt_ethercat_test_plugin_widget.stringdata0))
        return static_cast<void*>(const_cast< rqt_ethercat_test_plugin_widget*>(this));
    return QWidget::qt_metacast(_clname);
}

int rqt_ethercat_test_plugin_widget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
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
    return _id;
}
QT_END_MOC_NAMESPACE
