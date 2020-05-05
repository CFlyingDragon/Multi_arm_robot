/********************************************************************************
** Form generated from reading UI file 'rqt_ethercat_test_plugin_widget.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RQT_ETHERCAT_TEST_PLUGIN_WIDGET_H
#define UI_RQT_ETHERCAT_TEST_PLUGIN_WIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_rqt_ethercat_test_plugin_widget
{
public:
    QGridLayout *gridLayout;
    QLabel *label;
    QFrame *line;
    QTableWidget *device_table;
    QComboBox *control_word;
    QPushButton *set_mode;
    QLabel *label_3;
    QLabel *label_5;
    QDoubleSpinBox *desired_position;
    QLabel *label_6;
    QDoubleSpinBox *desired_velocity;
    QLabel *label_7;
    QDoubleSpinBox *desired_torque;
    QComboBox *ethernet_port;
    QLineEdit *ethernet_port_input;
    QComboBox *pdo_type;
    QPushButton *connect;
    QPushButton *disconnect;
    QPushButton *send;
    QPushButton *stop;
    QTextBrowser *info_display;
    QLabel *label_10;
    QLabel *label_8;
    QLineEdit *CoEIndex;
    QLineEdit *SDOResult;
    QLabel *label_9;
    QLineEdit *CoESubIndex;
    QComboBox *SDOType;
    QComboBox *number_system;
    QPushButton *writeSDO;
    QPushButton *readSDO;
    QLabel *label_4;
    QLabel *label_2;
    QLabel *actual_position;
    QLabel *actual_velocity;
    QLabel *actual_torque;
    QComboBox *mode_of_operation;

    void setupUi(QWidget *rqt_ethercat_test_plugin_widget)
    {
        if (rqt_ethercat_test_plugin_widget->objectName().isEmpty())
            rqt_ethercat_test_plugin_widget->setObjectName(QStringLiteral("rqt_ethercat_test_plugin_widget"));
        rqt_ethercat_test_plugin_widget->resize(785, 391);
        gridLayout = new QGridLayout(rqt_ethercat_test_plugin_widget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        label = new QLabel(rqt_ethercat_test_plugin_widget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 0, 0, 1, 2);

        line = new QFrame(rqt_ethercat_test_plugin_widget);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        gridLayout->addWidget(line, 1, 0, 1, 10);

        device_table = new QTableWidget(rqt_ethercat_test_plugin_widget);
        if (device_table->columnCount() < 6)
            device_table->setColumnCount(6);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        device_table->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        device_table->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        device_table->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        device_table->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        device_table->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        device_table->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        if (device_table->rowCount() < 1)
            device_table->setRowCount(1);
        device_table->setObjectName(QStringLiteral("device_table"));
        device_table->setShowGrid(true);
        device_table->setRowCount(1);

        gridLayout->addWidget(device_table, 2, 0, 7, 5);

        control_word = new QComboBox(rqt_ethercat_test_plugin_widget);
        control_word->setObjectName(QStringLiteral("control_word"));

        gridLayout->addWidget(control_word, 3, 5, 1, 3);

        set_mode = new QPushButton(rqt_ethercat_test_plugin_widget);
        set_mode->setObjectName(QStringLiteral("set_mode"));
        set_mode->setStyleSheet(QStringLiteral(""));

        gridLayout->addWidget(set_mode, 3, 8, 1, 1);

        label_3 = new QLabel(rqt_ethercat_test_plugin_widget);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 5, 8, 1, 1);

        label_5 = new QLabel(rqt_ethercat_test_plugin_widget);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout->addWidget(label_5, 6, 5, 1, 1);

        desired_position = new QDoubleSpinBox(rqt_ethercat_test_plugin_widget);
        desired_position->setObjectName(QStringLiteral("desired_position"));
        desired_position->setMinimum(-100);
        desired_position->setSingleStep(0.1);

        gridLayout->addWidget(desired_position, 6, 8, 1, 1);

        label_6 = new QLabel(rqt_ethercat_test_plugin_widget);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout->addWidget(label_6, 7, 5, 1, 1);

        desired_velocity = new QDoubleSpinBox(rqt_ethercat_test_plugin_widget);
        desired_velocity->setObjectName(QStringLiteral("desired_velocity"));
        desired_velocity->setMinimum(-160);
        desired_velocity->setMaximum(160);
        desired_velocity->setSingleStep(0.1);

        gridLayout->addWidget(desired_velocity, 7, 8, 1, 1);

        label_7 = new QLabel(rqt_ethercat_test_plugin_widget);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout->addWidget(label_7, 8, 5, 1, 1);

        desired_torque = new QDoubleSpinBox(rqt_ethercat_test_plugin_widget);
        desired_torque->setObjectName(QStringLiteral("desired_torque"));
        desired_torque->setMinimum(-100);
        desired_torque->setMaximum(100);
        desired_torque->setSingleStep(0.1);

        gridLayout->addWidget(desired_torque, 8, 8, 1, 1);

        ethernet_port = new QComboBox(rqt_ethercat_test_plugin_widget);
        ethernet_port->setObjectName(QStringLiteral("ethernet_port"));

        gridLayout->addWidget(ethernet_port, 9, 0, 1, 1);

        ethernet_port_input = new QLineEdit(rqt_ethercat_test_plugin_widget);
        ethernet_port_input->setObjectName(QStringLiteral("ethernet_port_input"));

        gridLayout->addWidget(ethernet_port_input, 9, 1, 1, 1);

        pdo_type = new QComboBox(rqt_ethercat_test_plugin_widget);
        pdo_type->setObjectName(QStringLiteral("pdo_type"));

        gridLayout->addWidget(pdo_type, 9, 2, 1, 1);

        connect = new QPushButton(rqt_ethercat_test_plugin_widget);
        connect->setObjectName(QStringLiteral("connect"));

        gridLayout->addWidget(connect, 9, 3, 1, 1);

        disconnect = new QPushButton(rqt_ethercat_test_plugin_widget);
        disconnect->setObjectName(QStringLiteral("disconnect"));

        gridLayout->addWidget(disconnect, 9, 4, 1, 1);

        send = new QPushButton(rqt_ethercat_test_plugin_widget);
        send->setObjectName(QStringLiteral("send"));

        gridLayout->addWidget(send, 9, 6, 1, 2);

        stop = new QPushButton(rqt_ethercat_test_plugin_widget);
        stop->setObjectName(QStringLiteral("stop"));

        gridLayout->addWidget(stop, 9, 8, 1, 1);

        info_display = new QTextBrowser(rqt_ethercat_test_plugin_widget);
        info_display->setObjectName(QStringLiteral("info_display"));

        gridLayout->addWidget(info_display, 10, 0, 4, 5);

        label_10 = new QLabel(rqt_ethercat_test_plugin_widget);
        label_10->setObjectName(QStringLiteral("label_10"));

        gridLayout->addWidget(label_10, 10, 8, 1, 1);

        label_8 = new QLabel(rqt_ethercat_test_plugin_widget);
        label_8->setObjectName(QStringLiteral("label_8"));

        gridLayout->addWidget(label_8, 11, 5, 1, 1);

        CoEIndex = new QLineEdit(rqt_ethercat_test_plugin_widget);
        CoEIndex->setObjectName(QStringLiteral("CoEIndex"));

        gridLayout->addWidget(CoEIndex, 11, 6, 1, 2);

        SDOResult = new QLineEdit(rqt_ethercat_test_plugin_widget);
        SDOResult->setObjectName(QStringLiteral("SDOResult"));

        gridLayout->addWidget(SDOResult, 11, 8, 1, 2);

        label_9 = new QLabel(rqt_ethercat_test_plugin_widget);
        label_9->setObjectName(QStringLiteral("label_9"));

        gridLayout->addWidget(label_9, 12, 5, 1, 1);

        CoESubIndex = new QLineEdit(rqt_ethercat_test_plugin_widget);
        CoESubIndex->setObjectName(QStringLiteral("CoESubIndex"));
        CoESubIndex->setMaximumSize(QSize(150, 16777215));

        gridLayout->addWidget(CoESubIndex, 12, 6, 1, 2);

        SDOType = new QComboBox(rqt_ethercat_test_plugin_widget);
        SDOType->setObjectName(QStringLiteral("SDOType"));

        gridLayout->addWidget(SDOType, 12, 8, 1, 1);

        number_system = new QComboBox(rqt_ethercat_test_plugin_widget);
        number_system->setObjectName(QStringLiteral("number_system"));

        gridLayout->addWidget(number_system, 12, 9, 1, 1);

        writeSDO = new QPushButton(rqt_ethercat_test_plugin_widget);
        writeSDO->setObjectName(QStringLiteral("writeSDO"));

        gridLayout->addWidget(writeSDO, 13, 7, 1, 1);

        readSDO = new QPushButton(rqt_ethercat_test_plugin_widget);
        readSDO->setObjectName(QStringLiteral("readSDO"));

        gridLayout->addWidget(readSDO, 13, 8, 1, 1);

        label_4 = new QLabel(rqt_ethercat_test_plugin_widget);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 4, 7, 1, 1);

        label_2 = new QLabel(rqt_ethercat_test_plugin_widget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 5, 7, 1, 1);

        actual_position = new QLabel(rqt_ethercat_test_plugin_widget);
        actual_position->setObjectName(QStringLiteral("actual_position"));

        gridLayout->addWidget(actual_position, 6, 7, 1, 1);

        actual_velocity = new QLabel(rqt_ethercat_test_plugin_widget);
        actual_velocity->setObjectName(QStringLiteral("actual_velocity"));

        gridLayout->addWidget(actual_velocity, 7, 7, 1, 1);

        actual_torque = new QLabel(rqt_ethercat_test_plugin_widget);
        actual_torque->setObjectName(QStringLiteral("actual_torque"));

        gridLayout->addWidget(actual_torque, 8, 7, 1, 1);

        mode_of_operation = new QComboBox(rqt_ethercat_test_plugin_widget);
        mode_of_operation->setObjectName(QStringLiteral("mode_of_operation"));

        gridLayout->addWidget(mode_of_operation, 4, 8, 1, 1);


        retranslateUi(rqt_ethercat_test_plugin_widget);

        QMetaObject::connectSlotsByName(rqt_ethercat_test_plugin_widget);
    } // setupUi

    void retranslateUi(QWidget *rqt_ethercat_test_plugin_widget)
    {
        rqt_ethercat_test_plugin_widget->setWindowTitle(QApplication::translate("rqt_ethercat_test_plugin_widget", "Form", 0));
        label->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "EtherCAT Motor Test", 0));
        QTableWidgetItem *___qtablewidgetitem = device_table->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Address", 0));
        QTableWidgetItem *___qtablewidgetitem1 = device_table->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Device", 0));
        QTableWidgetItem *___qtablewidgetitem2 = device_table->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "EtheCAT State", 0));
        QTableWidgetItem *___qtablewidgetitem3 = device_table->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Status", 0));
        QTableWidgetItem *___qtablewidgetitem4 = device_table->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Mode of operation", 0));
        QTableWidgetItem *___qtablewidgetitem5 = device_table->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Choose", 0));
        control_word->clear();
        control_word->insertItems(0, QStringList()
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Warm Reset", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Configure to Standby", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Standby to MotorPreOp", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "MotorOp to ControlOp", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "ControlOp to MotorOp", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Standby to Configure", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "MotorOp to Standby", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Clear Errors to MotorOp", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Clear Errors to Standby", 0)
        );
#ifndef QT_NO_WHATSTHIS
        set_mode->setWhatsThis(QApplication::translate("rqt_ethercat_test_plugin_widget", "<html><head/><body><p><br/></p></body></html>", 0));
#endif // QT_NO_WHATSTHIS
        set_mode->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Set", 0));
        label_3->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Command", 0));
        label_5->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Position(rad)", 0));
        label_6->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Velocity(rad/s)", 0));
        label_7->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Torque(Nm)", 0));
        ethernet_port->clear();
        ethernet_port->insertItems(0, QStringList()
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "eth0", 0)
        );
        ethernet_port_input->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "eno1", 0));
        pdo_type->clear();
        pdo_type->insertItems(0, QStringList()
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "PDOA", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "PDOB", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "PDOC", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "PDOD", 0)
        );
        connect->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Connect", 0));
        disconnect->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Disconnect", 0));
        send->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Send", 0));
        stop->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Stop", 0));
        label_10->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Result", 0));
        label_8->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Index", 0));
        label_9->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Subindex", 0));
        SDOType->clear();
        SDOType->insertItems(0, QStringList()
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "INT8", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "INT16", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "INT32", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "UINT8", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "UINT16", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "UINT32", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "STRING", 0)
        );
        number_system->clear();
        number_system->insertItems(0, QStringList()
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Binary", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Hex", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Dec", 0)
        );
        writeSDO->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Write SDO", 0));
        readSDO->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Read SDO", 0));
        label_4->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Mode", 0));
        label_2->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "Actual", 0));
        actual_position->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "0.0", 0));
        actual_velocity->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "0.0", 0));
        actual_torque->setText(QApplication::translate("rqt_ethercat_test_plugin_widget", "0.0", 0));
        mode_of_operation->clear();
        mode_of_operation->insertItems(0, QStringList()
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Position", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Velocity", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Torque", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Freeze", 0)
         << QApplication::translate("rqt_ethercat_test_plugin_widget", "Disable", 0)
        );
    } // retranslateUi

};

namespace Ui {
    class rqt_ethercat_test_plugin_widget: public Ui_rqt_ethercat_test_plugin_widget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RQT_ETHERCAT_TEST_PLUGIN_WIDGET_H
