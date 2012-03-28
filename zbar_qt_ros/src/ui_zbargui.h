/********************************************************************************
** Form generated from reading UI file 'zbargui.ui'
**
** Created: Fri Feb 17 17:05:49 2012
**      by: Qt User Interface Compiler version 4.7.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ZBARGUI_H
#define UI_ZBARGUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_zbarGui
{
public:
    QWidget *centralwidget;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *product_name;
    QLabel *label_5;
    QLabel *product_producer;
    QLabel *label_7;
    QLabel *product_category;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *zbarGui)
    {
        if (zbarGui->objectName().isEmpty())
            zbarGui->setObjectName(QString::fromUtf8("zbarGui"));
        zbarGui->resize(713, 473);
        centralwidget = new QWidget(zbarGui);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        pushButton = new QPushButton(centralwidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(10, 10, 91, 51));
        pushButton_2 = new QPushButton(centralwidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setGeometry(QRect(10, 70, 91, 51));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(110, 10, 251, 271));
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(340, 10, 251, 271));
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(80, 320, 121, 20));
        product_name = new QLabel(centralwidget);
        product_name->setObjectName(QString::fromUtf8("product_name"));
        product_name->setGeometry(QRect(210, 320, 431, 17));
        label_5 = new QLabel(centralwidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(80, 340, 131, 20));
        product_producer = new QLabel(centralwidget);
        product_producer->setObjectName(QString::fromUtf8("product_producer"));
        product_producer->setGeometry(QRect(220, 340, 421, 20));
        label_7 = new QLabel(centralwidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(80, 360, 131, 20));
        product_category = new QLabel(centralwidget);
        product_category->setObjectName(QString::fromUtf8("product_category"));
        product_category->setGeometry(QRect(220, 360, 421, 20));
        zbarGui->setCentralWidget(centralwidget);
        menubar = new QMenuBar(zbarGui);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 713, 25));
        zbarGui->setMenuBar(menubar);
        statusbar = new QStatusBar(zbarGui);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        zbarGui->setStatusBar(statusbar);

        retranslateUi(zbarGui);

        QMetaObject::connectSlotsByName(zbarGui);
    } // setupUi

    void retranslateUi(QMainWindow *zbarGui)
    {
        zbarGui->setWindowTitle(QApplication::translate("zbarGui", "MainWindow", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("zbarGui", "Start", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("zbarGui", "Quit", 0, QApplication::UnicodeUTF8));
        label->setText(QString());
        label_2->setText(QString());
        label_3->setText(QApplication::translate("zbarGui", "Product name: ", 0, QApplication::UnicodeUTF8));
        product_name->setText(QString());
        label_5->setText(QApplication::translate("zbarGui", "Product producer: ", 0, QApplication::UnicodeUTF8));
        product_producer->setText(QString());
        label_7->setText(QApplication::translate("zbarGui", "Product category: ", 0, QApplication::UnicodeUTF8));
        product_category->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class zbarGui: public Ui_zbarGui {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ZBARGUI_H
