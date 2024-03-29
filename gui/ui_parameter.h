/********************************************************************************
** Form generated from reading UI file 'parameter.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PARAMETER_H
#define UI_PARAMETER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_Dialog
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QGroupBox *groupBox;
    QFormLayout *formLayout_4;
    QLabel *qHzLabel;
    QDoubleSpinBox *fc_qdDoubleSpinBox;
    QLabel *qdLabel;
    QDoubleSpinBox *fc_qddDoubleSpinBox;
    QLabel *uLabel;
    QDoubleSpinBox *fc_uDoubleSpinBox;
    QHBoxLayout *horizontalLayout_2;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout_5;
    QDoubleSpinBox *Kc0DoubleSpinBox;
    QLabel *label_11;
    QDoubleSpinBox *Kc1DoubleSpinBox;
    QDoubleSpinBox *Kc2DoubleSpinBox;
    QLabel *label_10;
    QDoubleSpinBox *Bc2DoubleSpinBox;
    QDoubleSpinBox *Bc0DoubleSpinBox;
    QDoubleSpinBox *Bc1DoubleSpinBox;
    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_14;
    QGroupBox *formGroupBox;
    QFormLayout *formLayout;
    QDoubleSpinBox *MhDoubleSpinBox;
    QLabel *mholderLabel;
    QFormLayout *formLayout_5;
    QLabel *samplingPeriodLabel;
    QSpinBox *periodSpinBox;
    QLabel *feedforwardLabel;
    QCheckBox *feedforwardCheckBox;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *Dialog)
    {
        if (Dialog->objectName().isEmpty())
            Dialog->setObjectName(QString::fromUtf8("Dialog"));
        Dialog->setWindowModality(Qt::NonModal);
        Dialog->resize(1004, 622);
        Dialog->setModal(false);
        verticalLayout = new QVBoxLayout(Dialog);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(10);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        groupBox = new QGroupBox(Dialog);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setFlat(false);
        formLayout_4 = new QFormLayout(groupBox);
        formLayout_4->setObjectName(QString::fromUtf8("formLayout_4"));
        formLayout_4->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        qHzLabel = new QLabel(groupBox);
        qHzLabel->setObjectName(QString::fromUtf8("qHzLabel"));

        formLayout_4->setWidget(0, QFormLayout::LabelRole, qHzLabel);

        fc_qdDoubleSpinBox = new QDoubleSpinBox(groupBox);
        fc_qdDoubleSpinBox->setObjectName(QString::fromUtf8("fc_qdDoubleSpinBox"));
        fc_qdDoubleSpinBox->setAccelerated(true);
        fc_qdDoubleSpinBox->setDecimals(0);
        fc_qdDoubleSpinBox->setMinimum(1.000000000000000);
        fc_qdDoubleSpinBox->setMaximum(1000.000000000000000);
        fc_qdDoubleSpinBox->setValue(100.000000000000000);

        formLayout_4->setWidget(0, QFormLayout::FieldRole, fc_qdDoubleSpinBox);

        qdLabel = new QLabel(groupBox);
        qdLabel->setObjectName(QString::fromUtf8("qdLabel"));

        formLayout_4->setWidget(1, QFormLayout::LabelRole, qdLabel);

        fc_qddDoubleSpinBox = new QDoubleSpinBox(groupBox);
        fc_qddDoubleSpinBox->setObjectName(QString::fromUtf8("fc_qddDoubleSpinBox"));
        fc_qddDoubleSpinBox->setDecimals(0);
        fc_qddDoubleSpinBox->setMinimum(1.000000000000000);
        fc_qddDoubleSpinBox->setMaximum(1000.000000000000000);
        fc_qddDoubleSpinBox->setValue(100.000000000000000);

        formLayout_4->setWidget(1, QFormLayout::FieldRole, fc_qddDoubleSpinBox);

        uLabel = new QLabel(groupBox);
        uLabel->setObjectName(QString::fromUtf8("uLabel"));

        formLayout_4->setWidget(2, QFormLayout::LabelRole, uLabel);

        fc_uDoubleSpinBox = new QDoubleSpinBox(groupBox);
        fc_uDoubleSpinBox->setObjectName(QString::fromUtf8("fc_uDoubleSpinBox"));
        fc_uDoubleSpinBox->setDecimals(0);
        fc_uDoubleSpinBox->setMaximum(1000.000000000000000);
        fc_uDoubleSpinBox->setValue(100.000000000000000);

        formLayout_4->setWidget(2, QFormLayout::FieldRole, fc_uDoubleSpinBox);


        horizontalLayout->addWidget(groupBox);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        groupBox_3 = new QGroupBox(Dialog);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        gridLayout_5 = new QGridLayout(groupBox_3);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        Kc0DoubleSpinBox = new QDoubleSpinBox(groupBox_3);
        Kc0DoubleSpinBox->setObjectName(QString::fromUtf8("Kc0DoubleSpinBox"));
        Kc0DoubleSpinBox->setSingleStep(0.100000000000000);

        gridLayout_5->addWidget(Kc0DoubleSpinBox, 1, 1, 1, 1);

        label_11 = new QLabel(groupBox_3);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_5->addWidget(label_11, 1, 0, 1, 1);

        Kc1DoubleSpinBox = new QDoubleSpinBox(groupBox_3);
        Kc1DoubleSpinBox->setObjectName(QString::fromUtf8("Kc1DoubleSpinBox"));
        Kc1DoubleSpinBox->setSingleStep(0.100000000000000);

        gridLayout_5->addWidget(Kc1DoubleSpinBox, 1, 2, 1, 1);

        Kc2DoubleSpinBox = new QDoubleSpinBox(groupBox_3);
        Kc2DoubleSpinBox->setObjectName(QString::fromUtf8("Kc2DoubleSpinBox"));
        Kc2DoubleSpinBox->setSingleStep(0.100000000000000);

        gridLayout_5->addWidget(Kc2DoubleSpinBox, 1, 3, 1, 1);

        label_10 = new QLabel(groupBox_3);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout_5->addWidget(label_10, 0, 1, 1, 1);

        Bc2DoubleSpinBox = new QDoubleSpinBox(groupBox_3);
        Bc2DoubleSpinBox->setObjectName(QString::fromUtf8("Bc2DoubleSpinBox"));
        Bc2DoubleSpinBox->setSingleStep(0.010000000000000);

        gridLayout_5->addWidget(Bc2DoubleSpinBox, 2, 3, 1, 1);

        Bc0DoubleSpinBox = new QDoubleSpinBox(groupBox_3);
        Bc0DoubleSpinBox->setObjectName(QString::fromUtf8("Bc0DoubleSpinBox"));
        Bc0DoubleSpinBox->setSingleStep(0.010000000000000);

        gridLayout_5->addWidget(Bc0DoubleSpinBox, 2, 1, 1, 1);

        Bc1DoubleSpinBox = new QDoubleSpinBox(groupBox_3);
        Bc1DoubleSpinBox->setObjectName(QString::fromUtf8("Bc1DoubleSpinBox"));
        Bc1DoubleSpinBox->setSingleStep(0.010000000000000);

        gridLayout_5->addWidget(Bc1DoubleSpinBox, 2, 2, 1, 1);

        label_12 = new QLabel(groupBox_3);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        gridLayout_5->addWidget(label_12, 0, 2, 1, 1);

        label_13 = new QLabel(groupBox_3);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        gridLayout_5->addWidget(label_13, 0, 3, 1, 1);

        label_14 = new QLabel(groupBox_3);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        gridLayout_5->addWidget(label_14, 2, 0, 1, 1);


        horizontalLayout_2->addWidget(groupBox_3);

        formGroupBox = new QGroupBox(Dialog);
        formGroupBox->setObjectName(QString::fromUtf8("formGroupBox"));
        formLayout = new QFormLayout(formGroupBox);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        MhDoubleSpinBox = new QDoubleSpinBox(formGroupBox);
        MhDoubleSpinBox->setObjectName(QString::fromUtf8("MhDoubleSpinBox"));
        MhDoubleSpinBox->setDecimals(1);
        MhDoubleSpinBox->setMaximum(10.000000000000000);
        MhDoubleSpinBox->setSingleStep(0.100000000000000);

        formLayout->setWidget(1, QFormLayout::FieldRole, MhDoubleSpinBox);

        mholderLabel = new QLabel(formGroupBox);
        mholderLabel->setObjectName(QString::fromUtf8("mholderLabel"));

        formLayout->setWidget(1, QFormLayout::LabelRole, mholderLabel);


        horizontalLayout_2->addWidget(formGroupBox);


        verticalLayout->addLayout(horizontalLayout_2);

        formLayout_5 = new QFormLayout();
        formLayout_5->setObjectName(QString::fromUtf8("formLayout_5"));
        formLayout_5->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        samplingPeriodLabel = new QLabel(Dialog);
        samplingPeriodLabel->setObjectName(QString::fromUtf8("samplingPeriodLabel"));

        formLayout_5->setWidget(0, QFormLayout::LabelRole, samplingPeriodLabel);

        periodSpinBox = new QSpinBox(Dialog);
        periodSpinBox->setObjectName(QString::fromUtf8("periodSpinBox"));
        periodSpinBox->setMinimum(10);
        periodSpinBox->setMaximum(10000);
        periodSpinBox->setSingleStep(10);
        periodSpinBox->setValue(1000);

        formLayout_5->setWidget(0, QFormLayout::FieldRole, periodSpinBox);

        feedforwardLabel = new QLabel(Dialog);
        feedforwardLabel->setObjectName(QString::fromUtf8("feedforwardLabel"));

        formLayout_5->setWidget(1, QFormLayout::LabelRole, feedforwardLabel);

        feedforwardCheckBox = new QCheckBox(Dialog);
        feedforwardCheckBox->setObjectName(QString::fromUtf8("feedforwardCheckBox"));

        formLayout_5->setWidget(1, QFormLayout::FieldRole, feedforwardCheckBox);


        verticalLayout->addLayout(formLayout_5);

        buttonBox = new QDialogButtonBox(Dialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        verticalLayout->addWidget(buttonBox);

        QWidget::setTabOrder(fc_qdDoubleSpinBox, fc_qddDoubleSpinBox);
        QWidget::setTabOrder(fc_qddDoubleSpinBox, fc_uDoubleSpinBox);
        QWidget::setTabOrder(fc_uDoubleSpinBox, periodSpinBox);
        QWidget::setTabOrder(periodSpinBox, feedforwardCheckBox);
        QWidget::setTabOrder(feedforwardCheckBox, buttonBox);

        retranslateUi(Dialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), Dialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), Dialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(Dialog);
    } // setupUi

    void retranslateUi(QDialog *Dialog)
    {
        Dialog->setWindowTitle(QCoreApplication::translate("Dialog", "Dialog", nullptr));
        groupBox->setTitle(QCoreApplication::translate("Dialog", "Cutoff Frequency", nullptr));
        qHzLabel->setText(QCoreApplication::translate("Dialog", "q [Hz]", nullptr));
        qdLabel->setText(QCoreApplication::translate("Dialog", "qd", nullptr));
        uLabel->setText(QCoreApplication::translate("Dialog", "u", nullptr));
        groupBox_3->setTitle(QCoreApplication::translate("Dialog", "Impedance", nullptr));
        label_11->setText(QCoreApplication::translate("Dialog", "Kc", nullptr));
        label_10->setText(QCoreApplication::translate("Dialog", "x", nullptr));
        label_12->setText(QCoreApplication::translate("Dialog", "y", nullptr));
        label_13->setText(QCoreApplication::translate("Dialog", "z", nullptr));
        label_14->setText(QCoreApplication::translate("Dialog", "Bc", nullptr));
        formGroupBox->setTitle(QCoreApplication::translate("Dialog", "weight", nullptr));
        MhDoubleSpinBox->setSuffix(QCoreApplication::translate("Dialog", "kg", nullptr));
        mholderLabel->setText(QCoreApplication::translate("Dialog", "Mholder", nullptr));
        samplingPeriodLabel->setText(QCoreApplication::translate("Dialog", "sampling period", nullptr));
        periodSpinBox->setSuffix(QCoreApplication::translate("Dialog", " [ms]", nullptr));
        periodSpinBox->setPrefix(QString());
        feedforwardLabel->setText(QCoreApplication::translate("Dialog", "feedforward", nullptr));
        feedforwardCheckBox->setText(QCoreApplication::translate("Dialog", "feedforward", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Dialog: public Ui_Dialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PARAMETER_H
