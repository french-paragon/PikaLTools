#include "exportorthophotooptionsdialog.h"
#include "ui_exportorthophotooptionsdialog.h"

#include <steviapp/control/mainwindow.h>
#include <steviapp/datablocks/project.h>

#include "datablocks/inputdtm.h"

#include <QFileDialog>

namespace PikaLTools {

ExportOrthoPhotoOptionsDialog::ExportOrthoPhotoOptionsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ExportOrthoPhotoOptionsDialog)
{
    ui->setupUi(this);
    configureDtmList();
    configureOptimizationExportOption();

    ui->outFileLineEdit->setReadOnly(true);
    connect(ui->openOutFileButton, &QPushButton::clicked, this, &ExportOrthoPhotoOptionsDialog::selectOutFile);

    ui->minLineSpinBox->setMinimum(-1);
    ui->maxLineSpinBox->setMinimum(-1);
}

ExportOrthoPhotoOptionsDialog::~ExportOrthoPhotoOptionsDialog()
{
    delete ui;
}

qint64 ExportOrthoPhotoOptionsDialog::selectedDtmIdx() const {
    return _dtm_idxs[ui->dtmComboBox->currentIndex()];
}

QString ExportOrthoPhotoOptionsDialog::outFile() const {
    return ui->outFileLineEdit->text();
}

bool ExportOrthoPhotoOptionsDialog::useOptimizedTrajectory() const {
    return ui->selectedTrajectoryComboBox->currentData().toBool();
}
bool ExportOrthoPhotoOptionsDialog::useOptimizedCamera() const {
    return ui->selectedCameraComboBox->currentData().toBool();
}
bool ExportOrthoPhotoOptionsDialog::useOptimizedLeverArm() const {
    return ui->selectedLeverArmComboBox->currentData().toBool();
}

int ExportOrthoPhotoOptionsDialog::minLineId() const {
    if (ui->lineLimitGroupBox->isChecked()) {
        return ui->minLineSpinBox->value();
    }
    return -1;
}
int ExportOrthoPhotoOptionsDialog::maxLineId() const {
    if (ui->lineLimitGroupBox->isChecked()) {
        return ui->maxLineSpinBox->value();
    }
    return -1;
}

double ExportOrthoPhotoOptionsDialog::getTargetGSD() const {
    return ui->gsdSpinBox->value();
}

void ExportOrthoPhotoOptionsDialog::setLineFileId(int nLines) {
    ui->minLineSpinBox->setMaximum(nLines-1);
    ui->maxLineSpinBox->setMaximum(nLines);
}

void ExportOrthoPhotoOptionsDialog::configureDtmList() {

    QWidget* current = parentWidget();
    StereoVisionApp::MainWindow* mw = nullptr;

    while (mw == nullptr) {
        if (current == nullptr) {
            return;
        }

        mw = qobject_cast<StereoVisionApp::MainWindow*>(current);

        current = current->parentWidget();
    }

    StereoVisionApp::Project* proj = mw->activeProject();

    if (proj == nullptr) {
        return;
    }

    ui->dtmComboBox->clear();
    _dtm_idxs.clear();
    _dtm_names.clear();

    _dtm_idxs = proj->getIdsByClass(InputDtm::staticMetaObject.className());

    for (qint64 idx : _dtm_idxs) {
        InputDtm* dtm = proj->getDataBlock<InputDtm>(idx);

        if (dtm == nullptr) {
            return;
        }

        _dtm_names.push_back(dtm->objectName());
    }

    ui->dtmComboBox->addItems(_dtm_names.toList());

}


void ExportOrthoPhotoOptionsDialog::configureOptimizationExportOption() {

    ui->selectedTrajectoryComboBox->clear();

    ui->selectedTrajectoryComboBox->addItem(tr("Use optimized trajectory"), QVariant(true));
    ui->selectedTrajectoryComboBox->addItem(tr("Use initial trajectory"), QVariant(false));

    ui->selectedTrajectoryComboBox->setCurrentIndex(0);

    ui->selectedTrajectoryComboBox->setEditable(false);

    ui->selectedCameraComboBox->clear();

    ui->selectedCameraComboBox->addItem(tr("Use optimized camera"), QVariant(true));
    ui->selectedCameraComboBox->addItem(tr("Use initial camera"), QVariant(false));

    ui->selectedCameraComboBox->setCurrentIndex(0);

    ui->selectedCameraComboBox->setEditable(false);

    ui->selectedLeverArmComboBox->clear();

    ui->selectedLeverArmComboBox->addItem(tr("Use optimized lever arm"), QVariant(true));
    ui->selectedLeverArmComboBox->addItem(tr("Use initial lever arm"), QVariant(false));

    ui->selectedLeverArmComboBox->setCurrentIndex(0);

    ui->selectedLeverArmComboBox->setEditable(false);

}

void ExportOrthoPhotoOptionsDialog::selectOutFile() {

    QString outFile = QFileDialog::getSaveFileName(this, tr("Save orthophoto to"), "", "steviapp images (*.stevimg)");

    if (!outFile.isEmpty()) {
        ui->outFileLineEdit->setText(outFile);
    }

}

} // namespace PikaLTools
