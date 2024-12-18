#ifndef PIKALTOOLS_EXPORTORTHOPHOTOOPTIONSDIALOG_H
#define PIKALTOOLS_EXPORTORTHOPHOTOOPTIONSDIALOG_H

#include <QDialog>

namespace PikaLTools {

namespace Ui {
class ExportOrthoPhotoOptionsDialog;
}

class ExportOrthoPhotoOptionsDialog : public QDialog
{
    Q_OBJECT

public:

    explicit ExportOrthoPhotoOptionsDialog(QWidget *parent = nullptr);
    ~ExportOrthoPhotoOptionsDialog();

    qint64 selectedDtmIdx() const;

    QString outFile() const;

    int minLineId() const;
    int maxLineId() const;

    void setLineFileId(int nLines);

    double getTargetGSD() const;

private:

    void configureDtmList();
    void selectOutFile();

    QVector<qint64> _dtm_idxs;
    QVector<QString> _dtm_names;

    Ui::ExportOrthoPhotoOptionsDialog *ui;
};


} // namespace PikaLTools
#endif // PIKALTOOLS_EXPORTORTHOPHOTOOPTIONSDIALOG_H
