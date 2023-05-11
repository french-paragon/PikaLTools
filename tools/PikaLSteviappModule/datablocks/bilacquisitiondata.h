#ifndef PIKALTOOLS_BILACQUISITIONDATA_H
#define PIKALTOOLS_BILACQUISITIONDATA_H

#include <steviapp/datablocks/project.h>

#include <QMap>
#include <QString>
#include <QList>
#include <QJsonObject>

namespace PikaLTools {

class BilSequenceAcquisitionData : public StereoVisionApp::DataBlock
{
    Q_OBJECT
public:
    BilSequenceAcquisitionData(StereoVisionApp::Project *parent = nullptr);

    class BilAcquisitionData {
    public:

        BilAcquisitionData(QString path = "");

        void setBilFilePath(QString filePath);

        QString bilFilePath() const;
        QString headerFilePath() const;
        QString lcfFilePath() const;

        QMap<QString, QString> headerData() const;

    protected:
        QString _bil_file_path;
    };

    void setBilSequence(const QList<QString> &bilFiles);
    QList<QString> getBilFiles() const;
    QList<BilAcquisitionData> getBilInfos() const;

    void clearOptimized() override;
    bool hasOptimizedParameters() const override;

Q_SIGNALS:

    void bilSequenceChanged();

protected:

    virtual QJsonObject encodeJson() const override;
    virtual void configureFromJson(QJsonObject const& data) override;

    void extendDataModel();

    QList<BilAcquisitionData> _bilSequence;
};

class BilSequenceAcquisitionDataFactory : public StereoVisionApp::DataBlockFactory
{
    Q_OBJECT
public:
    explicit BilSequenceAcquisitionDataFactory(QObject* parent = nullptr);

    virtual QString TypeDescrName() const;
    virtual FactorizableFlags factorizable() const;
    virtual StereoVisionApp::DataBlock* factorizeDataBlock(StereoVisionApp::Project *parent = nullptr) const;

    virtual QString itemClassName() const;
};

} // namespace PikaLTools

#endif // PIKALTOOLS_BILACQUISITIONDATA_H
