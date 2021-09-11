#include "LasSaveDialog.h"
#include "LasDetails.h"

#include <QStringListModel>

#include <ccLog.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>

LasSaveDialog::LasSaveDialog(ccPointCloud *cloud, QWidget *parent)
    : QDialog(parent), m_cloud(cloud), m_model(new QStringListModel)
{
    setupUi(this);
    origRadioButton_2->setEnabled(false);
    customScaleDoubleSpinBox_2->setEnabled(true);
    bestRadioButton_2->setChecked(true);

    connect(versionComboBox,
            (void (QComboBox::*)(const QString &))(&QComboBox::currentIndexChanged),
            this,
            &LasSaveDialog::handleSelectedVersionChange);

    connect(pointFormatComboBox,
            (void (QComboBox::*)(int))(&QComboBox::currentIndexChanged),
            this,
            &LasSaveDialog::handleSelectedPointFormatChange);

    for (const char *versionStr : AvailableVersions)
    {
        versionComboBox->addItem(versionStr);
    }
    versionComboBox->setCurrentIndex(0);

    m_cloudScalarFieldsNames << QString();
    for (unsigned int i = 0; i < m_cloud->getNumberOfScalarFields(); ++i)
    {
        if (strcmp(m_cloud->getScalarFieldName(i), "Default") != 0)
        {
            m_cloudScalarFieldsNames << m_cloud->getScalarFieldName(i);
        }
    }
    m_model->setStringList(m_cloudScalarFieldsNames);
}

/// When the selected version changes, we need to update the combo box
/// of point format to match the ones supported by the version
void LasSaveDialog::handleSelectedVersionChange(const QString &version)
{
    const std::vector<unsigned int> *pointFormats = PointFormatsAvailableForVersion(qPrintable(version));
    if (pointFormats)
    {
        pointFormatComboBox->clear();
        for (unsigned int fmt : *pointFormats)
        {
            pointFormatComboBox->addItem(QString::number(fmt));
        }
    }
    pointFormatComboBox->setCurrentIndex(0);
}

/// When the user changes the point format, we need to redo the
/// scalar field form.
void LasSaveDialog::handleSelectedPointFormatChange(int index)
{

    const std::vector<unsigned int> *pointFormats =
        PointFormatsAvailableForVersion(qPrintable(versionComboBox->currentText()));
    if (!pointFormats)
    {
        Q_ASSERT(false);
        return;
    }

    if (index < 0)
    {
        return;
    }

    unsigned int selectedPointFormat = (*pointFormats)[index];

    m_userInput.clear();
    while (!scalarFieldFormLayout->isEmpty())
    {
        scalarFieldFormLayout->removeRow(0);
    }

    std::vector<LasScalarField> lasScalarFields = LasScalarFieldForPointFormat(selectedPointFormat);
    for (auto &lasScalarField : lasScalarFields)
    {
        auto *box = new QComboBox;
        box->setModel(m_model);
        box->setCurrentIndex(m_cloudScalarFieldsNames.indexOf(lasScalarField.name));
        scalarFieldFormLayout->addRow(lasScalarField.name, box);
        m_userInput.emplace_back(lasScalarField.name, box);
    }
}

void LasSaveDialog::setVersionAndPointFormat(const QString &version, unsigned int pointFormat)
{
    int i = versionComboBox->findText(version);
    if (i >= 0)
    {
        QString fmtStr = QString::number(pointFormat);
        versionComboBox->setCurrentIndex(i);
        int j = pointFormatComboBox->findText(fmtStr);
        if (j >= 0)
        {
            pointFormatComboBox->setCurrentIndex(j);
        }
    }
}

void LasSaveDialog::setOptimalScale(const CCVector3d &optimalScale)
{
    bestAccuracyLabel_2->setText(
        QString("(%1, %2, %3)").arg(optimalScale.x).arg(optimalScale.y).arg(optimalScale.z));
}

void LasSaveDialog::setSavedScale(const CCVector3d &savedScale)
{
    origAccuracyLabel_2->setText(
        QString("(%1, %2, %3)").arg(savedScale.x).arg(savedScale.y).arg(savedScale.z));
    origRadioButton_2->setEnabled(true);
    origRadioButton_2->setChecked(true);
}

unsigned int LasSaveDialog::selectedPointFormat() const
{
    return pointFormatComboBox->currentText().toUInt();
}

unsigned int LasSaveDialog::selectedVersionMinor() const
{
    return versionComboBox->currentText().splitRef('.').at(1).toUInt();
}

CCVector3d LasSaveDialog::chosenScale() const
{
    const auto vectorFromString = [](const QString &string) -> CCVector3d {
        QVector<QStringRef> splits = string.splitRef(',');
        if (splits.size() == 3)
        {
            double x = splits[0].right(splits[0].size() - 1).toDouble();
            double y = splits[1].toDouble();
            double z = splits[2].left(splits[2].size() - 1).toDouble();
            return {x, y, z};
        }
        return {};
    };
    if (bestRadioButton_2->isChecked())
    {
        QString text = bestAccuracyLabel_2->text();
        return vectorFromString(text);
    }
    else if (origRadioButton_2->isChecked())
    {
        QString text = origAccuracyLabel_2->text();
        return vectorFromString(text);
    }
    else if (customRadioButton_2->isChecked())
    {
        double value = customScaleDoubleSpinBox_2->value();
        return {value, value, value};
    }
    return {};
}

std::vector<LasScalarField> LasSaveDialog::fieldsToSave() const
{
    std::vector<LasScalarField> fields;
    fields.reserve(scalarFieldFormLayout->rowCount());
    unsigned int pointFormat = selectedPointFormat();

    for (const auto &item : m_userInput)
    {
        ccScalarField *sf = nullptr;
        if (item.second->currentIndex() >= 0)
        {
            sf = static_cast<ccScalarField *>(m_cloud->getScalarField(
                m_cloud->getScalarFieldIndexByName(qPrintable(item.second->currentText()))));
        }

        fields.emplace_back(item.first, LasScalarField::IdFromName(item.first, pointFormat), sf);
    }

    return fields;
}
