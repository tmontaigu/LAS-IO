#ifndef LASSAVEDIALOG_H
#define LASSAVEDIALOG_H

#include <QDialog>

#include <CCGeom.h>

#include "LasDetails.h"
#include "ui_lassavedialog.h"

class QStringListModel;
class ccScalarField;
class ccPointCloud;

class LasSaveDialog : public QDialog, public Ui::LASSaveDialog
{
    Q_OBJECT

  public:
    explicit LasSaveDialog(ccPointCloud *cloud, QWidget *parent = nullptr);

    void setVersionAndPointFormat(const QString &version, unsigned int pointFormat);
    void setOptimalScale(const CCVector3d &optimalScale);
    void setSavedScale(const CCVector3d &savedScale);

    unsigned int selectedPointFormat() const;
    unsigned int selectedVersionMinor() const;
    CCVector3d chosenScale() const;

    std::vector<LasScalarField> fieldsToSave() const;

  public Q_SLOTS:
    void handleSelectedVersionChange(const QString &);
    void handleSelectedPointFormatChange(int index);

  private:
    ccPointCloud *m_cloud{nullptr};
    QStringList m_cloudScalarFieldsNames;
    QStringListModel *m_model{nullptr};
    std::vector<std::pair<const char *, QComboBox *>> m_userInput;
};

#endif // LASSAVEDIALOG_H
