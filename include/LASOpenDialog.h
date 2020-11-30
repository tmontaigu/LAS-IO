//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef CC_LAS_OPEN_DIALOG
#define CC_LAS_OPEN_DIALOG

#include "LASDetails.h"

// GUIs generated by Qt Designer
#include <ui_lasopendialog.h>

// system
#include <string>
#include <vector>

// CCCoreLib
#include <CCGeom.h>
#include <ccLog.h>

//! Dialog to choose the LAS fields to load
class LASOpenDialog : public QDialog, public Ui::LASOpenDialog
{
    Q_OBJECT

  public:
    //! Default constructor
    explicit LASOpenDialog(QWidget *parent = nullptr);

    void setInfo(int versionMinor, int pointFormatId, int64_t numPoints);

    void setAvailableScalarFields(const std::vector<LasScalarField> &scalarFields,
                                  const std::vector<LasExtraScalarField> &extraScalarFields);

    void filterOutNotChecked(std::vector<LasScalarField> &scalarFields,
                             std::vector<LasExtraScalarField> &extraScalarFields);

    bool isChecked(const LasScalarField &lasScalarField) const;

    bool isChecked(const LasExtraScalarField &lasExtraScalarField) const;
};

#endif // CC_LAS_OPEN_DIALOG
