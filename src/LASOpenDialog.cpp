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

#include "LASOpenDialog.h"

static QListWidgetItem* createItem(const LasScalarField& lasScalarField) {
    auto item = new QListWidgetItem(lasScalarField.name);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Checked);
    return item;
}

LASOpenDialog::LASOpenDialog(QWidget *parent) : QDialog(parent) {
    setupUi(this);

    connect(applyButton, &QPushButton::clicked, this, &QDialog::accept);
    connect(applyAllButton, &QPushButton::clicked, this, &QDialog::accept);
    connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
}

void LASOpenDialog::setAvailableScalarFields(const std::vector<LasScalarField> &scalarFields)
{
    for (const LasScalarField& lasScalarField : scalarFields) {
        availableScalarFields->addItem(createItem(lasScalarField));
    }
}

bool LASOpenDialog::isChecked(const LasScalarField& lasScalarField) const {
    for (int i = 0; i < availableScalarFields->count(); ++i)
    {
        if (availableScalarFields->item(i)->text() == lasScalarField.name) {
            return availableScalarFields->item(i)->checkState() == Qt::Checked;
        }
    }
    return false;
}