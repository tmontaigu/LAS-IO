//##########################################################################
//#                                                                        #
//#                           ExampleIOPlugin                              #
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
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#pragma once

#include "FileIOFilter.h"

class LASIOFilter : public FileIOFilter
{
  public:
    LASIOFilter();

    // Inherited from FileIOFilter
    CC_FILE_ERROR
    loadFile(const QString &fileName, ccHObject &container, LoadParameters &parameters) override;

    bool canSave(CC_CLASS_ENUM type, bool &multiple, bool &exclusive) const override;
    CC_FILE_ERROR
    saveToFile(ccHObject *entity, const QString &filename, const SaveParameters &parameters) override;
};
