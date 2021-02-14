//##########################################################################
//#                                                                        #
//#          CLOUDCOMPARE PLUGIN: LasIO                                    #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#pragma once

#include "ccIOPluginInterface.h"


class LASPlugin : public QObject, public ccIOPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(ccPluginInterface ccIOPluginInterface)

    Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.LASIO" FILE "../info.json")

  public:
    explicit LASPlugin(QObject *parent = nullptr);
    ~LASPlugin() override = default;

    // Inherited from ccIOPluginInterface
    ccIOPluginInterface::FilterList getFilters() override;
};
