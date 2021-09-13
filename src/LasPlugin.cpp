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

#include "LasPlugin.h"

#include "LasIOFilter.h"

LasPlugin::LasPlugin(QObject *parent) : QObject(parent), ccIOPluginInterface(":/CC/plugin/LAS-IO/info.json")
{
}

ccIOPluginInterface::FilterList LasPlugin::getFilters()
{
    return {
        FileIOFilter::Shared(new LasIOFilter),
    };
}
