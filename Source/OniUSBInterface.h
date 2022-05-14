/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2020 Open Ephys

    ------------------------------------------------------------------

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef __ONIUSB_H_2C4CBD67__
#define __ONIUSB_H_2C4CBD67__

#include "DeviceThread.h"

namespace RhythmNode
{

    /**
        Communicates with devices powered by the Open Neural Interface (ONI) USB board

        https://github.com/open-ephys/ECP5U85-BSE-USB

        @see DataThread, SourceNode
        */

    class OniUSBInterface : public DeviceThread
    {
    public:

        OniUSBInterface(SourceNode* sn) : DeviceThread(sn, ONI_USB) { }

        ~OniUSBInterface() { }
    };

}

#endif

