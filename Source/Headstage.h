/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2021 Open Ephys

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


#ifndef __HEADSTAGE_H_2C4CBD67__
#define __HEADSTAGE_H_2C4CBD67__

#include <DataThreadHeaders.h>

#include <stdio.h>
#include <string.h>
#include <array>
#include <atomic>

#include "DeviceThread.h"

namespace RhythmNode
{


	/** 
		A headstage object represents a data source containing 
		one or more Intan chips.

		Each headstage can send 1 or 2 data streams, each with
		up to 64 channels.

		A headstage can be identified in the following ways:
		- dataSource   : port (A1-D2Ddr) that the headstage is connected to
		- streamIndex  : location in the array of active streams
		- startChannel : index of the first channel acquired by this headstage,
						 out of all actively acquired neural data channels

	*/
	class Headstage
	{
	public:

		/** Constructor */
		Headstage(int headstageIndex, int maxNumHeadstages);

		/** Destructor*/
		~Headstage() { }

		/** Returns the index of this headstage's data stream (offset = 0 or 1) */
		int getStreamIndex(int offset)  const;

		/** Sets the index of this headstage's data stream */
		void setFirstStreamIndex(int streamIndex);

		/** Sets the index of this headstage's first neural data channel*/
		void setFirstChannel(int channelIndex);

		/** Returns the number of channels this headstage sends*/
		int getNumChannels()            const;

		/** Sets the number of channels per stream*/
		void setChannelsPerStream(int nchan);

		/** Returns the number of streams this headstage sends*/
		int getNumStreams()             const;

		/** Sets the number of streams for this headstage (1 or 2)*/
		void setNumStreams(int num);

		/** Returns the number of actively acquired neural data channels*/
		int getNumActiveChannels()      const;

		/** Returns the name of a channel at a given index*/
		String getChannelName(int ch) const;

		/** Returns the prefix for this headstage's data stream*/
		String getStreamPrefix() const;

		/** Sets the name of a channel at a given index*/
		void setChannelName(String name, int ch);

		/** Sets the channel naming scheme*/
		void setNamingScheme(ChannelNamingScheme scheme);

		/** Returns true if the headstage is connected*/
		bool isConnected() const;

		/** Returns the stream number for a given index*/
		int getDataStream(int index) const;

		/** Sets the number of half-channels; mainly used for the 16-ch RHD2132 board */
		void setHalfChannels(bool half); 

		/** Auto-generates the channel names, based on the naming scheme*/
		void generateChannelNames();

		/** Sets impedance values after measurement*/
		void setImpedances(Impedances& impedances);

		/** Returns the impedance magnitude for a channel (if it exists)*/
		float getImpedanceMagnitude(int channel) const;

		/** Returns the impedance phase for a channel (if it exists)*/
		float getImpedancePhase(int channel) const;

		/** Returns true if impedance has been measured*/
		bool hasImpedanceData() const { return impedanceMagnitudes.size() > 0; }

	private:
		int headstageIndex;
		int maxNumHeadstages;

		int streamIndex;
		int firstChannelIndex;
		
		int numStreams;
		int channelsPerStream;
		
		bool halfChannels;

		ChannelNamingScheme namingScheme;

		StringArray channelNames;
		String prefix;

		Array<float> impedanceMagnitudes;
		Array<float> impedancePhases;

		JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(Headstage);
	};


}
#endif  // __HEADSTAGE_H_2C4CBD67__
