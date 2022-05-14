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


#ifndef __DEVICETHREAD_H_2C4CBD67__
#define __DEVICETHREAD_H_2C4CBD67__

#include <DataThreadHeaders.h>

#include <stdio.h>
#include <string.h>
#include <array>
#include <atomic>

#define MAX_NUM_CHANNELS MAX_NUM_DATA_STREAMS * 35

#define MAX_NUM_HEADSTAGES MAX_NUM_DATA_STREAMS / 2

#include "AbstractDevice.hpp"

namespace RhythmNode
{

	class Headstage;
	class ImpedanceMeter;

	enum BoardType
	{
		ACQUISITION_BOARD,
		INTAN_RHD_USB,
        ONI_USB,
		RHD_RECORDING_CONTROLLER,
        RHS_STIM_RECORDING_CONTROLLER
	};

	enum ChannelNamingScheme
	{
		GLOBAL_INDEX = 1,
		STREAM_INDEX = 2
	};

	/**
		Communicates with a device running Rhythm or ONI firmware

		@see DataThread, SourceNode
	*/
	class DeviceThread : public DataThread
	{
		friend class ImpedanceMeter;

	public:
        
        /** DataThread factory */
        static DataThread* createDataThread(SourceNode* sn);

        /** Stores the board type (ACQUISITION_BOARD, ONI_USB, etc.) */
        static BoardType boardType;
        
		/** Constructor; must specify the type of board used */
		DeviceThread(SourceNode* sn, BoardType boardType);

		/** Destructor */
		~DeviceThread();

		/** Creates the UI for this plugin */
		std::unique_ptr<GenericEditor> createEditor(SourceNode* sn) override;

		/** Fills the DataBuffer with incoming data */
		bool updateBuffer() override;

		/** Initializes data transfer*/
		bool startAcquisition() override;

		/** Stops data transfer */
		bool stopAcquisition() override;
        
        /** Informs the DataThread about whether to expect saved settings to be loaded*/
        void initialize(bool signalChainIsLoading) override;

		/* Passes the processor's info objects to DataThread, to allow them to be configured */
		void updateSettings(OwnedArray<ContinuousChannel>* continuousChannels,
			OwnedArray<EventChannel>* eventChannels,
			OwnedArray<SpikeChannel>* spikeChannels,
			OwnedArray<DataStream>* sourceStreams,
			OwnedArray<DeviceInfo>* devices,
			OwnedArray<ConfigurationObject>* configurationObjects) override;

		/** Updates the measured impedance values for each channel*/
		void impedanceMeasurementFinished();

		/** Returns an array of connected headstages*/
		Array<const Headstage*> getConnectedHeadstages();

		/** Sets the method for determining channel names*/
		void setNamingScheme(ChannelNamingScheme scheme);

		/** Gets the method for determining channel names*/
		ChannelNamingScheme getNamingScheme();

		/** Allow the thread to respond to messages sent by other plugins */
		void handleBroadcastMessage(String msg) override;

        /** Sets the channel count for a particular headstage */
		void setNumChannels(int hsNum, int nChannels);

        /** Returns the total number of channels for this device */
		int getNumChannels();

        /** Returns the total number of channels for a given type (HEADSTAGE, AUX, ADC) */
		int getNumDataOutputs(ContinuousChannel::Type type);

        /** Returns true if a particular headstage is enabled*/
		bool isHeadstageEnabled(int hsNum) const;

        /** Returns the channel count for a particular headstage */
		int getChannelsInHeadstage(int hsNum) const;

		/* Gets the absolute channel index from the headstage channel index*/
		int getChannelFromHeadstage(int hs, int ch);

		/*Gets the headstage relative channel index from the absolute channel index*/
		int getHeadstageChannel(int& hs, int ch) const;

		// for communication with SourceNode processors:
		bool foundInputSource() override;

        /** Scans all available ports for active headstages */
		void scanPorts();

        /** Saves measured impedances to a file */
		void saveImpedances(File& file);

		String getChannelName(int ch) const;

		float getAdcBitVolts(int channelNum) const;

		void setSampleRate(int index, bool temporary = false);

		double setUpperBandwidth(double upper); // set desired BW, returns actual BW
		double setLowerBandwidth(double lower);

		double setDspCutoffFreq(double freq);
		double getDspCutoffFreq() const;

		void setDSPOffset(bool state);

		int setNoiseSlicerLevel(int level);
		void setFastTTLSettle(bool state, int channel);
		void setTTLoutputMode(bool state);
		void setDAChpf(float cutoff, bool enabled);

		void enableAuxs(bool);
		void enableAdcs(bool);

		int TTL_OUTPUT_STATE[16];

		bool isAuxEnabled();
		bool isAcquisitionActive() const;

		Array<int> getDACchannels() const;

		void setDACchannel(int dacOutput, int channel);
		void setDACthreshold(int dacOutput, float threshold);

		int getHeadstageChannels(int hsNum) const;
		int getActiveChannelsInHeadstage(int hsNum) const;

		void runImpedanceTest();

		void enableBoardLeds(bool enable);

		int setClockDivider(int divide_ratio);

		void setAdcRange(int adcChannel, short rangeType);

		short getAdcRange(int adcChannel) const;



        /**
            A timer that stores the on/off times of digital output events
         */
		class DigitalOutputTimer : public Timer
		{
		public:

			/** Constructor */
			DigitalOutputTimer(DeviceThread*, int tllLine, int eventDurationMs);

			/** Destructor*/
			~DigitalOutputTimer() { }

			/** Sends signal to turn off event channel*/
			void timerCallback();

		private:
			DeviceThread* board;

			int tllOutputLine;
		};

		struct DigitalOutputCommand {
			int ttlLine;
			bool state;
		};

		void addDigitalOutputCommand(DigitalOutputTimer* timerToDelete,
			int ttlLine,
			bool state);

	private:
        
        /** Pointer to the AbstractDevice object */
        std::unique_ptr<AbstractDevice> device;
        
        /** Vector of headstage objects*/
        std::vector<Headstage> headstages;

        /** Queue for digital output commands */
		std::queue<DigitalOutputCommand> digitalOutputCommands;

        /** Array of digital output times */
		OwnedArray<DigitalOutputTimer> digitalOutputTimers;
        
        /** Thread for impedance measurement */
        ScopedPointer<ImpedanceMeter> impedanceThread;

		bool enableHeadstage(int hsNum, bool enabled, int nStr = 1, int strChans = 32);
		void updateBoardStreams();
		void setCableLength(int hsNum, float length);

		ScopedPointer<Rhd2000DataBlock> dataBlock;
		Array<Rhd2000EvalBoard::BoardDataSource> enabledStreams;

		/** True if device is available*/
		bool deviceFound;

		/** True if data is streaming*/
		bool isTransmitting;

		/** True if change in settings is needed during acquisition*/
		bool updateSettingsDuringAcquisition;

		int* dacChannels, *dacStream;
		float* dacThresholds;
		bool* dacChannelsToUpdate;
		Array<int> chipId;

		Array<int> numChannelsPerDataStream;

		ChannelNamingScheme channelNamingScheme;

		/** ADC info */
		std::array<std::atomic_short, 8> adcRangeSettings;
		Array<float> adcBitVolts;
		StringArray adcChannelNames;
		StringArray ttlLineNames;

		/** Impedance data*/
		Impedances impedances;

        /** Stores names of channels */
		StringArray channelNames;

		JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(DeviceThread);
	};

}
#endif  // __DEVICETHREAD_H_2C4CBD67__
