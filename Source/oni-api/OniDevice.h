/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2022 Open Ephys

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


#ifndef OniDevice_hpp
#define OniDevice_hpp

#include <stdio.h>

#include "oni.h"

#include "../rhx-api/Abstract/abstractrhxcontroller.h"

/**

    Class for communicating with an ONI Device
 
 */

class OniDevice : public AbstractRHXController
{

public:

    /** Constructor */
    OniDevice();

    /** Destructor */
    ~OniDevice();

    /** Returns true if device generates synthetic data */
    bool isSynthetic() const override { return false; }

    /** Opens the connection to the board */
    int open(const std::string& boardSerialNumber) override;

    /** Upload the configuration file (bitfile) to the FPGA. Return true if successful.*/
    bool uploadFPGABitfile(const std::string& filename) override;

    /** Reset FPGA. This clears all auxiliary command RAM banks, clears the USB FIFO, 
        and resets the per-channel samplingrate to 30.0 kS/s/ch. */
    void resetBoard() override;

    /** Initiate SPI data acquisition. */
    void run() override;

    /** Is the FPGA currently running?*/
    bool isRunning() override;

    /**  Flush all remaining data out of the FIFO.  
         This function should only be called when SPI data acquisition has been stopped. */
    void flush() override;
    void resetFpga() override;

    bool readDataBlock(RHXDataBlock* dataBlock) override;
    bool readDataBlocks(int numBlocks, std::deque<RHXDataBlock*>& dataQueue) override;
    long readDataBlocksRaw(int numBlocks, uint8_t* buffer) override;

    void setContinuousRunMode(bool continuousMode) override;
    void setMaxTimeStep(unsigned int maxTimeStep) override;
    void setCableDelay(BoardPort port, int delay) override;
    void setDspSettle(bool enabled) override;
    void setDataSource(int stream, BoardDataSource dataSource) override;  // used only with ControllerRecordUSB2
    void setTtlOut(const int* ttlOutArray) override;  // not used with ControllerStimRecordUSB2
    void setDacManual(int value) override;
    void setClockDivider(int divide_factor) override;
    void enableLeds(bool ledsOn) override;
    void setLedDisplay(const int* ledArray) override;
    void setSpiLedDisplay(const int* ledArray) override;  // not used with ControllerRecordUSB2
    void setDacGain(int gain) override;
    void setAudioNoiseSuppress(int noiseSuppress) override;
    void setExternalFastSettleChannel(int channel) override;             // not used with ControllerStimRecordUSB2
    void setExternalDigOutChannel(BoardPort port, int channel) override; // not used with ControllerStimRecordUSB2
    void setDacHighpassFilter(double cutoff) override;
    void setDacThreshold(int dacChannel, int threshold, bool trigPolarity) override;
    void setTtlMode(int mode) override;      // not used with ControllerStimRecordUSB2
    void setDacRerefSource(int stream, int channel) override;  // not used with ControllerRecordUSB2
    void setExtraStates(unsigned int extraStates) override;
    void setStimCmdMode(bool enabled) override;
    void setAnalogInTriggerThreshold(double voltageThreshold) override;
    void setManualStimTrigger(int trigger, bool triggerOn) override;
    void setGlobalSettlePolicy(bool settleWholeHeadstageA, bool settleWholeHeadstageB, bool settleWholeHeadstageC, bool settleWholeHeadstageD, bool settleAllHeadstages) override;
    void setTtlOutMode(bool mode1, bool mode2, bool mode3, bool mode4, bool mode5, bool mode6, bool mode7, bool mode8) override;
    void setAmpSettleMode(bool useFastSettle) override;
    void setChargeRecoveryMode(bool useSwitch) override;
    bool setSampleRate(AmplifierSampleRate newSampleRate) override;

    void enableDataStream(int stream, bool enabled) override;
    void enableDac(int dacChannel, bool enabled) override;
    void enableExternalFastSettle(bool enable) override;                 // not used with ControllerStimRecordUSB2
    void enableExternalDigOut(BoardPort port, bool enable) override;     // not used with ControllerStimRecordUSB2
    void enableDacHighpassFilter(bool enable) override;
    void enableDacReref(bool enabled) override;  // not used with ControllerRecordUSB2
    void enableDcAmpConvert(bool enable) override;
    void enableAuxCommandsOnAllStreams() override;
    void enableAuxCommandsOnOneStream(int stream) override;

    void selectDacDataStream(int dacChannel, int stream) override;
    void selectDacDataChannel(int dacChannel, int dataChannel) override;
    void selectAuxCommandLength(AuxCmdSlot auxCommandSlot, int loopIndex, int endIndex) override;
    void selectAuxCommandBank(BoardPort port, AuxCmdSlot auxCommandSlot, int bank) override; // not used with ControllerStimRecordUSB2

    int getBoardMode() override;
    int getNumSPIPorts(bool& expanderBoardDetected) override;

    void clearTtlOut() override;                 // not used with ControllerStimRecordUSB2
    void resetSequencers() override;
    void programStimReg(int stream, int channel, StimRegister reg, int value) override;
    void uploadCommandList(const std::vector<unsigned int>& commandList, AuxCmdSlot auxCommandSlot, int bank = 0) override;

    int findConnectedChips(std::vector<ChipType>& chipType, std::vector<int>& portIndex, std::vector<int>& commandStream,
        std::vector<int>& numChannelsOnPort) override;

private:
    // Objects of this class should not be copied.  Disable copy and assignment operators.
    OniDevice(const OniDevice&);            // declaration only
    OniDevice& operator=(const OniDevice&); // declaration only

    unsigned int numWordsInFifo() override;
    bool isDcmProgDone() const override;
    bool isDataClockLocked() const override;
    void forceAllDataStreamsOff() override;
    
    oni_ctx ctx;
    
    enum Rhythm_Registers
    {
        ENABLE = 0,
        MODE,
        MAX_TIMESTEP,
        CABLE_DELAY,
        AUXCMD_BANK_1,
        AUXCMD_BANK_2,
        AUXCMD_BANK_3,
        MAX_AUXCMD_INDEX_1,
        MAX_AUXCMD_INDEX_2,
        MAX_AUXCMD_INDEX_3,
        LOOP_AUXCMD_INDEX_1,
        LOOP_AUXCMD_INDEX_2,
        LOOP_AUXCMD_INDEX_3,
        DATA_STREAM_1, // - 8_SEL,
        DATA_STREAM_9, // - 16_SEL,
        DATA_STREAM_EN,
        EXTERNAL_FAST_SETTLE,
        EXTERNAL_DIGOUT_A,
        EXTERNAL_DIGOUT_B,
        EXTERNAL_DIGOUT_C,
        EXTERNAL_DIGOUT_D,
        SYNC_CLKOUT_DIVIDE,
        DAC_CTL,
        DAC_SEL_1,
        DAC_SEL_2,
        DAC_SEL_3,
        DAC_SEL_4,
        DAC_SEL_5,
        DAC_SEL_6,
        DAC_SEL_7,
        DAC_SEL_8,
        DAC_THRESH_1,
        DAC_THRESH_2,
        DAC_THRESH_3,
        DAC_THRESH_4,
        DAC_THRESH_5,
        DAC_THRESH_6,
        DAC_THRESH_7,
        DAC_THRESH_8,
        HPF
    };
    
    const oni_dev_idx_t DEVICE_RHYTHM = 0x0101;
    const oni_dev_idx_t DEVICE_TTL = 0x0102;
    const oni_dev_idx_t DEVICE_DAC = 0x0103;
   
};

#endif /* OniDevice_hpp */
