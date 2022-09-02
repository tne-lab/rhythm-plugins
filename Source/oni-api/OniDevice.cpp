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

#include "OniDevice.h"

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <thread>
#include <chrono>
#include <cmath>


OniDevice::OniDevice() :
    AbstractRHXController(ControllerOEECP5, AmplifierSampleRate::SampleRate30000Hz),
    ctx(NULL)
{

}

OniDevice::~OniDevice()
{
    if (ctx != NULL)
        oni_destroy_ctx(ctx);
}

int OniDevice::open(const std::string& boardSerialNumber)
{
   ctx = oni_create_ctx("ft600"); // "ft600" is the driver name for the usb 
   if (ctx == NULL) return -1;
   
   if ( oni_init_ctx(ctx, 0) != ONI_ESUCCESS)
   {
       oni_destroy_ctx(ctx);
       return -2;
   }

    return 1;
}

bool OniDevice::uploadFPGABitfile(const std::string& filename)
{
    // no bitfile required

    return true;
}
 
void OniDevice::resetBoard()
{

    oni_reg_val_t reg = 1;
    oni_set_opt(ctx, ONI_OPT_RESET, &reg, sizeof(oni_size_t));
}


void OniDevice::run()
{
    oni_reg_val_t reg = 2;
    oni_set_opt(ctx, ONI_OPT_RESETACQCOUNTER, &reg, sizeof(oni_size_t));
}


bool OniDevice::isRunning()
{
    return false;
}

void OniDevice::flush()
{
    //ONI COMMENTS: flush is not needed, as stopping acquisition flushes the board buffers automatically.
}


void OniDevice::resetFpga()
{

}


bool OniDevice::readDataBlock(RHXDataBlock* dataBlock)
{
    /*std::lock_guard<std::mutex> lockOk(okMutex);

    unsigned int numBytesToRead = BytesPerWord * RHXDataBlock::dataBlockSizeInWords(type, numDataStreams);

    if (numBytesToRead > usbBufferSize) {
        std::cerr << "Error in OniDevice::readDataBlock: USB buffer size exceeded.  " <<
            "Increase value of MAX_NUM_BLOCKS.\n";
        return false;
    }

    if (type == ControllerRecordUSB3) {
        long result = dev->ReadFromBlockPipeOut(PipeOutData, USB3BlockSize,
            USB3BlockSize * std::max(numBytesToRead / USB3BlockSize, (unsigned int)1),
            usbBuffer);
        if (result == ok_Failed) {
            std::cerr << "CRITICAL (readDataBlock): Failure on pipe read.  Check block and buffer sizes.\n";
        }
        else if (result == ok_Timeout) {
            std::cerr << "CRITICAL (readDataBlock): Timeout on pipe read.  Check block and buffer sizes.\n";
        }
    }
    else {
        dev->ReadFromPipeOut(PipeOutData, numBytesToRead, usbBuffer);
    }
    dataBlock->fillFromUsbBuffer(usbBuffer, 0);*/

    return true;
}


bool OniDevice::readDataBlocks(int numBlocks, std::deque<RHXDataBlock*>& dataQueue)
{
    /*std::lock_guard<std::mutex> lockOk(okMutex);

    unsigned int numWordsToRead = numBlocks * RHXDataBlock::dataBlockSizeInWords(type, numDataStreams);

    if (numWordsInFifo() < numWordsToRead)
        return false;

    unsigned int numBytesToRead = BytesPerWord * numWordsToRead;

    if (numBytesToRead > usbBufferSize) {
        std::cerr << "Error in OniDevice::readDataBlocks: USB buffer size exceeded.  " <<
            "Increase value of MaxNumBlocksToRead.\n";
        return false;
    }

    if (type == ControllerRecordUSB3) {
        long result = dev->ReadFromBlockPipeOut(PipeOutData, USB3BlockSize, numBytesToRead, usbBuffer);

        if (result == ok_Failed) {
            std::cerr << "CRITICAL (readDataBlocks): Failure on pipe read.  Check block and buffer sizes.\n";
        }
        else if (result == ok_Timeout) {
            std::cerr << "CRITICAL (readDataBlocks): Timeout on pipe read.  Check block and buffer sizes.\n";
        }
    }
    else {
        dev->ReadFromPipeOut(PipeOutData, numBytesToRead, usbBuffer);
    }

    for (int i = 0; i < numBlocks; ++i) {
        RHXDataBlock* dataBlock = new RHXDataBlock(type, numDataStreams);
        dataBlock->fillFromUsbBuffer(usbBuffer, i);
        dataQueue.push_back(dataBlock);
    }*/

    return true;
}


long OniDevice::readDataBlocksRaw(int numBlocks, uint8_t* buffer)
{
    
    int nSamples; //ONI TODO: For ONI the unit is a sample. We need to know how many samples are required 

    int samplesRead = 0;
    size_t bufferIndex = 0;
    
    do {
        oni_frame_t* frame;
        /***
        A note on how oni_read_frame works. The call does not actually transfer single frames. There is a ONI_OPT_BLOCKREADSIZE parameter that sets the actual transfer size.
        The first time oni_read_frame is called, it triggers a transfer of said size into a buffer. Subsequent calls of oni_read_frame return a frame from said buffer in a
        no-copy manner (i.e.: data is a pointer to the already existing buffer). If there is not enough data in the buffer for a new frame, a new transfer is triggered
        ***/
        oni_read_frame(ctx, &frame);

        if (frame->dev_idx == DEVICE_RHYTHM) 
        {
            //this is terribly inefficient and will probably a usb thread.
            //A better option could be to refactor DeviceThread::updateBuffer so it can convert frame by frame without the extra copy required here
            //This could be done by filling an array of frame pointers, maybe.

            memcpy(buffer+bufferIndex,frame->data+8,frame->data_sz-8);
            bufferIndex += frame->data_sz-8;
        }

        oni_destroy_frame(frame);

    } while (samplesRead < nSamples);

    long result = 100;

    return result;
}

// 
void OniDevice::setContinuousRunMode(bool continuousMode)
{

    if (continuousMode) {
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInResetRun, 0x02);
        //dev->SetWireInValue(WireInResetRun, 0x02, 0x02);
    }
    else {
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInResetRun, 0x00);
        //dev->SetWireInValue(WireInResetRun, 0x00, 0x02);
    }

    // dev->UpdateWireIns() // not needed?

}

void OniDevice::setMaxTimeStep(unsigned int maxTimeStep)
{

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPointUSB3::WireInMaxTimeStep_USB3, maxTimeStep);
    //dev->SetWireInValue(WireInMaxTimeStep_USB3, maxTimeStep);

}

void OniDevice::setCableDelay(BoardPort port, int delay)
{
    int bitShift = 0;

    if ((delay < 0) || (delay > 15)) {
        std::cerr << "Warning in OniDevice::setCableDelay: delay out of range: " << delay << '\n';
        if (delay < 0) delay = 0;
        else if (delay > 15) delay = 15;
    }

    switch (port) {
    case PortA:
        bitShift = 0;
        cableDelay[0] = delay;
        break;
    case PortB:
        bitShift = 4;
        cableDelay[1] = delay;
        break;
    case PortC:
        bitShift = 8;
        cableDelay[2] = delay;
        break;
    case PortD:
        bitShift = 12;
        cableDelay[3] = delay;
        break;
    case PortE:
        bitShift = 16;
        cableDelay[4] = delay;
        break;
    case PortF:
        bitShift = 20;
        cableDelay[5] = delay;
        break;
    case PortG:
        bitShift = 24;
        cableDelay[6] = delay;
        break;
    case PortH:
        bitShift = 28;
        cableDelay[7] = delay;
        break;
    default:
        std::cerr << "Error in OniDevice::setCableDelay: unknown port.\n";
    }

    oni_reg_val_t value;
    oni_read_reg(ctx, DEVICE_RHYTHM, Rhythm_Registers::CABLE_DELAY, &value); //read the current value
    value =  (value & (~(0xf < bitShift))) | (delay << bitShift); //clear only the relevant bits and set them to the new delay
    oni_write_reg(ctx, DEVICE_RHYTHM, Rhythm_Registers::CABLE_DELAY, value);
}


void OniDevice::setDspSettle(bool enabled)
{
    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInResetRun, (enabled ? 0x04 : 0x00));
    //dev->SetWireInValue(WireInResetRun, (enabled ? 0x04 : 0x00), 0x04);
}


void OniDevice::setTtlOut(const int* ttlOutArray)
{
 
    int32_t ttlOut = 0;

    for (int i = 0; i < 16; ++i) {
        if (ttlOutArray[i] > 0)
            ttlOut += 1 << i;
    }

    oni_frame_t* frame;

    if (oni_create_frame(ctx, &frame, DEVICE_TTL, &ttlOut, sizeof(ttlOut)) == ONI_ESUCCESS)
    {
        oni_write_frame(ctx, frame);
        oni_destroy_frame(frame);
    }   
    
}


void OniDevice::setDacManual(int value)
{

    if ((value < 0) || (value > 65535)) {
        std::cerr << "Error in OniDevice::setDacManual: value out of range.\n";
        return;
    }

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacManual, value);
    //dev->SetWireInValue(WireInDacManual, value);

}


void OniDevice::enableLeds(bool ledsOn)
{

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInMultiUse, ledsOn ? 1 : 0);
    //dev->SetWireInValue(WireInMultiUse, ledsOn ? 1 : 0);

    // TODO -- what is the equivalent of this one???
    //dev->ActivateTriggerIn(TrigInOpenEphys, 0);
    
}

// Set output BNC clock divide factor (Open Ephys boards only)
void OniDevice::setClockDivider(int divide_factor)
{
    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInMultiUse, divide_factor);
    //dev->SetWireInValue(WireInMultiUse, divide_factor);

    // TODO -- what is the equivalent of this one???
    //dev->ActivateTriggerIn(TrigInOpenEphys, 1);

}


void OniDevice::setDacGain(int gain)
{

    if ((gain < 0) || (gain > 7)) {
        std::cerr << "Error in OniDevice::setDacGain: gain setting out of range.\n";
        return;
    }

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInResetRun, gain << 13);
}

// 
void OniDevice::setAudioNoiseSuppress(int noiseSuppress)
{

    if ((noiseSuppress < 0) || (noiseSuppress > 127)) {
        std::cerr << "Error in OniDevice::setAudioNoiseSuppress: noiseSuppress out of range.\n";
        return;
    }

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInResetRun, noiseSuppress << 6);
    //dev->SetWireInValue(WireInResetRun, noiseSuppress << 6, 0x1fc0);
}

// 
void OniDevice::setExternalFastSettleChannel(int channel)
{

    if ((channel < 0) || (channel > 15)) {
        std::cerr << "Error in OniDevice::setExternalFastSettleChannel: channel out of range.\n";
        return;
    }

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInMultiUse, channel);
    //dev->SetWireInValue(WireInMultiUse, channel);

    // dev->ActivateTriggerIn(TrigInConfig_USB3, 7);
}

// 
void OniDevice::setExternalDigOutChannel(BoardPort port, int channel)
{

    if ((channel < 0) || (channel > 15)) {
        std::cerr << "Error in OniDevice::setExternalDigOutChannel: channel out of range.\n";
        return;
    }

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInMultiUse, channel);

    switch (port) {
    case PortA:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 24);
        break;
    case PortB:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 25);
        break;
    case PortC:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 26);
        break;
    case PortD:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 27);
        break;
    case PortE:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 28);
        break;
    case PortF:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 29);
        break;
    case PortG:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 30);
        break;
    case PortH:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 31);
        break;
    default:
        std::cerr << "Error in OniDevice::setExternalDigOutChannel: port out of range.\n";
    }
}

// 
void OniDevice::setDacHighpassFilter(double cutoff)
{

    // Note that the filter coefficient is a function of the amplifier sample rate, so this
    // function should be called after the sample rate is changed.
    double b = 1.0 - exp(-1.0 * TwoPi * cutoff / getSampleRate());

    // In hardware, the filter coefficient is represented as a 16-bit number.
    int filterCoefficient = (int)floor(65536.0 * b + 0.5);

    if (filterCoefficient < 1) {
        filterCoefficient = 1;
    }
    else if (filterCoefficient > 65535) {
        filterCoefficient = 65535;
    }

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInMultiUse, filterCoefficient);
    //dev->SetWireInValue(WireInMultiUse, filterCoefficient);

    //dev->ActivateTriggerIn(TrigInConfig_USB3, 5);

}

// 
void OniDevice::setDacThreshold(int dacChannel, int threshold, bool trigPolarity)
{

    if ((dacChannel < 0) || (dacChannel > 7)) {
        std::cerr << "Error in OniDevice::setDacThreshold: dacChannel out of range.\n";
        return;
    }

    if ((threshold < 0) || (threshold > 65535)) {
        std::cerr << "Error in OniDevice::setDacThreshold: threshold out of range.\n";
        return;
    }

    // Set threshold level.
    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInMultiUse, threshold);
    //dev->SetWireInValue(WireInMultiUse, threshold);

    //dev->ActivateTriggerIn(TrigInDacConfig_USB3, dacChannel);


    // Set threshold polarity.
    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInMultiUse, (trigPolarity ? 1 : 0));
    //dev->SetWireInValue(WireInMultiUse, (trigPolarity ? 1 : 0));

    //dev->ActivateTriggerIn(TrigInDacConfig_USB3, dacChannel + 8);

}

// 
void OniDevice::setTtlMode(int mode)
{

    if ((mode < 0) || (mode > 1)) {
        std::cerr << "Error in OniDevice::setTtlMode: mode out of range.\n";
        return;
    }

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInResetRun, mode << 3);
    //dev->SetWireInValue(WireInResetRun, mode << 3, 0x0008);

}
 
void OniDevice::setDacRerefSource(int stream, int channel)
{

    if (stream < 0 || stream >(maxNumDataStreams() - 1)) {
        std::cerr << "Error in OniDevice::setDacRerefSource: stream out of range.\n";
        return;
    }

    if (channel < 0 || channel > RHXDataBlock::channelsPerStream(type) - 1) {
        std::cerr << "Error in OniDevice::setDacRerefSource: channel out of range.\n";
        return;
    }

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPointRecordUSB3::WireInDacReref_R_USB3, (stream << 5) + channel);
    //dev->SetWireInValue(WireInDacReref_R_USB3, (stream << 5) + channel, 0x0000003ff);

}

bool OniDevice::setSampleRate(AmplifierSampleRate newSampleRate)
{

    // Assuming a 100 MHz reference clock is provided to the FPGA, the programmable FPGA clock frequency
    // is given by:
    //
    //       FPGA internal clock frequency = 100 MHz * (M/D) / 2
    //
    // M and D are "multiply" and "divide" integers used in the FPGA's digital clock manager (DCM) phase-
    // locked loop (PLL) frequency synthesizer, and are subject to the following restrictions:
    //
    //                M must have a value in the range of 2 - 256
    //                D must have a value in the range of 1 - 256
    //                M/D must fall in the range of 0.05 - 3.33
    //
    // (See pages 85-86 of Xilinx document UG382 "Spartan-6 FPGA Clocking Resources" for more details.)
    //
    // This variable-frequency clock drives the state machine that controls all SPI communication
    // with the RHD2000 chips.  A complete SPI cycle (consisting of one CS pulse and 16 SCLK pulses)
    // takes 80 clock cycles.  The SCLK period is 4 clock cycles; the CS pulse is high for 14 clock
    // cycles between commands.
    //
    // Rhythm samples all 32 channels and then executes 3 "auxiliary" commands that can be used to read
    // and write from other registers on the chip, or to sample from the temperature sensor or auxiliary ADC
    // inputs, for example.  Therefore, a complete cycle that samples from each amplifier channel takes
    // 80 * (32 + 3) = 80 * 35 = 2800 clock cycles.
    //
    // So the per-channel sampling rate of each amplifier is 2800 times slower than the clock frequency.
    //
    // Based on these design choices, we can use the following values of M and D to generate the following
    // useful amplifier sampling rates for electrophsyiological applications:
    //
    //   M    D     clkout frequency    per-channel sample rate     per-channel sample period
    //  ---  ---    ----------------    -----------------------     -------------------------
    //    7  125          2.80 MHz               1.00 kS/s                 1000.0 usec = 1.0 msec
    //    7  100          3.50 MHz               1.25 kS/s                  800.0 usec
    //   21  250          4.20 MHz               1.50 kS/s                  666.7 usec
    //   14  125          5.60 MHz               2.00 kS/s                  500.0 usec
    //   35  250          7.00 MHz               2.50 kS/s                  400.0 usec
    //   21  125          8.40 MHz               3.00 kS/s                  333.3 usec
    //   14   75          9.33 MHz               3.33 kS/s                  300.0 usec
    //   28  125         11.20 MHz               4.00 kS/s                  250.0 usec
    //    7   25         14.00 MHz               5.00 kS/s                  200.0 usec
    //    7   20         17.50 MHz               6.25 kS/s                  160.0 usec
    //  112  250         22.40 MHz               8.00 kS/s                  125.0 usec
    //   14   25         28.00 MHz              10.00 kS/s                  100.0 usec
    //    7   10         35.00 MHz              12.50 kS/s                   80.0 usec
    //   21   25         42.00 MHz              15.00 kS/s                   66.7 usec
    //   28   25         56.00 MHz              20.00 kS/s                   50.0 usec
    //   35   25         70.00 MHz              25.00 kS/s                   40.0 usec
    //   42   25         84.00 MHz              30.00 kS/s                   33.3 usec
    //
    // To set a new clock frequency, assert new values for M and D (e.g., using okWireIn modules) and
    // pulse DCM_prog_trigger high (e.g., using an okTriggerIn module).  If this module is reset, it
    // reverts to a per-channel sampling rate of 30.0 kS/s.

    unsigned long M, D;

    switch (newSampleRate) {
    case SampleRate1000Hz:
        M = 7;
        D = 125;
        break;
    case SampleRate1250Hz:
        M = 7;
        D = 100;
        break;
    case SampleRate1500Hz:
        M = 21;
        D = 250;
        break;
    case SampleRate2000Hz:
        M = 14;
        D = 125;
        break;
    case SampleRate2500Hz:
        M = 35;
        D = 250;
        break;
    case SampleRate3000Hz:
        M = 21;
        D = 125;
        break;
    case SampleRate3333Hz:
        M = 14;
        D = 75;
        break;
    case SampleRate4000Hz:
        M = 28;
        D = 125;
        break;
    case SampleRate5000Hz:
        M = 7;
        D = 25;
        break;
    case SampleRate6250Hz:
        M = 7;
        D = 20;
        break;
    case SampleRate8000Hz:
        M = 112;
        D = 250;
        break;
    case SampleRate10000Hz:
        M = 14;
        D = 25;
        break;
    case SampleRate12500Hz:
        M = 7;
        D = 10;
        break;
    case SampleRate15000Hz:
        M = 21;
        D = 25;
        break;
    case SampleRate20000Hz:
        M = 28;
        D = 25;
        break;
    case SampleRate25000Hz:
        M = 35;
        D = 25;
        break;
    case SampleRate30000Hz:
        M = 42;
        D = 25;
        break;
    default:
        return false;
    }

    sampleRate = newSampleRate;

    // Wait for DcmProgDone = 1 before reprogramming clock synthesizer.
    while (isDcmProgDone() == false) {}

    // Reprogram clock synthesizer.
    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDataFreqPll, (256 * M + D));
    //dev->SetWireInValue(WireInDataFreqPll, (256 * M + D));

    //dev->ActivateTriggerIn(TrigInConfig_USB3, 0);

    // Wait for DataClkLocked = 1 before allowing data acquisition to continue.
    while (isDataClockLocked() == false) {}

    return true;
}


void OniDevice::enableDataStream(int stream, bool enabled)
{

    if (stream < 0 || stream >(maxNumDataStreams() - 1)) {
        std::cerr << "Error in OniDevice::enableDataStream: stream out of range.\n";
        return;
    }

    if (enabled) {
        if (dataStreamEnabled[stream] == 0) {

            oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDataStreamEn, 0x00000001 << stream);
            //dev->SetWireInValue(WireInDataStreamEn, 0x00000001 << stream, 0x00000001 << stream);
   
            dataStreamEnabled[stream] = 1;
            numDataStreams++;
        }
    }
    else {
        if (dataStreamEnabled[stream] == 1) {

            oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDataStreamEn, 0x00000000 << stream);
            //dev->SetWireInValue(WireInDataStreamEn, 0x00000000 << stream, 0x00000001 << stream);

            dataStreamEnabled[stream] = 0;
            numDataStreams--;
        }
    }
}


void OniDevice::enableDac(int dacChannel, bool enabled)
{

    if ((dacChannel < 0) || (dacChannel > 7)) {
        std::cerr << "Error in OniDevice::enableDac: dacChannel out of range.\n";
        return;
    }

    unsigned int enableVal = 0x0800;

    switch (dacChannel) {
    case 0:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource1, (enabled ? enableVal : 0));
        //dev->SetWireInValue(WireInDacSource1, (enabled ? enableVal : 0), enableVal);
        break;
    case 1:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource2, (enabled ? enableVal : 0));
        //dev->SetWireInValue(WireInDacSource2, (enabled ? enableVal : 0), enableVal);
        break;
    case 2:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource3, (enabled ? enableVal : 0));
        //dev->SetWireInValue(WireInDacSource3, (enabled ? enableVal : 0), enableVal);
        break;
    case 3:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource4, (enabled ? enableVal : 0));
        //dev->SetWireInValue(WireInDacSource4, (enabled ? enableVal : 0), enableVal);
        break;
    case 4:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource5, (enabled ? enableVal : 0));
        //dev->SetWireInValue(WireInDacSource5, (enabled ? enableVal : 0), enableVal);
        break;
    case 5:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource6, (enabled ? enableVal : 0));
        //dev->SetWireInValue(WireInDacSource6, (enabled ? enableVal : 0), enableVal);
        break;
    case 6:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource7, (enabled ? enableVal : 0));
        //dev->SetWireInValue(WireInDacSource7, (enabled ? enableVal : 0), enableVal);
        break;
    case 7:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource8, (enabled ? enableVal : 0));
        //dev->SetWireInValue(WireInDacSource8, (enabled ? enableVal : 0), enableVal);
        break;
    }

}


void OniDevice::enableExternalFastSettle(bool enable)
{

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInMultiUse, enable ? 1 : 0);
    //dev->SetWireInValue(WireInMultiUse, enable ? 1 : 0);

    //dev->ActivateTriggerIn(TrigInConfig_USB3, 6);

}

// .
void OniDevice::enableExternalDigOut(BoardPort port, bool enable)
{

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInMultiUse, enable ? 1 : 0);
    //dev->SetWireInValue(WireInMultiUse, enable ? 1 : 0);

    switch (port) {
    case PortA:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 16);
        break;
    case PortB:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 17);
        break;
    case PortC:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 18);
        break;
    case PortD:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 19);
        break;
    case PortE:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 20);
        break;
    case PortF:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 21);
        break;
    case PortG:
        //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 22);
        break;
    case PortH:
       //dev->ActivateTriggerIn(TrigInDacConfig_USB3, 23);
        break;
    default:
        std::cerr << "Error in OniDevice::enableExternalDigOut: port out of range.\n";
    }
 
}


void OniDevice::enableDacHighpassFilter(bool enable)
{

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInMultiUse, enable ? 1 : 0);

    //dev->ActivateTriggerIn(TrigInConfig_USB3, 4);

}

void OniDevice::enableDacReref(bool enabled)
{

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInMultiUse, (enabled ? 0x00000400 : 0x00000000));
    //dev->SetWireInValue(WireInDacReref_R_USB3, (enabled ? 0x00000400 : 0x00000000), 0x00000400);

}


void OniDevice::selectDacDataStream(int dacChannel, int stream)
{

    if ((dacChannel < 0) || (dacChannel > 7)) {
        std::cerr << "Error in OniDevice::selectDacDataStream: dacChannel out of range.\n";
        return;
    }

    int maxStream = 16;

    if (stream < 0 || stream > maxStream) {
        std::cerr << "Error in OniDevice::selectDacDataStream: stream out of range.\n";
        return;
    }

    unsigned int mask = 0x07e0;

    switch (dacChannel) {
    case 0:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource1, stream << 5);
        //dev->SetWireInValue(WireInDacSource1, stream << 5, mask);
        break;
    case 1:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource2, stream << 5);
        //dev->SetWireInValue(WireInDacSource2, stream << 5, mask);
        break;
    case 2:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource3, stream << 5);
        //dev->SetWireInValue(WireInDacSource3, stream << 5, mask);
        break;
    case 3:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource4, stream << 5);
        //dev->SetWireInValue(WireInDacSource4, stream << 5, mask);
        break;
    case 4:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource5, stream << 5);
        //dev->SetWireInValue(WireInDacSource5, stream << 5, mask);
        break;
    case 5:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource6, stream << 5);
        //dev->SetWireInValue(WireInDacSource6, stream << 5, mask);
        break;
    case 6:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource7, stream << 5);
        //dev->SetWireInValue(WireInDacSource7, stream << 5, mask);
        break;
    case 7:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource8, stream << 5);
        //dev->SetWireInValue(WireInDacSource8, stream << 5, mask);
        break;
    }

}


void OniDevice::selectDacDataChannel(int dacChannel, int dataChannel)
{

    if ((dacChannel < 0) || (dacChannel > 7)) {
        std::cerr << "Error in OniDevice::selectDacDataChannel: dacChannel out of range.\n";
        return;
    }

    if ((dataChannel < 0) || (dataChannel > 31)) {
        std::cerr << "Error in OniDevice::selectDacDataChannel: dataChannel out of range.\n";
        return;
    }

    switch (dacChannel) {
    case 0:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource1, dataChannel << 0);
        //dev->SetWireInValue(WireInDacSource1, dataChannel << 0, 0x001f);
        break;
    case 1:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource2, dataChannel << 0);
        //dev->SetWireInValue(WireInDacSource2, dataChannel << 0, 0x001f);
        break;
    case 2:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource3, dataChannel << 0);
        //dev->SetWireInValue(WireInDacSource3, dataChannel << 0, 0x001f);
        break;
    case 3:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource4, dataChannel << 0);
        //dev->SetWireInValue(WireInDacSource4, dataChannel << 0, 0x001f);
        break;
    case 4:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource5, dataChannel << 0);
        //dev->SetWireInValue(WireInDacSource5, dataChannel << 0, 0x001f);
        break;
    case 5:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource6, dataChannel << 0);
        //dev->SetWireInValue(WireInDacSource6, dataChannel << 0, 0x001f);
        break;
    case 6:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource7, dataChannel << 0);
        //dev->SetWireInValue(WireInDacSource7, dataChannel << 0, 0x001f);
        break;
    case 7:
        oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDacSource8, dataChannel << 0);
        //dev->SetWireInValue(WireInDacSource8, dataChannel << 0, 0x001f);
        break;
    }

}


void OniDevice::selectAuxCommandLength(AuxCmdSlot auxCommandSlot, int loopIndex, int endIndex)
{

    int maxIndex = 1024;

    if (loopIndex < 0 || loopIndex > maxIndex - 1) {
        std::cerr << "Error in OniDevice::selectAuxCommandLength: loopIndex out of range.\n";
        return;
    }

    if (endIndex < 0 || endIndex > maxIndex - 1) {
        std::cerr << "Error in OniDevice::selectAuxCommandLength: endIndex out of range.\n";
        return;
    }

    switch (auxCommandSlot) {
    case AuxCmd1:
        oni_write_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireInAuxCmdLoop1, loopIndex);
        //dev->SetWireInValue(WireInAuxCmdLoop_R_USB3, loopIndex, 0x000003ff);
        oni_write_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireInAuxCmdLength1, endIndex);
        //dev->SetWireInValue(WireInAuxCmdLength_R_USB3, endIndex, 0x000003ff);
        break;
    case AuxCmd2:
        oni_write_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireInAuxCmdLoop2, loopIndex << 10);
        //dev->SetWireInValue(WireInAuxCmdLoop_R_USB3, loopIndex << 10, 0x000003ff << 10);
        oni_write_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireInAuxCmdLength2, endIndex << 10);
        //dev->SetWireInValue(WireInAuxCmdLength_R_USB3, endIndex << 10, 0x000003ff << 10);
        break;
    case AuxCmd3:
        oni_write_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireInAuxCmdLoop3, loopIndex << 20);
        //dev->SetWireInValue(WireInAuxCmdLoop_R_USB3, loopIndex << 20, 0x000003ff << 20);
        oni_write_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireInAuxCmdLength3, endIndex << 20);
        //dev->SetWireInValue(WireInAuxCmdLength_R_USB3, endIndex << 20, 0x000003ff << 20);
        break;
    case AuxCmd4:
        // Should not be reached, as AuxCmd4 is Stim-only.
        break;
    }


}


void OniDevice::selectAuxCommandBank(BoardPort port, AuxCmdSlot auxCommandSlot, int bank)
{

    int bitShift;

    if (auxCommandSlot != AuxCmd1 && auxCommandSlot != AuxCmd2 && auxCommandSlot != AuxCmd3) {
        std::cerr << "Error in OniDevice::selectAuxCommandBank: auxCommandSlot out of range.\n";
        return;
    }
    if ((bank < 0) || (bank > 15)) {
        std::cerr << "Error in OniDevice::selectAuxCommandBank: bank out of range.\n";
        return;
    }

    switch (port) {
    case PortA:
        bitShift = 0;
        break;
    case PortB:
        bitShift = 4;
        break;
    case PortC:
        bitShift = 8;
        break;
    case PortD:
        bitShift = 12;
        break;
    case PortE:
        bitShift = 16;
        break;
    case PortF:
        bitShift = 20;
        break;
    case PortG:
        bitShift = 24;
        break;
    case PortH:
        bitShift = 28;
        break;
    }

    switch (auxCommandSlot) {
    case AuxCmd1:
        oni_write_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireInAuxCmdBank1, bank << bitShift);
        //dev->SetWireInValue(WireInAuxCmdBank1_R, bank << bitShift, 0x0000000f << bitShift);
        break;
    case AuxCmd2:
        oni_write_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireInAuxCmdBank2, bank << bitShift);
        //dev->SetWireInValue(WireInAuxCmdBank2_R, bank << bitShift, 0x0000000f << bitShift);
        break;
    case AuxCmd3:
        oni_write_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireInAuxCmdBank3, bank << bitShift);
        //dev->SetWireInValue(WireInAuxCmdBank3_R, bank << bitShift, 0x0000000f << bitShift);
        break;
    case AuxCmd4:
        // Should not be reached, as AuxCmd4 is Stim-only.
        break;

    }

}

int OniDevice::getBoardMode()
{
    // not used?
    return 0; 
}

// 
int OniDevice::getNumSPIPorts(bool& expanderBoardDetected)
{
    return 4;
}

// 
void OniDevice::clearTtlOut()
{

    int ttlOutArray[16];

    setTtlOut(ttlOutArray);

}


void OniDevice::uploadCommandList(const std::vector<unsigned int>& commandList, AuxCmdSlot auxCommandSlot, int bank)
{

    if (auxCommandSlot != AuxCmd1 && auxCommandSlot != AuxCmd2 && auxCommandSlot != AuxCmd3) {
        std::cerr << "Error in OniDevice::uploadCommandList: auxCommandSlot out of range.\n";
        return;
    }

    if ((bank < 0) || (bank > 15)) {
        std::cerr << "Error in OniDevice::uploadCommandList: bank out of range.\n";
        return;
    }

    for (unsigned int i = 0; i < commandList.size(); ++i) 
    {
        oni_write_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireInCmdRamData, commandList[i]);
        //dev->SetWireInValue(WireInCmdRamData_R, commandList[i]);

        oni_write_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireInCmdRamAddr, i);
        //dev->SetWireInValue(WireInCmdRamAddr_R, i);

        oni_write_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireInCmdRamBank, bank);
 
        switch (auxCommandSlot) {
        case AuxCmd1:
            //dev->ActivateTriggerIn(TrigInConfig_USB3, 1);
            break;
        case AuxCmd2:
            //dev->ActivateTriggerIn(TrigInConfig_USB3, 2);
            break;
        case AuxCmd3:
           // dev->ActivateTriggerIn(TrigInConfig_USB3, 3);
            break;
        case AuxCmd4:
            // Should not be reached, as AuxCmd4 is Stim-only.
            break;
        }
    }
  
}

// Scan all SPI ports to find all connected RHD/RHS amplifier chips.  Read the chip ID from on-chip ROM
// register to determine the number of amplifier channels on each port.  This process is repeated at all
// possible MISO delays in the FPGA to determine the optimum MISO delay for each port to compensate for
// variable cable length.
//
// This function returns three vectors of length maxNumDataStreams(): chipType (the type of chip connected
// to each data stream); portIndex (the SPI port number [A=1, B=2,...] associated with each data stream);
// and commandStream (the stream index for sending commands to the FPGA for a particular read stream index).
// This function also returns a vector of length maxNumSPIPorts(): numChannelsOnPort (the total number of
// amplifier channels on each port).
//
// This function normally returns 1, but returns a negative number if a ControllerRecordUSB2 devices is used
// and its 256-channel capacity (limited by USB2 bus speed) is exceeded.  A value of -1 is returned, or a value
// of -2 if RHD2216 devices are present so that the user can be reminded that RHD2216 devices consume 32 channels
// of USB bus bandwidth.
int OniDevice::findConnectedChips(std::vector<ChipType>& chipType, std::vector<int>& portIndex, std::vector<int>& commandStream,
    std::vector<int>& numChannelsOnPort)
{
    int returnValue = 1;    // return 1 == everything okay
    int maxNumStreams = maxNumDataStreams();
    int maxSPIPorts = maxNumSPIPorts();
    int maxMISOLines = 2 * maxSPIPorts;

    chipType.resize(maxNumStreams);
    fill(chipType.begin(), chipType.end(), NoChip);
    std::vector<ChipType> chipTypeOld(maxNumStreams, NoChip);

    portIndex.resize(maxNumStreams);
    fill(portIndex.begin(), portIndex.end(), -1);
    std::vector<int> portIndexOld(maxMISOLines, -1);

    commandStream.resize(maxNumStreams);
    fill(commandStream.begin(), commandStream.end(), -1);

    numChannelsOnPort.resize(maxSPIPorts);
    fill(numChannelsOnPort.begin(), numChannelsOnPort.end(), 0);

    // ControllerRecordUSB2 only
    BoardDataSource initStreamPorts[8] = { PortA1, PortA2, PortB1, PortB2, PortC1, PortC2, PortD1, PortD2 };
    BoardDataSource initStreamDdrPorts[8] = { PortA1Ddr, PortA2Ddr, PortB1Ddr, PortB2Ddr,
                                              PortC1Ddr, PortC2Ddr, PortD1Ddr, PortD2Ddr };
    if (type == ControllerRecordUSB2) {
        for (int stream = 0; stream < maxNumStreams; stream++) {
            setDataSource(stream, initStreamPorts[stream]);
        }
    }

    portIndexOld[0] = 0; portIndexOld[1] = 0;
    portIndexOld[2] = 1; portIndexOld[3] = 1;
    portIndexOld[4] = 2; portIndexOld[5] = 2;
    portIndexOld[6] = 3; portIndexOld[7] = 3;
    if (type == ControllerRecordUSB3) {
        portIndexOld[8] = 4; portIndexOld[9] = 4;
        portIndexOld[10] = 5; portIndexOld[11] = 5;
        portIndexOld[12] = 6; portIndexOld[13] = 6;
        portIndexOld[14] = 7; portIndexOld[15] = 7;
    }

    // Enable all non-DDR data streams.
    for (int stream = 0; stream < maxNumStreams; stream++) {
        enableDataStream(stream, true);
    }
    if (type == ControllerRecordUSB3) {
        for (int stream = 1; stream < maxNumStreams; stream += 2) {
            enableDataStream(stream, false);
        }
    }

    // Run the SPI interface for multiple command sequences (i.e., NRepeats data blocks).
    const int NRepeats = 12;
    RHXDataBlock dataBlock(type, getNumEnabledDataStreams());
    setMaxTimeStep(NRepeats * dataBlock.samplesPerDataBlock());
    setContinuousRunMode(false);

    int auxCmdSlot = (type == ControllerStimRecordUSB2 ? AuxCmd1 : AuxCmd3);

    std::vector<std::vector<int> > goodDelays;
    goodDelays.resize(maxMISOLines);
    for (int i = 0; i < maxMISOLines; ++i) {
        goodDelays[i].resize(16);
        for (int j = 0; j < 16; ++j) {
            goodDelays[i][j] = 0;
        }
    }

    // Run SPI command sequence at all 16 possible FPGA MISO delay settings
    // to find optimum delay for each SPI interface cable.
    for (int delay = 0; delay < 16; ++delay) {
        setCableDelay(PortA, delay);
        setCableDelay(PortB, delay);
        setCableDelay(PortC, delay);
        setCableDelay(PortD, delay);
        if (type == ControllerRecordUSB3) {
            setCableDelay(PortE, delay);
            setCableDelay(PortF, delay);
            setCableDelay(PortG, delay);
            setCableDelay(PortH, delay);
        }
        run();

        // Wait for the run to complete.
        while (isRunning()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        for (int i = 0; i < NRepeats; ++i) {
            // Read one data block from the USB interface.
            readDataBlock(&dataBlock);

            // Read the Intan chip ID number from each RHD or RHS chip found.
            // Record delay settings that yield good communication with the chip.
            int register59Value;
            for (int stream = 0; stream < maxMISOLines; stream++) {
                int id = dataBlock.getChipID(stream, auxCmdSlot, register59Value);
                if (id == (int)RHD2132Chip || id == (int)RHD2216Chip || id == (int)RHS2116Chip ||
                    (id == (int)RHD2164Chip && register59Value == Register59MISOA)) {
                    goodDelays[stream][delay] = goodDelays[stream][delay] + 1;
                    chipTypeOld[stream] = (ChipType)id;
                }
            }
        }
    }

    // Set cable delay settings that yield good communication with each chip.
    std::vector<int> optimumDelay(maxMISOLines, 0);
    for (int stream = 0; stream < maxMISOLines; ++stream) {
        int bestCount = -1;
        for (int delay = 0; delay < 16; ++delay) {
            if (goodDelays[stream][delay] > bestCount) {
                bestCount = goodDelays[stream][delay];
            }
        }
        int numBest = 0;
        for (int delay = 0; delay < 16; ++delay) {
            if (goodDelays[stream][delay] == bestCount) {
                ++numBest;
            }
        }
        int bestDelay = -1;
        if (numBest == 2 && (chipTypeOld[stream] == RHD2164Chip)) {
            for (int delay = 15; delay >= 0; --delay) {  // DDR SPI from RHD2164 chip seems to work best with longer of two valid delays.
                if (goodDelays[stream][delay] == bestCount) {
                    bestDelay = delay;
                    break;
                }
            }
        }
        else {
            for (int delay = 0; delay < 16; ++delay) {
                if (goodDelays[stream][delay] == bestCount) {
                    bestDelay = delay;
                    break;
                }
            }
            if (numBest > 2) {  // If 3 or more valid delays, don't use the longest or shortest.
                for (int delay = bestDelay + 1; delay < 16; ++delay) {
                    if (goodDelays[stream][delay] == bestCount) {
                        bestDelay = delay;
                        break;
                    }
                }
            }
        }

        optimumDelay[stream] = bestDelay;
    }

    setCableDelay(PortA, std::max(optimumDelay[0], optimumDelay[1]));
    setCableDelay(PortB, std::max(optimumDelay[2], optimumDelay[3]));
    setCableDelay(PortC, std::max(optimumDelay[4], optimumDelay[5]));
    setCableDelay(PortD, std::max(optimumDelay[6], optimumDelay[7]));
    if (type == ControllerRecordUSB3) {
        setCableDelay(PortE, std::max(optimumDelay[8], optimumDelay[9]));
        setCableDelay(PortF, std::max(optimumDelay[10], optimumDelay[11]));
        setCableDelay(PortG, std::max(optimumDelay[12], optimumDelay[13]));
        setCableDelay(PortH, std::max(optimumDelay[14], optimumDelay[15]));
    }

    // Now that we know which chips are plugged into each SPI port, add up the total number of
    // amplifier channels on each port and calcualate the number of data streams necessary to convey
    // this data over the USB interface.
    int numStreamsRequired = 0;
    bool rhd2216ChipPresent = false;
    for (int stream = 0; stream < maxMISOLines; stream++) {
        if (chipTypeOld[stream] == RHD2216Chip) {
            numStreamsRequired++;
            if (numStreamsRequired <= maxNumStreams) numChannelsOnPort[portIndexOld[stream]] += 16;
            rhd2216ChipPresent = true;
        }
        if (chipTypeOld[stream] == RHD2132Chip) {
            numStreamsRequired++;
            if (numStreamsRequired <= maxNumStreams) numChannelsOnPort[portIndexOld[stream]] += 32;
        }
        if (chipTypeOld[stream] == RHD2164Chip) {
            numStreamsRequired += 2;
            if (numStreamsRequired <= maxNumStreams) numChannelsOnPort[portIndexOld[stream]] += 64;
        }
        if (chipTypeOld[stream] == RHS2116Chip) {
            numStreamsRequired++;
            if (numStreamsRequired <= maxNumStreams) numChannelsOnPort[portIndexOld[stream]] += 16;
        }
    }

    // Return a warning if 256-channel capacity of ControllerRecordUSB2 is exceeded.
    if (type == ControllerRecordUSB2 && numStreamsRequired > 8) {
        returnValue = (rhd2216ChipPresent ? -2 : -1);   // -1 == ControllerRecordUSB2 has >256 channels plugged in.
                                                        // -2 == ...and at least one of the chips is an RHD2216.
    }

    // Reconfigure USB data streams in consecutive order to accommodate all connected chips.
    if (type == ControllerRecordUSB2) {
        int stream = 0;
        for (int oldStream = 0; oldStream < maxMISOLines; ++oldStream) {
            if ((chipTypeOld[oldStream] == RHD2216Chip) && (stream < maxNumStreams)) {
                chipType[stream] = RHD2216Chip;
                portIndex[stream] = portIndexOld[oldStream];
                enableDataStream(stream, true);
                setDataSource(stream, initStreamPorts[oldStream]);
                stream++;
            }
            else if ((chipTypeOld[oldStream] == RHD2132Chip) && (stream < maxNumStreams)) {
                chipType[stream] = RHD2132Chip;
                portIndex[stream] = portIndexOld[oldStream];
                enableDataStream(stream, true);
                setDataSource(stream, initStreamPorts[oldStream]);
                stream++;
            }
            else if ((chipTypeOld[oldStream] == RHD2164Chip) && (stream < maxNumStreams - 1)) {
                chipType[stream] = RHD2164Chip;
                chipType[stream + 1] = RHD2164MISOBChip;
                portIndex[stream] = portIndexOld[oldStream];
                portIndex[stream + 1] = portIndexOld[oldStream];
                enableDataStream(stream, true);
                enableDataStream(stream + 1, true);
                setDataSource(stream, initStreamPorts[oldStream]);
                setDataSource(stream + 1, initStreamDdrPorts[oldStream]);
                stream += 2;
            }
        }
        // Note: commmandStream not defined for ControllerRecordUSB2 type.
        for (; stream < maxNumStreams; ++stream) {
            enableDataStream(stream, false);    // Disable unused data streams.
        }
    }
    else if (type == ControllerRecordUSB3) {
        int stream = 0;
        for (int oldStream = 0; oldStream < maxMISOLines; ++oldStream) {
            if ((chipTypeOld[oldStream] == RHD2216Chip) && (stream < maxNumStreams)) {
                chipType[stream] = RHD2216Chip;
                portIndex[stream] = portIndexOld[oldStream];
                enableDataStream(2 * oldStream, true);
                enableDataStream(2 * oldStream + 1, false);
                commandStream[stream] = 2 * oldStream;
                stream++;
            }
            else if ((chipTypeOld[oldStream] == RHD2132Chip) && (stream < maxNumStreams)) {
                chipType[stream] = RHD2132Chip;
                portIndex[stream] = portIndexOld[oldStream];
                enableDataStream(2 * oldStream, true);
                enableDataStream(2 * oldStream + 1, false);
                commandStream[stream] = 2 * oldStream;
                stream++;
            }
            else if ((chipTypeOld[oldStream] == RHD2164Chip) && (stream < maxNumStreams - 1)) {
                chipType[stream] = RHD2164Chip;
                chipType[stream + 1] = RHD2164MISOBChip;
                portIndex[stream] = portIndexOld[oldStream];
                portIndex[stream + 1] = portIndexOld[oldStream];
                enableDataStream(2 * oldStream, true);
                enableDataStream(2 * oldStream + 1, true);
                commandStream[stream] = 2 * oldStream;
                commandStream[stream + 1] = 2 * oldStream + 1;
                stream += 2;
            }
            else {
                enableDataStream(2 * oldStream, false);    // Disable unused data streams.
                enableDataStream(2 * oldStream + 1, false);
            }
        }
    }
    else if (type == ControllerStimRecordUSB2) {
        int stream = 0;
        for (int oldStream = 0; oldStream < maxMISOLines; ++oldStream) {
            if ((chipTypeOld[oldStream] == RHS2116Chip) && (stream < maxNumStreams)) {
                chipType[stream] = RHS2116Chip;
                portIndex[stream] = portIndexOld[oldStream];
                enableDataStream(oldStream, true);
                commandStream[stream] = oldStream;
                stream++;
            }
            else {
                enableDataStream(oldStream, false);    // Disable unused data streams.
            }
        }
    }

    return returnValue;
}


unsigned int OniDevice::numWordsInFifo()
{

    oni_reg_val_t numWordsLsb, numWordsMsb;

    oni_read_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireOutNumWordsLsb, &numWordsLsb);
    oni_read_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireOutNumWordsMsb, &numWordsMsb);

    lastNumWordsInFifo = (numWordsMsb << 16) + numWordsLsb;
    numWordsHasBeenUpdated = true;

    return lastNumWordsInFifo;
}


bool OniDevice::isDcmProgDone() const
{

    oni_reg_val_t value;
    oni_read_reg(ctx, DEVICE_RHYTHM, EndPoint::WireOutDataClkLocked, &value);

    return ((value & 0x0002) > 1);
}


bool OniDevice::isDataClockLocked() const
{
    oni_reg_val_t value;
    oni_read_reg(ctx, DEVICE_RHYTHM, EndPoint::WireOutDataClkLocked, &value);

    return ((value & 0x0001) > 0);
}


void OniDevice::forceAllDataStreamsOff()
{

    oni_write_reg(ctx, DEVICE_RHYTHM, EndPoint::WireInDataStreamEn, 0x00000000);

}

