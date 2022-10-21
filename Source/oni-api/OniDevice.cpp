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

int OniDevice::open(const std::string& boardSerialNumber, const char* libraryFilePath)
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

    uint32_t val = 1;
    oni_set_opt(ctx, ONI_OPT_RESET, &val, sizeof(val));
    oni_set_opt(ctx, ONI_OPT_BLOCKREADSIZE, &usbReadBlockSize, sizeof(usbReadBlockSize));
}


void OniDevice::run()
{
    oni_reg_val_t reg = 2;
    oni_set_opt(ctx, ONI_OPT_RESETACQCOUNTER, &reg, sizeof(oni_size_t));
}


bool OniDevice::isRunning()
{
    oni_reg_val_t val = 0;
    if (oni_read_reg(ctx, DEVICE_RHYTHM, SPI_RUNNING, &val) != ONI_ESUCCESS) return false;
    return val;
}

void OniDevice::flush()
{
    // flush is not needed, as stopping acquisition flushes the board buffers automatically
    oni_size_t reg = 0;
    oni_set_opt(ctx, ONI_OPT_RUNNING, &reg, sizeof(reg));
}


void OniDevice::resetFpga()
{

}

int OniDevice::readFrame(oni_frame_t** frame)
{
    int res;
    bool found = false;
    do
    {
        res = oni_read_frame(ctx, frame);
        if (res < ONI_ESUCCESS) return res;
        if ((*frame)->dev_idx == DEVICE_RHYTHM)
        {
            found = true;
        }
        else
        {
            oni_destroy_frame(*frame);
        }

    } while (!found);
    return res;
}


bool OniDevice::readDataBlock(RHXDataBlock* dataBlock)
{

    oni_frame_t* frame;
    size_t offset = 0;
    size_t bufSize = RHXDataBlock::dataBlockSizeInWords(type, numDataStreams);

    unsigned char* usbBuffer = (unsigned char*)malloc(sizeof(uint16_t) * bufSize);

    if (usbBuffer == NULL)
    {
        std::cerr << "Error allocating usb buffer in readDataBlock" << std::endl;
        return false;
    }

    if (readFrame(&frame) < ONI_ESUCCESS)
    {
        free(usbBuffer);
        return false;
    }

    memcpy(usbBuffer + offset, frame->data + 8, frame->data_sz - 8);
    offset += frame->data_sz - 8;
    oni_destroy_frame(frame);

    dataBlock->fillFromUsbBuffer(usbBuffer, 0);

    free(usbBuffer);
    return true;
}


bool OniDevice::readDataBlocks(int numBlocks, std::deque<RHXDataBlock*>& dataQueue)
{

    size_t bufSize = RHXDataBlock::dataBlockSizeInWords(type, numDataStreams);

    unsigned char* usbBuffer = (unsigned char*)malloc(sizeof(uint16_t) * bufSize);
    if (usbBuffer == NULL)
    {
        std::cerr << "Error allocating usb buffer in readDataBlocks" << std::endl;
        return false;
    }

    RHXDataBlock* dataBlock;
    oni_frame_t* frame;
    size_t offset = 0;;

    if (readFrame(&frame) < ONI_ESUCCESS)
    {
        free(usbBuffer);
        return false;
    }
    memcpy(usbBuffer + offset, frame->data + 8, frame->data_sz - 8);
    offset += frame->data_sz - 8;
    oni_destroy_frame(frame);

    dataBlock = new RHXDataBlock(type, numDataStreams);

    for (int i = 0; i < numBlocks; ++i) {
        dataBlock->fillFromUsbBuffer(usbBuffer, i);
        dataQueue.push_back(dataBlock);
    }
    delete dataBlock;
    free(usbBuffer);
    return true;

    return true;
}


void OniDevice::oni_write_reg_bit(const oni_ctx ctx,
    oni_dev_idx_t dev_idx,
    oni_reg_addr_t addr,
    int bit_index,
    bool state)
{
    oni_reg_val_t register_value;

    oni_read_reg(ctx, dev_idx, addr, &register_value);

    register_value ^= (-state ^ register_value) & (1UL << bit_index);

    oni_write_reg(ctx, dev_idx, addr, register_value);
}

int OniDevice::oni_write_reg_mask(const oni_ctx ctx,
    oni_dev_idx_t dev_idx, 
    oni_reg_addr_t addr, 
    oni_reg_val_t value, 
    unsigned int mask)
{
    int res;
    oni_reg_val_t val;

    res = oni_read_reg(ctx, dev_idx, addr, &val);
    if (res != ONI_ESUCCESS) return res;

    val = (val & ~mask) | (value & mask);
    res = oni_write_reg(ctx, dev_idx, addr, val);
    return res;
}

 
void OniDevice::setContinuousRunMode(bool continuousMode)
{

    oni_reg_val_t val = continuousMode ? 1 << SPI_RUN_CONTINUOUS : 0;
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, MODE, val, 1 << SPI_RUN_CONTINUOUS);

}

void OniDevice::setMaxTimeStep(unsigned int maxTimeStep)
{

    oni_write_reg(ctx, DEVICE_RHYTHM, MAX_TIMESTEP, maxTimeStep);
}

void OniDevice::setCableDelay(BoardPort port, int delay)
{
    int bitShift;

    if (delay < 0 || delay > 15) {
        std::cerr << "Warning in Rhd2000ONIBoard::setCableDelay: delay out of range: " << delay << std::endl;
    }

    if (delay < 0) delay = 0;
    if (delay > 15) delay = 15;

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
    default:
        std::cerr << "Error in Rhd2000ONIBoard::setCableDelay: unknown port." << std::endl;
    }

    oni_write_reg_mask(ctx, DEVICE_RHYTHM, CABLE_DELAY, delay << bitShift, 0x000f << bitShift);
}


void OniDevice::setDspSettle(bool enabled)
{

    oni_reg_val_t val = enabled ? 1 << DSP_SETTLE : 0;
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, MODE, val, 1 << DSP_SETTLE);

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

    std::cout << "OniDevice::setDacManual not implemented." << std::endl;

}


void OniDevice::enableLeds(bool ledsOn)
{

    oni_reg_val_t val = ledsOn ? 1 << LED_ENABLE : 0;
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, MODE, val, 1 << LED_ENABLE);
    
}

// Set output BNC clock divide factor (Open Ephys boards only)
void OniDevice::setClockDivider(int divide_factor)
{

    oni_reg_val_t val = divide_factor;
    oni_write_reg(ctx, DEVICE_RHYTHM, SYNC_CLKOUT_DIVIDE, val);

}


void OniDevice::setDacGain(int gain)
{

    if (gain < 0 || gain > 7) {
        std::cerr << "Error in Rhd2000ONIBoard::setDacGain: gain out of range." << std::endl;
        return;
    }
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, DAC_CTL, gain << 7, 0x07 << 7);

}

// 
void OniDevice::setAudioNoiseSuppress(int noiseSuppress)
{

    if (noiseSuppress < 0 || noiseSuppress > 127) {
        std::cerr << "Error in Rhd2000ONIBoard::setAudioNoiseSuppress: noiseSuppress out of range." << std::endl;
        return;
    }

    oni_write_reg_mask(ctx, DEVICE_RHYTHM, DAC_CTL, noiseSuppress, 0x7F);
}

// 
void OniDevice::setExternalFastSettleChannel(int channel)
{

    if (channel < 0 || channel > 15) {
        std::cerr << "Error in Rhd2000ONIBoard::setExternalFastSettleChannel: channel " << channel << " out of range." << std::endl;
        return;
    }

    oni_reg_val_t val = channel;
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, EXTERNAL_FAST_SETTLE, val, 0x0F);

}

// 
void OniDevice::setExternalDigOutChannel(BoardPort port, int channel)
{

    if (channel < 0 || channel > 15) {
        std::cerr << "Error in Rhd2000ONIBoard::setExternalDigOutChannel: channel out of range." << std::endl;
        return;
    }

    oni_reg_addr_t reg = EXTERNAL_DIGOUT_A + port;
    oni_reg_val_t val = channel;
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, reg, val, 0x0F);

}

// 
void OniDevice::setDacHighpassFilter(double cutoff)
{

    double b;
    int filterCoefficient;
    const double pi = 3.1415926535897;

    // Note that the filter coefficient is a function of the amplifier sample rate, so this
    // function should be called after the sample rate is changed.
    b = 1.0 - exp(-2.0 * pi * cutoff / getSampleRate());

    // In hardware, the filter coefficient is represented as a 16-bit number.
    filterCoefficient = (int)floor(65536.0 * b + 0.5);

    if (filterCoefficient < 1) {
        filterCoefficient = 1;
    }
    else if (filterCoefficient > 65535) {
        filterCoefficient = 65535;
    }

    oni_reg_val_t val = filterCoefficient;
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, HPF, val, 0xFFFF);

}

// 
void OniDevice::setDacThreshold(int dacChannel, int threshold, bool trigPolarity)
{

    if (dacChannel < 0 || dacChannel > 7) {
        std::cerr << "Error in Rhd2000ONIBoard::setDacThreshold: dacChannel out of range." << std::endl;
        return;
    }

    if (threshold < 0 || threshold > 65535) {
        std::cerr << "Error in Rhd2000ONIBoard::setDacThreshold: threshold out of range." << std::endl;
        return;
    }

    oni_reg_addr_t reg = DAC_THRESH_1 + dacChannel;
    oni_reg_val_t val = (threshold & 0x0FFFF) + ((trigPolarity ? 1 : 0) << 16);

    oni_write_reg(ctx, DEVICE_RHYTHM, reg, val);

}


void OniDevice::setTtlMode(int mode)
{

    if (mode < 0 || mode > 1) {
        std::cerr << "Error in Rhd2000ONIBoard::setTtlMode: mode out of range." << std::endl;
        return;
    }

    oni_write_reg_mask(ctx, DEVICE_RHYTHM, MODE, mode << TTL_OUT_MODE, 1 << TTL_OUT_MODE);

}
 
void OniDevice::setDacRerefSource(int stream, int channel)
{

    std::cerr << "OniDevice::setDacRerefSource: not implemented." << std::endl;

}

bool OniDevice::setSampleRate(AmplifierSampleRate newSampleRate)
{

    std::cerr << "OniDevice::setSampleRate: not implemented." << std::endl;

    return true;
}


void OniDevice::setDataSource(int stream, BoardDataSource dataSource)
{

    if (stream < 0 || stream >(maxNumDataStreams() - 1)) {
        std::cerr << "Error in Rhd2000ONIBoard::setDataSource: stream out of range." << std::endl;
        return;
    }

    oni_reg_addr_t reg = DATA_STREAM_1_8_SEL + int(stream / 8);
    oni_reg_val_t bitShift = 4 * (stream % 8);
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, reg, dataSource << bitShift, 0x000f << bitShift);
}


void OniDevice::enableDataStream(int stream, bool enabled)
{

    if (stream < 0 || stream >(maxNumDataStreams() - 1)) {
        std::cerr << "Error in OniDevice::enableDataStream: stream out of range.\n";
        return;
    }

    if (enabled) {
        if (dataStreamEnabled[stream] == 0) {
            oni_write_reg_mask(ctx, DEVICE_RHYTHM, DATA_STREAM_EN, 0x0001 << stream, 0x0001 << stream);
            dataStreamEnabled[stream] = 1;
            ++numDataStreams;
        }
    }
    else {
        if (dataStreamEnabled[stream] == 1) {
            oni_write_reg_mask(ctx, DEVICE_RHYTHM, DATA_STREAM_EN, 0x0000 << stream, 0x0001 << stream);
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

    oni_reg_addr_t reg = DAC_SEL_1 + dacChannel;
    oni_reg_val_t val = enabled ? 0x0400 : 0;
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, reg, val, 0x0400);

}


void OniDevice::enableExternalFastSettle(bool enable)
{

    oni_reg_val_t val = enable ? 1 << 4 : 0;
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, EXTERNAL_FAST_SETTLE, val, 1 << 4);

}

// .
void OniDevice::enableExternalDigOut(BoardPort port, bool enable)
{

    
    oni_reg_addr_t reg = EXTERNAL_DIGOUT_A + port;
    oni_reg_val_t val = enable ? 1 << 4 : 0;
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, reg, val, 1 << 4);
 
}


void OniDevice::enableDacHighpassFilter(bool enable)
{

    oni_reg_val_t val = enable ? 1 << 16 : 0;
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, HPF, val, 1 << 16);

}

void OniDevice::enableDacReref(bool enabled)
{

    std::cout << "OniDevice::enableDacRefer not implemented." << std::endl;

}


void OniDevice::selectDacDataStream(int dacChannel, int stream)
{

    if (dacChannel < 0 || dacChannel > 7) {
        std::cerr << "Error in Rhd2000ONIBoard::selectDacDataStream: dacChannel out of range." << std::endl;
        return;
    }

    if (stream < 0 || stream > maxNumDataStreams() + 1) {
        std::cerr << "Error in Rhd2000ONIBoard::selectDacDataStream: stream out of range." << std::endl;
        return;
    }

    oni_reg_addr_t reg = DAC_SEL_1 + dacChannel;
    oni_reg_val_t val = stream << 5;
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, reg, val, 0x1F << 5);

}


void OniDevice::selectDacDataChannel(int dacChannel, int dataChannel)
{

    if (dacChannel < 0 || dacChannel > 7) {
        std::cerr << "Error in Rhd2000ONIBoard::selectDacDataChannel: dacChannel out of range." << std::endl;
        return;
    }

    if (dataChannel < 0 || dataChannel > 31) {
        std::cerr << "Error in Rhd2000ONIBoard::selectDacDataChannel: dataChannel out of range." << std::endl;
        return;
    }

    oni_reg_addr_t reg = DAC_SEL_1 + dacChannel;
    oni_reg_val_t val = dataChannel;
    oni_write_reg_mask(ctx, DEVICE_RHYTHM, reg, val, 0x1F);

}


void OniDevice::selectAuxCommandLength(AuxCmdSlot auxCommandSlot, int loopIndex, int endIndex)
{

    if (auxCommandSlot != AuxCmd1 && auxCommandSlot != AuxCmd2 && auxCommandSlot != AuxCmd3) {
        std::cerr << "Error in OniDevice::selectAuxCommandLength: auxCommandSlot out of range." << std::endl;
        return;
    }

    if (loopIndex < 0 || loopIndex > 1023) {
        std::cerr << "Error in OniDevice::selectAuxCommandLength: loopIndex out of range." << std::endl;
        return;
    }

    if (endIndex < 0 || endIndex > 1023) {
        std::cerr << "Error in OniDevice::selectAuxCommandLength: endIndex out of range." << std::endl;
        return;
    }

    switch (auxCommandSlot) {
    case AuxCmd1:
        oni_write_reg(ctx, DEVICE_RHYTHM, LOOP_AUXCMD_INDEX_1, loopIndex);
        oni_write_reg(ctx, DEVICE_RHYTHM, MAX_AUXCMD_INDEX_1, endIndex);
        break;
    case AuxCmd2:
        oni_write_reg(ctx, DEVICE_RHYTHM, LOOP_AUXCMD_INDEX_2, loopIndex);
        oni_write_reg(ctx, DEVICE_RHYTHM, MAX_AUXCMD_INDEX_2, endIndex);
        break;
    case AuxCmd3:
        oni_write_reg(ctx, DEVICE_RHYTHM, LOOP_AUXCMD_INDEX_3, loopIndex);
        oni_write_reg(ctx, DEVICE_RHYTHM, MAX_AUXCMD_INDEX_3, endIndex);
        break;
    }


}


void OniDevice::selectAuxCommandBank(BoardPort port, AuxCmdSlot auxCommandSlot, int bank)
{

    int bitShift;

    if (auxCommandSlot != AuxCmd1 && auxCommandSlot != AuxCmd2 && auxCommandSlot != AuxCmd3) {
        std::cerr << "Error in Rhd2000ONIBoard::selectAuxCommandBank: auxCommandSlot out of range." << std::endl;
        return;
    }
    if (bank < 0 || bank > 15) {
        std::cerr << "Error in Rhd2000ONIBoard::selectAuxCommandBank: bank out of range." << std::endl;
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
    }

    switch (auxCommandSlot) {
    case AuxCmd1:
        oni_write_reg_mask(ctx, DEVICE_RHYTHM, AUXCMD_BANK_1, bank << bitShift, 0x000f << bitShift);
        break;
    case AuxCmd2:
        oni_write_reg_mask(ctx, DEVICE_RHYTHM, AUXCMD_BANK_2, bank << bitShift, 0x000f << bitShift);
        break;
    case AuxCmd3:
        oni_write_reg_mask(ctx, DEVICE_RHYTHM, AUXCMD_BANK_3, bank << bitShift, 0x000f << bitShift);
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

    // CHECK

    if (auxCommandSlot != AuxCmd1 && auxCommandSlot != AuxCmd2 && auxCommandSlot != AuxCmd3) {
        std::cerr << "Error in OniDevice::uploadCommandList: auxCommandSlot out of range.\n";
        return;
    }

    if ((bank < 0) || (bank > 15)) {
        std::cerr << "Error in OniDevice::uploadCommandList: bank out of range.\n";
        return;
    }

    oni_reg_addr_t base_address = 0x4000;

    for (unsigned int i = 0; i < commandList.size(); ++i) 
    {
        
        oni_reg_addr_t bank_select = (bank << 10);
 
        oni_reg_addr_t aux_select;

        switch (auxCommandSlot) {
        case AuxCmd1:
            aux_select = (1 << 14);
            break;
        case AuxCmd2:
            aux_select = (2 << 14);
            break;
        case AuxCmd3:
            aux_select = (3 << 14);
            break;
        case AuxCmd4:
            // Should not be reached, as AuxCmd4 is Stim-only.
            break;
        }

        oni_write_reg(ctx, DEVICE_RHYTHM, base_address + bank_select + aux_select + i, commandList[i]);

    }
  
}

void OniDevice::printCommandList(const std::vector<unsigned int>& commandList) const
{
    unsigned int i;
    int cmd, channel, reg, data;

    std::cout << std::endl;
    for (i = 0; i < commandList.size(); ++i) {
        cmd = commandList[i];
        if (cmd < 0 || cmd > 0xffff) {
            std::cout << "  command[" << i << "] = INVALID COMMAND: " << cmd << std::endl;
        }
        else if ((cmd & 0xc000) == 0x0000) {
            channel = (cmd & 0x3f00) >> 8;
            std::cout << "  command[" << i << "] = CONVERT(" << channel << ")" << std::endl;
        }
        else if ((cmd & 0xc000) == 0xc000) {
            reg = (cmd & 0x3f00) >> 8;
            std::cout << "  command[" << i << "] = READ(" << reg << ")" << std::endl;
        }
        else if ((cmd & 0xc000) == 0x8000) {
            reg = (cmd & 0x3f00) >> 8;
            data = (cmd & 0x00ff);
            std::cout << "  command[" << i << "] = WRITE(" << reg << ",";
            std::cout << std::hex << std::uppercase << std::internal << std::setfill('0') << std::setw(2) << data << std::nouppercase << std::dec;
            std::cout << ")" << std::endl;
        }
        else if (cmd == 0x5500) {
            std::cout << "  command[" << i << "] = CALIBRATE" << std::endl;
        }
        else if (cmd == 0x6a00) {
            std::cout << "  command[" << i << "] = CLEAR" << std::endl;
        }
        else {
            std::cout << "  command[" << i << "] = INVALID COMMAND: ";
            std::cout << std::hex << std::uppercase << std::internal << std::setfill('0') << std::setw(4) << cmd << std::nouppercase << std::dec;
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;
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

    // how to get this info?

    oni_reg_val_t numWordsLsb, numWordsMsb;

    //oni_read_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireOutNumWordsLsb, &numWordsLsb);
    //oni_read_reg(ctx, DEVICE_RHYTHM, OkEndPoint::WireOutNumWordsMsb, &numWordsMsb);

    lastNumWordsInFifo = (numWordsMsb << 16) + numWordsLsb;
    numWordsHasBeenUpdated = true;

    return lastNumWordsInFifo;
}


bool OniDevice::isDcmProgDone() const
{

    // DCM programming not supported

    return true;
}


bool OniDevice::isDataClockLocked() const
{
    // data clock cannot be locked?

    return false;
}


void OniDevice::forceAllDataStreamsOff()
{

    oni_write_reg(ctx, DEVICE_RHYTHM, ENABLE, 0);

}

