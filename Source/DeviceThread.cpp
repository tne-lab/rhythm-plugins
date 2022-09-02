/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2016 Open Ephys

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

#ifdef _WIN32
#define NOMINMAX
#endif

#include "DeviceThread.h"
#include "DeviceEditor.h"

#include "ImpedanceMeter.h"
#include "Headstage.h"
#include "USBThread.h"

#include "rhx-api/Hardware/rhxglobals.h"
#include "rhx-api/Hardware/rhxcontroller.h"
#include "oni-api/OniDevice.h"


#if defined(_WIN32)
#define okLIB_NAME "okFrontPanel.dll"
#define okLIB_EXTENSION "*.dll"
#elif defined(__APPLE__)
#define okLIB_NAME "libokFrontPanel.dylib"
#define okLIB_EXTENSION "*.dylib"
#elif defined(__linux__)
#define okLIB_NAME "./libokFrontPanel.so"
#define okLIB_EXTENSION "*.so"
#endif

using namespace RhythmNode;

BoardType DeviceThread::boardType = ACQUISITION_BOARD; // initialize static member

DataThread* DeviceThread::createDataThread(SourceNode *sn)
{
    return new DeviceThread(sn, boardType);
}

DeviceThread::DeviceThread(SourceNode* sn, BoardType boardType_) : DataThread(sn),
    deviceFound(false),
    isTransmitting(false),
    channelNamingScheme(GLOBAL_INDEX),
    updateSettingsDuringAcquisition(false)
{

    boardType = boardType_;

    impedanceThread = new ImpedanceMeter(this);

    for (int i = 0; i < 8; i++)
        adcRangeSettings[i] = 0;

    int maxNumHeadstages = (boardType == RHD_RECORDING_CONTROLLER) ? 16 : 8;

    for (int i = 0; i < maxNumHeadstages; i++)
        headstages.add(new Headstage(i, maxNumHeadstages));

    sourceBuffers.add(new DataBuffer(2, 10000)); // start with 2 channels and automatically resize

    dacStream = new int[8];
    dacChannels = new int[8];
    dacThresholds = new float[8];
    dacChannelsToUpdate = new bool[8];


#if defined(__APPLE__)
    File appBundle = File::getSpecialLocation(File::currentApplicationFile);
    const String executableDirectory = appBundle.getChildFile("Contents/Resources").getFullPathName();
#else
    File executable = File::getSpecialLocation(File::currentExecutableFile);
    const String executableDirectory = executable.getParentDirectory().getFullPathName();
#endif

    String dirName = executableDirectory;
    String libraryFilePath = dirName;
    libraryFilePath += File::getSeparatorString();
    libraryFilePath += okLIB_NAME;

    std::cout << "DLL search path: " << libraryFilePath << std::endl;

    if (!okFrontPanelDLL_LoadLib(libraryFilePath.getCharPointer())) {
                std::cerr << "FrontPanel DLL could not be loaded.  " <<
                        "Make sure this DLL is in the application start directory." << std::endl;
    }

    char dll_date[32], dll_time[32];
    std::string serialNumber = "1613000E2W";

    okFrontPanelDLL_GetVersion(dll_date, dll_time);

    std::cout << std::endl << "FrontPanel DLL loaded.  Built: " << dll_date << "  " << dll_time << std::endl;

    frontPanelLib = std::make_unique<okCFrontPanel>();

    std::cout << std::endl << "Scanning USB for Opal Kelly devices..." << std::endl << std::endl;

    int nDevices = frontPanelLib->GetDeviceCount(); 

    std::cout << "Found " << nDevices << " Opal Kelly device" << ((nDevices == 1) ? "" : "s") <<
        " connected:" << std::endl;

    for (int i = 0; i < nDevices; ++i) {
        std::cout << "  Device #" << i + 1 << ": Opal Kelly " <<
            frontPanelLib->opalKellyModelName(frontPanelLib->GetDeviceListModel(i)).c_str() <<
            " with serial number " << frontPanelLib->GetDeviceListSerial(i).c_str() << std::endl;
    }

    std::cout << std::endl;

    for (int i = 0; i < nDevices; ++i)
    {
        okCFrontPanel::BoardModel model = frontPanelLib->GetDeviceListModel(i);

        if (model == OK_PRODUCT_XEM6010LX45 || model == OK_PRODUCT_XEM6310LX45) //the two models we use
        {
            serialNumber = serialNumber = frontPanelLib->GetDeviceListSerial(i);

            if (model == OK_PRODUCT_XEM6310LX45)
                usbVersion = USB3;
            else
                usbVersion = USB2;

            std::cout << "Trying to open device with serial " << serialNumber.c_str() << std::endl;
            std::cout << "USB Version: " << usbVersion << std::endl;
        }
    }

    frontPanelLib.reset();

    AmplifierSampleRate ampSampleRate = AmplifierSampleRate::SampleRate30000Hz;

    if (boardType == ACQUISITION_BOARD)
    {
        if (usbVersion == USB3)
            device = std::make_unique<RHXController>(ControllerType::ControllerOEOpalKellyUSB3, ampSampleRate);
        else
            device = std::make_unique<RHXController>(ControllerType::ControllerOEOpalKellyUSB2, ampSampleRate);
    }
    else if (boardType == INTAN_RHD_USB)
        device = std::make_unique<RHXController>(ControllerType::ControllerRecordUSB2, ampSampleRate);
    else if (boardType == RHD_RECORDING_CONTROLLER)
        device = std::make_unique<RHXController>(ControllerType::ControllerRecordUSB3, ampSampleRate);
    else if (boardType == RHS_STIM_RECORDING_CONTROLLER)
        device = std::make_unique<RHXController>(ControllerType::ControllerStimRecordUSB2, ampSampleRate);
    else if (boardType == ONI_USB)
        device = std::make_unique<OniDevice>();
    
    // 1. attempt to open the device
    int errorCode = device->open(serialNumber.c_str());
    
    if (errorCode == -1)
    {
        AlertWindow::showMessageBox(AlertWindow::NoIcon,
                                                     "Opal Kelly library not found.",
                                                     "The Opal Kelly library file was not found in the directory of the executable. Please locate it and re-add the plugin to the signal chain.",
                                                     "Ok.");

        return;

    } else if (errorCode == -2)
    {
        AlertWindow::showMessageBox(AlertWindow::NoIcon,
                                                     "No device was found.",
                                                     "Please connect one and re-add the plugin to the signal chain.",
                                                     "Ok.");

        return;
    } 

    String bitfilename;

    bitfilename = executableDirectory;
    bitfilename += File::getSeparatorString();
    bitfilename += "shared";
    bitfilename += File::getSeparatorString();

    if (boardType == ACQUISITION_BOARD)
        bitfilename += usbVersion == USB3 ? "rhd2000_usb3.bit" : "rhd2000.bit";
    else if (boardType == INTAN_RHD_USB)
        bitfilename += "intan_rhd_usb.bit";
    else if (boardType == RHD_RECORDING_CONTROLLER)
        bitfilename += "intan_recording_controller.bit";
    else if (boardType == RHS_STIM_RECORDING_CONTROLLER)
        bitfilename += "intan_stim_record_controller.bit";

    bool success = true;
    
    if (boardType != ONI_USB)
    {

        if (!File(bitfilename).exists())
        {
            AlertWindow::showMessageBox(AlertWindow::NoIcon,
                "No bitfile was found.",
                "Please place one in the appropriate directory and re-add the plugin to the signal chain."
                "Ok.");
        }

        device->uploadFPGABitfile(bitfilename.toStdString());

        LOGD("Initializing device.");

        device->initialize();

        deviceFound = true;
    } 

    if (success)
    {
        scanPorts();
        
        for (int k = 0; k < 8; k++)
        {
            dacChannelsToUpdate[k] = true;
            dacStream[k] = 0;
            setDACthreshold(k, 65534);
            dacChannels[k] = 0;
            dacThresholds[k] = 0;
        }
    }
    
}

DeviceThread::~DeviceThread()
{

    LOGD("Closing device.");
    
    //device->close();

    delete[] dacStream;
    delete[] dacChannels;
    delete[] dacThresholds;
    delete[] dacChannelsToUpdate;
}

void DeviceThread::initialize(bool signalChainIsLoading)
{

    adcChannelNames.clear();
    ttlLineNames.clear();

    for (int i = 0; i < 8; i++)
    {
        adcChannelNames.add("ADC" + String(i + 1));
        ttlLineNames.add("TTL" + String(i + 1));
    }
}

std::unique_ptr<GenericEditor> DeviceThread::createEditor(SourceNode* sn)
{

    std::unique_ptr<DeviceEditor> editor = std::make_unique<DeviceEditor>(sn, this);

    return editor;
}

void DeviceThread::handleBroadcastMessage(String msg)
{
    StringArray parts = StringArray::fromTokens(msg, " ", "");

    //std::cout << "Received " << msg << std::endl;

    if (parts[0].equalsIgnoreCase("ACQBOARD"))
    {
        if (parts.size() > 1)
        {
            String command = parts[1];

            if (command.equalsIgnoreCase("TRIGGER"))
            {
                if (parts.size() == 4)
                {
                    int ttlLine = parts[2].getIntValue() - 1;

                    if (ttlLine < 0 || ttlLine > 7)
                        return;

                    int eventDurationMs = parts[3].getIntValue();

                    if (eventDurationMs < 10 || eventDurationMs > 5000)
                        return;

                    DigitalOutputCommand command;
                    command.ttlLine = ttlLine;
                    command.state = true;

                    digitalOutputCommands.push(command);

                    DigitalOutputTimer* timer = new DigitalOutputTimer(this, ttlLine, eventDurationMs);

                    digitalOutputTimers.add(timer);

                }
            }
        }
    }

}


void DeviceThread::addDigitalOutputCommand(DigitalOutputTimer* timerToDelete, int ttlLine, bool state)
{
    DigitalOutputCommand command;
    command.ttlLine = ttlLine;
    command.state = state;

    digitalOutputCommands.push(command);

    digitalOutputTimers.removeObject(timerToDelete);
}

DeviceThread::DigitalOutputTimer::DigitalOutputTimer(DeviceThread* board_, int ttlLine_, int eventDurationMs)
    : board(board_)
{

    tllOutputLine = ttlLine_;

    startTimer(eventDurationMs);
}

void DeviceThread::DigitalOutputTimer::timerCallback()
{
    stopTimer();

    board->addDigitalOutputCommand(this, tllOutputLine, false);
}

void DeviceThread::setDACthreshold(int dacOutput, float threshold)
{
    dacThresholds[dacOutput]= threshold;
    dacChannelsToUpdate[dacOutput] = true;
    updateSettingsDuringAcquisition = true;

    //evalBoard->setDacThresholdVoltage(dacOutput,threshold);
}

void DeviceThread::setDACchannel(int dacOutput, int channel)
{
    if (channel < getNumDataOutputs(ContinuousChannel::ELECTRODE))
    {
        int channelCount = 0;

        for (int i = 0; i < enabledStreams.size(); i++)
        {
            if (channel < channelCount + numChannelsPerDataStream[i])
            {
                dacChannels[dacOutput] = channel - channelCount;
                dacStream[dacOutput] = i;
                break;
            }
            else
            {
                channelCount += numChannelsPerDataStream[i];
            }
        }
        dacChannelsToUpdate[dacOutput] = true;
        updateSettingsDuringAcquisition = true;
    }
}

Array<int> DeviceThread::getDACchannels() const
{
    Array<int> dacChannelsArray;

    for (int k = 0; k < 8; ++k)
    {
        dacChannelsArray.add (dacChannels[k]);
    }

    return dacChannelsArray;
}


void DeviceThread::scanPorts()
{
    if (!deviceFound) //Safety to avoid crashes if board not present
    {
        return;
    }

    impedanceThread->stopThreadSafely();

    //Clear previous known streams
    enabledStreams.clear();

    std::vector<ChipType> chipType;
    std::vector<int> portIndex;
    std::vector<int> commandStream;
    std::vector<int> numChannelsOnPort;

    device->findConnectedChips(chipType, portIndex, commandStream, numChannelsOnPort);

    // initialize headstage objects

    for (int i = 0; i < chipType.size(); i++)
    {
        LOGC(int(chipType[i]), " ", portIndex[i], " ", commandStream[i], " ", numChannelsOnPort[i]);
    }
}



void DeviceThread::updateSettings(OwnedArray<ContinuousChannel>* continuousChannels,
    OwnedArray<EventChannel>* eventChannels,
    OwnedArray<SpikeChannel>* spikeChannels,
    OwnedArray<DataStream>* sourceStreams,
    OwnedArray<DeviceInfo>* devices,
    OwnedArray<ConfigurationObject>* configurationObjects)
{

    if (!deviceFound)
        return;

    continuousChannels->clear();
    eventChannels->clear();
    spikeChannels->clear();
    sourceStreams->clear();
    devices->clear();
    configurationObjects->clear();

    channelNames.clear();

    // create device
    // CODE GOES HERE

    DataStream::Settings dataStreamSettings
    {
        "Device Data",
        "description",
        "identifier",

        static_cast<float>(device->getSampleRate())

    };

    DataStream* stream = new DataStream(dataStreamSettings);

    sourceStreams->add(stream);

    int hsIndex = -1;

    for (auto headstage : headstages)
    {
        hsIndex++;

        if (headstage->isConnected())
        {
            for (int ch = 0; ch < headstage->getNumChannels(); ch++)
            {

                ContinuousChannel::Settings channelSettings{
                    ContinuousChannel::ELECTRODE,
                    headstage->getChannelName(ch),
                    "description",
                    "identifier",

                    0.195,

                    stream
                };

                continuousChannels->add(new ContinuousChannel(channelSettings));
                continuousChannels->getLast()->setUnits("uV");

                if (impedances.valid)
                {
                    continuousChannels->getLast()->impedance.magnitude = headstage->getImpedanceMagnitude(ch);
                    continuousChannels->getLast()->impedance.phase = headstage->getImpedancePhase(ch);
                }

            }

            if (settings.acquireAux)
            {
                for (int ch = 0; ch < 3; ch++)
                {

                    ContinuousChannel::Settings channelSettings{
                        ContinuousChannel::AUX,
                        headstage->getStreamPrefix() + "_AUX" + String(ch + 1),
                        "description",
                        "identifier",

                        0.0000374,

                        stream
                    };

                    continuousChannels->add(new ContinuousChannel(channelSettings));
                    continuousChannels->getLast()->setUnits("mV");

                }
            }
        }
    }

    if (settings.acquireAdc)
    {
        for (int ch = 0; ch < 8; ch++)
        {

            String name = "ADC" + String(ch + 1);

            ContinuousChannel::Settings channelSettings{
                ContinuousChannel::ADC,
                name,
                "description",
                "identifier",

                getAdcBitVolts(ch),

                stream
            };

            continuousChannels->add(new ContinuousChannel(channelSettings));
            continuousChannels->getLast()->setUnits("V");

        }
    }

    EventChannel::Settings settings{
            EventChannel::Type::TTL,
            "name",
            "description",
            "identifier",
            stream,
            8
    };

    eventChannels->add(new EventChannel(settings));

}

void DeviceThread::impedanceMeasurementFinished()
{

    if (impedances.valid)
    {
        LOGD( "Updating headstage impedance values" );

        for (auto hs : headstages)
        {
            if (hs->isConnected())
            {
                hs->setImpedances(impedances);
            }
        }
    }
}

void DeviceThread::saveImpedances(File& file)
{

    if (impedances.valid)
    {
        std::unique_ptr<XmlElement> xml = std::unique_ptr<XmlElement>(new XmlElement("IMPEDANCES"));

        int globalChannelNumber = -1;

        for (auto hs : headstages)
        {
            XmlElement* headstageXml = new XmlElement("HEADSTAGE");
            headstageXml->setAttribute("name", hs->getStreamPrefix());

            for (int ch = 0; ch < hs->getNumActiveChannels(); ch++)
            {
                globalChannelNumber++;

                XmlElement* channelXml = new XmlElement("CHANNEL");
                channelXml->setAttribute("name", hs->getChannelName(ch));
                channelXml->setAttribute("number", globalChannelNumber);
                channelXml->setAttribute("magnitude", hs->getImpedanceMagnitude(ch));
                channelXml->setAttribute("phase", hs->getImpedancePhase(ch));
                headstageXml->addChildElement(channelXml);
            }

            xml->addChildElement(headstageXml);
        }

        xml->writeTo(file);
    }

}

String DeviceThread::getChannelName(int i) const
{
    return channelNames[i];
}

bool DeviceThread::isAcquisitionActive() const
{
    return isTransmitting;
}

void DeviceThread::setNamingScheme(ChannelNamingScheme scheme)
{

    channelNamingScheme = scheme;

    for (auto hs : headstages)
    {
        hs->setNamingScheme(scheme);
    }
}

ChannelNamingScheme DeviceThread::getNamingScheme()
{
    return channelNamingScheme;
}

void DeviceThread::setNumChannels(int hsNum, int numChannels)
{
    if (headstages[hsNum]->getNumChannels() == 32)
    {
        if (numChannels < headstages[hsNum]->getNumChannels())
            headstages[hsNum]->setHalfChannels(true);
        else
            headstages[hsNum]->setHalfChannels(false);

        numChannelsPerDataStream.set(headstages[hsNum]->getStreamIndex(0), numChannels);
    }

    int channelIndex = 0;

    for (auto hs : headstages)
    {
        if (hs->isConnected())
        {
            hs->setFirstChannel(channelIndex);

            channelIndex += hs->getNumActiveChannels();
        }
    }
}

int DeviceThread::getHeadstageChannels (int hsNum) const
{
    return headstages[hsNum]->getNumChannels();
}


int DeviceThread::getNumChannels()
{
    int totalChannels = getNumDataOutputs(ContinuousChannel::ELECTRODE)
           + getNumDataOutputs(ContinuousChannel::AUX)
           + getNumDataOutputs(ContinuousChannel::ADC);

    return totalChannels;
}

int DeviceThread::getNumDataOutputs(ContinuousChannel::Type type)
{

    if (type == ContinuousChannel::ELECTRODE)
    {
        int totalChannels = 0;

        for (auto headstage : headstages)
        {
            if (headstage->isConnected())
            {
                totalChannels += headstage->getNumActiveChannels();
            }
        }

        return totalChannels;
    }
    if (type == ContinuousChannel::AUX)
    {
        if (settings.acquireAux)
        {
            int numAuxOutputs = 0;

            for (auto headstage : headstages)
            {
                if (headstage->isConnected())
                {
                    numAuxOutputs += 3;
                }
            }
            return numAuxOutputs;
        }
        else
        {
            return 0;
        }
    }
    if (type == ContinuousChannel::ADC)
    {
        if (settings.acquireAdc)
        {
            return 8;
        }
        else
        {
            return 0;
        }
    }

    return 0;
}


float DeviceThread::getAdcBitVolts (int chan) const
{
    if (chan < adcBitVolts.size())
    {
        return adcBitVolts[chan];
    }
    else
    {
        if (boardType == ACQUISITION_BOARD)
        {
            return 0.00015258789; // +/-5V / pow(2,16)
        }
        else if (boardType == INTAN_RHD_USB)
        {
            return 0.0000503540039;  // 3.3V / pow(2,16)
        }
    }
}

double DeviceThread::setUpperBandwidth(double upper)
{
    impedanceThread->stopThreadSafely();

    settings.dsp.upperBandwidth = upper;

    //updateRegisters();
    LOGC("DeviceThread::setUpperBandwidth NOT IMPLEMENTED.");

    return settings.dsp.upperBandwidth;
}


double DeviceThread::setLowerBandwidth(double lower)
{
    impedanceThread->stopThreadSafely();

    settings.dsp.lowerBandwidth = lower;

    LOGC("DeviceThread::setLowerBandwidth NOT IMPLEMENTED.");
//updateRegisters();

    return settings.dsp.lowerBandwidth;
}

double DeviceThread::setDspCutoffFreq(double freq)
{
    impedanceThread->stopThreadSafely();

    settings.dsp.cutoffFreq = freq;

    device->setDacHighpassFilter(freq);

    return settings.dsp.cutoffFreq;
}

double DeviceThread::getDspCutoffFreq() const
{
    return settings.dsp.cutoffFreq;
}

void DeviceThread::setDSPOffset(bool state)
{

    impedanceThread->stopThreadSafely();

    settings.dsp.enabled = state;

    LOGC("DeviceThread::setDSPOffset NOT IMPLEMENTED.");
}

void DeviceThread::setTTLoutputMode(bool state)
{
    settings.ttlMode = state;

    updateSettingsDuringAcquisition = true;
}

void DeviceThread::setDAChpf(float cutoff, bool enabled)
{
    settings.desiredDAChpf = cutoff;

    settings.desiredDAChpfState = enabled;

    updateSettingsDuringAcquisition = true;
}

void DeviceThread::setFastTTLSettle(bool state, int channel)
{
    settings.fastTTLSettleEnabled = state;

    settings.fastSettleTTLChannel = channel;

    updateSettingsDuringAcquisition = true;
}

int DeviceThread::setNoiseSlicerLevel(int level)
{
    settings.noiseSlicerLevel = level;

    LOGC("DeviceThread::setNoiseSlicerLevel NOT IMPLEMENTED.");

    // Level has been checked once before this and then is checked again in setAudioNoiseSuppress.
    // This may be overkill - maybe API should change so that the final function returns the value?

    return settings.noiseSlicerLevel;
}


bool DeviceThread::foundInputSource()
{
    return deviceFound;
}

bool DeviceThread::enableHeadstage(int hsNum, bool enabled, int nStr, int strChans)
{

    if (enabled)
    {
        headstages[hsNum]->setFirstChannel(getNumDataOutputs(ContinuousChannel::ELECTRODE));
        headstages[hsNum]->setNumStreams(nStr);
        headstages[hsNum]->setChannelsPerStream(strChans);
        headstages[hsNum]->setFirstStreamIndex(enabledStreams.size());
        enabledStreams.add(headstages[hsNum]->getDataStream(0));
        numChannelsPerDataStream.add(strChans);

        if (nStr > 1)
        {
            enabledStreams.add(headstages[hsNum]->getDataStream(1));
            numChannelsPerDataStream.add(strChans);
        }
    }
    else
    {
        int idx = enabledStreams.indexOf(headstages[hsNum]->getDataStream(0));

        if (idx >= 0)
        {
            enabledStreams.remove(idx);
            numChannelsPerDataStream.remove(idx);
        }

        if (headstages[hsNum]->getNumStreams() > 1)
        {
            idx = enabledStreams.indexOf(headstages[hsNum]->getDataStream(1));
            if (idx >= 0)
            {
                enabledStreams.remove(idx);
                numChannelsPerDataStream.remove(idx);
            }
        }

        headstages[hsNum]->setNumStreams(0);
    }

    sourceBuffers[0]->resize(getNumChannels(), 10000);

    return true;
}

bool DeviceThread::isHeadstageEnabled(int hsNum) const
{
    return headstages[hsNum]->isConnected();
}


int DeviceThread::getActiveChannelsInHeadstage (int hsNum) const
{
    return headstages[hsNum]->getNumActiveChannels();
}

int DeviceThread::getChannelsInHeadstage (int hsNum) const
{
    return headstages[hsNum]->getNumChannels();
}

/*void DeviceThread::assignAudioOut(int dacChannel, int dataChannel)
{
    if (deviceFound)
    {
        if (dacChannel == 0)
        {
            audioOutputR = dataChannel;
            dacChannels[0] = dataChannel;
        }
        else if (dacChannel == 1)
        {
            audioOutputL = dataChannel;
            dacChannels[1] = dataChannel;
        }

        updateSettingsDuringAcquisition = true; // set a flag and take care of setting wires
        // during the updateBuffer() method
        // to avoid problems
    }

}*/

void DeviceThread::enableAuxs(bool t)
{
    settings.acquireAux = t;
    sourceBuffers[0]->resize(getNumChannels(), 10000);
}

void DeviceThread::enableAdcs(bool t)
{
    settings.acquireAdc = t;
    sourceBuffers[0]->resize(getNumChannels(), 10000);
}

bool DeviceThread::isAuxEnabled()
{
    return settings.acquireAux;
}

void DeviceThread::setSampleRate(int sampleRateIndex, bool isTemporary)
{
    impedanceThread->stopThreadSafely();
    
    if (!isTemporary)
    {
        settings.savedSampleRateIndex = sampleRateIndex;
    }
    
    device->setSampleRate((AmplifierSampleRate) sampleRateIndex);
}

unsigned int DeviceThread::calculateDataBlockSizeInWords(int numDataStreams, bool usb3, int nSamples)
{
    unsigned int samps = nSamples <= 0 ? SAMPLES_PER_DATA_BLOCK(usb3) : nSamples;
    return samps * (4 + 2 + numDataStreams * 36 + 8 + 2);
}

bool DeviceThread::startAcquisition()
{
    
    impedanceThread->waitSafely();
    
    if (!deviceFound)
        return false;

    dataBlock.reset();

    dataBlock = std::make_unique<RHXDataBlock>(device->getType(), device->getNumEnabledDataStreams());

    // TODO: Fix this
    blockSize = calculateDataBlockSizeInWords(device->getNumEnabledDataStreams(), usbVersion == USB3, 1);

    LOGD("Starting usb thread with buffer of ", blockSize * 2, " bytes");
    usbThread->startAcquisition(blockSize * 2);

    int ledArray[8] = { 1, 1, 0, 0, 0, 0, 0, 0 };
    device->setLedDisplay(ledArray);

    device->flush();
    device->setContinuousRunMode(true);
    device->run();
    
    // reset TTL output state
    for (int k = 0; k < 16; k++)
    {
        TTL_OUTPUT_STATE[k] = 0;
    }
    
    startThread();

    return true;
}

bool DeviceThread::stopAcquisition()
{

    LOGD("DeviceThread stopping acquisition.");

    usbThread->stopAcquisition();

    if (isThreadRunning())
    {
        signalThreadShouldExit();

    }

    if (waitForThreadToExit(500))
    {
        LOGD("DeviceThread exited.");
    }
    else
    {
        LOGD("DeviceThread failed to exit, continuing anyway...");
    }

    if (deviceFound)
    {
        device->setContinuousRunMode(false);
        device->setMaxTimeStep(0);
        LOGD("Flushing FIFO.");
        device->flush();
    }

    if (deviceFound && boardType == ACQUISITION_BOARD)
    {
        int ledArray[8] = { 1, 0, 0, 0, 0, 0, 0, 0 };
        device->setLedDisplay(ledArray);
    }

    sourceBuffers[0]->clear();
    
    isTransmitting = false;
    updateSettingsDuringAcquisition = false;

    // remove timers
    digitalOutputTimers.clear();

    // remove commands
    while (!digitalOutputCommands.empty())
        digitalOutputCommands.pop();

    return true;
}

bool DeviceThread::updateBuffer()
{
            
    unsigned char* bufferPtr;
    double ts;

    if (usbVersion == USB3 || device->getNumWordsInFifo() >= blockSize)
    {

        if (usbThread->usbRead(bufferPtr) == 0)
            return true;

        int index = 0;
        int auxIndex, chanIndex;
        int numStreams = enabledStreams.size();
        int nSamps = dataBlock->samplesPerDataBlock();

        //evalBoard->printFIFOmetrics();
        for (int samp = 0; samp < nSamps; samp++)
        {
            int channel = -1;

            if (!dataBlock->checkUsbHeader(bufferPtr, index))
            {
                LOGE("Error in rhxcontroller::readDataBlock: Incorrect header.");
                break;
            }

            index += 8; // magic number header width (bytes)
            int64 timestamp = dataBlock->convertUsbTimeStamp(bufferPtr, index);
            index += 4; // timestamp width
            auxIndex = index; // aux chans start at this offset
            index += 6 * numStreams; // width of the 3 aux chans

            for (int dataStream = 0; dataStream < numStreams; dataStream++)
            {

                int nChans = numChannelsPerDataStream[dataStream];

                chanIndex = index + 2 * dataStream;

                if ((chipId[dataStream] == CHIP_ID_RHD2132) && (nChans == 16)) //RHD2132 16ch. headstage
                {
                    chanIndex += 2 * RHD2132_16CH_OFFSET * numStreams;
                }

                for (int chan = 0; chan < nChans; chan++)
                {
                    channel++;
                    thisSample[channel] = float(*(uint16*)(bufferPtr + chanIndex) - 32768) * 0.195f;
                    chanIndex += 2 * numStreams; // single chan width (2 bytes)
                }

            }
            index += 64 * numStreams; // neural data width
            auxIndex += 2 * numStreams; // skip AuxCmd1 slots (see updateRegisters())
            // copy the 3 aux channels
            if (settings.acquireAux)
            {
                for (int dataStream = 0; dataStream < numStreams; dataStream++)
                {
                    if (chipId[dataStream] != CHIP_ID_RHD2164_B)
                    {
                        int auxNum = (samp + 3) % 4;
                        if (auxNum < 3)
                        {
                            auxSamples[dataStream][auxNum] = float(*(uint16*)(bufferPtr + auxIndex) - 32768) * 0.0000374;
                        }
                        for (int chan = 0; chan < 3; chan++)
                        {
                            channel++;
                            if (auxNum == 3)
                            {
                                auxBuffer[channel] = auxSamples[dataStream][chan];
                            }
                            thisSample[channel] = auxBuffer[channel];
                        }
                    }
                    auxIndex += 2; // single chan width (2 bytes)
                }
            }
            index += 2 * numStreams; // skip over filler word at the end of each data stream
            // copy the 8 ADC channels
            if (settings.acquireAdc)
            {
                for (int adcChan = 0; adcChan < 8; ++adcChan)
                {

                    channel++;
                    // ADC waveform units = volts

                    if (boardType == ACQUISITION_BOARD)
                    {
                        thisSample[channel] = adcRangeSettings[adcChan] == 0 ?
                            0.00015258789 * float(*(uint16*)(bufferPtr + index)) - 5 - 0.4096 : // account for +/-5V input range and DC offset
                            0.00030517578 * float(*(uint16*)(bufferPtr + index)); 
                    }
                    else if (boardType == INTAN_RHD_USB) {
                        thisSample[channel] = 0.000050354 * float(*(uint16*)(bufferPtr + index));
                    }
                    index += 2; // single chan width (2 bytes)
                }
            }
            else
            {
                index += 16; // skip ADC chans (8 * 2 bytes)
            }

            uint64 ttlEventWord = *(uint64*)(bufferPtr + index) & 65535;

            index += 4;

            sourceBuffers[0]->addToBuffer(thisSample,
                &timestamp,
                &ts,
                &ttlEventWord,
                1);
        }

    }

    if (updateSettingsDuringAcquisition)
    {
        for (int k = 0; k < 8; k++)
        {
            if (dacChannelsToUpdate[k])
            {
                dacChannelsToUpdate[k] = false;
                
                if (dacChannels[k] >= 0)
                {
                    device->enableDac(k, true);
                    device->selectDacDataStream(k, dacStream[k]);
                    device->selectDacDataChannel(k, dacChannels[k]);
                    device->setDacThreshold(k, (int)abs((dacThresholds[k]/0.195) + 32768),dacThresholds[k] >= 0);
                   // evalBoard->setDacThresholdVoltage(k, (int) dacThresholds[k]);
                }
                else
                {
                    device->enableDac(k, false);
                }
            }
        }

        device->setTtlMode(settings.ttlMode ? 1 : 0);
        device->enableExternalFastSettle(settings.fastTTLSettleEnabled);
        device->setExternalFastSettleChannel(settings.fastSettleTTLChannel);
        device->setDacHighpassFilter(settings.desiredDAChpf);
        device->enableDacHighpassFilter(settings.desiredDAChpfState);
        device->enableLeds(settings.ledsEnabled);
        device->setClockDivider(settings.clockDivideFactor);

        updateSettingsDuringAcquisition = false;
    }

    if (!digitalOutputCommands.empty())
    {

        while (!digitalOutputCommands.empty())
        {
            DigitalOutputCommand command = digitalOutputCommands.front();
            TTL_OUTPUT_STATE[command.ttlLine] = command.state;
            digitalOutputCommands.pop();

        }

        device->setTtlOut(TTL_OUTPUT_STATE);

        LOGB("TTL OUTPUT STATE: ",
            TTL_OUTPUT_STATE[0],
            TTL_OUTPUT_STATE[1],
            TTL_OUTPUT_STATE[2],
            TTL_OUTPUT_STATE[3],
            TTL_OUTPUT_STATE[4],
            TTL_OUTPUT_STATE[5],
            TTL_OUTPUT_STATE[6],
            TTL_OUTPUT_STATE[7]);

    }

    return true;

}

int DeviceThread::getChannelFromHeadstage (int hs, int ch)
{
    int channelCount = 0;
    int hsCount = 0;
    if (hs < 0 || hs >= headstages.size() + 1)
        return -1;
    if (hs == headstages.size()) //let's consider this the ADC channels
    {
        int adcOutputs = getNumDataOutputs(ContinuousChannel::ADC);

        if (adcOutputs> 0)
        {
            return getNumDataOutputs(ContinuousChannel::ELECTRODE) + getNumDataOutputs(ContinuousChannel::AUX) + ch;
        }
        else
            return -1;
    }
    if (headstages[hs]->isConnected())
    {
        if (ch < 0)
            return -1;
        if (ch < headstages[hs]->getNumActiveChannels())
        {
            for (int i = 0; i < hs; i++)
            {
                channelCount += headstages[i]->getNumActiveChannels();
            }
            return channelCount + ch;
        }
        else if (ch < headstages[hs]->getNumActiveChannels() + 3)
        {
            for (int i = 0; i < headstages.size(); i++)
            {
                if (headstages[i]->isConnected())
                {
                    channelCount += headstages[i]->getNumActiveChannels();
                    if (i < hs)
                        hsCount++;
                }
            }
            return channelCount + hsCount * 3 + ch-headstages[hs]->getNumActiveChannels();
        }
        else
        {
            return -1;
        }

    }
    else
    {
        return -1;
    }
}

Array<const Headstage*> DeviceThread::getConnectedHeadstages()
{
    Array<const Headstage*> headstageArray;

    for (auto hs : headstages)
    {
        if (hs->isConnected())
            headstageArray.add(hs);
    }

    return headstageArray;
}

int DeviceThread::getHeadstageChannel (int& hs, int ch) const
{
    int channelCount = 0;
    int hsCount = 0;

    if (ch < 0)
        return -1;

    for (int i = 0; i < headstages.size(); i++)
    {
        if (headstages[i]->isConnected())
        {
            int chans = headstages[i]->getNumActiveChannels();

            if (ch >= channelCount && ch < channelCount + chans)
            {
                hs = i;
                return ch - channelCount;
            }
            channelCount += chans;
            hsCount++;
        }
    }
    if (ch < (channelCount + hsCount * 3)) //AUX
    {
        hsCount = (ch - channelCount) / 3;

        for (int i = 0; i < headstages.size(); i++)
        {
            if (headstages[i]->isConnected())
            {
                if (hsCount == 0)
                {
                    hs = i;
                    return ch - channelCount;
                }
                hsCount--;
                channelCount++;
            }
        }
    }
    return -1;
}

void DeviceThread::enableBoardLeds(bool enable)
{
    settings.ledsEnabled = enable;

    if (isAcquisitionActive())
        updateSettingsDuringAcquisition = true;
    else
        device->enableLeds(enable);
}

int DeviceThread::setClockDivider(int divide_ratio)
{
    if (!deviceFound)
        return 1;

    // Divide ratio should be 1 or an even number
    if (divide_ratio != 1 && divide_ratio % 2)
        divide_ratio--;

    // Format the divide ratio from its true value to the
    // format required by the firmware
    // Ratio    N
    // 1        0
    // >=2      Ratio/2
    if (divide_ratio == 1)
        settings.clockDivideFactor = 0;
    else
        settings.clockDivideFactor = static_cast<uint16>(divide_ratio/2);

    if (isAcquisitionActive())
        updateSettingsDuringAcquisition = true;
    else
        device->setClockDivider(settings.clockDivideFactor);

    return divide_ratio;
}

void DeviceThread::setAdcRange(int channel, short range)
{
    adcRangeSettings[channel] = range;
}

short DeviceThread::getAdcRange(int channel) const
{
    return adcRangeSettings[channel];
}

void DeviceThread::runImpedanceTest(double frequency)
{

    impedanceThread->stopThreadSafely();

    impedanceThread->setFrequency(frequency);

    impedanceThread->runThread();

}



