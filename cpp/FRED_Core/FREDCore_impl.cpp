#include "FREDCore_impl.h"
#include "HardwareRegisters.h"
#include "SystemConfiguration.h"
#include "PhysicsConstants.h"

#include "mkl.h"
#include <spdlog/spdlog.h>
#include <memory>
#include <exception>
#include <algorithm>

#include <iostream>

const int unknown_constant = SystemConfiguration::CHIRPS_PER_HADDAMARD_DECODE;
void FRED::FRED_Core::FREDCore_impl::zondas_config_pattern()
{
	//configuring Pattern Repeat
	int pattern_repeat_1 = 15;
	int pattern_repeat_2 = 1;
	int pattern_repeat_3 = 1;
	int pattern_repeat_4 = 1;
	int pattern_repeat_5 = 0;
	int pattern_repeat_6 = 0;
	int pattern_repeat_7 = 0;
	int pattern_repeat_8 = 0;
	
	uint16_t reg = 0;
	
	reg = (pattern_repeat_1 & 0x00FF)<<8;  // rep0
    reg |= (pattern_repeat_2 & 0x00FF);    // rep1
    LMS_WriteFPGAReg(this->m_fred_device, 0xE5, reg);

    reg = (pattern_repeat_3 & 0x00FF) << 8;// rep2
    reg |= (pattern_repeat_4 & 0x00FF);    // rep3
    LMS_WriteFPGAReg(this->m_fred_device, 0xE6, reg);

    reg = (pattern_repeat_5 & 0x00FF) << 8;// rep4
    reg |= (pattern_repeat_6 & 0x00FF);    // rep5
    LMS_WriteFPGAReg(this->m_fred_device, 0xE7, reg);

    reg = (pattern_repeat_7 & 0x00FF) << 8;// rep6
    reg |= (pattern_repeat_8 & 0x00FF);    // rep7	
	LMS_WriteFPGAReg(this->m_fred_device, 0xE8, reg);
	
	//configuring pattern length
	reg = 0;
	int pattern_length = 4;
	 reg = reg | (pattern_length << 1);
    LMS_WriteFPGAReg(this->m_fred_device, 0xFD, 0x06);

	//enabling pattern
	reg = 1;
    LMS_WriteFPGAReg(this->m_fred_device, 0xFF, reg);
	
	//configuring TX EN pattern
	
	int tx1_1 = 0;
	int tx1_2 = 1;
	int tx1_3 = 0;
	int tx1_4 = 0;
	int tx1_5 = 0;
	int tx1_6 = 0;
	int tx1_7 = 0;
	int tx1_8 = 0;
	
	int tx2_1 = 1;
	int tx2_2 = 0;
	int tx2_3 = 0;
	int tx2_4 = 0;
	int tx2_5 = 0;
	int tx2_6 = 0;
	int tx2_7 = 0;
	int tx2_8 = 0;
	
	int tx3_1 = 0;
	int tx3_2 = 0;
	int tx3_3 = 0;
	int tx3_4 = 1;
	int tx3_5 = 0;
	int tx3_6 = 0;
	int tx3_7 = 0;
	int tx3_8 = 0;
	
	int tx4_1 = 0;
	int tx4_2 = 0;
	int tx4_3 = 1;
	int tx4_4 = 0;
	int tx4_5 = 0;
	int tx4_6 = 0;
	int tx4_7 = 0;
	int tx4_8 = 0;
	
	reg = 0;
	reg = reg | ((tx1_1) << 0);
    reg = reg | ((tx1_2) << 1);
    reg = reg | ((tx1_3) << 2);
    reg = reg | ((tx1_4) << 3);

    reg = reg | ((tx2_1) << 4);
    reg = reg | ((tx2_2) << 5);
    reg = reg | ((tx2_3) << 6);
    reg = reg | ((tx2_4) << 7);

    reg = reg | ((tx3_1) << 8);
    reg = reg | ((tx3_2) << 9);
    reg = reg | ((tx3_3) << 10);
    reg = reg | ((tx3_4) << 11);

    reg = reg | ((tx4_1) << 12);
    reg = reg | ((tx4_2) << 13);
    reg = reg | ((tx4_3) << 14);
    reg = reg | ((tx4_4) << 15);

    LMS_WriteFPGAReg(this->m_fred_device, 0xFE, reg);
    reg = 0;
	// extended pattern
	reg = reg | ((tx1_5) << 0);
    reg = reg | ((tx1_6) << 1);
    reg = reg | ((tx1_7) << 2);
    reg = reg | ((tx1_8) << 3);

    reg = reg | ((tx2_5) << 4);
    reg = reg | ((tx2_6) << 5);
    reg = reg | ((tx2_7) << 6);
    reg = reg | ((tx2_8) << 7);

    reg = reg | ((tx3_5) << 8);
    reg = reg | ((tx3_6) << 9);
    reg = reg | ((tx3_7) << 10);
    reg = reg | ((tx3_8) << 11);

    reg = reg | ((tx4_5) << 12);
    reg = reg | ((tx4_6) << 13);
    reg = reg | ((tx4_7) << 14);
    reg = reg | ((tx4_8) << 15);

    LMS_WriteFPGAReg(this->m_fred_device, 0xE2, reg);
    reg = 0;
	
	std::cout<<"ZONDAS configuring pattern done"<<std::endl;
}
FRED::FRED_Core::FREDCore_impl::FREDCore_impl(int8_t device_id) :
m_is_processing(false)
{
    // Initialize to defaults
    this->m_fred_device = nullptr;
    this->m_chirp_profile = std::nullopt;
    
    // Perform device connection
    lms_info_str_t device_list[FRED::MAX_CONNECTED_DEVICE_ARRAY];
    int32_t device_count;
    
    // Get the device list
    if((device_count = LMS_GetDeviceList(device_list)) < 0)
        throw FRED::ConnectionException(FRED::ConnectionException::DEVICE_LIST_FAILURE);
    
    // Check if there are devices present
    if(device_count == 0)
        throw FRED::ConnectionException(FRED::ConnectionException::NO_DEVICES);

    // Check if device_id is within ranger
    if(device_id > device_count || device_id >= FRED::MAX_CONNECTED_DEVICE_ARRAY)
        throw FRED::ConnectionException(FRED::ConnectionException::BAD_DEVICE_ID);
    
    // Attempt to connect
    if(LMS_Open(&this->m_fred_device, device_list[device_id], NULL) != 0)
        throw FRED::ConnectionException(FRED::ConnectionException::CONNECTION_FAILURE, device_list[device_id]);
    
    // Create transmitter object
    this->m_transmitter = std::make_unique<Transmitter>(this->m_fred_device);
    
    // Create configuration object
    this->m_processing_configuration = new ProcessingConfiguration(&this->m_is_processing);
    
    // Force Configuration
    // TODO: This is temporary and will have to be replaced once actual control algorithms are designed


    LMS_WriteFPGAReg(this->m_fred_device, FRED::V2_RX_CONTROL_MAIN, 0x8C);
    LMS_WriteFPGAReg(this->m_fred_device, FRED::V2_RX_CONTROL_EN, 0x00);
	 zondas_config_pattern();
}

FRED::FRED_Core::FREDCore_impl::~FREDCore_impl()
{
    
    // TODO: Stop streaming if running
    
    // Delete processing configuration object
    delete this->m_processing_configuration;
    
    // Close the FRED device connection
    LMS_Close(this->m_fred_device);
}

void FRED::FRED_Core::FREDCore_impl::ManualTXConfiguration(FRED::ChirpProfile dds_profile, uint16_t synth_divider)
{
    // Clear chirp profile
    this->m_chirp_profile = std::nullopt;
    
    // Get set the chirp profile
    FRED::ChirpProfile chirp_profile = this->m_transmitter->ConfigureDDS(dds_profile);
    
    // Activate the DDS
    this->WriteParameter(FRED::FRED_Core::EN_DDS, static_cast<uint16_t>(true));
    
    // Set the Integer-N synthesizer
    this->m_transmitter->SetSynthDivider(synth_divider);
    
    // Calculate actual chirp profile
    chirp_profile.start_freq *= synth_divider;
    chirp_profile.stop_freq *= synth_divider;
    
    // Store actual chirp profile
    this->m_chirp_profile = std::optional<FRED::ChirpProfile>{chirp_profile};
    
    // Update sample count
    // TODO: Merge theese things
    this->m_processing_configuration->SetSamplePerChirpCount(static_cast<uint32_t>(std::round(SystemConfiguration::SAMPLE_RATE_HZ * chirp_profile.chirp_time)));
    this->m_processing_configuration->UpdateChirpProfile(this->m_chirp_profile.value());
}


void FRED::FRED_Core::FREDCore_impl::AutomaticTxConfiguration(FRED::ChirpProfile target_chirp)
{
    throw "Automatic TX Configuration is not implemented";
}

void FRED::FRED_Core::FREDCore_impl::SetActiveRXChannels(std::list<RX_ActiveChannels> channelList)
{
    // Clear the list
    this->m_rx_active_channels = {};
    
    // Check if any channels were provided
    if(channelList.empty())
        throw std::invalid_argument("Empty list of active channels provided");
    
    // Check if contains all channels
    if(std::find(channelList.begin(), channelList.end(), FRED::FRED_Core::RX_CH_ALL) != channelList.end())
    {
        // Fill the array
        for(uint8_t cnt = FRED::FRED_Core::RX_CH_0; cnt < FRED::FRED_Core::RX_CH_ALL; ++cnt)
            this->m_rx_active_channels.push_back(static_cast<FRED::FRED_Core::RX_ActiveChannels>(cnt));
        
        return;
    }
    
    // Remove duplicates
    channelList.sort();
    channelList.unique();
    
    // Initialize active channel list with sanitized channel list
    this->m_rx_active_channels = channelList;
}

void FRED::FRED_Core::FREDCore_impl::WriteParameter(FRED_Core::Parameter parameter, uint16_t value)
{
    // Calculate address and value variables
    uint32_t addr;
    uint16_t val;
    switch(parameter)
    {
        case FRED_Core::EN_DDS:
            addr = FRED::DDS_SWEEP_EN;
            val = value & 0x0001;
            break;
            
        case FRED_Core::SYNTH_LOCK:
            throw std::invalid_argument("SYNTH_LOCK is a read only parameter");
            
        case FRED_Core::EN_SAMPLE_ON_CHIRP:
            addr = FRED::SYNC_SAMPLE_ON_CHIRP;
            val = value & 0x0001;
            break;
            
        case FRED_Core::EN_PATTERN:
            addr = FRED::DAC_PATTERN_EN;

            if(LMS_ReadFPGAReg(this->m_fred_device, addr, &val))
                throw CommunicationException(CommunicationException::READ_CONTROL_ERROR);
            
            val = (val & 0xFFFE) | (value & 0x0001);
            
            break;
            
        case FRED_Core::PATTERN_DAC_VALUE:
            // Check limits
            if(value > SystemConfiguration::MAX_PATTERN_DAC_VALUE)
            {
                spdlog::warn("DAC value of {} is too large. Setting to {}", value, SystemConfiguration::MAX_PATTERN_DAC_VALUE);
                val = SystemConfiguration::MAX_PATTERN_DAC_VALUE;
            }
            else
                val = value;
            
            if(LMS_WriteFPGAReg(this->m_fred_device, FRED::DAC_PATTERN_VALUE_1, val) != 0)
                throw CommunicationException(CommunicationException::WRITE_CONTROL_ERROR);
            
            if(LMS_WriteFPGAReg(this->m_fred_device, FRED::DAC_PATTERN_VALUE_2, val) != 0)
                throw CommunicationException(CommunicationException::WRITE_CONTROL_ERROR);
            
            if(LMS_WriteFPGAReg(this->m_fred_device, FRED::DAC_PATTERN_VALUE_3, val) != 0)
                throw CommunicationException(CommunicationException::WRITE_CONTROL_ERROR);
            
            if(LMS_WriteFPGAReg(this->m_fred_device, FRED::DAC_PATTERN_VALUE_4, val) != 0)
                throw CommunicationException(CommunicationException::WRITE_CONTROL_ERROR);
            
            return;
            
        case FRED_Core::LPF_CUTTOFF:
            if(value > 3)
                throw std::invalid_argument("LPF Cutoff too high");

            addr = FRED::V2_RX_CONTROL_MAIN;
            
            if(LMS_ReadFPGAReg(this->m_fred_device, addr, &val))
                throw CommunicationException(CommunicationException::READ_CONTROL_ERROR);

            val = (val & 0xFFCF) | ((value & 0x1) << 5) | ((value & 0x2) << 3);
            break;
            
            
        case FRED_Core::LPF_GAIN:
            if(value > 3)
                throw std::invalid_argument("LPF Gain too high");

            addr = FRED::V2_RX_CONTROL_MAIN;
            
            if(LMS_ReadFPGAReg(this->m_fred_device, addr, &val))
                throw CommunicationException(CommunicationException::READ_CONTROL_ERROR);

            val = (val & 0xFFF3) | ((value & 0x1) << 3) | ((value & 0x2) << 1);
            
            break;
            
        default:
            throw std::invalid_argument("WriteParameter: Unknown parameter");
    }
    
    // Write parameter
    if(LMS_WriteFPGAReg(this->m_fred_device, addr, val) != 0)
        throw CommunicationException(CommunicationException::WRITE_CONTROL_ERROR);
}


uint16_t FRED::FRED_Core::FREDCore_impl::ReadParameter(FRED_Core::Parameter parameter)
{
    // TODO: This method does not need to call LMS_ReadFPGAReg upon every call. Optimize this
    switch(parameter)
    {
        case FRED_Core::SYNTH_LOCK:
            return static_cast<uint16_t>(this->m_transmitter->GetSynthLock());
            
        case FRED_Core::EN_DDS:
        {
            uint16_t val = 0;
            
            if(LMS_ReadFPGAReg(this->m_fred_device, FRED::DDS_SWEEP_EN, &val) != 0)
                throw CommunicationException(CommunicationException::READ_CONTROL_ERROR);
            
            return val & 0x1;
        }
        case FRED_Core::EN_SAMPLE_ON_CHIRP:
        {
            uint16_t val = 0;
            
            if(LMS_ReadFPGAReg(this->m_fred_device, FRED::SYNC_SAMPLE_ON_CHIRP, &val) != 0)
                throw CommunicationException(CommunicationException::READ_CONTROL_ERROR);
            
            return val & 0x1;
        }
        
        case FRED_Core::EN_PATTERN:
        {
            uint16_t val = 0;
            
            if(LMS_ReadFPGAReg(this->m_fred_device, FRED::DAC_PATTERN_EN, &val) != 0)
                throw CommunicationException(CommunicationException::READ_CONTROL_ERROR);
            
            return val & 0x1;
        }
        case FRED_Core::PATTERN_DAC_VALUE:
        {
            uint16_t val = 0;
            
            if(LMS_ReadFPGAReg(this->m_fred_device, FRED::DAC_PATTERN_VALUE_1, &val) != 0)
                throw CommunicationException(CommunicationException::READ_CONTROL_ERROR);
            
            return val;
        }
        case FRED_Core::LPF_CUTTOFF:
        {
            uint16_t val = 0;
            if(LMS_ReadFPGAReg(this->m_fred_device, FRED::V2_RX_CONTROL_MAIN, &val) != 0)
                throw CommunicationException(CommunicationException::READ_CONTROL_ERROR);
            
            return ((val >> 3) & 0x2) | ((val >> 5) & 0x1);
        }
        case FRED_Core::LPF_GAIN:
        {
            uint16_t val = 0;
            if(LMS_ReadFPGAReg(this->m_fred_device, FRED::V2_RX_CONTROL_MAIN, &val) != 0)
                throw CommunicationException(CommunicationException::READ_CONTROL_ERROR);
            
            return ((val >> 1) & 0x2) | ((val >> 3) & 0x1);
        }
        default:
            throw std::invalid_argument("ReadParameter: Unknown parameter");
    }
}

void FRED::FRED_Core::FREDCore_impl::WriteAlgorithmParameter(FRED_Core::AlgorithmParameter parameter, double value)
{
    switch(parameter)
    {
        case AlgorithmParameter::USE_L0_WINDOWING:
            this->m_processing_configuration->SetWindowingState(ProcessingStage::L0, static_cast<bool>(value));
            break;
            
        case AlgorithmParameter::USE_L1_WINDOWING:
            this->m_processing_configuration->SetWindowingState(ProcessingStage::L1, static_cast<bool>(value));
            break;
        
        case AlgorithmParameter::USE_L0_PADDING:
            this->m_processing_configuration->SetZeroPaddingState(ProcessingStage::L0, static_cast<bool>(value));
            break;
            
        case AlgorithmParameter::USE_L1_PADDING:
            this->m_processing_configuration->SetZeroPaddingState(ProcessingStage::L1, static_cast<bool>(value));
            break;
            
        case AlgorithmParameter::L0_PAD_EXPONENT:
            this->m_processing_configuration->SetZeroPaddingExponent(ProcessingStage::L0, static_cast<uint8_t>(value));
            break;
            
        case AlgorithmParameter::L1_PAD_EXPONENT:
            this->m_processing_configuration->SetZeroPaddingExponent(ProcessingStage::L1, static_cast<uint8_t>(value));
            break;
            
        case AlgorithmParameter::VELOCITY_CHIRPS:
            this->m_processing_configuration->SetVelocityChirps(static_cast<uint16_t>(value));
            break;
            
        case AlgorithmParameter::EFFECTIVE_FREQ:
            this->m_processing_configuration->SetEffectiveFrequency(value);
            break;
            
        case AlgorithmParameter::AMPLITUDE_THRESHOLD:
            this->m_processing_configuration->SetAmplitudeThreshold(value);
            break;
              
		// GZ params
        case AlgorithmParameter::GZ_CALC_AZIMUTH:
            this->m_processing_configuration->gz_calc_azimuth = static_cast<bool>(value);
            break;
            
        case AlgorithmParameter::GZ_CALC_ELEV:
            this->m_processing_configuration->gz_calc_elev = static_cast<bool>(value);
            break;
            
        case AlgorithmParameter::GZ_THROUGH_WALL:
            this->m_processing_configuration->gz_through_wall = static_cast<bool>(value);
            break;
            
        case AlgorithmParameter::GZ_RANGE_LIMIT:
            this->m_processing_configuration->gz_range_limit = value;
            break;
            
        case AlgorithmParameter::GZ_CUSTOM:
            this->m_processing_configuration->gz_custom = value;
            break;
                 
        case AlgorithmParameter::SCALED_RANGE:
            throw std::invalid_argument("SCALED_RANGE is a read only parameter");
            
        case AlgorithmParameter::SCALED_VELOCITY:
            throw std::invalid_argument("SCALED_VELOCITY is a read only parameter");
            
        default:
            spdlog::warn("Unknown algorithm parameter");
    }
}

double FRED::FRED_Core::FREDCore_impl::ReadAlgorithmParameter(FRED_Core::AlgorithmParameter parameter)
{
    switch(parameter)
    {
        case AlgorithmParameter::USE_L0_WINDOWING:
            return static_cast<double>(this->m_processing_configuration->GetWindowingState(ProcessingStage::L0));
            
        case AlgorithmParameter::USE_L1_WINDOWING:
            return static_cast<double>(this->m_processing_configuration->GetWindowingState(ProcessingStage::L1));
        
        case AlgorithmParameter::USE_L0_PADDING:
            return static_cast<double>(this->m_processing_configuration->GetZeroPaddingState(ProcessingStage::L0));
            
        case AlgorithmParameter::USE_L1_PADDING:
            return static_cast<double>(this->m_processing_configuration->GetZeroPaddingState(ProcessingStage::L1));    

        case AlgorithmParameter::L0_PAD_EXPONENT:
            return static_cast<double>(this->m_processing_configuration->GetZeroPaddingExponent(ProcessingStage::L0));
            
        case AlgorithmParameter::L1_PAD_EXPONENT:
            return static_cast<double>(this->m_processing_configuration->GetZeroPaddingExponent(ProcessingStage::L1));  
    
        case AlgorithmParameter::VELOCITY_CHIRPS:
            return static_cast<double>(this->m_processing_configuration->GetVelocityChirps());
            
        case AlgorithmParameter::EFFECTIVE_FREQ:
            return this->m_processing_configuration->GetEffectiveFrequency();
            
        case AlgorithmParameter::AMPLITUDE_THRESHOLD:
            return this->m_processing_configuration->GetAmplitudeThreshold();
            
        case AlgorithmParameter::SCALED_RANGE:
            return this->m_processing_configuration->GetScaledRangeResolution();
            
        case AlgorithmParameter::SCALED_VELOCITY:
            return this->m_processing_configuration->GetScaledVelocityResolution();
            
		 // GZ params
        case AlgorithmParameter::GZ_CALC_AZIMUTH:
            return static_cast<double>(this->m_processing_configuration->gz_calc_azimuth);
            
        case AlgorithmParameter::GZ_CALC_ELEV:
            return static_cast<double>(this->m_processing_configuration->gz_calc_elev);
            
        case AlgorithmParameter::GZ_THROUGH_WALL:
            return static_cast<double>(this->m_processing_configuration->gz_through_wall);
            
        case AlgorithmParameter::GZ_RANGE_LIMIT:
            return this->m_processing_configuration->gz_range_limit;
            
        case AlgorithmParameter::GZ_CUSTOM:
            return this->m_processing_configuration->gz_custom;
        default:
            spdlog::warn("Unknown algorithm parameter");
            return -1;
    }
}


void FRED::FRED_Core::FREDCore_impl::StartDataProcessing()
{
    // Check that all system is ready to start data processing
    {
        // DDS Profile check
        if(!this->m_chirp_profile.has_value())
            throw std::invalid_argument("DDS is not configured");
        
        // DDS Enabled check
        if(!static_cast<bool>(this->ReadParameter(FRED::FRED_Core::EN_DDS)))
            throw std::invalid_argument("DDS is not enabled");
        
        // Synthesizer check
        //if(!this->m_transmitter->GetSynthLock())
        //    throw "Synthesizer is not locked";
        
        // Check if we have enough RX channels
        if(this->m_rx_active_channels.size() < 1)
            throw std::invalid_argument("No RX channels enabled");
    }
    
    // Start the processing thread
     this->m_data_processor = new DataProcessorZondas(
        &this->m_is_processing,
        this->m_rx_active_channels.size(),
        &this->m_output_buffer,
        &this->m_chirp_profile.value(),
        this->m_processing_configuration
    #if EN_DEBUG_STREAMING_FEATURES
            ,&this->m_debug_buffers
    #endif
    );
    
    // Enable processing
    this->m_is_processing.store(true, std::memory_order_relaxed);
    
    // Start the data aquisition thread
    this->m_data_aquisition_thread = std::thread(&FREDCore_impl::M_Thread_DataAquisition, this);
}

void FRED::FRED_Core::FREDCore_impl::StopDataProcessing()
{
    // Stop the processing
    this->m_is_processing.store(false, std::memory_order_relaxed);
    
    // Join data aquisition thread thread
    if(this->m_data_aquisition_thread.joinable())
        this->m_data_aquisition_thread.join();
    
    // De-allocate and join data processor
    delete this->m_data_processor;
}

void FRED::FRED_Core::FREDCore_impl::M_Thread_DataAquisition()
{
    uint64_t pID = 0;
    const auto sample_count = this->m_processing_configuration->GetSamplePerChirpCount();




    
    // Create stream and meta-data object
    lms_stream_t streamer;
    lms_stream_meta_t stream_metadata;
    streamer.fifoSize = sample_count * SystemConfiguration::ANTENNA_COUNT * 512;
    streamer.throughputVsLatency = 0.8;
    
    // NOTE: This definition will be removed and Timestamp2 sync will be default once fully tested
#ifdef USE_TIMESTAMP2_SYNC
    // Define variables and buffers used in TimeStamp2 synchronization
    // Intermediary buffer to store data during syncronization
    int32_t* sync_buf = new int32_t[sample_count * SystemConfiguration::ANTENNA_COUNT * SystemConfiguration::SYNC_BUFFER_RESERVE_MULTIPLIER];
    
    // Number of samples to read from buffer
    // NOTE: Condition sync_read_size < sample_count has to be satisfied
    const uint32_t sync_read_size = (sample_count / SystemConfiguration::SYNC_CHUNK_DIVISOR) * SystemConfiguration::ANTENNA_COUNT;
    
    // Timestamp storage variables
    uint32_t buf_last_timestamp1 = 0;
    uint32_t buf_last_index = 0;
    uint32_t buf_timestamp2 = 0;
    
    int8_t sync_counter = SystemConfiguration::CHIRPS_PER_HADDAMARD_DECODE;
    
    // Sync_offet is stored in FPGA registers. Read it
    uint16_t sync_offset = 0;
    if(LMS_ReadFPGAReg(this->m_fred_device, FRED::SYNC_FIR_FILTER_DELAY, &sync_offset) != 0)
        throw CommunicationException(CommunicationException::READ_CONTROL_ERROR);
#endif
    
    // Allocate raw data buffer memory
    int32_t *raw_data_buffer = new int32_t[sample_count * SystemConfiguration::ANTENNA_COUNT];
    
    // Setup stream
    if(LMS_SetupStream(this->m_fred_device, &streamer) != 0)
        throw CommunicationException(CommunicationException::STREAM_INITIALIZATION_ERROR);
    
    // Start stream
    if(LMS_StartStream(&streamer) != 0)
        throw CommunicationException(CommunicationException::STREAM_INITIALIZATION_ERROR);

    // Create output channel list
    // TODO: Maybe optimize? Maybe store as a class member, since this data will not change during runtime
    std::list<uint8_t> temp_channel_list;
    for(auto iterator = this->m_rx_active_channels.begin(); iterator != this->m_rx_active_channels.end(); ++iterator)
        temp_channel_list.push_back(static_cast<uint8_t>(*iterator));

    // Data aquisition loop
    while(this->m_is_processing.load(std::memory_order_relaxed))
    {
#ifdef USE_TIMESTAMP2_SYNC
        // Perform sync process
        if(sync_counter == SystemConfiguration::CHIRPS_PER_HADDAMARD_DECODE)
        {
            // Recieve data from FRED device
            if(LMS_RecvStream(&streamer, &sync_buf[buf_last_index], sync_read_size, &stream_metadata, SystemConfiguration::STREAM_TIMEOUT_MS) < 0)
            {
                // On error stop data aquisition and thrown an error
                this->StopDataProcessing();
                throw CommunicationException(CommunicationException::STREAM_ERROR);
            }
            
            // Handle timestamp mismatch
            if(stream_metadata.timestamp1 != buf_last_timestamp1)
            {
                spdlog::warn("Packet loss detected. Expected Timestamp1: {} Got: {}", buf_last_timestamp1, stream_metadata.timestamp1);

                // Reset state. All data currently in buffer is ruined we will loose at max 4 chirps
                buf_last_timestamp1 = stream_metadata.timestamp1 + (sync_read_size / SystemConfiguration::ANTENNA_COUNT);
                buf_last_index = 0;
                buf_timestamp2 = 0;

                // Retry reading
                continue;
            }
            // Calculate TimeStamp1 of last sample in sync_buf
            //spdlog::info("TS1: {}, TS2: {}", stream_metadata.timestamp1, stream_metadata.timestamp2);
            buf_last_timestamp1 = stream_metadata.timestamp1 + (sync_read_size / SystemConfiguration::ANTENNA_COUNT);

            // Calculate last index of sync_buf
            buf_last_index += sync_read_size;
            
            // Check if timestamp2 has changed (4 chirps have been aquired)
            if(buf_timestamp2 != stream_metadata.timestamp2)
            {
                sync_counter = 0;
                
                // Check collected sample count
                if(stream_metadata.timestamp2 - buf_timestamp2 != unknown_constant * sample_count) // Mismatch
                {
                    spdlog::warn("Sample count mismatch. Expected {}, got: {}", unknown_constant * sample_count, stream_metadata.timestamp2 - buf_timestamp2);

                    // Move data remainder to the front of sync_buf
                    int64_t overlap_size = (static_cast<int64_t>(buf_last_timestamp1) - static_cast<int64_t>(stream_metadata.timestamp2)) * static_cast<int64_t>(SystemConfiguration::ANTENNA_COUNT);
                    
                    if(overlap_size > 0)
                    {
                        std::memmove(sync_buf, &sync_buf[buf_last_index - overlap_size], overlap_size * sizeof(int32_t));
                        buf_last_index = overlap_size;
                    }
                    else if(overlap_size < 0)
                        // NOTE: If this condition is triggered the program most likely will crash
                        spdlog::critical("Buffer overlap is negative\nLast known timestamp1: {}, last known timestamp2: {}", std::to_string(buf_last_timestamp1), std::to_string(buf_timestamp2));

                    // Data must be collected again
                    sync_counter = SystemConfiguration::CHIRPS_PER_HADDAMARD_DECODE;
                }
                
                buf_timestamp2 = stream_metadata.timestamp2;
            }
            
            // Check sync state
            continue; 
        }
        else
        {
            // Load sync'd chirp data to processing buffer
            // NOTE: Once Haddamard decoding is working, this could be optimized as sync_buf will contain 4 chirps (from 8 channels) to be used in haddamard decoding. This may reqruire re-writing Haddamard decoder in ProcessingAlgorithms
            if((sync_counter * sample_count + sync_offset) * SystemConfiguration::ANTENNA_COUNT < buf_last_index)
                std::memcpy(raw_data_buffer, &sync_buf[(sync_counter * sample_count + sync_offset) * SystemConfiguration::ANTENNA_COUNT], sample_count * SystemConfiguration::ANTENNA_COUNT * sizeof(int32_t));
            else
            {
                std::memcpy(raw_data_buffer, &sync_buf[(sync_counter * sample_count) * SystemConfiguration::ANTENNA_COUNT], sample_count * SystemConfiguration::ANTENNA_COUNT * sizeof(int32_t));
                spdlog::warn("Not enough data in buffer to offset by sync_offset. Using zero offset");
            }
            
            // Advance to next chirp
            sync_counter++;
            
            // Check if all chirps are processed
            if(sync_counter == SystemConfiguration::CHIRPS_PER_HADDAMARD_DECODE)
            {
                buf_timestamp2 = stream_metadata.timestamp2;
                
                // Move unused samples to front of buffer
                int64_t overlap_size = (static_cast<int64_t>(buf_last_timestamp1) - static_cast<int64_t>(buf_timestamp2)) * static_cast<int64_t>(SystemConfiguration::ANTENNA_COUNT);
                if(overlap_size > 0)
                    std::memmove(sync_buf, &sync_buf[buf_last_index - overlap_size], overlap_size * sizeof(int32_t));
                else if(overlap_size < 0)
                    // NOTE: When this condition is met - the program will crash
                    spdlog::critical("Buffer overlap is negative in write\nLast known timestamp1: {}, last known timestamp2: {}", std::to_string(buf_last_timestamp1), std::to_string(buf_timestamp2));

                // Adjust last known index
                buf_last_index = overlap_size;
            }
        }
#else
        // Recieve data from FRED device
        if(LMS_RecvStream(&streamer, raw_data_buffer, sample_count * SystemConfiguration::ANTENNA_COUNT, &stream_metadata, SystemConfiguration::STREAM_TIMEOUT_MS) < 0)
        {
            // On error stop data aquisition and thrown an error
            this->StopDataProcessing();
            throw CommunicationException(CommunicationException::STREAM_ERROR);
        }
#endif /* USE_TIMESTAMP2_SYNC */

        // Create output array with allocated memory
        // TODO: Maybe optimize this? Move out of the loop so it doesn't get re-initialized on every data receive
        std::vector<double*> output_data;
        output_data.reserve(this->m_rx_active_channels.size());
        for(auto iterator = this->m_rx_active_channels.begin(); iterator != this->m_rx_active_channels.end(); ++iterator)
        {
            double* alloc = (double*)mkl_malloc(sample_count * sizeof(double), 64);
            output_data.push_back(alloc);
        }
        
        // Decode received data
        for(uint32_t sample = 0; sample < sample_count; ++sample)
        {
            uint8_t ch_index = 0;
            for(auto ch_iter = this->m_rx_active_channels.begin(); ch_iter != this->m_rx_active_channels.end(); ++ch_iter, ++ch_index)
                output_data[ch_index][sample] = static_cast<double>(raw_data_buffer[sample * SystemConfiguration::ANTENNA_COUNT + static_cast<uint8_t>(*ch_iter)]);// / 32768.0;
        }

        // TODO: Haddamard decoding must be applied here if used
        
        // TODO: Add used channel sorting here (after haddamard)
        
        // Construct a new entry in buffer
        this->m_data_processor->EmplaceProcessingPacket(pID, output_data);
        
        // Adjust pID
        // NOTE: Maybe tie pID to stream metadata?
        ++pID;
    }

    // Stop the stream
    if(LMS_StopStream(&streamer) != 0)
        throw CommunicationException(CommunicationException::STREAM_DEINITIALIZATION_ERROR);
    
    // Destroy the stream
    if(LMS_DestroyStream(this->m_fred_device, &streamer) != 0)
        throw CommunicationException(CommunicationException::STREAM_DEINITIALIZATION_ERROR);
    
    // De-initialize memory
    delete[] raw_data_buffer;
}

#ifdef EN_DEBUG_STREAMING_FEATURES
    void FRED::FRED_Core::FREDCore_impl::SetDebugStreamActivity(FRED::FRED_Core::DebugStreams debug_stream_type, bool activity)
    {
        switch(debug_stream_type)
        {
            #ifdef EN_INITIAL_DEBUG_STREAM
            case FRED::FRED_Core::DebugStreams::LINITIAL_DATA_STREAM:
                this->m_debug_buffers.is_debug_LInitial.store(activity, std::memory_order_relaxed);
            #endif
            
            #ifdef EN_L0_DEBUG_STREAM
            case FRED::FRED_Core::DebugStreams::L0_DATA_STREAM:
                this->m_debug_buffers.is_debug_L0.store(activity, std::memory_order_relaxed);
                break;
            #endif

            #ifdef EN_L1_DEBUG_STREAM
            case FRED::FRED_Core::DebugStreams::L1_DATA_STREAM:
                this->m_debug_buffers.is_debug_L1.store(activity, std::memory_order_relaxed);
                break;
            #endif
                
            #ifdef EN_L2_DEBUG_STREAM
            case FRED::FRED_Core::DebugStreams::L2_DATA_STREAM:
                this->m_debug_buffers.is_debug_L2.store(activity, std::memory_order_relaxed);
                break;
            #endif
            default:
                spdlog::warn("Unknown stream type. Please check your compile options");
        }
    }
    
    void FRED::FRED_Core::FREDCore_impl::SetDebugStreamChannelIndex(FRED::FRED_Core::DebugStreams debug_stream_type, uint8_t channel_index)
    {
        // Check if RX_ALL was not passed
        if(channel_index >= this->m_rx_active_channels.size())
        {
            spdlog::warn("Unknown channel index passed");
            return;
        }
        
        switch(debug_stream_type)
        {
        #ifdef EN_L0_DEBUG_STREAM
            case FRED::FRED_Core::DebugStreams::L0_DATA_STREAM:
                this->m_debug_buffers.debug_channel_index_L0.store(channel_index, std::memory_order_relaxed);
                break;
        #endif
                
        #ifdef EN_L1_DEBUG_STREAM
            case FRED::FRED_Core::DebugStreams::L1_DATA_STREAM:
                this->m_debug_buffers.debug_channel_index_L1.store(channel_index, std::memory_order_relaxed);
                break;
        #endif
                
            default:
                spdlog::warn("Unknown stream type. Please check your compile options");
        }
    }
#endif
