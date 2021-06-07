#include "DataProcessing/DataProcessorZondas.h"
#include "DataProcessing/ProcessingAlgorithms.h"
#include "Buffers/Threadsafe_FIFO.h"
#include "SystemConfiguration.h"
#include <spdlog/spdlog.h>
#include <iostream>

#include <ipp.h>

// NOTE: The processing loop of this clas has WAY TOO MANY FUCKING DYNAMIC MEMORY ALLOCATIONS. Allocate all intermediary buffers and you will see a huge performance boost


extern int MTI_T1_3_16_2_2_hist_tracking_with_el(DataProcessorZondas*,std::atomic_bool* is_processing);


DataProcessorZondas::SourcePacket::SourcePacket()
{
    this->packet_id = 0;
}

DataProcessorZondas::SourcePacket::SourcePacket(uint64_t pid, std::vector<double *>& data)
{
    this->packet_id = pid;
    raw_channel_data = data;
}

DataProcessorZondas::DataProcessorZondas
(
	
	
    std::atomic_bool* is_processing,
    uint8_t channel_count,
    ThreadsafeFIFO<FRED::RadarPacket>* data_output_buffer,
    FRED::ChirpProfile* chirp_profile,
    ProcessingConfiguration* processing_configuration
#ifdef EN_DEBUG_STREAMING_FEATURES
    , DebugBuffers *debug_buffers
#endif
) : m_channel_count(channel_count)
{
this->m_processing_configuration = processing_configuration;
	// Assign buffers
    this->m_data_output_buffer = data_output_buffer;
    this->m_debug_buffers = debug_buffers;
	
	std::cout<<"DataProcessorZondas contructor called"<<std::endl;
    // Check channel count
   
    this->m_processor_main_thread = std::thread(&DataProcessorZondas::M_ProcessingLoop, this, is_processing);
}

DataProcessorZondas::~DataProcessorZondas()
{
    // Wait for thread to finish and join
    if(this->m_processor_main_thread.joinable())
        this->m_processor_main_thread.join();
    
    // Free buffers
    mkl_free(this->m_l0_kaiser_window_coefficients);
    mkl_free(this->m_l1_kaiser_window_coefficients);
    mkl_free(this->m_l2_kaiser_window_coefficients);
    
    // Clear the descriptors
    if(this->m_l0_fft_descriptor != nullptr)
        if(DftiFreeDescriptor(&this->m_l0_fft_descriptor) != DFTI_NO_ERROR)
            spdlog::error("L0 FFT Descriptor de-initialization error");
        
    if(this->m_l1_fft_descriptor != nullptr)
        if(DftiFreeDescriptor(&this->m_l1_fft_descriptor) != DFTI_NO_ERROR)
            spdlog::error("L1 FFT Descriptor de-initialization error");
        
    if(this->m_l2_fft_descriptor != nullptr)
        if(DftiFreeDescriptor(&this->m_l2_fft_descriptor) != DFTI_NO_ERROR)
            spdlog::error("L2 FFT Descriptor de-initialization error");
}




 #include <unistd.h>


extern int gui_elevation ;
extern int gui_azimuth;
extern int gui_throughwall;
extern int gui_Range_param ;


void test_points(DataProcessorZondas* obj)
{
	while(1)
	{
			{
			FRED::RadarPacket pkt;
			FRED::RadarPoint pt;
			
			pt.elevation = 0;
			
			pt.velocity = 1;
			pt.amplitude = 1;
			
			pkt.packet_id = 0;
			pt.azimuth = 0.17f;
			for(int i = 0;i<5;++i)
			{
				pt.azimuth = 0.35f;
				pt.distance = i;
				pkt.radar_points.push_back(pt);
			}
			

			obj->m_data_output_buffer->Push(pkt);
			
			
			usleep(2000000);
			}
			
			{
			FRED::RadarPacket pkt;
			FRED::RadarPoint pt;
			pt.elevation = 0;
			
			pt.velocity = 1;
			pt.amplitude = 1;
			
			pkt.packet_id = 0;
			pt.azimuth = 0.17f;
			for(int i = 0;i<5;++i)
			{
				pt.azimuth = -0.35f;
				pt.distance = i;
				pkt.radar_points.push_back(pt);
			}
			

			obj->m_data_output_buffer->Push(pkt);
			
			
			usleep(2000000);
			}
	}
}


void DataProcessorZondas::M_ProcessingLoop(std::atomic_bool* is_processing)
{
		std::cout<<"M_ProcessingLoop started"<<std::endl;

    

	std::cout<<"before MTI_T1_3_16_2_2_hist_tracking_with_el "<<std::endl;

	//test_points(this);
/*
while(1)
{
	for(int p = 0;p<10;++p)
		{
			FRED::RadarPacket pkt;
			FRED::RadarPoint pt;
			pt.azimuth = 0;
			pt.elevation = 0;
			pt.distance = p;
			pt.velocity = 1;
			pt.amplitude = 1;
			pkt.radar_points.push_back(pt);
			pkt.packet_id = p;
			pt.azimuth = 0.17f;
			pkt.radar_points.push_back(pt);
			(this)->m_data_output_buffer->Push(pkt);
			
			
			usleep(250000);
		}
	std::cout<<"20 points added"<<std::endl;
}*/
	

	/*int counter = 0;
	int pos = 0;
	while(is_processing->load(std::memory_order_relaxed))
	{
		auto packet = this->m_source_fifo.Pop(std::chrono::milliseconds(50));
		if (!packet)
			continue;

		if(counter % 1000 == 0)
		{
			FRED::RadarPacket pkt;
			FRED::RadarPoint pt;
			pt.azimuth = 0;
			pt.elevation = 0;
			pt.distance = pos;
			pt.velocity = 1;
			pt.amplitude = 1;
			pkt.radar_points.push_back(pt);
			pkt.packet_id = pos;
		
			(this)->m_data_output_buffer->Push(pkt);
			std::cout<<"***************************POINT ADDED at DISTANCE: "<<pos<<std::endl;
			++pos;
		}
		++counter;
		
	}
	return;*/


    gui_azimuth = m_processing_configuration->gz_calc_azimuth;
        gui_elevation = m_processing_configuration->gz_calc_elev;
    gui_throughwall = m_processing_configuration->gz_through_wall;
    gui_Range_param = 	m_processing_configuration->gz_range_limit;
	//while(is_processing->load(std::memory_order_relaxed)){}
		//std::cout<<"WORK"<<std::endl;

	while(is_processing->load(std::memory_order_relaxed))
	 {
		if(MTI_T1_3_16_2_2_hist_tracking_with_el(this,is_processing) == -1)
			return;
	}
	
	
    // Main processing loop
    /*while(is_processing->load(std::memory_order_relaxed))
    {
		
        // Get next data packet from FIFO
        auto packet = this->m_source_fifo.Pop(std::chrono::milliseconds(50));
        if(!packet)
            continue;

	}*/

}

void DataProcessorZondas::M_ProcessL0(const double* const packet_data, MKL_Complex16** output_data)
{
    
}

void DataProcessorZondas::M_ProcessL1(MKL_Complex16* chirp_matrix, MKL_Complex16*** output_data)
{
    
}

void DataProcessorZondas::M_ProcessL2(MKL_Complex16* angle_row, double* output_data)
{
    
}



/*

void DataProcessorZondas::add_point(float distance,float azimuth)
{
	static int id = 0;
	 FRED::RadarPacket pkt;
        pkt.packet_id = id;
            
		
			
		FRED::RadarPoint pt;
		pt.azimuth = azimuth;
		pt.elevation = 0.0;
		pt.distance = distance;
		pt.velocity = 2;
		pt.amplitude = 11;
		pkt.radar_points.push_back(pt);
		

		this->m_data_output_buffer->Push(pkt);
		
		++id;
}*/

/*
void DataProcessorZondas::get_signal_from_device(int i)
{
	
			
	int tx_num = 4;
	int tx_idx = 0;
	
		auto packet = this->m_source_fifo.Pop(std::chrono::milliseconds(50));
		while(!packet)
		packet = this->m_source_fifo.Pop(std::chrono::milliseconds(50));
	
		
		
		for(int p = 0;p<256;++p)
		{
			std::cout<<packet->raw_channel_data[0][p]<<std::endl;
		}
		
		return;
		
		
		for(int ch_idx = 0;ch_idx < 8;++ch_idx)
		{
			add_signal(packet->raw_channel_data[ch_idx],1024 * 8);
			
		}
		
		
		
		
	
}*/


void DataProcessorZondas::zondas_algo()
{
		}

