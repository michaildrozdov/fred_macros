#ifndef _DATA_PROCESSORZONDAS_H_
#define _DATA_PROCESSORZONDAS_H_

#include <complex>
#define MKL_Complex16 std::complex<double>
#include "mkl_types.h"

#include <libFRED.h>
#include <thread>
#include <atomic>

#include "mkl.h"
#include "Buffers/Threadsafe_FIFO.h"
#include "DataProcessing/ProcessingConfiguration.h"

#include "PhysicsConstants.h"

#ifdef EN_DEBUG_STREAMING_FEATURES
    #include "Buffers/DebugBufferStruct.h"
#endif


#include <iostream>
#include <string>


class DataProcessorZondas
{
public:
    DataProcessorZondas
    (
        std::atomic_bool* is_processing,
        uint8_t channel_count,
        ThreadsafeFIFO<FRED::RadarPacket>* data_output_buffer,
        FRED::ChirpProfile* chirp_profile,
        ProcessingConfiguration* processing_configuration
#ifdef EN_DEBUG_STREAMING_FEATURES
        , DebugBuffers *debug_buffers
#endif
    );
    ~DataProcessorZondas();

    struct SourcePacket
    {
        SourcePacket();
        SourcePacket(uint64_t pid, std::vector<double*>& data);
        uint64_t packet_id;
        std::vector<double*> raw_channel_data;
    };
    
    inline void PushProcessingPacket(SourcePacket& packet) {this->m_source_fifo.Push(packet);}
    inline void EmplaceProcessingPacket(uint64_t packet_id, std::vector<double*>& data) {this->m_source_fifo.Emplace(packet_id, data);}
public:
	void zondas_algo();
	void add_point(float distance,float azimuth);
	void get_signal_from_device(int i);
    ProcessingConfiguration* m_processing_configuration;
    // Processing thread
    void M_ProcessingLoop(std::atomic_bool* is_processing);
    
    // Processing steps
    void M_ProcessL0(const double* const packet_data, MKL_Complex16** output_data);
    void M_ProcessL1(MKL_Complex16* chirp_matrix, MKL_Complex16*** output_data);
    void M_ProcessL2(MKL_Complex16* angle_row, double* output_data);
    
    std::thread m_processor_main_thread;
    
    double* m_l0_kaiser_window_coefficients;
    double* m_l1_kaiser_window_coefficients;
    double* m_l2_kaiser_window_coefficients;
    
    // TODO: Theese three need better names
    uint32_t m_L0_initial_samples;
    uint32_t m_L0_padded_samples; // NOTE: Padded samples may not be the right name. m_L0_fft_samples maybe?
    uint32_t m_L0_efective_samples;

    uint32_t m_L1_chirps;
    uint32_t m_L2_angles;
    
    const uint8_t m_channel_count;
    
    DFTI_DESCRIPTOR_HANDLE m_l0_fft_descriptor;
    DFTI_DESCRIPTOR_HANDLE m_l1_fft_descriptor;
    DFTI_DESCRIPTOR_HANDLE m_l2_fft_descriptor;
    
    FRED::ChirpProfile* m_chirp_profile;
    
    ThreadsafeFIFO<SourcePacket> m_source_fifo;
    ThreadsafeFIFO<FRED::RadarPacket> *m_data_output_buffer;
    
    #ifdef EN_DEBUG_STREAMING_FEATURES
        DebugBuffers *m_debug_buffers;
    #endif
};

#endif /* _DATA_PROCESSORZONDAS_H_ */
