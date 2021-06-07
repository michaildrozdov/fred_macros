
#define PLATFORM_LINUX

int is_read_fpga = 1;

#ifdef PLATFORM_LINUX
#include "DataProcessing/DataProcessorZondas.h"
#include "DataProcessing/ProcessingAlgorithms.h"
#include "Buffers/Threadsafe_FIFO.h"
#include "SystemConfiguration.h"
#endif


#include "externs.h"
#include <iostream>



extern FloatSignal sig_read(std::string& path, int index);

extern int tracking_process_with_doppler_el(FloatSignal& timestamps,
		MatrixSimple& states, MatrixSimple& covariances, FloatSignal& thresholds, FloatSignal& ages, 
		int& numOfTracks,int& maxTracks, FloatSignal& rData, FloatSignal& aData, FloatSignal& eData, 
		FloatSignal& vData, FloatSignal& isUsedData, FloatSignal& isMatchedData, 
		FloatSignal& idsData, int& numOfData, float& currentTime,FloatSignal& idsTracks,int& lastTrackId);

//globals from maro
int wallPoint = 0;
int ie = 0;
FloatSignal cc1(1, 1);
ComplexSignal spectr_1(1, 1);
int nextIndex = 0;
int prevIndex = 0;
int MaximumNumber = 0;
FloatSignal maximums(1, 1024);
FloatSignal Sr3(1,1);
FloatSignal Sr15(1,1);
int TagN = 0;
int TagU = 0;
int targetsNum = 0;
float currentScore = 0;
int in = 0;
int numOfData = 0;
int lastIDused = 0;
//int isMatchedData = 0;
FloatSignal SM(1, 1);
int TagNs = 0;
int TagUs = 0;
float Step = 0;
float rangeNum = 0;
float R = 0;
float Vmag = 0;
FloatSignal L16(1, 1);
float m16 = 0;
float AzAngle = 0;
float ElAngle = 0;

float velocity = 0;
float azimuth = 0;
float elevation = 0;
float xtrack = 0;
float ytrack = 0;
float ztrack = 0;
float dTh = 0;
float dEl = 0;

float vx = 0;
float vy = 0;
float vz = 0;
FloatSignal histEntry(1, 1);
float validEntries = 0;
int histIndex = 0;
float val = 0;


#ifdef PLATFORM_LINUX
char path_raw_signal[] = "/home/radar/saniok/fred_project/fred/FRED_Core/src/zondas/0_radom_1nobreath_1udal_pribl.raw";
#endif
#ifdef PLATFORM_WIN
char path_raw_signal[] = "0_radom_1nobreath_1udal_pribl.raw";
#endif

const int sig_length = 8192;
const int sig_num = 1320;
FloatSignal signal_buffer(1, sig_length);

//this is used to replace signal from fpga with signals from file for testing
void prepaire_matrix_signals(MatrixSimple& mt)
{
	print_line("Storing signals into matrix");
	FILE* file_handle;
	file_handle = fopen(path_raw_signal, "rb");

	for (int ii = 0; ii < sig_num; ii = ii + 1)
	{
		fread(signal_buffer.float_data, 1,sig_length * sizeof(float), file_handle);
		mt.set_matrix_row(ii, signal_buffer);

	}

	fclose(file_handle);

	print_line("Signals stored into matrix");
}

uint64_t packet_id = 0;
void output_spectrum(FloatSignal& ch_1,void* data_processor)
{
	FloatSignal mag_spec = mag(fft(ch_1));
	const int size = mag_spec.point_num;

	std::vector<double> L0_output_block;
	L0_output_block.reserve(size);

	for(int i = 0;i<size;++i)
		L0_output_block.push_back(mag_spec.float_data[i]);

	/*std::cout<<"P1: "<<mag_spec.float_data[0]<<std::endl;
	  std::cout<<"P2: "<<mag_spec.float_data[1]<<std::endl;
	  std::cout<<"P3: "<<mag_spec.float_data[2]<<std::endl;
	  std::cout<<"P4: "<<mag_spec.float_data[3]<<std::endl;
	  std::cout<<"P5: "<<mag_spec.float_data[4]<<std::endl;
	  std::cout<<"P6: "<<mag_spec.float_data[5]<<std::endl;*/
	//std::cout<<"***************SIZE IS: "<<size<<std::endl;
	++packet_id;
	((DataProcessorZondas*)(data_processor))->m_debug_buffers->debug_buffer_L0.Emplace(packet_id, L0_output_block);
}

#ifdef PLATFORM_LINUX
//***********************************************DUMP 
FILE* dump_file;
//FILE* amplitude_file;
int dump_num = 1760;
int dump_idx = 0;
int is_dump_done = 0;
void start_dump()
{
	dump_file = fopen("0601signal.raw", "wb+");
	//amplitude_file = fopen("amplitude_file.txt", "w");
	dump_idx = 0;
}


void add_signal(float* data, int point_num)
{

	if (is_dump_done == 1) return;
	if (dump_file == NULL)
	{
		start_dump();
	}

	int size = point_num * sizeof(float);

	fwrite(data, 1, size, dump_file);
	++dump_idx;

	if (dump_idx == dump_num)
	{
		fclose(dump_file);
		//fclose(amplitude_file);
		//++dump_idx;
		std::cout << "***********************dump done" << std::endl;
		is_dump_done = 1;
	}

}

void stop_dump()
{
	/*if(dump_idx == dump_num)
	  {
	  fclose(dump_file);
	//++dump_idx;
	std::cout<<"dump done"<<std::endl;
	}*/
}
////////////////////////////////////////////////////
#endif


auto pid = uint64_t{ 0 };


void fill_point_array(void* object_pointer,float Range, float velocity, float azimuth, float elevation,FRED::RadarPacket* pkt)
{

	pid++;
	FRED::RadarPoint pt;
	pt.azimuth = azimuth/360.f * 2 * 3.1415926535f;
	pt.elevation = elevation/360.f * 2 * 3.1415926535f;
	pt.distance = Range;
	pt.velocity = velocity;
	pt.amplitude = 1;
	pkt->radar_points.push_back(pt);
	pkt->packet_id = pid;
	//	std::cout<<"****************************ARRAY POINT ADDED"<<std::endl;
}

void send_point_array(void* object_pointer,FRED::RadarPacket* pkt)
{
	((DataProcessorZondas*)object_pointer)->m_data_output_buffer->Push(*pkt);
}


void add_point(void* object_pointer,float Range, float velocity, float azimuth, float elevation)
{
#ifdef PLATFORM_LINUX

	FRED::RadarPacket pkt;	
	pid++;
	FRED::RadarPoint pt;
	pt.azimuth = azimuth/360.f * 2 * 3.1415926535f;
	pt.elevation = elevation/360.f * 2 * 3.1415926535f;
	pt.distance = Range;
	pt.velocity = velocity;
	pt.amplitude = 1;
	pkt.radar_points.push_back(pt);
	pkt.packet_id = pid;

	((DataProcessorZondas*)object_pointer)->m_data_output_buffer->Push(pkt);
	//	std::cout<<"****************************POINT ADDED"<<std::endl;

#endif

}

void send_points(void* object_pointer)
{
	//((DataProcessorZondas*)object_pointer)->m_data_output_buffer->Push(pkt);
}

FILE* file_handle;
const int signal_point_num = 1024 * 8;
const int channel_point_num = 1024;


int open_fifo()
{
	if (is_read_fpga == 0)
	{
		file_handle = fopen(path_raw_signal, "rb");
		if (file_handle == 0)
		{
			std::cout << "Failed to open fifo" << std::endl;
			return 1;
		}
		std::cout << "FIFO opened" << std::endl;
		return 0;
	}
	return 0;
}

#ifdef PLATFORM_LINUX
int read_fifo(FloatSignal& sig, DataProcessorZondas* obj,std::atomic_bool* is_processing)
#endif
#ifdef PLATFORM_WIN
int read_fifo(FloatSignal& sig,std::atomic_bool* is_processing)
#endif
{
	//std::cout<<"reading fifo"<<std::endl;
	if (is_read_fpga == 0)
	{
		const int bytes_num_to_read = signal_point_num * sizeof(float);
		fread(sig.float_data, 1, bytes_num_to_read, file_handle);
	}
	else
	{

#ifdef PLATFORM_LINUX
		while (is_processing->load(std::memory_order_relaxed))
		{
			auto packet = obj->m_source_fifo.Pop(std::chrono::milliseconds(50));
			if (!packet)
				continue;

			//std::cout<<"fifo read done"<<std::endl;
			//packing channels to original format
			for (int point = 0; point < channel_point_num; ++point)
			{
				for (int ch_idx = 0; ch_idx < 8; ++ch_idx)
				{
					sig.float_data[point * 8 + ch_idx] =
						(packet->raw_channel_data[ch_idx])[point];

				}
			}

			add_signal(sig.float_data, 1024 * 8);
			//std::cout<<"fifo read exit"<<std::endl;
			return 0;
		}
		return -1;
#endif
	}
}

void close_fifo()
{
	if (is_read_fpga == 0)
	{
		fclose(file_handle);
		std::cout << "FIFO closed" << std::endl;
	}
}


//****************TIME MEAS********************************
#include <unistd.h>

std::chrono::time_point<std::chrono::high_resolution_clock> t0 ;
std::chrono::time_point<std::chrono::high_resolution_clock> t1;

void test_delay()
{
	for(int i = 0;i < 10;++i)
	{
		std::chrono::time_point<std::chrono::high_resolution_clock> t0 = std::chrono::high_resolution_clock::now();
		usleep(200000);
		std::chrono::time_point<std::chrono::high_resolution_clock> t1 = std::chrono::high_resolution_clock::now();

		auto int_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
		//std::cout<<"**********test_delay TIME IS: "<<int_ms.count()<<std::endl;
	}
}

const int time_meas_num = 100;
int time_meas_cnt = 0;
int is_time_meas_started = 0;
FILE* time_meas_file;
int after_fifo_dt = 0;

void time_meas_before_read_fifo()
{
	//return;
	t1 = std::chrono::high_resolution_clock::now();

	auto int_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
	//std::cout<<"**********PROCESS TIME IS: "<<int_ms.count()<<std::endl;


	if(is_time_meas_started == 0)
	{
		time_meas_file = fopen("time_meas.raw", "wb+");
		is_time_meas_started = 1;
	}
	if((is_time_meas_started == 1) && (time_meas_cnt < 1000) )
	{
		char time_str[20];
		sprintf(time_str,"R:%d P:%d\n",after_fifo_dt,int_ms.count());
		fwrite(time_str, 1, strlen(time_str), time_meas_file);
		++time_meas_cnt;
		std::cout<<"Time conter is: "<<time_meas_cnt<<std::endl;
	}
	if(time_meas_cnt == 1000)
	{
		fclose(time_meas_file);
		std::cout<<"**********PROCESS TIME DONE: "<<std::endl;		
	}
}

void time_meas_after_read_fifo()
{
	//return;
	t0 = std::chrono::high_resolution_clock::now();

	auto int_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t0 - t1);
	//std::cout<<"**********FIFO READ TIME IS: "<<int_ms.count()<<std::endl;	

	after_fifo_dt = int_ms.count();
}



//****************TIME MEAS END****************************

//GLOBAL PARAMETRS FROM GUI
int gui_elevation = 1;
int gui_azimuth = 1;
int gui_throughwall = 1;
int gui_Range_param = 30;

#ifdef PLATFORM_LINUX
int MTI_T1_3_16_2_2_hist_tracking_with_el(DataProcessorZondas* object_pointer,std::atomic_bool* is_processing)
#endif
#ifdef PLATFORM_WIN
int MTI_T1_3_16_2_2_hist_tracking_with_el(void* object_pointer)
#endif
{
	std::cout << "Starting MTI_T1_3_16_2_2_hist_tracking_with_el " << std::endl;
	std::cout << "PARAMETER: throughwall = " << gui_throughwall << std::endl;
	std::cout << "PARAMETER: elevation = " << gui_elevation << std::endl;
	std::cout << "PARAMETER: azimuth = " << gui_azimuth << std::endl;
	std::cout << "PARAMETER: Range = " << gui_Range_param << std::endl;

	//test_delay();

	if (open_fifo())
		return 0;


	//Video_Dif_Ch_Av15_Angle _MTI-T1-3 										
	clear_variables();
	//path_t="C:\multitarget_data\Indoor\Fred-2021-03-05_16_2_2_2\TX_board_grey_16-2-2\test_389-T1-T2-T3-0_radom_udal_pribl.sig";
	//path_t="C:\multitarget_data\Indoor\Fred-2021-03-05_16_2_2_2\TX_board_grey_16-2-2\test_390-T1-T2-T3-0_radom_cross.sig";
	std::string path_t = "C:\\multitarget_data\\Indoor\\Fred-2021-03-10-16-2-2-2_green_stenka_2man\\test_402-T1-T2-T3-T4-0_radom_1nobreath_1udal_pribl.sig";
	//path_t = "C:\Users\matve\Desktop\Geozondas\04\25032021_green_wall\test_434-T1-T2-T3-T4-0_2man_2dist_breath.sig";
	std::string pathW = "C:\\multitarget_data\\FRED\\Kaizer.sig";
	//pathW="C:\Users\matve\Desktop\Geozondas\04\Kaizer.sig";

	//Parameteres from hardware                                   										
	int Nch = 8;                    // Number of Rx
	int N = 1024;                  // Nuber of input points in each channel											
	float c = 0.3f;                    // Light velocity, m/ns
	float freq0 = 3.1f;               // Start frequency, GHz
	float freqMax = 3.6f;           // Stop frecuency, GHz 
	float d = 0.061f;                 // MIMO array step, m  
	int RxAz = 5;                  //Number of Rx for Az 
	int RxEl = 4;                    //Number of Rx for El 

	//Parameters from designer
	int K = 1;                        // Number of Tx (columns in mt matrix)	
	float T = 10;                      // Chirp duration, ms
	int P = 1;                        // Chirp pause duration, ms 
	int Mn = 12;                    // Range padding window power
	float Alfa = 0.1f;                  // Avereging filter coefficient
	int perFrame = 16;            // Number of chirps in frame for velosity calculation
	//requiredAmpl = 2;	        // Minmum amplitude of spectrum extremums										
	//requiredRise = 0.1;       // of spectrum, mV per point											
	int scoreThreshold = 4;      // of channels spectrum maksimums positions  intersections
	int TagscoreThreshold = 7;  // of chirps avereged MTI specbitsrums intersections in frame	
	int dopplerPadding = 128;   // doppler padding window number
	int Azpadding = 128;	         // azimuth angle padding window number
	int Elpadding = 128;	         // elevation angle padding window number								
	int Nor = pow(2 , 15);	                 // Signal amplitude nor
	float NoiseLevel = 0.25f;        // FaLse maximums level
	float dMinLevel = 0.2f;          // FaLse adjacent minimum level
	float MinRise = 0.01f;             // FaLse rise level
	int breathingHistSamples = 32; // How many points there can be in the histogram for current distance
	int breathingHistRequired = 10; // How many points are needed to calculate the histogram
	int azHistMaxAngle = 52; // Maximum displacements from the boresight for the evalaution of angle histogram, degrees
	int azHistStep = 2; // Angle step in the angle histogram, degrees
	int elHistMaxAngle = 52; // Maximum displacements from the boresight for the evalaution of angle histogram, degrees
	int elHistStep = 2; // Angle step in the angle histogram, degrees
	int coordMemorySize = 7; // How many coordinate measurements should be averaged to get a final value 

	//GUI parametes
	float Range = gui_Range_param;                  // m
	int Az = gui_azimuth;                   // To measure Azimuth   (1,0)
	int El = gui_elevation;                  //To measure Elevation   (1,0), Az & El shall not be 0 simultaniously        

	int throughWall = gui_throughwall;
	int TxNum = 4;              // Number 0f Tx antennas (3, 4 optional);
	// Position of each TX in the array

	//macros section
	//TX_pos = signal(1, 4, false);
	//converted to
	FloatSignal TX_pos(1, 4);
	//end

	//macros section
	/*
	   TX_pos[0] = 0; // TX 1
	   TX_pos[1] = 1; // TX 2
	   TX_pos[2] = 3; // TX 3
	   Tx_pos[3] = 2; // TX 4
	   */
	//converted to
	TX_pos.float_data[0] = 1;
	TX_pos.float_data[1] = 0;
	TX_pos.float_data[2] = 3;
	TX_pos.float_data[3] = 2;
	//end

	int txesSelected = 0;
	for (int index = 0; index < 4; index = index + 1)
	{
		//if (TX_pos[0] >= 0)
		if (TX_pos.float_data[index] >= 0)
		{
			txesSelected = txesSelected + 1;
		}
	}

	if (txesSelected != TxNum)
	{
		print_line("Won't run. TxNum does not match the number of selected tx in the TX_pos array");
		//goto justend;
		return 0;
	}

	int TxAz = 2; // By the definition
	int TxEl = 1; // By the definition

	//if (TX_pos[0] < 0)
	if (TX_pos.float_data[0] < 0)
	{
		print_line("Won't run. 1st tx should always be present");
		//goto justend;
		return 0;
	}
	//if (TX_pos[1] < 0)
	if(TX_pos.float_data[1] < 0)
	{
		print_line("Won't run. 2nd tx should always be present");
		//goto justend;
		return 0;
	}

	//which tx in the switching order maps to which azimuth location (the 3rd tx is cosidered the last one in azimuth)
	//TxMapping = signal(1, TxNum, false);
	FloatSignal TxMapping(1, TxNum);
	TxMapping = -1; //We should clearly fail

	//macro section
	//TxMapping[Tx_pos[0]] = 1; //TX1 maps to the second azimuth location
	//TxMapping[Tx_pos[1]] = 0; //TX2 maps to the first azimuth location
	//converted to
	TxMapping.float_data[(int)(TX_pos.float_data[0])] = 1;
	TxMapping.float_data[(int)(TX_pos.float_data[1])] = 0;
	//end

	//if (TX_pos[2] >= 0)
	if(TX_pos.float_data[2] >= 0)
	{
		TxEl = 2;
		//TxMapping[Tx_pos[2]] = 3; //TX3 maps to the last azimuth location, or non-azimuth to be more exact
		TxMapping.float_data[(int)(TX_pos.float_data[2])] = 3;
	}

	//if (TX_pos[3] >= 0)
	if (TX_pos.float_data[3] >= 0)
	{
		TxAz = 3;
		//TxMapping[Tx_pos[3]] = 2; //TX$ maps to the third azimuth location, the last actual azimuth
		TxMapping.float_data[(int)(TX_pos.float_data[3])] = 2;
	}

	print_line("The switching order is:");
	for (int index = 0; index < TxNum; index = index + 1)
	{
		//if (TxMapping[index] != -1)
		if (TxMapping.float_data[index] != -1)
		{
			//print_line(index, " antenna to be switched on goes to ", TxMapping[index], " location in azimuth axis (3 is non-azimuth).");
			print_line(index, " antenna to be switched on goes to ", TxMapping.float_data[index], " location in azimuth axis (3 is non-azimuth).");
		}
	}

	out(TxAz);
	out(TxEl);

	//Device tuning parameters
	float startDelay = 0.35f;               // m
	int Ch = 4;                          // Velosity measuring channel number = 2. C0hanging Ch see rows 180 - 187

	//Tracking low level params
	int maxTracks = 20;
	int maxData = 40;

	int numOfTracks = 0;
	//timestamps = signal(1, maxTracks, false);
	FloatSignal timestamps(1,maxTracks);

	//state of tracks
	//x11 = signal(1, maxTracks, false);
	//x21 = signal(1, maxTracks, false);
	//x31 = signal(1, maxTracks, false);
	//x41 = signal(1, maxTracks, false);

	//macros line
	//states = matrix(maxTracks, 6); //including elevation
	//converted to
	MatrixSimple states(maxTracks, 6);
	//end
	states = 0;

	//covariances of tracks
	//covariances = matrix(maxTracks * 6, 6); //6 rows at a time is for a single 6x6 covar matrix
	MatrixSimple covariances(maxTracks * 6, 6);
	covariances = 0;

	//thresholds = signal(1, maxTracks, false);
	FloatSignal thresholds(1, maxTracks);

	//ages = signal(1, maxTracks, false);
	FloatSignal ages(1, maxTracks);
	//isValidatedArray = signal(1, maxTracks, false);
	FloatSignal isValidatedArray(1, maxTracks);

	FloatSignal idsTracks(1, maxTracks);
	idsTracks = (float)0;
	int lastTrackId = 0;

	//macros line
	//rData = signal(1, maxData, false);//array of distances
	//vData = signal(1, maxData, false);//array of velocities
	//aData = signal(1, maxData, false);//array of az angles
	//eData = signal(1, maxData, false);//array of el angles
	//converted to
	FloatSignal rData(1, maxData);
	FloatSignal vData(1, maxData);
	FloatSignal aData(1, maxData);
	FloatSignal eData(1, maxData);
	//end

	//macros line
	//isUsedData = signal(1, maxData, false);
	//isMatchedData = signal(1, maxData, false);
	//idsData = signal(1, maxData, false);
	//converted to
	FloatSignal isUsedData(1, maxData);
	FloatSignal isMatchedData(1, maxData);
	FloatSignal idsData(1, maxData);
	//end

	timestamps = (float)0;
	//end of tracking low level params

	//Tracking high level params
	float trueTrackThreshold = 0.8f;
	float maxVel = 2.f;//m/s
	float minVel = 0.1f;//m/s
	float minAgeToDisplayRadial = 2;//track with mostly radial movement needs to be updated for this amount of frames to be displayed
	float minAgeToDisplayTangental = 4;//track with mostly tangental movement needs to be updated for this amount of frames to be displayed
	//end of tracking igh level params

	//Tracks history for the averaging
	int maxNumOfTraces = maxTracks; // It may be different
	int curNumOfTraces = 0;
	MatrixSimple rHistory(maxNumOfTraces, coordMemorySize + 1); // +1 to avoid bund check
	MatrixSimple vHistory(maxNumOfTraces, coordMemorySize + 1);
	MatrixSimple azHistory(maxNumOfTraces, coordMemorySize + 1);
	MatrixSimple elHistory(maxNumOfTraces, coordMemorySize + 1);
	FloatSignal measurementsInMemory(1, maxNumOfTraces);
	FloatSignal memoryUpdatedFlags(1, maxNumOfTraces); // To remove old traces
	FloatSignal idsInMemory(1, maxNumOfTraces);

	rHistory = 0;
	vHistory = 0;
	azHistory = 0;
	elHistory = 0;
	measurementsInMemory = (float)0;
	idsInMemory = (float)0;
	//end of tracks history

	//Calculated parameters 											
	float Tc = T + P;                       // Chirp period, ms
	float Tf = Tc * perFrame / 1000.f;   // Frame period, s											
	int Nn = pow(2, Mn);           // Padding window number											
	float dT = T / N;                    // Time step											
	float Tn = dT * Nn;                // Padding window duration, ms
	float B = freqMax - freq0;      // Bandwidth											
	float slope = B / (T / 1000.f);       //0.5 GHz/10 ms	
	float deltaf = 1 / (T / 1000.f);      //Hz,   original IF delta										
	float distPerPoint = deltaf * c / 2 / slope * N / Nn; //m
	float pointsToView = ceil(Range / distPerPoint) + 2;
	float totalDistance = distPerPoint * (N / 2.f + 1);
	float lambda0 = c / freq0;
	float domega = 2 * pi() / dopplerPadding;
	float dvel = lambda0 * domega / 4 / pi() / Tc * 1000;
	float dsinAz = c / (freq0*Azpadding*d);
	float dsinEl = c / (freq0*Elpadding*d);

	//kVmsa=6/(perFrame*(perFrame-1)*(2*perFrame-1)); //for mean-scuare approximated velocity calculation
	float MaximumsDelay = ceil(startDelay / distPerPoint);        //Start delay number;

	//Variables      											
	// Don't used in this macros
	//perBreathFrame = 64;											
	//breathingPeriod = 128;//of slow samples
	//paddingRate = Nn/N; 
	//padAngleTo = 32;											
	//breathSkips = 8; //how many chirps are acquired before next brething sample
	// Don't used in this macros
	//breathingDeltaT = breathSkips*T/1000;//seconds per sample											
	//breathingWindow = breathingPeriod*breathingDeltaT;//seconds											
	//breathingDeltaF = 1/breathingWindow;//Hz																																																																																				
	//breathingDeltaT = breathSkips*T/1000;//seconds per sample											
	//breathingWindow = breathingPeriod*breathingDeltaT;//seconds											
	//breathingDeltaF = 1/breathingWindow;//Hz																						
	//dphase = 2*pi()/padAngleTo;											

	out(deltaf);
	out(distPerPoint);
	out(pointsToView);
	//out(paddingRate); //not defined in macro
	//out(breathingWindow); //not defined in macro
	out(dvel);

	//ch_1 = signal(T, N, false);   // Channel 1 signal											
	FloatSignal ch_1(T, N);

	//macro section
	//ch_1 = 0; ch_2 = ch_1; ch_3 = ch_1; ch_4 = ch_1; ch_5 = ch_1;
	//ch_6 = ch_1; ch_7 = ch_1; ch_8 = ch_1;
	//converted to
	ch_1 = (float)0;
	FloatSignal ch_2 = ch_1;
	FloatSignal ch_3 = ch_1;
	FloatSignal ch_4 = ch_1;
	FloatSignal ch_5 = ch_1;
	FloatSignal ch_6 = ch_1;
	FloatSignal ch_7 = ch_1;
	FloatSignal ch_8 = ch_1;
	//end

	FloatSignal ch_1Prev = ch_1;
	FloatSignal ch_2Prev = ch_1;
	FloatSignal ch_3Prev = ch_1;
	FloatSignal ch_4Prev = ch_1;
	FloatSignal ch_5Prev = ch_1;
	FloatSignal ch_6Prev = ch_1;
	FloatSignal ch_7Prev = ch_1;
	FloatSignal ch_8Prev = ch_1;
	//SigZP_1 = signal(Tn, Nn, false); //Zerrow padded signal of Cjhannel 1						                                                          //
	FloatSignal SigZP_1(Tn, Nn);

	//ph2 = signal(dopplerPadding, dopplerPadding, false); //Zerow padded phase for Doppler FFT 
	FloatSignal ph2(dopplerPadding, dopplerPadding);
	ph2 = (float)0;
	//phAz = signal(Azpadding, Azpadding, false); //Zerow padded phase for Azimuth angle FFT
	FloatSignal phAz(Azpadding, Azpadding);
	phAz = (float)0;
	//phEl = signal(Azpadding, Azpadding, false); //Zerow padded phase for Azimuth angle FFT
	FloatSignal phEl(Azpadding, Azpadding);
	phEl = (float)0;

	//s1 = signal(T, N*Nch, false);
	//	FloatSignal s1(T, N*Nch);
	int pos = 0;
	int posAz = 0;
	int posEl = 0;

	//averagedMat = matrix(Nch, pointsToView);
	MatrixSimple averagedMat(Nch, pointsToView);
	averagedMat = 0;
	//averaged = signal(Tn, pointsToView, false);
	FloatSignal averaged(Tn, pointsToView);
	averaged = (float)0;
	FloatSignal mean_sig = averaged;
	//histogramMatAz = matrix(breathingHistSamples, pointsToView);
	MatrixSimple histogramMatAz(breathingHistSamples, pointsToView);
	histogramMatAz = -1000; //Invalid values
	//histogramMatEl = matrix(breathingHistSamples, pointsToView);
	MatrixSimple histogramMatEl(breathingHistSamples, pointsToView);
	histogramMatEl = -1000; //Invalid values

	//mscros line
	//histEntryAz = signal(10, pointsToView, false);
	//histEntryEl = signal(10, pointsToView, false);
	//converted to
	FloatSignal histEntryAz(10, pointsToView);
	FloatSignal histEntryEl(10, pointsToView);
	//end

	//macros line
	//histAz = signal(2 * azHistMaxAngle, floor(2 * azHistMaxAngle / azHistStep + 1), false);
	//histEl = signal(2 * elHistMaxAngle, floor(2 * elHistMaxAngle / elHistStep + 1), false);
	//converted to
	FloatSignal histAz(2 * azHistMaxAngle, floor(2 * azHistMaxAngle / azHistStep + 1));
	FloatSignal histEl(2 * elHistMaxAngle, floor(2 * elHistMaxAngle / elHistStep + 1));
	//end

	//hitTag = signal(dT*(pointsToView), pointsToView, false);
	FloatSignal hitTag(dT*(pointsToView), pointsToView);
	hitTag = (float)0;
	FloatSignal hitM = hitTag;
	float PadK = dopplerPadding / perFrame; // PadK=8
	//I8 = signal(PadK, PadK, false);
	FloatSignal T8(PadK, PadK);

	// Here matrices only need to hold signals for the 1st tx										
	//chMat = matrix(Nch * TxNum, N); // matrix to hold normalized signals   											
	MatrixSimple chMat(Nch * 4, N);

	//prevChMat = matrix(Nch * TxNum, N);
	MatrixSimple prevChMat(Nch * 4, N);

	prevChMat = 0;
	//difMat = matrix(Nch * TxNum, N);
	MatrixSimple difMat(Nch * 4, N);

	//hitMaximums = signal(Tn, pointsToView, false);
	FloatSignal hitMaximums(Tn, pointsToView);

	FloatSignal scoreMaximums = hitMaximums;

	//macros section
	/*
	   phaseMat = matrix(Nch*TxNum, pointsToView); phaseMat = 0;
	   phaseMatT = matrix(pointsToView, Nch);
	   ph1Matrix = matrix(perFrame, pointsToView);
	//phMatrix=matrix(perFrame,pointsToView);                                         //
	ph1MatrixT = matrix(pointsToView, perFrame);
	//phMatrixT=matrix(pointsToView,perFrame);                                       //											
	ph1Tag1 = signal(perFrame, perFrame, false);
	//phTag1=signal(perFrame,perFrame,false);                                          //
	*/

	//converted to
	MatrixSimple phaseMat(Nch*4, pointsToView); phaseMat = 0;
	MatrixSimple phaseMatT(pointsToView, Nch);
	MatrixSimple phMatrix(perFrame * Nch, pointsToView);
	MatrixSimple phMatrixT(pointsToView, perFrame * Nch);
	FloatSignal ph1Tag1(perFrame, perFrame);
	FloatSignal phtagx(perFrame * Nch, perFrame);
	//end

	//Signal matrix loading from disk  											
	//int num = sig_get_count(path_t);
	int num = sig_num;
	int sigsPerFrame = perFrame + 2 * (TxNum - 1);
	int totalFrames = floor(num / sigsPerFrame);
	out(num);

	MatrixSimple ss1(sigsPerFrame, N*Nch*TxNum);
	//s1 = signal(num, N*Nch*TxNum, false);
	FloatSignal s1(num, N*Nch*TxNum);


	//prepaire_matrix_signals(ss1);



	//Window function for range FFT                      											
	//FloatSignal W = sig_read(pathW, 0); // Kaizer window 
	FloatSignal W(1, 1024);
	create_kaiser(W.float_data);

	//Window function for Doppler FFT	
	//WH = signal(dopplerPadding*Tc, dopplerPadding, false);   //Hamming Window for Doppler FFT
	FloatSignal WH(dopplerPadding*Tc, dopplerPadding);
	WH = (float)0;
	for(int i = 0; i < perFrame; i = i + 1)
	{
		//WH[i] = 0.54 - 0.46*cos(2 * pi()*i / perFrame);
		WH.float_data[i] = 0.54 - 0.46*cos(2 * pi()*i / perFrame);
	}
	//r13=WH;	

	//WHel = signal(Elpadding*TxEl, Elpadding, false);   //Hamming Window for elevation angle FFT
	FloatSignal WHel(Elpadding*TxEl, Elpadding);
	WHel = (float)0;
	for(int i = 0; i < RxEl; i = i + 1)
	{
		//WHel[i] = 0.54 - 0.46*cos(2 * pi()*i / Rxel);
		WHel.float_data[i] = 0.54 - 0.46*cos(2 * pi()*i / RxEl);
	} // Hamming Window for azimuth angle FFT
	//r13=WHel;	

	//WHaz = signal(Azpadding*TxAz, Azpadding, false);   //Hamming Window
	FloatSignal WHaz(Azpadding*TxAz, Azpadding);
	WHaz = (float)0;
	for(int i = 0; i < RxAz; i = i + 1)
	{
		//WHaz[i] = 0.54 - 0.46*cos(2 * pi()*i / RxAz);
		WHaz.float_data[i] = 0.54 - 0.46*cos(2 * pi()*i / RxAz);
	}
	//13=WHaz;																					
	Pause();

	float t1 = time();
	float currentTime = 0;//used for tracker		
	int frameIndex = 0;

	for (frameIndex = 0; frameIndex < totalFrames; frameIndex = frameIndex + 1)
	{

		time_meas_before_read_fifo();
		//The device is writing the new frame data
		for (int ii = 0; ii < sigsPerFrame; ii = ii + 1)
		{
			//s1 = sig_read(path_t, frameIndex * sigsPerFrame + ii);

#ifdef PLATFORM_LINUX
			//std::cout<<"before read"<<std::endl;
			if(read_fifo(s1, object_pointer,is_processing) == -1)
				return -1;

#endif
#ifdef PLATFORM_WIN
			read_fifo(s1);
#endif
			set_matrix_row(ss1, ii, s1);
		}
		time_meas_after_read_fifo();
		//New frame data is acquired
		ie = 0;

		//		std::cout<<"processing frames "<<frameIndex<<std::endl;
		for (int ii = 0; ii < perFrame; ii = ii + 1)
		{

			//std::cout << "Cycle num: " << ii << std::endl;

			/*if (ii == 22)
			  {
			  DWORD time = GetTickCount() - t0;
			  return;
			  }*/

			//s1 = get_matrix_row(ss1, ii) / Nor;
			s1 = ss1.get_matrix_row_real(ii) / Nor;
			for (int chNum = 0; chNum < Nch; chNum = chNum + 1)
			{
				for (int i = 0; i < N; i = i + 1)
				{
					//ch_1[i] = s1[i * Nch + chNum];
					ch_1.float_data[i] = s1.float_data[i * Nch + chNum];
				}
				set_matrix_row(chMat, chNum, ch_1);

				if(chNum == 4)
				{
					//ch_1 must be saved to GUI
					output_spectrum(ch_1,object_pointer);
				}
				//				float testMax = max_user(ch_1, &pos);
				//				print_line("channelMax pos=", pos, ", ampl=", testMax);
				//				std::cout << "channelMax pos=" << pos << ", ampl=" << testMax << '\n';
				//				if (!is_dump_done)
				//				{
				//					fprintf(amplitude_file, "channelMax pos=%d, ampl=%f\n", pos, testMax);
				//				}
			}
			//			if (!is_dump_done)
			//			{
			//				fprintf(amplitude_file, "+++++++++++\n");
			//			}

			if ((ii == 0) && (frameIndex == 0))
			{

				if (throughWall == 1)
				{
					mean_sig = (float)0;
					// Combining all signals
					for (int chNum = 0; chNum < Nch; chNum = chNum + 1)
					{
						//cc1 = get_matrix_row(chMat, chNum);
						cc1 = chMat.get_matrix_row_real(chNum);
						SigZP_1 = (float)0;
						insert(cc1 * W, 0, N - 1, SigZP_1, 0);

						spectr_1 = fft(SigZP_1) / T; // Range FFT
						//set_window(spectr_1, pointsToView * dT, pointsToView);
						spectr_1.window = pointsToView * dT;

						mean_sig = mean_sig + mag(spectr_1);
					}

					// The maximum will be 1
					//mean_sig = mean_sig / max(mean_sig, pos);
					mean_sig = mean_sig / max_user(mean_sig, &pos);

					for (int i = 1; i < pointsToView - 1; i = i + 1)
					{
						//if (mean_sig[i + 1] < mean_sig[i])
						if( (mean_sig.float_data[i + 1] < mean_sig.float_data[i]) && (i > MaximumsDelay))
						{
							// we found our maximum
							wallPoint = i;
							print_line("Delay in meters: ", wallPoint * distPerPoint);
							goto stopwallmax;
						}
					}
stopwallmax:

					for (int i = wallPoint; i < pointsToView - 1; i = i + 1)
					{
						//if (mean_sig[i + 1] > mean_sig[i])
						if (mean_sig.float_data[i + 1] > mean_sig.float_data[i])
						{
							MaximumsDelay = i;
							goto stopwallend;
						}
					}
stopwallend:
					Pause();
				}
			}

			hitMaximums = (float)0;
			hitTag = hitMaximums;
			difMat = chMat - prevChMat; //There are only valid changes for TX_pos[0]
			prevChMat = chMat;

			for (int chNum = 0; chNum < Nch; chNum = chNum + 1)
			{
				cc1 = difMat.get_matrix_row_real(chNum);

				SigZP_1 = (float)0;
				insert(cc1*W, 0, N - 1, SigZP_1, 0);

				spectr_1 = fft(SigZP_1) / T;  //Range FFT                      
				//set_window(spectr_1, pointsToView*dT, pointsToView);
				spectr_1.window = pointsToView * dT;

				if (ie == perFrame - 1) //Store signals for the calculation of phases only during the last measurements
				{
					//set_matrix_row(phaseMat, TxMapping[0] * Nch + chNum, phase(spectr_1)); // Store where the 1st tx (by the switching order) points
					set_matrix_row(phaseMat, TxMapping.float_data[0] * Nch + chNum, phase(spectr_1)); // Store where the 1st tx (by the switching order) points
				}

				//spectr=fft(sigZP)/T; 	                                              //										
				FloatSignal currentRangeSig = mag(spectr_1);
				averaged = averagedMat.get_matrix_row_real(chNum);
				//averaged = (1 - Alfa)*averaged + Alfa * currentRangeSig;  // not rewriting averagedMat
				averaged = averaged * (1 - Alfa) + currentRangeSig * Alfa;
				//r12 = averaged;
				set_matrix_row(averagedMat, chNum, averaged);
				//ie = ii - perFrame * floor(ii/perFrame);

				FloatSignal ph1 = phase(spectr_1);
				set_matrix_row(phMatrix, ie * Nch + chNum, ph1);
				if (chNum == Ch)
				{
					//r11=currentRangeSig; //MTI spectrum of ch1                      
					//r9 = averaged;           //LPF of r11	
					//r8 = currentRangeSig;
					//FloatSignal ph1 = phase(spectr_1);
					//ph=phase(spectr);                                              //
					//set_matrix_row(ph1Matrix, ie, ph1);
					//set_label(r9, (pointsToView / 2 + 1) / Tn, "kHz", "mV"); //set_label(r11,(Nn/2+1)/Tn,"kHz","mV");             											
				}
				FloatSignal targetSignal = averaged;
				int prevMin = 0;
				float mainMax = max_user(targetSignal, &pos);
				// Print_line(ie,"   ",chNum,"   ",pos,"   ",mainMax);
				//print_line("mainMax  pos=", pos, ", ampl=", mainMax); 
				float requiredLevel = mainMax / 4;
				requiredLevel = mainMax * NoiseLevel;
				float requiredAmpl = mainMax * dMinLevel;
				float requiredRise = mainMax * MinRise;
				for (int ia = 1; ia < pointsToView - 2; ia = ia + 1)
				{
					nextIndex = ia + 1;
					prevIndex = ia - 1;
					if (targetSignal.float_data[ia] > targetSignal.float_data[nextIndex])
					{
						if (targetSignal.float_data[ia] > targetSignal.float_data[prevIndex])
						{
							if (targetSignal.float_data[ia] - targetSignal.float_data[prevMin] > requiredAmpl)
							{
								if ((targetSignal.float_data[ia] - targetSignal.float_data[prevMin]) / (ia - prevMin) > requiredRise)
								{
									if (targetSignal.float_data[ia] > requiredLevel)
									{
										hitMaximums.float_data[ia] = hitMaximums.float_data[ia] + 1;
									}
								}
							}
						}
					}
					if (targetSignal.float_data[ia] < targetSignal.float_data[nextIndex])
					{
						if (targetSignal.float_data[ia] < targetSignal.float_data[prevIndex])
						{
							prevMin = ia;
						}
					}
				}
				//if (MaximumNumber > lastMaximumNumber) {lastMaximumNumber = MaximumNumber;} 											 											
				set_matrix_row(averagedMat, chNum, averaged);
			}
			//r6 = hitMaximums; // hit maximums number point number respons
			Sr3 = hitMaximums; Sr3 = (float)0;
			Sr15 = hitMaximums; Sr15 = (float)0;

			TagN = 0;
			TagU = 0;
			scoreMaximums = (float)0; //Point number of Maximums with score > scoreThreshold
			targetsNum = 0;
			for (int ia = MaximumsDelay; ia < pointsToView; ia = ia + 1)
			{
				if (hitMaximums.float_data[ia] < 2) /*{ goto early3; }*/continue;
				currentScore = (hitMaximums.float_data[ia - 2] + hitMaximums.float_data[ia + 2]) / 4 + (hitMaximums.float_data[ia - 1] + hitMaximums.float_data[ia + 1]) / 2 + hitMaximums.float_data[ia];
				if (currentScore >= scoreThreshold)
				{
					scoreMaximums.float_data[TagN] = ia;
					TagN = TagN + 1;
					//r5[ia]=currentScore;
				}
				//early3:
			}
			SM = scoreMaximums; //Points numbers of Maximums with score > scoreTreshold
			scoreMaximums = (float)0; //Point numbers of Targets detachment
			for (int ib = 0; ib < TagN; ib = ib + 1)
			{
				in = SM.float_data[ib];
				if (hitMaximums.float_data[in] > hitMaximums.float_data[in - 1])
				{
					if (hitMaximums.float_data[in] >= hitMaximums.float_data[in + 1])
					{
						//r4[in]=1; //Moving targets demonstration
						hitTag.float_data[in] = hitTag.float_data[in] + 1;
						scoreMaximums.float_data[TagU] = in;
						TagU = TagU + 1;
					}
				}
			}
			//out(TagU);             //Moving targets number
			//r2 = scoreMaximums; //Point numbers of Targets
			hitM = hitM + hitTag;
			//r4 = hitTag; //Moving targets demonstration
			//set_matrix_row(tagMatrix,ie,scoreMaximums);
			//Speed calculations
			if (ie == perFrame - 1)
			{

				numOfData = 0;//num of raw detections for current frame
				lastIDused = 0;//global variable to assigng unique ID for each detection, should be incrementd when detection is added to the list
				isUsedData = (float)0;//assigning false to flags of detections data for tracker to use
				isMatchedData = (float)0;//assigning false to flags of detections data for tracker to use

				if (TxNum > 1)
				{
					//std::cout << "Entering prcessing" << std::endl;
					// Now there are (TxNum - 1)*2 signals left before the next 16 cycle
					for (int Tx_i = 1; Tx_i < TxNum; Tx_i = Tx_i + 1) //tx switch
					{
						//std::cout << "prcessing cycle:"<<Tx_i << std::endl;
						// Tx_i+1 because we are working with 2nd and above tx
						//print_line("Getting the signal from array ", ii + 2 * Tx_i+1);
						s1 = ss1.get_matrix_row_real(ii + 2 * (Tx_i - 1) + 1) / Nor; //acquisition

						// Some processing
						for (int i = 0; i < N; i = i + 1)
						{

							ch_1Prev.float_data[i] = s1.float_data[i * Nch + 0];
							ch_2Prev.float_data[i] = s1.float_data[i * Nch + 1];
							ch_3Prev.float_data[i] = s1.float_data[i * Nch + 2];
							ch_4Prev.float_data[i] = s1.float_data[i * Nch + 3];
							ch_5Prev.float_data[i] = s1.float_data[i * Nch + 4];
							ch_6Prev.float_data[i] = s1.float_data[i * Nch + 5];
							ch_7Prev.float_data[i] = s1.float_data[i * Nch + 6];
							ch_8Prev.float_data[i] = s1.float_data[i * Nch + 7];

						}

						//s1 = get_matrix_row(ss1, ii + 2 * (Tx_i - 1) + 2) / Nor; //acquisition
						s1 = ss1.get_matrix_row_real(ii + 2 * (Tx_i - 1) + 2) / Nor; //acquisition
						for (int i = 0; i < N; i = i + 1)
						{
							ch_1.float_data[i] = s1.float_data[i * Nch + 0];
							ch_2.float_data[i] = s1.float_data[i * Nch + 1];
							ch_3.float_data[i] = s1.float_data[i * Nch + 2];
							ch_4.float_data[i] = s1.float_data[i * Nch + 3];
							ch_5.float_data[i] = s1.float_data[i * Nch + 4];
							ch_6.float_data[i] = s1.float_data[i * Nch + 5];
							ch_7.float_data[i] = s1.float_data[i * Nch + 6];
							ch_8.float_data[i] = s1.float_data[i * Nch + 7];
						}
						set_matrix_row(difMat, TxMapping.float_data[Tx_i] * Nch, ch_1 - ch_1Prev);
						set_matrix_row(difMat, TxMapping.float_data[Tx_i] * Nch + 1, ch_2 - ch_2Prev);
						set_matrix_row(difMat, TxMapping.float_data[Tx_i] * Nch + 2, ch_3 - ch_3Prev);
						set_matrix_row(difMat, TxMapping.float_data[Tx_i] * Nch + 3, ch_4 - ch_4Prev);
						set_matrix_row(difMat, TxMapping.float_data[Tx_i] * Nch + 4, ch_5 - ch_5Prev);
						set_matrix_row(difMat, TxMapping.float_data[Tx_i] * Nch + 5, ch_6 - ch_6Prev);
						set_matrix_row(difMat, TxMapping.float_data[Tx_i] * Nch + 6, ch_7 - ch_7Prev);
						set_matrix_row(difMat, TxMapping.float_data[Tx_i] * Nch + 7, ch_8 - ch_8Prev);


						for (int chNum = 0; chNum < Nch; chNum = chNum + 1)
						{
							//std::cout << "prcessing last cycle" << chNum << std::endl;
							//cc1 = get_matrix_row(difMat, TxMapping.float_data[Tx_i] * Nch + chNum);
							cc1 = difMat.get_matrix_row_real(TxMapping.float_data[Tx_i] * Nch + chNum);

							SigZP_1 = (float)0;
							insert(cc1*W, 0, N - 1, SigZP_1, 0);

							spectr_1 = fft(SigZP_1) / T;  //Range FFT                      
							//set_window(spectr_1, pointsToView*dT, pointsToView);
							spectr_1.window = (pointsToView*dT);

							//Should not corrupt TX_pos[0] rows
							set_matrix_row(phaseMat, TxMapping.float_data[Tx_i] * Nch + chNum, phase(spectr_1));
						}
					}
				}

				ie = -1;
				ii = ii + (TxNum - 1) * 2;
				TagNs = 0;
				TagUs = 0;
				for (int ia = MaximumsDelay; ia < pointsToView; ia = ia + 1)
				{
					if (hitM.float_data[ia] == 0) /*{ goto early4; }*/continue;
					//mich coment
					//currentScore=(hitTag[ia-2]+hitTag[ia+2])/4+(hitTag[ia-1]+hitTag[ia+1])/2+hitTag[ia];
					currentScore = (hitM.float_data[ia - 1] + hitM.float_data[ia + 1]) + hitM.float_data[ia] + hitM.float_data[ia - 2] + hitM.float_data[ia + 2];
					if (currentScore >= TagscoreThreshold)
					{
						scoreMaximums.float_data[TagNs] = ia;
						TagNs = TagNs + 1; //add to the count of maximums
						Sr3.float_data[ia] = currentScore;
					}
					//early4:
				}
				out(TagNs);
				//r14=scoreMaximums;                      
				SM = scoreMaximums; //Points numbers of Maximums with score >= scoreTreshold
				scoreMaximums = (float)0; //Point numbers of Targets detachment
				for (int ib = 0; ib < TagNs; ib = ib + 1)
				{
					in = SM.float_data[ib]; // Targets // Targets ranges numbers
					if (Sr3.float_data[in] > Sr3.float_data[in - 1])
					{
						if (Sr3.float_data[in] >= Sr3.float_data[in + 1])
						{
							Sr15.float_data[in] = 1;
							//hitM[in]=hitM[in]+1;
							scoreMaximums.float_data[TagUs] = in;
							TagUs = TagUs + 1;
						}
					}
				}
				//Out(TagUs); 
				//r10 = Sr15;  // Moving targets demonstration     
				//Pause();
				//r3 = Sr3;
				//r16=scoreMaximums; 
				//r7 = hitM;
				hitTag = (float)0;
				hitM = (float)0;
				phMatrixT = MatrixSimple::transpose(phMatrix);
				phaseMatT = MatrixSimple::transpose(phaseMat);

				histEntryAz = -1000; // Setting the single row to be entered into a histogram of angles with invalid values
				histEntryEl = -1000; // Setting the single row to be entered into a histogram of angles with invalid values

				Step = frameIndex + 1; //Number of speed calculation cycle
				//Print_line("Ch",CH+1,"   Step=",Step,"  ",ii );
				for (int it = 0; it < TagUs; it = it + 1)
				{
					//Ph2=0;
					rangeNum = scoreMaximums.float_data[it];
					R = rangeNum * distPerPoint;
					if (throughWall == 1)
					{
						R = R - wallPoint * distPerPoint;
					}
					else
					{
						R = R - startDelay;
					}
					//ph1Tag1 = get_matrix_row(ph1MatrixT, rangeNum);
					phtagx = phMatrixT.get_matrix_row_real(rangeNum);
					//set_window(ph1Tag1,perFrame,128);
					//set_label(ph1Tag1, 128, "v", "m/s)");
					Vmag = 0;
					for(int chNum = 0;chNum < Nch;chNum = chNum + 1)
					{
						for(int kk = 0;kk < perFrame;kk = kk + 1)
						{
							ph1Tag1.float_data[kk] = phtagx.float_data[kk * Nch + chNum];
						}					
						insert(ph1Tag1, 0, perFrame - 1, ph2, 0);
						//macros line
						//K11 = fft(WH*(cos(ph2) + j()*sin(ph2))); // Doppler FFT on frame of Phazors
						//converted to
						ComplexSignal K11(1, 1);
						ComplexSignal arg1(ph2.window, ph2.point_num);
						for (int i = 0; i < ph2.point_num; ++i)
						{
							arg1.complex_data[i].re = WH.float_data[i] * (cos(ph2.float_data[i]));
							arg1.complex_data[i].im = WH.float_data[i] * (sin(ph2.float_data[i]));
						}
						K11 = fft(arg1);
						//end
						FloatSignal L11 = mag(K11); //L11[0]=L11[1];

						float m11 = max_user(L11, &pos);
						//out(pos);
						if (pos > dopplerPadding / 2)
						{
							Vmag = Vmag + (pos - dopplerPadding)*dvel;  //Negative velocity (Approach)
							//11=push(L11,2*(dopplerPadding-pos));  
							//set_marker(11, 1, dopplerPadding - pos);
						}
						else
						{
							Vmag = Vmag + pos * dvel;   //Positive velocity (Away)
							//r11=L11;
							//set_marker(11, 1, pos);
						}
						//Print_line("TagNo=",it+1," R=",R," m"," Vmag=",Vmag," m/s");
					}
					Vmag = Vmag / Nch;
					FloatSignal phaseTag1 = phaseMatT.get_matrix_row_real(rangeNum);

					if (Az == 1)
					{

						//Rows should be in the correct order by now
						//Should store from 0th element always
						for (int txAzCurrent = 0; txAzCurrent < TxAz; txAzCurrent = txAzCurrent + 1)
						{
							insert(phaseTag1, Nch * txAzCurrent, Nch* txAzCurrent + RxAz - 1, phAz, txAzCurrent * RxAz);
						}
						//r2=phAz;
						//set_label(phAz, Azpadding, "v", "m/s)");
						//if (it == 0)
						//{
						//	  r12=phAz;
						//}
						//macros line
						//Az16 = fft(WHaz*(cos(phAz) + j * sin(phAz))); //Azimuth FFT on channels Phazors
						//converted to
						ComplexSignal arg11(phAz.window, phAz.point_num);
						for (int i = 0; i < phAz.point_num; ++i)
						{
							arg11.complex_data[i].re = WHaz.float_data[i] * (cos(phAz.float_data[i]));
							arg11.complex_data[i].im = WHaz.float_data[i] * (sin(phAz.float_data[i]));
						}
						ComplexSignal Az16 = fft(arg11);
						//end
						L16 = mag(Az16);
						m16 = max_user(L16, &posAz);
						//r12=L16;
						//out(posAz);
						if (posAz > Azpadding / 2)
						{
							AzAngle = asin((posAz - Azpadding)*dsinAz) * 180 / pi();  //Negative velocity (From the left)
							//r12=push(L16,2*(Azpadding-posAz));  
							//set_marker(12, 1, Azpadding - posAz);
						}
						else
						{
							AzAngle = asin(posAz*dsinAz) * 180 / pi();   //Positive velocity (From the right)
							//r12=L16; 
							//set_marker(12, 1, posAz);
						}
						//Print_line(" Az=",AzAngle," deg");

					}
					else
					{
						AzAngle = 0;
					}
					histEntryAz.float_data[(int)rangeNum] = AzAngle;
					// Splash the detection to neighboring ranges
					histEntryAz.float_data[(int)rangeNum - 1] = AzAngle;
					histEntryAz.float_data[(int)rangeNum + 1] = AzAngle;	

					if (El == 1)
					{
						//tx1 is used for the elevation, it is always stored at the second 8ch region
						phEl.float_data[0] = phaseTag1.float_data[Nch + 5];
						phEl.float_data[1] = phaseTag1.float_data[Nch + 2];
						phEl.float_data[2] = phaseTag1.float_data[Nch + 6];
						phEl.float_data[3] = phaseTag1.float_data[Nch + 7];

						if (TxEl > 1)
						{
							//If there are two el txes, this second is in the last (4th) 8ch region
							phEl.float_data[4] = phaseTag1.float_data[3 * Nch + 5];
							phEl.float_data[5] = phaseTag1.float_data[3 * Nch + 2];
							phEl.float_data[6] = phaseTag1.float_data[3 * Nch + 6];
							phEl.float_data[7] = phaseTag1.float_data[3 * Nch + 7];
						}

						//r1=phEl;
						//set_label(phEl, Elpadding, "v", "m/s)");
						phEl.window = Elpadding;

						//macros line
						//El16 = fft(WHel*(cos(phEl) + j * sin(phEl))); //Elevation FFT on channels Phazors
						//converted to
						ComplexSignal arg(phEl.window, phEl.point_num);
						for (int i = 0; i < phEl.point_num; ++i)
						{
							arg.complex_data[i].re = WHel.float_data[i] * cos(phEl.float_data[i]);
							arg.complex_data[i].im = WHel.float_data[i] * sin(phEl.float_data[i]);
						}
						ComplexSignal El16 = fft(arg);
						//end
						L16 = mag(El16);
						m16 = max_user(L16, &posEl);
						if (it == 0)
						{
							//r12 = PhEl;
						}
						//out(posEl);
						if (posEl > Elpadding / 2)
						{
							ElAngle = asin((posEl - Elpadding)*dsinEl) * 180 / pi();  //Negative velocity (From the bottom)
							//r12=push(L16,2*(Elpadding-posAz));  
							//set_marker(12,1,Elpadding-posAz); 
						}
						else
						{
							ElAngle = asin(posEl*dsinEl) * 180 / pi();   //Positive velocity (From the right)
							//r12=L16; 
							//set_marker(12,1,posEl);   
						}
						//r12=L16;
						//set_marker(12, 1, posEl);
						//Print_line(" El=",ElAngle," deg");

					}
					else
					{
						ElAngle = 0;
					}

					histEntryEl.float_data[(int)rangeNum] = ElAngle;

					// Splash the detection to neighboring ranges
					histEntryEl.float_data[(int)rangeNum - 1] = ElAngle;
					histEntryEl.float_data[(int)rangeNum + 1] = ElAngle;

					//					std::cout << Step << " " << it + 1 << " " << R << " " << Vmag << " " << AzAngle << " " << ElAngle << std::endl;

					//data for tracking
					rData.float_data[numOfData] = R;
					vData.float_data[numOfData] = Vmag;
					aData.float_data[numOfData] = AzAngle;
					eData.float_data[numOfData] = ElAngle;
					idsData.float_data[numOfData] = lastIDused;

					lastIDused = lastIDused + 1;
					if (numOfData + 1 < maxData)
					{
						numOfData = numOfData + 1;
					}
					//data for tracking is updated

				}

				//Kalman filter based tracking update
				//macro line
				//() = tracking_process_with_doppler_el(timestamps, states, covariances, thresholds, ages, numOfTracks, maxTracks, rData, aData, eData, vData, isUsedData, isMatchedData, idsData, numOfData, currentTime);
				//converted to
				int newLastTrackId = tracking_process_with_doppler_el(timestamps, states, covariances, thresholds, ages, numOfTracks, maxTracks, rData, aData, eData, vData, isUsedData, isMatchedData, idsData, numOfData, currentTime, idsTracks, lastTrackId);
				//end
				//print_line("Number of tracks after update ", numOfTracks);
				lastTrackId = newLastTrackId;
				memoryUpdatedFlags = (float)0;

				{
					FRED::RadarPacket pkt;
					for (int tr = 0; tr < numOfTracks; tr = tr + 1)
					{
						int traceNum = -1;
						bool traceFound = false;
						for (int trace = 0; trace < curNumOfTraces; trace = trace + 1)
						{
							//print_line("Checking the existence of a trace with ID ", idsTracks[tr], ". Current trace ID ", idsInMemory[trace]);
							if (idsTracks.float_data[tr] == idsInMemory.float_data[trace])
							{
								traceNum = trace;
								traceFound = true;
								memoryUpdatedFlags.float_data[trace] = 1;
								//print_line("Trace found");
							}
						}

						if (thresholds.float_data[tr] / ages.float_data[tr] > trueTrackThreshold)
						{
							//print_line("Track didn't pass by precision ", thresholds[tr]);
							//print_line("Track ID=", idsdata[tr], ", R=", x11[tr], ", v=", x21[tr], ", az=", x31[tr], ", inactive, error value=", thresholds[tr]);

							//goto endofloop;
							continue;
						}

						//measured qualities
						Range = get_matrix(states, tr, 0);
						velocity = get_matrix(states, tr, 1);
						azimuth = get_matrix(states, tr, 2);
						elevation = get_matrix(states, tr, 4);

						xtrack = Range * sin(azimuth * pi() / 180) * cos(elevation * pi() / 180); //along az
						ytrack = Range * cos(azimuth * pi() / 180) * sin(elevation * pi() / 180); //along el
						ztrack = Range * cos(azimuth * pi() / 180) * cos(elevation * pi() / 180); //from the radar

						//derived qualities
						dTh = get_matrix(states, tr, 3) * pi() / 180;
						dEl = get_matrix(states, tr, 5) * pi() / 180;

						vx = velocity * sin(azimuth* pi() / 180) + Range * dTh * cos(azimuth * pi() / 180);
						vy = velocity * sin(elevation* pi() / 180) + Range * dEl * cos(elevation * pi() / 180);
						vz = velocity * cos(azimuth* pi() / 180) * cos(elevation * pi() / 180) - Range * dTh * sin(azimuth * pi() / 180)*dEl * sin(elevation * pi() / 180);
						//old
						//vx = velocity * sin(azimuth* pi() / 180) + range*dTh * cos(azimuth * pi() / 180);
						//vy = velocity * cos(azimuth* pi() / 180) - range*dTh * sin(azimuth * pi() / 180);

						float vel = sqrt(vx*vx + vy * vy + vz * vz);


						if (vel < minVel)
						{
							//print_line("Track ID=", idsTracks[tr], ", R=", x11[tr], ", v=", x21[tr], ", az=", x31[tr], ", inactive, error value=", thresholds[tr]);

							//goto endofloop;
							continue;
						}

						if (vel > maxVel)
						{
							//print_line("Track ID=", idsTracks[tr], ", R=", x11[tr], ", v=", x21[tr], ", az=", x31[tr], ", inactive, error value=", thresholds[tr]);
							//goto endofloop;
							continue;
						}

						if (ages.float_data[tr] < minAgeToDisplayRadial)
						{
							//print_line("Track ID=", idsdata[tr], ", R=", x11[tr], ", v=", x21[tr], ", az=", x31[tr], ", inactive, error value=", thresholds[tr]);
							//goto endofloop;
							continue;
						}

						//tangental only movement can be easily faked by several stationary reflections, so should be registered after more updates
						if (abs(velocity) < 0.5*Range*tan(azimuth*pi() / 180)*tan(elevation*pi() / 180))//tangental movement is twice faster than radial
						{
							if (ages.float_data[tr] < minAgeToDisplayTangental)
							{
								//print_line("Track ID=", idsdata[tr], ", R=", x11[tr], ", v=", x21[tr], ", az=", x31[tr], ", inactive, error value=", thresholds[tr]);
								//goto endofloop;
								continue;
							}
						}
						//print_line("Have valid track at position ", x, ", ", y, " and velocity ", vx, ", ", vy);
						//print_line("Track ID=", idsData.float_data[tr], ", R=", Range, ", v=", velocity, ", az=", azimuth, ", el=", elevation, ", active, error value=", thresholds.float_data[tr]);
						//					std::cout<<Step<< "Track ID= " << idsTracks.float_data[tr] << " R= " << Range << " v= " << velocity << " az= " << azimuth << " el= " << elevation << " active, error value= " << thresholds.float_data[tr] << std::endl;
						//print_line("In Cartesian: ", xtrack, " ", ytrack, " ", ztrack);
						//					std::cout << "In Cartesian: " << xtrack << " " << ytrack << " " << ztrack << std::endl;
						float meanRValue = 0;
						float meanVValue = 0;
						float meanAzValue = 0;
						float meanElValue = 0;
						float measuredTimes = 0;

						if (traceFound)
						{
							//Memory update
							FloatSignal currentRHistory = rHistory.get_matrix_row_real(traceNum);
							FloatSignal currentVHistory = vHistory.get_matrix_row_real(traceNum);
							FloatSignal currentAzHistory = azHistory.get_matrix_row_real(traceNum);
							FloatSignal currentElHistory = elHistory.get_matrix_row_real(traceNum);

							for (int memNum = measurementsInMemory.float_data[traceNum]; memNum > 0; memNum = memNum - 1) // Does not go to 0
							{
								currentRHistory.float_data[memNum] = currentRHistory.float_data[memNum - 1];
								currentVHistory.float_data[memNum] = currentVHistory.float_data[memNum - 1];
								currentAzHistory.float_data[memNum] = currentAzHistory.float_data[memNum - 1];
								currentElHistory.float_data[memNum] = currentElHistory.float_data[memNum - 1];
							}
							currentRHistory.float_data[0] = Range;
							currentVHistory.float_data[0] = velocity;
							currentAzHistory.float_data[0] = azimuth;
							currentElHistory.float_data[0] = elevation;

							if (measurementsInMemory.float_data[traceNum] < coordMemorySize)
							{
								measurementsInMemory.float_data[traceNum] = measurementsInMemory.float_data[traceNum] + 1;
							}

							for (int memNum = 0; memNum < measurementsInMemory.float_data[traceNum]; memNum = memNum + 1)
							{
								//print_line("Adding ", currentRHistory[memNum], " to the mean.");
								meanRValue = meanRValue + currentRHistory.float_data[memNum];
								meanVValue = meanVValue + currentVHistory.float_data[memNum];
								meanAzValue = meanAzValue + currentAzHistory.float_data[memNum];
								meanElValue = meanElValue + currentElHistory.float_data[memNum];
							}
							measuredTimes = measurementsInMemory.float_data[traceNum];
							meanRValue = meanRValue / measuredTimes;
							meanVValue = meanVValue / measuredTimes;
							meanAzValue = meanAzValue / measuredTimes;
							meanElValue = meanElValue / measuredTimes;

							set_matrix_row(rHistory, traceNum, currentRHistory);
							set_matrix_row(vHistory, traceNum, currentVHistory);
							set_matrix_row(azHistory, traceNum, currentAzHistory);
							set_matrix_row(elHistory, traceNum, currentElHistory);

							goto aftertraces; // break
						}

aftertraces:
						if (traceFound) // Just display the result of the averaging
						{
							//print_line("Averaged values: R=", meanRValue, ", v=", meanVValue, ", az=", meanAzValue, ", el=", meanElValue, ". Calculated from ", measuredTimes, " measurements.");
							std::cout << "Averaged values: R=" << meanRValue << ", v=" << meanVValue << ", az=" << meanAzValue << ", el=" << meanElValue << ". Calculated from " << measuredTimes << " measurements." << std::endl;
							//add_point(object_pointer,meanRValue, meanVValue, meanAzValue, meanElValue);
							fill_point_array(object_pointer,meanRValue, meanVValue, meanAzValue, meanElValue,&pkt);
						}
						else
						{
							// This is a new trace
							if (curNumOfTraces < maxNumOfTraces)
							{
								FloatSignal currentRHistory = rHistory.get_matrix_row_real(curNumOfTraces);
								FloatSignal currentVHistory = vHistory.get_matrix_row_real(curNumOfTraces);
								FloatSignal currentAzHistory = azHistory.get_matrix_row_real(curNumOfTraces);
								FloatSignal currentElHistory = elHistory.get_matrix_row_real(curNumOfTraces);

								// Reset whatever was there
								currentRHistory = (float)0;
								currentVHistory = (float)0;
								currentAzHistory = (float)0;
								currentElHistory = (float)0;

								//Add the only valid measurement
								currentRHistory.float_data[0] = Range;
								currentVHistory.float_data[0] = velocity;
								currentAzHistory.float_data[0] = azimuth;
								currentElHistory.float_data[0] = elevation;

								set_matrix_row(rHistory, curNumOfTraces, currentRHistory);
								set_matrix_row(vHistory, curNumOfTraces, currentVHistory);
								set_matrix_row(azHistory, curNumOfTraces, currentAzHistory);
								set_matrix_row(elHistory, curNumOfTraces, currentElHistory);

								measurementsInMemory.float_data[curNumOfTraces] = 1;
								memoryUpdatedFlags.float_data[curNumOfTraces] = 1;
								idsInMemory.float_data[curNumOfTraces] = idsTracks.float_data[tr];
								curNumOfTraces = curNumOfTraces + 1;
								print_line("This is the first measurement for this track ID. Mean values match current values.");
							}
						}



						//endofloop:
						//print_line("end of loop");
					}

					send_point_array(object_pointer,&pkt);
				}
				// Need to remove traces of tracks which were already deleted
				int trace = 0;
				while (trace < curNumOfTraces)
				{
					if (memoryUpdatedFlags.float_data[trace] < 1)
					{
						print_line("Removing the trace with ID ", idsInMemory.float_data[trace]);
						set_matrix_row(rHistory, trace, get_matrix_row(rHistory, curNumOfTraces - 1));
						set_matrix_row(vHistory, trace, get_matrix_row(vHistory, curNumOfTraces - 1));
						set_matrix_row(azHistory, trace, get_matrix_row(azHistory, curNumOfTraces - 1));
						set_matrix_row(elHistory, trace, get_matrix_row(elHistory, curNumOfTraces - 1));

						measurementsInMemory.float_data[trace] = measurementsInMemory.float_data[curNumOfTraces - 1];
						curNumOfTraces = curNumOfTraces - 1;
					}
					else
					{
						trace = trace + 1;
					}
				}

				currentTime = currentTime + perFrame * (T + P) / 1000; //T+P should be approx. 11 ms with current implementation
				//print_line("Current tracker time is ", currentTime, " s");

				for (int histInd = 1; histInd < breathingHistSamples; histInd = histInd + 1)
				{
					set_matrix_row(histogramMatAz, histInd - 1, get_matrix_row(histogramMatAz, histInd));
					set_matrix_row(histogramMatEl, histInd - 1, get_matrix_row(histogramMatEl, histInd));
				}
				set_matrix_row(histogramMatAz, breathingHistSamples - 1, histEntryAz);
				set_matrix_row(histogramMatEl, breathingHistSamples - 1, histEntryEl);

				if (TagUs > 0) // There was a new entry
				{
					MatrixSimple histogramMatAzT = MatrixSimple::transpose(histogramMatAz);
					MatrixSimple histogramMatElT = MatrixSimple::transpose(histogramMatEl);
					for (int it = 0; it < TagUs; it = it + 1)
					{
						//print_line("");
						//print_line("Target number ", it);
						//print_line("azimuth hist:");
						rangeNum = scoreMaximums.float_data[it];
						R = rangeNum * distPerPoint;
						if (throughWall == 1)
						{
							R = R - wallPoint * distPerPoint;
						}
						else
						{
							R = R - startDelay;
						}
						histAz = (float)0;
						histEl = (float)0;
						// Azimuth from the histogram
						histEntry = histogramMatAzT.get_matrix_row_real(rangeNum);
						validEntries = 0;
						for (int ih = 0; ih < breathingHistSamples; ih = ih + 1)
						{
							if (histEntry.float_data[ih] < -999) // Invalid entry
							{
								/*goto nexthistaz;*/continue;
							}
							if (abs(histEntry.float_data[ih]) > azHistMaxAngle) // Outside of the estimation range
							{
								/*goto nexthistaz;*/continue;
							}

							histIndex = floor((histEntry.float_data[ih] + azHistMaxAngle) / azHistStep + 0.5);
							histAz.float_data[histIndex] = histAz.float_data[histIndex] + 1;
							//print_line("Adding to the az histogram at ", histIndex);
							validEntries = validEntries + 1;

							//nexthistaz:
						}
						posAz = -1000;
						posEl = -1000;
						if (validEntries > breathingHistRequired)
						{
							val = max_user(histAz, &posAz);
						}

						//print_line("elevation hist:");
						// Elevation from the histogram
						histEntry = histogramMatElT.get_matrix_row_real(rangeNum);
						validEntries = 0;
						for (int ih = 0; ih < breathingHistSamples; ih = ih + 1)
						{
							if (histEntry.float_data[ih] < -999) // Invalid entry
							{
								/*goto nexthistel;*/continue;
							}
							if (abs(histEntry.float_data[ih]) > elHistMaxAngle) // Outside of the estimation range
							{
								/*goto nexthistel;*/continue;
							}

							histIndex = floor((histEntry.float_data[ih] + elHistMaxAngle) / elHistStep + 0.5);
							histEl.float_data[histIndex] = histEl.float_data[histIndex] + 1;
							//print_line("Adding to the el histogram at ", histIndex);
							validEntries = validEntries + 1;

							//nexthistel:
						}

						if (validEntries > breathingHistRequired)
						{
							val = max_user(histEl, &posEl);
						}

						if ((posAz > -1000) and (posEl > -1000))
						{
							print_line("Immobile detected at az=", posAz*azHistStep - azHistMaxAngle, ", el=", posEl*elHistStep - elHistMaxAngle, ", R=", R);
						}
					}
				}

				//pause();     

			}

			//pause();
			ie = ie + 1;
			//early1:
		}
	}
	int t2 = time();
	int tM = t2 - t1;
	out(tM);
justend:

	int dummy = 0;

	close_fifo();
	//send_points(object_pointer);
}
