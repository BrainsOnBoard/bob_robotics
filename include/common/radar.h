#pragma once

// Standard C includes
#include <cassert>
#include <cmath>

// Serial port includes
#include <SerialPort.h>

// BoB robotics includes
#include "common/fsm.h"

//----------------------------------------------------------------------------
// BoBRobotics::RadarState
//----------------------------------------------------------------------------
namespace BoBRobotics
{
enum class RadarState
{
    Invalid,
    Waiting,
    ReadingHeader,
    ReadingData,
    SearchKeyword,
    GetHeader,
    SearchTLV,
    GetRangeProfile,
    GetNoiseProfile,
    GetObjects,
    MalformedPacket,
    GetHeaderOfObjects,
    Preprocessing,
    GetStats,
};

//----------------------------------------------------------------------------
// Radar
//----------------------------------------------------------------------------
class Radar : public FSM<RadarState>::StateHandler
{
    size_t m_GoodFrame = 0;
    size_t m_Counts;
    size_t counter;
    size_t header_char_count;
    size_t tlv_counter;
    uint32_t TLV_required;
    uint8_t header [40] = {};
    uint8_t TLV_header[8] = {};
    size_t aux;

    // List of objects
    size_t counter_obj_bytes;
    size_t counter_captured_objects;
    size_t obj_group;

    // header list of objects
    uint8_t ObjHeaderdummy_pair[2] = {};
    uint16_t ObjHeaderItemsprint;
    size_t count_obj_header_bytes;
    //
    float obj_range[100] = {}; // Range List of objects. Max value is given by MMW_MAX_OBJ_OUT 100. The total RAM Needed for the max case is MMW_MAX_OBJ_OUT*sizeof(MmwDemo_detectedObj)=100*12bytes
    float obj_doppler[100] = {};
    uint16_t obj_peak[100] = {};
    float obj_x[100] = {};
    float obj_y[100] = {};
    float obj_z[100] = {};

    // Temporary uint16_t indices
    uint16_t dopIdx_unsigned_int;
    int32_t dopIdx_int;

    //int16_t dopIdx_int_neg;

    uint16_t rangeIdx_unsigned_int;
    int32_t rangeIdx_int;

    int32_t x_coord_int;
    int32_t y_coord_int;
    int32_t z_coord_int;

    // Pairs to create the bytes
    uint8_t range_pair[2] = {};
    uint8_t doppler_pair[2] = {};
    uint8_t peak_pair[2] = {};
    uint8_t x_pair[2] = {};
    uint8_t y_pair[2] = {};
    uint8_t z_pair[2] = {};

    size_t malformation_flag;


    // Range Profile
    uint16_t range_profile[256];
    size_t counter_bytes_in_range_profile;
    size_t counter_values_in_range_profile;
    uint8_t range_profile_pair[2] = {};

    // Noise Profile
    uint16_t noise_profile[256];
    size_t counter_bytes_in_noise_profile;
    size_t counter_values_in_noise_profile;
    uint8_t noise_profile_pair[2] = {};

    // Stats Profile
    uint32_t stats_profile[6];
    size_t counter_bytes_in_stats_profile;
    size_t counter_values_in_stats_profile;
    uint8_t stats_profile_pair[2] = {};


    uint32_t packet_length_int;
    uint32_t frameNumber_int;
    uint32_t numDetObj_int;
    uint32_t numTLVs_int;
    uint32_t TLV_index;
    uint32_t TLV_length;

    // Constants needed. // Is it the right type
    // Double is 64 bits or 4 bytes, which is pretty long but we ddon't have time to do a proper conversion so we select the safest approach
    size_t RangeFFT_size = 512; // Multiplied by two. The real range FFT size is 256. 512 is including two bytes for each
    size_t numRangeBins = (int)RangeFFT_size/2; // # This is the real range FFT size
    double FsamplingRate = 5.5*std::pow(10,6); // MSps
    size_t numChirpsPerFrame = 64;
    double c = 3*std::pow(10,8);
    double idle_time = 7*std::pow(10,-6);
    double ramp_time = 58*std::pow(10,-6);
    double Ground_height  = 0.5; // #The height (m) of radar above the ground, In this case it is 50cm
    double Slope = 68*std::pow(10,12); // # 68 MHz/us
    size_t startFreq = 77*std::pow(10,9); // # 77GHz
    size_t numTxAnt = 2;
    size_t numRxAnt = 4;
    size_t leakage_thresh_dB = 128; //# Static definition of leakage
    double log2ToLog10 = 20*log10(2);
    double log2Qformat = 1/(256*numRxAnt*numTxAnt); // Linear scale for AWR1642 Chip. Use this for range and noise profile
    size_t numDopplerBins=numChirpsPerFrame/numTxAnt; // Reduction due to the modulation (TDM)

    // Range resolution
    float rangeIdxToMeters = c*FsamplingRate/(2*Slope*numRangeBins); // Range Resolution
    double dopplerResolutionMps = c/(2*startFreq*(idle_time+ramp_time)*numDopplerBins*numTxAnt); // Doppler Resolution

    double xyzQFormat = std::pow(2,9);

    //std::ofstream m_Stream;
public:
    Radar(const char *device) : m_Counts(0), m_FSM(this, RadarState::Invalid), m_SerialPort(device), m_HasData(false), m_BufferPosition(0)//, m_Stream("radar_bytes.csv")
    {
        // Open serial port
        m_SerialPort.Open(SerialPort::BAUD_921600);

        // Wait for data
        m_FSM.transition(RadarState::Waiting);
    }

    //------------------------------------------------------------------------
    // Statehandler virtuals
    //------------------------------------------------------------------------
    virtual bool handleEvent(RadarState state, Event event) override
    {
        uint8_t byte;
	if(event == Event::Update) {


	    // If buffer size is 4096 then go to append old slice
            //byte = m_SerialPort.ReadByte(5000);//readByte();
            byte = readByte();
            //m_Stream << (unsigned int)byte << ",";
            if (malformation_flag == 1){
                //std::cout << "Next bytes "<< (unsigned int)byte <<std::endl;
            }
	    //std::cout << (unsigned int)byte << ","; // Used to check if we are receiving data

	}

	if (state == RadarState::Waiting){
                if(event == Event::Update) {
			if((byte == 2) && (counter == 0)){ // First Character of the Magic word
				counter = 1; // First word detected
				//std::cout << "Byte 2 detected";
				header[header_char_count] = byte;
				header_char_count++;
				m_FSM.transition(RadarState::SearchKeyword);
					}
			else {
		                //std::cout << (unsigned int)byte;
				counter = 0;
				header_char_count = 0;
					}
             	}
	}
	else if (state == RadarState::SearchKeyword){ // MagicWord = [2,1,4,3,6,5,8,7]
                if(event == Event::Update) {
			//std::cout << "Searching for keyword"<<std::endl; // Add std::endl otherwise sometimes it doesnt print

			if((byte == 1) && (counter == 1)){
				counter++;
				header[header_char_count] = byte;
				header_char_count++;
			}
			else if((byte == 4) && (counter == 2)){
				counter++;
				header[header_char_count] = byte;
				header_char_count++;
				}
			else if((byte == 3) && (counter == 3)){
				counter++;
				header[header_char_count] = byte;
				header_char_count++;
				}
			else if((byte == 6) && (counter == 4)){
				counter++;
				header[header_char_count] = byte;
				header_char_count++;
				}
			else if((byte == 5) && (counter == 5)){
				counter++;
				header[header_char_count] = byte;
				header_char_count++;
				}
			else if((byte == 8) && (counter == 6)){
				counter++;
				header[header_char_count] = byte;
				header_char_count++;
				}
			else if((byte == 7) && (counter == 7)){
				counter = 0;
				header[header_char_count] = byte; // Number 7
				header_char_count++; // Reaches 8 here
#ifdef DEBUG_OUT
        std::cout << " " <<std::endl; // Magic Word
        std::cout << "Current State: SearchKeyword -> Magic Word Detected!" <<std::endl; // Magic Word
        std::cout << " " <<std::endl; // Magic Word
#endif
				// std::cout << "header[0]" << (unsigned int)header[0] << "" <<std::endl; // Magic Word
				// std::cout << "header[1]" << (unsigned int)header[1] << "" <<std::endl; // Magic Word
				// std::cout << "header[2]" << (unsigned int)header[2] << "" <<std::endl; // Magic Word
				// std::cout << "header[3]" << (unsigned int)header[3] << "" <<std::endl; // Magic Word
				// std::cout << "header[4]" << (unsigned int)header[4] << "" <<std::endl; // Magic Word
				// std::cout << "header[5]" << (unsigned int)header[5] << "" <<std::endl; // Magic Word
				// std::cout << "header[6]" << (unsigned int)header[6] << "" <<std::endl; // Magic Word
				// std::cout << "header[7]" << (unsigned int)header[7] << "" <<std::endl; // Magic Word

                                //m_Stream << std::endl;
				m_FSM.transition(RadarState::GetHeader);
				// 7 is the last char of the MagicWOrd. Jump to Get the header

				}
			else { // No luck
				counter = 0;
				m_FSM.transition(RadarState::Waiting);
			}
		}
	}
	else if (state == RadarState::GetHeader){
                if(event == Event::Update) {
			//std::cout << "Get Header";
			header[header_char_count] = byte; // FIrst one to be stored is number 8


			//std::cout << "header[" << header_char_count << "] " << (unsigned int)header[header_char_count] << "" <<std::endl;

      m_HasData = false;
			header_char_count++;

			if(header_char_count == 40){ // You got all the bytes from the header. The header goes from 0 to 39
#ifdef DEBUG_OUT
        std::cout << " " <<std::endl;
				std::cout << "Current State: GetHeader -> Header captured! " << std::endl;
        std::cout << " " <<std::endl;
#endif
				header_char_count = 0;
        malformation_flag = 0;


				packet_length_int = (header[15] << 24) | (header[14] << 16) | (header[13] << 8) | header[12];
				frameNumber_int = (header[23] << 24) | (header[22] << 16) | (header[21] << 8) | header[20];
				numDetObj_int = (header[31] << 24) | (header[30] << 16) | (header[29] << 8) | header[28];
				numTLVs_int = (header[35] << 24) | (header[34] << 16) | (header[33] << 8) | header[32];
#ifdef DEBUG_OUT
				std::cout << "packet_length_int " << (uint32_t)packet_length_int <<std::endl; // Packet length // Note: Packets longer than 3000 are an error. Check for those
				std::cout << "frameNumber_int " << (uint32_t)frameNumber_int <<std::endl; // frameNumber_int
				std::cout << "numDetObj_int " << (uint32_t)numDetObj_int <<std::endl; // numDetObj_int
				std::cout << "numTLVs_int " << (uint32_t)numTLVs_int <<std::endl; // numTLVs_int
        std::cout << " " <<std::endl;
#endif

        tlv_counter = 0;
				TLV_required = 1; // Search for this TLV. We can't jump to 2 and 3 in this byte based approach

        if ((packet_length_int < 3000)&&(numDetObj_int<=100)){
				      m_FSM.transition(RadarState::SearchTLV);
        }
        else { // Malformed Packet length
              m_FSM.transition(RadarState::MalformedPacket);
#ifdef DEBUG_OUT
              std::cout << " " <<std::endl;
					    std::cout << "Malformed Packet Length or numDetObj_int in GetHeader" << std::endl;
              std::cout << " " <<std::endl;
              #endif
        }

			}


		}
	}
	else if (state == RadarState::SearchTLV){ // Any TLV 2, 3 or 1 in that order
		if(event == Event::Update){
			// Get the length and the type
			TLV_header[tlv_counter] = byte;
			//std::cout << "TLV_header[" << tlv_counter << "] " << (unsigned int)TLV_header[tlv_counter] << "" <<std::endl;
			tlv_counter++;

			if (tlv_counter == 8){ // TLV header goes from 0 to 7
				TLV_index = (TLV_header[3] << 24) | (TLV_header[2] << 16) | (TLV_header[1] << 8) | TLV_header[0];
				TLV_length = (TLV_header[7] << 24) | (TLV_header[6] << 16) | (TLV_header[5] << 8) | TLV_header[4];
#ifdef DEBUG_OUT
        std::cout << " " <<std::endl;
        std::cout << "Current State: SearchTLV" <<std::endl;
				std::cout << "TLV Required " << (uint32_t)TLV_required << std::endl;
				std::cout << "TLV Obtained " << (uint32_t)TLV_index << std::endl;
				std::cout << "TLV length " << (uint32_t)TLV_length << std::endl;
        std::cout << " " <<std::endl;
#endif
				tlv_counter = 0;

				if ((TLV_required == 2) && (TLV_required == TLV_index) && (TLV_length == 512)){ // If we were looking for TLV 2
					m_FSM.transition(RadarState::GetRangeProfile);
#ifdef DEBUG_OUT
          std::cout << " " <<std::endl;
					std::cout << "Going to state GetRangeProfile" << std::endl;
          std::cout << " " <<std::endl;
#endif
          counter_bytes_in_range_profile = 0;
          counter_values_in_range_profile = 0;
           m_HasData = true; // Put this flag
          m_GoodFrame++;


						}
				else if ((TLV_required == 3) && (TLV_required == TLV_index) && TLV_length == 512) {
					m_FSM.transition(RadarState::GetNoiseProfile);
#ifdef DEBUG_OUT
          std::cout << " " <<std::endl;
					std::cout << "Going to GetNoiseProfile" << std::endl;
          std::cout << " " <<std::endl;

  #endif
          counter_bytes_in_noise_profile = 0;
          counter_values_in_noise_profile = 0;
          //m_HasData = true; // Put this flag
          //m_GoodFrame++;
						}
        else if ((TLV_required == 6) && (TLV_required == TLV_index)) {

          m_FSM.transition(RadarState::GetStats);
          #ifdef DEBUG_OUT
          std::cout << " " <<std::endl;
          std::cout << "Going to GetStats" << std::endl;
          std::cout << " " <<std::endl;
          #endif
          counter_bytes_in_stats_profile = 0;
          counter_values_in_stats_profile = 0;
            }

				else if ((TLV_required == 1) && (TLV_required == TLV_index) && (TLV_length == (4+12*numDetObj_int))) {

          m_FSM.transition(RadarState::GetHeaderOfObjects);
          #ifdef DEBUG_OUT
          std::cout << " " << std::endl;
          std::cout << "Going to GetHeaderOfObjects" << std::endl;
          std::cout << " " <<std::endl;
#endif
          count_obj_header_bytes = 0;

						}
				else { // Malformed packet
					m_FSM.transition(RadarState::MalformedPacket);
#ifdef DEBUG_OUT
          std::cout << " " <<std::endl;
					std::cout << "Malformed Packet in Search TLV" << std::endl;
          std::cout << "TLV" << TLV_required << std::endl;
          std::cout << " " <<std::endl;
          std::cout << "Current State: SearchTLV" <<std::endl;
  				std::cout << "TLV Required " << (uint32_t)TLV_required << std::endl;
  				std::cout << "TLV Obtained " << (uint32_t)TLV_index << std::endl;
  				std::cout << "TLV length " << (uint32_t)TLV_length << std::endl;
          malformation_flag = 1;
  #endif

						}
			}
		}
	}
  else if (state == RadarState::GetHeaderOfObjects){ // 4 bytes only. This gets executed after TLV 1, which contains the object.
		if(event == Event::Update){ // oNE PROBlem can be in get objects and the counters

      switch (count_obj_header_bytes){
           case 0:
                ObjHeaderdummy_pair[0] = byte;
                break;
           case 1:
                ObjHeaderdummy_pair[1] = byte;
                ObjHeaderItemsprint = (ObjHeaderdummy_pair[1] << 8) | ObjHeaderdummy_pair[0];
#ifdef DEBUG_OUT
                std::cout << " " <<std::endl;
                std::cout << "Current State -> GetHeaderOfObjects" <<std::endl;
                std::cout << "Number Of Objects in this TLV: " << ObjHeaderItemsprint << std::endl;
#endif
                break;
           case 2:
                ObjHeaderdummy_pair[0] = byte;
                break;
           case 3:
                ObjHeaderdummy_pair[1] = byte;
                ObjHeaderItemsprint = (ObjHeaderdummy_pair[1] << 8) | ObjHeaderdummy_pair[0];
#ifdef DEBUG_OUT
                std::cout << "xyzQFormat Obtained: " << ObjHeaderItemsprint << std::endl;
                std::cout << " " <<std::endl;

                std::cout << " " <<std::endl;

                std::cout << "xyzQFormat Obtained: " << ObjHeaderItemsprint << std::endl;
                std::cout << " " <<std::endl;
#endif
                break;
      }

      count_obj_header_bytes++; // Count how many of those we have

			if (count_obj_header_bytes == 4){ // The object header goes from 0 to 3
              count_obj_header_bytes = 0; // Restart

              m_FSM.transition(RadarState::GetObjects);
#ifdef DEBUG_OUT
              std::cout << " " << std::endl;
              std::cout << "Going to GetObjects" << std::endl;
              std::cout << " " << std::endl;
#endif
              counter_obj_bytes = 0;
              counter_captured_objects = 0;
              obj_group = 0;

                }

          }

  }
	else if (state == RadarState::GetObjects){
		if(event == Event::Update){
      //std::cout << " " <<std::endl;
      //std::cout << "GETOBJECTS STATE: bytes "<< (unsigned int)byte <<std::endl;

			// Assignations should go here before the counters increase
      switch (obj_group){
				case 0:
					range_pair[0] = byte;
					break;
				case 1: // Get Range
					range_pair[1] = byte;
          rangeIdx_unsigned_int = (range_pair[1] << 8) | range_pair[0];
          rangeIdx_int = rangeIdx_unsigned_int;

          //std::cout << " " << std::endl;
          //std::cout << "rangeIdx_int " << rangeIdx_int << std::endl;
          //std::cout << " " << std::endl;
          //if (rangeIdx_int > 32767){
          //        rangeIdx_int = rangeIdx_int - 65536;
          //                        }

          //std::cout << " " << std::endl;
          //std::cout << "rangeIdxToMeters " << rangeIdxToMeters << std::endl;
          //std::cout << " " << std::endl;

					obj_range[counter_captured_objects] = rangeIdx_int*rangeIdxToMeters;
					break;
				case 2:
					doppler_pair[0] = byte;
					break;
				case 3: // Get Doppler
					doppler_pair[1] = byte;
          dopIdx_unsigned_int = (doppler_pair[1] << 8) | doppler_pair[0]; // Temporary value for conversion
          dopIdx_int = dopIdx_unsigned_int; // Cast to 32-bit signed integer. Is this ok<ay?
          if (dopIdx_int > 32767){
                  dopIdx_int = dopIdx_int - 65536;
                                  }
          //if(dopIdx_int > numDopplerBins/2-1){ // The second half of doppler indexes are the negative values
          //        dopIdx_int = dopIdx_int - numDopplerBins;
          //                        }
          obj_doppler[counter_captured_objects] = dopIdx_int*dopplerResolutionMps;
					break;
				case 4:
					peak_pair[0] = byte;
					break;
				case 5: // Get Peak
					peak_pair[1] = byte;
					obj_peak[counter_captured_objects] = (peak_pair[1] << 8) | peak_pair[0];
					break;
				case 6:
					x_pair[0] = byte;
					break;
				case 7: // Get x
					x_pair[1] = byte;
          x_coord_int = (x_pair[1] << 8) | x_pair[0];
          if(x_coord_int > 32767){
                  x_coord_int = x_coord_int - 65536;
                }
          obj_x[counter_captured_objects] = x_coord_int/xyzQFormat;
					break;
				case 8:
					y_pair[0] = byte;
					break;
				case 9: // Get y
          y_pair[1] = byte;
          y_coord_int = (y_pair[1] << 8) | y_pair[0];
          if(y_coord_int > 32767){
                y_coord_int = y_coord_int - 65536;
              }

          obj_y[counter_captured_objects] = y_coord_int/xyzQFormat;
					break;
				case 10:
					z_pair[0] = byte;
					break;
				case 11: // Get z
          z_pair[1] = byte;
          z_coord_int = (z_pair[1] << 8) | z_pair[0];
          if(z_coord_int > 32767){
              z_coord_int = z_coord_int - 65536;
            }

          obj_z[counter_captured_objects] =  z_coord_int/xyzQFormat;
					break;
			}


			obj_group++; // This helps to detect which bytes are from the same object

			if (obj_group == 12){
				// Executed in the same cycle as obj_group = 11
				obj_group = 0;

  			counter_captured_objects++; // Increases when a group of 12 bytes was captured

#ifdef DEBUG_OUT
        std::cout << "" << std::endl;
        std::cout << "Object captured number " << counter_captured_objects << std::endl;
        std::cout << "Range " << obj_range[counter_captured_objects-1] << std::endl;
        std::cout << "Doppler " << obj_doppler[counter_captured_objects-1] << std::endl;
        std::cout << "Peak " << obj_peak[counter_captured_objects-1] << std::endl;
        std::cout << "X " << obj_x[counter_captured_objects-1] << std::endl;
        std::cout << "y " << obj_y[counter_captured_objects-1] << std::endl;
        std::cout << "z " << obj_z[counter_captured_objects-1] << std::endl;
        std::cout << "" << std::endl;
#endif


			}

      //std::cout << "counter_obj_bytes" << counter_obj_bytes << std::endl;
			counter_obj_bytes++; // Counts all bytes received for the list of objects // It helps to determine when we are done getting the objects
			if (counter_obj_bytes == numDetObj_int*12){ // List of objects goes from 0 to numDetObj_int*2-1, so we are done here
#ifdef DEBUG_OUT
        std::cout << " " << std::endl;
        std::cout << "Current State GetObjects" << std::endl;
				std::cout << "Total Captured Objects: " << counter_captured_objects << std::endl;
				std::cout << "Total Expected Objects: " << numDetObj_int << std::endl;
        std::cout << " " << std::endl;
        std::cout << "Testing Range IDX to meters  " << (double)rangeIdxToMeters << std::endl;
        std::cout << "Testing Doppler resoltuion  " << (double)dopplerResolutionMps << std::endl;
        std::cout << "Number of doppler bins  " << numDopplerBins << std::endl;
        std::cout << " " << std::endl;
#endif
				counter_obj_bytes = 0; // Restart
				counter_captured_objects = 0;
				obj_group = 0;

				tlv_counter = 0;
				TLV_required = 2;
				m_FSM.transition(RadarState::SearchTLV); // Go and search for TLV 2
		#ifdef DEBUG_OUT
        std::cout << "Going to Search TLV 2" << std::endl;
        std::cout << " " << std::endl;
#endif

			}

			// obj_range[100] obj_doppler[100] obj_x[100]
			// After getting the objects we could get to get RangeProfile
		}

	}
	else if (state == RadarState::GetRangeProfile){
		if(event == Event::Update){

       if(counter_bytes_in_range_profile % 2 == 0){ // Even byte
          range_profile_pair[0] = byte;
       }
       else { // Odd byte
         range_profile_pair[1] = byte;
         range_profile[counter_values_in_range_profile] = (range_profile_pair[1] << 8) | range_profile_pair[0];
         counter_values_in_range_profile++;
       }


        counter_bytes_in_range_profile++;
        if (counter_bytes_in_range_profile == 512){ // Bytes in range profile go from 0 to 511
            // Then we are done. Go to search TLV 3
            counter_values_in_range_profile = 0; // Reset
            counter_bytes_in_range_profile = 0; // Reset

            tlv_counter = 0;
    				TLV_required = 3;
    				m_FSM.transition(RadarState::SearchTLV); // Go and search for TLV 2

  #ifdef DEBUG_OUT
            std::cout << " " << std::endl;
            std::cout << "Current State Range Profile" << std::endl;

            std::cout << "Going to Get Search TLV 3" << std::endl;
            std::cout << " " << std::endl;
#endif


        }

				// After getting the range profile we could go to get the NOiseProfile
		}
	}
	else if (state == RadarState::GetNoiseProfile){
		if(event == Event::Update){

				// After getting the Noise profile we could go for a possible cleaning of the objects using a loop and the info from range and noise profile
        if(counter_bytes_in_noise_profile % 2 == 0){ // Even byte
           noise_profile_pair[0] = byte;
        }
        else { // Odd byte
          noise_profile_pair[1] = byte;
          noise_profile[counter_values_in_noise_profile] = (noise_profile_pair[1] << 8) | noise_profile_pair[0];
          counter_values_in_noise_profile++;
        }


         counter_bytes_in_noise_profile++;
         if (counter_bytes_in_noise_profile == 512){ // Bytes in range profile go from 0 to 511
             // Then we are done. Go to search TLV 3
            counter_values_in_noise_profile = 0; // Reset
            counter_bytes_in_noise_profile = 0; // Reset

            tlv_counter = 0;
     				TLV_required = 6;
            m_FSM.transition(RadarState::SearchTLV); // Go and search for TLV 2
     				// Probably to restart pos_next_TLV, but I dont think it is necessary
#ifdef DEBUG_OUT
            std::cout << " " << std::endl;
            std::cout << "Current State Noise Profile" << std::endl;
            std::cout << "Going to Search TLV 6" << std::endl;
            std::cout << " " << std::endl;
#endif

         }

		}

	}
  else if (state == RadarState::GetStats){
		if(event == Event::Update){

      // After getting the Noise profile we could go for a possible cleaning of the objects using a loop and the info from range and noise profile
      if(counter_bytes_in_stats_profile % 2 == 0){ // Even byte
         stats_profile_pair[0] = byte;
      }
      else { // Odd byte
        stats_profile_pair[1] = byte;
        stats_profile[counter_values_in_stats_profile] = (stats_profile_pair[1] << 8) | stats_profile_pair[0];
        counter_values_in_stats_profile++;
      }


       counter_bytes_in_stats_profile++;
       if (counter_bytes_in_stats_profile == 24){ // Bytes in stats profile go from 0 to 23
           // Then we are done. Go to search TLV 3
          counter_values_in_stats_profile = 0; // Reset
          counter_bytes_in_stats_profile = 0; // Reset

          //tlv_counter = 0;
          //TLV_required = 6;
          m_FSM.transition(RadarState::Preprocessing); // Go and search for TLV 2
#ifdef DEBUG_OUT
          std::cout << " " << std::endl;
          std::cout << "Current State Stats Profile" << std::endl;
          std::cout << "Going to Preprocess" << std::endl;
          std::cout << " " << std::endl;
#endif

       }


    }
  }
  else if (state == RadarState::Preprocessing){ // If we preprocess, do we lose bytes?
		if(event == Event::Update){ // So far no preprocessing is implemented, so just start again
      m_FSM.transition(RadarState::SearchKeyword); // Are we losing a frame here? Are we having consecutive frames?
#ifdef DEBUG_OUT
      std::cout << " " << std::endl;
      std::cout << "Current State Preprocessing" << std::endl; // Not implemented yet
      std::cout << " " << std::endl;
#endif
    }
  }
	else if (state == RadarState::MalformedPacket){
		if(event == Event::Update){
      #ifdef DEBUG_OUT
      std::cout << " " << std::endl;
      std::cout << "Current State Malformed packet :(" << std::endl; // Not implemented yet
      std::cout << " " << std::endl;
#endif
      m_FSM.transition(RadarState::SearchKeyword); // Go and start looking for the next keyword because this packet is malformed
		}

	}


        return true;
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool update()
    {
        return m_FSM.update();
    }

    void getObjCoords(std::vector<std::tuple<double,double,double>> &coords)
    {
      coords.clear();

    {
      coords.reserve(numDetObj_int);

      for(int i = 0; i < numDetObj_int; i++)
      {
        coords.emplace_back(obj_x[i], obj_y[i], obj_z[i]);
      }
    }
    }
    bool hasData() const{ return m_HasData; }
    size_t getGoodFrame() const{ return m_GoodFrame; }
private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    uint8_t readByte()
    {
        if(m_Buffer.empty() || m_BufferPosition == m_Buffer.size()) {
            m_SerialPort.Read(m_Buffer, 4096, 5000);
            assert(m_Buffer.size() == 4096);
            m_BufferPosition = 0;
        }

        return (uint8_t)m_Buffer[m_BufferPosition++];
    }


    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    FSM<RadarState> m_FSM;

    SerialPort m_SerialPort;

    SerialPort::DataBuffer m_Buffer;
    size_t m_BufferPosition;


    bool m_HasData;
};
}
