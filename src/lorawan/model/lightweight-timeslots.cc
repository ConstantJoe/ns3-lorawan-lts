// Copyright (c) 2013 Your Company. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <iostream>
#include <exception>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <complex> 
#include <ns3/log.h>

#include "lightweight-timeslots.h"

#define PI 3.14159265

#define DEBUG 0

#define MAX_C 3 //DR dependent

/*

The scoring function of TiCom was used to judge the quality of (p, o) pairs.

TiCom does an exhaustive search for candidate solutions, which is not realistic here as there are many time slots. So we come up with a way of finding decent
quality candidate solutions, then do a quick search around the solutions with the scoring function to find the best fit.

First approach tried: FFT. But as the sequence is in the form of a pulse wave, there are many peaks in the FFT because the sine waves only approximate the squareness
of the wave

Second approach tried: Walsh spectrum. But Walsh spectrum is only applicable for square waves, and our sequence is not a square wave. And I couldn't find a way of
converting it to a square wave that wouldn't require prior knowledge of the periodicities.

Third approach tried: autocorrelation. Autocorrelation is not a sufficient approach by itself, but if we do a search with the scoring function around the result, and 
filter out matched values, then the results are good.

The third approach appears to work. But it is slow (0.08s per periodicity picked up in the sequence). Optimisation ideas:

TODO:
Factor in IDs in collision avoidance - all periodicities will have to change.if one change is proposed
Factor in MAX_C, which is DR dependant
Definitely do!: generics
Move over to ns-3

*/



namespace ns3 {



NS_LOG_COMPONENT_DEFINE ("LightWeightTimeslots");

NS_OBJECT_ENSURE_REGISTERED (LightweightTimeslots);


Ptr<LightweightTimeslots> LightweightTimeslots::m_lorawanLightweightTimeslotsPtr = NULL;

const std::vector<LoRaWANTimeSlotsPerDataRate> LightweightTimeslots::m_timeSlotsPerDataRate = {
  /*{0, 1289}, 
  {1, 2306},
  {2, 5158},
  {3, 9231},
  {4, 16666},
  {5, 30508},*/
  {0, 1986}, 
  {1, 3972},
  {2, 7944},
  {3, 15888},
  {4, 31776},
  {5, 63552},
  //{6, }, //TODO: decide on use of DR6
};


TypeId
LightweightTimeslots::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LightWeightTimeslots")
    .SetParent<Object> ()
    .SetGroupName("LoRaWAN")
    .AddConstructor<LightweightTimeslots> ()
  ;
  return tid;
}

  LightweightTimeslots::LightweightTimeslots (void) {

  /*alpha = 0.8;
  slot_size = 0.118; //2.793 for DR0;
  periodicity = 989;
  offset = 564;
  time_slots = 30508;
  O = std::vector<unsigned char>(time_slots, 0); //1289 for DR0*/

  //correlation_holder = std::vector<float>(O.size()*2 - 1);

  /*int total_c = 0;
  int total_c_wo_colls = 0;

  //generate input
  int inte = round(periodicity / slot_size);
  for(int x=0;x<int(time_slots / inte);x++) {
    O[inte * x + offset] = 1;
    total_c++;
    total_c_wo_colls++;
  }
  std::cout << "slots added: " << int(time_slots / inte) << std::endl;

  periodicity = 600;
  offset = 101;
  inte = round(periodicity / slot_size);
  for(int x=0;x<int(time_slots / inte);x++) {
    if(!O[inte * x + offset]) total_c_wo_colls++;

    O[inte * x + offset] = 1;
    total_c++;
  } 
  std::cout << "slots added: " << int(time_slots / inte) << std::endl;

  std::cout << "total 1s added: " << total_c << " and excluding collisions: " << total_c_wo_colls << std::endl;*/

  /*
  Here are the sizes required for each DR:
  {0, 1289},
  {1, 2306},
  {2, 5158},
  {3, 9231},
  {4, 16666},
  {5, 30508},
  */

  //For a 64 byte packet:
  uint32_t DR0_size = m_timeSlotsPerDataRate[0].m_slots;
  uint32_t DR1_size = m_timeSlotsPerDataRate[1].m_slots;
  uint32_t DR2_size = m_timeSlotsPerDataRate[2].m_slots;
  uint32_t DR3_size = m_timeSlotsPerDataRate[3].m_slots;
  uint32_t DR4_size = m_timeSlotsPerDataRate[4].m_slots;
  uint32_t DR5_size = m_timeSlotsPerDataRate[5].m_slots;


  //For a 33 byte packet:
  /*int DR0_size = 1989;
  int DR1_size = 3647;
  int DR2_size = 7954;
  int DR3_size = 14588;
  int DR4_size = 26940;
  int DR5_size = 50045;*/

  /*int DR0_size = 2000;
  int DR1_size = 3600;
  int DR2_size = 8000;
  int DR3_size = 14600;
  int DR4_size = 27000;
  int DR5_size = 50000;*/

  //DR0
  inX_DR0 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR0_size + DR0_size*2 - 1)); //x.size for the data, and x.size*2-1 for the 0 padding
  outX_DR0 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR0_size + DR0_size*2 - 1)); //x.size for the data, and x.size*2-1 for the 0 padding

  inY_DR0 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR0_size + DR0_size*2 - 1));
  outY_DR0 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR0_size + DR0_size*2 - 1));

  inZ_DR0 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR0_size + DR0_size*2 - 1));
  outZ_DR0 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR0_size + DR0_size*2 - 1));

  pX_DR0 = fftw_plan_dft_1d(DR0_size + DR0_size*2 - 1, inX_DR0, outX_DR0, FFTW_FORWARD, FFTW_ESTIMATE);
  pY_DR0 = fftw_plan_dft_1d(DR0_size + DR0_size*2 - 1, inY_DR0, outY_DR0, FFTW_FORWARD, FFTW_ESTIMATE);
  pZ_DR0 = fftw_plan_dft_1d(DR0_size + DR0_size*2 - 1, inZ_DR0, outZ_DR0, FFTW_BACKWARD, FFTW_ESTIMATE);

  correlation_holder_DR0 = std::vector<float>(DR0_size*2 - 1);

  //DR1
  inX_DR1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR1_size + DR1_size*2 - 1)); //x.size for the data, and x.size*2-1 for the 0 padding
  outX_DR1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR1_size + DR1_size*2 - 1)); //x.size for the data, and x.size*2-1 for the 0 padding

  inY_DR1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR1_size + DR1_size*2 - 1));
  outY_DR1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR1_size + DR1_size*2 - 1));

  inZ_DR1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR1_size + DR1_size*2 - 1));
  outZ_DR1 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR1_size + DR1_size*2 - 1));

  pX_DR1 = fftw_plan_dft_1d(DR1_size + DR1_size*2 - 1, inX_DR1, outX_DR1, FFTW_FORWARD, FFTW_ESTIMATE);
  pY_DR1 = fftw_plan_dft_1d(DR1_size + DR1_size*2 - 1, inY_DR1, outY_DR1, FFTW_FORWARD, FFTW_ESTIMATE);
  pZ_DR1 = fftw_plan_dft_1d(DR1_size + DR1_size*2 - 1, inZ_DR1, outZ_DR1, FFTW_BACKWARD, FFTW_ESTIMATE);

  correlation_holder_DR1 = std::vector<float>(DR1_size*2 - 1);

  //DR2
  inX_DR2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR2_size + DR2_size*2 - 1)); //x.size for the data, and x.size*2-1 for the 0 padding
  outX_DR2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR2_size + DR2_size*2 - 1)); //x.size for the data, and x.size*2-1 for the 0 padding

  inY_DR2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR2_size + DR2_size*2 - 1));
  outY_DR2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR2_size + DR2_size*2 - 1));

  inZ_DR2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR2_size + DR2_size*2 - 1));
  outZ_DR2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR2_size + DR2_size*2 - 1));

  pX_DR2 = fftw_plan_dft_1d(DR2_size + DR2_size*2 - 1, inX_DR2, outX_DR2, FFTW_FORWARD, FFTW_ESTIMATE);
  pY_DR2 = fftw_plan_dft_1d(DR2_size + DR2_size*2 - 1, inY_DR2, outY_DR2, FFTW_FORWARD, FFTW_ESTIMATE);
  pZ_DR2 = fftw_plan_dft_1d(DR2_size + DR2_size*2 - 1, inZ_DR2, outZ_DR2, FFTW_BACKWARD, FFTW_ESTIMATE);

  correlation_holder_DR2 = std::vector<float>(DR2_size*2 - 1);

  //DR3
  inX_DR3 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR3_size + DR3_size*2 - 1)); //x.size for the data, and x.size*2-1 for the 0 padding
  outX_DR3 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR3_size + DR3_size*2 - 1)); //x.size for the data, and x.size*2-1 for the 0 padding

  inY_DR3 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR3_size + DR3_size*2 - 1));
  outY_DR3 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR3_size + DR3_size*2 - 1));

  inZ_DR3 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR3_size + DR3_size*2 - 1));
  outZ_DR3 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR3_size + DR3_size*2 - 1));

  pX_DR3 = fftw_plan_dft_1d(DR3_size + DR3_size*2 - 1, inX_DR3, outX_DR3, FFTW_FORWARD, FFTW_ESTIMATE);
  pY_DR3 = fftw_plan_dft_1d(DR3_size + DR3_size*2 - 1, inY_DR3, outY_DR3, FFTW_FORWARD, FFTW_ESTIMATE);
  pZ_DR3 = fftw_plan_dft_1d(DR3_size + DR3_size*2 - 1, inZ_DR3, outZ_DR3, FFTW_BACKWARD, FFTW_ESTIMATE);

  correlation_holder_DR3 = std::vector<float>(DR3_size*2 - 1);

  //DR4
  inX_DR4 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR4_size + DR4_size*2 - 1)); //x.size for the data, and x.size*2-1 for the 0 padding
  outX_DR4 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR4_size + DR4_size*2 - 1)); //x.size for the data, and x.size*2-1 for the 0 padding

  inY_DR4 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR4_size + DR4_size*2 - 1));
  outY_DR4 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR4_size + DR4_size*2 - 1));

  inZ_DR4 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR4_size + DR4_size*2 - 1));
  outZ_DR4 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR4_size + DR4_size*2 - 1));

  pX_DR4 = fftw_plan_dft_1d(DR4_size + DR4_size*2 - 1, inX_DR4, outX_DR4, FFTW_FORWARD, FFTW_ESTIMATE);
  pY_DR4 = fftw_plan_dft_1d(DR4_size + DR4_size*2 - 1, inY_DR4, outY_DR4, FFTW_FORWARD, FFTW_ESTIMATE);
  pZ_DR4 = fftw_plan_dft_1d(DR4_size + DR4_size*2 - 1, inZ_DR4, outZ_DR4, FFTW_BACKWARD, FFTW_ESTIMATE);

  correlation_holder_DR4 = std::vector<float>(DR4_size*2 - 1);

  //DR5
  inX_DR5 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR5_size + DR5_size*2 - 1)); //x.size for the data, and x.size*2-1 for the 0 padding
  outX_DR5 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR5_size + DR5_size*2 - 1)); //x.size for the data, and x.size*2-1 for the 0 padding

  inY_DR5 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR5_size + DR5_size*2 - 1));
  outY_DR5 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR5_size + DR5_size*2 - 1));

  inZ_DR5 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (DR5_size + DR5_size*2 - 1));
  outZ_DR5 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *(DR5_size + DR5_size*2 - 1));

  pX_DR5 = fftw_plan_dft_1d(DR5_size + DR5_size*2 - 1, inX_DR5, outX_DR5, FFTW_FORWARD, FFTW_ESTIMATE);
  pY_DR5 = fftw_plan_dft_1d(DR5_size + DR5_size*2 - 1, inY_DR5, outY_DR5, FFTW_FORWARD, FFTW_ESTIMATE);
  pZ_DR5 = fftw_plan_dft_1d(DR5_size + DR5_size*2 - 1, inZ_DR5, outZ_DR5, FFTW_BACKWARD, FFTW_ESTIMATE);

  correlation_holder_DR5 = std::vector<float>(DR5_size*2 - 1);

}

LightweightTimeslots::~LightweightTimeslots (void) {
  fftw_free(inX_DR0);
  fftw_free(inY_DR0);
  fftw_free(outX_DR0);
  fftw_free(outY_DR0);
  fftw_free(inZ_DR0);
  fftw_free(outZ_DR0);

  fftw_free(inX_DR1);
  fftw_free(inY_DR1);
  fftw_free(outX_DR1);
  fftw_free(outY_DR1);
  fftw_free(inZ_DR1);
  fftw_free(outZ_DR1);

  fftw_free(inX_DR2);
  fftw_free(inY_DR2);
  fftw_free(outX_DR2);
  fftw_free(outY_DR2);
  fftw_free(inZ_DR2);
  fftw_free(outZ_DR2);


  fftw_free(inX_DR3);
  fftw_free(inY_DR3);
  fftw_free(outX_DR3);
  fftw_free(outY_DR3);
  fftw_free(inZ_DR3);
  fftw_free(outZ_DR3);

  fftw_free(inX_DR4);
  fftw_free(inY_DR4);
  fftw_free(outX_DR4);
  fftw_free(outY_DR4);
  fftw_free(inZ_DR4);
  fftw_free(outZ_DR4);

  fftw_free(inX_DR5);
  fftw_free(inY_DR5);
  fftw_free(outX_DR5);
  fftw_free(outY_DR5);
  fftw_free(inZ_DR5);
  fftw_free(outZ_DR5);
}

/*void
LoRaWANNetworkServer::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);

  Object::DoInitialize ();

  //this->m_lorawanLightweightTimeslotsPtr =  new LightweightTimeslots::LightweightTimeslots();


  LightweightTimeslots::m_lorawanLightweightTimeslotsPtr = CreateObject<LightweightTimeslots> ();
  LightweightTimeslots::m_lorawanLightweightTimeslotsPtr->Initialize ();

  m_lightweightTimeslotsPtr = LightweightTimeslots::m_lorawanLightweightTimeslotsPtr; //!< Pointer to singleton
}*/

void LightweightTimeslots::Run() {


  // run TiCom on each sequence, store output
  /*std::cout << "Starting TiCom!" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  std::vector<std::tuple <int, int>> S = TiComWithAutocorrelation(O, alpha);
  auto finish = std::chrono::high_resolution_clock::now();
  std::cout << "Finished TiCom!" << std::endl;

  for(uint x=0;x<S.size();x++) {
    std::cout << round(std::get<0>(S[x])*slot_size) <<  " " << std::get<1>(S[x]) << std::endl;
  }
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Elapsed time: " << elapsed.count() << " s\n";*/
  
  //each device has a sequence, and outputs a vector with elements in the format (p, o, uid, change). In the format Periodicity

  // TODO:
  // take code from collision_avoidance.py, convert to C++.
  // test collision prediction with randomised err data (false positives i.e. event-based data)
  // add this to ns-3 module

  /*std::vector<Periodicity> periodicities { 
    {200, 0, 1, 0}, 
    {200, 0, 2, 0}, 
    {200, 0, 3, 0}, 
    {200, 1, 4, 0}, 
    {200, 2, 5, 0},  
    {400, 2, 7, 0}, 
    {125, 5, 8, 0}, 
    {125, 3, 9, 0} 
  };
  CollisionAvoidance(periodicities, 0.2);
  for(uint i=0; i<periodicities.size(); i++) {
    std::cout << periodicities[i].p << " " << periodicities[i].o << " " << periodicities[i].change << std::endl;
  }*/
}

std::vector<std::tuple <int, int>> LightweightTimeslots::TiComWithAutocorrelation(std::vector<unsigned char>& O, float alpha, uint8_t dr) {
  std::vector<unsigned char> O_filter(O);

  std::tuple<int, int> s_candidate = FindCandidateSolutionAutocorrelation(O, dr);
  int s_p = std::get<0>(s_candidate);
  int rough_o = std::get<1>(s_candidate);

  std::vector< std::tuple <int, int> > S(0);
  S.reserve(20);
  int search_space = 10;

  //check the 20 values around it TODO: this should be relative to the size of O.
  if(rough_o - search_space >= 0) {
    if(rough_o + search_space < s_p) {
      for(int i=rough_o - search_space;i< rough_o + search_space; i++) {
        S.push_back(std::tuple<int, int>(s_p, i));
      }
    } else {
      for(int i=rough_o - search_space; i<s_p; i++) {
        S.push_back(std::tuple<int, int>(s_p, i));
      }
      for(int i=0;i< rough_o + search_space - s_p; i++) {
        S.push_back(std::tuple<int, int>(s_p, i));
      }
    }
  } else {
    for(int i=0; i<rough_o + search_space; i++) {
      S.push_back(std::tuple<int, int>(s_p, i));
    }
    for(int i=s_p - rough_o - search_space;i< s_p; i++) {
      S.push_back(std::tuple<int, int>(s_p, i));
    }
  }
  //std::cout << "Local search for p value " << s_p << std::endl;
  //std::cout << "rough_o this time was " << rough_o << std::endl;

  float abs_T = 0;
  float abs_F = 0;
  for(uint x=0;x<O.size();x++) {
    if(O[x]) {
      abs_T++;
    }
    else{
      abs_F++;
    }
  }

  std::vector<std::tuple <int, int>> S_star(0); //S_star = [] #begins empty.
  S_star.reserve(10); //reserve enough space
  std::vector<unsigned char> Ps_star(O.size(), 0);  //Ps_star = [0] * len(O) #begins all 0

  //argmaxOverList
  float max_found = -100000;
  int max_ind = 0;
  for(uint i=0; i<S.size(); i++) {
    float res = TiComScore(S.at(i), Ps_star, O, alpha, abs_T, abs_F);
    //std::cout << std::get<1>(S.at(i)) << " " << res << std::endl;
    if (max_found < res) {
      
      max_found = res;
      max_ind = i;
    }
  }
  //std::cout << "going for o val " << std::get<1>(S.at(max_ind)) << " with score " << max_found << std::endl;
  std::tuple <int, int> s = S.at(max_ind);
  float score = max_found;

  while (score > 0) { 
    S_star.push_back(std::tuple <int, int>(std::get<0>(s), std::get<1>(s)));

    int l = std::get<0>(s);
    int i = std::get<1>(s);

    int O_count = 0;
    for (int x=0; x<int(O.size()); x++) {
      if(x % l == i && Ps_star[x] == 0) {
        Ps_star[x] = 1;
        O_filter[x] = 0;
      }
      if(O_filter[x] == 1) {
        O_count++;
      }
    }

    //remove s pairs that are subsets of other pairs
    std::vector< std::tuple <int, int> > to_remove(0);
    to_remove.reserve(S_star.size());      

    for (uint x=0; x<S_star.size(); x++) {
      int l_dash = std::get<0>(S_star[x]);
      int i_dash = std::get<1>(S_star[x]);
      if(l_dash % l  == 0 && i_dash % l == i) {
        if (!(l_dash == l && i_dash == i)) {
          to_remove.push_back(S_star[x]);
        }
      }
    }
    for (uint x=0; x<to_remove.size(); x++) {
      S_star.erase(std::remove(S_star.begin(), S_star.end(), to_remove.at(x)), S_star.end());  // can prob. be done in a faster way       
    }

    if(O_count == 0) {
      //all places are covered, so we're finished
      break;
    }

    s_candidate = FindCandidateSolutionAutocorrelation(O_filter, dr);
    s_p = std::get<0>(s_candidate);
    rough_o = std::get<1>(s_candidate);
    S.clear();

    //check the 20 values around it TODO: this should be relative to the size of O.
    //std::cout << "Local search for p value " << s_p << std::endl;
    //std::cout << "rough_o this time was " << rough_o << std::endl;
    if(rough_o - search_space >= 0) {
      if(rough_o + search_space < s_p) {
        for(int i=rough_o - search_space;i< rough_o + search_space; i++) {
          S.push_back(std::tuple<int, int>(s_p, i));
        }
      } else {
        for(int i=rough_o - search_space; i<s_p; i++) {
          S.push_back(std::tuple<int, int>(s_p, i));
        }
        for(int i=0;i< rough_o + search_space - s_p; i++) {
          S.push_back(std::tuple<int, int>(s_p, i));
        }
      }
    } else {
      for(int i=0; i<rough_o + search_space; i++) {
        S.push_back(std::tuple<int, int>(s_p, i));
      }
      for(int i=s_p - rough_o - search_space;i< s_p; i++) {
        S.push_back(std::tuple<int, int>(s_p, i));
      }
    }

    max_found = -100000;
    max_ind = 0;
    for(uint i=0; i<S.size(); i++) {
      float res = TiComScore(S.at(i), Ps_star, O, alpha, abs_T, abs_F);
      //std::cout << std::get<1>(S.at(i)) << " " << res << std::endl;
      if (max_found < res) {
       max_found = res;
       max_ind = i;
     }
   }
   //std::cout << "going for o val " << std::get<1>(S.at(max_ind)) << " with score " << max_found << std::endl;
   s = S.at(max_ind);
   score = max_found;
 }

 return S_star;
}


float LightweightTimeslots::TiComScore(std::tuple <int, int> s, std::vector<unsigned char>& Ps_star, std::vector<unsigned char>& O, float alpha, float abs_T, float abs_F) {
    //abs_T is the count of 1s in O
    //abs_F is the count of 0s in O
    //abs_TPs is the count of how many 1s the candidate solution covers, that haven't already been covered by Ps_star
    //abs_FPs is the count of how many false positives the candidate solution provides, that haven't already been provided in Ps_star

  int l = std::get<0>(s);
  int i = std::get<1>(s);

  int abs_TPs = 0;
  int abs_FPs = 0;

  for(int x=0;x<int(O.size());x++) {

    if(O[x]==1 && x%l==i && Ps_star[x]==0) {
      abs_TPs++;
    } else if(O[x]==0 && x%l==i && Ps_star[x]==0) {
      abs_FPs++;
    }
  }
  return (1 - alpha)*((float) abs_TPs / (float) abs_T) - (alpha)*((float) abs_FPs / (float) abs_F);
} 



std::tuple<int, int> LightweightTimeslots::FindCandidateSolutionAutocorrelation(const std::vector<unsigned char>& O, uint8_t dr) {

  if(dr == 0) {
       //autocorrelate the sequence to find a candidate periodicity
  CorrelationDR0(O, O, correlation_holder_DR0);

  float periodicity = 0.0;
  int start_point = correlation_holder_DR0.size() / 2;

  //choose the max value found
  float max_autoc = 0.0;
  for(uint i=start_point+2; i<correlation_holder_DR0.size();i++){
    if(correlation_holder_DR0[i] + correlation_holder_DR0[i-1] > max_autoc) {
      max_autoc = correlation_holder_DR0[i] + correlation_holder_DR0[i-1];
      if(correlation_holder_DR0[i] > correlation_holder_DR0[i-1]) {
          periodicity = i - start_point;
      } else {
          periodicity = i - 1 -  start_point;  
      }
    }
  }

  if(periodicity == 0) {
    return std::tuple<int, int>(1, 0);
  }

  /*if(periodicity == 214) {
    periodicity++;
  }*/
  //std::cout << "periodicity DR0 " << periodicity << std::endl;

  // generate a sequence with that periodicity with the offset 0
  std::vector<unsigned char> O_copy2(O.size(), 0);
  for(uint i=0;i<O_copy2.size();i+=periodicity) {
    O_copy2[i] = 1;
  }
  

  //get the correlation of that sequence with the original sequence, and find the angle between them 
  CorrelationDR0(O, O_copy2, correlation_holder_DR0);
  float max = 0.0;
  int index = 0;
  for(uint i=start_point; i<correlation_holder_DR0.size();i++) {
    if(correlation_holder_DR0[i] > max) {
      max = correlation_holder_DR0[i];
      index = i-start_point;
    }
  }

  return std::tuple<int, int>(int(periodicity), index);
  }
  else if(dr == 1)
  {
    CorrelationDR1(O, O, correlation_holder_DR1);

  float periodicity = 0.0;
  int start_point = correlation_holder_DR1.size() / 2;

  //choose the max value found
  float max_autoc = 0.0;
  for(uint i=start_point+2; i<correlation_holder_DR1.size();i++){
    if(correlation_holder_DR1[i] + correlation_holder_DR1[i-1] > max_autoc) {
      max_autoc = correlation_holder_DR1[i] + correlation_holder_DR1[i-1];
      if(correlation_holder_DR1[i] > correlation_holder_DR1[i-1]) {
          periodicity = i - start_point;
      } else {
          periodicity = i - 1 -  start_point;  
      }
    }
  }

  if(periodicity == 0) {
    return std::tuple<int, int>(1, 0);
  }

  /*if(periodicity == 385) { //temp fix
    periodicity--;
  }*/

  //std::cout << "periodicity DR1 " << periodicity << std::endl;

  // generate a sequence with that periodicity with the offset 0
  std::vector<unsigned char> O_copy2(O.size(), 0);
  for(uint i=0;i<O_copy2.size();i+=periodicity) {
    O_copy2[i] = 1;
  }
  

  //get the correlation of that sequence with the original sequence, and find the angle between them 
  //std::cout << "correlation to find angle" << std::endl;
  CorrelationDR1(O, O_copy2, correlation_holder_DR1);
  float max = 0.0;
  int index = 0;
  for(uint i=start_point; i<correlation_holder_DR1.size();i++) {
    if(correlation_holder_DR1[i] > max) {
      max = correlation_holder_DR1[i];
      index = i-start_point;
    }
  }

  return std::tuple<int, int>(int(periodicity), index);
  }
  else if(dr == 2) 
  {
    CorrelationDR2(O, O, correlation_holder_DR2);

  float periodicity = 0.0;
  int start_point = correlation_holder_DR2.size() / 2;

  //choose the max value found
  float max_autoc = 0.0;
  for(uint i=start_point+2; i<correlation_holder_DR2.size();i++){
    if(correlation_holder_DR2[i] + correlation_holder_DR2[i-1] > max_autoc) {
      max_autoc = correlation_holder_DR2[i] + correlation_holder_DR2[i-1];
      if(correlation_holder_DR2[i] > correlation_holder_DR2[i-1]) {
          periodicity = i - start_point;
      } else {
          periodicity = i - 1 -  start_point;  
      }
    }
  }

  if(periodicity == 0) {
    return std::tuple<int, int>(1, 0);
  }
  /*if(periodicity == 859) {
    periodicity++;
  }*/

  //std::cout << "periodicity DR2 " << periodicity << std::endl;

  // generate a sequence with that periodicity with the offset 0
  std::vector<unsigned char> O_copy2(O.size(), 0);
  for(uint i=0;i<O_copy2.size();i+=periodicity) {
    O_copy2[i] = 1;
  }
  

  //get the correlation of that sequence with the original sequence, and find the angle between them 
  CorrelationDR2(O, O_copy2, correlation_holder_DR2);
  float max = 0.0;
  int index = 0;
  for(uint i=start_point; i<correlation_holder_DR2.size();i++) {
    if(correlation_holder_DR2[i] > max) {
      max = correlation_holder_DR2[i];
      index = i-start_point;
    }
  }

  return std::tuple<int, int>(int(periodicity), index);
  }
  else if(dr == 3) 
  {
    CorrelationDR3(O, O, correlation_holder_DR3);

  float periodicity = 0.0;
  int start_point = correlation_holder_DR3.size() / 2;

  //choose the max value found
  float max_autoc = 0.0;
  //std::cout << "autocorrelation start" << std::endl;
  for(uint i=start_point+2; i<correlation_holder_DR3.size();i++){
    if(correlation_holder_DR3[i] + correlation_holder_DR3[i-1] > max_autoc) {

      max_autoc = correlation_holder_DR3[i] + correlation_holder_DR3[i-1];
      if(correlation_holder_DR3[i] > correlation_holder_DR3[i-1]) {
          periodicity = i - start_point;
      } else {
          periodicity = i - 1 -  start_point;  
      }
      //periodicity = i - start_point;
      //std::cout << "found a max at: " << periodicity << " " << max_autoc << std::endl;

    }
    /*if(i-start_point == 1538) {
        std::cout << "val at 1538 is " << correlation_holder_DR3[i] << std::endl;
      }
    if(i-start_point == 1539) {
        std::cout << "val at 1539 is " << correlation_holder_DR3[i] << std::endl;
      }
    if(i-start_point == 1540) {
        std::cout << "val at 1540 is " << correlation_holder_DR3[i] << std::endl;
      }
    if(i-start_point == 3076) {
        std::cout << "val at 3076 is " << correlation_holder_DR3[i] << std::endl;
      }
    if(i-start_point == 3077) {
        std::cout << "val at 3077 is " << correlation_holder_DR3[i] << std::endl;
      }
    if(i-start_point == 3078) {
        std::cout << "val at 3078 is " << correlation_holder_DR3[i] << std::endl;
      }*/
  }
  //std::cout << "max value was " << correlation_holder_DR3.size() - (start_point+1) << std::endl;
 // std::cout << std::endl;

  if(periodicity == 0) {
    return std::tuple<int, int>(1, 0);
  }
  /*if(periodicity == 1538) {
    periodicity++;
  }*/

  //std::cout << "periodicity DR3 " << periodicity << std::endl;

  // generate a sequence with that periodicity with the offset 0
  std::vector<unsigned char> O_copy2(O.size(), 0);
  for(uint i=0;i<O_copy2.size();i+=periodicity) {
    O_copy2[i] = 1;
  }
  

  //get the correlation of that sequence with the original sequence, and find the angle between them 
  CorrelationDR3(O, O_copy2, correlation_holder_DR3);
  float max = 0.0;
  int index = 0;
  for(uint i=start_point; i<correlation_holder_DR3.size();i++) {
    if(correlation_holder_DR3[i] > max) {
      max = correlation_holder_DR3[i];
      index = i-start_point;
    }
  }

  return std::tuple<int, int>(int(periodicity), index);
  }
  else if (dr == 4) {
    CorrelationDR4(O, O, correlation_holder_DR4);

  float periodicity = 0.0;
  int start_point = correlation_holder_DR4.size() / 2;

  //choose the max value found
  float max_autoc = 0.0;

  for(uint i=start_point+2; i<correlation_holder_DR4.size();i++){
    if(correlation_holder_DR4[i] + correlation_holder_DR4[i-1] > max_autoc) {
      max_autoc = correlation_holder_DR4[i] + correlation_holder_DR4[i-1];
      if(correlation_holder_DR4[i] > correlation_holder_DR4[i-1]) {
          periodicity = i - start_point;
      } else {
          periodicity = i - 1 -  start_point;  
      }
    }
  }

  if(periodicity == 0) {
    return std::tuple<int, int>(1, 0);
  }
  /*if(periodicity == 2777) {
    periodicity++;
  }*/

  //std::cout << "periodicity DR4 " << periodicity << std::endl;

  // generate a sequence with that periodicity with the offset 0
  std::vector<unsigned char> O_copy2(O.size(), 0);
  for(uint i=0;i<O_copy2.size();i+=periodicity) {
    O_copy2[i] = 1;
  }
  

  //get the correlation of that sequence with the original sequence, and find the angle between them 
  CorrelationDR4(O, O_copy2, correlation_holder_DR4);
  float max = 0.0;
  int index = 0;
  for(uint i=start_point; i<correlation_holder_DR4.size();i++) {
    if(correlation_holder_DR4[i] > max) {
      max = correlation_holder_DR4[i];
      index = i-start_point;
    }
  }

  return std::tuple<int, int>(int(periodicity), index);
  }
  else /*if(dr == 5) */
  {
    CorrelationDR5(O, O, correlation_holder_DR5);

  float periodicity = 0.0;
  int start_point = correlation_holder_DR5.size() / 2;

  //choose the max value found
  float max_autoc = 0.0;
  for(uint i=start_point+2; i<correlation_holder_DR5.size();i++){
    if(correlation_holder_DR5[i] + correlation_holder_DR5[i-1] > max_autoc) {
      max_autoc = correlation_holder_DR5[i] + correlation_holder_DR5[i-1];
      if(correlation_holder_DR5[i] > correlation_holder_DR5[i-1]) {
          periodicity = i - start_point;
      } else {
          periodicity = i - 1 -  start_point;  
      }
    }
  }

  if(periodicity == 0) {
    return std::tuple<int, int>(1, 0);
  }

  /*if(periodicity == 5084) { //these must be changed
    periodicity++;
  }*/
  //std::cout << "periodicity DR5 " << periodicity << std::endl;

  // generate a sequence with that periodicity with the offset 0
  std::vector<unsigned char> O_copy2(O.size(), 0);
  for(uint i=0;i<O_copy2.size();i+=periodicity) {
    O_copy2[i] = 1;
  }
  

  //get the correlation of that sequence with the original sequence, and find the angle between them 
  CorrelationDR5(O, O_copy2, correlation_holder_DR5);
  float max = 0.0;
  int index = 0;
  for(uint i=start_point; i<correlation_holder_DR5.size();i++) {
    if(correlation_holder_DR5[i] > max) {
      max = correlation_holder_DR5[i];
      index = i-start_point;
    }
  }

  return std::tuple<int, int>(int(periodicity), index);
  }
  //autocorrelate the sequence to find a candidate periodicity
  /*Correlation(O, O, correlation_holder);

  float periodicity = 0.0;
  int start_point = correlation_holder.size() / 2;

  //choose the max value found
  float max_autoc = 0.0;
  for(uint i=start_point+1; i<correlation_holder.size();i++){
    if(correlation_holder[i] > max_autoc) {

      max_autoc = correlation_holder[i];
      periodicity = i - start_point;
    }
  }

  if(periodicity == 0) {
    return std::tuple<int, int>(1, 0);
  }

  // generate a sequence with that periodicity with the offset 0
  std::vector<unsigned char> O_copy2(O.size(), 0);
  for(uint i=0;i<O_copy2.size();i+=periodicity) {
    O_copy2[i] = 1;
  }
  

  //get the correlation of that sequence with the original sequence, and find the angle between them 
  Correlation(O, O_copy2, correlation_holder);
  float max = 0.0;
  int index = 0;
  for(uint i=start_point; i<correlation_holder.size();i++) {
    if(correlation_holder[i] > max) {
      max = correlation_holder[i];
      index = i-start_point;
    }
  }

  return std::tuple<int, int>(int(periodicity), index);*/
}

/*void LightweightTimeslots::Correlation(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z)
{
  std::chrono::high_resolution_clock::time_point start, before, now, finish; 
  std::chrono::duration<double> elapsed;
  if(DEBUG) {
    start = std::chrono::high_resolution_clock::now();
    before = std::chrono::high_resolution_clock::now(); 
  }

  //put input data in to in inX and inY. Put y in backwards as this is correlation, not convolution
  
  int ysize = y.size();
  for(int i=0;i<ysize;i++) {
    inX[i][0] = x[i];
  }


  for(int i=0;i<ysize;i++) {
    inY[i][0] = y[ysize - 1 - i];
  }

  if(DEBUG) {
    now = std::chrono::high_resolution_clock::now();
    elapsed = now - before;
    std::cout << "Time to fill in x and y: " << elapsed.count() << " s\n";
    before = std::chrono::high_resolution_clock::now();
  }
  

  fftw_execute(pX);

  if(DEBUG) {
    now = std::chrono::high_resolution_clock::now();
    elapsed = now - before;
    std::cout << "Time to execute fft x: " << elapsed.count() << " s\n";
    before = std::chrono::high_resolution_clock::now();
  }
  

  fftw_execute(pY);

  if(DEBUG) {
    now = std::chrono::high_resolution_clock::now();
    elapsed = now - before;
    std::cout << "Time to execute fft y: " << elapsed.count() << " s\n";
    before = std::chrono::high_resolution_clock::now();
  }

  //perform element-wise multiplication of x and y (multiplying the complex numbers)
  //(x + yi)(u + vi) = (xu - yv) + (xv + yu)i
  for(uint i=0;i<x.size()*3 - 1;i++) {
    inZ[i][0] = (outX[i][0] * outY[i][0]) - (outX[i][1] * outY[i][1]);
    inZ[i][1] = (outX[i][0] * outY[i][1]) + (outX[i][1] * outY[i][0]);    
  }

  if(DEBUG) {
    now = std::chrono::high_resolution_clock::now();
    elapsed = now - before;
    std::cout << "Time to fill in inZ (complex calculations): " << elapsed.count() << " s\n";
    before = std::chrono::high_resolution_clock::now();
  }

  //get ifft of result
  fftw_execute(pZ);

  if(DEBUG) {
    now = std::chrono::high_resolution_clock::now();
    elapsed = now - before;
    std::cout << "Time to execute fft z: " << elapsed.count() << " s\n";
    before = std::chrono::high_resolution_clock::now();
  }

  for(uint i=0;i<z.size();i++) {
    z[i] = std::abs(std::complex<double>(outZ[i][0], outZ[i][1])) / (x.size()*3 - 1);
  }

  if(DEBUG) {
    now = std::chrono::high_resolution_clock::now();
    elapsed = now - before;
    std::cout << "Time to move z data out: " << elapsed.count() << " s\n";
    before = std::chrono::high_resolution_clock::now();

    finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double>  elapsed = finish - start;
    std::cout << "Elapsed time for correlation: " << elapsed.count() << " s\n\n";
  }
}*/

void LightweightTimeslots::CorrelationDR0(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z)
{
  //put input data in to in inX and inY. Put y in backwards as this is correlation, not convolution
  int ysize = y.size();
  for(int i=0;i<ysize;i++) {
    inX_DR0[i][0] = x[i];
  }

  for(int i=0;i<ysize;i++) {
    inY_DR0[i][0] = y[ysize - 1 - i];
  }

  fftw_execute(pX_DR0);
  fftw_execute(pY_DR0);

  //perform element-wise multiplication of x and y (multiplying the complex numbers)
  //(x + yi)(u + vi) = (xu - yv) + (xv + yu)i
  for(uint i=0;i<x.size()*3 - 1;i++) {
    inZ_DR0[i][0] = (outX_DR0[i][0] * outY_DR0[i][0]) - (outX_DR0[i][1] * outY_DR0[i][1]);
    inZ_DR0[i][1] = (outX_DR0[i][0] * outY_DR0[i][1]) + (outX_DR0[i][1] * outY_DR0[i][0]);    
  }

  //get ifft of result
  fftw_execute(pZ_DR0);

  for(uint i=0;i<z.size();i++) {
    z[i] = std::abs(std::complex<double>(outZ_DR0[i][0], outZ_DR0[i][1])) / (x.size()*3 - 1);
  }
}

void LightweightTimeslots::CorrelationDR1(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z)
{
  //put input data in to in inX and inY. Put y in backwards as this is correlation, not convolution
  int ysize = y.size();
  for(int i=0;i<ysize;i++) {
    inX_DR1[i][0] = x[i];
  }

  for(int i=0;i<ysize;i++) {
    inY_DR1[i][0] = y[ysize - 1 - i];
  }

  fftw_execute(pX_DR1);
  fftw_execute(pY_DR1);

  //perform element-wise multiplication of x and y (multiplying the complex numbers)
  //(x + yi)(u + vi) = (xu - yv) + (xv + yu)i
  for(uint i=0;i<x.size()*3 - 1;i++) {
    inZ_DR1[i][0] = (outX_DR1[i][0] * outY_DR1[i][0]) - (outX_DR1[i][1] * outY_DR1[i][1]);
    inZ_DR1[i][1] = (outX_DR1[i][0] * outY_DR1[i][1]) + (outX_DR1[i][1] * outY_DR1[i][0]);    
  }

  //get ifft of result
  fftw_execute(pZ_DR1);

  for(uint i=0;i<z.size();i++) {
    z[i] = std::abs(std::complex<double>(outZ_DR1[i][0], outZ_DR1[i][1])) / (x.size()*3 - 1);
  }
}

void LightweightTimeslots::CorrelationDR2(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z)
{
  //put input data in to in inX and inY. Put y in backwards as this is correlation, not convolution
  int ysize = y.size();
  for(int i=0;i<ysize;i++) {
    inX_DR2[i][0] = x[i];
  }

  for(int i=0;i<ysize;i++) {
    inY_DR2[i][0] = y[ysize - 1 - i];
  }

  fftw_execute(pX_DR2);
  fftw_execute(pY_DR2);

  //perform element-wise multiplication of x and y (multiplying the complex numbers)
  //(x + yi)(u + vi) = (xu - yv) + (xv + yu)i
  for(uint i=0;i<x.size()*3 - 1;i++) {
    inZ_DR2[i][0] = (outX_DR2[i][0] * outY_DR2[i][0]) - (outX_DR2[i][1] * outY_DR2[i][1]);
    inZ_DR2[i][1] = (outX_DR2[i][0] * outY_DR2[i][1]) + (outX_DR2[i][1] * outY_DR2[i][0]);    
  }

  //get ifft of result
  fftw_execute(pZ_DR2);

  for(uint i=0;i<z.size();i++) {
    z[i] = std::abs(std::complex<double>(outZ_DR2[i][0], outZ_DR2[i][1])) / (x.size()*3 - 1);
  }
}

void LightweightTimeslots::CorrelationDR3(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z)
{
  //put input data in to in inX and inY. Put y in backwards as this is correlation, not convolution
  int ysize = y.size();
  for(int i=0;i<ysize;i++) {
    inX_DR3[i][0] = x[i];
  }

  for(int i=0;i<ysize;i++) {
    inY_DR3[i][0] = y[ysize - 1 - i];
  }

  fftw_execute(pX_DR3);
  fftw_execute(pY_DR3);

  //perform element-wise multiplication of x and y (multiplying the complex numbers)
  //(x + yi)(u + vi) = (xu - yv) + (xv + yu)i
  for(uint i=0;i<x.size()*3 - 1;i++) {
    inZ_DR3[i][0] = (outX_DR3[i][0] * outY_DR3[i][0]) - (outX_DR3[i][1] * outY_DR3[i][1]);
    inZ_DR3[i][1] = (outX_DR3[i][0] * outY_DR3[i][1]) + (outX_DR3[i][1] * outY_DR3[i][0]);    
  }

  //get ifft of result
  fftw_execute(pZ_DR3);

  for(uint i=0;i<z.size();i++) {
    z[i] = std::abs(std::complex<double>(outZ_DR3[i][0], outZ_DR3[i][1])) / (x.size()*3 - 1);
  }
}

void LightweightTimeslots::CorrelationDR4(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z)
{
  //put input data in to in inX and inY. Put y in backwards as this is correlation, not convolution
  int ysize = y.size();
  for(int i=0;i<ysize;i++) {
    inX_DR4[i][0] = x[i];
  }

  for(int i=0;i<ysize;i++) {
    inY_DR4[i][0] = y[ysize - 1 - i];
  }

  fftw_execute(pX_DR4);
  fftw_execute(pY_DR4);

  //perform element-wise multiplication of x and y (multiplying the complex numbers)
  //(x + yi)(u + vi) = (xu - yv) + (xv + yu)i
  for(uint i=0;i<x.size()*3 - 1;i++) {
    inZ_DR4[i][0] = (outX_DR4[i][0] * outY_DR4[i][0]) - (outX_DR4[i][1] * outY_DR4[i][1]);
    inZ_DR4[i][1] = (outX_DR4[i][0] * outY_DR4[i][1]) + (outX_DR4[i][1] * outY_DR4[i][0]);    
  }

  //get ifft of result
  fftw_execute(pZ_DR4);

  for(uint i=0;i<z.size();i++) {
    z[i] = std::abs(std::complex<double>(outZ_DR4[i][0], outZ_DR4[i][1])) / (x.size()*3 - 1);
  }
}


void LightweightTimeslots::CorrelationDR5(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z)
{
  //put input data in to in inX and inY. Put y in backwards as this is correlation, not convolution
  int ysize = y.size();
  for(int i=0;i<ysize;i++) {
    inX_DR5[i][0] = x[i];
  }

  for(int i=0;i<ysize;i++) {
    inY_DR5[i][0] = y[ysize - 1 - i];
  }

  fftw_execute(pX_DR5);
  fftw_execute(pY_DR5);

  //perform element-wise multiplication of x and y (multiplying the complex numbers)
  //(x + yi)(u + vi) = (xu - yv) + (xv + yu)i
  for(uint i=0;i<x.size()*3 - 1;i++) {
    inZ_DR5[i][0] = (outX_DR5[i][0] * outY_DR5[i][0]) - (outX_DR5[i][1] * outY_DR5[i][1]);
    inZ_DR5[i][1] = (outX_DR5[i][0] * outY_DR5[i][1]) + (outX_DR5[i][1] * outY_DR5[i][0]);    
  }

  //get ifft of result
  fftw_execute(pZ_DR5);

  for(uint i=0;i<z.size();i++) {
    z[i] = std::abs(std::complex<double>(outZ_DR5[i][0], outZ_DR5[i][1])) / (x.size()*3 - 1);
  }
}

int LightweightTimeslots::Main(int argc, const char** argv) {
  LightweightTimeslots* lightweightTimeslots = new LightweightTimeslots();

  try {
    lightweightTimeslots->Run();
  } catch(std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  delete lightweightTimeslots;

  return 0;
}

void LightweightTimeslots::CollisionAvoidance(std::vector<Periodicity>& periodicities, float alpha, uint8_t dataRateIndex) {
  //elements are in the format (p, o, uid, change)


  //sort by p, and then sort the sublists by o
  //full_list = sorted(full_list, key=itemgetter(0, 1))
  sort(periodicities.begin(), periodicities.end(), sortByPthenO);

  /*
    //move one sequence if it overlaps with another by more than alpha percent
    for i, x in enumerate(full_list):
      for j, y in enumerate(full_list[i+1:]):
        o = overlap(x[0], y[0], x[1], y[1])
        if o > alpha:
          y[1] += 1
          y[3] += 1
          //TODO: ensure y[3] doesn't exceed MAX_C
          //well, it can temporarilly if it gets fixed later.
  */

  /*for(uint i=0; i<periodicities.size(); i++) {
    std::cout << periodicities[i].p << " " << periodicities[i].o << " " << periodicities[i].change << " " << (int) periodicities[i].changeThisRound << " " <<  periodicities[i].uID << std::endl;
    
  }*/


  for(uint p=0; p<4; p++) {
    float acceptableNewOverlap = 0.0;
    if(p==0) {
      acceptableNewOverlap = 0.0;
    } else if (p==1) {
      acceptableNewOverlap = 0.25;
    } else if (p==2) {
      acceptableNewOverlap = 0.75;
    } else if (p==3) {
      acceptableNewOverlap = 1000;
    }
    //std::cout << "round " << p << std::endl;

    uint32_t DR_size = m_timeSlotsPerDataRate[dataRateIndex].m_slots;
  std::vector<unsigned int> period_counter = std::vector<unsigned int>(DR_size, 0);
  for(uint i=0; i<periodicities.size(); i++) {
      for(int l=0;l<=(DR_size / periodicities[i].p);l++) {
          if((l*periodicities[i].p)+periodicities[i].o < period_counter.size()) {
              period_counter[(l*periodicities[i].p)+periodicities[i].o]++;
          }
        
      }
  }
  /*std::cout << "start of counter (before)" << std::endl;
  for(uint i=0; i<DR_size / 6;i++) {
    std::cout << period_counter[i] << " ";
  }
  std::cout << std::endl;
  std::cout << "end of counter" << std::endl;
*/
  for(uint i=0; i<periodicities.size(); i++) {
      float overlapCount = 0;
      int numSlots = 0;
      for(int l=0;l<=(DR_size / periodicities[i].p);l++) { //count the number of transmissions that overlapped with this device in this period
          if((l*periodicities[i].p)+periodicities[i].o < period_counter.size()) {
              overlapCount += period_counter[(l*periodicities[i].p)+periodicities[i].o] - 1; // -1 to discount it's own transmission in this slot
              numSlots++;
          } 
        
      }   
      /*std::cout << "overlapCount before division " << overlapCount << " and numSlots " << numSlots << std::endl;
      if(numSlots == 0) {
        std::cout << "numSlots is 0, here is the po: " << periodicities[i].p << " " << periodicities[i].o << std::endl;
      } */
      overlapCount /= numSlots; //get an average count per transmission
      //std::cout << "overlapCount for device " << periodicities[i].uID << " is " << overlapCount << std::endl;

      if(overlapCount >= 1) { //if there was on average 1 or more overlapping transmissions for this device using this offset 
          //check if there is a better potential spot
          float bestOverlap = overlapCount;
          int bestSlot = periodicities[i].o;
          for(int m=periodicities[i].o - periodicities[i].change; m <= periodicities[i].o - periodicities[i].change + m_maxTimeSlotPushPerDataRate[dataRateIndex]; m++) {
              //std::cout << m << " ";
              if(m != periodicities[i].o) {
                  float currOverlap = 0;
                  int numSlotM = 0;
                  for(int n=0;n<(DR_size / periodicities[i].p) ;n++) {
                      if((n*periodicities[i].p)+m < period_counter.size()) {
                          currOverlap += period_counter[(n*periodicities[i].p)+m];
                          numSlotM++;  
                      } 
                  }
                  currOverlap /= numSlotM;

                  if(currOverlap < bestOverlap) {
                    bestOverlap = currOverlap;
                    bestSlot = m;
                  }
                  if(bestOverlap == 0.0) {
                    break; //we've found an offset that won't collide with anyone, definitely use it.
                  }
              }
          }
          //std::cout << std::endl;

          if(bestOverlap != overlapCount && bestOverlap <= acceptableNewOverlap) { //we're going to move this device to a better slot
              //std::cout << "old overlap: " << overlapCount  << " best overlap: " << bestOverlap << std::endl;
              //std::cout << "changing from " << periodicities[i].o << " to " << bestSlot << std::endl; 
            //printf("change we're going to use: from: %u to %u for device %u\r\n", periodicities[i].o, bestSlot, periodicities[i].uID);
            //std::cout << "change we're going to use: from: " << periodicities[i].o << " to " << bestSlot << std::endl; 
              //modify the period_counter to reflect this

            //TODO: check if, based on the changes to the other periodicities of this device, this will overall improve the system or not. If not, don't do it.

              for(int l=0;l<(DR_size / periodicities[i].p);l++) {
                  if((l*periodicities[i].p)+periodicities[i].o < period_counter.size()) {
                      period_counter[(l*periodicities[i].p)+periodicities[i].o]--;  
                  }
                  if((l*periodicities[i].p)+bestSlot < period_counter.size()) {
                      period_counter[(l*periodicities[i].p)+bestSlot]++;  
                  }
                    
              }
               

              if(bestSlot > periodicities[i].o) { //then make the change
                  periodicities[i].changeThisRound += bestSlot - periodicities[i].o;
                  periodicities[i].change += bestSlot - periodicities[i].o;
                  periodicities[i].o = bestSlot;
              } else {
                  //std::cout << "j before (o>=B) " << periodicities[j].o << " " << periodicities[j].change << " "  << (int) periodicities[j].changeThisRound << std::endl;
                  periodicities[i].changeThisRound += bestSlot - periodicities[i].o;
                  periodicities[i].change -= periodicities[i].o - bestSlot;
                  periodicities[i].o = bestSlot;
                  //std::cout << "j after (o>=B)" << periodicities[j].o << " " << periodicities[j].change << " "  << (int) periodicities[j].changeThisRound << std::endl;
              }

              //then also modify all of the device's other periodicities
              //TODO: removing possibility of other periodicities for now
              /*for(uint k=0; k<periodicities.size(); k++) {
                  
                  if(periodicities[i].uID == periodicities[k].uID && i!=k) {
                      for(int l=0;l<(DR_size / periodicities[k].p);l++) {
                          if((l*periodicities[k].p)+periodicities[k].o < period_counter.size()) {
                              period_counter[(l*periodicities[k].p)+periodicities[k].o]--;  
                          }
                          if((l*periodicities[k].p) + (periodicities[k].o + periodicities[i].changeThisRound) < period_counter.size()) {
                              period_counter[(l*periodicities[k].p) + (periodicities[k].o + periodicities[i].changeThisRound)]++;  
                          }
                      }
                      std::cout << "the device is " << periodicities[k].uID << std::endl;
                      std::cout << "i that has been changed: " << periodicities[i].o << " " << periodicities[i].change << " "  << (int) periodicities[i].changeThisRound << std::endl;
                      std::cout << "k is going to change: " << periodicities[k].o << " " << periodicities[k].change << " "  << (int) periodicities[k].changeThisRound << std::endl;
                      //TODO: this is definitely wrong somehow.
                      if(periodicities[i].changeThisRound > 0) {
                          periodicities[k].changeThisRound += periodicities[i].changeThisRound;
                          periodicities[k].change += periodicities[i].changeThisRound;
                          periodicities[k].o += periodicities[i].changeThisRound;  
                      } else {
                          periodicities[k].changeThisRound += periodicities[i].changeThisRound;
                          periodicities[k].change -= periodicities[i].changeThisRound; //TODO: double check this works. Comparing signed and unsigned.
                          periodicities[k].o -= periodicities[i].changeThisRound;
                      }
                      std::cout << "the changed k: " << periodicities[k].o << " " << periodicities[k].change << " "  << (int) periodicities[k].changeThisRound << std::endl;
                  }
              }*/
              //std::cout << "there was a significant overlap for this periodicity, so we've chosen a better slot. Old: " << overlapCount << ". And best found: " << bestOverlap << std::endl;
          } else {
            //std::cout << "there is a significant overlap for this periodicity, but there is no better slot to place the device in. Old: " << overlapCount << ". And best found: " << bestOverlap << std::endl;
          }  
      } else {
        //std::cout << "no significant overlap for this periodicity: " << overlapCount << std::endl;
      }     
  }
    //if(p==3){
    //std::cout << "start of counter (after)" << std::endl;
  //for(uint i=0; i<DR_size / 6;i++) {
  //  std::cout << period_counter[i] << " ";
 // }
  //std::cout << std::endl;
  //std::cout << "end of counter" << std::endl;  
	//}
  }

  

  /*std::cout << "start of counter (2)" << std::endl;
  for(uint i=0; i<period_counter.size();i++) {
    std::cout << i << " " << period_counter[i] << std::endl;
  }
  std::cout << "end of counter (2)" << std::endl;*/


  /*uint32_t DR_size = m_timeSlotsPerDataRate[dataRateIndex].m_slots;
  std::vector<unsigned char> period_counter = std::vector<unsigned char>(DR_size, 0); 
  for(uint i=0; i<periodicities.size(); i++) {
    //TODO: should this add all of the periodicities of this device?
    //std::cout << "adding to period_counter " << periodicities[i].p << " " << periodicities[i].o << std::endl;
    for(int l=0;l<(DR_size / periodicities[i].p);l++) {
      period_counter[(l*periodicities[i].p)+periodicities[i].o] = 1;
      //std::cout << "putting a 1 at " << (l*periodicities[i].p)+periodicities[i].o << std::endl;
    }

    for(uint j=i+1; j<periodicities.size(); j++) {
      int o = CalculatePercentageOverlap(periodicities[i].p, periodicities[j].p, periodicities[i].o, periodicities[j].o);

      if(o > alpha && periodicities[j].change < m_maxTimeSlotPushPerDataRate[dataRateIndex]) {
        //std::cout << "overlap between (" << periodicities[i].p << ", " << periodicities[i].o << ") and ( " <<  periodicities[j].p << ", " << periodicities[j].o << ")" << "which are devices " << periodicities[i].uID << " and " << periodicities[j].uID << std::endl;  
        
        //this may possibly be a load of shite
        //we need to find a new value of o for j.
        //using the period_counter, do a local search for the best fit
        uint8_t best_m = 0;
        uint8_t best_m_val = 255; 
        for(int m=periodicities[j].o - periodicities[j].change; m < periodicities[j].o - periodicities[j].change + m_maxTimeSlotPushPerDataRate[dataRateIndex]; m++) {
            uint8_t curr_m_val = 0;
            for(int n=0;n<(DR_size / periodicities[j].p) ;n++) {
              if(period_counter[(n*periodicities[j].p)+m] != 0) {
                curr_m_val++;
                //std::cout << "there's a 1 at " << (n*periodicities[j].p)+m << std::endl;
              }
            }
            if(curr_m_val < best_m_val) {
              best_m_val = curr_m_val;
              best_m = m;
              //printf("new best m %u %u\r\n", best_m, best_m_val);
            }
            if(best_m_val == 0){
              break;
            }
        }

        if(best_m_val < DR_size / periodicities[j].p) {
            if(best_m > periodicities[j].o) {
            //std::cout << "j before (B>o) " << periodicities[j].o << " " << periodicities[j].change << " "  << (int) periodicities[j].changeThisRound << std::endl;
            periodicities[j].changeThisRound += best_m - periodicities[j].o;
            periodicities[j].change += best_m - periodicities[j].o;
            periodicities[j].o = best_m;
            //std::cout << "j after (B>o)" << periodicities[j].o << " " << periodicities[j].change << " "  << (int) periodicities[j].changeThisRound << std::endl;
            } else {
              //std::cout << "j before (o>=B) " << periodicities[j].o << " " << periodicities[j].change << " "  << (int) periodicities[j].changeThisRound << std::endl;
              periodicities[j].changeThisRound += best_m - periodicities[j].o;
              periodicities[j].change -= periodicities[j].o - best_m;
              periodicities[j].o = best_m;
              //std::cout << "j after (o>=B)" << periodicities[j].o << " " << periodicities[j].change << " "  << (int) periodicities[j].changeThisRound << std::endl;
            }

        //old version
        //periodicities[j].o++;
        //periodicities[j].change++;
        //periodicities[j].changeThisRound++;

        //TODO: as long as change remains >= 0, the changeThisRound push can be negative as well as positive.

        for(uint k=0; k<periodicities.size(); k++) {
            //we have to move all the other periodicities of this device too
            //TODO: but this can also cause a new overlap? 
            if(periodicities[j].uID == periodicities[k].uID && j!=k) {
                //TODO: what if
                  //if(best_m > periodicities[j].o) { //this will always be 0...
                    //std::cout << "k before " << periodicities[k].o << " " << periodicities[k].change << " "  << periodicities[k].changeThisRound << std::endl;
                    periodicities[k].changeThisRound += periodicities[j].changeThisRound;
                    if(periodicities[j].changeThisRound > 0) {
                        periodicities[k].change += periodicities[j].changeThisRound;
                        periodicities[k].o += periodicities[j].changeThisRound;  
                    } else {
                        periodicities[k].change -= periodicities[j].changeThisRound; //TODO: double check this works. Comparing signed and unsigned.
                        periodicities[k].o -= periodicities[j].changeThisRound;
                    }
                    //std::cout << "k after" << periodicities[k].o << " " << periodicities[k].change << " "  << periodicities[k].changeThisRound << std::endl;
                  //}/ else {
                    //std::cout << "k before (o>=B) " << periodicities[k].o << " " << periodicities[k].change << " "  << periodicities[k].changeThisRound << std::endl;
                    //std::cout << "best_m: " << best_m << std::endl;
                    //periodicities[k].changeThisRound += periodicities[j].changeThisRound;
                    //p/eriodicities[k].change -= periodicities[j].changeThisRound;
                    //periodicities[k].o -= periodicities[j].o - best_m;
                    //std::cout << "k after (o>=B)" << periodicities[k].o << " " << periodicities[k].change << " "  << periodicities[k].changeThisRound << std::endl;
                    
                  //}

                  //old version
                  //periodicities[k].o++;
                  //periodicities[k].change++;
                  //periodicities[k].changeThisRound++;
                  //std::cout << "multi periods for device "  << periodicities[k].uID << std::endl;
            }
        }
        } else {
          std::cout << "no improvement possible." << std::endl;
        }

        
        //TODO: ensure periodicities[j].change doesn't exceed MAX_C
        //(though it can here temporarily if it gets fixed later)
      }
    }
  }*/


  /*
    //reorder to minimise number of changes
    for i, x in enumerate(full_list):
      for j, y in enumerate(full_list[i+1:]):
        if x[0] == y[0] and x[1] + y[3] == y[1] and x[3] + y[3] <= MAX_C:
          x[1] += y[3]
          x[3] += y[3]
          y[1] -= y[3]
          y[3] -= y[3]
  */

  uint c = 0;
  for(uint i=0; i<periodicities.size(); i++) {
    //std::cout << periodicities[i].p << " " << periodicities[i].o << " " << periodicities[i].change << " " << (int) periodicities[i].changeThisRound << " " <<  periodicities[i].uID << std::endl;
    if(periodicities[i].changeThisRound != 0) {
      c++;
    }
  }
  //printf("dr %u, ts c1: %u \r\n", dataRateIndex, c);
  c = 0;
  //std::cout << "now reorder" << std::endl;

  //we want to minimise the number of p's with changeThisRound > 0
  //so for all the p's with changeThisRound > 0, we check if any of them can move their offsets around to make one of the pair have a changeThisRound of 0.
  for(uint i=0; i<periodicities.size(); i++) {
    for(uint j=i+1; j<periodicities.size(); j++) {
        if(periodicities[i].p == periodicities[j].p 
          && periodicities[i].o + periodicities[j].changeThisRound == periodicities[j].o
          && periodicities[i].change + periodicities[j].changeThisRound  <= m_maxTimeSlotPushPerDataRate[dataRateIndex]
          && periodicities[j].changeThisRound != 0 && periodicities[i].changeThisRound != 0) {

          //std::cout << "1. increase for device (before) " << periodicities[i].uID << " " << periodicities[i].o << " " << periodicities[i].change << " " << periodicities[i].changeThisRound << " " << std::endl;

          periodicities[i].o += periodicities[j].changeThisRound;
          periodicities[i].change += periodicities[j].changeThisRound;
          periodicities[i].changeThisRound += periodicities[j].changeThisRound;

          //std::cout << "2. increase for device (after) " << periodicities[i].uID << " " << periodicities[i].o << " " << periodicities[i].change << " " << periodicities[i].changeThisRound << " " << std::endl;

          for(uint k=0; k<periodicities.size(); k++) { 
            if(periodicities[k].uID == periodicities[i].uID && k!=i) {
              periodicities[k].o += periodicities[j].changeThisRound; 
              periodicities[k].change += periodicities[j].changeThisRound;
              periodicities[k].changeThisRound += periodicities[j].changeThisRound;
              //std::cout << "o and c of same device: " << periodicities[i].o << " " << periodicities[i].change << " " << periodicities[k].o << " " << periodicities[k].change << std::endl;  
            }
          }
          //std::cout << "3. decrease for device (before) " << periodicities[j].uID << " " << periodicities[j].o << " " << periodicities[j].change << " " << periodicities[j].changeThisRound << " " << std::endl;
          periodicities[j].o -= periodicities[j].changeThisRound;
          periodicities[j].change -= periodicities[j].changeThisRound;
          periodicities[j].changeThisRound -= periodicities[j].changeThisRound;
          //std::cout << "4. decrease for device (after) " << periodicities[j].uID << " " << periodicities[j].o << " " << periodicities[j].change << " " << periodicities[j].changeThisRound << " " << std::endl;

        } 
    }
  }

  for(uint i=0; i<periodicities.size(); i++) {

    //std::cout << periodicities[i].p << " " << periodicities[i].o << " " << periodicities[i].change <<  " " << (int) periodicities[i].changeThisRound << " " << periodicities[i].uID << std::endl;
    if(periodicities[i].changeThisRound != 0) {
      c++;
      //std::cout << periodicities[i].p << " " << periodicities[i].o << " " << periodicities[i].change <<  " " << (int) periodicities[i].changeThisRound << " " << periodicities[i].uID << std::endl;
    }
    /*if(periodicities[i].uID == 88) {
        std::cout << periodicities[i].p << " " << periodicities[i].o << " " << periodicities[i].change <<  " " << (int) periodicities[i].changeThisRound << " " << periodicities[i].uID << std::endl;
    }*/
  }
  //printf("dr %u, ts c2: %u \r\n", dataRateIndex, c);
  c = 0;

}

int LightweightTimeslots::lcm(int a, int b) {
  return a * b / std::__gcd(a, b);
}

float LightweightTimeslots::CalculatePercentageOverlap(int p1, int p2, int o1, int o2) {
  /*o = abs(o1 - o2)
  d = math.gcd(p1, p2)
  if o % d == 0:
    # there will be a shared member of the set
    # calculate the percentage overlap:
    return p2 / lcm(p1, p2)
  else:
    theres no overlap
    return 0.0*/

  int o = std::abs(o1 - o2);
  int d = std::__gcd(p1, p2);

  if(o % d == 0) {
    // there will be a shared member of the set
    // calculate the percentage overlap
    return p2 / lcm(p1, p2);
  } else {
    // there's no overlap
    return 0.0;
  }
}

bool LightweightTimeslots::sortByPthenO(Periodicity p1, Periodicity p2) {
  if(p1.p < p2.p) {
    return true;
  } else if (p1.p > p2.p) {
    return false;
  } else { // if p1 == p2
    return (p1.o < p2.o);
  }
}

} // namespace ns3
