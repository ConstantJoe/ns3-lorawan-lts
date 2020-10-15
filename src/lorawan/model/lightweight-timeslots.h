// Copyright (c) 2013 Your Company. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIGHTWEIGHT_TIMESLOTS_H_
#define LIGHTWEIGHT_TIMESLOTS_H_

#include <tuple>
#include <vector>
#include <ns3/object.h>

#include <fftw3.h>

namespace ns3 {

struct Periodicity {
	int p, o; 
  uint32_t uID;
  uint8_t change;
  int8_t changeThisRound;
};

typedef struct
  {
    uint8_t m_drIndex;
    uint32_t m_slots; // in Hz
  } LoRaWANTimeSlotsPerDataRate;
  
class LightweightTimeslots : public Object{
public:

  static TypeId GetTypeId (void);

  LightweightTimeslots();
  ~LightweightTimeslots();

  float TiComScore(std::tuple <int, int> s, std::vector<unsigned char>& Ps_star, std::vector<unsigned char>& O, float alpha, float abs_T, float abs_F);

  //std::vector<std::tuple <int, int>> TiComWithFFT(std::vector<unsigned char> O, float alpha);

  std::vector<std::tuple <int, int>> TiComWithAutocorrelation(std::vector<unsigned char>& O, float alpha, uint8_t dr);

  //void Correlation(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z);

  void CorrelationDR0(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z);
  void CorrelationDR1(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z);
  void CorrelationDR2(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z);
  void CorrelationDR3(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z);
  void CorrelationDR4(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z);
  void CorrelationDR5(const std::vector<unsigned char>& x, const std::vector<unsigned char>& y, std::vector<float>& z);

  std::tuple<int, int> FindCandidateSolutionAutocorrelation(const std::vector<unsigned char>& O, uint8_t dr);

  //std::tuple<int, int> FindCandidateSolutionFFT(std::vector<unsigned char> O); 

  //std::tuple <int, int> ArgmaxOverList(std::vector<std::tuple <int, int>> S, std::vector<unsigned char> Ps_star, std::vector<unsigned char> O, float alpha, float abs_T, float abs_F);

  static int Main(int argc, const char** argv);

  void CollisionAvoidance(std::vector<Periodicity>& periodicities, float alpha, uint8_t dataRateIndex);

  int lcm(int a, int b);

  float CalculatePercentageOverlap(int p1, int p2, int o1, int o2); 

  static bool sortByPthenO(Periodicity p1, Periodicity p2);

  void Run();

  float alpha;
  float slot_size; //2.793 for DR0;
  float periodicity;
  float offset;
  int time_slots;
  std::vector<unsigned char> O;

  
  //DR0
  fftw_complex *inX_DR0, *outX_DR0;
  fftw_plan pX_DR0;

  fftw_complex *inY_DR0, *outY_DR0;
  fftw_plan pY_DR0;

  fftw_complex *inZ_DR0, *outZ_DR0;
  fftw_plan pZ_DR0;

  std::vector<float> correlation_holder_DR0;

  //DR1
  fftw_complex *inX_DR1, *outX_DR1;
  fftw_plan pX_DR1;

  fftw_complex *inY_DR1, *outY_DR1;
  fftw_plan pY_DR1;

  fftw_complex *inZ_DR1, *outZ_DR1;
  fftw_plan pZ_DR1;

  std::vector<float> correlation_holder_DR1;

  //DR2
  fftw_complex *inX_DR2, *outX_DR2;
  fftw_plan pX_DR2;

  fftw_complex *inY_DR2, *outY_DR2;
  fftw_plan pY_DR2;

  fftw_complex *inZ_DR2, *outZ_DR2;
  fftw_plan pZ_DR2;

  std::vector<float> correlation_holder_DR2;

  //DR3
  fftw_complex *inX_DR3, *outX_DR3;
  fftw_plan pX_DR3;

  fftw_complex *inY_DR3, *outY_DR3;
  fftw_plan pY_DR3;

  fftw_complex *inZ_DR3, *outZ_DR3;
  fftw_plan pZ_DR3;

  std::vector<float> correlation_holder_DR3;

  //DR4
  fftw_complex *inX_DR4, *outX_DR4;
  fftw_plan pX_DR4;

  fftw_complex *inY_DR4, *outY_DR4;
  fftw_plan pY_DR4;

  fftw_complex *inZ_DR4, *outZ_DR4;
  fftw_plan pZ_DR4;

  std::vector<float> correlation_holder_DR4;

  //DR5
  fftw_complex *inX_DR5, *outX_DR5;
  fftw_plan pX_DR5;

  fftw_complex *inY_DR5, *outY_DR5;
  fftw_plan pY_DR5;

  fftw_complex *inZ_DR5, *outZ_DR5;
  fftw_plan pZ_DR5;

  std::vector<float> correlation_holder_DR5;

  //for a 64 byte packet
  std::vector<uint8_t> m_maxTimeSlotPushPerDataRate = { //ensuring a max delay of 10s
 5 , //DR0, time to transmit 33 byte packet = 1.81268882175 note that these are slightly adjusted to fit a periodicity of 10 mins.
 11 , //DR1, = 0.90634441087s
 22, //..., = 0.45317220543s
 44, //..., = 0.22658610271
 88, //..., = 0.11329305135
 176, //..., = 0.05664652568
  //{6, }, //TODO: decide on use of DR6
};

static const std::vector<LoRaWANTimeSlotsPerDataRate> m_timeSlotsPerDataRate;

  //for a 33 byte packet:
 /* std::vector<uint8_t> m_maxTimeSlotPushPerDataRate = { //ensuring a max delay of 10s
 5 , //DR0, time to transmit 33 byte packet = 1.810432s
 10 , //DR1, = 0.987136s
 22, //..., = 0.452608s
 40, //..., = 0.246784s
 74, //..., = 0.133632s
 139, //..., = 0.071936s
  //{6, }, //TODO: decide on use of DR6
};*/

  static Ptr<LightweightTimeslots> m_lorawanLightweightTimeslotsPtr;
};

} // namespace ns3

#endif // LIGHTWEIGHT_TIMESLOTS_H_
