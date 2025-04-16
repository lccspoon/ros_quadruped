 
#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H

#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include "common/enumClass.h"
#include <unistd.h>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

/*generate linear wave, [0, 1]*/
class WaveGenerator{
public:
    WaveGenerator(double period, double stancePhaseRatio, Vec6 bias);
    ~WaveGenerator();
    void calcContactPhase(Vec6 &phaseResult, VecInt6 &contactResult, WaveStatus status, Vec6 cpg_phase, Vec6 cpg_contact);
    float getTstance();
    float getTswing();
    float getT();

    // lcc 20250415
    void GaitChange(double period, double stancePhaseRatio, Vec6 bias);
    double ret_period(void);
    double ret_stancePhaseRatio(void);
    Vec6 ret_bias(void);

private:
    void calcWave(Vec6 &phase, VecInt6 &contact, WaveStatus status, Vec6 cpg_phase, Vec6 cpg_contact);

    double _period;
    double _stRatio;
    Vec6 _bias;

    Vec6 _normalT;                   // [0, 1)
    Vec6 _phase, _phasePast;
    VecInt6 _contact, _contactPast;
    VecInt6 _switchStatus;          // 1: switching, 0: do not switch
    WaveStatus _statusPast;

    double _passT;                   // unit: second
    long long _startT;    // unit: us
#ifdef COMPILE_DEBUG
    PyPlot _testPlot;
#endif  // COMPILE_DEBUG

};

#endif  // WAVEGENERATOR_H