 
#include "Gait/WaveGenerator.h"
#include <iostream>
#include <sys/time.h>
#include <math.h>

WaveGenerator::WaveGenerator(double period, double stancePhaseRatio, Vec6 bias)
    : _period(period), _stRatio(stancePhaseRatio), _bias(bias)
{
    if ((_stRatio >= 1) || (_stRatio <= 0))
    {
        std::cout << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)" << std::endl;
        exit(-1);
    }

    for (int i(0); i < bias.rows(); ++i)
    {
        if ((bias(i) > 1) || (bias(i) < 0))
        {
            std::cout << "[ERROR] The bias of WaveGenerator should between [0, 1]" << std::endl;
            exit(-1);
        }
    }

    _startT = getSystemTime();
    _contactPast.setZero();
    _phasePast << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
    // _phasePast << 0, 0, 0, 0;
    // _phasePast << 1, 1, 1, 1;
    _statusPast = WaveStatus::SWING_ALL;

    _switchStatus.setZero();
    // std::cout<<" _switchStatus: \n"<< _switchStatus <<std::endl;
}

WaveGenerator::~WaveGenerator()
{
}

void WaveGenerator::calcContactPhase(Vec6 &phaseResult, VecInt6 &contactResult, WaveStatus status)
{
    calcWave(_phase, _contact, status);

    if (status != _statusPast)
    {
        if (_switchStatus.sum() == 0)
        {
            _switchStatus.setOnes();
        }
        calcWave(_phasePast, _contactPast, _statusPast);
        // two special case
        if ((status == WaveStatus::STANCE_ALL) && (_statusPast == WaveStatus::SWING_ALL))
        {
            _contactPast.setOnes();
        }
        else if ((status == WaveStatus::SWING_ALL) && (_statusPast == WaveStatus::STANCE_ALL))
        {
            _contactPast.setZero();
        }
    }

    if (_switchStatus.sum() != 0)
    {
        for (int i(0); i < 6; ++i)
        {
            if (_contact(i) == _contactPast(i))
            {
                _switchStatus(i) = 0;
            }
            else
            {
                _contact(i) = _contactPast(i);
                _phase(i) = _phasePast(i);
            }
        }
        if (_switchStatus.sum() == 0)
        {
            _statusPast = status;
        }
    }

    phaseResult = _phase;
    contactResult = _contact;
}

float WaveGenerator::getTstance()
{
    return _period * _stRatio;
}

float WaveGenerator::getTswing()
{
    return _period * (1 - _stRatio);
}

float WaveGenerator::getT()
{
    return _period;
}

void WaveGenerator::calcWave(Vec6 &phase, VecInt6 &contact, WaveStatus status)
{
    if (status == WaveStatus::WAVE_ALL)
    {
        _passT = (double)(getSystemTime() - _startT) * 1e-6;
        for (int i(0); i < 6; ++i)
        {
            //在C++中，fmod 是一个用于计算浮点数取余的函数，其原型为：
            //double fmod(double numer, double denom);
            //在C++中，`fmod` 是一个用于计算浮点数取余的函数，其原型为：
            //`fmod` 返回两个参数 `numer` 和 `denom` 的余数。例如，`fmod(5.5, 2.0)` 将返回 `1.5`，因为 `5.5` 除以 `2.0` 的余数是 `1.5`。
            //在下述代码中，`_normalT(i)` 被 `(_passT + _period - _period * _bias(i))` 除以 `_period`，
            //然后取余数，以确保 `_normalT(i)` 的值在 `0` 到 `_period` 之间。
            _normalT(i) = fmod(_passT + _period - _period * _bias(i), _period) / _period; 
            if (_normalT(i) < _stRatio) //stand
            {
                contact(i) = 1;
                phase(i) = _normalT(i) / _stRatio; // phase相位进度：属于[0,1]，不区分摆动进度和支撑进度，全在phase里。
            }
            else
            { //swing
                contact(i) = 0;
                phase(i) = (_normalT(i) - _stRatio) / (1 - _stRatio);
            }
        }
    }
    else if (status == WaveStatus::SWING_ALL)
    {
        contact.setZero();
        // phase << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
        phase << 0, 0, 0, 0, 0, 0;
        // phase << 1, 1, 1, 1;
    }
    else if (status == WaveStatus::STANCE_ALL)
    {
        contact.setOnes();
        // phase << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
        phase << 0, 0, 0, 0, 0, 0;
        // phase << 1, 1, 1, 1;
    }
    // std::cout<<" contact: \n"<< contact <<std::endl;
    // std::cout<<" phase: \n"<< phase <<std::endl;
    // std::cout<<" _passT: \n"<< _passT <<std::endl;
    // std::cout<<" _startT: \n"<< _startT <<std::endl;
}
