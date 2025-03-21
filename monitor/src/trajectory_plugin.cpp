#include "trajectory_plugin.h"

using namespace XBot;

bool TrajectoryPlugin::on_initialize()
{
    // create trj from the mandatory amplitude param
    std::map<std::string, double> amplitude;
    getParam("~amplitude", amplitude);

    // ctrl map
    std::map<std::string, ControlMode> ctrl_map;

    for(auto [jname, a] : amplitude)
    {
        Chirp trj;
        trj.amplitude = a;

        auto jptr = _robot->getDeviceInstance<Hal::JointBase>(jname);

        if(!jptr)
        {
            jerror("joint {} does not exist", jname);
            continue;
        }

        _trj_map[_robot->getDeviceInstance<Hal::JointBase>(jname)] = trj;

        // set ctrl mode to be acquired on start
        ctrl_map[jname] = ControlMode::Position();
    }

    // set optional parameters
    double freq_min = 0.1;
    double freq_max = 0.1;
    double period = 30.0;
    std::map<std::string, double> phase;

    getParam("~freq_min", freq_min);
    getParam("~freq_max", freq_max);
    getParam("~period", period);
    getParam("~phase_offset", phase);

    for(auto& [unused, trj] : _trj_map)
    {
        trj.freq_min_hz = freq_min;
        trj.freq_max_hz = freq_max;
        trj.period = period;
    }

    for(auto [jname, p] : phase)
    {

        auto jptr = _robot->getDeviceInstance<Hal::JointBase>(jname);

        if(!jptr)
        {
            jerror("joint {} does not exist", jname);
            continue;
        }

        _trj_map.at(jptr).phase_offset = p;
    }

    setDefaultControlMode(ctrl_map);

    return true;

}

void TrajectoryPlugin::on_start()
{
    for(auto& [j, t] : _trj_map)
    {
        j->sense();
        t.reset();
        t.initial_value = j->get_pos_ref();
    }
}

void TrajectoryPlugin::starting()
{
    start_completed();
}

void TrajectoryPlugin::run()
{
    for(auto& [j, t] : _trj_map)
    {
        double val = t.step(getPeriodSec());

        j->set_pos_ref(val);

        j->move();
    }
}

void TrajectoryPlugin::on_stop()
{
}

double TrajectoryPlugin::Chirp::step(double dt)
{
    double dt_scaling = 1.0;

    if(_time <= warmup_time)
    {
        double tau = _time/warmup_time;
        dt_scaling = ((6*tau - 15)*tau + 10)*tau*tau*tau;
    }

    _freq_hz = freq_min_hz + (1 - std::cos(2*M_PI/period*_time))/2.0*(freq_max_hz - freq_min_hz);

    _phase += _freq_hz*2*M_PI*dt_scaling*dt;

    _time += dt;

    return initial_value + amplitude*std::sin(dt_scaling*phase_offset + _phase);
}

void TrajectoryPlugin::Chirp::reset()
{
    _phase = 0;
    _time = 0;
}

XBOT2_REGISTER_PLUGIN(TrajectoryPlugin, trajectory_plugin)
