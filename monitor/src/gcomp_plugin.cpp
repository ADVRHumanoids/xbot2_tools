#include "gcomp_plugin.h"

using namespace XBot;

bool GcompPlugin::on_initialize()
{
    // preallocate
    _robot->model().computeGravityCompensation(_g);

    // declare used ctrl
    setDefaultControlMode(ControlMode::Effort());

    return true;
}

void GcompPlugin::on_start()
{
}

void GcompPlugin::starting()
{
    start_completed();
}

void GcompPlugin::run()
{
    _robot->sense();

    // TBD handle IMU

    _robot->model().computeGravityCompensation(_g);
    _robot->setEffortReference(_g);
    _robot->move();

}

void GcompPlugin::on_stop()
{
}

XBOT2_REGISTER_PLUGIN(GcompPlugin, gcomp_plugin)
