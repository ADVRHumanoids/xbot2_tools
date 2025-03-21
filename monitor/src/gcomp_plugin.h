#ifndef GCOMP_PLUGIN_H
#define GCOMP_PLUGIN_H

#include <xbot2/rt_plugin/control_plugin.h>
#include <xbot2/hal/dev_joint.h>

namespace XBot {

class GcompPlugin : public ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void on_start() override;
    void starting() override;
    void run() override;
    void on_stop() override;

private:

    Eigen::VectorXd _g;

};

}

#endif // TRAJECTORY_PLUGIN_H
