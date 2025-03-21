#include "monitor.h"

#include <xbot2/ros/ros2_support.h>
#include <xbot_msgs/msg/custom_state.hpp>

using namespace XBot::tools;

template <typename T>
struct Data;

template <>
struct Data<double>
{
    Data(const double& var);
    const double * data;
};

template <>
struct Data<Eigen::VectorXd>
{
    Data(const Eigen::VectorXd& var);
    const Eigen::VectorXd * data;
    int size;
};

class Monitor::Impl
{

public:

    Impl(std::string name);

    bool registerVariable(std::string name, double& var);

    bool registerVariable(std::string name, Eigen::VectorXd& var);

    bool publish();

private:



    PublisherPtr<xbot_msgs::msg::CustomState> _pub;

    std::vector<std::pair<std::string, double*>> _vars;

};

Monitor::Impl::Impl(std::string name)
{
    XBot::Ros2Support ros(XBot::Ros2Support::get_main_node());

    _pub = ros.advertise<xbot_msgs::msg::CustomState>(name, 10);

    if(!_pub)
    {
        throw std::runtime_error(fmt::format(
                                     "[Monitor] could not advertise topic {}", name)
                                 );
    }
}

bool Monitor::Impl::registerVariable(std::string name, double &var)
{
    _vars.emplace_back(name, &var);
    return true;
}

bool Monitor::Impl::registerVariable(std::string name, Eigen::VectorXd &var)
{
    bool success = true;

    for(int i = 0; i < var.size(); i++)
    {
        success = registerVariable(name + "_" + std::to_string(i), var[i]) && success;
    }

    return success;
}

bool Monitor::Impl::publish()
{
    auto msgptr = _pub->loanMessage();

    if(!msgptr)
    {
        return false;
    }

    auto& msg = **msgptr;

    auto now = XBot::chrono::system_clock::now();
    auto now_ts = XBot::chrono::duration_chrono_to_timespec(now.time_since_epoch());
    msg.header.stamp.sec = now_ts.tv_sec;
    msg.header.stamp.nanosec = now_ts.tv_nsec;

    if(msg.name.size() != _vars.size())
    {
        msg.name.resize(_vars.size());
        msg.value.resize(_vars.size());

        for(int i = 0; i < _vars.size(); i++)
        {
            msg.name[i] = _vars[i].first;
        }
    }

    for(int i = 0; i < _vars.size(); i++)
    {
        msg.value[i] = *(_vars[i].second);
    }

    return _pub->publishLoaned(std::move(msgptr));

}

Monitor::Monitor(std::string name)
{
    i = std::make_unique<Impl>(name);
}

bool Monitor::registerVariable(std::string name, double &var)
{
    return i->registerVariable(name, var);
}

bool Monitor::registerVariable(std::string name, Eigen::VectorXd &var)
{
    return i->registerVariable(name, var);
}

bool Monitor::publish()
{
    return i->publish();
}

Monitor::~Monitor()
{

}
