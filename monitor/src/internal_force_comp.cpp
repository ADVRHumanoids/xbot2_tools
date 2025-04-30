#include <xbot2/rt_plugin/control_plugin.h>

#include <Eigen/Dense>

namespace XBot {

class InternalForceComp : public ControlPlugin
{

public:

using ControlPlugin::ControlPlugin;

bool on_initialize() override
{
    getParam("~contact_names", _contact_names);

    getParam("~do_gcomp", _do_gcomp);

    _model = ModelInterface::getModel(_robot->getConfigOptions());

    setDefaultControlMode(ControlMode::Effort());

    run();

    return true;
}

void on_start() override
{

}

void on_stop() override
{

}

void run() override
{
    update_model_state();

    compute_contact_jac();

    solve_delta_tau();

    _robot->setEffortReference(_delta_tau);
    _robot->move();
}

void update_model_state()
{
    // TBD IMU
    
    _robot->sense(false, true);
    _model->syncFrom(*_robot);
    _model->update();

    _robot->getMotorPosition(_q);
    _robot->getPositionReference(_qref);
    _robot->getStiffness(_kp);
    _tau0 = _kp.cwiseProduct(_qref - _q);

    // std::cout << "tau0 = " << _tau0.transpose().format(3) << std::endl;
    
    _model->computeGravityCompensation(_g);
    
    if(!_do_gcomp)
    {
        _g.setZero();
    }
    
    _tau0 += _g.tail(_robot->getJointNum());
}

void compute_contact_jac()
{
    _Jct.setZero(_model->getJointNum(), _contact_names.size() * 3);

    int r = 0;

    for(auto& c : _contact_names)
    {
        _model->getJacobian(c, _Jtmp);
        _Jct.middleCols(r, 3) = _Jtmp.topRows(3).transpose();
        r += 3;
    }
}

void solve_delta_tau()
{
    // shorthands
    auto A = _Jct.bottomRows(_robot->getJointNum()); // nq x nf
    auto C = _Jct.topRows(6);  // nc x nf
    const auto& b = _tau0;

    // construct kkt system
    const int nf = 3*_contact_names.size();
    const int nc = 6;

    _K.setZero(nf + nc, nf + nc);
    _k.setZero(nf + nc);
    _K.topLeftCorner(nf, nf).noalias() = A.transpose() * A;
    _K.topRightCorner(nf, nc) = C.transpose();
    _K.bottomLeftCorner(nc, nf) = C;

    _k.head(nf).noalias() = A.transpose() * b;
    _k.tail(nc).noalias() = _g.head(nc);

    // solve kkt
    _lu.compute(_K);
    _sol = _lu.solve(_k);

    // std::cout << "F = \n" << _sol.head(nf).format(3) << std::endl;
    
    // compute delta tau
    _delta_tau.noalias() = -A * _sol.head(nf) + _g.tail(_robot->getJointNum());
    // std::cout << "delta tau = " << _delta_tau.transpose().format(3) << std::endl;
}

private:

bool _do_gcomp = false;

ModelInterface::Ptr _model;
std::vector<std::string> _contact_names;

Eigen::VectorXd _kp, _q, _qref;

Eigen::MatrixXd _Jtmp;
Eigen::MatrixXd _Jct;
Eigen::MatrixXd _K;
Eigen::VectorXd _k;
Eigen::VectorXd _tau0;
Eigen::VectorXd _g;

Eigen::FullPivLU<Eigen::MatrixXd> _lu;
Eigen::VectorXd _sol;

Eigen::VectorXd _delta_tau;

};

}

XBOT2_REGISTER_PLUGIN(XBot::InternalForceComp, internal_force_comp)