#include <xbot2/rt_plugin/control_plugin.h>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace XBot {

class InternalForceComp : public ControlPlugin
{
public:

using ControlPlugin::ControlPlugin;

bool on_initialize() override
{
    getParam("~contact_names", _contact_names);
    getParam("~do_gcomp", _do_gcomp);

    setDefaultControlMode(ControlMode::Effort());

    run();
    return true;
}

void on_start() override {}
void on_stop() override {}

void run() override
{
    if(!update_model_state())
    {
        return;
    }

    compute_contact_jac();
    solve_delta_tau();

    // v2 effort references have size Nv.  The first six entries are the
    // unactuated floating-base wrench and are kept at zero.
    _robot->setEffortReference(_delta_tau);
    _robot->move();
}

private:

bool update_model_state()
{
    // RobotInterface and its embedded ModelInterface share the complete
    // floating-base state. sense() updates that model as well.
    if(!_robot->sense(true))
    {
        return false;
    }

    const int nq = _robot->getNq();
    const int nv = _robot->getNv();

    _robot->getMotorPosition(_q);       // size Nq
    _robot->getPositionReference(_qref); // size Nq
    _robot->getStiffness(_kp);          // size Nv

    if(_q.size() != nq || _qref.size() != nq || _kp.size() != nv)
    {
        return false;
    }

    // qref - q must be computed on the configuration manifold. In particular,
    // a free-flyer has Nq = Nv + 1 because its orientation is a quaternion.
    _robot->difference(_qref, _q, _qerr); // size Nv

    // Build generalized quantities with the same Nv layout used by the model.
    // Only the actuated (tail) entries are commanded.
    _tau0.setZero(nv);
    _tau0.tail(nv - kFloatingBaseNv).noalias() =
        _kp.tail(nv - kFloatingBaseNv).cwiseProduct(
            _qerr.tail(nv - kFloatingBaseNv));

    _robot->computeGravityCompensation(_g); // size Nv

    if(!_do_gcomp)
    {
        _g.setZero();
    }

    _tau0.tail(nv - kFloatingBaseNv) +=
        _g.tail(nv - kFloatingBaseNv);

    return true;
}

void compute_contact_jac()
{
    const int nv = _robot->getNv();
    _Jct.setZero(nv, 3 * _contact_names.size());

    int col = 0;
    for(const auto& contact : _contact_names)
    {
        _robot->getJacobian(contact, _Jtmp); // 6 x Nv, world-aligned
        _Jct.middleCols(col, 3) = _Jtmp.topRows(3).transpose();
        col += 3;
    }
}

void solve_delta_tau()
{
    const int nv = _robot->getNv();
    const int nact = nv - kFloatingBaseNv;
    const int nf = 3 * _contact_names.size();

    // Jct maps contact forces to generalized forces. Its first six rows are
    // the floating-base wrench and its tail is the actuated joint torque.
    const auto A = _Jct.bottomRows(nact);              // nact x nf
    const auto C = _Jct.topRows(kFloatingBaseNv);      // 6 x nf
    const auto b = _tau0.tail(nact);                   // nact

    // min ||A f - b||^2 subject to C f = g_base.
    _K.setZero(nf + kFloatingBaseNv, nf + kFloatingBaseNv);
    _k.setZero(nf + kFloatingBaseNv);
    _K.topLeftCorner(nf, nf).noalias() = A.transpose() * A;
    _K.topRightCorner(nf, kFloatingBaseNv) = C.transpose();
    _K.bottomLeftCorner(kFloatingBaseNv, nf) = C;

    _k.head(nf).noalias() = A.transpose() * b;
    _k.tail(kFloatingBaseNv) = _g.head(kFloatingBaseNv);

    _lu.compute(_K);
    _sol = _lu.solve(_k);

    _delta_tau.setZero(nv);
    _delta_tau.tail(nact).noalias() =
        -A * _sol.head(nf) + _g.tail(nact);
}

static constexpr int kFloatingBaseNv = 6;

bool _do_gcomp = false;
std::vector<std::string> _contact_names;

Eigen::VectorXd _kp, _q, _qref, _qerr;
Eigen::MatrixXd _Jtmp, _Jct, _K;
Eigen::VectorXd _k, _tau0, _g, _sol, _delta_tau;
Eigen::FullPivLU<Eigen::MatrixXd> _lu;
};

}

XBOT2_REGISTER_PLUGIN(XBot::InternalForceComp, internal_force_comp)
