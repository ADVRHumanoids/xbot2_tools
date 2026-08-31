#include <xbot2/rt_plugin/control_plugin.h>

#include <Eigen/Cholesky>

#include <string>
#include <vector>

namespace XBot
{

/**
 * Projects the active position reference onto the manifold on which the
 * selected contact-link positions match the link-side encoder state.
 *
 * Only Cartesian translations are constrained: contact-link orientations are
 * deliberately left free, so the projection removes spring deflection without
 * over-constraining a quadruped stance.
 */
class InternalStressRemovalPlugin : public ControlPlugin
{
public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override
    {
        if(!getParam("~contact_links", _contact_links) || _contact_links.empty())
        {
            jerror("missing mandatory '~contact_links' parameter");
            return false;
        }

        for(const auto& link : _contact_links)
        {
            if(_robot->getLinkId(link) < 0)
            {
                jerror("contact link '{}' is not present in the URDF", link);
                return false;
            }
        }

        getParam("~max_iterations", _max_iterations);
        getParam("~tolerance", _tolerance);
        getParam("~damping", _damping);
        getParam("~max_step", _max_step);
        getParam("~max_reference_rate", _max_reference_rate);

        if(_max_iterations <= 0 || _tolerance <= 0.0 || _damping < 0.0 ||
           _max_step <= 0.0 || _max_reference_rate <= 0.0)
        {
            jerror("invalid projection parameters");
            return false;
        }

        // The plugin publishes the resulting reference and therefore needs
        // position ownership for the whole robot.
        setDefaultControlMode(ControlMode::Position());
        _floating_base = _robot->isFloatingBase();

        return true;
    }

    void on_start() override
    {
        // RobotInterfaceXBot2Rt updates its joint state from get_link_pos(),
        // hence these poses are computed from the link-side encoders.
        if(!_robot->sense())
        {
            jerror("could not read link-side joint positions");
            stop();
            return;
        }

        _robot->getPositionReferenceFeedback(_command_reference);
        _target_reference = _command_reference;

        if(_command_reference.size() != _robot->getNq())
        {
            jerror("position reference has size {}, expected {}", _command_reference.size(), _robot->getNq());
            stop();
            return;
        }

        _contact_positions.resize(_contact_links.size());
        for(std::size_t i = 0; i < _contact_links.size(); ++i)
        {
            _contact_positions[i] = _robot->getPose(_contact_links[i]).translation();
        }

        _model = _robot->modelSharedPtr()->clone();
        if(!_model)
        {
            jerror("could not clone robot model");
            stop();
            return;
        }

        if(!project_reference())
        {
            stop();
            return;
        }

        start_completed();
    }

    void run() override
    {
        Eigen::VectorXd dq;
        _model->difference(_target_reference, _command_reference, dq);

        const double max_step = _max_reference_rate * getPeriodSec();
        if(dq.norm() <= max_step)
        {
            _command_reference = _target_reference;
        }
        else
        {
            dq *= max_step / dq.norm();
            _model->sum(_command_reference, dq, _command_reference);
        }

        _robot->setPositionReference(_command_reference);
        _robot->move();
    }

private:

    bool project_reference()
    {
        Eigen::VectorXd q = _target_reference;

        const Eigen::Index contact_rows = 3 * static_cast<Eigen::Index>(_contact_links.size());
        const Eigen::Index base_rows = _floating_base ? 6 : 0;
        Eigen::VectorXd error(contact_rows + base_rows);
        Eigen::MatrixXd jacobian(contact_rows + base_rows, _model->getNv());

        for(int iteration = 0; iteration < _max_iterations; ++iteration)
        {
            _model->setJointPosition(q);
            _model->update();

            for(std::size_t i = 0; i < _contact_links.size(); ++i)
            {
                const auto row = static_cast<Eigen::Index>(3 * i);
                error.segment<3>(row) = _contact_positions[i] - _model->getPose(_contact_links[i]).translation();

                Eigen::MatrixXd link_jacobian;
                if(!_model->getJacobian(_contact_links[i], link_jacobian))
                {
                    jerror("could not compute Jacobian for contact link '{}'", _contact_links[i]);
                    return false;
                }

                // XBot's spatial Jacobian stores the linear velocity in the
                // first three rows.
                jacobian.middleRows<3>(row) = link_jacobian.topRows<3>();
            }

            if(_floating_base)
            {
                // The floating-base velocity occupies the leading six model
                // coordinates.  Add [I_6 0] dq = 0 to keep it fixed while
                // the joint reference is projected.
                error.tail<6>().setZero();
                jacobian.bottomRows<6>().setZero();
                jacobian.bottomRows<6>().leftCols<6>().setIdentity();
            }

            if(error.norm() <= _tolerance)
            {
                _target_reference = q;
                jinfo("projected reference in {} iteration(s), residual {} m", iteration, error.norm());
                return true;
            }

            const Eigen::MatrixXd normal = jacobian * jacobian.transpose()
                + _damping * Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows());
            Eigen::VectorXd dq = jacobian.transpose() * normal.ldlt().solve(error);

            if(!dq.allFinite())
            {
                jerror("contact projection produced a non-finite correction");
                return false;
            }

            const double dq_norm = dq.norm();
            if(dq_norm > _max_step)
            {
                dq *= _max_step / dq_norm;
            }

            _model->sum(q, dq, q);
            _model->enforceJointLimits(q);
        }

        _model->setJointPosition(q);
        _model->update();
        for(std::size_t i = 0; i < _contact_links.size(); ++i)
        {
            error.segment<3>(static_cast<Eigen::Index>(3 * i)) =
                _contact_positions[i] - _model->getPose(_contact_links[i]).translation();
        }

        jerror("contact projection did not converge: residual {} m after {} iterations",
               error.norm(), _max_iterations);
        return false;
    }

    std::vector<std::string> _contact_links;
    std::vector<Eigen::Vector3d> _contact_positions;
    ModelInterface::UniquePtr _model;
    Eigen::VectorXd _command_reference;
    Eigen::VectorXd _target_reference;
    bool _floating_base{false};
    int _max_iterations{20};
    double _tolerance{1e-5};
    double _damping{1e-8};
    double _max_step{0.1};
    double _max_reference_rate{1.0};
};

} // namespace XBot

XBOT2_REGISTER_PLUGIN(XBot::InternalStressRemovalPlugin, internal_stress_removal_plugin)
