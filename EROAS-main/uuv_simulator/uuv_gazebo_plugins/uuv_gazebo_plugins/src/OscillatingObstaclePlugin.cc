// Move a model with configurable oscillatory motion.
// Primary motion is a constant-speed triangle wave along `axis`.
// Optional secondary motion is sinusoidal along `secondary_axis` for richer trajectories.

#include <cmath>
#include <functional>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
class OscillatingObstaclePlugin final : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model = std::move(model);
    if (!this->model)
      return;

    this->world = this->model->GetWorld();
    if (!this->world)
      return;

    this->startTime = this->world->SimTime();

    // Store initial pose as the center position.
#if GAZEBO_MAJOR_VERSION >= 8
    this->initialPose = this->model->WorldPose();
#else
    this->initialPose = this->model->GetWorldPose().Ign();
#endif

    // Params
    if (sdf && sdf->HasElement("axis"))
      this->axis = sdf->Get<ignition::math::Vector3d>("axis");
    if (this->axis.Length() < 1e-9)
      this->axis = ignition::math::Vector3d(0, 1, 0);
    this->axis.Normalize();

    if (sdf && sdf->HasElement("amplitude"))
      this->amplitude = sdf->Get<double>("amplitude");
    if (this->amplitude < 0.0)
      this->amplitude = -this->amplitude;
    if (this->amplitude < 1e-9)
      this->amplitude = 2.0;

    if (sdf && sdf->HasElement("speed"))
      this->speed = sdf->Get<double>("speed");
    if (this->speed <= 0.0)
      this->speed = 0.2;

    if (sdf && sdf->HasElement("secondary_axis"))
      this->secondaryAxis = sdf->Get<ignition::math::Vector3d>("secondary_axis");
    if (this->secondaryAxis.Length() > 1e-9)
    {
      // Keep secondary axis orthogonal to primary axis for stable 2D motion.
      this->secondaryAxis -= this->axis * this->secondaryAxis.Dot(this->axis);
      if (this->secondaryAxis.Length() > 1e-9)
        this->secondaryAxis.Normalize();
      else
        this->secondaryAxis = ignition::math::Vector3d::Zero;
    }
    else
      this->secondaryAxis = ignition::math::Vector3d::Zero;

    if (sdf && sdf->HasElement("secondary_amplitude"))
      this->secondaryAmplitude = sdf->Get<double>("secondary_amplitude");
    this->secondaryAmplitude = std::abs(this->secondaryAmplitude);

    if (sdf && sdf->HasElement("secondary_speed"))
      this->secondarySpeed = sdf->Get<double>("secondary_speed");
    if (this->secondarySpeed < 0.0)
      this->secondarySpeed = 0.0;

    if (sdf && sdf->HasElement("secondary_phase"))
      this->secondaryPhase = sdf->Get<double>("secondary_phase");

    if (sdf && sdf->HasElement("motion"))
      this->motion = sdf->Get<std::string>("motion");

    this->updateConn = event::Events::ConnectWorldUpdateBegin(
      std::bind(&OscillatingObstaclePlugin::OnUpdate, this, std::placeholders::_1));
  }

private:
  void OnUpdate(const common::UpdateInfo& info)
  {
    if (!this->model)
      return;

    const double t = (info.simTime - this->startTime).Double();
    const double A = this->amplitude;
    const double v = this->speed;

    ignition::math::Vector3d displacement = ignition::math::Vector3d::Zero;
    if (this->motion == "circle")
    {
      ignition::math::Vector3d axis2 = this->secondaryAxis;
      if (axis2.Length() < 1e-9)
      {
        // Fallback axis orthogonal to primary axis.
        if (std::abs(this->axis.Z()) < 0.9)
          axis2 = this->axis.Cross(ignition::math::Vector3d(0, 0, 1));
        else
          axis2 = this->axis.Cross(ignition::math::Vector3d(0, 1, 0));
        axis2.Normalize();
      }
      const double radius1 = std::max(1e-6, A);
      const double radius2 = (this->secondaryAmplitude > 1e-6) ? this->secondaryAmplitude : radius1;
      const double avgRadius = 0.5 * (radius1 + radius2);
      const double omega = v / std::max(1e-6, avgRadius);
      const double theta = omega * t + this->secondaryPhase;

      // Starts at initial pose and follows a circular/elliptic loop.
      const double offset1 = radius1 * std::sin(theta);
      const double offset2 = radius2 * (1.0 - std::cos(theta));
      displacement = this->axis * offset1 + axis2 * offset2;
    }
    else
    {
      // Triangle wave: start at 0, go to +A, then to -A, then back to 0 with constant |dy/dt| = v.
      const double seg = A / v;      // duration to travel distance A
      const double period = 4 * seg; // total distance 4A at speed v
      const double s = std::fmod(t, period);

      double offset = 0.0;
      if (s < seg)
        offset = v * s;
      else if (s < 3 * seg)
        offset = A - v * (s - seg);
      else
        offset = -A + v * (s - 3 * seg);

      displacement = this->axis * offset;
      if (this->secondaryAxis.Length() > 1e-9 &&
          this->secondaryAmplitude > 1e-9 &&
          this->secondarySpeed > 1e-9)
      {
        const double omega = this->secondarySpeed / this->secondaryAmplitude;
        const double secondaryOffset =
          this->secondaryAmplitude * std::sin(omega * t + this->secondaryPhase);
        displacement += this->secondaryAxis * secondaryOffset;
      }
    }

    auto pose = this->initialPose;
    pose.Pos() = this->initialPose.Pos() + displacement;

#if GAZEBO_MAJOR_VERSION >= 8
    this->model->SetWorldPose(pose);
#else
    this->model->SetWorldPose(pose.Ign());
#endif
  }

  physics::ModelPtr model;
  physics::WorldPtr world;
  event::ConnectionPtr updateConn;

  ignition::math::Pose3d initialPose;
  ignition::math::Vector3d axis{0, 1, 0};
  common::Time startTime;

  double amplitude{2.0};
  double speed{0.2};
  ignition::math::Vector3d secondaryAxis{0, 0, 0};
  double secondaryAmplitude{0.0};
  double secondarySpeed{0.0};
  double secondaryPhase{0.0};
  std::string motion{"triangle"};
};

GZ_REGISTER_MODEL_PLUGIN(OscillatingObstaclePlugin)
}  // namespace gazebo
