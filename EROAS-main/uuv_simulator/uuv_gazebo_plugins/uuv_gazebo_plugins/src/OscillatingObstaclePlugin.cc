// Move a model back-and-forth along a configurable axis with constant speed.
// Intended for simple dynamic obstacles (e.g., a cylinder) without requiring ROS topics.

#include <cmath>
#include <functional>

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

    auto pose = this->initialPose;
    pose.Pos() = this->initialPose.Pos() + this->axis * offset;

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
};

GZ_REGISTER_MODEL_PLUGIN(OscillatingObstaclePlugin)
}  // namespace gazebo
