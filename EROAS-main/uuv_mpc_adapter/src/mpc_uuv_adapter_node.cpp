#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <uuv_control_msgs/Trajectory.h>
#include <uuv_control_msgs/TrajectoryPoint.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_predictor/utils.h>
#include <map_manager/occupancyMap.h>
#include <trajectory_planner/mpcPlanner.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <regex>
#include <string>
#include <utility>
#include <vector>

namespace {

struct ObstacleTrack {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  double radius = 1.0;
  ros::Time stamp;
};

std::string trim(const std::string& input) {
  const std::string whitespace = " \t\r\n";
  const std::size_t first = input.find_first_not_of(whitespace);
  if (first == std::string::npos) {
    return "";
  }
  const std::size_t last = input.find_last_not_of(whitespace);
  return input.substr(first, last - first + 1);
}

class UuvMpcAdapterNode {
 public:
  enum class PlanningMode {
    GLOBAL_PASS = 0,
    LOCAL_MPC = 1,
    REJOIN_HOLD = 2
  };

  UuvMpcAdapterNode() : nh_(), pnh_("~") {
    loadParams();

    waypoints_ = loadWaypointsFromFile(global_waypoint_file_);
    if (smooth_global_path_) {
      dense_path_ = buildLipbPath(waypoints_, waypoint_resolution_, path_lipb_radius_);
    } else {
      dense_path_ = densifyPath(waypoints_, waypoint_resolution_);
    }
    path_msg_ = buildPathMsg(dense_path_);
    if (dense_path_.size() < 2) {
      ROS_WARN("uuv_mpc_adapter: loaded path has <2 points, planner will stay idle");
    }

    // occMap/mpcPlanner read params from the provided NodeHandle namespace.
    // Launch file loads local_mpc_adapter.yaml into this node's private namespace,
    // so pass private NH here to avoid falling back to invalid defaults.
    map_.reset(new mapManager::occMap(pnh_));
    mpc_.reset(new trajPlanner::mpcPlanner(pnh_));
    mpc_->setMap(map_);
    mpc_->updateMaxVel(max_vel_);
    mpc_->updateMaxAcc(max_acc_);

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &UuvMpcAdapterNode::odomCB, this);
    detector_sub_ = nh_.subscribe(detector_velocity_topic_, 1, &UuvMpcAdapterNode::detectorCB, this);
    traj_pub_ = nh_.advertise<uuv_control_msgs::Trajectory>(output_trajectory_topic_, 1);
    global_path_pub_ = nh_.advertise<nav_msgs::Path>(smoothed_global_path_topic_, 1, true);
    publishSmoothedGlobalPath();

    plan_timer_ = nh_.createTimer(ros::Duration(plan_period_), &UuvMpcAdapterNode::planCB, this);

    ROS_INFO("uuv_mpc_adapter: odom=%s detector=%s trajectory_output=%s waypoints=%s global_path_pub=%s",
             odom_topic_.c_str(),
             detector_velocity_topic_.c_str(),
             output_trajectory_topic_.c_str(),
             global_waypoint_file_.c_str(),
             smoothed_global_path_topic_.c_str());
  }

 private:
  void loadParams() {
    pnh_.param<std::string>("uuv_name", uuv_name_, "rexrov");
    pnh_.param<std::string>("inertial_frame_id", inertial_frame_id_, "world");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/" + uuv_name_ + "/pose_gt");
    pnh_.param<std::string>("detector_velocity_topic", detector_velocity_topic_, "/onboard_detector/velocity_visualizaton");
    pnh_.param<std::string>("output_trajectory_topic", output_trajectory_topic_, "/" + uuv_name_ + "/dp_controller/input_trajectory");
    pnh_.param<std::string>("global_waypoint_file", global_waypoint_file_, "");
    pnh_.param<std::string>("smoothed_global_path_topic", smoothed_global_path_topic_, "/nav/global_path");
    pnh_.param<bool>("publish_smoothed_global_path", publish_smoothed_global_path_, true);

    pnh_.param<double>("plan_period", plan_period_, 0.15);
    pnh_.param<double>("trajectory_dt", trajectory_dt_, 0.1);
    pnh_.param<double>("mpc_dt", mpc_dt_, 0.1);
    pnh_.param<double>("waypoint_resolution", waypoint_resolution_, 0.5);
    pnh_.param<bool>("smooth_global_path", smooth_global_path_, true);
    pnh_.param<double>("path_lipb_radius", path_lipb_radius_, 10.0);
    pnh_.param<double>("obstacle_timeout", obstacle_timeout_, 1.0);
    pnh_.param<double>("default_obstacle_radius", default_obstacle_radius_, 2.0);
    pnh_.param<double>("max_vel", max_vel_, 0.8);
    pnh_.param<double>("max_acc", max_acc_, 0.6);
    pnh_.param<double>("publish_epsilon", publish_epsilon_, 0.2);
    pnh_.param<double>("publish_keepalive", publish_keepalive_, 4.0);
    pnh_.param<double>("robot_collision_radius", robot_collision_radius_, 0.8);
    pnh_.param<double>("local_trigger_distance", local_trigger_distance_, 8.0);
    pnh_.param<double>("local_release_distance", local_release_distance_, 10.0);
    pnh_.param<double>("local_forward_window", local_forward_window_, 25.0);
    pnh_.param<double>("local_ignore_behind", local_ignore_behind_, 3.0);
    pnh_.param<double>("local_max_duration", local_max_duration_, 20.0);
    pnh_.param<double>("rejoin_hold_time", rejoin_hold_time_, 2.5);
    pnh_.param<double>("rejoin_emergency_trigger_distance", rejoin_emergency_trigger_distance_, 2.5);
    pnh_.param<double>("global_pass_max_speed", global_pass_max_speed_, 0.5);
    pnh_.param<double>("min_dynamic_obstacle_speed", min_dynamic_obstacle_speed_, 0.12);
    pnh_.param<double>("local_clear_confirm_time", local_clear_confirm_time_, 1.0);
    pnh_.param<double>("rejoin_hold_max_speed", rejoin_hold_max_speed_, 0.35);
    pnh_.param<double>("local_rejoin_path_distance_threshold",
                       local_rejoin_path_distance_threshold_,
                       1.5);
    pnh_.param<double>("predicted_lateral_speed_ratio", predicted_lateral_speed_ratio_, 0.6);

    pnh_.param<int>("max_obstacles", max_obstacles_, 8);
    pnh_.param<bool>("yaw_from_path", yaw_from_path_, true);

    local_trigger_distance_ = std::max(0.0, local_trigger_distance_);
    local_release_distance_ = std::max(local_trigger_distance_, local_release_distance_);
    local_forward_window_ = std::max(0.0, local_forward_window_);
    local_ignore_behind_ = std::max(0.0, local_ignore_behind_);
    local_max_duration_ = std::max(0.0, local_max_duration_);
    rejoin_hold_time_ = std::max(0.0, rejoin_hold_time_);
    rejoin_emergency_trigger_distance_ = std::max(0.0, rejoin_emergency_trigger_distance_);
    global_pass_max_speed_ = std::max(0.0, global_pass_max_speed_);
    min_dynamic_obstacle_speed_ = std::max(0.0, min_dynamic_obstacle_speed_);
    local_clear_confirm_time_ = std::max(0.0, local_clear_confirm_time_);
    rejoin_hold_max_speed_ = std::max(0.0, rejoin_hold_max_speed_);
    local_rejoin_path_distance_threshold_ =
        std::max(0.0, local_rejoin_path_distance_threshold_);
    predicted_lateral_speed_ratio_ = std::max(0.0, predicted_lateral_speed_ratio_);
  }

  static std::pair<double, double> parseVelocityXY(const std::string& text) {
    const std::regex vx_vy_pattern(
        "Vx\\s*=?\\s*([-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?)\\s*,?\\s*Vy\\s*=?\\s*([-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?)");
    std::smatch match;
    if (std::regex_search(text, match, vx_vy_pattern) && match.size() >= 3) {
      return std::make_pair(std::stod(match[1].str()), std::stod(match[2].str()));
    }

    const std::regex number_pattern("[-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?");
    std::sregex_iterator it(text.begin(), text.end(), number_pattern);
    std::sregex_iterator end;
    std::vector<double> nums;
    for (; it != end; ++it) {
      nums.push_back(std::stod(it->str()));
      if (nums.size() >= 2) {
        break;
      }
    }
    if (nums.size() >= 2) {
      return std::make_pair(nums[0], nums[1]);
    }
    return std::make_pair(0.0, 0.0);
  }

  std::vector<Eigen::Vector3d> loadWaypointsFromFile(const std::string& file_path) const {
    std::vector<Eigen::Vector3d> waypoints;
    if (file_path.empty()) {
      ROS_WARN("uuv_mpc_adapter: global_waypoint_file is empty");
      return waypoints;
    }

    std::ifstream fin(file_path.c_str());
    if (!fin.is_open()) {
      ROS_ERROR("uuv_mpc_adapter: cannot open waypoint file: %s", file_path.c_str());
      return waypoints;
    }

    const std::regex number_pattern("[-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?");
    bool in_point_block = false;
    std::vector<double> coords;
    std::string line;
    while (std::getline(fin, line)) {
      const std::string t = trim(line);
      if (t.rfind("point:", 0) == 0) {
        in_point_block = true;
        coords.clear();
        continue;
      }
      if (!in_point_block) {
        continue;
      }
      std::smatch m;
      if (!std::regex_search(t, m, number_pattern)) {
        continue;
      }
      coords.push_back(std::stod(m.str()));
      if (coords.size() == 3) {
        waypoints.emplace_back(coords[0], coords[1], coords[2]);
        in_point_block = false;
        coords.clear();
      }
    }

    ROS_INFO("uuv_mpc_adapter: loaded %zu raw waypoints", waypoints.size());
    return waypoints;
  }

  std::vector<Eigen::Vector3d> densifyPath(const std::vector<Eigen::Vector3d>& points, double resolution) const {
    if (points.size() < 2) {
      return points;
    }
    const double res = std::max(1e-3, resolution);
    std::vector<Eigen::Vector3d> dense;
    dense.reserve(points.size() * 4);
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      const Eigen::Vector3d& p0 = points[i];
      const Eigen::Vector3d& p1 = points[i + 1];
      const double dist = (p1 - p0).norm();
      const int steps = std::max(1, static_cast<int>(std::ceil(dist / res)));
      for (int s = 0; s < steps; ++s) {
        const double t = static_cast<double>(s) / static_cast<double>(steps);
        dense.push_back(p0 + t * (p1 - p0));
      }
    }
    dense.push_back(points.back());
    ROS_INFO("uuv_mpc_adapter: densified path points=%zu", dense.size());
    return dense;
  }

  static void appendPointIfFar(std::vector<Eigen::Vector3d>& path,
                               const Eigen::Vector3d& p,
                               double min_dist = 1e-6) {
    if (path.empty() || (path.back() - p).norm() > min_dist) {
      path.push_back(p);
    }
  }

  static void appendLineSamples(std::vector<Eigen::Vector3d>& out,
                                const Eigen::Vector3d& p0,
                                const Eigen::Vector3d& p1,
                                double resolution) {
    const double dist = (p1 - p0).norm();
    if (dist < 1e-9) {
      appendPointIfFar(out, p1);
      return;
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(dist / std::max(1e-3, resolution))));
    for (int s = 1; s <= steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(steps);
      appendPointIfFar(out, p0 + t * (p1 - p0));
    }
  }

  static void appendQuadraticBezierSamples(std::vector<Eigen::Vector3d>& out,
                                           const Eigen::Vector3d& p0,
                                           const Eigen::Vector3d& p1,
                                           const Eigen::Vector3d& p2,
                                           double resolution) {
    // Control-polygon length is a stable upper estimate for sample count.
    const double est_len = (p1 - p0).norm() + (p2 - p1).norm();
    const int steps = std::max(2, static_cast<int>(std::ceil(est_len / std::max(1e-3, resolution))));
    for (int s = 1; s <= steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(steps);
      const double omt = 1.0 - t;
      const Eigen::Vector3d p = omt * omt * p0 + 2.0 * omt * t * p1 + t * t * p2;
      appendPointIfFar(out, p);
    }
  }

  std::vector<Eigen::Vector3d> buildLipbPath(const std::vector<Eigen::Vector3d>& waypoints,
                                             double resolution,
                                             double radius) const {
    if (waypoints.size() < 2) {
      return waypoints;
    }

    std::vector<Eigen::Vector3d> out;
    out.reserve(waypoints.size() * 6);
    appendPointIfFar(out, waypoints.front());

    Eigen::Vector3d line_start = waypoints.front();
    const double r_cfg = std::max(0.0, radius);

    for (std::size_t i = 1; i < waypoints.size(); ++i) {
      const Eigen::Vector3d& curr_wp = waypoints[i];
      const Eigen::Vector3d first_vec = curr_wp - line_start;
      const double first_len = first_vec.norm();
      if (first_len < 1e-9) {
        continue;
      }

      double turn_radius = std::min(r_cfg, 0.5 * first_len);
      const bool has_next = (i + 1 < waypoints.size());

      if (has_next) {
        const double second_len = (waypoints[i + 1] - curr_wp).norm();
        turn_radius = std::min(turn_radius, 0.5 * second_len);
      }

      if (!has_next || turn_radius < 1e-6) {
        appendLineSamples(out, line_start, curr_wp, resolution);
        line_start = curr_wp;
        continue;
      }

      const Eigen::Vector3d p_before =
          line_start + (first_len - turn_radius) / first_len * first_vec;
      appendLineSamples(out, line_start, p_before, resolution);

      const Eigen::Vector3d second_vec = waypoints[i + 1] - curr_wp;
      const double second_len = second_vec.norm();
      if (second_len < 1e-9) {
        line_start = p_before;
        continue;
      }
      const Eigen::Vector3d p_after =
          curr_wp + (turn_radius / second_len) * second_vec;

      appendQuadraticBezierSamples(out, p_before, curr_wp, p_after, resolution);
      line_start = p_after;
    }

    if ((out.back() - waypoints.back()).norm() > 1e-6) {
      appendLineSamples(out, out.back(), waypoints.back(), resolution);
    }

    ROS_INFO("uuv_mpc_adapter: LIPB-smoothed path points=%zu radius=%.2f resolution=%.2f",
             out.size(),
             r_cfg,
             std::max(1e-3, resolution));
    return out;
  }

  nav_msgs::Path buildPathMsg(const std::vector<Eigen::Vector3d>& path) const {
    nav_msgs::Path msg;
    msg.header.frame_id = inertial_frame_id_;
    msg.header.stamp = ros::Time::now();
    msg.poses.reserve(path.size());
    for (const auto& p : path) {
      geometry_msgs::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = p.z();
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    return msg;
  }

  static int nearestIndex(const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& pos) {
    if (path.empty()) {
      return 0;
    }
    int best_idx = 0;
    double best_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(path.size()); ++i) {
      const double d = (path[i] - pos).squaredNorm();
      if (d < best_dist) {
        best_dist = d;
        best_idx = i;
      }
    }
    return best_idx;
  }

  const char* modeName(PlanningMode mode) const {
    switch (mode) {
      case PlanningMode::GLOBAL_PASS:
        return "GLOBAL_PASS";
      case PlanningMode::LOCAL_MPC:
        return "LOCAL_MPC";
      case PlanningMode::REJOIN_HOLD:
        return "REJOIN_HOLD";
    }
    return "UNKNOWN";
  }

  bool switchMode(PlanningMode new_mode, const ros::Time& now, const std::string& reason) {
    if (mode_ == new_mode) {
      return false;
    }
    ROS_INFO("uuv_mpc_adapter: mode %s -> %s (%s)",
             modeName(mode_),
             modeName(new_mode),
             reason.c_str());
    mode_ = new_mode;
    mode_enter_time_ = now;
    return true;
  }

  void logModeHeartbeat(const ros::Time& now,
                        std::size_t obs_count,
                        std::size_t moving_obs_count,
                        double min_clearance_xy,
                        double dist_to_global_path_xy) const {
    const int is_global = (mode_ == PlanningMode::GLOBAL_PASS) ? 1 : 0;
    const int is_local = (mode_ == PlanningMode::LOCAL_MPC) ? 1 : 0;
    const int is_rejoin = (mode_ == PlanningMode::REJOIN_HOLD) ? 1 : 0;
    const double hold_left =
        (mode_ == PlanningMode::REJOIN_HOLD)
            ? std::max(0.0, (rejoin_hold_until_ - now).toSec())
            : 0.0;
    const double clearance = std::isfinite(min_clearance_xy) ? min_clearance_xy : -1.0;
    const double dist_to_global =
        std::isfinite(dist_to_global_path_xy) ? dist_to_global_path_xy : -1.0;
    ROS_INFO_THROTTLE(
        3.0,
        "uuv_mpc_adapter state[GLOBAL_PASS=%d LOCAL_MPC=%d REJOIN_HOLD=%d] mode=%s obs=%zu moving_obs=%zu "
        "clearance_xy=%.2f dist_to_global=%.2f hold_left=%.2f",
        is_global,
        is_local,
        is_rejoin,
        modeName(mode_),
        obs_count,
        moving_obs_count,
        clearance,
        dist_to_global,
        hold_left);
  }

  Eigen::Vector2d estimatePathTangent2D(const Eigen::Vector3d& curr_pos) const {
    Eigen::Vector2d t_hat(1.0, 0.0);
    if (dense_path_.size() < 2) {
      return t_hat;
    }
    const int idx = nearestIndex(dense_path_, curr_pos);
    int idx_next = std::min(idx + 1, static_cast<int>(dense_path_.size()) - 1);
    if (idx_next == idx && idx > 0) {
      idx_next = idx - 1;
    }
    Eigen::Vector2d d = (dense_path_[idx_next] - dense_path_[idx]).head<2>();
    const double n = d.norm();
    if (n > 1e-6) {
      t_hat = d / n;
    }
    return t_hat;
  }

  double distanceToPath2D(const Eigen::Vector3d& curr_pos) const {
    if (dense_path_.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    double best = std::numeric_limits<double>::infinity();
    for (const auto& p : dense_path_) {
      const double d = (p - curr_pos).head<2>().norm();
      best = std::min(best, d);
    }
    return best;
  }

  void buildPredictedObstacles(
      const std::vector<Eigen::Vector3d>& ob_pos,
      const std::vector<Eigen::Vector3d>& ob_vel,
      const std::vector<Eigen::Vector3d>& ob_size,
      std::vector<std::vector<std::vector<Eigen::Vector3d>>>& pred_pos,
      std::vector<std::vector<std::vector<Eigen::Vector3d>>>& pred_size,
      std::vector<Eigen::VectorXd>& intent_prob) const {
    pred_pos.clear();
    pred_size.clear();
    intent_prob.clear();
    if (ob_pos.empty()) {
      return;
    }

    const int num_intent = dynamicPredictor::STOP + 1;
    const int horizon = std::max(2, static_cast<int>(std::round(mpc_->getHorizon())));
    const double dt = std::max(1e-3, mpc_dt_);

    pred_pos.resize(ob_pos.size());
    pred_size.resize(ob_pos.size());
    intent_prob.resize(ob_pos.size());
    for (std::size_t i = 0; i < ob_pos.size(); ++i) {
      pred_pos[i].resize(num_intent);
      pred_size[i].resize(num_intent);

      const Eigen::Vector3d& p0 = ob_pos[i];
      const Eigen::Vector3d& v0 = ob_vel[i];
      const Eigen::Vector2d v_xy = v0.head<2>();
      const double speed_xy = v_xy.norm();
      Eigen::Vector2d t_hat(1.0, 0.0);
      if (speed_xy > 1e-6) {
        t_hat = v_xy / speed_xy;
      }
      const Eigen::Vector2d left_hat(-t_hat.y(), t_hat.x());
      const double lateral_speed = predicted_lateral_speed_ratio_ * speed_xy;

      Eigen::Vector3d v_forward = v0;
      Eigen::Vector3d v_left(v0.x() + lateral_speed * left_hat.x(),
                             v0.y() + lateral_speed * left_hat.y(),
                             v0.z());
      Eigen::Vector3d v_right(v0.x() - lateral_speed * left_hat.x(),
                              v0.y() - lateral_speed * left_hat.y(),
                              v0.z());
      Eigen::Vector3d v_stop = Eigen::Vector3d::Zero();
      if (speed_xy < 1e-3) {
        v_forward = Eigen::Vector3d::Zero();
        v_left = Eigen::Vector3d::Zero();
        v_right = Eigen::Vector3d::Zero();
      }

      const Eigen::Vector3d intent_vel[4] = {v_forward, v_left, v_right, v_stop};
      for (int intent = 0; intent < num_intent; ++intent) {
        pred_pos[i][intent].reserve(horizon);
        pred_size[i][intent].reserve(horizon);
        for (int k = 0; k < horizon; ++k) {
          const double t = static_cast<double>(k) * dt;
          pred_pos[i][intent].push_back(p0 + intent_vel[intent] * t);
          pred_size[i][intent].push_back(ob_size[i]);
        }
      }

      Eigen::VectorXd prob(num_intent);
      prob.setZero();
      if (speed_xy < min_dynamic_obstacle_speed_) {
        prob(dynamicPredictor::FORWARD) = 0.10;
        prob(dynamicPredictor::LEFT) = 0.10;
        prob(dynamicPredictor::RIGHT) = 0.10;
        prob(dynamicPredictor::STOP) = 0.70;
      } else {
        prob(dynamicPredictor::FORWARD) = 0.55;
        prob(dynamicPredictor::LEFT) = 0.20;
        prob(dynamicPredictor::RIGHT) = 0.20;
        prob(dynamicPredictor::STOP) = 0.05;
      }
      intent_prob[i] = prob / std::max(1e-6, prob.sum());
    }
  }

  std::vector<Eigen::Vector3d> buildFallbackTrajectory(const Eigen::Vector3d& curr_pos, int n_pts) const {
    std::vector<Eigen::Vector3d> out;
    if (dense_path_.empty()) {
      return out;
    }
    const int target_n = std::max(4, n_pts);
    const int start_idx = nearestIndex(dense_path_, curr_pos);
    for (int i = start_idx; i < static_cast<int>(dense_path_.size()) && static_cast<int>(out.size()) < target_n; ++i) {
      out.push_back(dense_path_[i]);
    }
    while (!out.empty() && static_cast<int>(out.size()) < target_n) {
      out.push_back(out.back());
    }
    if (!out.empty()) {
      out.front() = curr_pos;
    }
    return out;
  }

  void limitTrajectorySpeed(std::vector<Eigen::Vector3d>& traj, double max_speed) const {
    if (traj.size() < 2 || max_speed <= 0.0) {
      return;
    }
    const double dt = std::max(1e-3, trajectory_dt_);
    const double max_step = max_speed * dt;
    for (std::size_t i = 1; i < traj.size(); ++i) {
      const Eigen::Vector3d delta = traj[i] - traj[i - 1];
      const double dist = delta.norm();
      if (dist <= max_step || dist < 1e-9) {
        continue;
      }
      traj[i] = traj[i - 1] + (delta / dist) * max_step;
    }
  }

  void publishTrajectory(const std::vector<Eigen::Vector3d>& traj) {
    if (traj.empty()) {
      return;
    }
    const ros::Time now = ros::Time::now();
    uuv_control_msgs::Trajectory msg;
    msg.header.stamp = now;
    msg.header.frame_id = inertial_frame_id_;
    msg.points.reserve(traj.size());

    const double dt = std::max(1e-3, trajectory_dt_);
    for (std::size_t i = 0; i < traj.size(); ++i) {
      uuv_control_msgs::TrajectoryPoint tp;
      tp.header.stamp = now + ros::Duration(static_cast<double>(i) * dt);
      tp.pose.position.x = traj[i].x();
      tp.pose.position.y = traj[i].y();
      tp.pose.position.z = traj[i].z();

      double yaw = 0.0;
      if (yaw_from_path_ && i + 1 < traj.size()) {
        const Eigen::Vector3d d = traj[i + 1] - traj[i];
        yaw = std::atan2(d.y(), d.x());
      }
      const double half_yaw = 0.5 * yaw;
      tp.pose.orientation.x = 0.0;
      tp.pose.orientation.y = 0.0;
      tp.pose.orientation.z = std::sin(half_yaw);
      tp.pose.orientation.w = std::cos(half_yaw);

      Eigen::Vector3d vel = Eigen::Vector3d::Zero();
      if (i + 1 < traj.size()) {
        vel = (traj[i + 1] - traj[i]) / dt;
      } else if (i > 0) {
        vel = (traj[i] - traj[i - 1]) / dt;
      }
      tp.velocity.linear.x = vel.x();
      tp.velocity.linear.y = vel.y();
      tp.velocity.linear.z = vel.z();
      tp.velocity.angular.x = 0.0;
      tp.velocity.angular.y = 0.0;
      tp.velocity.angular.z = 0.0;
      tp.acceleration.linear.x = 0.0;
      tp.acceleration.linear.y = 0.0;
      tp.acceleration.linear.z = 0.0;
      tp.acceleration.angular.x = 0.0;
      tp.acceleration.angular.y = 0.0;
      tp.acceleration.angular.z = 0.0;

      msg.points.push_back(tp);
    }
    traj_pub_.publish(msg);
  }

  bool shouldPublishTrajectory(const std::vector<Eigen::Vector3d>& traj,
                               const ros::Time& now,
                               bool force = false) const {
    if (force) {
      return true;
    }
    if (traj.empty()) {
      return false;
    }
    if (last_published_traj_.empty()) {
      return true;
    }
    if ((now - last_publish_time_).toSec() >= std::max(0.0, publish_keepalive_)) {
      return true;
    }
    if (last_published_traj_.size() != traj.size()) {
      return true;
    }
    double max_diff_xy = 0.0;
    for (std::size_t i = 0; i < traj.size(); ++i) {
      const Eigen::Vector2d diff_xy = (traj[i] - last_published_traj_[i]).head<2>();
      max_diff_xy = std::max(max_diff_xy, diff_xy.norm());
    }
    return max_diff_xy > std::max(0.0, publish_epsilon_);
  }

  void publishSmoothedGlobalPath() {
    if (!publish_smoothed_global_path_ || path_msg_.poses.empty()) {
      return;
    }
    path_msg_.header.stamp = ros::Time::now();
    for (auto& pose : path_msg_.poses) {
      pose.header.stamp = path_msg_.header.stamp;
    }
    global_path_pub_.publish(path_msg_);
    ROS_INFO("uuv_mpc_adapter: published smoothed global path to %s (%zu poses)",
             smoothed_global_path_topic_.c_str(),
             path_msg_.poses.size());
  }

  void odomCB(const nav_msgs::OdometryConstPtr& msg) {
    std::lock_guard<std::mutex> lk(data_mtx_);
    curr_pos_.x() = msg->pose.pose.position.x;
    curr_pos_.y() = msg->pose.pose.position.y;
    curr_pos_.z() = msg->pose.pose.position.z;
    curr_vel_.x() = msg->twist.twist.linear.x;
    curr_vel_.y() = msg->twist.twist.linear.y;
    curr_vel_.z() = msg->twist.twist.linear.z;
    odom_ready_ = true;
  }

  void detectorCB(const visualization_msgs::MarkerArrayConstPtr& msg) {
    std::vector<ObstacleTrack> tracks;
    const ros::Time now = ros::Time::now();
    tracks.reserve(msg->markers.size());
    for (const auto& mk : msg->markers) {
      ObstacleTrack ob;
      ob.pos = Eigen::Vector3d(mk.pose.position.x, mk.pose.position.y, mk.pose.position.z);
      const auto vel_xy = parseVelocityXY(mk.text);
      ob.vel = Eigen::Vector3d(vel_xy.first, vel_xy.second, 0.0);
      ob.radius = default_obstacle_radius_;
      if (std::isfinite(mk.scale.x) && mk.scale.x > 1e-3) {
        ob.radius = std::max(0.1, 0.5 * mk.scale.x);
      }
      ob.stamp = mk.header.stamp.isZero() ? now : mk.header.stamp;
      tracks.push_back(ob);
    }

    std::lock_guard<std::mutex> lk(data_mtx_);
    obstacles_ = tracks;
  }

  void planCB(const ros::TimerEvent&) {
    Eigen::Vector3d curr_pos;
    Eigen::Vector3d curr_vel;
    bool ready = false;
    std::vector<ObstacleTrack> obstacles;
    {
      std::lock_guard<std::mutex> lk(data_mtx_);
      ready = odom_ready_;
      curr_pos = curr_pos_;
      curr_vel = curr_vel_;
      obstacles = obstacles_;
    }

    if (!ready || dense_path_.size() < 2) {
      return;
    }

    if (!mpc_path_initialized_) {
      mpc_->updatePath(path_msg_, mpc_dt_);
      mpc_path_initialized_ = true;
    }

    mpc_->updateCurrStates(curr_pos, curr_vel);

    const ros::Time now = ros::Time::now();
    if (mode_enter_time_.isZero()) {
      mode_enter_time_ = now;
    }

    struct RelevantObstacle {
      Eigen::Vector3d pos;
      Eigen::Vector3d vel;
      double inflated_radius = 0.0;
    };
    std::vector<RelevantObstacle> relevant_obs;
    relevant_obs.reserve(obstacles.size());
    const Eigen::Vector2d path_tangent = estimatePathTangent2D(curr_pos);
    double min_clearance_xy = std::numeric_limits<double>::infinity();
    std::size_t moving_relevant_obs_count = 0;

    for (const auto& ob : obstacles) {
      if ((now - ob.stamp).toSec() > obstacle_timeout_) {
        continue;
      }
      const double inflated_radius = std::max(0.0, ob.radius) + std::max(0.0, robot_collision_radius_);
      const double speed_xy = ob.vel.head<2>().norm();
      const bool is_moving_obstacle = (speed_xy >= min_dynamic_obstacle_speed_);
      if (is_moving_obstacle) {
        ++moving_relevant_obs_count;
      } else if (mode_ != PlanningMode::LOCAL_MPC) {
        // Only consider slow/static obstacles while in LOCAL_MPC avoidance mode.
        continue;
      }
      const Eigen::Vector2d rel_xy = (ob.pos - curr_pos).head<2>();
      const double forward = rel_xy.dot(path_tangent);
      if (forward < -local_ignore_behind_ || forward > local_forward_window_) {
        continue;
      }
      const double clearance_xy = rel_xy.norm() - inflated_radius;
      min_clearance_xy = std::min(min_clearance_xy, clearance_xy);
      relevant_obs.push_back({ob.pos, ob.vel, inflated_radius});
      if (max_obstacles_ > 0 && static_cast<int>(relevant_obs.size()) >= max_obstacles_) {
        break;
      }
    }

    const bool has_relevant_obstacles = !relevant_obs.empty();
    const bool trigger_local =
        has_relevant_obstacles && (min_clearance_xy <= local_trigger_distance_);
    const bool obstacle_cleared_for_rejoin =
        (!has_relevant_obstacles) || (min_clearance_xy >= local_release_distance_);
    const double dist_to_global_path_xy = distanceToPath2D(curr_pos);
    const bool near_global_path_for_rejoin =
        dist_to_global_path_xy <= local_rejoin_path_distance_threshold_;

    bool mode_changed = false;
    if (mode_ == PlanningMode::GLOBAL_PASS && trigger_local) {
      mode_changed |= switchMode(PlanningMode::LOCAL_MPC, now, "trigger local avoidance");
      obstacle_clear_since_ = ros::Time(0);
    }
    if (mode_ == PlanningMode::LOCAL_MPC) {
      if (obstacle_cleared_for_rejoin) {
        if (obstacle_clear_since_.isZero()) {
          obstacle_clear_since_ = now;
        }
      } else {
        obstacle_clear_since_ = ros::Time(0);
      }
      const bool clear_confirmed =
          !obstacle_clear_since_.isZero() &&
          ((now - obstacle_clear_since_).toSec() >= local_clear_confirm_time_);
      const bool local_timeout =
          local_max_duration_ > 0.0 && ((now - mode_enter_time_).toSec() >= local_max_duration_);
      if ((clear_confirmed || local_timeout) && near_global_path_for_rejoin) {
        rejoin_hold_until_ = now + ros::Duration(rejoin_hold_time_);
        mode_changed |= switchMode(PlanningMode::REJOIN_HOLD,
                                   now,
                                   local_timeout ? "local mode timeout" : "obstacle cleared (confirmed)");
        obstacle_clear_since_ = ros::Time(0);
      } else if (clear_confirmed || local_timeout) {
        ROS_WARN_THROTTLE(
            2.0,
            "uuv_mpc_adapter: keep LOCAL_MPC until near global path (dist=%.2f > threshold=%.2f)",
            dist_to_global_path_xy,
            local_rejoin_path_distance_threshold_);
      }
    } else if (mode_ == PlanningMode::REJOIN_HOLD) {
      const bool emergency_obstacle =
          has_relevant_obstacles && (min_clearance_xy <= rejoin_emergency_trigger_distance_);
      if (emergency_obstacle) {
        mode_changed |= switchMode(PlanningMode::LOCAL_MPC, now, "rejoin emergency obstacle");
        obstacle_clear_since_ = ros::Time(0);
      } else if (now >= rejoin_hold_until_ && near_global_path_for_rejoin) {
        mode_changed |= switchMode(PlanningMode::GLOBAL_PASS, now, "rejoin hold elapsed");
      } else if (now >= rejoin_hold_until_) {
        ROS_WARN_THROTTLE(
            2.0,
            "uuv_mpc_adapter: hold REJOIN_HOLD until near global path (dist=%.2f > threshold=%.2f)",
            dist_to_global_path_xy,
            local_rejoin_path_distance_threshold_);
      }
    }
    logModeHeartbeat(now,
                     relevant_obs.size(),
                     moving_relevant_obs_count,
                     min_clearance_xy,
                     dist_to_global_path_xy);

    std::vector<Eigen::Vector3d> traj;
    bool use_mpc = (mode_ == PlanningMode::LOCAL_MPC);
    bool ok = false;
    if (use_mpc) {
      std::vector<Eigen::Vector3d> ob_pos;
      std::vector<Eigen::Vector3d> ob_vel;
      std::vector<Eigen::Vector3d> ob_size;
      ob_pos.reserve(relevant_obs.size());
      ob_vel.reserve(relevant_obs.size());
      ob_size.reserve(relevant_obs.size());
      for (const auto& ob : relevant_obs) {
        ob_pos.push_back(ob.pos);
        ob_vel.push_back(ob.vel);
        const double d = std::max(0.2, 2.0 * ob.inflated_radius);
        ob_size.emplace_back(d, d, d);
      }
      std::vector<std::vector<std::vector<Eigen::Vector3d>>> pred_pos;
      std::vector<std::vector<std::vector<Eigen::Vector3d>>> pred_size;
      std::vector<Eigen::VectorXd> intent_prob;
      buildPredictedObstacles(ob_pos, ob_vel, ob_size, pred_pos, pred_size, intent_prob);
      mpc_->updatePredObstacles(pred_pos, pred_size, intent_prob);
      ok = mpc_->makePlanWithPred();
      if (!ok) {
        // Safety fallback: keep local avoidance alive even if intent-based solve fails.
        mpc_->updateDynamicObstacles(ob_pos, ob_vel, ob_size);
        ok = mpc_->makePlan();
      }
      if (ok) {
        mpc_->getTrajectory(traj);
      }
    } else {
      std::vector<Eigen::Vector3d> empty;
      std::vector<std::vector<std::vector<Eigen::Vector3d>>> empty_pred;
      std::vector<Eigen::VectorXd> empty_prob;
      mpc_->updatePredObstacles(empty_pred, empty_pred, empty_prob);
      mpc_->updateDynamicObstacles(empty, empty, empty);
    }

    if (!use_mpc || !ok || traj.size() < 2) {
      if (use_mpc && (!ok || traj.size() < 2)) {
        ROS_WARN_THROTTLE(1.0, "uuv_mpc_adapter: LOCAL_MPC failed, fallback to GLOBAL_PASS trajectory");
      }
      const int n_fallback = std::max(4, static_cast<int>(std::round(mpc_->getHorizon())));
      traj = buildFallbackTrajectory(curr_pos, n_fallback);
    }
    if (traj.size() < 2) {
      return;
    }
    if (mode_ == PlanningMode::GLOBAL_PASS) {
      limitTrajectorySpeed(traj, global_pass_max_speed_);
    } else if (mode_ == PlanningMode::REJOIN_HOLD) {
      limitTrajectorySpeed(traj, rejoin_hold_max_speed_);
    }
    const ros::Time pub_now = ros::Time::now();
    if (!shouldPublishTrajectory(traj, pub_now, mode_changed)) {
      return;
    }
    publishTrajectory(traj);
    last_published_traj_ = traj;
    last_publish_time_ = pub_now;
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber odom_sub_;
  ros::Subscriber detector_sub_;
  ros::Publisher traj_pub_;
  ros::Publisher global_path_pub_;
  ros::Timer plan_timer_;

  std::string uuv_name_;
  std::string inertial_frame_id_;
  std::string odom_topic_;
  std::string detector_velocity_topic_;
  std::string output_trajectory_topic_;
  std::string global_waypoint_file_;
  std::string smoothed_global_path_topic_;

  double plan_period_ = 0.15;
  double trajectory_dt_ = 0.1;
  double mpc_dt_ = 0.1;
  double waypoint_resolution_ = 0.5;
  bool smooth_global_path_ = true;
  double path_lipb_radius_ = 10.0;
  double obstacle_timeout_ = 1.0;
  double default_obstacle_radius_ = 2.0;
  double max_vel_ = 0.8;
  double max_acc_ = 0.6;
  double publish_epsilon_ = 0.2;
  double publish_keepalive_ = 4.0;
  double robot_collision_radius_ = 0.8;
  double local_trigger_distance_ = 8.0;
  double local_release_distance_ = 10.0;
  double local_forward_window_ = 25.0;
  double local_ignore_behind_ = 3.0;
  double local_max_duration_ = 20.0;
  double rejoin_hold_time_ = 2.5;
  double rejoin_emergency_trigger_distance_ = 2.5;
  double global_pass_max_speed_ = 0.5;
  double min_dynamic_obstacle_speed_ = 0.12;
  double local_clear_confirm_time_ = 1.0;
  double rejoin_hold_max_speed_ = 0.35;
  double local_rejoin_path_distance_threshold_ = 1.5;
  double predicted_lateral_speed_ratio_ = 0.6;
  int max_obstacles_ = 8;
  bool yaw_from_path_ = true;
  bool publish_smoothed_global_path_ = true;

  std::vector<Eigen::Vector3d> waypoints_;
  std::vector<Eigen::Vector3d> dense_path_;
  nav_msgs::Path path_msg_;

  std::shared_ptr<mapManager::occMap> map_;
  std::shared_ptr<trajPlanner::mpcPlanner> mpc_;
  bool mpc_path_initialized_ = false;

  std::mutex data_mtx_;
  bool odom_ready_ = false;
  Eigen::Vector3d curr_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d curr_vel_ = Eigen::Vector3d::Zero();
  std::vector<ObstacleTrack> obstacles_;
  std::vector<Eigen::Vector3d> last_published_traj_;
  ros::Time last_publish_time_;
  PlanningMode mode_ = PlanningMode::GLOBAL_PASS;
  ros::Time mode_enter_time_;
  ros::Time rejoin_hold_until_;
  ros::Time obstacle_clear_since_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "mpc_uuv_adapter_node");
  UuvMpcAdapterNode node;
  ros::spin();
  return 0;
}
