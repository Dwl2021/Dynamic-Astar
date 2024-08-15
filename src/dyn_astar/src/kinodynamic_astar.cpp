/**
 * This file is part of Fast-Planner.
 *
 * Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and
 * Technology, <uav.ust.hk> Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>,
 * <uv dot boyuzhou at gmail dot com> for more information see
 * <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>. If you use this code, please
 * cite the respective publications as listed on the above website.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 */

#include <dyn_astar/kinodynamic_astar.h>

#include <sstream>

using namespace Eigen;
bool _debug = false;

KinodynamicAstar::~KinodynamicAstar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v,
                             Eigen::Vector3d start_a, Eigen::Vector3d start_j,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v,
                             Eigen::Vector3d end_a, bool dynamic, double time_start)
{
  start_vel_ = start_v;
  start_acc_ = start_a;
  start_jer_ = start_j;

  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;
  cur_node->state.segment(3, 3) = start_v;
  cur_node->state.tail(3) = start_a;
  cur_node->vel_dis = discretizeVel(start_vel_);
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;
  Eigen::VectorXd end_state(9);
  Eigen::Vector3i end_index;
  double optimal_time;
  end_pos_ = end_pt;

  end_state.head(3) = end_pt;
  end_state.segment(3, 3) = end_v;
  end_state.tail(3) = end_a;
  end_index = posToIndex(end_pt);
  cur_node->f_score =
      lambda_heu_ * estimateHeuristic(cur_node->state, end_state, optimal_time);

  ROS_INFO("heuristic: %f", cur_node->f_score);

  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_ += 1;

  if (dynamic)
  {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
    // std::cout << "time start: " << time_start <<std::endl;
  }
  else
    expanded_nodes_.insert(cur_node->index, cur_node->vel_dis, cur_node);

  PathNodePtr neighbor = NULL;
  PathNodePtr terminate_node = NULL;
  const int tolerance = ceil(tolerance_ / resolution_);

  while (!open_set_.empty())
  {
    cur_node = open_set_.top();

    // Terminate?
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;
    // bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
    //                 abs(cur_node->index(1) - end_index(1)) <= tolerance &&
    //                 abs(cur_node->index(2) - end_index(2)) <= tolerance;
    bool near_end = (cur_node->state.head(3) - end_pt).norm() <= tolerance_;

    if (near_end)
    {
      ROS_INFO("NEAR END");
      // Check whether shot traj exist
      estimateHeuristic(cur_node->state, end_state, optimal_time);
      computeShotTraj(cur_node->state, end_state, optimal_time);
      if (is_shot_succ_)
      {
        terminate_node = cur_node;
        retrievePath(terminate_node);
        std::cout << "shot success state: " << cur_node->state.head(3).transpose()
                  << std::endl;
        std::cout << "end state : " << end_state.head(3).transpose() << std::endl;
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else
      {
        // ROS_INFO("Current pos: %f, %f, %f", cur_node->state(0), cur_node->state(1),
        //          cur_node->state(2));
        // ROS_INFO("Current vel: %f, %f, %f", cur_node->state(3), cur_node->state(4),
        //          cur_node->state(5));
        // ROS_INFO("Shot traj not exist!");
      }
    }

    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;
    // ROS_INFO("iter num: %d", iter_num_);
    // ROS_INFO("heuristic: %f", cur_node->f_score - cur_node->g_score);

    double res = jerk_res_, time_res = time_res_;
    Eigen::Matrix<double, 9, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 9, 1> pro_state;
    std::vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;
    std::vector<Eigen::Vector3d> inputs;
    std::vector<double> durations;
    Eigen::Vector3d cur_pos = cur_state.head(3);
    // ROS_INFO("traverse points: %f, %f, %f", cur_pos(0), cur_pos(1), cur_pos(2));
    Eigen::Vector3d cur_vel = cur_state.segment(3, 3);
    // ROS_INFO("traverse vel: %f, %f, %f", cur_vel(0), cur_vel(1), cur_vel(2));

    for (double jerk_x = -max_jer_; jerk_x <= max_jer_ + 1e-3; jerk_x += max_jer_ * res)
      for (double jerk_y = -max_jer_; jerk_y <= max_jer_ + 1e-3; jerk_y += max_jer_ * res)
      // for (double jerk_z = -max_jer_; jerk_z <= max_jer_ + 1e-3;
      //  jerk_z += max_jer_ * res)
      {
        um << jerk_x, jerk_y, 0;
        inputs.push_back(um);
      }
    for (double tau = time_res * max_tau_; tau <= max_tau_ + 1e-3;
         tau += time_res * max_tau_)
      durations.push_back(tau);

    // std::cout << "cur state:" << cur_state.head(3).transpose() <<std::endl;
    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)
      {
        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = cur_node->time + tau;  // next node's time

        Eigen::Vector3d pro_pos = pro_state.head(3);

        // Check if in close set
        Eigen::Vector3i pro_id = posToIndex(pro_pos);
        Eigen::Vector3d pro_vel = pro_state.segment(3, 3);
        Eigen::Vector3d pro_vel_dis = discretizeVel(pro_vel);
        Eigen::Vector3d pro_acc = pro_state.segment(6, 3);

        int pro_t_id = timeToIndex(pro_t);
        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id)
                                       : expanded_nodes_.find(pro_id, pro_vel_dis);
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          if (_debug) std::cout << "close" << std::endl;
          continue;
        }

        // Check maximal velocity
        Eigen::Vector3d pro_v = pro_state.segment(3, 3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ ||
            fabs(pro_v(2)) > max_vel_)
        {
          if (_debug) std::cout << "vel" << std::endl;
          continue;
        }

        // Check maximal acceleration
        Eigen::Vector3d pro_a = pro_state.tail(3);
        if (fabs(pro_a(0)) > max_acc_ || fabs(pro_a(1)) > max_acc_ ||
            fabs(pro_a(2)) > max_acc_)
        {
          if (_debug) std::cout << "acc" << std::endl;
          continue;
        }

        if ((pro_pos(2) < start_pt(2) && pro_pos(2) < end_pt(2)) ||
            (pro_pos(2) > start_pt(2) && pro_pos(2) > end_pt(2)))
        {
          if (_debug) std::cout << "z exceed" << std::endl;
          continue;
        }

        // Check not in the same voxel
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        Eigen::Vector3d diff_vel = pro_vel_dis - cur_node->vel_dis;

        if (diff.norm() == 0 && diff_vel.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          if (_debug) std::cout << "same" << std::endl;
          continue;
        }

        // Check safety
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 9, 1> xt;
        bool is_occ = false;
        Eigen::Vector3d check_pos;
        for (int k = 1; k <= check_num_; ++k)
        {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          check_pos = xt.head(3);
          if (map_util_->isOccupied(check_pos) == true)
          {
            if (_debug)
            {
              ROS_INFO("extend fail, occupied pos = %f, %f, %f", check_pos(0),
                       check_pos(1), check_pos(2));
              std::cout << "occ" << std::endl;
            }
            is_occ = true;
            break;
          }
        }
        if (is_occ)
        {
          // ROS_INFO("occupied spos: %f, %f, %f", pos(0), pos(1), pos(2));
          continue;
        }
        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + rho_) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score +
                      lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // Compare nodes expanded from the same parent
        bool prune = false;

        for (int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];

          Eigen::Vector3d vel_diff = pro_vel_dis - expand_node->vel_dis;

          if ((pro_id - expand_node->index).norm() == 0 && vel_diff.norm() == 0 &&
              ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;
            if (tmp_f_score < expand_node->f_score)
            {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->vel_dis = pro_vel_dis;
              expand_node->input = um;
              expand_node->duration = tau;
              if (dynamic) expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }

        // This node end up in a voxel different from others
        if (!prune)
        {
          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->vel_dis = pro_vel_dis;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic)
            {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_vel_dis, pro_node);

            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              std::cout << "run out of memory." << std::endl;
              return NO_PATH;
            }
          }
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
            {
              // pro_node->index = pro_id;
              pro_node->state = pro_state;
              pro_node->vel_dis = pro_vel_dis;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              if (dynamic) pro_node->time = cur_node->time + tau;
            }
          }
          else
          {
            std::cout << "error type in searching: " << pro_node->node_state << std::endl;
          }
        }
      }
    // init_search = false;
  }

  std::cout << "open set empty, no path!" << std::endl;
  std::cout << "use node num: " << use_node_num_ << std::endl;
  std::cout << "iter num: " << iter_num_ << std::endl;
  return NO_PATH;
}

void KinodynamicAstar::setParam(ros::NodeHandle& nh)
{
  nh.param("/search/max_tau", max_tau_, -1.0);
  nh.param("/search/init_max_tau", init_max_tau_, -1.0);
  nh.param("/search/max_vel", max_vel_, -1.0);
  nh.param("/search/max_acc", max_acc_, -1.0);
  nh.param("/search/max_jer", max_jer_, -1.0);
  nh.param("/search/max_omega", max_omega_, -1.0);
  nh.param("/search/rho", rho_, -1.0);
  nh.param("/search/horizon", horizon_, -1.0);
  nh.param("/search/resolution_astar", resolution_, -1.0);
  nh.param("/search/resolution_input", jerk_res_, -1.0);
  nh.param("/search/resolution_time", time_res_, -1.0);
  nh.param("/search/time_id_resolution", time_resolution_, -1.0);
  nh.param("/search/lambda_heu", lambda_heu_, -1.0);
  nh.param("/search/allocate_num", allocate_num_, -1);
  nh.param("/search/check_num", check_num_, -1);
  nh.param("/search/optimistic", optimistic_, true);
  nh.param("/search/tolerance", tolerance_, 1.0);
  tie_breaker_ = 1.0 + 1.0 / 10000;

  double vel_margin, acc_margin;
  nh.param("/search/vel_margin", vel_margin, 0.0);
  nh.param("/search/acc_margin", acc_margin, -1.0);
  max_vel_ += vel_margin;
  max_acc_ += acc_margin;
}

void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                                           double& optimal_time)
{
  const Vector3d p_o = x1.head(3);
  const Vector3d v_o = x1.segment(3, 3);
  const Vector3d a_o = x1.segment(6, 3);
  const Vector3d p_f = x2.head(3);
  const Vector3d v_f = x2.segment(3, 3);
  const Vector3d a_f = x2.segment(6, 3);

  double c4 = (-18 * a_o.dot(a_o) + 12 * a_o.dot(a_f) - 18 * a_f.dot(a_f));
  double c3 =
      (144 * a_f.dot(v_o) - 144 * a_o.dot(v_f) - 216 * a_o.dot(v_o) + 216 * a_f.dot(v_f));
  double c2 =
      (-768 * v_o.dot(v_o) - 1344 * v_o.dot(v_f) - 768 * v_f.dot(v_f) -
       480 * a_o.dot(p_o) + 480 * a_o.dot(p_f) + 480 * a_f.dot(p_o) - 480 * a_f.dot(p_f));
  double c1 = (3600 * p_f.dot(v_o) - 3600 * p_o.dot(v_f) - 3600 * p_o.dot(v_o) +
               3600 * p_f.dot(v_f));
  double c0 = (-4320 * p_o.dot(p_o) + 8640 * p_o.dot(p_f) - 4320 * p_f.dot(p_f));

  std::vector<double> ts = quartic(c4, c3, c2, c1, c0);

  double v_max = max_vel_ * 0.5;
  double t_bar = (p_o - p_f).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;
  double max_omega;

  for (auto t : ts)
  {
    if (t < t_bar) continue;
    double c = jerkCost(p_o, v_o, a_o, p_f, v_f, a_f, t);
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }
  optimal_time = t_d;
  return 1.0 * (1 + tie_breaker_) * cost;
}

double KinodynamicAstar::jerkCost(const Vector3d& p_o, const Vector3d& v_o,
                                  const Vector3d& a_o, const Vector3d& p_f,
                                  const Vector3d& v_f, const Vector3d& a_f, double T)
{
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = p_o;
  const Vector3d v0 = v_o;
  const Vector3d a0 = a_o;
  const Vector3d p1 = p_f;
  const Vector3d v1 = v_f;
  const Vector3d a1 = a_f;
  double t_d = T;
  MatrixXd coef(3, 6);

  /*
  p = c0 + c1t + c2t^2 + c3t^3 + c4t^4 + c5t^5
  */
  Vector3d c5 = -(12 * p0 - 12 * p1 + 6 * t_d * v1 + 6 * t_d * v0 - t_d * t_d * a1 +
                  t_d * t_d * a0) /
                (2 * pow(t_d, 5));
  Vector3d c4 = (30 * p0 - 30 * p1 + 14 * t_d * v1 + 16 * t_d * v0 - 2 * t_d * t_d * a1 +
                 3 * t_d * t_d * a0) /
                (2 * pow(t_d, 4));
  Vector3d c3 = -(20 * p0 - 20 * p1 + 8 * t_d * v1 + 12 * t_d * v0 - t_d * t_d * a1 +
                  3 * t_d * t_d * a0) /
                (2 * pow(t_d, 3));
  Vector3d c2 = a0 / 2.0;
  Vector3d c1 = v0;
  Vector3d c0 = p0;

  // Fill the coefficients matrix
  coef.col(5) = c5;
  coef.col(4) = c4;
  coef.col(3) = c3;
  coef.col(2) = c2;
  coef.col(1) = c1;
  coef.col(0) = c0;

  // Calculate the jerk cost
  double objective = 0.0;
  for (int i = 0; i < 3; i++)
  {  // For each dimension (x, y, z)
    objective += 36.0 * pow(coef(i, 3), 2) * T +
                 144.0 * coef(i, 4) * coef(i, 3) * pow(T, 2) +
                 192.0 * pow(coef(i, 4), 2) * pow(T, 3) +
                 240.0 * coef(i, 5) * coef(i, 3) * pow(T, 3) +
                 720.0 * coef(i, 5) * coef(i, 4) * pow(T, 4) +
                 720.0 * pow(coef(i, 5), 2) * pow(T, 5);
  }

  return objective;
}

bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                                       double time)
{
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = state1.head(3);
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d a0 = state1.tail(3);
  const Vector3d p1 = state2.head(3);
  const Vector3d v1 = state2.segment(3, 3);
  const Vector3d a1 = state2.tail(3);
  double t_d = time;
  // t_d = 10;
  // std::cout << "t_d: " << t_d << std::endl;
  MatrixXd coef(3, 6);
  end_vel_ = v1;

  Vector3d c5 = -(12 * p0 - 12 * p1 + 6 * t_d * v1 + 6 * t_d * v0 - t_d * t_d * a1 +
                  t_d * t_d * a0) /
                (2 * pow(t_d, 5));
  Vector3d c4 = (30 * p0 - 30 * p1 + 14 * t_d * v1 + 16 * t_d * v0 - 2 * t_d * t_d * a1 +
                 3 * t_d * t_d * a0) /
                (2 * pow(t_d, 4));
  Vector3d c3 = -(20 * p0 - 20 * p1 + 8 * t_d * v1 + 12 * t_d * v0 - t_d * t_d * a1 +
                  3 * t_d * t_d * a0) /
                (2 * pow(t_d, 3));
  Vector3d c2 = a0 / 2.0;
  Vector3d c1 = v0;
  Vector3d c0 = p0;

  // Fill the coefficients matrix
  coef.col(5) = c5;
  coef.col(4) = c4;
  coef.col(3) = c3;
  coef.col(2) = c2;
  coef.col(1) = c1;
  coef.col(0) = c0;

  Vector3d pos, vel, acc, jer;
  Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
  double s1, s2, s3, s4, s5;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = 0.05;
  for (double time = t_delta; time <= t_d; time += t_delta)
  {
    s1 = time;
    s2 = s1 * s1;
    s3 = s2 * s1;
    s4 = s2 * s2;
    s5 = s2 * s3;
    beta0 << 1, s1, s2, s3, s4, s5;
    // beta1 << 0, 1, 2 * s1, 3 * s2, 4 * s3, 5 * s4;
    beta2 << 0, 0, 2, 6 * s1, 12 * s2, 20 * s3;
    beta3 << 0, 0, 0, 6, 24 * s1, 60 * s2;

    pos = coef * beta0;
    // vel = coef * beta1;
    acc = coef * beta2;
    jer = coef * beta3;

    Vector3d thrust = acc + Eigen::Vector3d(0, 0, 9.81);
    Vector3d zb_dot = f_DN(thrust) * jer;
    double z_dot_norm = zb_dot.norm();
    // ROS_INFO("SHOT OMEGA: %f", z_dot_norm);

    if (pos(0) < origin_(0) || pos(0) >= origin_(0) + map_size_3d_(0) ||
        pos(1) < origin_(1) || pos(1) >= origin_(1) + map_size_3d_(1) ||
        pos(2) < origin_(2) || pos(2) >= origin_(2) + map_size_3d_(2))
    {
      if (_debug)
        ROS_INFO("shot fail, out of map, pos = %f, %f, %f", pos(0), pos(1), pos(2));
      return false;
    }

    if (map_util_->isOccupied(pos) == true)
    {
      if (_debug)
        ROS_INFO("shot fail, occupied pos = %f, %f, %f", pos(0), pos(1), pos(2));
      return false;
    }
  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

std::vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
  std::vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

std::vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d,
                                              double e)
{
  std::vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  std::vector<double> ys =
      cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0) return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 +
             0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 -
             0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

void KinodynamicAstar::init()
{
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  map_util_->getOrigin(origin_);
  map_util_->getMapSize(map_size_3d_);

  std::cout << "origin_: " << origin_.transpose() << std::endl;
  std::cout << "map size: " << map_size_3d_.transpose() << std::endl;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  ROS_INFO("allocate_num: %d", allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
}

void KinodynamicAstar::setMap(const std::shared_ptr<MapUtil<3>>& map_util)
{
  this->map_util_ = map_util;
}

bool KinodynamicAstar::hasMap() { return map_util_->has_map_(); }

void KinodynamicAstar::reset()
{
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
}

double KinodynamicAstar::getKinoTraj(double delta_t, std::vector<Eigen::Vector3d>& path)
{
  std::vector<Vector3d> state_list;
  double total_time;

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  Matrix<double, 9, 1> x0, xt;

  while (node->parent != NULL)
  {
    Vector3d ut = node->input;
    double duration = node->duration;
    total_time += duration;
    x0 = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->parent;
  }
  reverse(state_list.begin(), state_list.end());
  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    Vector3d pos, vel, acc, jer;
    Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      s1 = t;
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s2 * s3;
      beta0 << 1, s1, s2, s3, s4, s5;
      beta1 << 0, 1, 2 * s1, 3 * s2, 4 * s3, 5 * s4;
      beta2 << 0, 0, 2, 6 * s1, 12 * s2, 20 * s3;
      beta3 << 0, 0, 0, 6, 24 * s1, 60 * s2;
      pos = coef_shot_ * beta0;
      vel = coef_shot_ * beta1;
      acc = coef_shot_ * beta2;
      jer = coef_shot_ * beta3;

      /* omega */
      Vector3d thrust = acc + Eigen::Vector3d(0, 0, 9.81);
      Vector3d zb_dot = f_DN(thrust) * jer;
      double z_dot_norm = zb_dot.norm();

      ROS_INFO("pos: %f, %f, %f", pos(0), pos(1), pos(2));
      ROS_INFO("vel: %f, %f, %f", vel(0), vel(1), vel(2));
      ROS_INFO("acc: %f, %f, %f", acc(0), acc(1), acc(2));
      ROS_INFO("jer: %f, %f, %f", jer(0), jer(1), jer(2));
      ROS_INFO("Omega: %f", z_dot_norm);
      ROS_INFO("---------------------------------------------");
      state_list.push_back(pos);
    }
  }
  else
  {
    std::cout << "no shot traj" << std::endl;
    return inf;
  }
  path = state_list;

  return total_time;
}

void KinodynamicAstar::getKinoTraj(double delta_t, vec_Vec3f& path)
{
  std::vector<Vector3d> state_list;
  path.clear();

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  Matrix<double, 9, 1> x0, xt;

  while (node->parent != NULL)
  {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->parent;
  }
  reverse(state_list.begin(), state_list.end());
  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    Vector3d pos, vel, acc, jer;
    Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      s1 = t;
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s2 * s3;
      beta0 << 1, s1, s2, s3, s4, s5;
      beta1 << 0, 1, 2 * s1, 3 * s2, 4 * s3, 5 * s4;
      beta2 << 0, 0, 2, 6 * s1, 12 * s2, 20 * s3;
      beta3 << 0, 0, 0, 6, 24 * s1, 60 * s2;
      pos = coef_shot_ * beta0;
      vel = coef_shot_ * beta1;
      acc = coef_shot_ * beta2;
      jer = coef_shot_ * beta3;

      ROS_INFO("pos: %f, %f, %f", pos(0), pos(1), pos(2));
      ROS_INFO("vel: %f, %f, %f", vel(0), vel(1), vel(2));
      ROS_INFO("acc: %f, %f, %f", acc(0), acc(1), acc(2));
      ROS_INFO("jer: %f, %f, %f", jer(0), jer(1), jer(2));
      ROS_INFO("---------------------------------------------");
      state_list.push_back(pos);
    }
  }
  else
  {
    std::cout << "no shot traj" << std::endl;
  }

  Vec3f tmp_path;
  for (auto tmp_state : state_list)
  {
    tmp_path(0) = tmp_state(0);
    tmp_path(1) = tmp_state(1);
    tmp_path(2) = tmp_state(2);
    path.push_back(tmp_path);
  }

  return;
}
std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
{
  std::vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
  return idx;
}

Eigen::Vector3d KinodynamicAstar::discretizeVel(const Eigen::Vector3d& vel)
{
  const int NUM_DIRECTIONS = 48;
  const int NUM_LEVELS = 20;

  // Calculate angle and magnitude
  double angle = std::atan2(vel.y(), vel.x());
  double magnitude = vel.norm();

  // Discretize direction
  int direction = std::round(angle / (2 * M_PI / NUM_DIRECTIONS));
  direction = (direction + NUM_DIRECTIONS) % NUM_DIRECTIONS;  // Ensure positive index

  // Discretize magnitude
  int level = std::round(magnitude / (max_vel_ / NUM_LEVELS));
  level = std::min(level, NUM_LEVELS - 1);  // Clamp to max level

  // Calculate discretized velocity
  double discretized_angle = direction * (2 * M_PI / NUM_DIRECTIONS);
  double discretized_magnitude = level * (max_vel_ / NUM_LEVELS);

  Eigen::Vector3d discretized_vel;
  discretized_vel.x() = discretized_magnitude * std::cos(discretized_angle);
  discretized_vel.y() = discretized_magnitude * std::sin(discretized_angle);
  discretized_vel.z() = vel.z();

  return discretized_vel;
}

int KinodynamicAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 9, 1>& state0,
                                    Eigen::Matrix<double, 9, 1>& state1,
                                    Eigen::Vector3d um, double tau)
{
  phi_.setZero();
  for (int i = 0; i < 3; ++i)
  {
    phi_(i, i) = 1.0;
    phi_(i, i + 3) = tau;
    phi_(i, i + 6) = 0.5 * pow(tau, 2);

    phi_(i + 3, i + 3) = 1.0;
    phi_(i + 3, i + 6) = tau;

    phi_(i + 6, i + 6) = 1.0;
  }

  // Define the integral term for the control input
  Eigen::Matrix<double, 9, 1> integral;
  integral.setZero();

  integral.segment<3>(0) = (1.0 / 6.0) * pow(tau, 3) * um;  // pos
  integral.segment<3>(3) = 0.5 * pow(tau, 2) * um;          // vel
  integral.segment<3>(6) = tau * um;                        // acc
  state1 = phi_ * state0 + integral;
}

void KinodynamicAstar::convert_path(const std::vector<Eigen::Vector3d>& result,
                                    vec_Vec3f& path)
{
  path.clear();
  path.reserve(result.size());
  for (const auto& vec : result)
  {
    path.push_back(vec);
  }
}

Eigen::MatrixXd KinodynamicAstar::f_DN(const Eigen::Vector3d& x)
{
  double x_norm_2 = x.squaredNorm();
  return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) /
         sqrt(x_norm_2);
}
