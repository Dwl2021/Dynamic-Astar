#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

#include <ros/console.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>

#include "map_util/map_util.h"

// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

class PathNode
{
 public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Matrix<double, 9, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  double time;  // dyn
  int time_idx;
  PathNode* parent;
  char node_state;

  /* -------------------- */
  PathNode()
  {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode() {};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode* PathNodePtr;

class NodeComparator
{
 public:
  bool operator()(PathNodePtr node1, PathNodePtr node2)
  {
    return node1->f_score > node2->f_score;
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  std::size_t operator()(T const& matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^=
          std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable
{
 private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>> data_4d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node)
  {
    data_3d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node)
  {
    data_4d_.insert(
        std::make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx)
  {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx)
  {
    auto iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear()
  {
    data_3d_.clear();
    data_4d_.clear();
  }
};

class KinodynamicAstar
{
 private:
  /* ---------- main data structure ---------- */
  std::vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable expanded_nodes_;
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;
  std::vector<PathNodePtr> path_nodes_;

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_, start_jer_;
  Eigen::Matrix<double, 9, 9> phi_;  // state transit matrix
  // shared_ptr<SDFMap> sdf_map;
  std::shared_ptr<MapUtil<3>> map_util_;
  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_, init_max_tau_;
  double max_vel_, max_acc_, max_jer_;
  double rho_, horizon_, lambda_heu_;
  int allocate_num_, check_num_;
  double tie_breaker_;
  bool optimistic_;

  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;

  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int timeToIndex(double time);
  void retrievePath(PathNodePtr end_node);

  /* shot trajectory */
  std::vector<double> cubic(double a, double b, double c, double d);
  std::vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                       double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);
  double obvpCost(const Vector3d& p_o, const Vector3d& v_o, const Vector3d& a_o,
                  const Vector3d& p_f, const Vector3d& v_f, const Vector3d& a_f,
                  double T);

  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 9, 1>& state0,
                    Eigen::Matrix<double, 9, 1>& state1, Eigen::Vector3d um, double tau);

 public:
  KinodynamicAstar() {};
  ~KinodynamicAstar();

  enum
  {
    REACH_HORIZON = 1,
    REACH_END = 2,
    NO_PATH = 3,
    NEAR_END = 4
  };

  /* main API */
  void setParam(ros::NodeHandle& nh);
  void init();
  void reset();

  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
             Eigen::Vector3d start_j, Eigen::Vector3d end_pt, Eigen::Vector3d end_v,
             Eigen::Vector3d end_a, bool init = false, bool dynamic = false,
             double time_start = -1.0);

  void setMap(const std::shared_ptr<MapUtil<3>>& map_util);

  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);

  // convert from std::vector<Eigen::Vector3d> to vec_Vec3f
  static void convert_path(const std::vector<Eigen::Vector3d>& path, vec_Vec3f& result);

  std::vector<PathNodePtr> getVisitedNodes();

  typedef std::shared_ptr<KinodynamicAstar> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif