#ifndef SRC_PLANNER_PLAN_ENV_INCLUDE_PLAN_ENV_RAYCAST
#define SRC_PLANNER_PLAN_ENV_INCLUDE_PLAN_ENV_RAYCAST

#pragma region include
#pragma region include::project

#pragma endregion include::project
#pragma region include::third
#include <Eigen/Eigen>
#pragma endregion include::third
#pragma region include::standard
#include <vector>
#pragma endregion include::standard
#pragma endregion include

double signum(double x);

double mod(double value, double modulus);

double intbound(double s, double ds);

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
             const Eigen::Vector3d& min, const Eigen::Vector3d& max,
             int& output_points_cnt, Eigen::Vector3d* output);

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
             const Eigen::Vector3d& min, const Eigen::Vector3d& max,
             std::vector<Eigen::Vector3d>* output);

class RayCaster {
 public:
  RayCaster(/* args */) = default;
  ~RayCaster() = default;

  bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  bool step(Eigen::Vector3d& ray_pt);

 private:
  /* data */
  Eigen::Vector3d start_;
  Eigen::Vector3d end_;
  Eigen::Vector3d direction_;
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  int x_;
  int y_;
  int z_;
  int endX_;
  int endY_;
  int endZ_;
  double maxDist_;
  double dx_;
  double dy_;
  double dz_;
  int stepX_;
  int stepY_;
  int stepZ_;
  double tMaxX_;
  double tMaxY_;
  double tMaxZ_;
  double tDeltaX_;
  double tDeltaY_;
  double tDeltaZ_;
  double dist_;

  int step_num_;
};

#endif /* SRC_PLANNER_PLAN_ENV_INCLUDE_PLAN_ENV_RAYCAST */
