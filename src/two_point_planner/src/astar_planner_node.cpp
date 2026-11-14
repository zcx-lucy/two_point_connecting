// ============================================================================
// astar_planner_node.cpp —— RViz 两点最短路径规划节点（C++ 版）
//
// 数据流：
//   nav2_map_server ──/map(OccupancyGrid, transient_local)──▶ 本节点
//   RViz「2D Pose Estimate」──/initialpose──▶ 起点（绿球）
//   RViz「2D Nav Goal」     ──/goal_pose ──▶ 终点（红球）
//   本节点 ──/planned_path(Path)──▶ RViz 绿色路径线
//   本节点 ──/start_goal_markers(MarkerArray)──▶ 起终点球标记
//
// 算法概要：
//   1. 收到地图后先做 chamfer(3-4) 双扫描距离变换，得到每格到最近障碍的
//      近似欧氏距离 dist（米）；
//   2. dist ≤ robot_radius           → 致死层（机器人本体进不去）；
//      robot_radius < dist < inflation_radius
//                                    → 衰减层（代价指数衰减，仅供 A* 使用）；
//   3. 两点都就绪后按 planner 参数选择 JPS（默认）或 A* 搜索最短路径。
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace two_point_planner
{

using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Path;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr double SQRT2 = 1.4142135623730951;

class TwoPointPlanner : public rclcpp::Node
{
public:
  TwoPointPlanner() : Node("astar_planner")
  {
    // ---------------- 参数（与 Python 版完全同名同默认值） ----------------
    // 栅格值 >= occupied_thresh 视为障碍（ROS 约定：0=空闲，100=占据，-1=未知）
    occ_thresh_ = declare_parameter<int>("occupied_thresh", 65);
    // 是否允许穿过未知(-1)区域；false=严格模式（搜不到路时自动放宽一次兜底）
    allow_unknown_ = declare_parameter<bool>("allow_unknown", false);
    // 机器人外切圆半径（米）。参考 RoboMaster：600mm 底盘 × 场地比例 1.25 ≈ 0.38
    robot_radius_ = declare_parameter<double>("robot_radius", 0.38);
    // 代价衰减层外边界（米）
    inflation_radius_ = declare_parameter<double>("inflation_radius", 1.2);
    // 衰减系数 k 与权重 w：penalty = w * exp(-k*(d - robot_radius))
    cost_scaling_ = declare_parameter<double>("cost_scaling_factor", 3.0);
    penalty_weight_ = declare_parameter<double>("penalty_weight", 10.0);
    // 规划器："jps"（默认，跳点搜索，均匀代价）或 "astar"（带衰减层）
    planner_ = declare_parameter<std::string>("planner", "jps");

    // ---------------- 订阅 ----------------
    // 地图是 transient_local（latched）QoS，订阅方必须匹配，否则收不到
    map_sub_ = create_subscription<OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local().reliable(),
      [this](OccupancyGrid::SharedPtr msg) { onMap(msg); });
    start_sub_ = create_subscription<PoseWithCovarianceStamped>(
      "/initialpose", 10,
      [this](PoseWithCovarianceStamped::SharedPtr msg) { onStart(msg); });
    goal_sub_ = create_subscription<PoseStamped>(
      "/goal_pose", 10,
      [this](PoseStamped::SharedPtr msg) { onGoal(msg); });

    // ---------------- 发布 ----------------
    path_pub_ = create_publisher<Path>(
      "/planned_path", rclcpp::QoS(1).transient_local());
    // 路径与标记都用 transient_local：RViz 后启动也能收到最近一次结果
    marker_pub_ = create_publisher<MarkerArray>(
      "/start_goal_markers", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(
      get_logger(),
      "规划器已启动（planner=%s, robot_radius=%.2fm, inflation=%.2fm），"
      "等待 /map 与两个选点…",
      planner_.c_str(), robot_radius_, inflation_radius_);
  }

private:
  // ============================ 回调 ============================

  void onMap(const OccupancyGrid::SharedPtr msg)
  {
    map_ = *msg;
    const int w = static_cast<int>(map_.info.width);
    const int h = static_cast<int>(map_.info.height);
    RCLCPP_INFO(
      get_logger(), "收到地图: %d x %d, 分辨率 %.3f m/格, 原点 (%.2f, %.2f)",
      w, h, map_.info.resolution,
      map_.info.origin.position.x, map_.info.origin.position.y);
    buildCostLayers();
  }

  void onStart(const PoseWithCovarianceStamped::SharedPtr msg)
  {
    start_ = {msg->pose.pose.position.x, msg->pose.pose.position.y};
    RCLCPP_INFO(get_logger(), "设置起点: (%.2f, %.2f)", start_->first, start_->second);
    publishMarkers();
    plan();
  }

  void onGoal(const PoseStamped::SharedPtr msg)
  {
    goal_ = {msg->pose.position.x, msg->pose.position.y};
    RCLCPP_INFO(get_logger(), "设置终点: (%.2f, %.2f)", goal_->first, goal_->second);
    publishMarkers();
    plan();
  }

  // ============================ 代价层 ============================

  // 原始栅格值 → 是否障碍源（严格模式 unknown_as_obstacle=true 时未知区也算）
  bool isOccSource(int8_t v, bool unknown_as_obstacle) const
  {
    if (v < 0) {
      return unknown_as_obstacle;
    }
    return v >= occ_thresh_;
  }

  // chamfer(3-4) 双扫描距离变换 + 致死/衰减分层。
  // C++ 实现 170 万栅格仅需百毫秒级（Python 版约 1.4s），每次收到地图执行一次。
  void buildCostLayers()
  {
    const int w = static_cast<int>(map_.info.width);
    const int h = static_cast<int>(map_.info.height);
    const double res = map_.info.resolution;
    const size_t n = static_cast<size_t>(w) * h;
    const auto t0 = std::chrono::steady_clock::now();

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(n, INF);

    // 严格/宽松两套障碍源（宽松：未知区不算障碍，供兜底用）
    for (int mode = 0; mode < 2; ++mode) {
      const bool unknown_as_obstacle = (mode == 0) ? !allow_unknown_ : false;
      // 多源初始化：障碍格 dist=0
      for (size_t i = 0; i < n; ++i) {
        dist[i] = isOccSource(map_.data[i], unknown_as_obstacle) ? 0.0 : INF;
      }
      // 前向扫描（左上 → 右下），步长权重 (res, res·√2)
      for (int y = 0; y < h; ++y) {
        const size_t base = static_cast<size_t>(y) * w;
        for (int x = 0; x < w; ++x) {
          const size_t i = base + x;
          double d = dist[i];
          if (d == 0.0) {continue;}
          if (x > 0) {d = std::min(d, dist[i - 1] + res);}
          if (y > 0) {
            d = std::min(d, dist[i - w] + res);
            if (x > 0) {d = std::min(d, dist[i - w - 1] + res * SQRT2);}
            if (x < w - 1) {d = std::min(d, dist[i - w + 1] + res * SQRT2);}
          }
          dist[i] = d;
        }
      }
      // 后向扫描（右下 → 左上）
      for (int y = h - 1; y >= 0; --y) {
        const size_t base = static_cast<size_t>(y) * w;
        for (int x = w - 1; x >= 0; --x) {
          const size_t i = base + x;
          double d = dist[i];
          if (d == 0.0) {continue;}
          if (x < w - 1) {d = std::min(d, dist[i + 1] + res);}
          if (y < h - 1) {
            d = std::min(d, dist[i + w] + res);
            if (x < w - 1) {d = std::min(d, dist[i + w + 1] + res * SQRT2);}
            if (x > 0) {d = std::min(d, dist[i + w - 1] + res * SQRT2);}
          }
          dist[i] = d;
        }
      }

      // 由距离场生成致死层 + 衰减层
      auto & blocked = (mode == 0) ? blocked_strict_ : blocked_relaxed_;
      blocked.assign(n, 0);
      size_t lethal_cnt = 0;
      for (size_t i = 0; i < n; ++i) {
        if (dist[i] <= robot_radius_) {
          blocked[i] = 1;
          ++lethal_cnt;
        }
      }
      if (mode == 0) {
        // 衰减层只建一份（与严格/宽松无关）
        penalty_.assign(n, 0.0f);
        for (size_t i = 0; i < n; ++i) {
          const double d = dist[i];
          if (d > robot_radius_ && d < inflation_radius_) {
            penalty_[i] = static_cast<float>(
              penalty_weight_ * std::exp(-cost_scaling_ * (d - robot_radius_)));
          }
        }
        RCLCPP_INFO(
          get_logger(), "代价层构建完成：致死 %zu 格 (robot_radius=%.2fm)，"
          "衰减层至 %.2fm，耗时 %.0f ms",
          lethal_cnt, robot_radius_, inflation_radius_,
          std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count());
      }
    }
    blocked_ = &blocked_strict_;  // 默认严格模式
  }

  // ============================ 坐标与阻挡 ============================

  // 世界坐标(米) → 栅格坐标；越界返回 nullopt
  std::optional<std::pair<int, int>> worldToGrid(double x, double y) const
  {
    const int w = static_cast<int>(map_.info.width);
    const int h = static_cast<int>(map_.info.height);
    const double res = map_.info.resolution;
    const int gx = static_cast<int>((x - map_.info.origin.position.x) / res);
    const int gy = static_cast<int>((y - map_.info.origin.position.y) / res);
    if (0 <= gx && gx < w && 0 <= gy && gy < h) {
      return std::make_pair(gx, gy);
    }
    return std::nullopt;
  }

  bool blockedAt(int x, int y) const
  {
    const int w = static_cast<int>(map_.info.width);
    const int h = static_cast<int>(map_.info.height);
    if (x < 0 || x >= w || y < 0 || y >= h) {
      return true;  // 越界视为不可通行
    }
    return (*blocked_)[static_cast<size_t>(y) * w + x] != 0;
  }

  // ============================ 规划入口 ============================

  void plan()
  {
    if (map_.data.empty()) {
      RCLCPP_WARN(get_logger(), "尚未收到 /map，无法规划");
      return;
    }
    if (!start_ || !goal_) {
      return;  // 两点没齐，等下一个
    }

    auto s = worldToGrid(start_->first, start_->second);
    auto g = worldToGrid(goal_->first, goal_->second);
    if (!s || !g) {
      RCLCPP_WARN(get_logger(), "起点或终点超出地图范围");
      return;
    }

    // 端点落在致死区 → 先尝试放宽未知区重试一次，仍不行才报错
    auto endpoint_ok = [&](const std::pair<int, int> & p) {
        return !blockedAt(p.first, p.second);
      };
    if (!endpoint_ok(*s) || !endpoint_ok(*g)) {
      if (!allow_unknown_ && blocked_ == &blocked_strict_) {
        blocked_ = &blocked_relaxed_;
        if (endpoint_ok(*s) && endpoint_ok(*g)) {
          RCLCPP_WARN(
            get_logger(),
            "端点位于严格致死区（多为未知区膨胀所致），已放宽未知区限制重试");
        } else {
          blocked_ = &blocked_strict_;
          RCLCPP_WARN(
            get_logger(), "起点或终点距障碍不足 %.2fm（膨胀致死区），请重新选择",
            robot_radius_);
          return;
        }
      } else {
        RCLCPP_WARN(
          get_logger(), "起点或终点距障碍不足 %.2fm（膨胀致死区），请重新选择",
          robot_radius_);
        return;
      }
    }

    const auto t0 = std::chrono::steady_clock::now();
    std::optional<std::vector<std::pair<int, int>>> cells =
      (planner_ == "jps") ? jps(*s, *g) : astar(*s, *g);

    // 严格模式无解 → 放宽未知区兜底一次
    if (!cells && !allow_unknown_ && blocked_ == &blocked_strict_) {
      RCLCPP_WARN(
        get_logger(), "严格模式（未知区禁行）未找到路径，放宽未知区后重试…");
      blocked_ = &blocked_relaxed_;
      cells = (planner_ == "jps") ? jps(*s, *g) : astar(*s, *g);
      blocked_ = &blocked_strict_;
      if (cells) {
        RCLCPP_WARN(
          get_logger(),
          "已在放宽未知区的代价层上找到路径（部分路段穿越未知区，请人工确认安全）");
      }
    }
    const double dt_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    if (!cells) {
      RCLCPP_WARN(get_logger(), "%s 未找到可行路径（两点不连通）",
                  planner_ == "jps" ? "JPS" : "A*");
      return;
    }

    // JPS 只返回跳点序列，按直线插值补密，便于 RViz 显示
    std::vector<std::pair<int, int>> dense = densify(*cells);
    publishPath(dense);
    RCLCPP_INFO(
      get_logger(), "[%s] 规划成功：%zu 个路径点，长度约 %.2f 米，耗时 %.1f ms",
      planner_ == "jps" ? "JPS" : "ASTAR",
      dense.size(), pathLength(dense), dt_ms);
  }

  // ============================ A* ============================

  // octile 启发式：八连通栅格下不高于真实代价的最紧下界 → 保证最优
  double heuristic(int x, int y, int gx, int gy) const
  {
    const double dx = std::abs(x - gx);
    const double dy = std::abs(y - gy);
    return std::max(dx, dy) + (SQRT2 - 1.0) * std::min(dx, dy);
  }

  std::optional<std::vector<std::pair<int, int>>> astar(
    std::pair<int, int> s, std::pair<int, int> g)
  {
    const int w = static_cast<int>(map_.info.width);
    const int h = static_cast<int>(map_.info.height);
    const size_t n = static_cast<size_t>(w) * h;
    const size_t si = static_cast<size_t>(s.second) * w + s.first;
    const size_t gi = static_cast<size_t>(g.second) * w + g.first;

    // f 值最小堆：(f, 栅格索引)
    using QItem = std::pair<double, size_t>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;
    std::vector<double> gscore(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    std::vector<uint8_t> closed(n, 0);

    gscore[si] = 0.0;
    open.emplace(heuristic(s.first, s.second, g.first, g.second), si);

    while (!open.empty()) {
      const auto [f, cur] = open.top();
      open.pop();
      if (closed[cur]) {continue;}
      closed[cur] = 1;
      if (cur == gi) {
        return reconstruct(parent, cur, w);
      }
      const int cx = static_cast<int>(cur % w);
      const int cy = static_cast<int>(cur / w);

      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {continue;}
          const int nx = cx + dx, ny = cy + dy;
          if (nx < 0 || nx >= w || ny < 0 || ny >= h) {continue;}
          const size_t ni = static_cast<size_t>(ny) * w + nx;
          if ((*blocked_)[ni] || closed[ni]) {continue;}
          // 斜走禁止切角：两个正交邻居必须都可走
          if (dx != 0 && dy != 0) {
            if ((*blocked_)[static_cast<size_t>(cy) * w + nx] ||
                (*blocked_)[static_cast<size_t>(ny) * w + cx]) {
              continue;
            }
          }
          // 步进代价 ×（1 + 衰减惩罚）：越贴墙越贵，A* 自然走走廊中线
          const double step = (dx != 0 && dy != 0) ? SQRT2 : 1.0;
          const double tentative =
            gscore[cur] + step * (1.0 + static_cast<double>(penalty_[ni]));
          if (tentative < gscore[ni]) {
            gscore[ni] = tentative;
            parent[ni] = static_cast<int>(cur);
            open.emplace(tentative + heuristic(nx, ny, g.first, g.second), ni);
          }
        }
      }
    }
    return std::nullopt;
  }

  // ============================ JPS ============================

  // JPS 跳点搜索（Harabor & Grastien, 2011）：
  // 均匀代价栅格上，大量中间栅格可以从父节点方向“剪枝”掉，
  // 只有出现强迫邻居或到达目标方向的点才需要入队（称为跳点）。
  // 均匀代价下与 A* 同样保证最短，但扩展节点数少几个数量级。
  // 注意：衰减层是非均匀代价，JPS 不适用 —— 所以 JPS 只用致死层。

  int jump(int x, int y, int dx, int dy, int gx, int gy) const
  {
    const int nx = x + dx;
    const int ny = y + dy;
    const int w = static_cast<int>(map_.info.width);
    if (blockedAt(nx, ny)) {
      return -1;
    }
    if (nx == gx && ny == gy) {
      return ny * w + nx;  // 到达终点
    }
    if (dx != 0 && dy != 0) {
      // 对角移动：检查强迫邻居（被挡一侧的斜前方突然可走）
      if ((blockedAt(nx - dx, ny) && !blockedAt(nx - dx, ny + dy)) ||
          (blockedAt(nx, ny - dy) && !blockedAt(nx + dx, ny - dy))) {
        return ny * w + nx;
      }
      // 对角跳跃：两个正交分量任一撞出跳点，当前点也是跳点
      if (jump(nx, ny, dx, 0, gx, gy) >= 0 || jump(nx, ny, 0, dy, gx, gy) >= 0) {
        return ny * w + nx;
      }
    } else if (dx != 0) {
      // 水平移动
      if ((blockedAt(nx, ny + 1) && !blockedAt(nx + dx, ny + 1)) ||
          (blockedAt(nx, ny - 1) && !blockedAt(nx + dx, ny - 1))) {
        return ny * w + nx;
      }
    } else {
      // 垂直移动
      if ((blockedAt(nx + 1, ny) && !blockedAt(nx + 1, ny + dy)) ||
          (blockedAt(nx - 1, ny) && !blockedAt(nx - 1, ny + dy))) {
        return ny * w + nx;
      }
    }
    return jump(nx, ny, dx, dy, gx, gy);  // 继续沿当前方向跳
  }

  std::optional<std::vector<std::pair<int, int>>> jps(
    std::pair<int, int> s, std::pair<int, int> g)
  {
    const int w = static_cast<int>(map_.info.width);
    const size_t n = static_cast<size_t>(w) * map_.info.height;
    const size_t si = static_cast<size_t>(s.second) * w + s.first;
    const size_t gi = static_cast<size_t>(g.second) * w + g.first;

    using QItem = std::pair<double, size_t>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;
    std::vector<double> gscore(n, std::numeric_limits<double>::infinity());
    std::vector<int> parent(n, -1);
    std::vector<uint8_t> closed(n, 0);

    gscore[si] = 0.0;
    open.emplace(heuristic(s.first, s.second, g.first, g.second), si);

    static const int ALL_DIRS[8][2] = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (!open.empty()) {
      const auto [f, cur] = open.top();
      open.pop();
      if (closed[cur]) {continue;}
      closed[cur] = 1;
      if (cur == gi) {
        return reconstruct(parent, cur, w);
      }
      const int cx = static_cast<int>(cur % w);
      const int cy = static_cast<int>(cur / w);
      const int p = parent[cur];

      // ---- 邻居剪枝：确定需要检查的方向集合 ----
      std::vector<std::pair<int, int>> dirs;
      if (p < 0) {
        for (const auto & d : ALL_DIRS) {dirs.emplace_back(d[0], d[1]);}
      } else {
        const int px = p % w;
        const int py = p / w;
        const int dx = (cx > px) - (cx < px);  // sign(cx-px)
        const int dy = (cy > py) - (cy < py);
        if (dx != 0 && dy != 0) {
          // 对角方向：自然邻居
          if (!blockedAt(cx + dx, cy)) {dirs.emplace_back(dx, 0);}
          if (!blockedAt(cx, cy + dy)) {dirs.emplace_back(0, dy);}
          if (!blockedAt(cx + dx, cy + dy)) {dirs.emplace_back(dx, dy);}
          // 强迫邻居
          if (blockedAt(cx - dx, cy) && !blockedAt(cx - dx, cy + dy)) {
            dirs.emplace_back(-dx, dy);
          }
          if (blockedAt(cx, cy - dy) && !blockedAt(cx + dx, cy - dy)) {
            dirs.emplace_back(dx, -dy);
          }
        } else if (dx != 0) {
          // 水平方向
          if (!blockedAt(cx + dx, cy)) {dirs.emplace_back(dx, 0);}
          if (blockedAt(cx, cy + 1) && !blockedAt(cx + dx, cy + 1)) {
            dirs.emplace_back(dx, 1);
          }
          if (blockedAt(cx, cy - 1) && !blockedAt(cx + dx, cy - 1)) {
            dirs.emplace_back(dx, -1);
          }
        } else {
          // 垂直方向
          if (!blockedAt(cx, cy + dy)) {dirs.emplace_back(0, dy);}
          if (blockedAt(cx + 1, cy) && !blockedAt(cx + 1, cy + dy)) {
            dirs.emplace_back(1, dy);
          }
          if (blockedAt(cx - 1, cy) && !blockedAt(cx - 1, cy + dy)) {
            dirs.emplace_back(-1, dy);
          }
        }
      }

      // ---- 沿各方向跳跃，撞出跳点则入队 ----
      for (const auto & d : dirs) {
        const int jp = jump(cx, cy, d.first, d.second, g.first, g.second);
        if (jp < 0) {continue;}
        const size_t ji = static_cast<size_t>(jp);
        if (closed[ji]) {continue;}
        const int jx = jp % w;
        const int jy = jp / w;
        const double cost = std::hypot(
          static_cast<double>(jx - cx), static_cast<double>(jy - cy));
        const double tentative = gscore[cur] + cost;
        if (tentative < gscore[ji]) {
          gscore[ji] = tentative;
          parent[ji] = static_cast<int>(cur);
          open.emplace(tentative + heuristic(jx, jy, g.first, g.second), ji);
        }
      }
    }
    return std::nullopt;
  }

  // ============================ 路径后处理 ============================

  // 由 parent 链回溯路径（栅格坐标序列，起点在前）
  std::vector<std::pair<int, int>> reconstruct(
    const std::vector<int> & parent, size_t cur, int w) const
  {
    std::vector<std::pair<int, int>> cells;
    while (cur != static_cast<size_t>(-1)) {
      cells.emplace_back(static_cast<int>(cur % w), static_cast<int>(cur / w));
      const int p = parent[cur];
      if (p < 0) {break;}
      cur = static_cast<size_t>(p);
    }
    std::reverse(cells.begin(), cells.end());
    return cells;
  }

  // JPS 跳点之间是直线，按栅格分辨率线性插值补密
  std::vector<std::pair<int, int>> densify(
    const std::vector<std::pair<int, int>> & pts) const
  {
    std::vector<std::pair<int, int>> out;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
      const auto [x1, y1] = pts[i];
      const auto [x2, y2] = pts[i + 1];
      const int steps = std::max(std::abs(x2 - x1), std::abs(y2 - y1));
      for (int t = 0; t < steps; ++t) {
        out.emplace_back(
          x1 + static_cast<int>(std::lround((x2 - x1) * 1.0 * t / steps)),
          y1 + static_cast<int>(std::lround((y2 - y1) * 1.0 * t / steps)));
      }
    }
    if (!pts.empty()) {out.push_back(pts.back());}
    return out;
  }

  double pathLength(const std::vector<std::pair<int, int>> & cells) const
  {
    const double res = map_.info.resolution;
    double total = 0.0;
    for (size_t i = 0; i + 1 < cells.size(); ++i) {
      total += std::hypot(
        static_cast<double>(cells[i + 1].first - cells[i].first),
        static_cast<double>(cells[i + 1].second - cells[i].second));
    }
    return total * res;
  }

  // ============================ 发布 ============================

  void publishPath(const std::vector<std::pair<int, int>> & cells)
  {
    const double res = map_.info.resolution;
    const double ox = map_.info.origin.position.x;
    const double oy = map_.info.origin.position.y;

    Path path;
    path.header.frame_id = map_.header.frame_id;
    path.header.stamp = now();
    for (const auto & [gx, gy] : cells) {
      PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = ox + (gx + 0.5) * res;  // 栅格中心
      ps.pose.position.y = oy + (gy + 0.5) * res;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);
  }

  void publishMarkers()
  {
    MarkerArray arr;
    int id = 0;
    auto make_sphere = [&](double x, double y, float r, float g, float b) {
        Marker m;
        m.header.frame_id = map_.data.empty() ? "map" : map_.header.frame_id;
        m.header.stamp = now();
        m.ns = "start_goal";
        m.id = id++;
        m.type = Marker::SPHERE;
        m.action = Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.05;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.70;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0f;
        return m;
      };
    if (start_) {arr.markers.push_back(make_sphere(start_->first, start_->second, 0.1f, 0.9f, 0.1f));}
    if (goal_) {arr.markers.push_back(make_sphere(goal_->first, goal_->second, 0.95f, 0.1f, 0.1f));}
    if (!arr.markers.empty()) {marker_pub_->publish(arr);}
  }

  // ============================ 成员 ============================

  OccupancyGrid map_{};
  std::vector<uint8_t> blocked_strict_;    // 严格致死层（未知区按障碍膨胀）
  std::vector<uint8_t> blocked_relaxed_;   // 宽松致死层（未知区可通行）
  std::vector<uint8_t> * blocked_ = nullptr;  // 当前生效层
  std::vector<float> penalty_;             // 衰减层附加代价（仅 A* 使用）

  std::optional<std::pair<double, double>> start_;
  std::optional<std::pair<double, double>> goal_;

  int occ_thresh_;
  bool allow_unknown_;
  double robot_radius_;
  double inflation_radius_;
  double cost_scaling_;
  double penalty_weight_;
  std::string planner_;

  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace two_point_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<two_point_planner::TwoPointPlanner>());
  rclcpp::shutdown();
  return 0;
}
