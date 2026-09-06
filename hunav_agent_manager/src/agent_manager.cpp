#include "hunav_agent_manager/agent_manager.hpp"

namespace hunav
{

// using std::placeholders::_1;
// using std::placeholders::_2;

AgentManager::AgentManager()
{  //: initialized_(false), max_dist_view_(10.0) {

  init();
  //printf("[AgentManager.Constructor] AgentManager initialized \n");
}

AgentManager::~AgentManager()
{
}

void AgentManager::init()
{
  agents_initialized_ = false;
  robot_initialized_ = false;
  agents_received_ = false;
  robot_received_ = false;
  max_dist_view_ = 10.0;
  time_step_secs_ = 0.0;
  step_count = 1;
  step_count2 = 1;
  move = false;
  // max_dist_view_squared_ = max_dist_view_ * max_dist_view_;
  //printf("[AgentManager.init] initialized \n");
}

float AgentManager::robotSquaredDistance(int id)
{
  // std::lock_guard<std::mutex> guard(mutex_);
  // mutex_.lock();

  // hunav_msgs/msg/Agents
  float xa = agents_[id].sfmAgent.position.getX();  // position.position.x;
  float ya = agents_[id].sfmAgent.position.getY();  // position.y;
  float xr = robot_.sfmAgent.position.getX();       // position.x;
  float yr = robot_.sfmAgent.position.getY();       // position.y;

  //   sfm agents
  //   float xa = agents_[request->agent_id].position.getX();
  //   float ya = agents_[request->agent_id].position.getY();
  //   float xr = robot_.position.getX();
  //   float yr = robot_.position.getY();

  // mutex_.unlock();
  // double d = (xr - xa) * (xr - xa) + (yr - ya) * (yr - ya);
  // std::cout << "AgentManager.robotSquaredDistance:" << sqrt(d) << std::endl;
  return (xr - xa) * (xr - xa) + (yr - ya) * (yr - ya);
}

bool AgentManager::lineOfSight(int id)
{
  // ---------------------------------------------------------------------------
  // ORIGINALLY (upstream): front FOV only — robot must lie within ~±90° of the
  // agent's current yaw (plus 0.17 rad). That breaks Scared: avoidRobot faces
  // AWAY from the robot → next tick IsRobotVisible is false → Fallback to
  // RegularNav → yaw flips toward the robot → Scared again → ~180° thrash.
  // The BT already gates on distance; keep awareness omnidirectional here.
  // ---------------------------------------------------------------------------
  // PATCH (isaac-social-nav): always true once within the distance check.
  (void)id;
  return true;
  // ORIGINALLY:
  // float ax = agents_[id].sfmAgent.position.getX();
  // float ay = agents_[id].sfmAgent.position.getY();
  // float rx = robot_.sfmAgent.position.getX();
  // float ry = robot_.sfmAgent.position.getY();
  // double yaw = agents_[id].sfmAgent.yaw.toRadian();
  // float nrx = (rx - ax) * cos(yaw) + (ry - ay) * sin(yaw);
  // float nry = -(rx - ax) * sin(yaw) + (ry - ay) * cos(yaw);
  // float rangle = atan2(nry, nrx);
  // if (abs(rangle) > (M_PI / 2.0 + 0.17)) return false;
  // return true;
}

bool AgentManager::isRobotVisible(int id, double dist)
{
  std::lock_guard<std::mutex> guard(mutex_);
  float d = sqrt(robotSquaredDistance(id));
  // PATCH (isaac-social-nav): hysteresis / latch.
  // Scared was: flee a bit → unlatch → Regular patrol back in → flee again
  // (GUI: turn away, walk a second, turn back, repeat). For once:true
  // behaviours, keep latched so the BT timer owns the reaction end — no
  // re-arming mid-duration.
  const double release = dist + 1.5;
  if (robot_visible_latched_[id])
  {
    if (agents_[id].behavior.once)
      return true;
    if (d > release)
      robot_visible_latched_[id] = false;
    else
      return true;
  }
  if (d <= dist && lineOfSight(id))
  {
    robot_visible_latched_[id] = true;
    return true;
  }
  return false;
}

// void AgentManager::isRobotVisible(
//     const std::shared_ptr<hunav_msgs::srv::IsRobotVisible::Request>
//     request, std::shared_ptr<hunav_msgs::srv::IsRobotVisible::Response>
//     response) {
//   response->visible = isRobotVisible(request->agent_id)
// }

void AgentManager::lookAtTheRobot(int id)
{
  std::lock_guard<std::mutex> guard(mutex_);
  // Robot position
  float rx = robot_.sfmAgent.position.getX();
  float ry = robot_.sfmAgent.position.getY();
  // Agent position
  float ax = agents_[id].sfmAgent.position.getX();
  float ay = agents_[id].sfmAgent.position.getY();
  // PATCH (isaac-social-nav): absolute world heading toward robot (see approximateRobot).
  utils::Angle face;
  face.setRadian(atan2(ry - ay, rx - ax));
  agents_[id].sfmAgent.yaw = face;
  // PATCH (isaac-social-nav): freeze must clear linear velocity — otherwise Isaac
  // keeps WalkLoop playing while position never updates (look-only / no updatePosition).
  agents_[id].sfmAgent.velocity.set(0.0, 0.0);
  agents_[id].sfmAgent.linearVelocity = 0.0;
  agents_[id].sfmAgent.angularVelocity = 0.0;
  agents_[id].behavior.state = 1;
}

void AgentManager::approximateRobot(int id, double dt, double closest_dist, double max_vel)
{
  std::lock_guard<std::mutex> guard(mutex_);

  agents_[id].behavior.state = 1;

  // Robot position
  float rx = robot_.sfmAgent.position.getX();
  float ry = robot_.sfmAgent.position.getY();
  float dist = sqrt(robotSquaredDistance(id));

  // PATCH (isaac-social-nav): hysteresis around stop distance. Without it the
  // agent surges in (goal=robot), crosses closest_dist, stops, drifts out,
  // surges again → sudden speed-up + thrash at ~1.5 m.
  const double release_dist = closest_dist + 2.5;
  bool holding = curious_holding_[id];
  if (holding)
  {
    if (dist > release_dist)
    {
      curious_holding_[id] = false;
      curious_hold_xy_.erase(id);
      holding = false;
    }
  }
  else if (dist <= closest_dist)
  {
    curious_holding_[id] = true;
    holding = true;
    float ax = agents_[id].sfmAgent.position.getX();
    float ay = agents_[id].sfmAgent.position.getY();
    // Pin on the stop ring (not closer) so we never walk into the robot.
    if (dist > 1e-3)
    {
      float s = static_cast<float>(closest_dist / dist);
      curious_hold_xy_[id] = {rx + (ax - rx) * s, ry + (ay - ry) * s};
    }
    else
    {
      curious_hold_xy_[id] = {ax, ay};
    }
  }

  if (holding)
  {
    auto hold = curious_hold_xy_[id];
    agents_[id].sfmAgent.position.set(hold.first, hold.second);
    float ax = hold.first;
    float ay = hold.second;
    utils::Angle face;
    face.setRadian(atan2(ry - ay, rx - ax));
    agents_[id].sfmAgent.yaw = face;
    agents_[id].sfmAgent.velocity.set(0.0, 0.0);
    agents_[id].sfmAgent.linearVelocity = 0.0;
    agents_[id].sfmAgent.angularVelocity = 0.0;
  }
  else
  {
    float ax0 = agents_[id].sfmAgent.position.getX();
    float ay0 = agents_[id].sfmAgent.position.getY();

    sfm::Goal g;
    g.center.set(rx, ry);
    g.radius = robot_.sfmAgent.radius;
    agents_[id].sfmAgent.goals.push_front(g);

    float ini_desired_vel = agents_[id].sfmAgent.desiredVelocity;
    float span = std::max(1.0f, max_dist_view_ - static_cast<float>(closest_dist));
    float t = (dist - static_cast<float>(closest_dist)) / span;
    if (t < 0.05f)
      t = 0.05f;
    if (t > 1.0f)
      t = 1.0f;
    agents_[id].sfmAgent.desiredVelocity = max_vel * t;

    computeForces(id);
    sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);

    // Radial-only step + real planar velocity so WalkLoop plays (zeroing
    // velocity.linear caused medical to slide with no animation).
    {
      float ax1 = agents_[id].sfmAgent.position.getX();
      float ay1 = agents_[id].sfmAgent.position.getY();
      float sx = ax1 - ax0;
      float sy = ay1 - ay0;
      float tox = rx - ax0;
      float toy = ry - ay0;
      float tn2 = tox * tox + toy * toy;
      float rad = 0.0f;
      if (tn2 > 1e-6f)
      {
        rad = (sx * tox + sy * toy) / tn2;
        if (rad < 0.0f)
          rad = 0.0f;
        // Cap step so we do not overshoot the stop ring in one tick.
        float max_rad = 1.0f;
        if (dist > closest_dist && tn2 > 1e-6f)
        {
          float room = static_cast<float>((dist - closest_dist) / sqrtf(tn2));
          if (room < max_rad)
            max_rad = std::max(0.0f, room);
        }
        if (rad > max_rad)
          rad = max_rad;
        agents_[id].sfmAgent.position.set(ax0 + rad * tox, ay0 + rad * toy);
      }
      float vx = (agents_[id].sfmAgent.position.getX() - ax0) /
                 std::max(static_cast<float>(dt), 1e-3f);
      float vy = (agents_[id].sfmAgent.position.getY() - ay0) /
                 std::max(static_cast<float>(dt), 1e-3f);
      utils::Angle face;
      face.setRadian(atan2(ry - ay0, rx - ax0));
      agents_[id].sfmAgent.yaw = face;
      agents_[id].sfmAgent.velocity.set(vx, vy);
      agents_[id].sfmAgent.linearVelocity = sqrtf(vx * vx + vy * vy);
      agents_[id].sfmAgent.angularVelocity = 0.0;
    }

    agents_[id].sfmAgent.goals.pop_front();
    agents_[id].sfmAgent.desiredVelocity = ini_desired_vel;
  }
}

void AgentManager::blockRobot(int id, double dt, double front_dist)
{
  std::lock_guard<std::mutex> guard(mutex_);

  agents_[id].behavior.state = 1;

  float rx = robot_.sfmAgent.position.getX();
  float ry = robot_.sfmAgent.position.getY();
  float h = robot_.sfmAgent.yaw.toRadian();
  float rvx = robot_.sfmAgent.velocity.getX();
  float rvy = robot_.sfmAgent.velocity.getY();
  if ((rvx * rvx + rvy * rvy) > (0.05f * 0.05f))
    h = atan2(rvy, rvx);
  float ax = agents_[id].sfmAgent.position.getX();
  float ay = agents_[id].sfmAgent.position.getY();
  float dist_robot = sqrtf((rx - ax) * (rx - ax) + (ry - ay) * (ry - ay));

  // Stand-off in front of the robot (YAML behavior.dist, ~1.4 m).
  //
  // PATCH (isaac-social-nav): ROS yaw uses (cos, sin). Upstream sin/cos is
  // 90° off (yaw 0 = +X). An eastbound robot would place "front" 1.4 m north.
  // Holding then teleports XY there every tick (no PhysX) → wall clip into
  // adjacent rooms. Isaac robot.yaw is ROS euler. While |v| > 0.05 use travel
  // heading atan2(vy, vx) so we block where the chassis is going, not a
  // stale quaternion.
  //
  // Not changed here (kinematic SFM limits, still true): holding still
  // teleports to (newgx,newgy) with v=0 so the body slides with no WalkLoop;
  // a 180° MPPI/yaw flip pops them behind then ahead; once/duration expiry
  // falls back to RegularNav and they return to the YAML goal.
  float newgx = rx + front_dist * cosf(h);
  float newgy = ry + front_dist * sinf(h);
  float dist_goal =
      sqrtf((newgx - ax) * (newgx - ax) + (newgy - ay) * (newgy - ay));

  // PATCH (isaac-social-nav): once close enough to the block pose OR the robot,
  // pin and face the robot. Previously kept chasing the front goal → walked
  // into Stretch and strafed with yaw locked (male_adult_police_04).
  const float stop_r = std::max(0.35f, static_cast<float>(front_dist) * 0.35f);
  bool holding = threatening_holding_[id];
  if (!holding && (dist_goal <= stop_r || dist_robot <= front_dist + 0.15f))
  {
    threatening_holding_[id] = true;
    holding = true;
    // Prefer standing on the front goal if reachable; else current XY.
    if (dist_goal < 1.5f)
      threatening_hold_xy_[id] = {newgx, newgy};
    else
      threatening_hold_xy_[id] = {ax, ay};
  }

  if (holding)
  {
    auto hold = threatening_hold_xy_[id];
    // Keep stand-off in front of the *current* robot pose each tick.
    hold = {newgx, newgy};
    threatening_hold_xy_[id] = hold;
    agents_[id].sfmAgent.position.set(hold.first, hold.second);
    utils::Angle face;
    face.setRadian(atan2(ry - hold.second, rx - hold.first));
    agents_[id].sfmAgent.yaw = face;
    agents_[id].sfmAgent.velocity.set(0.0, 0.0);
    agents_[id].sfmAgent.linearVelocity = 0.0;
    agents_[id].sfmAgent.angularVelocity = 0.0;
    return;
  }

  std::list<sfm::Goal> gls = agents_[id].sfmAgent.goals;
  sfm::Goal g;
  g.center.set(newgx, newgy);
  g.radius = 0.25;
  agents_[id].sfmAgent.goals.push_front(g);

  float ini_desired_vel = agents_[id].sfmAgent.desiredVelocity;
  agents_[id].sfmAgent.desiredVelocity = 0.85;

  float ax0 = ax;
  float ay0 = ay;
  computeForces(id);
  sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);

  // Face robot; feed planar velocity for WalkLoop.
  {
    float ax1 = agents_[id].sfmAgent.position.getX();
    float ay1 = agents_[id].sfmAgent.position.getY();
    float vx = (ax1 - ax0) / std::max(static_cast<float>(dt), 1e-3f);
    float vy = (ay1 - ay0) / std::max(static_cast<float>(dt), 1e-3f);
    utils::Angle face;
    face.setRadian(atan2(ry - ay1, rx - ax1));
    agents_[id].sfmAgent.yaw = face;
    agents_[id].sfmAgent.velocity.set(vx, vy);
    agents_[id].sfmAgent.linearVelocity = sqrtf(vx * vx + vy * vy);
    agents_[id].sfmAgent.angularVelocity = 0.0;
  }

  agents_[id].sfmAgent.goals = gls;
  agents_[id].sfmAgent.desiredVelocity = ini_desired_vel;
}

void AgentManager::avoidRobot(int id, double dt, double scary_factor_force, double max_vel)
{
  std::lock_guard<std::mutex> guard(mutex_);

  agents_[id].behavior.state = 1;

  // we decrease the maximum velocity
  double init_vel = agents_[id].sfmAgent.desiredVelocity;
  agents_[id].sfmAgent.desiredVelocity = max_vel;

  // PATCH (isaac-social-nav): replace patrol goals with a short flee goal away
  // from the robot. Upstream only added repulsion while still chasing the
  // waypoint → goal vs scary cancel → stand still, then eventually drift off.
  std::list<sfm::Goal> saved_goals = agents_[id].sfmAgent.goals;
  utils::Vector2d away = agents_[id].sfmAgent.position - robot_.sfmAgent.position;
  double away_n = away.norm();
  if (away_n < 1e-3)
  {
    // Coincident / on top of robot — pick an arbitrary away axis.
    away.set(1.0, 0.0);
    away_n = 1.0;
  }
  away = away / away_n;
  sfm::Goal flee;
  flee.center.set(
      agents_[id].sfmAgent.position.getX() + away.getX() * 5.0,
      agents_[id].sfmAgent.position.getY() + away.getY() * 5.0);
  flee.radius = 0.4;
  agents_[id].sfmAgent.goals.clear();
  agents_[id].sfmAgent.goals.push_front(flee);

  computeForces(id);

  // We add an extra repulsive force from the robot
  utils::Vector2d minDiff = agents_[id].sfmAgent.position - robot_.sfmAgent.position;
  double distance = minDiff.norm() - agents_[id].sfmAgent.radius;
  if (distance < 0.15)
  {
    distance = 0.15;  // avoid huge 1/distance spikes / nan
  }

  utils::Vector2d Scaryforce =
      scary_factor_force * (agents_[id].sfmAgent.params.forceSigmaObstacle / distance) * minDiff.normalized();
  agents_[id].sfmAgent.forces.globalForce += Scaryforce;

  // update position
  sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);

  // PATCH (isaac-social-nav): absolute flee heading with slew limit.
  // Instant face-away snaps ~180° when the robot teleports / passes the agent
  // (headless probe waypoint jumps showed yaw>90 thrash on Scared).
  {
    float ax = agents_[id].sfmAgent.position.getX();
    float ay = agents_[id].sfmAgent.position.getY();
    float rx = robot_.sfmAgent.position.getX();
    float ry = robot_.sfmAgent.position.getY();
    float desired = atan2(ay - ry, ax - rx);
    float current = agents_[id].sfmAgent.yaw.toRadian();
    float diff = desired - current;
    while (diff > M_PI)
      diff -= 2.0f * static_cast<float>(M_PI);
    while (diff < -M_PI)
      diff += 2.0f * static_cast<float>(M_PI);
    const float max_step = 0.40f;  // ~23 deg / tick
    if (diff > max_step)
      diff = max_step;
    else if (diff < -max_step)
      diff = -max_step;
    utils::Angle face_away;
    face_away.setRadian(current + diff);
    agents_[id].sfmAgent.yaw = face_away;
    agents_[id].sfmAgent.angularVelocity = 0.0;
  }

  // restore desired vel and patrol goals
  agents_[id].sfmAgent.desiredVelocity = init_vel;
  agents_[id].sfmAgent.goals = saved_goals;
}

bool AgentManager::goalReached(int id)
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (!agents_[id].sfmAgent.goals.empty() &&
      (agents_[id].sfmAgent.goals.front().center - agents_[id].sfmAgent.position).norm() <=
          (agents_[id].sfmAgent.goals.front().radius + 0.1))
  {
    // printf("Agent %s goal reached!", agents_[id].name.c_str());
    return true;
  }
  else
    return false;
}

bool AgentManager::updateGoal(int id)
{
  std::lock_guard<std::mutex> guard(mutex_);

  // printf("Updating goal for agent %i\n\n", id);
  sfm::Goal g = agents_[id].sfmAgent.goals.front();
  agents_[id].sfmAgent.goals.pop_front();
  if (agents_[id].sfmAgent.cyclicGoals)
  {
    agents_[id].sfmAgent.goals.push_back(g);
  }
  return true;
}

void AgentManager::initializeAgents(const hunav_msgs::msg::Agents::SharedPtr msg)
{
  printf("Initializing SFM Agents...\n");
  for (auto a : msg->agents)
  {
    agent ag;
    ag.name = a.name;
    ag.type = a.type;

    // behavior
    ag.behavior.type = a.behavior.type;
    ag.behavior.state = a.behavior.state;
    ag.behavior.configuration = a.behavior.configuration;  // def, manual, random
    ag.behavior.duration = a.behavior.duration;
    ag.behavior.once = a.behavior.once;
    ag.behavior.vel = a.behavior.vel;
    ag.behavior.dist = a.behavior.dist;
    ag.behavior.forceFactor = a.behavior.other_force_factor;

    ag.sfmAgent.id = a.id;
    ag.sfmAgent.groupId = a.group_id;
    ag.sfmAgent.desiredVelocity = a.desired_velocity;
    ag.sfmAgent.radius = a.radius;
    ag.sfmAgent.cyclicGoals = a.cyclic_goals;
    ag.sfmAgent.position.set(a.position.position.x, a.position.position.y);
    ag.sfmAgent.yaw.setRadian(a.yaw);
    ag.sfmAgent.velocity.set(a.velocity.linear.x, a.velocity.linear.y);
    ag.sfmAgent.linearVelocity =
        sqrt(a.velocity.linear.x * a.velocity.linear.x + a.velocity.linear.y * a.velocity.linear.y);
    ag.sfmAgent.angularVelocity = a.velocity.angular.z;
    for (auto g : a.goals)
    {
      sfm::Goal sfmg;
      sfmg.center.setX(g.position.x);
      sfmg.center.setY(g.position.y);
      sfmg.radius = a.goal_radius;
      ag.sfmAgent.goals.push_back(sfmg);
    }
    ag.sfmAgent.obstacles1.clear();
    if (!a.closest_obs.empty())
    {
      for (auto obs : a.closest_obs)
      {
        utils::Vector2d o;
        o.set(obs.x, obs.y);
        ag.sfmAgent.obstacles1.push_back(o);
      }
    }
    ag.sfmAgent.params.forceFactorSocial = a.behavior.social_force_factor;  // 5.0; // 2.1 by default
    ag.sfmAgent.params.forceFactorDesired = a.behavior.goal_force_factor;
    ag.sfmAgent.params.forceFactorObstacle = a.behavior.obstacle_force_factor;

    agents_[ag.sfmAgent.id] = ag;
    printf("\tagent %s, x:%.2f, y:%.2f, th:%.2f\n", agents_[ag.sfmAgent.id].name.c_str(),
           agents_[ag.sfmAgent.id].sfmAgent.position.getX(), agents_[ag.sfmAgent.id].sfmAgent.position.getY(),
           agents_[ag.sfmAgent.id].sfmAgent.yaw.toRadian());
  }
  agents_initialized_ = true;
  printf("SFM Agents initialized\n");
}

void AgentManager::initializeRobot(const hunav_msgs::msg::Agent::SharedPtr msg)
{
  robot_.name = msg->name;
  robot_.type = msg->type;
  // robot_.behavior = msg->behavior;
  robot_.sfmAgent.id = msg->id;
  robot_.sfmAgent.groupId = msg->group_id;
  robot_.sfmAgent.desiredVelocity = msg->desired_velocity;
  robot_.sfmAgent.radius = msg->radius;
  robot_.sfmAgent.cyclicGoals = msg->cyclic_goals;
  robot_.sfmAgent.position.set(msg->position.position.x, msg->position.position.y);
  robot_.sfmAgent.yaw.setRadian(msg->yaw);
  robot_.sfmAgent.velocity.set(msg->velocity.linear.x, msg->velocity.linear.y);
  robot_.sfmAgent.linearVelocity =
      sqrt(msg->velocity.linear.x * msg->velocity.linear.x + msg->velocity.linear.y * msg->velocity.linear.y);
  robot_.sfmAgent.angularVelocity = msg->velocity.angular.z;

  printf("\trobot %i, x:%.2f, y:%.2f\n", robot_.sfmAgent.id, robot_.sfmAgent.position.getX(),
         robot_.sfmAgent.position.getY());

  robot_initialized_ = true;
  printf("SFM Robot initialized\n");
}

bool AgentManager::updateAgents(const hunav_msgs::msg::Agents::SharedPtr msg)
{
  // update velocities
  // if (agents_initialized_ && time_step_secs_ > 0.0) {

  // Update agents velocites only in the ROS cycle
  for (auto a : msg->agents)
  {
    // velocities computed by myself
    // double anvel =
    //     normalizeAngle(a.yaw - agents_[a.id].sfmAgent.yaw.toRadian()) /
    //     (time_step_secs_); // * step_count);
    // agents_[a.id].sfmAgent.angularVelocity = anvel;
    // double xi = agents_[a.id].sfmAgent.position.getX();
    // double yi = agents_[a.id].sfmAgent.position.getY();
    // double xf = a.position.position.x;
    // double yf = a.position.position.y;
    // double dist = sqrt((xf - xi) * (xf - xi) + (yf - yi) * (yf - yi));
    // agents_[a.id].sfmAgent.linearVelocity =
    //     dist / (time_step_secs_);              // * step_count);
    // double vx = (xf - xi) / (time_step_secs_); // * step_count);
    // double vy = (yf - yi) / (time_step_secs_); // * step_count);
    // agents_[a.id].sfmAgent.velocity.set(vx, vy);

    // position
    agents_[a.id].sfmAgent.position.set(a.position.position.x, a.position.position.y);
    agents_[a.id].sfmAgent.yaw.setRadian(a.yaw);

    // velocities
    agents_[a.id].sfmAgent.velocity.set(a.velocity.linear.x, a.velocity.linear.y);
    agents_[a.id].sfmAgent.linearVelocity =
        sqrt(a.velocity.linear.x * a.velocity.linear.x + a.velocity.linear.y * a.velocity.linear.y);
    agents_[a.id].sfmAgent.angularVelocity = a.velocity.angular.z;

    // update goals
    // agents_[a.id].sfmAgent.goals.clear();
    // for (auto g : a.goals) {
    //   sfm::Goal sfmg;
    //   sfmg.center.setX(g.position.x);
    //   sfmg.center.setY(g.position.y);
    //   sfmg.radius = a.goal_radius;
    //   agents_[a.id].sfmAgent.goals.push_back(sfmg);
    // }

    // update closest obstacles
    agents_[a.id].sfmAgent.obstacles1.clear();
    if (!a.closest_obs.empty())
    {
      for (auto obs : a.closest_obs)
      {
        utils::Vector2d o;
        o.set(obs.x, obs.y);
        agents_[a.id].sfmAgent.obstacles1.push_back(o);
      }
    }

    // printf("%s updating agents vels computed lv:%.3f, lvx:%.3f, lvy:%.3f, "
    //        "av:%.3f\n",
    //        a.name.c_str(), agents_[a.id].sfmAgent.linearVelocity,
    //        agents_[a.id].sfmAgent.velocity.getX(),
    //        agents_[a.id].sfmAgent.velocity.getY(),
    //        agents_[a.id].sfmAgent.angularVelocity);
  }
  step_count = 1;
  return true;

  // } else {
  //   step_count++;
  //   return false;
  // }
}

void AgentManager::updateAgentRobot(const hunav_msgs::msg::Agent::SharedPtr msg)
{
  // Update robot
  robot_.sfmAgent.position.set(msg->position.position.x, msg->position.position.y);
  // tf2::Quaternion q(
  //     robot_.position.orientation.x, robot_.position.orientation.y,
  //     robot_.position.orientation.z, robot_.position.orientation.w);
  // tf2::Matrix3x3 m(q);
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  robot_.sfmAgent.yaw.setRadian(msg->yaw);
  robot_.sfmAgent.velocity.set(msg->velocity.linear.x, msg->velocity.linear.y);
  robot_.sfmAgent.linearVelocity =
      sqrt(msg->velocity.linear.x * msg->velocity.linear.x + msg->velocity.linear.y * msg->velocity.linear.y);
  robot_.sfmAgent.angularVelocity = msg->velocity.angular.z;
}

// void AgentManager::robotCallback(
//     const hunav_msgs::msg::Agent::SharedPtr msg) const {
//   robot_ = *msg;
//   if (!robot_initialized_) {
//     initializeSFMRobot();
//     robot_initialized_ = true;
//   } else {
//     updateSFMRobot();
//   }
// }

hunav_msgs::msg::Agent AgentManager::getUpdatedAgentMsg(int id)
{
  // std::lock_guard<std::mutex> guard(mutex_);
  hunav_msgs::msg::Agent a;
  a.id = agents_[id].sfmAgent.id;
  a.name = agents_[id].name;
  a.type = agents_[id].type;
  a.position.position.x = agents_[id].sfmAgent.position.getX();
  a.position.position.y = agents_[id].sfmAgent.position.getY();
  a.yaw = agents_[id].sfmAgent.yaw.toRadian();
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, agents_[id].sfmAgent.yaw.toRadian());
  a.position.orientation = tf2::toMsg(myQuaternion);
  a.linear_vel = agents_[id].sfmAgent.linearVelocity;
  a.angular_vel = agents_[id].sfmAgent.angularVelocity;
  a.velocity.linear.x = agents_[id].sfmAgent.velocity.getX();
  a.velocity.linear.y = agents_[id].sfmAgent.velocity.getY();
  a.velocity.angular.z = agents_[id].sfmAgent.angularVelocity;
  // behavior
  a.behavior.type = agents_[id].behavior.type;
  a.behavior.state = agents_[id].behavior.state;
  a.behavior.configuration = agents_[id].behavior.configuration;  // def, manual, random
  a.behavior.duration = agents_[id].behavior.duration;
  a.behavior.once = agents_[id].behavior.once;
  a.behavior.vel = agents_[id].behavior.vel;
  a.behavior.dist = agents_[id].behavior.dist;
  a.behavior.other_force_factor = agents_[id].behavior.forceFactor;
  a.behavior.social_force_factor = agents_[id].sfmAgent.params.forceFactorSocial;  // 5.0; // 2.1 by default
  a.behavior.goal_force_factor = agents_[id].sfmAgent.params.forceFactorDesired;
  a.behavior.obstacle_force_factor = agents_[id].sfmAgent.params.forceFactorObstacle;

  for (auto g : agents_[id].sfmAgent.goals)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = g.center.getX();
    p.position.y = g.center.getY();
    a.goals.push_back(p);
  }

  //(agents[i].goals.front().center - agents[i].position).norm() <=
  //          agents[i].goals.front().radius)
  // robot_received_ = false;
  // agents_received_ = false;
  return a;
}

// create a agents msg from the sfm_agents_
hunav_msgs::msg::Agents AgentManager::getUpdatedAgentsMsg()
{
  // Make a copy of agents_
  // Then update them with the sfm_agents_ data
  std::lock_guard<std::mutex> guard(mutex_);
  hunav_msgs::msg::Agents agents_msg;
  agents_msg.header = header_;

  std::unordered_map<int, agent>::iterator itr;
  for (itr = agents_.begin(); itr != agents_.end(); itr++)
  {
    hunav_msgs::msg::Agent a = getUpdatedAgentMsg(itr->first);
    agents_msg.agents.push_back(a);
  }
  robot_received_ = false;
  agents_received_ = false;
  return agents_msg;
}

std::vector<sfm::Agent> AgentManager::getSFMAgents()
{
  std::vector<sfm::Agent> agents;
  std::unordered_map<int, agent>::iterator itr;
  for (itr = agents_.begin(); itr != agents_.end(); itr++)
  {
    agents.push_back(itr->second.sfmAgent);
  }
  return agents;
}

void AgentManager::computeForces(int id)
{
  std::vector<sfm::Agent> otherAgents = getSFMAgents();

  utils::Vector2d ob;
  if (agents_[id].behavior.state == 1)
  {
    switch (agents_[id].behavior.type)
    {
      case hunav_msgs::msg::AgentBehavior::BEH_REGULAR:
        // We add the robot as another human agent.
        otherAgents.push_back(robot_.sfmAgent);
        sfm::SFM.computeForces(agents_[id].sfmAgent, otherAgents);
        break;
      case hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE:
        // the human treats the robot like an obstacle.
        // We add the robot to the obstacles of this agent.
        ob.set(robot_.sfmAgent.position.getX(), robot_.sfmAgent.position.getY());
        agents_[id].sfmAgent.obstacles1.push_back(ob);
        sfm::SFM.computeForces(agents_[id].sfmAgent, otherAgents);
        break;
      default:
        // Compute forces as usual (not taking into account the robot)
        sfm::SFM.computeForces(agents_[id].sfmAgent, otherAgents);
    }
    // PATCH (isaac-social-nav): do NOT clear behavior.state here.
    // Scared/Curious/Threatening call computeForces mid-reaction; zeroing
    // state made getUpdatedAgents report idle so viewport REACTING never
    // showed (only Threatening hold skipped computeForces). Regular nav
    // clears state in updatePosition().
  }
  else
  {
    otherAgents.push_back(robot_.sfmAgent);
    sfm::SFM.computeForces(agents_[id].sfmAgent, otherAgents);
  }

  bool compute = false;
  if (isnan(agents_[id].sfmAgent.forces.desiredForce.norm()))
  {
    printf("[AgentManager.ComputeForces] \tgoal force is nan. Using zero.. \n");
    agents_[id].sfmAgent.forces.desiredForce.set(0, 0);
    compute = true;
  }
  if (isnan(agents_[id].sfmAgent.forces.groupForce.norm()))
  {
    printf(
        "[AgentManager.ComputeForces] \tgroup force is nan. Using zero.. "
        "\n");
    agents_[id].sfmAgent.forces.groupForce.set(0, 0);
    compute = true;
  }
  if (isnan(agents_[id].sfmAgent.forces.obstacleForce.norm()))
  {
    printf(
        "[AgentManager.ComputeForces] \tobstacle force is nan. Using "
        "zero.. \n");
    agents_[id].sfmAgent.forces.obstacleForce.set(0, 0);
    compute = true;
  }
  if (isnan(agents_[id].sfmAgent.forces.socialForce.norm()))
  {
    // printf(
    //     "[AgentManager.ComputeForces] \tsocial force is nan. Using "
    //     "zero.. \n");
    agents_[id].sfmAgent.forces.socialForce.set(0, 0);
    compute = true;
  }
  if (compute)
  {
    agents_[id].sfmAgent.forces.globalForce =
        agents_[id].sfmAgent.forces.desiredForce + agents_[id].sfmAgent.forces.socialForce +
        agents_[id].sfmAgent.forces.obstacleForce + agents_[id].sfmAgent.forces.groupForce;
  }
  // if(id == 1)
  // {
  //   printf("[AgentManager.ComputeForces] \tForces. global:%.4f, goal:%.4f "
  //          "soc:%.4f, "
  //          "obs:%.4f \n",
  //          agents_[id].sfmAgent.forces.globalForce.norm(),
  //          agents_[id].sfmAgent.forces.desiredForce.norm(),
  //          agents_[id].sfmAgent.forces.socialForce.norm(),
  //          agents_[id].sfmAgent.forces.obstacleForce.norm());
  // }
}

void AgentManager::computeForces()
{
  std::vector<sfm::Agent> otherAgents = getSFMAgents();

  std::unordered_map<int, agent>::iterator itr;
  for (itr = agents_.begin(); itr != agents_.end(); itr++)
  {
    // printf("[AgentManager.ComputeForces] Agent %s, x:%.2f, y:%.2f
    // dvel:%.2f\n",
    //        itr->second.name.c_str(), itr->second.sfmAgent.position.getX(),
    //        itr->second.sfmAgent.position.getY(),
    //        itr->second.sfmAgent.desiredVelocity);

    computeForces(itr->second.sfmAgent.id);

    // printf("[AgentManager.ComputeForces] \tForces. global:%.4f, goal:%.4f "
    //        "soc:%.4f, "
    //        "obs:%.4f \n",
    //        itr->second.sfmAgent.forces.globalForce.norm(),
    //        itr->second.sfmAgent.forces.desiredForce.norm(),
    //        itr->second.sfmAgent.forces.socialForce.norm(),
    //        itr->second.sfmAgent.forces.obstacleForce.norm());
  }
}

void AgentManager::updateAllAgents(const hunav_msgs::msg::Agent::SharedPtr robot_msg,
                                   const hunav_msgs::msg::Agents::SharedPtr agents_msg)
{
  std::lock_guard<std::mutex> guard(mutex_);

  // printf("[AgentManager.updateAllAgents] Receiving agents from
  // simulator...\n"); for (auto &a : agents_msg->agents) {
  //   printf("[AgentManager.updateAllAgents]\tagent %s\n", a.name.c_str());
  // }

  header_ = agents_msg->header;

  if (!robot_initialized_)
  {
    initializeRobot(robot_msg);
  }

  if (!agents_initialized_)
  {
    // initializeBehaviorTree();
    initializeAgents(agents_msg);
    // prev_time_ = agents_msg->header.stamp;
    // time_step_secs_ = 0.0;
    // computeForces();
  }
  else if (robot_initialized_ && agents_initialized_)
  {
    // time_step_secs_ =
    //     (rclcpp::Time(agents_msg->header.stamp) - prev_time_).seconds();
    // printf("AgentManager. time_step_secs_: %.5f\n", time_step_secs_);
    // if (time_step_secs_ < 0.009) {
    //   return;
    // }
    // prev_time_ = agents_msg->header.stamp;
    updateAgentRobot(robot_msg);
    move = updateAgents(agents_msg);
  }
  agents_received_ = true;
  robot_received_ = true;
  computeForces();
}

void AgentManager::updateAgentsAndRobot(const hunav_msgs::msg::Agents::SharedPtr agents_msg)
{
  std::lock_guard<std::mutex> guard(mutex_);

  // printf("[AgentManager.updateAgentsAndRobot] Receiving agents from "
  //        "simulator...\n");

  header_ = agents_msg->header;

  // The robot is the last agent of the vector!
  // or we could look for the type "robot" in the vector
  hunav_msgs::msg::Agent::SharedPtr rob = std::make_shared<hunav_msgs::msg::Agent>(agents_msg->agents.back());

  // we remove the robot from the agents vector
  hunav_msgs::msg::Agents ags = *agents_msg;
  ags.agents.pop_back();

  if (!robot_initialized_)
  {
    initializeRobot(rob);
  }
  if (!agents_initialized_)
  {
    initializeAgents(std::make_shared<hunav_msgs::msg::Agents>(ags));
    // prev_time_ = agents_msg->header.stamp;
    // time_step_secs_ = 0.0;
  }
  else
  {
    // // time_step_secs_ = 0.001;
    // time_step_secs_ =
    //     (rclcpp::Time(agents_msg->header.stamp) - prev_time_).seconds();
    // if (time_step_secs_ < 0.009) {
    //   return;
    // }
    // prev_time_ = agents_msg->header.stamp;
    updateAgentRobot(rob);
    move = updateAgents(std::make_shared<hunav_msgs::msg::Agents>(ags));
  }

  agents_received_ = true;
  robot_received_ = true;
  computeForces();
}

bool AgentManager::canCompute()
{
  if (agents_received_ && robot_received_)
    return true;
  else
    return false;
}

// bool AgentManager::running() {
//   printf("[agentManager.running] running called!\n");
//   std::lock_guard<std::mutex> guard(mutex_);
//   if (std::find(agent_status_.begin(), agent_status_.end(), false) ==
//       agent_status_.end()) {
//     agent_status_.clear();
//     agent_status_.assign(agents_.size(), false);
//     printf("[agentManager.running] return false - task completed\n");
//     return false;
//   } else {
//     printf("[agentManager.running] return true - task running\n");
//     return true;
//   }
// }

void AgentManager::updatePosition(int id, double dt)
{
  std::lock_guard<std::mutex> guard(mutex_);

  agents_[id].behavior.state = 0;

  // double newyaw = atan2(agents_[id].sfmAgent.forces.globalForce.getY(),
  //                      agents_[id].sfmAgent.forces.globalForce.getX());
  // agents_[id].sfmAgent.yaw.setRadian(newyaw);

  // if (!agents_initialized_ || dt < 0.008) {
  //   printf("[agentManager.updatePosition] NOT UPDATING agent id:%i,
  //   dt:%.3f\n",
  //          id, dt);
  //   return;
  // }

  // printf("[agentManager.updatePosition] UPDATING agent id:%i, dt:%.3f\n", id,
  //        dt);

  // printf(
  //     "[agentManager.updatePosition] %s time:%.5f PREVpos x:%.3f, "
  //     "y:%.3f, th:%.3f, lv:%.3f, av:%.3f, velx:%.3f, vely:%.3f\n",
  //     agents_[id].name.c_str(), dt, agents_[id].sfmAgent.position.getX(),
  //     agents_[id].sfmAgent.position.getY(),
  //     agents_[id].sfmAgent.yaw.toRadian(),
  //     agents_[id].sfmAgent.linearVelocity,
  //     agents_[id].sfmAgent.angularVelocity,
  //     agents_[id].sfmAgent.velocity.getX(),
  //     agents_[id].sfmAgent.velocity.getY());

  // printf("[AgentManager.UpdatePosition]\t %s Forces. global:%.4f, goal:%.4f
  // "
  //        "soc:%.4f, "
  //        "obs:%.4f \n",
  //        agents_[id].name.c_str(),
  //        agents_[id].sfmAgent.forces.globalForce.norm(),
  //        agents_[id].sfmAgent.forces.desiredForce.norm(),
  //        agents_[id].sfmAgent.forces.socialForce.norm(),
  //        agents_[id].sfmAgent.forces.obstacleForce.norm());

  sfm::SFM.updatePosition(agents_[id].sfmAgent, dt);
  // step_count2 = 1;
  // double newyaw = atan2(agents_[id].sfmAgent.forces.globalForce.getY(),
  //                       agents_[id].sfmAgent.forces.globalForce.getX());
  // agents_[id].sfmAgent.yaw.setRadian(newyaw);

  // printf(
  //     "[agentManager.updatePosition] %s NEWpos x:%.3f, y:%.3f, "
  //     "th:%.3f, lv:%.3f, av:%.3f, velx: %.3f, vely:%.3f\n",
  //     agents_[id].name.c_str(), agents_[id].sfmAgent.position.getX(),
  //     agents_[id].sfmAgent.position.getY(),
  //     agents_[id].sfmAgent.yaw.toRadian(),
  //     agents_[id].sfmAgent.linearVelocity,
  //     agents_[id].sfmAgent.angularVelocity,
  //     agents_[id].sfmAgent.velocity.getX(),
  //     agents_[id].sfmAgent.velocity.getY());

  // if (!agents_[id].sfmAgent.goals.empty()) {
  //   printf("[agentManager.updatePosition] id:%i Current goal x:%.2f, y:
  //   %.2f\n",
  //          id, agents_[id].sfmAgent.goals.front().center.getX(),
  //          agents_[id].sfmAgent.goals.front().center.getY());
  // }
}

}  // namespace hunav
