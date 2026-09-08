# Human Navigation behavior Simulator (HuNavSim)

## Origin

Fork of [robotics-upo/hunav_sim](https://github.com/robotics-upo/hunav_sim) (`v1.0-humble`, `d97ac2c`).

**Why:** upstream is built and tested for ROS 2 Humble. This tree keeps the agent manager, messages, and evaluator usable on **ROS 2 Jazzy** (Ubuntu 24.04) next to Isaac Sim 6.0.1.

**What this tree adds:** Jazzy link/build depends, loader parameters, near-robot behaviour, BT `dt` flooring, and Arena-style `arena_*` metric keys. Pedestrian rendering and Isaac world loading live in the companion wrapper.

SHAs, lightsfm, and `people_msgs`: [UPSTREAM.md](UPSTREAM.md).

HuNavSim is a ROS 2 simulator for human navigation. It controls the navigation behaviour of human agents spawned in a robotics simulator (Gazebo, Webots, Isaac Sim, or another wrapper).

The simulated people are affected by obstacles and other people using the [Social Force Model](https://github.com/robotics-upo/lightsfm) library (**external**; not vendored here). A set of human reactions to the presence of robots is included.

If you use this simulator in your work, please cite:

N. Pérez-Higueras, R. Otero, F. Caballero and L. Merino, "HuNavSim: A ROS 2 Human Navigation Simulator for Benchmarking Human-Aware Robot Navigation," in IEEE Robotics and Automation Letters, vol. 8, no. 11, pp. 7130-7137, Nov. 2023, doi: 10.1109/LRA.2023.3316072.

```
@ARTICLE{PerezRal2023,
  author={Pérez-Higueras, Noé and Otero, Roberto and Caballero, Fernando and Merino, Luis},
  journal={IEEE Robotics and Automation Letters},
  title={HuNavSim: A ROS 2 Human Navigation Simulator for Benchmarking Human-Aware Robot Navigation},
  year={2023},
  month={September},
  volume={8},
  number={11},
  pages={7130-7137},
  issn={2377-3766},
  doi={10.1109/LRA.2023.3316072}}
```

A pre-print of the accepted paper is available [here](https://arxiv.org/abs/2305.01303).

## Packages

| Package | Role |
|---|---|
| `hunav_agent_manager` | Pedestrian state, Social Force Model, behaviour trees |
| `hunav_msgs` | Agent messages and compute/evaluate services |
| `hunav_evaluator` | Social-navigation metrics (including Arena-style `arena_*` keys) |
| `hunav_rviz2_panel` | RViz2 GUI to configure agents |
| `hunav_sim` | Metapackage |

## Dependencies

* **lightsfm** (C++ Social Force Model). Install from https://github.com/robotics-upo/lightsfm. This repository does not vendor it.
* **people_msgs**. Declared as a dependency of `hunav_agent_manager`; not vendored here. Source: https://github.com/wg-perception/people/tree/ros2
* Nav2 behaviour trees:

  ```sh
  sudo apt install ros-jazzy-nav2-behavior-tree
  ```

* BehaviorTree.CPP 4.x and BehaviorTree.ROS2, as required by `hunav_agent_manager`. Place them in the same colcon workspace if they are not already installed.

Upstream also publishes a [container deployment](https://github.com/robotics-upo/hunavsim_containers) (Humble-oriented). This tree is built and tested for **ROS 2 Jazzy** on Ubuntu 24.04.

## Features

* Human navigation based on an adapted Social Force Model and its extension for groups.
* Simulator-independent core: a wrapper collects agent and robot state, calls HuNav services, and writes the updated poses back.
* Wrappers exist for [Gazebo Classic](https://github.com/robotics-upo/hunav_gazebo_wrapper), [Gazebo Fortress](https://github.com/robotics-upo/hunav_gazebo_fortress_wrapper), and [Webots](https://github.com/robotics-upo/hunav_webots_wrapper). Isaac Sim uses a separate wrapper package.
* RViz2 panel for agent configuration: see [hunav_rviz2_panel](hunav_rviz2_panel/README.md).
* Reactions to a robot:

    * *regular*: treat the robot like another human
    * *impassive*: treat the robot like a static obstacle
    * *surprised*: stop and look at the robot
    * *curious*: leave the current goal for a while and approach the robot
    * *scared*: stay far from the robot
    * *threatening*: block the robot by walking in front of it

* Each agent’s behaviour is led by a configurable behaviour tree.
* Metrics for social navigation evaluation, including Hall intimate/personal/social space and Arena-style `arena_*` implementations. Default keys: [hunav_evaluator/config/metrics.yaml](hunav_evaluator/config/metrics.yaml). Further notes: [hunav_evaluator/README.md](hunav_evaluator/README.md).

## Build

```bash
source /opt/ros/jazzy/setup.bash
# workspace must also provide people_msgs and lightsfm headers
colcon build --packages-up-to hunav_sim
source install/setup.bash
```

## Tests (no Isaac Sim)

Message-contract checks parse `hunav_msgs` IDL. Metric regression uses synthetic Hall / `arena_*` ticks (stubs ROS message types when `hunav_msgs` is not on `PYTHONPATH`).

```bash
python3 -m pytest tests/ -q
```

Existing ament lint tests (`copyright` / `flake8` / `pep257`) remain under `hunav_evaluator/test/` and run with `colcon test`.

## Using HuNavSim with a robotics simulator

At each simulation step the wrapper must collect the current state of the human agents and the robot (positions and velocities) and send them to HuNavSim. HuNavSim returns the next agent states; the wrapper updates the simulator.

Services (package `hunav_msgs`):

* `/compute_agents` — request: `hunav_msgs/Agents` plus robot `hunav_msgs/Agent`; response: updated `Agents`
* `/compute_agent` — request: agent id; response: updated `Agent`
* `/move_agent` — request: agents, robot, and agent id; response: updated `Agent`

Initial agent parameters are loaded by the `hunav_loader` node (YAML under `hunav_agent_manager`) and can be read with `/hunav_loader/get_parameters`.

![](images/HuNavSim.png)

## Configuration

Define agents in an `agents.yaml` (or create it with the [RViz2 panel](hunav_rviz2_panel/README.md)):

```yaml
hunav_loader:
  ros__parameters:
    map: cafe
    publish_people: true
    agents:
      - agent1
      - agent2
    agent1:
      id: 1
      skin: 2
      group_id: -1
      max_vel: 1.5
      radius: 0.4
      behavior:
        type: 4 # REGULAR=1, IMPASSIVE=2, SURPRISED=3, SCARED=4, CURIOUS=5, THREATENING=6
        configuration: 0 # def: 0, custom:1, random_normal:2, random_uniform:3
        duration: 40.0  # seg
        once: true
        vel: 0.6
        dist: 0.0
        goal_force_factor: 2.0
        obstacle_force_factor: 10.0
        social_force_factor: 5.0
        other_force_factor: 20.0
      init_pose:
        x: -3.973340
        y: -8.576801
        z: 1.250000
        h: 0.0
      goal_radius: 0.3
      cyclic_goals: true
      goals:
        - g0
        - g1
        - g2
      g0:
        x: -3.133759
        y: -4.166653
        h: 1.250000
      g1:
        x: 0.997901
        y: -4.131655
        h: 1.250000
      g2:
        x: -0.227549
        y: -10.187146
        h: 1.250000
    agent2:
      id: 2
      skin: 3
      group_id: -1
      max_vel: 1.5
      radius: 0.4
      behavior:
        type: 6
        configuration: 2
        duration: 40.0  # seg
        once: true
        vel: 0.6
        dist: 0.0
        goal_force_factor: 2.0
        obstacle_force_factor: 10.0
        social_force_factor: 5.0
        other_force_factor: 20.0
      init_pose:
        x: 2.924233
        y: 5.007970
        z: 1.250000
        h: 0.0
      goal_radius: 0.3
      cyclic_goals: true
      goals:
        - g0
        - g1
      g0:
        x: -2.644067
        y: 2.066231
        h: 1.250000
      g1:
        x: -1.663169
        y: -3.291318
        h: 1.250000
```

### Global parameters

* `map`. Name of the 2D map that corresponds to the scenario. **Currently unused.**
* `publish_people`. If true, publish agents on `/people` as `people_msgs/People`. Yaw (rad) is in `position.z`; angular velocity (rad/s) is in `velocity.z`.
* `agents`. List of agent names.

### Agent parameters

Names must match the `agents` list.

* `id`. Unique integer.
* `skin`. Integer 3D model index for Gazebo meshes, range [0-4].
* `group_id`. Shared by walking-group members; `-1` means not in a group.
* `max_vel`. Maximum velocity (m/s).
* `radius`. Circumscribed footprint radius (m).
* `behavior`:
  * `type`: **1** Regular, **2** Impassive, **3** Surprised, **4** Scared, **5** Curious, **6** Threatening
  * `configuration`: **0** default, **1** custom, **2** random normal, **3** random uniform
  * Custom-mode only: `duration`, `once`, `vel`, `dist`, `goal_force_factor`, `obstacle_force_factor`, `social_force_factor`, `other_force_factor`
* `init_pose`. `x`, `y`, `z` (m) and heading `h` (rad).
* `goal_radius`. Goal capture radius (m).
* `cyclic_goals`. Restart the goal list after the last goal.
* `goals`. Goal identifiers; each has `x`, `y`, and heading `h`.

### Metrics

Enable or disable metrics in [hunav_evaluator/config/metrics.yaml](hunav_evaluator/config/metrics.yaml). Campaign-specific enablement (which keys a hop records) lives in the platform, not this package.

## Licence

MIT. Copyright 2022 Service Robotics Lab. See [LICENSE](LICENSE) and [UPSTREAM.md](UPSTREAM.md).

## Acknowledgements

This work is partially supported by Programa Operativo FEDER Andalucia 2014-2020, Consejeria de Economía, Conocimiento y Universidades (DeepBot, PY20\_00817) and the project NHoA (PLEC2021-007868) and NORDIC (TED2021-132476B-I00), funded by MCIN/AEI/10.13039/501100011033 and the European Union NextGenerationEU/PRTR.

<img src="images/image.png" width="400">
<img src="images/logos2.jpg" width="400">
