# Upstream

- Original URL: https://github.com/robotics-upo/hunav_sim
- Base SHA: `d97ac2c96b5de1ef9cd8835f99718504a4a005ae` (robotics-upo `v1.0-humble`)
- Freeze SHA compared: `5cba35e74b6d7f8f08fc8bd857953f21064a873d`
- Licence: **MIT**. Root [LICENSE](LICENSE) is the upstream file (copyright 2022 Service Robotics Lab). All `package.xml` files declare MIT.
- Date inspected: 2026-09-06

`hunav_evaluator/setup.py` upstream string was `license='Apache-2.0'` while `package.xml` and root LICENSE are MIT. That was an upstream mismatch, not a second grant. This tree sets the setuptools string to MIT.

## External dependencies (not vendored)

- **lightsfm** — C++ Social Force Model. Headers are expected on the include path (commonly `/usr/local/include/lightsfm`). Upstream URL: https://github.com/robotics-upo/lightsfm. Licence on installed headers: BSD-3-Clause (copyright 2016 Service Robotics Lab). **Git revision of the freeze install is unknown**; this repository does not vendor a guessed SHA.
- **people_msgs** — ROS 2 dependency of `hunav_agent_manager` (`<depend>people_msgs</depend>`). Source: https://github.com/wg-perception/people (`people_msgs`, ROS 2). Not copied into this tree.

Evaluator `hunav_evaluator/hunav_evaluator/sfm.py` is a Python Social Force Model used for *metrics*, not the C++ library.

## Divergence summary

Relative to `d97ac2c`, this tree ports `hunav_agent_manager` to ROS 2 Jazzy (link/build depends, loader parameters, near-robot behaviour, BT `dt` flooring) and adds reusable Arena-style `arena_*` metric implementations plus the package-default keys in `hunav_evaluator/config/metrics.yaml` (`frequency` remains 1.0). Campaign hop metric *enablement* is not in this repository.
