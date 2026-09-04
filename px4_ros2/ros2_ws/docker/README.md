# Raspberry Pi 5 Docker build

This directory builds the onboard `collision_avoidance` image for Raspberry
Pi 5. The host may run Ubuntu 24.04; ROS 2 Humble and its Ubuntu 22.04 runtime
remain inside the container.

## Required `px4_msgs` fork

The production package subscribes to the project-specific
`EstimatorTrajectoryBelief` message. That message is not present in the
upstream PX4 commit previously used by this repository. The required source is
fixed in `../px4_dependencies.repos`:

```text
repository: https://github.com/sdfkjsdf/px4_msgs.git
commit:     f7741616e14c8330b54acdaf8daf382750896081
```
Do not replace this dependency with the upstream `PX4/px4_msgs` repository
unless `EstimatorTrajectoryBelief.msg` is also ported and compatibility with
the PX4 firmware is revalidated.

## Repair an existing Raspberry Pi checkout

First update the project checkout:

```bash
cd /home/agent0/workspace/swarm_fixed_wing_collision_avoidance
git pull --ff-only origin main
```

If `src/px4_msgs` was cloned from the upstream PX4 repository, point the
existing checkout at the project fork and select the fixed commit:

```bash
cd /home/agent0/workspace/swarm_fixed_wing_collision_avoidance/px4_ros2/ros2_ws/src/px4_msgs

git remote set-url origin \
  https://github.com/sdfkjsdf/px4_msgs.git

git fetch origin feature/trajectory-belief-message

git switch --detach \
  f7741616e14c8330b54acdaf8daf382750896081
```

Verify both the commit and required message before building:

```bash
git rev-parse --short HEAD

test -f msg/EstimatorTrajectoryBelief.msg \
  && echo "EstimatorTrajectoryBelief: PASS"
```

Expected output:

```text
f774161
EstimatorTrajectoryBelief: PASS
```

## Build the image without starving SSH

`px4_msgs` generates and compiles a large number of ROS type-support files.
On a four-core Raspberry Pi, an unrestricted build can use all cores and make
SSH temporarily unresponsive. Limit the build container to two cores:

```bash
cd /home/agent0/workspace/swarm_fixed_wing_collision_avoidance/px4_ros2/ros2_ws

docker build \
  --cpuset-cpus="0,1" \
  --pull \
  -t collision-avoidance:distributed \
  -f docker/Dockerfile \
  .
```

The legacy Docker builder may print a deprecation warning. It does not support
`--progress=plain`, so that option is intentionally omitted.

Verify the completed image:

```bash
docker image ls collision-avoidance

docker run --rm \
  --network host \
  --entrypoint /bin/bash \
  collision-avoidance:distributed \
  -lc '
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    test -x /ros2_ws/install/collision_avoidance/lib/collision_avoidance/run_guidance_vehicle.sh
    echo "container verification: PASS"
  '
```
