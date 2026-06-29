# Known Issues

This document consolidates commonly reported problems for `apollo_perception_ros`.
It is an educational / historical port — many reports reflect environment
mismatch rather than regressions in current maintenance.

Each item links to the GitHub issue(s) where it was reported.

Related docs:

- [DEMO_BAG.md](DEMO_BAG.md) — demo bag download and playback
- [SUPPORTED_ENVIRONMENTS.md](SUPPORTED_ENVIRONMENTS.md) — reference and unverified environments

---

## Build and toolchain

### C++11 / catkin / ROS environment errors

**Symptoms**

- `#error This file requires compiler and library support for the ISO C++ 2011 standard`
- Intermittent `catkin build` failures on first attempt
- `catkin CMake module was not found` or missing `roscppConfig.cmake`
- Building outside Docker without a sourced ROS environment

**Status:** Known on non-reference toolchains; often transient inside the reference Docker image.

**Workaround**

- Build inside `cedricxie/apollo-perception-ros` Docker
- Source ROS inside the container before building (e.g. `source /opt/ros/indigo/setup.bash`)
- Retry `catkin build`
- Capture the first hard error if warnings are only about deprecated GPU targets

**Reported in:** [#3](https://github.com/cedricxie/apollo_perception_ros/issues/3), [#13](https://github.com/cedricxie/apollo_perception_ros/issues/13), [#2](https://github.com/cedricxie/apollo_perception_ros/issues/2)

---

### Eigen version mismatch (historical)

**Symptoms**

- Runtime assertion or crash when starting `perception_yx_node` after a local build

**Status:** Resolved in at least one report by downgrading Eigen (for example from 3.3.90 to 3.2.10).

**Workaround**

- Prefer the reference Docker image rather than mixing host Eigen versions

**Reported in:** [#10](https://github.com/cedricxie/apollo_perception_ros/issues/10) (closed)

---

### Missing Caffe layers (`reorg`, `permute`, `roi_pooling`)

**Symptoms**

- `Caffe::LayerParameter` cannot find `reorg` or `roi_pooling`
- Build fails when using stock Caffe instead of the Apollo-era fork

**Status:** Expected outside the bundled Docker environment.

**Workaround**

- Build inside the provided Docker image, which includes the custom Caffe layers used by this port

**Reported in:** [#23](https://github.com/cedricxie/apollo_perception_ros/issues/23)

---

## CUDA and GPU compatibility

### CUDA 10+ build failures

**Symptoms**

- Build fails on CUDA 10 with errors such as `incomplete type is not allowed` in `cuda_util/util.cu`

**Status:** Unsupported. The reference Docker image targets CUDA 8.

**Workaround**

- Use the provided CUDA 8 Docker workflow rather than porting to CUDA 10+
- Community report on [#6](https://github.com/cedricxie/apollo_perception_ros/issues/6): adding OpenCV headers to `cuda_util/util.cu` may unblock some CUDA 10 builds, but this is **unverified** and not part of the maintained workflow

**Reported in:** [#6](https://github.com/cedricxie/apollo_perception_ros/issues/6)

---

### `invalid device function`

**Symptoms**

- `permute_layer.cu: Check failed: error == cudaSuccess (8 vs. 0) invalid device function`

**Status:** Common on GPUs newer than the original CUDA 8 / SM targets.

**Workaround**

- Prefer the reference GTX 10-series + CUDA 8 Docker workflow
- Rebuilding CUDA artifacts for newer GPUs is not part of the reference maintenance path

**Reported in:** [#18](https://github.com/cedricxie/apollo_perception_ros/issues/18), [#27](https://github.com/cedricxie/apollo_perception_ros/issues/27)

---

## Docker and NVIDIA runtime

### `Unknown runtime specified nvidia`

**Symptoms**

- Docker fails to start the container with an unknown `nvidia` runtime

**Status:** Host/runtime configuration issue.

**Workaround**

- Install NVIDIA Container Toolkit
- Avoid legacy `nvidia-docker` v1-only setups

**Reported in:** [#1](https://github.com/cedricxie/apollo_perception_ros/issues/1)

---

### `nvml error: driver not loaded`

**Symptoms**

- `nvidia-container-cli: initialization error: nvml error: driver not loaded: unknown`

**Status:** Host NVIDIA driver not visible to Docker.

**Workaround**

- Verify `nvidia-smi` on the host
- Confirm GPU flags/runtime are passed to `docker run`

**Reported in:** [#20](https://github.com/cedricxie/apollo_perception_ros/issues/20)

---

## Demo bag and sensor data

### Demo bag download / playback

**Symptoms**

- Apollo open data platform link unavailable
- Unclear where to place or how to play `demo-2.0.bag`
- Questions about Velodyne-only bags vs Apollo demo bag

**Status:** Documented with a verified maintainer mirror. This repo is tested with the Apollo 2.0 vehicle demo bag, not arbitrary Velodyne-only bags.

**Workaround**

- Follow [DEMO_BAG.md](DEMO_BAG.md)

**Reported in:** [#15](https://github.com/cedricxie/apollo_perception_ros/issues/15), [#19](https://github.com/cedricxie/apollo_perception_ros/issues/19), [#25](https://github.com/cedricxie/apollo_perception_ros/issues/25), [#26](https://github.com/cedricxie/apollo_perception_ros/issues/26)

---

## TF, coordinates, and visualization

### Missing `base_link` / `camera_long` transforms

**Symptoms**

- `Cannot transform frame: base_link to frame camera_long`
- `failed to get trans at timestamp`

**Status:** Usually launch order or `use_sim_time` mismatch. [#11](https://github.com/cedricxie/apollo_perception_ros/issues/11) may also reflect a deeper coordinate-transform bug and remains open.

**Workaround**

1. `roslaunch vehicle_base detect_sim.launch`
2. `rosbag play ~/shared_dir/demo-2.0.bag --clock`
3. Inspect TF tree with `rqt_tf_tree` or `tf_monitor`

**Reported in:** [#11](https://github.com/cedricxie/apollo_perception_ros/issues/11), [#22](https://github.com/cedricxie/apollo_perception_ros/issues/22)

---

### Fusion velocity markers show zeros

**Symptoms**

- `/perception/output/fusion_velocity_marker` positions are `0`

**Status:** Under investigation; often related to TF/timestamp alignment, incomplete sensor inputs, or GPU incompatibility. One report ([#24](https://github.com/cedricxie/apollo_perception_ros/issues/24)) improved after switching from RTX 4000 to GTX 1050 Ti.

**Workaround**

- Confirm camera + LiDAR topics are present from the demo bag
- Verify TF tree and `use_sim_time`

**Reported in:** [#24](https://github.com/cedricxie/apollo_perception_ros/issues/24)

---

### RViz image / encoding issues during demo playback

**Symptoms**

- No images in RViz
- `unsupported encoding yuyv`
- Expectation that LiDAR clustering requires camera input

**Status:** Demo workflow requires both camera and LiDAR topics. Camera topics in the Apollo bag may use `yuyv`, which RViz does not display directly; the code path also logs that `yuyv` is not supported for conversion.

**Workaround**

- Use `detect_sim.launch` + `rosbag play ... --clock`
- See [DEMO_BAG.md](DEMO_BAG.md)
- Check perception output image topics in RViz instead of the raw bag encoding if needed
- Share `rostopic list` and camera topic metadata if it persists

**Reported in:** [#16](https://github.com/cedricxie/apollo_perception_ros/issues/16), [#21](https://github.com/cedricxie/apollo_perception_ros/issues/21) (closed)

---

## Radar

### Continental radar in Apollo demo bag

**Symptoms**

- Radar fusion unavailable when playing the stock Apollo demo bag
- Questions about why radar-related code is commented out

**Status:** Continental radar messages in the bag use Protobuf and are not fully supported in this ROS port. By default, `use_radar` is `false` in `perception_yx_detect.launch`, and the Continental radar topic subscription in `perception_yx.cpp` is commented out.

**Workaround**

- Leave radar disabled for the demo bag workflow
- To experiment with your own radar, use the modest radar detector code path and provide a compatible ROS message source

**Reported in:** [#15](https://github.com/cedricxie/apollo_perception_ros/issues/15#issuecomment-867289918), [#16](https://github.com/cedricxie/apollo_perception_ros/issues/16), [#18](https://github.com/cedricxie/apollo_perception_ros/issues/18)

---

## Environment and platform support

### ROS Kinetic / Ubuntu 16.04

**Symptoms**

- VTK5 vs VTK6 / PCL build conflicts when porting off the reference image

**Status:** Community/unverified.

**Workaround**

- Use the reference Ubuntu 14.04 Docker workflow in the README

**Reported in:** [#14](https://github.com/cedricxie/apollo_perception_ros/issues/14)

---

### Hardware requirements and Docker usage

**Symptoms**

- Questions about required GPU/hardware
- Whether Docker is mandatory

**Status:** Documented at a high level.

**Workaround**

- See [SUPPORTED_ENVIRONMENTS.md](SUPPORTED_ENVIRONMENTS.md)

**Reported in:** [#4](https://github.com/cedricxie/apollo_perception_ros/issues/4), [#5](https://github.com/cedricxie/apollo_perception_ros/issues/5)

---

## Documentation

### ROS distro / CMake path confusion

**Symptoms**

- Confusion about whether the project targets ROS Indigo or Kinetic
- References to `src/perception/CMakeLists.txt` or Kinetic catkin paths

**Status:** The reference workflow is the Ubuntu 14.04 Docker image with the catkin packages under `src/perception/apollo_perception_ros/`. Some older reports used outdated paths or non-reference ROS installs.

**Workaround**

- Use the Docker workflow in the README
- See [SUPPORTED_ENVIRONMENTS.md](SUPPORTED_ENVIRONMENTS.md)

**Reported in:** [#9](https://github.com/cedricxie/apollo_perception_ros/issues/9), [#14](https://github.com/cedricxie/apollo_perception_ros/issues/14)

---

## Reporting a new issue

Before opening a duplicate, search [open issues](https://github.com/cedricxie/apollo_perception_ros/issues).
Include GPU model, driver version, OS, Docker command, and logs.
