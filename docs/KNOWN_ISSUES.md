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

### C++11 / catkin build errors

**Symptoms**

- `#error This file requires compiler and library support for the ISO C++ 2011 standard`
- Intermittent `catkin build` failures on first attempt

**Status:** Known on non-reference toolchains; often transient inside the reference Docker image.

**Workaround**

- Build inside `cedricxie/apollo-perception-ros` Docker
- Retry `catkin build`
- Capture the first hard error if warnings are only about deprecated GPU targets

**Reported in:** [#3](https://github.com/cedricxie/apollo_perception_ros/issues/3), [#13](https://github.com/cedricxie/apollo_perception_ros/issues/13)

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

### `invalid device function`

**Symptoms**

- `permute_layer.cu: Check failed: error == cudaSuccess (8 vs. 0) invalid device function`

**Status:** Common on GPUs newer than the original CUDA 8 / SM targets.

**Workaround**

- Prefer the reference GTX 10-series + CUDA 8 Docker workflow
- Rebuilding CUDA artifacts for newer GPUs is not part of the reference maintenance path

**Reported in:** [#6](https://github.com/cedricxie/apollo_perception_ros/issues/6), [#18](https://github.com/cedricxie/apollo_perception_ros/issues/18), [#27](https://github.com/cedricxie/apollo_perception_ros/issues/27)

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

**Status:** Documented with a verified maintainer mirror.

**Workaround**

- Follow [DEMO_BAG.md](DEMO_BAG.md)

**Reported in:** [#15](https://github.com/cedricxie/apollo_perception_ros/issues/15), [#19](https://github.com/cedricxie/apollo_perception_ros/issues/19), [#25](https://github.com/cedricxie/apollo_perception_ros/issues/25), [#26](https://github.com/cedricxie/apollo_perception_ros/issues/26)

---

## TF, coordinates, and visualization

### Missing `base_link` / `camera_long` transforms

**Symptoms**

- `Cannot transform frame: base_link to frame camera_long`
- `failed to get trans at timestamp`

**Status:** Usually launch order or `use_sim_time` mismatch.

**Workaround**

1. `roslaunch vehicle_base detect_sim.launch`
2. `rosbag play ~/shared_dir/demo-2.0.bag --clock`
3. Inspect TF tree with `rqt_tf_tree` or `tf_monitor`

**Reported in:** [#11](https://github.com/cedricxie/apollo_perception_ros/issues/11), [#22](https://github.com/cedricxie/apollo_perception_ros/issues/22)

---

### Fusion velocity markers show zeros

**Symptoms**

- `/perception/output/fusion_velocity_marker` positions are `0`

**Status:** Under investigation; often related to TF/timestamp alignment or incomplete sensor inputs.

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

**Status:** Demo workflow requires both camera and LiDAR topics; image display depends on topic encoding and sim time settings.

**Workaround**

- Use `detect_sim.launch` + `rosbag play ... --clock`
- See [DEMO_BAG.md](DEMO_BAG.md)
- Share `rostopic list` and camera topic metadata if it persists

**Reported in:** [#16](https://github.com/cedricxie/apollo_perception_ros/issues/16)

---

## Radar

### Continental radar in Apollo demo bag

**Symptoms**

- Radar fusion unavailable when playing the stock Apollo demo bag

**Status:** Continental radar messages in the bag use Protobuf and are not fully supported in this ROS port.

**Workaround**

- Disable radar in launch parameters, or integrate your own radar with the modest radar detector code path

**Reported in:** Documented limitation (see README). Radar integration questions also appear in [#15](https://github.com/cedricxie/apollo_perception_ros/issues/15#issuecomment-867289918).

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

### Broken documentation links

**Symptoms**

- README or docs reference missing paths (e.g. `CMakeLists.txt` link)

**Status:** Tracked for cleanup.

**Workaround**

- Open a PR or comment on the issue with the correct target path

**Reported in:** [#9](https://github.com/cedricxie/apollo_perception_ros/issues/9)

---

## Reporting a new issue

Before opening a duplicate, search [open issues](https://github.com/cedricxie/apollo_perception_ros/issues).
Include GPU model, driver version, OS, Docker command, and logs.
