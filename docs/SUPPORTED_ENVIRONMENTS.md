# Supported Environments

This project is a **historical / educational** ROS port of Apollo 3.0 perception.
Only the environment below is treated as the reference configuration.

For common failures on other setups, see [KNOWN_ISSUES.md](KNOWN_ISSUES.md).

## Reference environment (tested originally)

| Component | Version / hardware |
| --- | --- |
| Host GPU | NVIDIA GeForce GTX 1080 Ti, GTX 1070 Max-Q |
| NVIDIA driver | 384.130 (historical) |
| Container OS | Ubuntu 14.04 |
| CUDA | 8.0 |
| cuDNN | 7 |
| ROS workflow | catkin inside provided Docker image |
| GPU runtime | `nvidia-docker` v2 era / NVIDIA Container Toolkit equivalent |
| Docker image | `cedricxie/apollo-perception-ros` |

## Community / unverified environments

| Environment | Status | Notes |
| --- | --- | --- |
| Ubuntu 18.04 / 20.04 | Unverified | May require Docker image rebuild |
| ROS Kinetic on Ubuntu 16.04 | Unverified | VTK/PCL conflicts reported ([#14](https://github.com/cedricxie/apollo_perception_ros/issues/14)) |
| CUDA 10+ | Unsupported | Reference image targets CUDA 8 ([#6](https://github.com/cedricxie/apollo_perception_ros/issues/6)) |
| NVIDIA RTX 20xx / 30xx / 40xx | Likely broken | `invalid device function` without CUDA rebuild ([#18](https://github.com/cedricxie/apollo_perception_ros/issues/18), [#27](https://github.com/cedricxie/apollo_perception_ros/issues/27)) |
| Builds outside provided Docker | Unverified | Custom Caffe/CUDA layers may be missing ([#23](https://github.com/cedricxie/apollo_perception_ros/issues/23)) |
| Legacy `nvidia-docker` v1 only | Deprecated | Use NVIDIA Container Toolkit ([#1](https://github.com/cedricxie/apollo_perception_ros/issues/1)) |

## Recommended workflow

1. Install NVIDIA driver on the host and verify `nvidia-smi`
2. Install Docker and NVIDIA Container Toolkit
3. Pull and run `cedricxie/apollo-perception-ros`
4. Build with `catkin build` inside the container
5. Use `demo-2.0.bag` per [DEMO_BAG.md](DEMO_BAG.md)

## Hardware questions

General hardware expectations are discussed in
[#5](https://github.com/cedricxie/apollo_perception_ros/issues/5).
Docker is strongly recommended but not theoretically mandatory ([#4](https://github.com/cedricxie/apollo_perception_ros/issues/4)).

If your setup differs, please open an issue with GPU model, driver version, OS,
Docker command, and build/runtime logs.
