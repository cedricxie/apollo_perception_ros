# Demo Bag Guide

This guide explains how to obtain and play the Apollo 2.0 demo sensor data used
with `apollo_perception_ros`.

## What you need

| Item | Value |
| --- | --- |
| Archive name | `demo-sensor-data-apollo-2.0.tar.gz` |
| Approximate size | ~3.4 GB |
| Bag file inside | `demo-2.0.bag` |

The perception node expects topics such as:

- `/apollo/sensor/velodyne64/compensator/PointCloud2`
- `/apollo/sensor/camera/traffic/image_long`
- `/apollo/sensor/camera/traffic/image_short`

## Download options

### Option 1: Maintainer mirror (recommended)

As of 2026-06-29, this Dropbox mirror is available and verified:

**Direct download**

https://www.dropbox.com/scl/fi/vjshuaxcp97z0s55ghyd1/demo-sensor-data-apollo-2.0.tar.gz?rlkey=abhtws4qijkx35wh84drj9qcm&dl=1

**Share page**

https://www.dropbox.com/scl/fi/vjshuaxcp97z0s55ghyd1/demo-sensor-data-apollo-2.0.tar.gz?rlkey=abhtws4qijkx35wh84drj9qcm&dl=0

An older Dropbox URL still redirects to the link above:

https://www.dropbox.com/s/ftvp3i3a5oeuo6t/demo-sensor-data-apollo-2.0.tar.gz?dl=1

### Option 2: Apollo Data Open Platform (may be unavailable)

The original README pointed to the Apollo open data platform:

- Portal: http://data.apollo.auto
- Category: Vehicle System Demo Data
- File: `demo-sensor-data-apollo-2.0.tar.gz`

Users have reported that this portal or tarball link may no longer work. If the
official source becomes available again, prefer it and open an issue so this
document can be updated.

## Setup

1. Download the tarball to your host machine.
2. Extract it:

```bash
tar xzf demo-sensor-data-apollo-2.0.tar.gz
```

3. Place `demo-2.0.bag` in the directory mounted into Docker as `shared_dir`.
   The default `docker/run.sh` maps host `~/shared_dir` to `/home/yx/shared_dir`
   inside the container.

Example on the host:

```bash
mkdir -p ~/shared_dir
mv demo-2.0.bag ~/shared_dir/
```

## Playback

Inside the Docker container, after building and sourcing the workspace:

```bash
source ~/shared_dir/apollo_perception_ros/devel/setup.bash
roslaunch vehicle_base detect_sim.launch
```

In another terminal inside the container:

```bash
rosbag play ~/shared_dir/demo-2.0.bag --clock
```

`detect_sim.launch` sets `use_sim_time` to true, so `--clock` is required.

## Troubleshooting

| Problem | Notes |
| --- | --- |
| Dropbox link fails | Try the direct-download `dl=1` URL above. If it stops working, comment on issue #26. |
| Bag not found in container | Confirm the file is under the mounted `shared_dir` path, not only on the host outside the mount. |
| No perception output | Verify topics with `rostopic list` and check GPU/Docker setup. See README known limitations. |
| Radar not working | Continental radar messages in the bag use Protobuf and are not fully supported in this port. |

## Related issues

- #15 — original bag access report (2020 mirror)
- #19 — bag playback path notes
- #25 — Velodyne bag usage questions
- #26 — invalid bag package link

If you have an updated official or mirror link, please open a pull request or
issue.
