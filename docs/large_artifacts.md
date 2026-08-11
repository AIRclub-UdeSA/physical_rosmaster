# Large Artifacts

Date: 2026-08-11

These files are intentionally excluded from Git because they are large, optional, and not needed for normal X3 driver/base bringup.

## Artifact Bundle

Release tag: `large-artifacts-v1`

Archive:

`https://github.com/AIRclub-UdeSA/physical_rosmaster/releases/download/large-artifacts-v1/physical_rosmaster_large_artifacts_v1.tar.gz`

Archive SHA256:

`ebf52f25958b958c30f1b1999ba0b6e6baa8850fac519202349f3fb8d38dcf05`

Archive size:

`73639608` bytes

## Included Files

| Path | Size | SHA256 |
| --- | ---: | --- |
| `yahboomcar_slam/params/ORBvoc.txt` | `145250924` bytes | `f8dd027f7a6cb88129821341194d7f2c75b77b3394257ddd0d2229863d1a3570` |
| `yahboomcar_slam/pcl/resultPointCloudFile.pcd` | `39384107` bytes | `d4fc51ecaabc012c9c80b8e7f037009df3ae0244ab5995bd9a0269668ca477f6` |

## Restore

From the repository root:

```bash
tools/fetch_large_artifacts.sh
```

Or manually:

```bash
curl -L \
  https://github.com/AIRclub-UdeSA/physical_rosmaster/releases/download/large-artifacts-v1/physical_rosmaster_large_artifacts_v1.tar.gz \
  -o /tmp/physical_rosmaster_large_artifacts_v1.tar.gz

echo "ebf52f25958b958c30f1b1999ba0b6e6baa8850fac519202349f3fb8d38dcf05  /tmp/physical_rosmaster_large_artifacts_v1.tar.gz" | sha256sum -c -
tar -xzf /tmp/physical_rosmaster_large_artifacts_v1.tar.gz -C .
```

## When These Are Needed

Normal robot driver bringup, joystick driving, camera, LiDAR, IMU, EKF, and base odometry should not require these files.

Restore them only for ORB-SLAM-related workflows or point-cloud examples that explicitly reference these paths.
