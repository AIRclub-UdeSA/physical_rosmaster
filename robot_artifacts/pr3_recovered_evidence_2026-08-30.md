# PR #3 read-only recovery manifest: 2026-08-30

Status: recovery from the old `x3-c` host image is complete. The recovered
payloads are historical evidence only; they do not accept the current PR head.

## Preservation boundary

The original full-card image was not mounted or modified. It was already mode
`0444`, and its full SHA256 was independently recomputed before extraction:

```text
source: /home/juan/rosmaster-x3-before-reflash-2026-08-29.img
size: 62534975488 bytes
SHA256: 006e7d46cc932f3ab2d2966a8eeb203f6bfdace5dbb1cd27b8fa4bdf69f8b529
```

The Docker metadata archive was also verified before extraction:

```text
source: /home/juan/rosmaster-humble-container-metadata.tgz
size: 7520 bytes
SHA256: f426b9677c2fb9ca629af5ccab9d49417f22161e5f9e34764a7c3c66ef0df151
```

The archive confirms old container ID
`7905e90e013d3b522ed00db4da334c3325ce2285e86287a410104a02d3d891ed`,
name `rosmaster_humble`, tag `yahboomtechnology/ros-humble:4.1.2`, local image
ID `sha256:dedc515c326ad7ac812822e60c7796e70bc639d682b09662d46d882dfc0e89fa`,
host networking, privileged mode, `/dev:/dev`, restart policy
`unless-stopped`, `ROS_DOMAIN_ID=11`, and `DISPLAY=:0`.

Privileged loop-device setup was unavailable on the workstation. Exact
byte-range copies of both MBR partitions were therefore made from the source
image and inspected without repair. All derived images and recovered payload
files were marked read-only after hashing.

## Evidence location and manifest

Large payloads remain outside Git:

```text
evidence root: /home/juan/rosmaster-evidence/2026-08-30
file manifest: /home/juan/rosmaster-evidence/2026-08-30/manifests/recovered-files.sha256
manifest entries: 1163
manifest SHA256: e54f74067bbe469ca7067df66c8d38060c88353da0608041541b6231c23b60ef
```

The file manifest covers `source-metadata/` and `recovered/`. The two large
derived partition images are recorded separately:

| Derived image | Size | SHA256 |
| --- | ---: | --- |
| `derived-partitions/bootfs-c777c139-01.img` | 536870912 | `101daada4aaad1616c522f9363b688259a5f7dca169aeb516093213dcced81b4` |
| `derived-partitions/rootfs-c777c139-02.img` | 38754320384 | `2adac9e91819236ec1b9f53783243ad02fa32e3d069858076bbd4ceaa1bdbf5d` |

The source MBR has disk ID `0xc777c139`. Partition 1 starts at sector 8192
and contains 1048576 sectors; partition 2 starts at sector 1056768 and contains
75692032 sectors.

Read-only checks against the derived copies found:

- `fsck.fat -n -v`: no current error; 405 files and 38807 of 261116 clusters
  in use. The preserved full image was made after the previously observed dirty
  bit had been cleared, so this result cannot independently reproduce that
  earlier dirty state.
- `e2fsck -fn`: no structural repair required; 595278 of 2339744 files and
  8327994 of 9461504 blocks in use. It only offered to make inode 129778's
  extent tree narrower; optimization was declined.
- `dumpe2fs`: filesystem state `clean`; last recorded mount
  `2026-08-29 13:28:33`, last write `2026-08-29 13:32:54`, mount count 95,
  and lifetime writes approximately 98 GB.

## Recovered PR #3 evidence

The following payloads survive beneath `recovered/old-container/`:

- all attempt-1-through-6 launch logs and the later lifted-retry logs in
  `pr3-roslog/roslog/`;
- 271 isolated build/test log files in `pr3-temp-log/log/`;
- the final test-result XML in `pr3-test-results/`;
- the repository and its `robot_artifacts/` content;
- the old container's `Rosmaster_Lib.py`.

The recovered final test XML independently reproduces the reported colcon
result: **65 tests, 0 errors, 0 failures, 3 intentional skips**. The recovered
ROS logs contain two successful contract-probe records, before and after the
motion sequence:

```text
python3_46817_1787775113932.log: Physical contract PASSED ... /tf_static=2, /diagnostics=5
python3_56203_1787775792152.log: Physical contract PASSED ... /tf_static=2, /diagnostics=5
```

Earlier failed probe attempts are also present; they were not discarded. The
watchdog records include the two reported stops at 0.522 s and 0.509 s.

The recovered library is exact:

```text
recovered/old-container/Rosmaster_Lib/Rosmaster_Lib.py
SHA256: e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c
```

### Rosbag databases

Each database was opened with SQLite in read-only URI mode. `PRAGMA
integrity_check` returned `ok`, and each database count matches its recovered
`metadata.yaml`.

| Bag database | SHA256 | Topics | Messages | Meaning |
| --- | --- | ---: | ---: | --- |
| `x3_lifted_pr3_2026-08-26_0.db3` | `0793285d090a324d8bf69683aef5bcaff983a4190cb64833b5ff697d019bf101` | 8 | 5612 | Primary PR #3 run; later confirmed to have occurred on the floor despite the name |
| `x3_watchdog_pr3_2026-08-26_0.db3` | `80409fbb368c95b5103536f5c34fc384b73f47150cc2f0ef7e3b7133cceeca69` | 8 | 2277 | Floor watchdog run |
| `x3_lifted_retry_pr3_2026-08-26_0.db3` | `4ef34e4ea15d3bc8501574405404837ab8a4d64bf69ec5504218f37da13d9ae2` | 8 | 5623 | Confirmed lifted retry |
| `x3_lifted_probe_0.db3` | `a98151ccdaec08e2278eafa076d8aa0b900a70102905417833ec2b9096362371` | 7 | 1030 | Legacy 2026-08-16 `/odom_raw` probe; not current-contract evidence |

On 2026-08-31, the four unique directories were copied to the fresh robot
container's root-only recovery-evidence area. All four opened successfully with
the ROS 2 Humble `ros2 bag info` command. Reported topic and message totals
matched this table, and the copied DB3 SHA256 values matched the preserved
workstation files. SQLite integrity and `ros2 bag info` do not prove semantic
correctness; replay or targeted deserialization is separate if later analysis
requires it.

### Recovered repository state

The recovered final repository state is:

```text
branch: platform/simulator-parity
HEAD: 5100be38e40139200691d0f307063cdd1a5a115e
cached origin/platform/simulator-parity: 5100be38e40139200691d0f307063cdd1a5a115e
tracked worktree: clean
untracked: robot_artifacts/x3_lifted_probe/metadata.yaml
```

The historical verification report says the run began at `1bdb7a7`. The
recovered final tree being at `5100be3` is consistent with later commits made
after evidence collection, but it does not prove that interpretation. Neither
revision accepts current workstation HEAD `680c6f7`.

`git fsck --full --no-reflogs` reports an empty loose object at
`.git/objects/db/b58139ad9fe82b9cb60a374180f71a1a4edd23`, created on 2026-08-20.
It is not referenced by `git rev-list --objects --all`; the recorded HEAD and
tracked tree remain readable. This is evidence of an interrupted or failed Git
object write, not evidence that the later `systemd` SIGILL had the same cause.

## Recovered host-failure evidence

The complete old `/var/log` tree was recovered: 92 files totaling 215564853
bytes, including persistent journal files and `boot.log` rotations.

Plain boot logs record repeated `rootfs: recovering journal` events on the
recorded dates August 11, 17, 19, 21, and 27. Some August 19 and 21 boots also
cleared orphaned inodes. An August 19 boot records repeated failures of journal
and udev services and a timeout waiting for boot PARTUUID `c777c139-01`. These
are evidence of earlier host instability, but they do not identify what caused
the August 29 PID 1 SIGILL. The recorded CST times repeat after some boots and
must not be treated as a reliable chronology without time-sync context.

The workstation's `journalctl` is too old to read features used by the recovered
journal files (`Protocol not supported`). Raw journals are preserved in the
manifest. A strings-only scan exposes repeated boot-filesystem dirty/removal
messages, but it is not a substitute for structured journal parsing.

Selected installed package files were recovered and compared directly with the
old dpkg MD5 manifests. The exact `systemd` executable,
`libsystemd-core-252.so`, `libsystemd-shared-252.so`,
`libsystemd.so.0.35.0`, `libc.so.6`, and `ld-linux-aarch64.so.1` all match.
Their SHA256 values are in the external manifest. This narrows the search but
does not verify every runtime input or identify the SIGILL source.

Three core files survive:

- old host `/core`: `/usr/bin/runc --version`, SHA256
  `d579b64a4b57e0bf8c1089f039682a88333d3e61f928ee8db2b3a5c77008edf4`;
- container upper `/core`: `joint_state_publisher_gui`, SHA256
  `1875415dd1a82bf4f8875917425fc2301303055f5e1520b08a09e10d57d3d5b8`;
- container `/root/core`: `usb_cam_node_exe`, SHA256
  `02e8ec95bde71311bd52696bbd9503244a9a761c7da75aca1231aa54f6e2280a`.

No `systemd`/PID 1 core was found. The host core is from the earlier, separately
documented `runc` incident and must not be attributed to this failure.

## Explicit limits and losses

- No expected PR #3 bag, launch-log set, contract output, or final test result
  was found missing.
- Message replay or targeted deserialization has not been performed; it is not
  required to establish that the four preserved bags open and expose metadata.
- Structured journal analysis remains pending a compatible `journalctl`.
- No PID 1 core survives, and the final console-observed SIGILL/panic text was
  not found in the preserved plain boot logs.
- The original dirty FAT state is not present in the later full-card image.
- The old image's recovered repository state is historical evidence only and
  is not a deployable source of truth.

## Conclusion

The old validation payloads survived and are now independently hashed. The
host image also contains repeated signs of prior unclean/recovery boots and one
unreferenced empty Git object, while the checked core system binaries match
their package manifests. Those facts justify the hardening controls, but they
do not prove a single initiating cause. The host incident root cause remains
**Undetermined**.
