# Nodes Undiscoverable After the Robot Changes Network

## Status

Reproduced on `raphael` on 2026-10-16. Workaround: restart the platform after
the network change. No configuration fix tried.

## Symptom

After the robot moves to another WiFi network with the platform already
running, `ros2 node list` returns nothing, both from a workstation on the new
network and from inside the robot's own container. `rosmaster-platform.service`
stays `active`, the journal shows no error, and ping, SSH and
`ros2 multicast send`/`receive` work in both directions.

## Affected Environment

- `raphael`, ROS 2 Humble in the `rosmaster_humble` container (host
  networking), default Fast DDS, no `RMW_IMPLEMENTATION` or DDS profile set.
- The platform started on the faculty network; NetworkManager then moved
  `wlan0` to the competition router, on a different subnet.

## Safety / Data-Loss Warning

While the nodes are undiscoverable, no workstation can send a zero `/cmd_vel`
to the robot. If it was moving when the network changed, only the driver's
0.5 s `cmd_vel_timeout` stops the wheels. No data loss.

## Fast Diagnosis

```bash
systemctl is-active rosmaster-platform          # active
docker exec rosmaster_humble bash -lc \
  'source /opt/ros/humble/setup.bash; ros2 node list | wc -l'   # 0
```

`0` from inside the container, with the service active, is this issue. A
workstation-only problem (distro mismatch, firewall, client isolation) still
shows 7 nodes from inside the container.

## Distinguishing Evidence

On 2026-10-16, `tools/sensor_capability_probe.py` run inside the container
received no camera message at all (every topic at 0 Hz) while the platform was
active. After `sudo systemctl restart rosmaster-platform.service` and about
50 s, the container listed 7 nodes and the same probe measured the camera at
30 Hz. Nothing else changed.

## Root Cause

Not isolated. Consistent with the platform's DDS participants keeping the
locators of the address they started with: the new address is never
announced, so nothing on the new network, including a fresh participant on the
same host, matches them.

## Fix

```bash
sudo systemctl restart rosmaster-platform.service
```

Wait about 50 s, then run `ros2 daemon stop` on the workstation before
checking again. [tools/wifi_test/switch_network.sh](../../../tools/wifi_test/switch_network.sh)
changes the connection and restarts the platform in one step.

## Verification

From the workstation, on the robot's new network, with its `ROS_DOMAIN_ID`:

```bash
ros2 node list | wc -l             # 7
ros2 topic echo /voltage --once    # data arrives
```

## What Not To Do

- Do not debug the router, firewall or distro first: if the container itself
  sees no nodes, the network between robot and workstation is not the cause.
- Do not restart the container; the platform restart is enough.

## Prevention

Power the robot on with the network it will use already available, so the
platform starts on its final address. At a competition: router first, then
robots. Restart the platform after any manual network change.

## Affected Robot / Incident

- Robot: `raphael`, 2026-10-16, during the competition router test
- Symptom: 0 nodes from the workstation and from inside the container after
  `wlan0` moved from the faculty network to the router
- Confirmed mitigation: `sudo systemctl restart rosmaster-platform.service`

## Not Tested

Whether the same happens when the robot keeps its address but loses the
network briefly (router reboot, out of range).

## Related Documentation

- [Troubleshooting Index](../README.md)
- [WiFi test](../../../tools/wifi_test/README.md)
- [raphael on the competition router, 2026-10-16](../../../robot_artifacts/raphael_router_wifi_2026-10-16.md)
