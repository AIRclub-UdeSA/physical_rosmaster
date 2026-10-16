"""Compare per-topic message counts of two `ros2 bag info` outputs.

Usage: python3 compare_bags.py <robot_bag_info.txt> <workstation_bag_info.txt>
"""

import re
import sys

TOPIC_COUNT = re.compile(r"Topic: (\S+) \| .*?Count: (\d+)")
# Latched once per publisher, so its count says nothing about delivery.
NOT_GRADED = {"/tf_static"}


def parse_counts(text):
    """Map each topic in `ros2 bag info` output to its message count."""
    return {topic: int(count) for topic, count in TOPIC_COUNT.findall(text)}


def compare(robot, workstation, threshold=0.9):
    """Yield (topic, robot count, workstation count, ratio or None, short)."""
    for topic in sorted(set(robot) | set(workstation)):
        sent, received = robot.get(topic, 0), workstation.get(topic, 0)
        ratio = received / sent if sent else None
        short = (
            ratio is not None and ratio < threshold and topic not in NOT_GRADED
        )
        yield topic, sent, received, ratio, short


def main(argv):
    with open(argv[1]) as robot_file, open(argv[2]) as workstation_file:
        robot_text = robot_file.read()
        robot = parse_counts(robot_text)
        workstation = parse_counts(workstation_file.read())
    if not robot:
        print("   >>> the robot recorded nothing:\n" + robot_text)
    print(f"   {'topic':34} {'robot':>7} {'workst.':>7} {'%':>5}")
    for topic, sent, received, ratio, short in compare(robot, workstation):
        percent = f"{100 * ratio:4.0f}%" if ratio is not None else "    -"
        flag = "  <<<" if short else ""
        print(f"   {topic:34} {sent:7} {received:7} {percent}{flag}")


if __name__ == "__main__":
    main(sys.argv)
