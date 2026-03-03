"""Launch all ROS2 nodes found in the nodes/ directory."""

import signal
import subprocess
import sys
from pathlib import Path

NODES_DIR = Path(__file__).parent


SPIN_MARKER = "rclpy" + ".spin(node)"


def is_node(path: Path) -> bool:
    """A node file spins a rclpy node as its main loop."""
    return SPIN_MARKER in path.read_text()


def main():
    nodes = []
    for f in sorted(NODES_DIR.glob("*.py")):
        if f.name.startswith("_"):
            continue
        if is_node(f):
            nodes.append(f.stem)

    if not nodes:
        print("No nodes found")
        return

    print(f"Launching: {', '.join(nodes)}")
    procs = []
    for name in nodes:
        p = subprocess.Popen(
            [sys.executable, "-c", f"from nodes.{name} import main; main()"],
        )
        procs.append((name, p))
        print(f"  {name} (pid {p.pid})")

    def shutdown(*_):
        print("\nStopping...")
        for _, p in procs:
            p.send_signal(signal.SIGINT)
        for name, p in procs:
            p.wait(timeout=5)
            print(f"  {name} stopped")

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Wait for any child to exit — if one dies, stop all
    while procs:
        for name, p in procs:
            ret = p.poll()
            if ret is not None:
                print(f"{name} exited ({ret}), stopping all...")
                procs = [(n, q) for n, q in procs if q.pid != p.pid]
                shutdown()
                return


if __name__ == "__main__":
    main()
