import zmq
import yaml
import csv
import time
from datetime import datetime

CONFIG_FILE = "real_robot/config/network.yaml"
OUTPUT_FILE = "zmq_log.csv"


def load_config(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def build_endpoints(config):
    endpoints = []

    # Robot publishers
    for robot in config.get("robots", []):
        ip = robot["ip"]
        port = robot["pub_port"]
        endpoints.append(f"tcp://{ip}:{port}")

    # Laptop publishers
    laptop = config.get("laptop", {})
    if "ip" in laptop:
        ip = laptop["ip"]

        if "mocap_pub_port" in laptop:
            endpoints.append(f"tcp://{ip}:{laptop['mocap_pub_port']}")

        if "central_pub_port" in laptop:
            endpoints.append(f"tcp://{ip}:{laptop['central_pub_port']}")

    return endpoints


def main():
    config = load_config(CONFIG_FILE)
    endpoints = build_endpoints(config)

    print("Connecting to endpoints:")
    for ep in endpoints:
        print(f"  {ep}")

    context = zmq.Context()
    socket = context.socket(zmq.SUB)

    # Subscribe to ALL topics
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    # Connect to all endpoints
    for ep in endpoints:
        socket.connect(ep)

    print("Logging started... Press Ctrl+C to stop.")

    with open(OUTPUT_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "endpoint", "message"])

        try:
            while True:
                msg = socket.recv()

                timestamp = datetime.utcnow().isoformat()

                # If messages are bytes, decode safely
                try:
                    msg_str = msg.decode("utf-8")
                except UnicodeDecodeError:
                    msg_str = str(msg)

                # NOTE: ZMQ SUB does not tell you which endpoint sent it
                writer.writerow([timestamp, "unknown", msg_str])
                f.flush()

        except KeyboardInterrupt:
            print("\nLogging stopped.")


if __name__ == "__main__":
    main()