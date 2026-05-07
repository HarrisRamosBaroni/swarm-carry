"""
Ensure your network.yaml is correct before running.


Usage:
python real_robot\laptop\record.py

"""



import zmq
import yaml
import csv
import time
import msgpack
from datetime import datetime

CONFIG_FILE = "real_robot/config/network.yaml"
OUTPUT_FILE = "zmq_log_" + str(int(time.time())) + ".csv"


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

        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)

        try:
            while True:
                events = dict(poller.poll(100))  # timeout in ms

                if socket in events:
                
                    # msg_parts = socket.recv_multipart()

                    # timestamp = datetime.utcnow().isoformat()

                    # if len(msg_parts) == 2:
                    #     topic = msg_parts[0].decode("utf-8")

                    #     try:
                    #         data = msgpack.unpackb(msg_parts[1], raw=False)
                    #     except Exception:
                    #         data = str(msg_parts[1])

                    # else:
                    #     topic = "unknown"
                    #     data = str(msg_parts)

                    # writer.writerow([timestamp, topic, data])
                    # f.flush()

                    msg_parts = socket.recv_multipart()
                    timestamp = datetime.utcnow().isoformat()

                    topic = None
                    data = None

                    try:
                        if len(msg_parts) == 1:
                            # No topic, just payload
                            try:
                                unpacked = msgpack.unpackb(msg_parts[0], raw=False)
                                topic = unpacked.get("t", "no_topic")
                                data = unpacked
                            except Exception:
                                topic = "raw"
                                data = str(msg_parts[0])

                        elif len(msg_parts) == 2:
                            # Standard topic + payload
                            topic = msg_parts[0].decode("utf-8")

                            try:
                                data = msgpack.unpackb(msg_parts[1], raw=False)
                            except Exception:
                                data = str(msg_parts[1])

                        else:
                            # Unexpected format
                            topic = "multi_part"
                            data = str(msg_parts)

                    except Exception as e:
                        topic = "error"
                        data = str(e)

                    writer.writerow([timestamp, topic, data])
                    f.flush()

        except KeyboardInterrupt:
            print("\nLogging stopped cleanly.")


if __name__ == "__main__":
    main()