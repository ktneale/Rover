import json
import serial
from rover.common.udp import udp_send_json

#from rover.common.rover_ports import HOST, EVENT_PORT

SERIAL_PORT = "/dev/ttyACM0"
BAUD = 115200

def main():
    #sock_send = make_udp_sender()
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line or not line.startswith("{"):
            continue

        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue

        if msg.get("T") != "odom":
            continue

        out = {
            "type": "odom",
            "seq": msg.get("seq"),
            "ms": msg.get("ms"),
            "dL": msg.get("dL"),
            "dR": msg.get("dR"),
            "tL": msg.get("tL"),
            "tR": msg.get("tR"),
            "x": msg.get("x"),
            "y": msg.get("y"),
            "th": msg.get("th"),
            "v": msg.get("v"),
            "w": msg.get("w"),
        }


        udp_send_json("10.42.0.213", 5017, out)

if __name__ == "__main__":
    main()
