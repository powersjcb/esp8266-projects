from pythonosc import osc_message_builder
from pythonosc import udp_client
import argparse
import random
import time


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.4.22",
                        help="The ip of the OSC server")
    parser.add_argument("--port", type=int, default=8888,
                        help="The port the OSC server is listening on")
    args = parser.parse_args()

    client = udp_client.SimpleUDPClient(args.ip, args.port)

    for x in range(10):
        print('sending message: ' + str(x))
        client.send_message("/", str(x))
        time.sleep(1)
