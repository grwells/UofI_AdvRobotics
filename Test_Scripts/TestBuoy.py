#! /usr/bin/env python3
# Imports.
import json
import paho.mqtt.client as mqtt
import time

import argparse 


# Main function.
def main(buoy_id: str, mqtt_broker: str, location: tuple):
    # Set variables.
    namespace = buoy_id
    server = mqtt_broker
    port = 1883
    x_pos = location[0]
    y_pos = location[1]

    # Create client.
    client = mqtt.Client()
    client.connect(server, port)

    # Loop.
    while True:
        # Create message.
        gcsp_dict = {
            'pose': {
                'position': {
                    'x': x_pos,
                    'y': y_pos,
                    'z': 0.0,
                },
                'orientation': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'w': 1.0,
                },
            },
            'twist': {
                'linear': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                },
                'angular': {
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                },
            }
        }

        # Broadcast.
        gcsp_message = json.dumps(gcsp_dict)
        client.publish(f"irobot_create3_swarm/pose/{namespace}", gcsp_message)
        time.sleep(0.1)



# Boilerplate.
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                        prog='TestBuoy',
                        description='creates a buoy at point (x,y) which acts as an imaginary obstacle')

    parser.add_argument('-id',
                        '--buoy-id',
                        action='store',
                        dest='id',
                        default='dummy',
                        help='assign a custom buoy id string like \'dummy\' to support differentiation of multiple buoys in one space')

    parser.add_argument('-b', 
                        '--broker', 
                        action='store', 
                        dest='mqtt_broker',
                        default='192.168.0.104',
                        help='specify the location of the MQTT broker to publish to, defaults to 192.168.0.104')

    parser.add_argument('-x', 
                        '--located-at', 
                        action='store', 
                        dest='coordinate',
                        default=(1,0),
                        nargs=2,
                        type=float,
                        help='specify the location of the buoy in a right handed coordinate system, pass as a pair of int/float values like <x> <y>')

    args = parser.parse_args()
    args.coordinate = tuple(args.coordinate)
    print(f'Creating Buoy [{args.id}] @ {args.coordinate} \nMQTT Broker: {args.mqtt_broker}')
    main(buoy_id=args.id, mqtt_broker=args.mqtt_broker, location=args.coordinate)
