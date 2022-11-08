#! /usr/bin/env python3
import argparse
import paho.mqtt.client as mqtt 
import json


class MQTTEcho():
    """
    A class that connectes to an MQTT broker for debugging. 

    Once connected, this class will listen on a specified MQTT topic for messages
    and echo them to the command line with their topic once received.
    """

    def __init__(self, mqtt_server: str, topic: str):
        self._server = mqtt_server
        self._topic = topic

        client = mqtt.Client()
        client.on_connect = self._on_connect
        client.on_message = self._on_message

        client.connect(mqtt_server, 1883, 60)
        client.loop_forever()

    def _on_connect(self, client, userdata, flags, rc):
        print(f'CONNECTED to {self._server}: rc({rc})' + f'\n\tsubscribing to: {self._topic}')
        client.subscribe(self._topic)

    def _on_message(self, client, userdata, msg):
        dict_msg = json.loads(msg.payload)
        print(f'{msg.topic}: {json.dumps(dict_msg)}')


if __name__ == '__main__':

    parser = argparse.ArgumentParser(
                                        prog='MQTTEcho',
                                        description='creates a listener for messages published on topic and prints them to the command line'
                                    )

    parser.add_argument(
                            'topic',
                            type=str,
                            help='topic to listen to, of form \'topic/str/\' (add # to get all topics at a level)'
                        )

    parser.add_argument(
                            '-b',
                            '-broker',
                            action='store',
                            dest='broker',
                            default='192.168.0.104',
                            help='address for the mqtt broker to listen to, should be IP address or other address'
                        )
    
    args = parser.parse_args()

    MQTTEcho(mqtt_server=args.broker, topic=args.topic)
