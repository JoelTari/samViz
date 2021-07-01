#!/usr/bin/python3

# Copyright 2021 AKKA Technologies (joel.tari-summerfield@akka.eu)

# Licensed under the EUPL, Version 1.2 or – as soon they will be approved by
# the European Commission - subsequent versions of the EUPL (the "Licence");
# You may not use this work except in compliance with the Licence.
# You may obtain a copy of the Licence at:

# https://joinup.ec.europa.eu/software/page/eupl

# Unless required by applicable law or agreed to in writing, software
# distributed under the Licence is distributed on an "AS IS" basis,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the Licence for the specific language governing permissions and
# limitations under the Licence.



# This program shows how to use the paho.mqtt library to send a json-formatted
# graph to samViz.
# While this code doesn't explicitly subscribe to any topic (only publish one 
# msg). The on_message() method and the looping are left for reference.

# For more info on paho.mqtt, go to: https://pypi.org/project/paho-mqtt


import paho.mqtt.client as mqtt
import json
import sys

# ----------------------------------------------------------------------------
#                           Globals
# ----------------------------------------------------------------------------
broker = 'localhost'

# ----------------------------------------------------------------------------
#         mqtt layer callbacks overrides (not yet users callbacks)
# ----------------------------------------------------------------------------


def on_log(client, userdata, level, buf):
    print('log: ' + buf)


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print('connected')
    else:
        print('connection error. code :', rc)
    # do the subscriptions here
    # client.subscribe("a_topic_name/data")
    # client.subscribe("another_topic_name")


def on_disconnect(client, userdata, flags, rc=0):
    print('Disconnected result code : ' + str(rc))


def on_message(client, userdata, message):
    print("Received message '" + str(message.payload) + "' on topic '"
          + message.topic + "' with QoS " + str(message.qos) + "'\n")
    # decode payload as string
    msg = message.payload.decode('utf-8')
    # manage subscriptions here


# -----------------------------------------------------------------------------
#                           Main
# -----------------------------------------------------------------------------

if __name__ == '__main__':

    # For this example, we assume the data to publish is given in a file
    try:
        filename=str(sys.argv[1])
    except IndexError:
        print("Error, no filename argument given. Exit code 1")
        sys.exit(1)

    with open(filename) as f:
        # read all lines and join in a string
        json_content="".join(f.readlines())

    # Note: we can convert it to a python dictio with:
    # content = json.loads(json_content)
    #       and back to a str() with:
    # str_content = json.dumps(content)

    # -------------------------------------------------------------------------
    #                       MQTT connection
    # -------------------------------------------------------------------------

    mqttc = mqtt.Client('graph-publish-example-py')
    print("Connecting to broker : ", broker)

    # the on_connect method will set the subscriber(s)
    mqttc.on_connect = on_connect
    # mqttc.on_log=on_log
    mqttc.on_message = on_message

    # connect to the broker
    mqttc.connect(broker)

    # publish stdin
    print('Publish content')
    mqttc.publish("default/graphs",json_content)


    # block the code here, process the messages for the subscribed topics
    # print('looping')
    # mqttc.loop_forever()
    # print('disconnecting')
    mqttc.disconnect()
