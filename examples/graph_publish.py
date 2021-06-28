#!/usr/bin/python3

import paho.mqtt.client as mqtt
# import time
import json
import sys
# import numpy as np
# import copy
# import math
# import random as rd
# import numpy.random as nprd

# ----------------------------------------------------------------------------
#                           Globals
# ----------------------------------------------------------------------------
# some mqtt related globals
broker = 'localhost'

# cmd_topic = 'cmd'
# cmd_feedback_topic = 'cmd_feedback'
# measures_topic = 'measures_feedback'
# request_ground_truth_topic = 'request_ground_truth'
# ground_truth_topic = 'ground_truth'
# request_position_ini_topic = 'request_position_ini'
# position_ini_topic = 'position_ini'


# ----------------------------------------------------------------------------
#                        User defined callback functions
# ----------------------------------------------------------------------------



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
    print('looping')
    mqttc.loop_forever()
    print('disconnecting')
    mqttc.disconnect()
