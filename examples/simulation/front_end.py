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

import paho.mqtt.client as mqtt
import time
import json
import numpy as np
import copy
import math
import random as rd
import numpy.random as nprd
import sys

if __name__ == '__main__':
    # Robot name
    robot_id = sys.argv[1]
# robot_id = "r2"
    broker = 'localhost'

#inputs (subs)
    cmd_feedback_topic = '/'.join([robot_id,'cmd_feedback']) 
    synchronized_measures_feedback_topic = '/'.join([robot_id,'measures_feedback']) 
#outputs (pubs)
    synchronized_measures_pose_topic = '/'.join([robot_id,'measures_pose']) 
    odom_topic ='/'.join([robot_id,'odom'])

# aggr_state
    g_aggr_state = np.zeros([3,1])
    g_aggr_cov = np.zeros([3,3])
    g_seq = 0

# return zero_state,zero_cov
    def reset_aggr():
        global g_aggr_cov
        global g_aggr_state
        g_aggr_state = np.zeros([3,1])
        g_aggr_cov = np.zeros([3,3])

    def ecpi(a):
        return math.atan2(math.sin(a),math.cos(a))

    def compute_helper_odom_DD(cmd_dd: np.ndarray
                            , cmd_dd_cov: np.ndarray
                            , current_th
                            , dt = 1) -> np.ndarray:
        th=current_th
        v = cmd_dd[0,0]
        w = cmd_dd[1,0]
        sth = math.sin(th)
        sth_n = math.sin(th+w*dt)          # sin(theta_new)
        cth = math.cos(th)
        cth_n = math.cos(th+w*dt)
        if (math.fabs(w) > 1e-6 and math.fabs(v/w) < 1e+6):
            print(f'\n[FrontEnd::{robot_id}] DD: Precise model')
            # V
            V = np.array([[(sth_n-sth)/w, v*(sth-sth_n)/w**2+ v*cth_n*dt/w]
                        , [(cth-cth_n)/w,-v*(cth-cth_n)/w**2+ v*sth_n*dt/w]
                        , [        0     ,       dt                       ]])
            # G
            G = np.array([[ 1 , 0 , v/w*(cth_n - cth)]
                         ,[ 0 , 1 , v/w*(sth_n - sth)]
                         ,[ 0 , 0 ,         1        ]])
            # dstate_vec (the additive state)
            dstate_vec = np.array([[v/w*(sth_n - sth)
                                   ,v/w*(cth - cth_n)
                                   ,    w*dt]]).T
        else : # euler
            print(f'\n[FrontEnd::{robot_id}] DD: Euler simpler model')
            # V
            V = np.array([[cth*dt, 0]
                         ,[sth*dt, 0]
                         ,[ 0    ,dt]])
            # G
            G = np.array([[ 1 , 0 , -v*sth*dt]
                         ,[ 0 , 1 ,  v*cth*dt]
                         ,[ 0 , 0 ,     1    ]])
            # dstate_vec (the additive state)
            dstate_vec = np.array([[v*cth*dt,v*sth*dt,w*dt]]).T

        # TODO: this belong to the simu
        # M = np.array([[alpha1*v**2,0],[0,alpha2*w**2]]) 
        M = cmd_dd_cov
        return G , V@M@V.T, dstate_vec

    def process_cmd_feedback_AA(cmd_AA_vec: np.ndarray,cmd_AA_cov : np.ndarray):
        # Update step of the odom covariance
        global g_aggr_cov
        global g_aggr_state
        g_aggr_cov[0:2,0:2] += cmd_AA_cov
        g_aggr_state[0:2] += cmd_AA_vec

    def process_cmd_feedback_DD(cmd_DD_vec: np.ndarray,cmd_DD_cov : np.ndarray):

        global g_aggr_state
        # compute the helper that will help us compute the new covariance state
        current_th = g_aggr_state[2,0]
        G,R, dstate_vec = compute_helper_odom_DD(cmd_DD_vec,cmd_DD_cov,current_th)

        # Update step of the odom covariance
        global g_aggr_cov
        tmp = copy.deepcopy(g_aggr_cov)
        g_aggr_cov = G@tmp@G.T + R
        # g_aggr_cov = G@g_aggr_cov@G.T + R  # TODO: check if impacting
        g_aggr_state += dstate_vec
        g_aggr_state[2,0] = ecpi(g_aggr_state[2,0])

        # Pb, TODO : comment le back end va s'en servir de ce facteur (re-lineariser)
        #           (ignorer pb pour l'instant)




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
        client.subscribe(cmd_feedback_topic)
        client.subscribe(synchronized_measures_feedback_topic)


    def on_disconnect(client, userdata, flags, rc=0):
        print('Disconnected result code : ' + str(rc))


    def on_message(client, userdata, message):
        # print("Received message '" + str(message.payload) + "' on topic '"
        #       + message.topic + "' with QoS " + str(message.qos) + "'\n")

        # decode payload as string, and transform as JSON
        msg = json.loads(message.payload.decode('utf-8'))
        # depending on the topic, disptach to the user defined functions
        if (message.topic == cmd_feedback_topic) and (msg['robot_id']==robot_id):
            data = msg['feedback_vel']
            cmd_feedback = np.array([data['cmd']]).T
            cmd_feedback_cov = np.array([data['cmd_cov']]).reshape(2,2)
            print(f'\n[FrontEnd::{robot_id}] R {cmd_feedback_topic} :\n {data}')
            if (data['type'] == 'AA'):
                process_cmd_feedback_AA(cmd_feedback,cmd_feedback_cov)
                # fill in the output msg
                odom = { 'robot_id':msg['robot_id']
                        ,'type':data['type']
                        ,'state':{'dx':g_aggr_state[0,0],'dy':g_aggr_state[1,0]}
                        ,'covariance': g_aggr_cov[0:2,0:2].reshape(4,).tolist()
                        ,'visual_covariance':getVisualFromCovMatrix(g_aggr_cov[0:2,0:2])
                        }
            elif (data['type'] == 'DD'):
                process_cmd_feedback_DD(cmd_feedback,cmd_feedback_cov)
                # fill in the output msg
                odom = { 'robot_id':msg['robot_id']
                        ,'type':'SE2'
                        ,'state':{'dx':g_aggr_state[0,0],'dy':g_aggr_state[1,0]
                                    ,'dth':g_aggr_state[2,0]}
                        ,'covariance': g_aggr_cov.reshape(9,).tolist()
                        ,'visual_covariance':getVisualFromCovMatrix(g_aggr_cov[0:2,0:2])
                        }
            else:
                raise NotImplementedError

            # I add another field, with sigmas and angle to make things easier in JS
            # odom['visual_covariance'] = getVisualFromCovMatrix(g_aggr_cov[0:2,0:2])
            client.publish(odom_topic, json.dumps(odom) )
        elif (message.topic == synchronized_measures_feedback_topic) \
                and (msg['robot_id'] == robot_id):
            # The measures have been synchronized upstream with one feedback 
            # The feedback must be treated as 
            data = msg['feedback_vel']
            cmd_feedback = np.array([data['cmd']]).T
            cmd_feedback_cov = np.array([data['cmd_cov']]).reshape(2,2)
            print(f'\n[FrontEnd::{robot_id}] R {cmd_feedback_topic} :\n {data}')
            if (data['type'] == 'DD'):
                process_cmd_feedback_DD(cmd_feedback,cmd_feedback_cov)
                # fill in the output msg
                odom = {
                         'type': 'SE2'
                        ,'state':{'dx':g_aggr_state[0,0],'dy':g_aggr_state[1,0]
                                    ,'th':g_aggr_state[2,0]}
                        ,'covariance': g_aggr_cov.reshape(9,).tolist()
                        # ,'true_pose': msg['true_pose']
                        }
            elif (data['type'] == 'AA'):
                process_cmd_feedback_AA(cmd_feedback,cmd_feedback_cov)
                # fill in the output msg
                odom = {
                         'type':data['type']
                        ,'state':{'dx':g_aggr_state[0,0],'dy':g_aggr_state[1,0]}
                        ,'covariance': g_aggr_cov[0:2,0:2].reshape(4,).tolist()
                        # ,'true_pose': msg['true_pose']
                        }
            else:
                raise NotImplementedError
            msg.pop('feedback_vel')
            msg['odom']=odom
            client.publish(synchronized_measures_pose_topic,json.dumps(msg))
            reset_aggr()
            


    def getVisualFromCovMatrix(cov : np.ndarray) -> dict:
        eVa,eVec = np.linalg.eig(cov)
        R = eVec
        angle = math.atan2(R[1,0],R[0,0])
        sigmax=math.sqrt(eVa[0])
        sigmay=math.sqrt(eVa[1])
        return { 'sigma': [sigmax, sigmay], 'rot': angle }

    print("This is the estimation front end for robot : ", robot_id)

    # -------------------------------------------------------------------------
    #                       MQTT connection
    # -------------------------------------------------------------------------

    mqttc = mqtt.Client('front-end-py_'+ robot_id)
    print("Connecting to broker : ", broker)

    # the on_connect method will set the subscriber
    mqttc.on_connect = on_connect
    # mqttc.on_log=on_log
    mqttc.on_message = on_message

    # connect to the broker
    mqttc.connect(broker)

    # block the code here, process the messages for the subscribed topics
    print('looping')
    mqttc.loop_forever()
    print('disconnecting')
    mqttc.disconnect()
