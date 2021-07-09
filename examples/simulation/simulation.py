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

# TODO introduce global flags such as type of commands, AA (Axis-Aligned) etc..
# TODO dynamic change of std-dev noise
# TODO support for bearing (only use bearing sighting of range-bearing)


# type supported for observation sensor: range-AA (linear), range-bearing
# type supported for odometry sensor: odometry-AA (linear), DD (differential-drive)

# ----------------------------------------------------------------------------
#                           Globals
# ----------------------------------------------------------------------------
world = {
    "robots":
    {
        'r1': {
            "robot_id": "r1",
            "state": {"x": 30, "y": 8, "th": 120*math.pi/180},
            # "state": {"x": 5, "y": 6.1, "th": 30*math.pi/180},
            "sensors": { "observation": {"range": 12, "angle_coverage": 0.75, 'type': 'range-AA'},
                "odometry": {"type": "AA"}
                         }
        },
        'r2': {
            "robot_id": "r2",
            "state": {"x": 50, "y": 8, "th": 90*math.pi/180},
            "sensors": { "observation": {"range": 12, "angle_coverage": 0.167, 'type': 'range-bearing'},
                "odometry": {"type": "DD"}
                         }
        },
        'r3': {
            "robot_id": "r3",
            "state": {"x": 70, "y": 8, "th": 60*math.pi/180},
            "sensors": { "observation": {"range": 12, "angle_coverage": 0.5, 'type': 'range-bearing'},
                "odometry": {"type": "DD"}
                         }
        },
    },
    "landmarks":
    [
        {"landmark_id": "l1", "state": {"x": 8, "y": 8}},
        {"landmark_id": "l2", "state": {"x": 16, "y": 8}},
        {"landmark_id": "l3", "state": {"x": 8, "y": 16}},

        {"landmark_id": "l4", "state": {"x": 92, "y": 8}},
        {"landmark_id": "l5", "state": {"x": 84, "y": 8}},
        {"landmark_id": "l6", "state": {"x": 92, "y": 16}},

        {"landmark_id": "l7", "state": {"x": 92, "y": 52}},
        {"landmark_id": "l8", "state": {"x": 84, "y": 52}},
        {"landmark_id": "l9", "state": {"x": 92, "y": 44}},

        {"landmark_id": "l10", "state": {"x": 8, "y": 52}},
        {"landmark_id": "l11", "state": {"x": 16, "y": 52}},
        {"landmark_id": "l12", "state": {"x": 8, "y": 44}},

        {"landmark_id": "l13", "state": {"x": 45, "y": 33}},
        {"landmark_id": "l14", "state": {"x": 55, "y": 33}},
        {"landmark_id": "l15", "state": {"x": 50, "y": 26}},
        # {"landmark_id": "l13", "state": {"x": 50, "y": 30}},
    ]
}
# store positions ini of each agent
# robots_ini_poses = [{"robot_id": ritem["robot_id"],
#                      "position_ini":ritem["state"]} for rkey,ritem in
#                      world['robots'].items()]
robots_ini_poses = {key: {'robot_id': val['robot_id'], 'state': val['state']} for (
    key, val) in world['robots'].items()}
print(robots_ini_poses)


# last referenced pose to decide when to generate a new odometry measurement
last_pose = copy.deepcopy(world['robots'])

new_pose_distance_threshold = 10

# seeding randomness (for the numpy stuff)
# https://numpy.org/doc/stable/reference/random/index.html#random-quick-start
rng = nprd.default_rng()

# noise cmd (diag)
# 0 -> no noise
cmd_std_dev_ratio_y = 5.0/100
cmd_std_dev_ratio_x = 5.0/100

# noise cmd (diag, DD)
# 0 -> no noise
# cmd_std_dev_ratio_v = 5.0/100
# cmd_std_dev_ratio_w = 9.0/100

# meas noise (diag)
# 0 -> no noise
# lets say a std dev of 3cm for each measured unit on X
# lets say a std dev of 2cm for each measured meter on Y
measure_std_dev_ratio_x = 8.0/100
measure_std_dev_ratio_y = 8.0/100

# the cumulative odom store the effect of the accumulation of cmd topics.
# Once the distance threshold is reached, this object is consummed and reset
# in order to construct the odometry measurement.
# It evolved like the variance of a random walk.
# cumulative_odom_cov = np.zeros([2, 2])

# some mqtt related globals
broker = 'localhost'

# input (subs) topics
cmd_topic = 'cmd'
request_ground_truth_topic = 'request_ground_truth'
request_position_ini_topic = 'request_position_ini'
# output (published) topics
cmd_feedback_topic = 'cmd_feedback'
measures_topic = 'measures_feedback'
ground_truth_topic = 'ground_truth'
position_ini_topic = 'position_ini'

# init flag
firstTime = True

# ----------------------------------------------------------------------------
#                           math utility
# ----------------------------------------------------------------------------


def sq_dist(p1: dict, p2: dict) -> float:
    return (p1['x']-p2['x'])**2+(p1['y']-p2['y'])**2


def sqrt_dist(p1: dict, p2: dict) -> float:
    return math.sqrt(sq_dist(p1, p2))


def dx(p_ref: dict, p_target: dict) -> float:
    return p_target['x']-p_ref['x']


def dy(p_ref: dict, p_target: dict) -> float:
    return p_target['y']-p_ref['y']


def ecpi(a):
    return math.atan2(math.sin(a), math.cos(a))


def relative_angle(sensorPos: dict, target: dict):
    angle_world_frame = math.atan2(
        target['y'] - sensorPos['y'], target['x'] - sensorPos['x'])
    return ecpi(angle_world_frame-sensorPos['th'])


def noisify_cmd_AA(exact_cmd: np.ndarray):
    # cmd_array = [cmd['x'],cmd['y']];
    # exact_cmd = np.array([cmd_array]).T
    # print(f'exact_cmd is of shape {exact_cmd.shape}')
    # compute the covariance that will generate the noise
    cov = generate_covariance_noise(
        exact_cmd, [cmd_std_dev_ratio_x, cmd_std_dev_ratio_y])
    # generate the random measure with additive noise
    return rng.multivariate_normal(exact_cmd.reshape(2,), cov).reshape(2, 1), cov


def noisify_cmd_DD(exact_cmd_vec: np.ndarray):
    v = exact_cmd_vec[0, 0]
    w = exact_cmd_vec[1, 0]
    # exact_cmd_vec = np.array([[v,w]]).T;
    alpha1 = 0.0029  # -> 5% sigma on v
    # alpha2 = 0.0002 #-> 1.4% sigma of w on v
    # alpha3 = 0.00002#-> 0.44% sigma of v on w
    alpha4 = 0.01  # -> 5% sigma on v
    alpha2 = 0.00002
    alpha3 = 0.002
    dt = 1
    # + np.square(np.diag([1e-3,1e-5]))
    cov = np.diag([alpha1*v**2+alpha2*w**2, alpha3*v**2+alpha4*w**2])

    # the process noise covariance must be translate in state space noise
    return \
        rng.multivariate_normal(exact_cmd_vec.reshape(
            2,), cov).reshape(2, 1), cov  # *1.2 # TODO coef


def measure_robot_landmark(robotstate: dict, landmark: dict, sensor_type: str) -> dict:
    xl = landmark['state']['x']
    yl = landmark['state']['y']
    xr = robotstate['x']
    yr = robotstate['y']
    thr = robotstate['th']

    if(sensor_type == 'range-AA'):
        # an exact measurement would be
        exact_measure = np.array([[xl-xr, yl-yr]]).T

        # compute the covariance that will generate the noise
        cov = generate_covariance_noise(
            exact_measure, [measure_std_dev_ratio_x, measure_std_dev_ratio_y])

        # generate the random measure with additive noise
        vect = exact_measure + \
            rng.multivariate_normal(np.zeros(2), cov).reshape(2, 1)
    elif(sensor_type == 'range-bearing'):
        # an exact measurement would be [r,alpha]
        r, alpha = sqrt_dist(landmark['state'], robotstate), relative_angle(
            robotstate, landmark['state'])
        exact_measure = np.array([[r, alpha]]).T

        # compute the covariance that will generate the noise
        cov = np.array([[(r*5./100)**2, 0], [0, 0.1**2]])

        # generate the random measure with additive noise
        vect = exact_measure + \
            rng.multivariate_normal(np.zeros(2), cov).reshape(2, 1)
    else:  # TODO: bearing
        raise NotImplementedError

    # return the measure and its covariance confidence
    # Important: the covariance returned corresponds to the real noise,
    #            which might be different than in reality
    # lastly, the np objects are serialized
    return {
        'landmark_id': landmark['landmark_id'], 'type': sensor_type, 'vect':
        vect.reshape(2,).tolist(), 'covariance': cov.reshape(4,).tolist()
    }

# TODO : bearing only sensor mes

#


# def generate_odom_measurement(current_pose: dict, last_pose: dict):
#     global cumulative_odom_cov
#     # exact odometry measurement
#     exact_odom_mes = np.array(
#         [[dx(last_pose, current_pose), dy(last_pose, current_pose)]])

#     # TODO change when its 3x3 with theta
#     # generate the noisy odometry according to the cumulative noise
#     vect_odom_mes = exact_odom_mes + \
#         rng.multivariate_normal(np.zeros(2), cumulative_odom_cov)

#     full_odom_mes = {'type': 'AAOdom',        # AA means axis-aligned
#                      'vect': vect_odom_mes.reshape(2,).tolist(),
#                      'covariance': cumulative_odom_cov.reshape(4,).tolist()
#                      }

#     # reset cumulative_cov value until next time
#     cumulative_odom_cov = np.zeros([2, 2])

#     return full_odom_mes

def in_sensor_coverage(sensorPos: dict, target: dict, sensor_observation_info: dict) -> bool:
    isInRange = sqrt_dist(sensorPos, target) < sensor_observation_info['range']
    isAngleCovered = abs(relative_angle(sensorPos, target)) / \
        math.pi < sensor_observation_info['angle_coverage']
    return isInRange and isAngleCovered


def generate_transient_odom_covariance_AA(exact_vect: np.ndarray,
        std_dev_stats: list, isNoiseAxisAligned=False,
        maxCovarianceSkew=math.pi/8) -> np.ndarray:
    return generate_covariance_noise(exact_vect, std_dev_stats,
            isNoiseAxisAligned, maxCovarianceSkew)


def generate_covariance_noise(exact_vect: np.ndarray, std_dev_stats: list,
        isNoiseAxisAligned=False, maxCovarianceSkew=math.pi/8,
        zero_move_std_noise=1e-3) -> np.ndarray:
    # axis aligned covariance I want for odom measurement
    # It's a noise that depend on distance plus a smaller noise (that apply even at zero)
    cov_AA = np.square(np.diag(std_dev_stats)
                       @ np.diag(exact_vect.reshape(2,).tolist())
                       + np.diag([zero_move_std_noise, zero_move_std_noise]))

    if not isNoiseAxisAligned:
        # a rotational angle is defined randomly to skew the axis aligned covariance
        rot_angle = rd.uniform(-maxCovarianceSkew, maxCovarianceSkew)
        rot_mat = np.array([[math.cos(rot_angle), - math.sin(rot_angle)],
                            [math.sin(rot_angle), math.cos(rot_angle)]])
        cov = rot_mat @ cov_AA @ rot_mat.T
    else:
        cov = cov_AA
    return cov

# def integrate_cumulative_odometry(cmd_cov: np.ndarray) -> None:
#     global cumulative_odom_cov
#     cumulative_odom_cov += cmd_cov
#     # TODO if its not linear, use recursive bayesian filter
#     #       In linear X,Y this is easy because the cmd and the position are on the same space
#     #       But in NL, the command is v,w and the pos is 3d (x y theta)
#     #       In that case, apply the recursive linearized filter, which the
#     update part of the traditional EKF SLAM in 2.5d

# ----------------------------------------------------------------------------
#                        User defined callback functions
# ----------------------------------------------------------------------------

# currently the exact cmd


def apply_cmd_to_ground_truth_AA(cmd: np.ndarray, cur_state: dict) -> dict:
    # return {'x': cur_state['x']+cmd['x'],'y': cur_state['y']+cmd['y'],'th': cur_state['th']}
    x = cmd[0, 0]
    y = cmd[1, 0]
    th = cur_state['th']
    dt = 1
    return {'x': cur_state['x'] + x*dt*math.cos(th) - y*dt*math.sin(th), 'y':
            cur_state['y'] + x*dt*math.sin(th) + y*dt*math.cos(th), 'th': th}


def apply_cmd_to_ground_truth_DD(cmd: np.ndarray, cur_state: dict) -> dict:
    # v = cmd['linear']
    # w = cmd['angular']
    v = cmd[0, 0]
    w = cmd[1, 0]
    dt = 1
    th = cur_state['th']
    return {'x': cur_state['x'] + v*math.cos(th)*dt, 'y': cur_state['y'] +
            v*math.sin(th)*dt, 'th': ecpi(th + w*dt)}


def get_robot_index_in_world(world: dict, robot_id: str):
    # generator
    g = (i for i, d in
         enumerate(world['robots'])
         if d['robot_id'] == robot_id)
    #
    return next(g)


def cmd_vel_callback(client, msg):
    # print('cmd')
    # 0a/ prepare variables from the message
    received_cmd = json.loads(msg)
    robot_id = received_cmd['robot_id']
    cmd_type = received_cmd['type']
    # robot_cmd_type = world['robot'][robot_id][sensors][odometry]["type"]
    cmd_vel = received_cmd['cmd_vel']
    # 0b/ get the ground truth associated with this robot
    current_robot_pos = copy.deepcopy(world['robots'][robot_id]['state'])
    global last_pose # TODO proper way to do it
    #
    if (cmd_type == 'AA'):
        cmd_vel_vec = np.array(
            [[cmd_vel['x'], cmd_vel['y']]]).T*1.0  # cast as float
        # 1/ noisify the order and update cumulative odom cov
        # hundred fold to make it easy on the covariance calculations
        cmd_vel_vec *= 100
        noisy_cmd, cov_cmd = noisify_cmd_AA(cmd_vel_vec)
        cmd_vel_vec /= 100
        noisy_cmd /= 100
        cov_cmd /= 100
        # 2/ update robot pos in the world (ground truth)
        world['robots'][robot_id]['state'] = \
            apply_cmd_to_ground_truth_AA(noisy_cmd, current_robot_pos)
    elif (received_cmd['type'] == 'DD'):
        cmd_vel_vec = np.array(
            [[cmd_vel['linear'], cmd_vel['angular']]]).T*1.0  # float-cast
        # 1/ noisify the order and update cumulative odom cov
        # hundred fold to make it easy on the covariance calculations
        cmd_vel_vec *= 100
        noisy_cmd, cov_cmd = noisify_cmd_DD(cmd_vel_vec)
        cmd_vel_vec /= 100
        noisy_cmd /= 100
        cov_cmd /= 100
        # 2/ update robot pos in the world (ground truth)
        world['robots'][robot_id]['state'] = \
            apply_cmd_to_ground_truth_DD(noisy_cmd, current_robot_pos)
    else:
        print('I don''t support that type of cmd yet')
        raise NotImplementedError

    # update some variables in preparation for step 5
    current_robot_pos = world['robots'][robot_id]['state']
    feedback_vel = \
        {
            'type': received_cmd['type'],
            'cmd': cmd_vel_vec.reshape(2,).tolist(),
            'cmd_cov': cov_cmd.reshape(4,).tolist()
        }
    # 4/ publish ground truth
    client.publish(robot_id+'/'+ground_truth_topic,
                   json.dumps(world['robots'][robot_id]))
    # 5/ check if measures should be generated
    # 5.1/ if no, publish just the cmd_feedback and return
    if sqrt_dist(current_robot_pos, last_pose[robot_id]['state']) \
            < new_pose_distance_threshold:
        message_payload = \
            {
                'robot_id': received_cmd['robot_id'],
                'feedback_vel': feedback_vel
            }
        client.publish(robot_id+'/'+cmd_feedback_topic,
                       json.dumps(message_payload))
        # return
        # 5.2 if yes, generate sensor measurements and publish them with cmd_feedback
    else:
        # 5.2.1 generate robot to landmarks measure
        #       np arrays are not json serializable, so I to convert them to list first
        #       with the np::tolist() method
        sensor_observation_info = world['robots'][robot_id]['sensors']['observation']
        # comprehension list
        landmarks_measurements = \
            [
                measure_robot_landmark(
                    current_robot_pos, l, sensor_observation_info['type'])
                for l in world['landmarks']
                if in_sensor_coverage(current_robot_pos, l['state'], sensor_observation_info)
            ]
        # 5.2.2 save the new robot position as the last pose node
        last_pose[robot_id]['state'] = copy.deepcopy(current_robot_pos)
        # add a truth
        # 5.2.3 publish the measurements in the same package
        mes_payload = \
            {
                'robot_id': received_cmd['robot_id'],
                'feedback_vel': feedback_vel,
                'measures': landmarks_measurements,
                'true_pose': copy.deepcopy(current_robot_pos)
            }
        client.publish(robot_id+'/'+measures_topic, json.dumps(mes_payload))


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
    for robot_id, _ in world['robots'].items():
        client.subscribe('/'.join([robot_id, cmd_topic]))
        client.subscribe('/'.join([robot_id, request_position_ini_topic]))
    client.subscribe(request_ground_truth_topic)


def on_disconnect(client, userdata, flags, rc=0):
    print('Disconnected result code : ' + str(rc))


def on_message(client, userdata, message):
    # print("Received message '" + str(message.payload) + "' on topic '"
    #       + message.topic + "' with QoS " + str(message.qos) + "'\n")

    # decode payload as string
    msg = message.payload.decode('utf-8')
    # depending on the topic, disptach to the user defined functions
    # break down the topic name
    if (len(message.topic.split('/')) == 1):
        if message.topic == request_ground_truth_topic:
            client.publish(ground_truth_topic, json.dumps(world))
            print(f'[SimuPy] R {ground_truth_topic}')
        # elif message.topic == request_position_ini_topic:
        #     client.publish(position_ini_topic, json.dumps(robots_ini_poses))
        else:
            print('No callback for topic : ' + message.topic)
            raise NotImplementedError
    elif (len(message.topic.split('/')) == 2):
        robot_id, topic_suffix = message.topic.split('/')
        if topic_suffix == cmd_topic:
            cmd_vel_callback(client, msg)
            print(f'\n[SimuPy::{robot_id}] R {cmd_feedback_topic}:\n {msg}')
        elif topic_suffix == request_ground_truth_topic:
            client.publish(
                '/'.join([robot_id, ground_truth_topic]), json.dumps(world['robot_id']))
            print(f'\n[SimuPy::{robot_id}] R {ground_truth_topic}:\n {msg}')
        elif topic_suffix == request_position_ini_topic:
            client.publish(
                '/'.join([robot_id, position_ini_topic]), json.dumps(robots_ini_poses[robot_id]))
            print(
                f'\n[SimuPy::{robot_id}] R {request_position_ini_topic}:\n {msg}')
        else:
            print('No callback for topic : ' + message.topic)
            raise NotImplementedError
    else:
        print('Unexpected topic name breakdown')
        raise NotImplementedError

    # if firstTime:
    #      # publish
    # print('publishing initial pose for robots')
# # TODO: make it accessible on demand
    #  request_position_ini_topic
    #  position_ini_topic


# print('\n')


# -----------------------------------------------------------------------------
#                           Main
# -----------------------------------------------------------------------------
if __name__ == '__main__':
    # -------------------------------------------------------------------------
    #                       Print Simulation Data
    # -------------------------------------------------------------------------
    print(f"Simulation World :\n${world}")

    # -------------------------------------------------------------------------
    #                       MQTT connection
    # -------------------------------------------------------------------------

    mqttc = mqtt.Client('simulation-py')
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
