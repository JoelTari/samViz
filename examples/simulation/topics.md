# Mqtt message structures and topics

## TLDR: topic names

- `cmd`
- `cmd_feedback`
- `ground_truth`
- `measures`
- `request_position_ini`
- `position_ini`

## Command: JS to Simu/Robot

Topic name: `cmd`

```json
{
  "robot_id":"r1",
  "type":"AA",       // or DD, ACKER
  "cmd_vel":{"x": dx, "y":dy } # depend on type
}
```
Other types for `cmd_vel`: `"cmd_vel":{"linear": dlinear, "angular": dangular }`

## Command: Simu/Robot to Estimation

Topic name: `cmd_feedback`

```json
{
  "robot_id":"r1",
  "feedback_vel" :
        {
            'type': 'AA',   // or DD
            'cmd': [dx,dy]
            'cmd_cov': cov
        }
}
```

## World: simu <-> JS

Topic request: `request_ground_truth`

Topic response: `ground_truth`

```json
{
  "robots": [
    {
      "robot_id": "r1",
      "state": { "x": 5, "y": 10, "th": 0 },
      "sensors": { "observation": {"range": 12, "angle_coverage": 0.167, 'type': 'range-bearing'},
          "odometry": {"type": "DD"} // or AA
                   }
    }
  ],
  "landmarks": [
    { "id": "l1", "state": { "x": 6, "y": 2 } },
    { "id": "l2", "state": { "x": 35, "y": 10 } }
  ]
}
```

## Measure package: Simu to Estimation (front-end)

Topic names: `measures`

```json
{
    "robot_id":"r1",
    "feedback_vel" :
    {
        'type': 'AA',   // AA (axis-aligned) or DD (differential drive)
        'cmd': [dx,dy]
        'cmd_cov': cov  // 2x2 serialized to list of 4 elements
    }
      "landmarks":[
      {
          "landmark_id": "l2",
          "type": 'range-bearing'
          "vect": [r,alpha], 
          "cov": covariance,
        },
      {
          "landmark_id": "l5",
          "type": 'range-AA'
          "vect": [lx-px,ly-py], 
          "cov": covariance, 
        },
        ]
  }
```

## Position Initial: Simu <-> Estimation

Request position topic name: `request_position_ini`

Answer position topic name: `position_ini`

```json
# the request
{ "robot_id": 'r2'}
# the response
{
  "robot_id":'r2',
  "type":"AAposition",  # means 2d position, could be SO2position to account for orientation
  "vect_position": vect,  # serialized
  "covariance": cov
}
```

## Estimation resut:  JS <- Estimation (on request)

Request estimation result topic name: `request_estimation`

Answer estimation result topic name:  `estimation`

```python
full_estimations = [
    {
        'header': {
            'robot_id': 'r1',
            'seq': 0,
        },
        'rt_state': {'type': 'relative',  # absolute or relative to last state (last pose mst exist)
                     "state": {"x": 40.3, "y": 17.4, "th": 47*math.pi/180},
                     'covariance': {'sigma': [2, 0.9], 'rot': 0.314}},
        'last_pose': {'last_pose_id': 'x4'},
        # TODO : plural graph with graph id for each graph and a header designating
        #        the main hypothesis
        # 'main_hypothesis_graph_id':'g1'
        'graph': {  # TODO: plural-> array, one header per element and the following encapsulated in 'data',
            #
            'marginals': [
                {'var_id': 'x0', 'mean': {'x': 6.5, 'y': 12},
                 'covariance': {'sigma': [rSig(0.2, 0.5), rSig(0.1, 0.3)], 'rot':rRot()}},
                {'var_id': 'x1', 'mean': {'x': 14, 'y': 10.6},
                 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'x2', 'mean': {'x': 21.1, 'y': 10.5},
                 'covariance': {'sigma': [rSigMul(3/2), rSigMul(1)], 'rot':rRot()}},
                {'var_id': 'x3', 'mean': {'x': 27.7, 'y': 10.9},
                 'covariance': {'sigma': [rSigMul(2), rSigMul(3/2)], 'rot':rRot()}},
                {'var_id': 'x4', 'mean': {'x': 34.88, 'y': 13.8},
                 'covariance': {'sigma': [rSigMul(2.8), rSigMul(2.5)], 'rot':rRot()}},
                {'var_id': 'l2', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l2')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l2')[
                            'state']['y']+rXY()
                }, 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'l7', 'mean': {
                    'x': 45.9, 'y': 8.75
                }, 'covariance': {'sigma': [rSigMul(2), rSigMul(1.5)], 'rot':rRot()}},
            ],

            'factors': [
                {'factor_id': 'f0',
                 'type': 'ini_position',
                 'vars_id': ['x0'],
                 },
                {'factor_id': 'f1',
                 'type': 'rb',
                 'vars_id': ['l2', 'x0'],
                 },
                {'factor_id': 'f2',
                 'type': 'odometry',
                 'vars_id': ['x1', 'x0'],
                 },
                {'factor_id': 'f3',
                 'type': 'odometry',
                 'vars_id': ['l2', 'x1'],
                 },
                {'factor_id': 'f4',
                 'type': 'odometry',
                 'vars_id': ['x2', 'l2'],
                 },
                {'factor_id': 'f5',
                 'type': 'odometry',
                 'vars_id': ['x1', 'x2'],
                 },
                {'factor_id': 'f6',
                 'type': 'odometry',
                 'vars_id': ['x2', 'x3'],
                 },
                {'factor_id': 'f7',
                 'type': 'odometry',
                 'vars_id': ['x4', 'x3'],
                 },
                {'factor_id': 'f8',
                 'type': 'odometry',
                 'vars_id': ['x4', 'l7'],
                 },
            ],
        }
    },
    # The second robot
    {
        'header': {
            'robot_id': 'r2',
            'seq': 0,
        },
        'rt_state': {'type': 'relative',  # absolute or relative to last state (last pose mst exist)
                     "state": {"x": 26.5, "y": 50.46, "th": -0.5},
                     'covariance': {'sigma': [2, 0.9], 'rot': 0.314}},
        'last_pose': {'last_pose_id': 'x2'},
        'graph': {
            'marginals': [
                {'var_id': 'x0', 'mean': {'x': 7.3, 'y': 51.6},
                 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'x1', 'mean': {'x': 15, 'y': 50.5},
                 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'x2', 'mean': {'x': 21.12, 'y': 50.1},
                 'covariance': {'sigma': [rSigMul(1.1), rSigMul(1.2)], 'rot':rRot()}},
                {'var_id': 'l1', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l1')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l1')[
                            'state']['y']+rXY()
                },
                    'covariance': {'sigma': [rSigMul(1.5), rSigMul(0.75)], 'rot':rRot()}},
                {'var_id': 'l3', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l3')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l3')[
                            'state']['y']+rXY()
                },
                    'covariance': {'sigma': [rSigMul(0.8), rSigMul(1)], 'rot':rRot()}},
            ],

            'factors': [
                {'factor_id': 'f0',
                 'type': 'ini_position',
                 'vars_id': ['x0'],
                 },
                {'factor_id': 'f1',
                 'type': 'range-bearing',
                 'vars_id': ['l1', 'x0'],
                 },
                {'factor_id': 'f2',
                 'type': 'range-bearing',
                 'vars_id': ['l3', 'x0'],
                 },
                {'factor_id': 'f3',
                 'type': 'odometry',
                 'vars_id': ['x0', 'x1'],
                 },
                {'factor_id': 'f4',
                 'type': 'range-bearing',
                 'vars_id': ['x1', 'l3'],
                 },
                {'factor_id': 'f5',
                 'type': 'odometry',
                 'vars_id': ['x1', 'x2'],
                 },
            ],
        }
    },
    # the third robot
    {
        'header': {
            'robot_id': 'r3',
            'seq': 0,
        },
        'rt_state': {'type': 'relative',  # absolute or relative to last state (last pose mst exist)
                     "state": {"x": 71.3, "y": 30.5, "th": 2.9},
                     'covariance': {'sigma': [2, 0.9], 'rot': 0.314}},
        'last_pose': {'last_pose_id': 'x2'},
        'last_pose': {'last_pose_id': 'x3'},
        'graph': {
            'marginals': [
                {'var_id': 'x0', 'mean': {'x': 90.8, 'y': 42.2},
                 'covariance': {'sigma': [rSigMul(1/2), rSigMul(1/2)], 'rot':rRot()}},
                {'var_id': 'x1', 'mean': {'x': 90.3, 'y': 33.8},
                 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'x2', 'mean': {'x': 85.5, 'y': 29.2},
                 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'x3', 'mean': {'x': 76.5, 'y': 29.1},
                 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'l11', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l11')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l11')[
                            'state']['y']+rXY()
                }, 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'l14', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l14')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l14')[
                            'state']['y']+rXY()
                }, 'covariance': {'sigma': [1.2/2, 1.2/2], 'rot':0}},
                {'var_id': 'l15', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l15')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l15')[
                            'state']['y']+rXY()
                }, 'covariance': {'sigma': [1.2/2, 1.2/2], 'rot':0}},
            ],

            'factors': [
                {'factor_id': 'f0',
                 'type': 'ini_position',
                 'vars_id': ['x0'],
                 },
                {'factor_id': 'f1',
                 'type': 'odometry',
                 'vars_id': ['x1', 'x0'],
                 },
                {'factor_id': 'f2',
                 'type': 'odometry',
                 'vars_id': ['x2', 'x1'],
                 },
                {'factor_id': 'f3',
                 'type': 'odometry',
                 'vars_id': ['x3', 'x2'],
                 },
                {'factor_id': 'f4',
                 'type': 'range-bearing',
                 'vars_id': ['x0', 'l15'],
                 },
                {'factor_id': 'f5',
                 'type': 'range-bearing',
                 'vars_id': ['x1', 'l15'],
                 },
                {'factor_id': 'f6',
                 'type': 'range-bearing',
                 'vars_id': ['x2', 'l14'],
                 },
                # {'factor_id': 'f7',
                #  'type': 'range-bearing',
                #  'vars_id': ['x2', 'l11'],
                #  },
                {'factor_id': 'f8',
                 'type': 'range-bearing',
                 'vars_id': ['x3', 'l11'],
                 },
                {'factor_id': 'f9',
                 'type': 'range-bearing',
                 'vars_id': ['x2', 'l15'],
                 },
            ],
        }
    }
]
```

