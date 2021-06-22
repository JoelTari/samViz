/*
 * Copyright 2021 AKKA Technologies (joel.tari-summerfield@akka.eu)
 *
 * Licensed under the EUPL, Version 1.2 or – as soon they will be approved by the European Commission - subsequent versions of the EUPL (the "Licence");
 * You may not use this work except in compliance with the Licence.
 * You may obtain a copy of the Licence at:
 *
 * https://joinup.ec.europa.eu/software/page/eupl
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the Licence is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the Licence for the specific language governing permissions and limitations under the Licence.
 */

// DOM related
const elBody = d3.select("body");
const elCanvas = d3.select("svg.canvas");
// Define the div for the tooltip
const elDivTooltip = d3
  .select("body")
  .append("div")
  .classed("tooltip", true)
  .style("opacity", 0);

const aratio = 0.6;

// initially no robot is selected
GlobalUI = {
  selected_robot_id: "",
  base_unit_graph: 1, // unit that controls the dimensions of the graph
  // (set when receiving a graph and getting a medium distance between the nodes)
  // this is the default setting, it can instead be set by the message header of graph
};
/******************************************************************************
 *                           SVG Group binding to d3
 *****************************************************************************/
const elMainGroup = d3.select(".main_group");
const elAgents = elMainGroup.append("g").classed("agents", true);
const elLandmarks = elMainGroup.append("g").classed("landmarks", true);
// dynamic
let elsLandmark = elLandmarks.selectAll(".landmark");

/******************************************************************************
 *                            D3 ZOOM
 *****************************************************************************/
// scat_data = {
//   const randomX : d3.randomNormal(100 / 2, 80);
//   const randomY : d3.randomNormal(60 / 2, 80);
//   return Array.from({length: 2000}, () => [randomX(), randomY()]);
// }

// const scat_data = gen_scat_data()

// const gscat = elCanvas.append('gscat')

// gscat.selectAll("circle")
//     .data(data)
//     .join("circle")
//       .attr("cx", ([x]) => x)
//       .attr("cy", ([, y]) => y)
//       .attr("r", 1.5);

// const zoom = d3.zoom()
//       .scaleExtent([1, 40])
//       .on("zoom", zoomed);

elCanvas.call(
  d3
    .zoom()
    .extent([
      [-300, -180],
      [300, 180],
    ])
    .scaleExtent([0.1, 40])
    .on("zoom", zoomed)
);

function zoomed({ transform }) {
  elMainGroup.attr("transform", transform);
}

/******************************************************************************
 *                            MQTT events
 *****************************************************************************/

let client = mqtt.connect(`ws://${location.host}:9001`);

client.on("connect", function () {
  console.log("[mqtt] Connected to broker !");

  client.subscribe("ground_truth", function (err) {
    if (!err) {
      console.log("[mqtt] subscribed to the topic >> ground_truth");
    }
  });

  client.subscribe("estimation", function (err) {
    if (!err) {
      console.log("[mqtt] subscribed to the topic >> estimation_graph");
    }
  });
});

// this event receive incoming mqtt topics (for which there is a subscription) and acts accordingly
client.on("message", function (topic, message) {
  if (topic == "meta_info"){
    // TODO: create stuff according to meta
  }
  else if (topic == "ground_truth") {
    // This topic should be called in fact metaInfo or something when no ground truth is available
    // create the AgentTeam instanciate the robot objects
    // create a GeUpPa for the landmarks
    const msg = JSON.parse(message.toString());
    // draw landmarks first, I dont do a group for each individual
    // landmark  (no updates as they are supposed fixed)
    if (msg.landmarks != null) {
      elsLandmark = elsLandmark
        .data(msg.landmarks, (d) => d.landmark_id)
        .join(
          (enter) =>
            enter
              .append("path")
              // size is the area, for a cross: area= desired_tot_length**2 *5
              .attr("d", `${d3.symbol(d3.symbolCross, 1.1 * 1.1)()}`)
              .classed("landmark", true)
              .attr("id", (d) => d.landmark_id)
              .attr("transform", (d) => `translate(${d.state.x},${d.state.y})`)
              .on("mouseover", function (e, d) {
                // TODO: use the UI event function
                //       like others mouseover func do
                // note: use d3.pointer(e) to get pointer coord wrt the target element
                elDivTooltip
                  .style("left", `${e.pageX}px`)
                  .style("top", `${e.pageY - 6}px`)
                  .style("visibility", "visible")
                  // .transition()
                  // .duration(200)
                  .style("opacity", 0.9);
                elDivTooltip.html(`${d.landmark_id}`);
                d3.select(this).style("cursor", "pointer");
              })
              .on("mouseout", mouseout_mg())
          // ,(update) => update // the default if not specified
        );
    }

    for (const [robot_id, robot_ground_truth] of Object.entries(msg.robots)) {
      // Merge UI and ground_truth
      robot_ground_truth.isSelected = robot_id === GlobalUI.selected_robot_id;
      AgentTeam[robot_id] = new AgentViz(robot_ground_truth, client, elAgents);
      // activate mqtt subscriptions
      AgentTeam[robot_id].subscribeTopicsToMqtt();
    }
  } else if (topic.split("/").length == 2) {
    const msg = JSON.parse(message.toString());
    const [agent_id, topic_suffix] = topic.split("/");
    AgentTeam.checkSubscriptions(agent_id, topic_suffix, msg);
  }
});


/******************************************************************************
 *                            Class AgentViz
 *****************************************************************************/
const AgentTeam = {
  checkSubscriptions: function (agent_id, topic_suffix, msg) {
    this[agent_id].mqttProcessTopicSuffix(topic_suffix, msg);
  },
};
// create a base default agent in any case
AgentTeam["default"] = new BaseAgentViz("default", client, elAgents);

class BaseAgentViz {
  constructor(id, mqttc, parent_container) {
    this.id = id;
    this.d3parent_container = parent_container; // d3 selection this visual is a child of. parent_container.children are the other agents !
    this.d3container = parent_container
      .append("g")
      .classed("agent", true)
      .attr("id", this.id); // d3 selection inside which this agent will append stuff
    this.mqttc = mqttc;
    // Object sub_topics keys: topic name, value: callback function
    this.sub_topics = {};
    this.pub_topics = [];
    // add 'graphs' as a subscribed topic
    this.sub_topics["graphs"] = this.graphsCallback;
    // d3: create the factor graph structure (only 1 FG per robot for now)
    this.d3FactorGraph = constructD3FactorGraph(this.d3container, this.id);
  }

  subscribeTopicsToMqtt = function () {
    // do the subscriptions, for each topic in the sub_topics array member
    Object.keys(this.sub_topics).forEach((t) => {
      this.mqttc.subscribe(`${this.id}/${t}`, (err) => {
        if (!err)
          console.log(`[mqtt] subscribed to the topic >> ${this.id}/${t}`);
      });
    });
  };

  // entry point to get the correct callback depending on the topic name
  // throw error if topic name is not in the list
  mqttProcessTopicSuffix = function (suffix_topic_name, data) {
    this.sub_topics[suffix_topic_name](data);
  };

  graphsCallback = function (graph) {
    console.log("Receive some graph return :" + this.id);
    console.log(graph);

    // TEMPORARY (TODO)
    // apply base unit graph if specified
    if (graph.header.base_unit != null)
      GlobalUI.base_unit_graph = graph.header.base_unit;

    // massage data
    estimation_data_massage(graph);

    // general update pattern
    this.d3FactorGraph
      .select("g.factors_group")
      .selectAll(".factor")
      .data(graph.factors, (d) => d.factor_id)
      .join(join_enter_factor, join_update_factor, join_exit_factor);

    this.d3FactorGraph
      .select("g.vertices_group")
      .selectAll(".vertex")
      .data(graph.marginals, (d) => d.var_id)
      .join(join_enter_vertex, join_update_vertex); // TODO: exit vertex
  };
}

class fullAgentViz extends BaseAgentViz {
  constructor(robot_data, mqttc, parent_container) {
    // base class ctor
    super(robot_data.id, mqttc, parent_container);

    this.sensor_svg_path = this.sensorVisual(robot_data.sensor);
    this.history_odom = []; // successive poses of the odometry (from the last graph pose): emptied when a new pose is created on the graph
    this.history_graph = []; // succesives poses of the graph (x0 to x{last_pose})
    this.history_true = []; // successives true poses
    // this.svg_history_truth = ""; // str version of the history truth
    this.max_history_elements = 100; // TODO: apply (maybe not the same size-reducing rules for the 3)...
    //      ex: history_true: there could be a min eucl. distance btw elements to reduce size
    //          history_odom: remove every other element each time the threshold is reached
    this.registerGroundTruthData(robot_data.state);
    // d3 : create the truth structure
    this.d3Truth = constructD3Truth(
      this.d3container,
      this.d3parent_container,
      robot_data,
      this.sensor_svg_path,
      this.state_history
    );
    // topic names (INs)
    this.sub_topics["odom"] = this.odomCallback;
    this.sub_topics["measures_feedback"] = this.measuresCallback;
    this.sub_topics["ground_truth"] = this.groundTruthCallback;

    // topic names (OUTs)
    this.topic_request_ground_truth = `${this.id}/request_ground_truth`;
    this.topic_cmd = `${this.id}/cmd`;
    this.topic_request_graph = "request_graphs";
    // d3 : create the odom structure
    this.d3Odom = constructD3Odom(this.d3container, this.id);
    // d3: create the measure visualization structure
    this.d3MeasuresViz = constructD3MeasuresViz(this.d3container, this.id);
  }
  // update Visual truth
  updateVisualTruth = function (state_history, state) {
    // translate
    this.d3Truth
      .select(".gtranslate")
      .attr("transform", () => `translate(${state.x},${state.y})`);
    // rotate (TODO: transition? CSS probably)
    this.d3Truth
      .select(".grotate")
      .attr("transform", `rotate(${(state.th * 180) / Math.PI})`);
    // history
    if (
      this.d3Truth.select("polyline.state_history").empty() &&
      state_history.length >= 2
    ) {
      // the element polyline doesnt exist -> create it
      // AND there is at least 2 elements
      this.d3Truth
        .append("polyline")
        .classed("state_history", true)
        .attr("points", state_history.map((e) => `${e.x},${e.y}`).join(" "));
    } else if (state_history.length >= 2) {
      //update
      this.d3Truth.select("polyline.state_history").attr(
        "points",
        state_history.map((e) => `${e.x},${e.y}`).join(" ") //+ ` ${state.x},${state.y}`
      );
    }
  };
  // update visual odom
  updateVisualOdom = function (odom_history, state, visual_covariance = null) {
    // odom becomes visible
    this.d3Odom.style("visibility", null);

    this.d3Odom
      .select(".gtranslate")
      .attr("transform", `translate(${state.x},${state.y})`)
      .call(function (g_tra) {
        if (visual_covariance !== null) {
          // covariance could be 0 (impossible to draw, therefore not transmited in the data)
          g_tra
            .select("ellipse.odom_covariance")
            .attr("rx", visual_covariance.sigma[0] * Math.sqrt(9.21))
            .attr("ry", visual_covariance.sigma[1] * Math.sqrt(9.21))
            .attr(
              "transform",
              `rotate(${(visual_covariance.rot * 180) / Math.PI})`
            );
        }
        g_tra
          .select(".grotate")
          .attr("transform", `rotate(${(state.th * 180) / Math.PI})`);
      });

    if (odom_history.length >= 2) {
      this.d3Odom
        .select("polyline.odom_history")
        .attr("points", odom_history.map((e) => `${e.x},${e.y}`).join(" "));
    }
  };
  transcientMeasureVisual = function (state, measure_set) {
    const measures_data = measure_set.map((e) => {
      if (e.type === "range-AA") {
        return { x: e.vect[0], y: e.vect[1], type: e.type };
      } else if (e.type === "range-bearing") {
        return { r: e.vect[0], a: e.vect[1], type: e.type };
      } else console.error("Unsupported measure type");
    });
    const time_illumination = 200; // TODO: Global UI

    this.d3MeasuresViz
      .selectAll("line")
      .data(measures_data)
      .join("line")
      .classed("measure_ray", true)
      .each(function (d) {
        if (d.type === "range-AA") {
          d3.select(this)
            .attr("x1", state.x)
            .attr("y1", state.y)
            .attr("x2", (d) => d.x + state.x)
            .attr("y2", (d) => d.y + state.y);
        } else if (d.type === "range-bearing") {
          const x2r = d.r * Math.cos(d.a);
          const y2r = d.r * Math.sin(d.a);
          d3.select(this)
            .attr("x1", state.x)
            .attr("y1", state.y)
            .attr("x2", (d) => state.x + d.r * Math.cos(state.th + d.a))
            .attr("y2", (d) => state.y + d.r * Math.sin(state.th + d.a));
          // .attr('x2',d => state.x + x2r*Math.cos(state.th) + y2r*Math.sin(state.th))
          // .attr('y2',d => state.y - x2r*Math.sin(state.th) + y2r*Math.cos(state.th))
        } else {
          // TODO: bearing
          console.error("Unsupported measure type");
        }
      })
      .style("opacity", 0)
      .style("stroke-width", 0.05)
      .transition("reveal_mes")
      .duration(time_illumination)
      .style("opacity", null)
      .style("stroke-width", null)
      .transition("remove_mes")
      .duration(time_illumination)
      .style("opacity", 0)
      .style("stroke-width", 0.05)
      .remove();

    // class the landmark as illumitated
    elsLandmark
      .filter((dl) =>
        measure_set.map((e) => e.landmark_id).includes(dl.landmark_id)
      )
      .transition("reveal_illum")
      .duration(time_illumination)
      .style("fill", "darksalmon")
      .transition("remove_illum")
      .duration(time_illumination)
      .style("fill", null);
  };

  // add ground_truth data
  registerGroundTruthData = function (state) {
    this.current_true_state = state;
    this.history_true = history2d_push(
      state,
      3,
      this.history_true,
      this.max_history_elements
    );
  };

  registerOdomData = function (data) {
    this.current_odom_state = data.state;
    this.current_odom_cov = data.visual_covariance;
    this.history_odom = history2d_push(
      data.state,
      2,
      this.history_odom,
      this.max_history_elements
    );
  };

  // function that given the data angle cover and range, draws the string data for the
  // svg-path element
  sensorVisual = function (sensor) {
    const rx = sensor.range;
    const ry = rx;
    if (sensor.angle_coverage > 1 || sensor.angle_coverage < 0)
      console.error("Sensor angle_coverage out of bound");
    // def px py as the starting point along the sensor range arc
    const px = Math.cos(Math.PI * sensor.angle_coverage) * sensor.range;
    const py = Math.sin(Math.PI * sensor.angle_coverage) * sensor.range;
    const sweep = true * (sensor.angle_coverage > 0.5);
    return `M0,0 l${px},${py} A ${rx} ${ry} 0 ${sweep} 0 ${px} ${-py}`;
  };

  // define the callbacks
  odomCallback = function (data) {
    console.log("Receive some odom response " + this.id + "with data: ");
    console.log(data);
    this.registerOdomData(data);
    this.updateVisualOdom(
      this.history_odom,
      data.state,
      data.visual_covariance
    );
  };
  graphsCallback = function (graph) {
    console.log("Receive some graph return :" + this.id);
    console.log(graph);

    // TEMPORARY (TODO)
    // apply base unit graph if specified
    if (graph.header.base_unit != null)
      GlobalUI.base_unit_graph = graph.header.base_unit;

    // massage data
    estimation_data_massage(graph);

    // general update pattern
    this.d3FactorGraph
      .select("g.factors_group")
      .selectAll(".factor")
      .data(graph.factors, (d) => d.factor_id)
      .join(join_enter_factor, join_update_factor, join_exit_factor);

    this.d3FactorGraph
      .select("g.vertices_group")
      .selectAll(".vertex")
      .data(graph.marginals, (d) => d.var_id)
      .join(join_enter_vertex, join_update_vertex); // TODO: exit vertex
  };
  measuresCallback = function (data) {
    console.log("Receive some measure :" + this.id);
    this.transcientMeasureVisual(this.current_true_state, data.measures);
  };
  groundTruthCallback = function (data) {
    // console.log("Receive some GT info :" + this.id);
    this.registerGroundTruthData(data.state);
    this.updateVisualTruth(this.history_true, data.state);
  };
}

/******************************************************************************
 *                            UI Events
 *****************************************************************************/

// mouse over-out
function mouseover_mg(text_str) {
  return function (e, d) {
    elDivTooltip
      .style("left", `${e.pageX}px`)
      .style("top", `${e.pageY - 6}px`)
      .style("visibility", "visible")
      // .transition()
      // .duration(200)
      .style("opacity", 0.9);
    elDivTooltip.html(text_str);
    d3.select(this).style("cursor", "pointer");
  };
}
function mouseout_mg() {
  return function (e, d) {
    d3.select(this).style("cursor", "default");
    elDivTooltip
      // .transition()
      // .duration(300)
      .style("opacity", 0)
      .style("visibility", "hidden");
  };
}

// TODO: put this var in globalUI
let keyPressedBuffer = {
  ArrowUp: false,
  ArrowDown: false,
  ArrowRight: false,
  ArrowLeft: false,
};

elBody.on("keydown", (e) => {
  // discard unmanaged keys
  if (
    e.key != "ArrowUp" &&
    e.key != "ArrowDown" &&
    e.key != "ArrowRight" &&
    e.key != "ArrowLeft"
  ) {
    return;
  }

  if (!keyPressedBuffer[e.key]) keyPressedBuffer[e.key] = true;

  inputCmdModel = "DD"; // TODO: centralize in globalUI
  const cmdObj = inputToMove(inputCmdModel);

  client.publish(
    `${GlobalUI.selected_robot_id}/cmd`,
    JSON.stringify({
      robot_id: GlobalUI.selected_robot_id,
      type: inputCmdModel,
      cmd_vel: cmdObj,
    })
  );
});

elCanvas.on("click", (e) => console.log(d3.pointer(e, elMainGroup.node())));

elBody.on("keyup", (e) => (keyPressedBuffer[e.key] = false));

function getTransform_gg(d3_single_selected) {
  // only works on double group descendant framework
  // (first descendant is translation, second is rotation)
  curx = d3_single_selected.selectChild("g").node().transform.baseVal[0].matrix
    .e;
  cury = d3_single_selected.selectChild("g").node().transform.baseVal[0].matrix
    .f;
  curth = d3_single_selected.selectChild("g").selectChild("g").node().transform
    .baseVal[0].angle;
  return [curx, cury, curth];
}

function applyRelativeMove_gg(d3_single_selected, dmove) {
  const [curx, cury, curth] = getTransform_gg(d3_single_selected);
  const [dX, dY, dth] = dmove;
  applyMove_gg(d3_single_selected, [curx + dX, cury + dY, curth + dth]);
}

function applyMove_gg(d3_single_selected, pose) {
  [x, y, th] = pose;
  // I left the transitions related lines commented for posterity
  d3_single_selected
    .selectChild("g")
    .transition("tra")
    .duration(60)
    .attr("transform", "translate(" + x + "," + y + ")")
    .selection()
    .selectChild("g")
    .transition("rot")
    .duration(60)
    .attr("transform", `rotate(${(th * 180) / Math.PI})`);
}

/******************************************************************************
 *                           KeyPresses Helper
 *****************************************************************************/

function history2d_push(
  state,
  distance_treshold,
  current_history,
  max_history_size
) {
  if (current_history.length < 2) {
    current_history.push(state);
  } else {
    // const last_truth_pose = current_history[current_history.length - 1];
    const ante_truth_pose = current_history[current_history.length - 2];
    // do we replace last truth pose or do we push a new elem ?
    if (
      (state.x - ante_truth_pose.x) ** 2 + (state.y - ante_truth_pose.y) ** 2 >
      distance_treshold ** 2
    ) {
      // current_history.push(state);
      current_history.push(state);
      // protecting against array overflow (wrt the defined max size)
      if (current_history.length > max_history_size) {
        current_history.shift();
      }
    } else {
      current_history[current_history.length - 1] = state;
    }
  }
  return current_history;
}

function inputToMove(model) {
  // get key or combination from global buffer
  up = keyPressedBuffer["ArrowUp"];
  down = keyPressedBuffer["ArrowDown"];
  left = keyPressedBuffer["ArrowLeft"];
  right = keyPressedBuffer["ArrowRight"];

  // console.log("Moving using model : " + model); // TODO globalUI
  const speed = 0.5; // TODO: globalUI

  if (model === "AA") {
    dx = 0;
    dy = 0;

    if ((up && down) || (right && left)) {
      // if contradictory order(s)
      // nothing to do
    } else {
      dy -= speed * right;
      dy += speed * left;
      dx += speed * up;
      dx -= speed * down;
    }
    return { x: dx, y: dy };
  } else if (model === "DD") {
    dlinear = 0;
    dangular = 0;
    if ((up && down) || (right && left)) {
      // if contradictory order(s)
      // nothing to do
    } else {
      // TODO: decouple speeds
      dangular -= (speed / 10.0) * right; // rads
      dangular += (speed / 10.0) * left;
      dlinear += speed * up;
      dlinear -= speed * down;
    }
    return { linear: dlinear, angular: dangular };
  } else {
    console.error("Unknown model for sending cmd", model);
  }
}

/******************************************************************************
 *                            HELPER
 *****************************************************************************/
function arraytised(obj_or_array) {
  if (obj_or_array[Symbol.iterator] == null) {
    console.warn("received data is not an array: attempting to arraytised");
    return [obj_or_array];
  }
  // if already an array, all is gud
  else return obj_or_array;
}

function ecpi(a) {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

function indexOfMax(arr) {
  if (arr.length === 0) {
    return -1;
  }

  var max = arr[0];
  var maxIndex = 0;

  for (var i = 1; i < arr.length; i++) {
    if (arr[i] > max) {
      maxIndex = i;
      max = arr[i];
    }
  }

  return maxIndex;
}

function getRandomInt(max) {
  return Math.floor(Math.random() * Math.floor(max));
}

/******************************************************************************
 *                            HELPER Visual
 *****************************************************************************/
constructD3Truth = function (
  d3container,
  d3parent_container,
  robot_data,
  sensor_svg_path,
  state_history
) {
  // d3 truth
  return d3container
    .append("g")
    .classed("selected", robot_data.isSelected)
    .classed("agent__truth", true)
    .attr("id", robot_data.robot_id)
    .call(function (g_truth) {
      // call rather ??
      g_truth
        .append("g")
        .attr(
          "transform",
          () => `translate(${robot_data.state.x},${robot_data.state.y})`
        )
        .classed("gtranslate", true)
        .append("g")
        .classed("grotate", true)
        .attr("transform", `rotate(${(robot_data.state.th * 180) / Math.PI})`)
        .call(function (g) {
          // adding all display components
          // 1. the sensor
          if (robot_data.sensor != null)
            g.append("path").classed("sensor", true).attr("d", sensor_svg_path);
          // 2. the robot
          g.append("polygon")
            .attr("points", "0,-1 0,1 3,0") // TODO: append a <g> first
            .on("mouseover", mouseover_mg(`${robot_data.robot_id}`))
            .on("mouseout", mouseout_mg());
          g.append("line")
            .attr("x1", 0)
            .attr("y1", 0)
            .attr("x2", 1)
            .attr("y2", 0);
        });
      // the state history   TODO: necessary ??? Since I create it in update
      if (state_history >= 2) {
        g_truth
          .append("polyline")
          .classed("state_history", true)
          // .attr("id", (d) => d.seq)
          // the points of the polyline are the history + the rt state
          .attr(
            "points",
            state_history.map((e) => `${e.x},${e.y}`).join(" ") //+ ` ${state.x},${state.y}`
          );
      }
    })
    .transition()
    .duration(500)
    .style("opacity", 1)
    .selection()
    .on("click", function () {
      // for next joining of data
      GlobalUI.selected_robot_id = d3.select(this).attr("id");
      // for current data session: put others to false, and the selected to true
      d3parent_container.selectAll(".agent__truth").classed("selected", false);
      d3.select(this).classed("selected", true);
    });
};

constructD3Odom = function (d3container, robot_id) {
  return d3container
    .append("g")
    .classed("agent__odom", true)
    .style("visibility", "hidden")
    .attr("id", robot_id)
    .call(function (g_odom) {
      g_odom
        .append("g")
        .classed("gtranslate", true)
        .call(function (g_tra) {
          g_tra
            .append("g")
            .classed("grotate", true)
            .classed("ghost", true)
            .call(function (g_rot) {
              g_rot
                .append("polygon")
                .attr("points", "0,-1 0,1 3,0")
                .style("stroke-opacity", 0)
                .transition()
                .duration(400)
                .style("stroke-opacity", null);
              g_rot
                .append("line")
                .attr("x1", 0)
                .attr("y1", 0)
                .attr("x2", 1)
                .attr("y2", 0)
                .style("stroke-opacity", 0)
                .transition()
                .duration(400)
                .style("stroke-opacity", null);
            });
          g_tra.append("ellipse").classed("odom_covariance", true);
        });
      g_odom.append("polyline").classed("odom_history", true);
    });
};

constructD3MeasuresViz = function (d3container, robot_id) {
  return d3container
    .append("g")
    .classed("agent__measuresViz", true)
    .attr("id", robot_id);
};

constructD3FactorGraph = function (d3container, robot_id) {
  return d3container
    .append("g")
    .classed("factor_graph", true)
    .attr("id", robot_id)
    .call(function (g_factor_graph) {
      g_factor_graph.append("g").classed("factors_group", true);
      g_factor_graph.append("g").classed("vertices_group", true);
    });
};


/******************************************************************************
 *                          UPDATE PATTERN ROUTINES
 *                  enter,update,exit of the various d3 selections
 *****************************************************************************/

function join_enter_robot_estimates(enter) {
  return (
    enter
      .append("g")
      .classed("robot_estimates", true)
      .attr("id", (d) => d.header.robot_id)
      // prepare underlayers the fg group and the rt_estimate group
      .each(function (d, _, _) {
        // for each robot
        // prepare fg group
        d3.select(this)
          .append("g")
          .classed("factor_graph", true)
          // prepare underlayers: factors_group and vertices_group
          .each(function (_, _, _) {
            // for each graph
            d3.select(this).append("g").classed("factors_group", true);
            d3.select(this).append("g").classed("vertices_group", true);
          });
        // prepare and fill in rt_estimate_group
        d3.select(this)
          .append("g")
          .classed("rt_estimate", true)
          .attr("id", `${d.header.robot_id}`)
          .call(function (g_rt_estimate) {
            const t_ghost_entry = d3.transition("ghost_entry").duration(400);
            const t_ghost_entry2 = d3
              .transition("ghost_entry_last_pose_line")
              .duration(400);
            // the ghost
            g_rt_estimate
              .append("g")
              .classed("rt_ghost", true)
              .on("mouseover", mouseover_mg(`${d.header.robot_id}`))
              .on("mouseout", mouseout_mg())
              // .attr('id',`${d.header.robot_id}`)
              .append("g")
              .attr(
                "transform",
                `translate(${d.rt_estimate.state.x}, ${d.rt_estimate.state.y} )`
              )
              // the rt ellipse
              .call(function (g_rt_tra) {
                g_rt_tra
                  .append("ellipse")
                  .classed("rt_covariance_odom", true)
                  .attr(
                    "rx",
                    d.rt_estimate.covariance.sigma[0] * Math.sqrt(9.21)
                  )
                  .attr(
                    "ry",
                    d.rt_estimate.covariance.sigma[1] * Math.sqrt(9.21)
                  )
                  .attr(
                    "transform",
                    `rotate(${(d.rt_estimate.covariance.rot * 180) / Math.PI})`
                  )
                  .style("opacity", 0) // wow! (see next wow) Nota: doesnt  work with attr()
                  .transition(t_ghost_entry)
                  .style("opacity", null); // wow! this will look for the CSS (has to a style)
              })
              .append("g")
              .attr(
                "transform",
                (local_d) =>
                  "rotate(" +
                  (local_d.rt_estimate.state.th * 180) / Math.PI +
                  ")"
              )
              .call(function (g_rt_ghost) {
                // 1. the robot
                g_rt_ghost
                  .append("polygon")
                  .attr("points", "0,-1 0,1 3,0")
                  .style("stroke-opacity", 0)
                  .transition(t_ghost_entry)
                  .style("stroke-opacity", null);

                g_rt_ghost
                  .append("line")
                  .attr("x1", 0)
                  .attr("y1", 0)
                  .attr("x2", 1)
                  .attr("y2", 0)
                  .style("stroke-opacity", 0)
                  .transition(t_ghost_entry)
                  .style("stroke-opacity", null);
              });
            // the line to the last pose
            g_rt_estimate
              .append("line")
              .classed("rt_line_to_last_pose", true)
              // .attr('id',`${d.header.robot_id}`)
              .attr("x1", d.rt_estimate.state.x)
              .attr("y1", d.rt_estimate.state.y)
              .attr("x2", d.last_pose.state.x)
              .attr("y2", d.last_pose.state.y)
              .style("stroke-opacity", 0)
              .transition(t_ghost_entry2)
              .style("stroke-opacity", null);
          });
      })
  );
}

function join_update_robot_estimates(update) {
  const t_graph_motion = d3
    .transition("m1")
    .duration(1000)
    .ease(d3.easeCubicInOut);
  const t_graph_motion2 = d3
    .transition("m2")
    .duration(1000)
    .ease(d3.easeCubicInOut);

  return update.each(function (d, _, _) {
    d3.select(this)
      .select("g.rt_estimate")
      .call(function (g_rt_estimate) {
        // update the rt ghost dasharray-ed drawing
        g_rt_estimate
          .select("g.rt_ghost")
          .select("g")
          .transition(t_graph_motion)
          .attr(
            "transform",
            `translate(${d.rt_estimate.state.x}, ${d.rt_estimate.state.y} )`
          )
          .selection()
          .call(function (gtranslate) {
            gtranslate
              .select("ellipse")
              .transition(t_graph_motion2)
              .attr("rx", d.rt_estimate.covariance.sigma[0] * Math.sqrt(9.21))
              .attr("ry", d.rt_estimate.covariance.sigma[1] * Math.sqrt(9.21))
              .attr(
                "transform",
                `rotate(${(d.rt_estimate.covariance.rot * 180) / Math.PI})`
              );
            gtranslate
              .select("g")
              .transition(t_graph_motion2)
              .attr(
                "transform",
                (local_d) =>
                  "rotate(" +
                  (local_d.rt_estimate.state.th * 180) / Math.PI +
                  ")"
              );
          });
        // .selection();
        // update the line to the last pose
        g_rt_estimate
          .select("line.rt_line_to_last_pose")
          .transition(t_graph_motion)
          .attr("x1", (d) => d.rt_estimate.state.x)
          .attr("y1", (d) => d.rt_estimate.state.y)
          .attr("x2", (d) => d.last_pose.state.x)
          .attr("y2", (d) => d.last_pose.state.y);
      });
  });
}

function join_enter_factor(enter) {
  // TODO:
  // Imho best way to avoid to define those transitions everywhere is to
  // transform those functions in classes of which the transitions are members
  const t_factor_entry = d3.transition().duration(400);
  const t_graph_motion = d3.transition().duration(1000).ease(d3.easeCubicInOut);

  return enter
    .append("g")
    .classed("factor", true)
    .attr("id", (d) => d.factor_id)
    .each(function (d) {
      d3.select(this)
        .append("g")
        .attr("transform", "translate(0,0)")
        .append("g")
        .attr("transform", "rotate(0)")
        .style("opacity", 0)
        .transition(t_factor_entry) // ugly (im interest in the child opacity not this node) but necessary to run concurrent transitions on the line (which doesnt work if I place it below)
        .style("opacity", null)
        .selection()
        .call(function (g) {
          if (d.vars.length > 1) {
            // bi-factor, tri-factor etc...
            d.vars.forEach((v) =>
              g
                .append("line")
                .attr("stroke-width", 0.2 * GlobalUI.base_unit_graph)
                .attr("x1", d.dot_factor_position.x)
                .attr("y1", d.dot_factor_position.y)
                .attr("x2", 0.2 * v.mean.x + 0.8 * d.dot_factor_position.x)
                .attr("y2", 0.2 * v.mean.y + 0.8 * d.dot_factor_position.y)
                .classed(v.var_id, true)
                .transition(t_graph_motion)
                .attr("x1", d.dot_factor_position.x)
                .attr("y1", d.dot_factor_position.y)
                .attr("x2", v.mean.x)
                .attr("y2", v.mean.y)
            );
          } else {
            // unifactor
            g.append("line")
              .attr("stroke-width", 0.2 * GlobalUI.base_unit_graph)
              .attr("x1", d.dot_factor_position.x)
              .attr("y1", d.dot_factor_position.y)
              .attr("x2", d.dot_factor_position.x)
              .attr("y2", d.dot_factor_position.y)
              .transition(t_graph_motion)
              .attr("x1", d.vars[0].mean.x)
              .attr("y1", d.vars[0].mean.y)
              .attr("x2", d.dot_factor_position.x)
              .attr("y2", d.dot_factor_position.y);
          }

          g.append("circle")
            .attr(
              "cx",
              (d) => d.dot_factor_position.x
              // (d) => (d.vars[0].mean.x + d.vars[1].mean.x) / 2
            )
            .attr(
              "cy",
              (d) => d.dot_factor_position.y
              // (d) => (d.vars[0].mean.y + d.vars[1].mean.y) / 2
            )
            // .style("opacity", 0)
            .attr("r", 0.3 * 2 * GlobalUI.base_unit_graph)
            .on("mouseover", mouseover_mg(`${d.factor_id}`))
            .on("mouseout", mouseout_mg())
            // opacity transition not necessary here
            .transition("fc")
            .duration(2200)
            // .style("opacity")
            .attr("r", 0.3 * GlobalUI.base_unit_graph);
        });
    });
}

function join_update_factor(update) {
  // TODO:
  // Imho best way to avoid to define those transitions everywhere is to
  // transform those functions in classes of which the transitions are members
  const t_graph_motion = d3.transition().duration(1000).ease(d3.easeCubicInOut);

  return update.each(function (d) {
    d3.select(this)
      .select("g")
      .select("g")
      .selectAll("line")
      // .selectChildren("line")
      // .selectChild("line") // TODO: all children
      // .call(function (lines) {
      // lines.each(function (dd, i) {
      .each(function (dd, i) {
        if (d.vars.length > 1) {
          // line
          d3.select(this)
            .transition(t_graph_motion)
            .attr("x1", d.dot_factor_position.x)
            .attr("y1", d.dot_factor_position.y)
            .attr("x2", d.vars[i].mean.x)
            .attr("y2", d.vars[i].mean.y);
        } else {
          // update unary factor
          // WARN TODO: a factor_id should not change its vars_id
          d3.select(this)
            .transition(t_graph_motion)
            .attr("x1", d.vars[0].mean.x)
            .attr("y1", d.vars[0].mean.y)
            .attr("x2", d.dot_factor_position.x)
            .attr("y2", d.dot_factor_position.y);
        }
        // });
      });
    // the little factor circle (to visually differentiate from with MRF)
    d3.select(this)
      .selectChild("g")
      .select("circle")
      .transition(t_graph_motion)
      .attr("cx", d.dot_factor_position.x)
      .attr("cy", d.dot_factor_position.y);
  });
}

function join_exit_factor(exit) {
  return (
    exit
      // .call((ex) =>
      //   ex
      .select("g")
      .call(function (ex) {
        ex.select("g").selectAll("line").style("stroke", "brown");
        ex.select("circle").style("fill", "brown");
      })
      // .selectAll("line")
      // .style("stroke", "brown")
      // )
      // .call((ex) => ex.selectChild("g").select("circle").style("fill", "brown"))
      .transition("exit_factor") // TODO: Define outside
      .duration(1000)
      .style("opacity", 0)
      .remove()
  );
}

function join_enter_vertex(enter) {
  // TODO:
  // Imho best way to avoid to define those transitions everywhere is to
  // transform those functions in classes of which the transitions are members
  const t_vertex_entry = d3.transition().duration(400);

  return (
    enter
      .append("g")
      .classed("vertex", true)
      .attr("id", (d) => d.var_id)
      .each(function (d) {
        d3.select(this)
          .append("g")
          .attr("transform", "translate(" + d.mean.x + "," + d.mean.y + ")")
          .append("g")
          .attr("transform", "rotate(0)")
          .call(function (g) {
            g.append("circle")
              .attr("r", GlobalUI.base_unit_graph * 3)
              .style("opacity", 0)
              .attr("stroke-width", 0.28 * GlobalUI.base_unit_graph)
              .transition(t_vertex_entry)
              .attr("r", GlobalUI.base_unit_graph)
              .style("opacity", null);
            // text: variable name inside the circle
            g.append("text")
              .text((d) => d.var_id)
              // .attr("stroke-width", "0.1px")
              .attr("text-anchor", "middle")
              .attr("alignment-baseline", "central")
              .style("opacity", 0)
              .transition(t_vertex_entry)
              .attr("font-size", GlobalUI.base_unit_graph)
              .style("opacity", null);
            // covariance (-> a rotated group that holds an ellipse)
            g.append("g")
              .attr(
                "transform",
                `rotate(${(d.covariance.rot * 180) / Math.PI})`
              )
              .append("ellipse")
              .attr("rx", d.covariance.sigma[0] * Math.sqrt(9.21))
              .attr("ry", d.covariance.sigma[1] * Math.sqrt(9.21))
              .style("opacity", 0) // wow! (see next wow) Nota: doesnt  work with attr()
              .transition(t_vertex_entry)
              .style("opacity", null); // wow! this will look for the CSS (has to a style)
          });
      })
      // on hover, the texts and circles of .vertex will grow in size by 1.4
      .on("mouseover", (e, _) => {
        //circle first
        e.currentTarget
          .selectAll("circle")
          .attr("r", 1.4 * GlobalUI.base_unit_graph);
        // text should grow as well
        e.currentTarget
          .selectAll("text")
          .attr("font-size", `{1.4*GlobalUI.base_unit_graph}px`);
      })
      // on hover out, rebase to default
      .on("mouseout", (e, _) => {
        e.currentTarget
          .selectAll("circle")
          .attr("r", 1 * GlobalUI.base_unit_graph);
        e.currentTarget
          .selectAll("text")
          .attr("font-size", `{1*GlobalUI.base_unit_graph}px`);
      })
  );
}

function join_update_vertex(update) {
  // TODO:
  // Imho best way to avoid to define those transitions everywhere is to
  // transform those functions in classes of which the transitions are members
  const t_graph_motion = d3.transition().duration(1000).ease(d3.easeCubicInOut);

  return update.each(function (d) {
    d3.select(this)
      .selectChild("g")
      .transition(t_graph_motion)
      .attr("transform", "translate(" + d.mean.x + "," + d.mean.y + ")")
      .selection();

    d3.select(this)
      .selectChild("g") //translate
      .selectChild("g") //rotate
      .selectChild("g") // group (incl. rotate)
      .transition(t_graph_motion)
      .attr("transform", `rotate(${(d.covariance.rot * 180) / Math.PI})`)
      .selection()
      .selectChild("ellipse")
      .transition(t_graph_motion)
      .attr("rx", d.covariance.sigma[0] * Math.sqrt(9.21))
      .attr("ry", d.covariance.sigma[1] * Math.sqrt(9.21))
      .selection();
  });
}

function join_exit_vertex(exit) {
  // TODO:
  return exit;
}

/******************************************************************************
 *                            Data Massage estimation
 *****************************************************************************/

// in-place changes to the data structure for convenience when joining
function estimation_data_massage(estimation_data) {
  // Data massage before integration: some data on the vertices array are needed
  // for the factors (1), and the other way around is also true (2)
  // (1) the factors need the position of the vertices (which is found in the data array)
  //     in order to draw the factor/edge at the right position (a line between fact-vertex)
  //     and the position of the full 'dot' representing the factor.
  estimation_data.factors.forEach((f) => {
    f.vars = estimation_data.marginals.filter((marginal) =>
      f.vars_id.includes(marginal.var_id)
    );
    // automagically compute the factor position
    // For a factor involving several variables, the dot is positioned at the
    // barycenter of the variables mean position
    // Obviously (or not), for an unary factor, the factor dot position will reduce
    // to its unique associated node, which is suboptimal...
    if (f.vars.length > 1) {
      f.dot_factor_position = {
        x:
          f.vars.map((a_var) => a_var.mean.x).reduce((a, b) => a + b, 0) /
          f.vars.length,
        y:
          f.vars.map((a_var) => a_var.mean.y).reduce((a, b) => a + b, 0) /
          f.vars.length,
      };
    } else {
      f.dot_factor_position = {
        x: f.vars[0].mean.x,
        y: f.vars[0].mean.y + 5 * GlobalUI.base_unit_graph,
      };
    }
  });

  // (2) the unary factors need the neighbors of their associated node to position
  //      intuitively this factor
  //      So the proposed solution is to add a neighbors array to each vertex containing
  //      the vertices id of its neighbors.
  //      This rely on first step
  //      Seems that there is 2 cases, the node has neighbor(s) or has not (typicaly
  //      happens initially with the initial pose)
  estimation_data.factors
    .filter((f) => f.vars_id.length == 1) // unary factor selection
    .forEach((uf) => {
      const unique_node = uf.vars_id[0];
      //vectors of thetas
      const neighbors = estimation_data.factors.filter(
        (f) => f.factor_id !== uf.factor_id && f.vars_id.includes(unique_node)
      ); // neighbors factors of the node associated with that unary factor
      // TODO: care if no neighbor
      if (neighbors.length > 0) {
        // if there are neighbors factors, the unary factor position must be placed
        // at the biggest angle gap
        const thetas = neighbors
          .map((neighbors_f) =>
            Math.atan2(
              neighbors_f.dot_factor_position.y - uf.vars[0].mean.y,
              neighbors_f.dot_factor_position.x - uf.vars[0].mean.x
            )
          )
          .sort((a, b) => a - b); // mandatory sorting

        const thetas_2pi = thetas.map((t) => t - thetas[0]);
        const dthetas2 = thetas_2pi.map((n, i) => {
          if (i !== thetas_2pi.length - 1) {
            return thetas_2pi[i + 1] - n;
          } else return 2 * Math.PI - n;
        });
        const idx_max = indexOfMax(dthetas2);
        const theta_unary = ecpi(thetas[idx_max] + dthetas2[idx_max] / 2);

        // distance of the factor wrt the vertex.
        const squares_distances = neighbors.map(
          (nf) =>
            (nf.dot_factor_position.y - uf.vars[0].mean.y) ** 2 +
            (nf.dot_factor_position.x - uf.vars[0].mean.x) ** 2
        );
        const u_distance = Math.sqrt(
          Math.min(25, Math.max(...squares_distances))
        );
        // TODO: place the hard-coded 25 in globalUI

        // position of factor dot infered from polar coordinates
        const new_uf_position = {
          x: uf.vars[0].mean.x + u_distance * Math.cos(theta_unary),
          y: uf.vars[0].mean.y + u_distance * Math.sin(theta_unary),
        };
        // giving new position
        uf.dot_factor_position = new_uf_position;
      } else {
        // no neighbors
        const theta_unary = Math.PI / 2;
        const u_distance = 5;
        const new_uf_position = {
          x: uf.vars[0].mean.x + u_distance * Math.cos(theta_unary),
          y: uf.vars[0].mean.y + u_distance * Math.sin(theta_unary),
        };
        // giving new position
        uf.dot_factor_position = new_uf_position;
      }
    });
}

// TODO : deal with this
function estimation_query_last_pose(an_agent_estimation) {
  an_agent_estimation.last_pose.state = an_agent_estimation.graph.marginals.filter(
    (marginal) => marginal.var_id == an_agent_estimation.last_pose.last_pose_id
  )[0].mean;
}
