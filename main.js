/*
 * Copyright 2021 AKKA Technologies (joel.tari-summerfield@akka.eu)
 *
 * Licensed under the EUPL, Version 1.2 or – as soon they will be approved by
 * the European Commission - subsequent versions of the EUPL (the "Licence");
 * You may not use this work except in compliance with the Licence.
 * You may obtain a copy of the Licence at:
 *
 * https://joinup.ec.europa.eu/software/page/eupl
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the Licence is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the Licence for the specific language governing permissions and
 * limitations under the Licence.
 */

// viewbox and aspect ratio values, this assume we start the viz centered
// around origin, with unit-sized-width
const aspect_ratio = 0.6;
const viewbox = [-1 / 2.0, -aspect_ratio / 2.0, 1.0, aspect_ratio];

// DOM related
const elBody = d3.select("body");
const elSvg = d3.select("svg.svg-content").attr("viewBox", viewbox.toString());
const elAxes = d3.select("svg .axes");
// Define the div for the tooltip
const elDivTooltip = d3.select("body").append("div").classed("tooltip", true);

// initially no robot is selected
GlobalUI = {
  selected_robot_id: null,
  base_unit_graph: 1, // unit that controls the dimensions of the graph
  // (set when receiving a graph and getting a medium distance between the nodes)
  // this is the default setting, it can instead be set by the message header of graph
  excess_zoom_compensator: 1, // different than 1 when zoom too much/too little
  covariance_visible: true,
  dim: {
    factor_dot_r: 0.3,
    factor_dot_r_mouseover: 0.42,
    factor_dot_width: 0.05,
    factor_dot_width_mouseover: 0.1,
    factor_line_width: 0.15,
    vertex_circle_r: 1,
    vertex_circle_width: 0.12,
    // 1340 font-size, mouseover vertex, mousemv etc..
    vertex_font_size: function (str_size) {
      return (3 - str_size) / 6 + 1;
    },
    covariance_ellipse_width: 0.03,
  },
  get_unified_scaling_coefficient: function () {
    return this.base_unit_graph * this.excess_zoom_compensator;
  },
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
const zoom = d3.zoom().on("zoom", zoomed);

elSvg.call(zoom);

// handler that receive the zoom transform, and apply it to elements
function zoomed({ transform }) {
  // apply the zoom transform to the main group
  elMainGroup.attr("transform", transform);

  const max_scale = 0.2; // TODO: automate this max_scale: based on an ideal font maybe
  if (transform.k > max_scale) {
    // must compensate for excessive scaling
    GlobalUI.excess_zoom_compensator = max_scale / transform.k;
    // update_graph_dimensions(GlobalUI.get_unified_scaling_coefficient());
  } else {
    GlobalUI.excess_zoom_compensator = 1;
    // update_graph_dimensions(GlobalUI.get_unified_scaling_coefficient());
  }

  // rescale the axes x & y
  const sc_xz = transform.rescaleX(sc_x);
  const sc_yz = transform.rescaleY(sc_y);

  // adjust the axis-object for the new scale
  // & re-apply the axis-object on the axis-dom-element
  elAxes.select(".Xaxis-bot").call(xaxis_bot.scale(sc_xz));
  elAxes.select(".Xaxis-top").call(xaxis_top.scale(sc_xz));
  elAxes.select(".Yaxis-left").call(yaxis_left.scale(sc_yz));
  elAxes.select(".Yaxis-right").call(yaxis_right.scale(sc_yz));

  // (currently in CSS)
  // elAxes.selectAll('text').attr('font-size','0.007px')
  // elAxes.selectAll('path').attr('stroke-width',0.0004)
  // elAxes.selectAll('line').attr('stroke-width',0.0004)
}

// it's like an update, but outside the d3js pattern as there are no new data per say,
// but some dimensional aspect of the graph need to change to ease the viz experience
// (due to excessive zoom for example)
function update_graph_dimensions(coef) {
  d3.selectAll(".factor circle")
    .attr("r", GlobalUI.dim.factor_dot_r * coef)
    .attr("stroke-width", GlobalUI.dim.factor_dot_width * coef);

  d3.selectAll(".factor line").attr(
    "stroke-width",
    GlobalUI.dim.factor_line_width * coef
  );

  d3.selectAll(".vertex circle")
    .attr("r", GlobalUI.dim.vertex_circle_r * coef)
    .attr("stroke-width", GlobalUI.dim.vertex_circle_width * coef);
  d3.selectAll(".vertex text").attr(
    "font-size",
    (d) => GlobalUI.dim.vertex_font_size(d.var_id.length) * coef
  );
}

/******************************************************************************
 *                            D3 Axes and Scales
 *****************************************************************************/

// Define the axes with 1:1 scales
let sc_x = d3.scaleLinear().domain([-0.5, 0.5]).nice().range([-0.5, 0.5]);
let sc_y = d3.scaleLinear().domain([-0.3, 0.3]).nice().range([-0.3, 0.3]);
// define the axes objects, based on the scale, and adjust for the default viewbox
let xaxis_bot = d3
  .axisTop(sc_x) // top=[..]Bottom  is  c-intuitive, but I want the ticks & text to extend inward
  .tickPadding(0.005) // how far the text is from axis (normal direction)
  .tickSizeInner(-0.005) // length of the ticks (in the normal direction to the axis)
  .tickSizeOuter(0) // no  outer ticks (modify the path)
  .offset(0); // no offset between path and lines

let xaxis_top = d3
  .axisBottom(sc_x) // top=[..]Bottom  is  c-intuitive, but I want the ticks & text to extend inward
  .tickPadding(0.005) // how far the text is from axis (normal direction)
  .tickSizeInner(-0.005) // length of the ticks (in the normal direction to the axis)
  .tickSizeOuter(0) // no  outer ticks (modify the path)
  .offset(0); // no offset between path and lines

let yaxis_left = d3
  .axisRight(sc_y) // right=[..]Left  is  c-intuitive, but I want the ticks & text to extend inward
  .tickPadding(0.005) // how far the text is from axis (normal direction)
  .tickSizeInner(0.005) // length of the ticks (in the normal direction to the axis)
  .tickSizeOuter(0) // no  outer ticks (modify the path)
  .offset(0); // no offset between path and lines

let yaxis_right = d3
  .axisLeft(sc_y) // right=[..]Left  is  c-intuitive, but I want the ticks & text to extend inward
  .tickPadding(0.005) // how far the text is from axis (normal direction)
  .tickSizeInner(0.005) // length of the ticks (in the normal direction to the axis)
  .tickSizeOuter(0) // no  outer ticks (modify the path)
  .offset(0); // no offset between path and lines

// generate the axes elements from the d3-axis object
elAxes
  .select(".Xaxis-top")
  .attr("transform", "translate(0,0.3)")
  .call(xaxis_top);
elAxes
  .select(".Xaxis-bot")
  .attr("transform", "translate(0,-0.3)")
  .call(xaxis_bot);
elAxes
  .select(".Yaxis-left")
  .attr("transform", "translate(-0.5,0)")
  .call(yaxis_left);
elAxes
  .select(".Yaxis-right")
  .attr("transform", "translate(0.5,0)")
  .call(yaxis_right);

// (currently in CSS)
// elAxes.selectAll('text').attr('font-size','0.007px')
// elAxes.selectAll('path').attr('stroke-width',0.0004)
// elAxes.selectAll('line').attr('stroke-width',0.0004)

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

  client.subscribe("meta_info", function (err) {
    if (!err) {
      console.log("[mqtt] subscribed to the topic >> estimation_graph");
    }
  });
});

// this event receive incoming mqtt topics (for which there is a subscription) and acts accordingly
client.on("message", function (topic, message) {
  // parse the msg by assuming it's json
  const msg = JSON.parse(message.toString());

  if (topic == "meta_info") {
    // msg from meta_info should contain info such as agents, landmarks if any
    // or any other info that helps structure the DOM for the visualization
    // This is mandatory to use this topic when deviating from the base default
  } else if (topic == "ground_truth") {
    // This topic should be called in fact metaInfo or something when no ground truth is available
    // create the AgentTeam instanciate the robot objects
    // create a GeUpPa for the landmarks
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
      AgentTeam[robot_id] = new fullAgentViz(
        robot_ground_truth,
        client,
        elAgents
      );
      // activate mqtt subscriptions
      AgentTeam[robot_id].subscribeTopicsToMqtt();
    }
  } else if (topic.split("/").length == 2) {
    const [agent_id, topic_suffix] = topic.split("/");
    AgentTeam.checkSubscriptions(agent_id, topic_suffix, msg);
  }
  
  // finally adapt the zoom to the new bounding-box
  const bbox=elMainGroup.node().getBBox();

  // A 1.2 coef is applied to obtain some margin around the bounding-box
  const scaleValue = 1.0 / (1.2*Math.max(bbox.width, bbox.height));
    
  // translate value are the center , 
  // (but with minus, as its a transform applied to the target zoom)
  const translateValueX = - (bbox.x + bbox.width/2);
  const translateValueY = -(bbox.y + bbox.height/2);

  elSvg.transition().duration(500).call(
    zoom.transform,
    d3.zoomIdentity
      .scale(scaleValue) 
      .translate(translateValueX, translateValueY)
  );

});

/******************************************************************************
 *                            Class AgentViz
 *****************************************************************************/
constructD3FactorGraph = function (d3container, robot_id) {
  return d3container
    .append("g")
    .classed("factor_graph", true)
    .attr("id", robot_id)
    .call(function (g_factor_graph) {
      g_factor_graph.append("g").classed("covariances_group", true);
      g_factor_graph.append("g").classed("factors_group", true);
      g_factor_graph.append("g").classed("vertices_group", true);
    });
};

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
    this.sub_topics["graphs"] = this.graphsCallback.bind(this);
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
    // console.log(graph);

    // base unit graph is a coefficient to help size the components of the graph
    // different graphs requires different size of vertex circle, stroke-width etc..
    // The graph can have such a coefficient in the header, otherwise a value is computed
    if (graph.header.base_unit != null) {
      GlobalUI.base_unit_graph = graph.header.base_unit;
    } else {
      // compute the base unit graph based on the median calculation of all distances
      // between connected nodes
      const node_distances = graph.factors.map((f) => {
        if (f.type === "odometry") {
          return sqDist(
            graph.marginals.find((v) => v.var_id === f.vars_id[0]),
            graph.marginals.find((v) => v.var_id === f.vars_id[1])
          );
        }
      });
      node_distances.sort((a, b) => a - b);
      const half = Math.floor(node_distances.length / 2);
      const median = Math.sqrt(node_distances[half]);
      // for a median distance of 1 between nodes, 0.15 is the coefficient
      GlobalUI.base_unit_graph = 0.15 * median;
    }

    // This part define a zoom transform that is relevant, based on the new data
    //        get, for this graph, the left/right/top/bottom-most values
    //        save them in the Agent Class
    //        Then, the globalUI retrieve the left/right/top/bottom-most values
    //        among the agent-team, and that is what defines the XY-aligned
    //        bounding-box (so that we see all graphs)
    const [mx, Mx, my, My] = graph.marginals.reduce(
      (tmp_bb, cur) => [
        Math.min(tmp_bb[0], cur.mean.x),
        Math.max(tmp_bb[1], cur.mean.x),
        Math.min(tmp_bb[2], cur.mean.y),
        Math.max(tmp_bb[3], cur.mean.y),
      ],
      [Infinity, -Infinity, Infinity, -Infinity] // initial extreme values
    );

    // console.log(`Bounding box is [${mx.toFixed(2)}, ${my.toFixed(2)}, ${Mx.toFixed(2)}, ${My.toFixed(2)}]`)

    // The axis that has the longuest span imposes the scale on the zoom.
    // Note that the vertical span must also be corrected by the aspect ratio
    // of the viewbox.
    // A 1.2 coef is applied to obtain some margin around the bounding-box
    const scaleValue =
      1.0 / (1.2 * Math.max(Mx - mx, (My - my) * aspect_ratio));

    // translate value are the center ,
    // (but with minus, as its a transform applied to the target zoom)
    const translateValueX = -(mx + Mx) / 2;
    const translateValueY = -(my + My) / 2;

    // apply the zoom transform
    elSvg
      .transition()
      .duration(1500)
      .call(
        zoom.transform,
        d3.zoomIdentity
          .scale(scaleValue)
          .translate(translateValueX, translateValueY)
      );

    // massage data: the dot factor positions must be explicitly computed here:
    //    - when a factor connects 2 or more variables: position = barycenter
    //    - when a factor connects only 1 variable: try to find an intuitive positioning
    //                                              that depends on the other factors
    //                                              linked to that variable
    graph.obj_marginals = objectify_marginals(graph.marginals);
    graph.obj_factors = objectify_factors(graph.factors);
    compute_factor_set(graph);
    estimation_data_massage(graph);
    console.log("Pre-visualization treatment done");

    // general update pattern
    // first the covariances
    // then the factors (therefore on top of the cov)
    // then the vertices (therefore on top of the factors)
    this.d3FactorGraph
      .select("g.covariances_group")
      .selectAll(".covariance")
      .data(graph.marginals)
      .join(join_enter_covariance, join_update_covariance); // TODO: exit covariance

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

    // update if necessary the last pose
    // only necessary when there is id/odom topic
    // because the odom SE2 or AA value is in reference to the last graph pose
    if (graph.marginals.length > 0) {
      const most_recent_variable = graph.marginals.find(
        (m) => m.most_recent === "true"
      );
      if (most_recent_variable != null) {
        this.last_pose_id = most_recent_variable.var_id;
      } else {
        // try to guess what the most recent pose is.
        // It's most likely the pose 'x??' with the biggest number !
        // It is assumed pose start with 'x'
        const biggest_number = Math.max(
          ...graph.marginals
            .filter((m) => m.var_id.match("x"))
            .map((m) => m.var_id.split("x")[1])
        );
        this.last_pose_id = `x${biggest_number}`;
      }
      this.mean_reference =
        graph.marginals[
          graph.marginals.findIndex((m) => m.var_id == this.last_pose_id)
        ].mean;
    } else {
      // there is no pose to refer to, so use 0
      this.last_pose_id = null;
      this.mean_reference = { x: 0, y: 0, th: 0 };
      console.warn("No pose in the graph to refer odometry");
    }
    console.log(
      `Odometry pose reference ${this.last_pose_id} at ${this.mean_reference}`
    );
  };
}

class fullAgentViz extends BaseAgentViz {
  constructor(robot_data, mqttc, parent_container) {
    // base class ctor
    // console.log(robot_data)
    super(robot_data.robot_id, mqttc, parent_container);

    this.sensor_svg_path = this.sensorVisual(robot_data.sensors.observation);
    this.odometry_type = robot_data.sensors.odometry.type;
    this.history_odom = []; // successive poses of the odometry (from the last graph pose): emptied when a new pose is created on the graph
    this.history_graph = []; // succesives poses of the graph (x0 to x{last_pose})
    this.history_true = []; // successives true poses
    // this.svg_history_truth = ""; // str version of the history truth
    this.max_history_elements = 100; // TODO: apply (maybe not the same size-reducing rules for the 3)...
    //      ex: history_true: there could be a min eucl. distance btw elements to reduce size
    //          history_odom: remove every other element each time the threshold is reached
    this.registerGroundTruthData(robot_data.state);
    // initialise the reference (from odometry display) to the ground_truth
    this.mean_reference = robot_data.state;
    // d3 : create the truth structure
    this.d3Truth = constructD3Truth(
      this.d3container,
      this.d3parent_container,
      robot_data,
      this.sensor_svg_path,
      this.state_history
    );
    // topic names (INs)
    this.sub_topics["odom"] = this.odomCallback.bind(this);
    this.sub_topics["measures_feedback"] = this.measuresCallback.bind(this);
    this.sub_topics["ground_truth"] = this.groundTruthCallback.bind(this);

    // topic names (OUTs)
    this.topic_request_ground_truth = `${this.id}/request_ground_truth`;
    this.topic_cmd = `${this.id}/cmd`;
    this.topic_request_graph = "request_graphs";
    // d3 : create the odom structure
    this.d3Odom = constructD3Odom(this.d3container, this.id);
    this.last_pose_id = null;
    // this.mean_reference = { x: 0, y: 0, th: 0 }; // reference pose and its mean for odometry
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
        .attr("points", state_history.map((e) => `${e[0]},${e[1]}`).join(" "));
    } else if (state_history.length >= 2) {
      //update
      this.d3Truth.select("polyline.state_history").attr(
        "points",
        state_history.map((e) => `${e[0]},${e[1]}`).join(" ") //+ ` ${state.x},${state.y}`
      );
    }
  };
  // update visual odom
  updateVisualOdom = function (odom_history, state, visual_covariance = null) {
    // odom becomes visible
    this.d3Odom.style("visibility", null);

    this.d3Odom
      .attr(
        "transform",
        `translate(${this.mean_reference.x},${this.mean_reference.y}) rotate(${
          (this.mean_reference.th * 180) / Math.PI
        })`
      )
      .select("g.odom_reference")
      .attr(
        "transform",
        `translate(${state.dx},${state.dy}) rotate(${
          (state.dth * 180) / Math.PI
        })`
      )
      .select("ellipse.odom_covariance")
      .call(function (ellipse) {
        if (visual_covariance !== null) {
          // covariance could be 0 (impossible to draw, therefore not transmited in the data)
          ellipse
            .attr("rx", visual_covariance.sigma[0] * Math.sqrt(9.21))
            .attr("ry", visual_covariance.sigma[1] * Math.sqrt(9.21))
            .attr(
              "transform",
              `rotate(${((visual_covariance.rot - state.dth) * 180) / Math.PI})`
            ); // I substract because I must cancel the state.dth rotation for the ellipse
          // (I don't want to create another DOM wrapping element just for that)
        }
      });
    if (odom_history.length >= 2) {
      this.d3Odom
        .select("polyline.odom_history")
        .attr("points", odom_history.map((e) => `${e[0]},${e[1]}`).join(" "));
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
      state.x,
      state.y,
      state.th,
      3,
      this.history_true,
      this.max_history_elements
    );
  };

  registerOdomData = function (data) {
    this.current_odom_state = data.state;
    this.current_odom_cov = data.visual_covariance;
    this.history_odom = history2d_push(
      data.state.dx,
      data.state.dy,
      data.state.dth,
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
    // console.log("Receive some odom response " + this.id + " with data: ");
    // console.log(data);
    this.registerOdomData(data);
    this.updateVisualOdom(
      this.history_odom,
      data.state,
      data.visual_covariance
    );
  };
  measuresCallback = function (data) {
    console.log("Receive some measure :" + this.id);
    this.transcientMeasureVisual(this.current_true_state, data.measures);
  };
  groundTruthCallback = function (data) {
    // console.log("Receive some GT info :" + this.id);
    // console.log(data)
    this.registerGroundTruthData(data.state);
    this.updateVisualTruth(this.history_true, data.state);
  };
}

/******************************************************************************
 *                          Declaration of AgentTeam
 *****************************************************************************/
const AgentTeam = {
  checkSubscriptions: function (agent_id, topic_suffix, msg) {
    this[agent_id].mqttProcessTopicSuffix(topic_suffix, msg);
  },
};
// create a base default agent in any case
AgentTeam["default"] = new BaseAgentViz("default", client, elAgents);
AgentTeam.default.subscribeTopicsToMqtt();

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

  // determine type of input to apply (AA/DD)
  if (GlobalUI.selected_robot_id == null) {
    console.warn("impossible to apply input cmd, no robot is selected.");
    return;
  }
  const inputCmdModel = AgentTeam[GlobalUI.selected_robot_id].odometry_type;

  // command to execute
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

elSvg.on("click", (e) => console.log(d3.pointer(e, elMainGroup.node())));

elBody.on("keyup", (e) => (keyPressedBuffer[e.key] = false));

function getTransform_gg(d3_single_selected) {
  // only works on double group descendant framework
  // (first descendant is translation, second is rotation)
  curx = d3_single_selected.selectChild("g").node().transform.baseVal[0]
    .matrix.e;
  cury = d3_single_selected.selectChild("g").node().transform.baseVal[0]
    .matrix.f;
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
  x,
  y,
  th,
  distance_treshold,
  current_history,
  max_history_size
) {
  if (current_history.length < 2) {
    current_history.push([x, y, th]);
  } else {
    // const last_truth_pose = current_history[current_history.length - 1];
    const ante_truth_pose = current_history[current_history.length - 2];
    // do we replace last truth pose or do we push a new elem ?
    if (
      (x - ante_truth_pose[0]) ** 2 + (y - ante_truth_pose[1]) ** 2 >
      distance_treshold ** 2
    ) {
      // current_history.push(state);
      current_history.push([x, y, th]);
      // protecting against array overflow (wrt the defined max size)
      if (current_history.length > max_history_size) {
        current_history.shift();
      }
    } else {
      current_history[current_history.length - 1] = [x, y, th];
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
          if (robot_data.sensors.observation != null)
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
        .classed("odom_reference", true)
        .call(function (g_tra) {
          g_tra
            .append("polygon")
            .classed("ghost", true)
            .attr("points", "0,-1 0,1 3,0")
            .style("stroke-opacity", 0)
            .transition()
            .duration(400)
            .style("stroke-opacity", null);
          g_tra
            .append("line")
            .classed("ghost", true)
            .attr("points", "0,-1 0,1 3,0")
            .attr("x1", 0)
            .attr("y1", 0)
            .attr("x2", 1)
            .attr("y2", 0)
            .style("stroke-opacity", 0)
            .transition()
            .duration(400)
            .style("stroke-opacity", null);
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
                .attr(
                  "stroke-width",
                  GlobalUI.dim.factor_line_width *
                    GlobalUI.get_unified_scaling_coefficient()
                )
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
              .attr(
                "stroke-width",
                GlobalUI.dim.factor_line_width *
                  GlobalUI.get_unified_scaling_coefficient()
              )
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
            .attr(
              "r",
              2 *
                GlobalUI.dim.factor_dot_r *
                GlobalUI.get_unified_scaling_coefficient()
            ) // *2 is transitory
            .attr(
              "stroke-width",
              GlobalUI.dim.factor_dot_width *
                GlobalUI.get_unified_scaling_coefficient()
            )
            // on hover, dot-circle of factor grows and tooltip displays
            // define remotely for clarity
            .call(factor_hover)
            // opacity transition not necessary here
            .transition("fc")
            .duration(2200)
            // .style("opacity")
            .attr(
              "r",
              GlobalUI.dim.factor_dot_r *
                GlobalUI.get_unified_scaling_coefficient()
            );
        });
    });
}

function join_update_factor(update) {
  // TODO:
  // Imho best way to avoid to define those transitions everywhere is to
  // transform those functions in classes of which the transitions are members
  const t_graph_motion = d3.transition().duration(1000).ease(d3.easeCubicInOut);

  update.each(function (d) {
    d3.select(this)
      .selectAll("line")
      .each(function (_, i, n) {
        if (d.vars.length > 1) {
          // line
          d3.select(n[i])
            .transition(t_graph_motion)
            .attr("x1", d.dot_factor_position.x)
            .attr("y1", d.dot_factor_position.y)
            .attr("x2", d.vars[i].mean.x)
            .attr("y2", d.vars[i].mean.y);
        } else {
          // update unary factor
          // WARN TODO: a factor_id should not change its vars_id
          d3.select(n[i])
            .transition(t_graph_motion)
            .attr("x1", d.vars[0].mean.x)
            .attr("y1", d.vars[0].mean.y)
            .attr("x2", d.dot_factor_position.x)
            .attr("y2", d.dot_factor_position.y);
        }
      });
  });
  // the little factor circle (to visually differentiate from with MRF)
  update
    .select("circle")
    .transition(t_graph_motion)
    .attr("cx", (d) => d.dot_factor_position.x)
    .attr("cy", (d) => d.dot_factor_position.y);
}

function join_exit_factor(exit) {
  return (
    exit
      .call(function (ex) {
        ex.selectAll("line").style("stroke", "brown");
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

  return enter
    .append("g")
    .classed("vertex", true) // TODO: necessary ?
    .attr("id", (d) => d.var_id)
    .attr("transform", "rotate(0)")
    .each(function (d) {
      d3.select(this).attr(
        "transform",
        "translate(" + d.mean.x + "," + d.mean.y + ")"
      );
      //   .append("g")
      //   .append("g")
      // .call(function (g) {
      d3.select(this)
        .append("circle")
        .attr(
          "r",
          10 *
            GlobalUI.dim.vertex_circle_r *
            GlobalUI.get_unified_scaling_coefficient()
        ) // 10-fold is transitory
        .style("opacity", 0)
        .attr(
          "stroke-width",
          GlobalUI.dim.vertex_circle_width *
            GlobalUI.get_unified_scaling_coefficient()
        )
        .transition(t_vertex_entry)
        .attr(
          "r",
          GlobalUI.dim.vertex_circle_r *
            GlobalUI.get_unified_scaling_coefficient()
        )
        .style("opacity", null);
      // text: variable name inside the circle
      d3.select(this)
        .append("text")
        .text((d) => d.var_id)
        // .attr("stroke-width", "0.1px")
        // .attr("text-anchor", "middle")
        // .attr("alignment-baseline", "central")
        .style("opacity", 0)
        .transition(t_vertex_entry)
        .attr(
          "font-size",
          GlobalUI.dim.vertex_font_size(d.var_id.length) *
            GlobalUI.get_unified_scaling_coefficient()
        )
        .style("opacity", null);
      d3.select(this)
        .append("circle")
        .classed("hover_transparent_circle", true)
        .style("opacity", 0)
        .attr(
          "r",
          GlobalUI.dim.vertex_circle_r *
            GlobalUI.get_unified_scaling_coefficient()
        )
        .attr(
          "stroke-width",
          GlobalUI.dim.vertex_circle_width *
            GlobalUI.get_unified_scaling_coefficient()
        )
        // hover methods (defined remotely for clarity)
        .call(vertex_hover);
    });
}

function join_update_vertex(update) {
  const t_graph_motion = d3.transition().duration(1000).ease(d3.easeCubicInOut);
  update
    // .each(function (d) {
    // d3.select(this)
    .transition(t_graph_motion)
    .attr("transform", (d) => `translate(${d.mean.x}, ${d.mean.y})`);
  // });
}

function join_exit_vertex(exit) {
  // TODO:
  return exit;
}

function join_enter_covariance(enter) {
  return enter
    .append("ellipse")
    .classed("covariance", true)
    .attr("id", (d) => d.var_id)
    .attr(
      "transform",
      (d) =>
        `translate(${d.mean.x},${d.mean.y}) rotate(${
          (d.covariance.rot * 180) / Math.PI
        })`
    )
    .attr("rx", (d) => d.covariance.sigma[0] * Math.sqrt(9.21))
    .attr("ry", (d) => d.covariance.sigma[1] * Math.sqrt(9.21))
    .attr(
      "stroke-width",
      GlobalUI.dim.covariance_ellipse_width *
        GlobalUI.get_unified_scaling_coefficient()
    );
  // .style("opacity", 0) // wow! (see next wow) Nota: doesnt  work with attr()
  // .transition()
  // .duration(400)
  // .style("opacity", null); // wow! this will look for the CSS (has to a style)
}

function join_update_covariance(update) {
  const t_graph_motion = d3.transition().duration(1000).ease(d3.easeCubicInOut);
  update
    .transition(t_graph_motion)
    .attr(
      "transform",
      (d) =>
        `translate(${d.mean.x},${d.mean.y}) rotate(${
          (d.covariance.rot * 180) / Math.PI
        })`
    )
    .attr("rx", (d) => d.covariance.sigma[0] * Math.sqrt(9.21))
    .attr("ry", (d) => d.covariance.sigma[1] * Math.sqrt(9.21));
}

/******************************************************************************
 *                            Hover behaviour
 *****************************************************************************/

function vertex_hover(vertex_circle) {
  vertex_circle
    // on hover, the texts and circles of .vertex will grow in size by 1.4
    .on("mouseover", (e, d) => {
      // grow the vertex circle, raise the element
      d3.select(`.vertex#${d.var_id}`)
        .selectAll("circle") // TODO: necessary hover circle ??
        .attr(
          "r",
          1.4 *
            GlobalUI.dim.vertex_circle_r *
            GlobalUI.get_unified_scaling_coefficient()
        )
        .attr(
          "stroke-width",
          1.4 *
            GlobalUI.dim.vertex_circle_width *
            GlobalUI.get_unified_scaling_coefficient()
        );
      // text should grow as well
      d3.select(`.vertex#${d.var_id}`)
        .selectAll("text")
        .attr(
          "font-size",
          1.4 *
            GlobalUI.dim.vertex_font_size(d.var_id.length) *
            GlobalUI.get_unified_scaling_coefficient()
        );
      // fill the covariance
      d3.select(`#${d.var_id}.covariance`).style('visibility','visible');
      d3.select(`#${d.var_id}.covariance`).classed("highlight", true);
      // compute the separator for the tooltip
      separator_set=compute_separator(d);
      // highlights on the factor and separator set
      d.factor_set.forEach(factor_id => 
        {
          d3.select(`.factor#${factor_id}`).classed("link_highlight", true).raise();
          d3.select(`.factor#${factor_id}`).datum().vars_id.forEach((var_str) =>
            d3.select(`.vertex#${var_str}`).classed("link_highlight", true).raise()
          );
        }
      )
      // raise the hovered vertex at the top
      d3.select(`.vertex#${d.var_id}`)
        .raise()
      // the tooltip
      elDivTooltip
        .style("left", `${e.pageX}px`)
        .style("top", `${e.pageY - 6}px`)
        .style("visibility", "visible").html(`<p class="tooltip-title">
                          <strong><em>${d.var_id}</em></strong>
                         </p>
                         <br>
                         <span class="tooltip-field"><strong>Mean</strong></span>: 
                         <span class="tooltip-value">${JSON.stringify(
                           d.mean,
                           (k, v) => (v.toPrecision ? v.toPrecision(4) : v),
                           "\t"
                         )}</span>
                         <br>
                         <span class="tooltip-field"><strong>Separator</strong></span>: 
                         <span class="tooltip-value">${separator_set}</span>
                         `);
      // change the pointer
      d3.select(`.vertex#${d.var_id}`).style("cursor", "pointer");
    })
    .on("mousemove", (e) =>
      elDivTooltip.style("top", e.pageY + "px").style("left", e.pageX + "px")
    )
    // on hover out, rebase to default
    .on("mouseout", (e, d) => {
      d3.select(`.vertex#${d.var_id}`)
        .selectAll("circle")
        .attr(
          "stroke-width",
          GlobalUI.dim.vertex_circle_width *
            GlobalUI.get_unified_scaling_coefficient()
        )
        .attr(
          "r",
          GlobalUI.dim.vertex_circle_r *
            GlobalUI.get_unified_scaling_coefficient()
        );
      d3.select(`.vertex#${d.var_id}`)
        .selectAll("text")
        .attr(
          "font-size",
          GlobalUI.dim.vertex_font_size(d.var_id.length) *
            GlobalUI.get_unified_scaling_coefficient()
        );
      // remove the highlight on the factor and separator
      d.factor_set.forEach(factor_id => 
        {
          d3.select(`.factor#${factor_id}`).classed("link_highlight", false);
          d3.select(`.factor#${factor_id}`).datum().vars_id.forEach((var_str) =>
            d3.select(`.vertex#${var_str}`).classed("link_highlight", false)
          );
        }
      )
      // remove covariance highlight
      d3.select(`#${d.var_id}.covariance`).classed("highlight", false);
      // remove covariance visibility, depending on global UI
      d3.select(`#${d.var_id}.covariance`).style("visibility", `${GlobalUI.covariance_visible ? 'visible' : 'hidden'}`);
      // hide the tooltip
      d3.select(`.vertex#${d.var_id}`).style("cursor", "default");
      elDivTooltip.style("visibility", "hidden");
    });
}

function factor_hover(factor_dot) {
  factor_dot
    .on("mouseover", (e, d) => {
      //circle first
      d3.select(e.currentTarget)
        .attr(
          "r",
          1.4 *
            GlobalUI.dim.factor_dot_r *
            GlobalUI.get_unified_scaling_coefficient()
        )
        .attr(
          "stroke-width",
          2 *
            GlobalUI.dim.factor_dot_width *
            GlobalUI.get_unified_scaling_coefficient()
        );
      // highlight the line(s) from the dot factor to the vertex/ices , and also the stroke of this/ese vertex/ices
      d3.select(`.factor#${d.factor_id}`)
        .classed("link_highlight", true)
        .raise();
      d.vars_id.forEach((var_str) =>
        d3.select(`.vertex#${var_str}`).classed("link_highlight", true).raise()
      );
      // raise this factor, as well as the connected nodes (the .raise() in previous loop)
      d3.select(e.currentTarget).raise();
      // the tooltip
      elDivTooltip
        .style("left", `${e.pageX}px`)
        .style("top", `${e.pageY - 6}px`)
        .style("visibility", "visible").html(`<p class="tooltip-title">
                        <strong><em>${d.factor_id}</em></strong>
                       </p>
                       <br>
                       <span class="tooltip-field"><strong>Type</strong></span>: 
                       <span class="tooltip-value">${d.type}</span>
                       <br>
                       <span class="tooltip-field"><strong>Vars</strong></span>: 
                       <span class="tooltip-value">${d.vars_id}</span>
                       `);
      // cursor pointer
      d3.select(e.currentTarget).style("cursor", "pointer");
    })
    .on("mousemove", (e) =>
      elDivTooltip.style("top", e.pageY + "px").style("left", e.pageX + "px")
    )
    // on hover out, rebase to default
    .on("mouseout", (e, d) => {
      // retract the radius of the factor dot
      d3.select(e.currentTarget)
        .attr(
          "r",
          GlobalUI.dim.factor_dot_r * GlobalUI.get_unified_scaling_coefficient()
        )
        .attr(
          "stroke-width",
          GlobalUI.dim.factor_dot_width *
            GlobalUI.get_unified_scaling_coefficient()
        );
      // remove the highlight on the surroundings
      d3.select(`.factor#${d.factor_id}`).classed("link_highlight", false);
      d.vars_id.forEach((var_str) =>
        d3.select(`.vertex#${var_str}`).classed("link_highlight", false)
      );
      // hide the tooltip
      d3.select(e.currentTarget).style("cursor", "default");
      elDivTooltip.style("visibility", "hidden");
    });
}

/******************************************************************************
 *                            Data Massage estimation
 *****************************************************************************/

// in-place changes to the data structure for convenience when joining
function estimation_data_massage({factors: d_factors, marginals: d_marginals, obj_factors: d_obj_factors, obj_marginals: d_obj_marginals}) {
  // Data massage before integration: some data on the vertices array are needed
  // to position spatially the factors (1), and the other way around is also true (2)
  // (1) the factors need the position of the vertices (which is found in the marginals part of the data array)
  //     in order to draw the factor/edge at the right position (a line between fact-vertex)
  //     and the position of the full 'dot' representing the factor.
  d_factors.forEach((f) => {
    f.vars = d_marginals.filter((marginal) => // construct d_factors[..].vars : a subset of the marginals
        f.vars_id.includes(marginal.var_id)
      );
    // // the node set of this factor
    // const node_set= 
    // automagically compute the factor position
    // For a factor involving several variables, the dot is positioned at the
    // barycenter of the variables mean position
    // Obviously (or not), for an unary factor, the factor dot position will reduce
    // to its unique associated node, which is suboptimal...
    if (f.vars_id.length > 1) {
      f.dot_factor_position = {
        x:
          f.vars_id.reduce((sum_acc, var_id) => sum_acc + d_obj_marginals[var_id].mean.x ,0 ) 
          /f.vars_id.length,// mean.x
          // f.vars.map((a_var) => a_var.mean.x).reduce((a, b) => a + b, 0) /
          // f.vars.length,
        y:
          f.vars_id.reduce((sum_acc, var_id) => sum_acc + d_obj_marginals[var_id].mean.y ,0 ) 
          /f.vars_id.length,// mean.x
          // f.vars.map((a_var) => a_var.mean.y).reduce((a, b) => a + b, 0) /
          // f.vars.length,
      };
    } else {
      f.dot_factor_position = {
        x: d_obj_marginals[f.vars_id[0]].mean.y,
        y: d_obj_marginals[f.vars_id[0]].mean.x + 5 * GlobalUI.base_unit_graph,
        // TODO: place the hard-coded 5 in globalUI
        // x: f.vars[0].mean.x,
        // y: f.vars[0].mean.y + 5 * GlobalUI.base_unit_graph,
      };
    }
    // also add it in d_obj_factors
    d_obj_factors[f.factor_id].dot_factor_position=f.dot_factor_position;
  });

  // (2) This solves the problem on how to position the unary factor relative to it's
  //      (only) vertex.
  //      Assume this vertex is connected to other factors, we choose to position the unary
  //      factor in a free area around the vertex : in the center of the widest free angle
  //      available.
  //      the unary factors need the position of the neighbors of their associated node to position
  //      intuitively this factor
  //      So the proposed solution is to add a neighbors array to each vertex containing
  //      the vertices id of its neighbors.
  //      This rely on first step
  //      Seems that there is 2 cases, the node has neighbor(s) or has not (typically
  //      happens initially with the initial pose)
  d_factors
    .filter((f) => f.vars_id.length == 1) 
    // with an array of the unifactors
    .forEach((uf) => {
      // store the id of the node this unifactor connects to
      const unique_node = uf.vars_id[0];
      // subset array of factors: the other factors connected to this unique node
      // const f_neighbors_of_uf = d_factors.filter(
      //   // to be in the club of the neighbors of uf, a factor must not be uf itself AND must be connected to the 'unique node' of the unifactor
      //   (f) => (f.factor_id !== uf.factor_id) && f.vars_id.includes(unique_node)
      // ); 
      const f_neighbors_of_uf = d_obj_marginals[unique_node].factor_set.filter(fid => fid !== uf.factor_id); // and filter out the uf
      // main case: 
      if (f_neighbors_of_uf.length > 0) {
        // if there are neighbors factors, the unary factor position must be placed
        // at the biggest angle gap.

        // Get relative orientation of the neighbor factors (take the node as the center)
        // TODO: fix issue if neighbor has no dot_factor_position field
        const thetas = f_neighbors_of_uf
          .map((f_neighbor_id) =>
            Math.atan2(
              // f_neighbor.dot_factor_position.y - uf.vars[0].mean.y,
              // f_neighbor.dot_factor_position.x - uf.vars[0].mean.x
              d_obj_factors[f_neighbor_id].dot_factor_position.y - d_obj_marginals[uf.vars_id[0]].mean.y,
              d_obj_factors[f_neighbor_id].dot_factor_position.x - d_obj_marginals[uf.vars_id[0]].mean.x
            )
          )
          .sort((a, b) => a - b); // mandatory sorting

        // Find the biggest gap in the relative orientations disposition of the neighbor factors
        const thetas_2pi = thetas.map((t) => t - thetas[0]);
        const dthetas2 = thetas_2pi.map((n, i) => {
          if (i !== thetas_2pi.length - 1) {
            return thetas_2pi[i + 1] - n;
          } else return 2 * Math.PI - n;
        });
        // compute the unifactor orientation angle (wrt node) in the middle of the biggest gap
        const idx_max = indexOfMax(dthetas2);
        const theta_unary = ecpi(thetas[idx_max] + dthetas2[idx_max] / 2);

        // distance of the factor wrt the vertex.
        const squares_distances = f_neighbors_of_uf.map(
          (nf) =>
            // (nf.dot_factor_position.y - uf.vars[0].mean.y) ** 2 +
            // (nf.dot_factor_position.x - uf.vars[0].mean.x) ** 2
            (d_obj_factors[nf].dot_factor_position.y - d_obj_marginals[uf.vars_id[0]].mean.y) ** 2 +
            (d_obj_factors[nf].dot_factor_position.x - d_obj_marginals[uf.vars_id[0]].mean.x) ** 2
        );
        const u_distance = Math.sqrt(
          Math.min(25, Math.max(...squares_distances))
        );
        // TODO: place the hard-coded 5^2=25 in globalUI (same as the previous 5)

        // position of factor dot infered from polar coordinates
        const new_uf_position = {
          x: d_obj_marginals[uf.vars_id[0]].mean.x + u_distance * Math.cos(theta_unary),
          y: d_obj_marginals[uf.vars_id[0]].mean.y + u_distance * Math.sin(theta_unary),
        };
        // giving new position
        uf.dot_factor_position = new_uf_position;
      } else {
        // corner case if this unifactor's has no other factor connected
        // (= an isolated node with a single factor)
        const theta_unary = Math.PI / 2;
        const u_distance = 5;
        const new_uf_position = {
          x: d_obj_marginals[uf.vars_id[0]].mean.x + u_distance * Math.cos(theta_unary),
          y: d_obj_marginals[uf.vars_id[0]].mean.y + u_distance * Math.sin(theta_unary),
        };
        // giving new position
        uf.dot_factor_position = new_uf_position;
      }
      // also add it in d_obj_factors
      d_obj_factors[uf.factor_id].dot_factor_position=uf.dot_factor_position;
    });
  
}

function add_associative_map(graph){
  // each marginal gets a map to it's separator set to the factor(s)
  // each marginal gets also a map from its factor_set to the nodes associated with 

  graph.marginals.forEach(m=>{m.factor_set=[];m.separator_set=[]}); // create empty arrays
  
  // factor_set in each marginal
  graph.factors.forEach(f=> f.vars_id.forEach( 
      node_var => {
                     graph.marginals.find( m => m.var_id===node_var).factor_set.push(f.factor_id);
                  }
      )                                                            
  );
  // fill separator_set in each marginal
  graph.marginals.forEach( m=> m.factor_set)
  

}

// TODO : deal with this
function estimation_query_last_pose(an_agent_estimation) {
  an_agent_estimation.last_pose.state =
    an_agent_estimation.graph.marginals.filter(
      (marginal) =>
        marginal.var_id == an_agent_estimation.last_pose.last_pose_id
    )[0].mean;
}

function sqDist(v1, v2) {
  return (
    Math.pow(v1.mean.x - v2.mean.x, 2) + Math.pow(v1.mean.y - v2.mean.y, 2)
  );
}

function objectify_marginals(marginals){
  // Expected result: object where the 'var_id' are the keys
  // Input array of where each element is an object.
  //  'var_id' is one field of each element
  
  // Procedure: isolate the var_id of each object using
  // the spread/rest operator
  return marginals
          .reduce(
            (acc,marginal)=>
              {
                const {var_id, ...rest_of_object} = marginal;
                acc[var_id]=rest_of_object;
                return acc;
              }
          ,{}
          );
}

function objectify_factors(factors){
  // Expected result: object where the 'factor_id' are the keys
  // Input array of where each element is an object.
  //  'factor_id' is one field of each element
  
  // Procedure: isolate the factor_id of each object using
  // the spread/rest operator
  return factors
          .reduce(
            (acc,factor)=>
              {
                const {factor_id, ...rest_of_object} = factor;
                acc[factor_id]=rest_of_object;
                return acc;
              }
          ,{}
          );
}

function compute_factor_set({marginals,obj_marginals,factors}){
  // require loop over the factors, and another over the marginals
  factors.forEach(
    f=> f.vars_id.forEach(
      node_id=> {
        // initialise empty array if field 'factor_set' doesnt exist yet
        if(obj_marginals[node_id].factor_set == undefined){
          obj_marginals[node_id].factor_set= [];
        }
        obj_marginals[node_id].factor_set.push(f.factor_id)
      }
    )
  )
  // we added the 'factor_set' in the object representation of marginals
  // now add it to the array marginals
  marginals.forEach(node => node.factor_set=obj_marginals[node.var_id].factor_set);
}

function compute_separator(d_marginal){
  separator_set=[];
  d_marginal.factor_set.forEach(
    factor_id => 
    separator_set.push(
      d3.select(`#${factor_id}`)
        .datum()
        .vars_id
        .filter(node_id => node_id !== d_marginal.var_id )
    )
  )
  return separator_set;
}
