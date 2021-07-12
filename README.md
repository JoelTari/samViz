# samViz

(Work in progress)

A visualization system for smoothing-and-mapping graphs, embedded in the web browser.

`samViz` display a factor graphs representation given by external processes through
the `MQTT` protocol.

## Supported OS

`GNU/Linux`

## Dependencies

- web-browser: tested on `chromium` and `firefox`
- `live-server`
- `mosquitto`

The examples are using the following `mqtt` libraries:

- for `Python` :
  ```
  pip3 install paho-mqtt
  ```
- for `C++` : install the `eMQTTv5` library ([link](https://github.com/X-Ryl669/eMQTT5)).

## Installation


```
git clone https://github.com/JoelTari/samViz
cd samViz
```

## Launch

### Launch mosquitto


Mosquitto must be launch with the following config `mosquitto.conf`:

```
allow_anonymous true
listener 1883
listener 9001
protocol websockets
```

If mosquitto is already launched by systemd, stop (and optionally disable) the
process with :

```
sudo systemctl stop mosquitto
sudo systemctl disable mosquitto # to prevent launch at the next reboot
```

Open a terminal and run:

```
cd samViz
mosquitto -c mosquitto.conf
```

Under this configuration, `mosquitto` starts a websocket
 
### Launch the web page

In an other terminal window:

 ```
 cd samViz
 live-server
 ```

The web page should appear in your default browser.

## Quick-Start Examples

### Command line

```
cd examples
mosquitto_pub -t default/graphs -m $(cat hello_graph.json)
```

### Python

```
cd examples
python3 graph_publish hello_graph.json
```

## Using `docker`

TODO.

## Licence

This work is provided under the terms of the EUPL v1.2 by AKKA Technologies.

## Maintainer

Joel Tari


