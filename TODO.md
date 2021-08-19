# TODO list (unordered)

- zoom add zoom-out limiter (scaleExtent)
- grid
- gifs in intro README
- one script launcher
- tooltip info: separator set for each vertex
- read optional 'shared' data on vertex
- auto max_scale instead of 0.2
- dockerfile
- zoom reset button in the nav bar or somewhere
- add zoom bounding-boxes info on the globalUI (when dealing with more than one graph) 
- graph of interest (GoI) -> adapt behavious of zoom_reset and declutter buttons to specific graph (or all)

- Fix visual representation of factors that have the same variable scope (currently they stack exactly on top of each which impedes  the visual)

- full example with GTSAM on M3500
- full example with G20 on M3500
- full example with minisam on M3500
- full example with ceres on M3500

- readme completion: clarify all the optionnal and required field in the graph.json entry
- induced graph
- bayes net based on ordering
- examples with MQTT communications in python
- examples with MQTT communications in C++
- C++ minimal SAM solver

- Have a nav bar: separate options specific to one graph or for all the graphs, then, even for one graph
- zoom slider in the nav bar
  some options are valid only for one panel
- nav bar option: allows for overlap
- color/highlight var/factor on hover, as well as factor/var neighbors 

- other panel: bar chart of graph error evolution
- other panel: spy() matrix for small problems
- position the graph elements based on repulsive force (will allow to represent more abstract problems)
- on-click behaviour on vertex : display covariance (z-index first)
- UI option in the nav bar (as well as reset to default options since there will be many)

## misc

- responsive layout (especially investigate consequences on svg aspect ratio)
- back-end websocket. Currently mosquitto daemon provides the websocket, however it would be beneficial
  to have our own websocket for various features, such as, saving the graphs in a database to replay it
  thanks to a timeline panel.
- mqtt: wait for a meta topic (the entity that publishes meta must wait for samViz to be ready to process)

- divide in modules
- vuejs icremental integration.

