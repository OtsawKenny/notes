ros2 launch rmf_demos_gz kenny.launch.xml

# How the splitting loks like
-rmf_demos_maps
-rmf_demos_assets 
-- GZ_SIM_RESOURCE_PATH
-rmf_demos_gz 
--rmf_demos, kenny.launch.xml
--simulation.launch.xml
-rmf_demos
-- common.launch.xml,
--- rmf_traffic_schedule 
--- rmf_traffic_blockade 
--- building_map_server
--- visualisation.launch.xml
--- door_supervisor
--- lift_supervisor
--- rmf_task_dispatcher
--- rmf_reservation_node
-- kenny.launch.xml
--- common.launch.xml
--- fleet_adapter, requires the rmf_demos_map navgraph and rmf_demos config ( fleet configs)



## RMF Tutorial
1- Traffic editor, Building a Map
https://osrf.github.io/ros2multirobotbook/traffic-editor.html

Useful docs here: rmf/rmf_traffic_editor/README.md

1a Adding a level 
1b Adding a vertex
1c Adding a measurement
1d Adding a wall
1e Adding a floor
1f Adding a door
1g Adding a traffic lane
1e Adding fiducials
1g Adding environment assests

Notes: 
as part of the rmf_demo_maps package , when u colcon build, 2 keys exec are run
- transform the .building.yaml ->  .world & models
- transform the .building.yaml -> nav_graph 

```
ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}
ros2 run rmf_building_map_tools building_map_generator nav ${map_path} ${output_nav_graphs_dir}
```

2- Simulation , Running a sim 

Important packages
-rmf_demos_maps
-rmf_demos_assets 
-- GZ_SIM_RESOURCE_PATH
-rmf_demos_gz 
--rmf_demos, kenny.launch.xml
--simulation.launch.xml
-rmf_demos
-- common.launch.xml,
--- rmf_traffic_schedule 
--- rmf_traffic_blockade 
--- building_map_server
--- visualisation.launch.xml
--- door_supervisor
--- lift_supervisor
--- rmf_task_dispatcher
--- rmf_reservation_node
-- kenny.launch.xml
--- common.launch.xml
--- fleet_adapter, requires the rmf_demos_map navgraph and rmf_demos config ( fleet configs)



- In rmf_gz, just ensure *simulation_launch is enabled to check gazebo simulation is ok
```bash
ros2 launch rmf_demos_gz kenny.launch.xml 
```

## Running with Api server
### Start the backend API server via `docker` with host network access, using the default configuration. The API server will be accessible at `localhost:8000` by default.
```bash
docker run \
  --network host \
  -it \
  -e ROS_DOMAIN_ID=42 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ghcr.io/open-rmf/rmf-web/api-server:latest
```

echo $ROS_DOMAIN_ID = 42
echo $RMW_IMPLEMENTATION =rmw_cyclonedds_cpp

Docs: http://localhost:8000/openapi.json

### Running the Gazebo Sim along with the server_uri
```bash 
# In order to interact with the default configuration of the web application, the `server_uri` launch parameter will need to be changed to `ws://localhost:8000/_internal`, for example,
ros2 launch rmf_demos_gz kenny.launch.xml server_uri:="ws://localhost:8000/_internal"

```

### Start the frontend dashboard via `docker` with host network access, using the default configuration. The dashboard will be accessible at `localhost:3000` by default.

```bash
docker run \
  --network host -it --rm \
  -e RMF_SERVER_URL=http://localhost:8000 \
  -e TRAJECTORY_SERVER_URL=ws://localhost:8006 \
  ghcr.io/open-rmf/rmf-web/demo-dashboard:jazzy-nightly
```

### Dispatching task from the package
```
ros2 run rmf_demos_tasks  dispatch_go_to_place -F tinyRobot -R tinyRobot1 -p lounge -o 105 --use_sim_time
ros2 run rmf_demos_tasks dispatch_patrol -F tinyRobot -R tinyRobot1 -p lounge -n 2 --use_sim_time
ros2 run rmf_demos_tasks dispatch_action -F tinyRobot -R tinyRobot1 -a teleop -s coe --use_sim_time


ros2 run rmf_demos_tasks  dispatch_go_to_place -F tinyRobot -R tinyRobot1 -p point_show_room -o 105 --use_sim_time
ros2 run rmf_demos_tasks dispatch_action -F tinyRobot -R tinyRobot1 -a teleop -s coe --use_sim_time

```
