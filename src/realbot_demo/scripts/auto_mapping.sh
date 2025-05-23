rosparam set /move_base/global_costmap/width 5.0
rosparam set /move_base/global_costmap/height 5.0

rosrun explore_lite explore _robot_base_frame:=base_link \
                          _costmap_topic:=/map \
                          _planner_frequency:=1.0 \
                          _progress_timeout:=60.0 \
                          _potential_scale:=10.0 \
                          _min_frontier_size:=0.5 \
                          _visualize:=true

rosrun map_server map_saver -f map