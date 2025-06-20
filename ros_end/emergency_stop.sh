rosservice call /arm_emergency_stop
rosnode kill /wpb_home_grab_action
rosservice call /emergency_stop