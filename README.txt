run_navigation.webctl
avviare fino a planner, poi posizionare su rviz la posizione e goal

rosrun srrg2_navigation_2d_ros path_follower_app _pure_rotation_threshold:=0.785398 _rotation_reach_threshold:=0.19635 _translation_reach_threshold:=0.5 _tv_gain:=1 _rv_gain:=1 _map_frame_id:=map _base_link_frame_id:=base_link _cmd_vel_topic:=/Prog_Cmd_vel _path_topic:=/local_path _status_topic:=/path_follower_status

