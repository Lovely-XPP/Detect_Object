##sitl_gazebo
gnome-terminal --window -e 'bash -c "sleep 1; roslaunch px4_realsense2_bridge bridge_mavros.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_cmd send_cmd; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_cmd set_mode; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_cmd set_cmd; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun detect_object detect_balloon simulation:=false save_video:=true; exec bash"' \
