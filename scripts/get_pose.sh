gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch zoo_bringup bringup_w2a.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch zoo_robot get_pose_demo.launch; exec bash"' \

