gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch zoo_bringup bringup_w2a.launch; exec bash"' \
--tab -e 'bash -c "sleep 7; roslaunch zoo_control keyboard_teleop.launch; exec bash"' \

