gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch zoo_bringup bringup_w2a.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch robot_slam navigation_carto_w2a.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch robot_slam view_nav.launch; exec bash"' \

