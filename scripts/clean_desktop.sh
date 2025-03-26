gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch zoo_bringup bringup_w2a.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch robot_slam navigation_carto_w2a.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch clean_desktop desktop_clean.launch ; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch usb_cam usb_cam-test.launch ; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch zoo_control keyboard_teleop.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch robot_slam view_nav.launch; exec bash"' \
