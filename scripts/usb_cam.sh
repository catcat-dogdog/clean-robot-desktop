gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch usb_cam usb_cam-test.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; rosrun rqt_image_view rqt_image_view; exec bash"' \

