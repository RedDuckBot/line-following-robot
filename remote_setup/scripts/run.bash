#--device=/dev/input/js0 \
 docker run  -it --rm \
        --env DISPLAY=$DISPLAY \
        --network host \
        --name remote_bot \
        --volume ./ros2_ws:/remote_bot/ros2_ws \
        remote_bot
