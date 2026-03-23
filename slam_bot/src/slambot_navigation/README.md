start action server: ros2 run slambot_navigation a_star_action_server

send command:

ros2 action send_goal /a_star slambot_navigation/action/AStar "{
goal: {
header: {frame_id: map},
pose: {
position: {x: 2.0, y: -2.0, z: 0.0},
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
}
}
}" --feedback
