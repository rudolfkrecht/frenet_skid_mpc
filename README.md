# frenet_skid_mpc

ros2 run frenet_skid_mpc frenet_skid_mpc_node \
  --ros-args --params-file $(ros2 pkg prefix frenet_skid_mpc)/share/frenet_skid_mpc/config/frenet_skid_mpc.yaml


ros2 run clipped_skid_p2p clipped_skid_p2p_node \
  --ros-args --params-file \
  ~/ros2_ws/src/clipped_skid_p2p/config/clipped_skid_p2p.yaml
