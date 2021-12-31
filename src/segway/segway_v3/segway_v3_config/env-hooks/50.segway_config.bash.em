@[if DEVELSPACE]@
. "@(CMAKE_CURRENT_SOURCE_DIR)/segway_config.bash"
@[else]@
if [ -z "$CATKIN_ENV_HOOK_WORKSPACE" ]; then
  CATKIN_ENV_HOOK_WORKSPACE="@(CMAKE_INSTALL_PREFIX)"
fi
. "$CATKIN_ENV_HOOK_WORKSPACE/share/segway_v3_config/segway_config.bash"
@[end if]@
