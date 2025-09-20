from dexcontrol.config.vega import get_vega_config
from dexcontrol.robot import Robot

configs = get_vega_config()
configs.sensors.head_camera.enable = True
robot = Robot(configs=configs)

camera_data = robot.sensors.head_camera.get_obs(
    obs_keys=["left_rgb", "right_rgb", "depth"]
)

print(f"Left RGB image shape: {camera_data['left_rgb'].shape}")
print(f"Right RGB image shape: {camera_data['right_rgb'].shape}")
print(f"Depth image shape: {camera_data['depth'].shape}")

robot.shutdown()
