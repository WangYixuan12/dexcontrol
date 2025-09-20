from dexcontrol.robot import Robot
from dexcontrol.config.vega import get_vega_config

configs = get_vega_config()
# launch the subscribers for base camera data if enable=True
configs.sensors.base_left_camera.enable = True
configs.sensors.base_right_camera.enable = True
configs.sensors.base_front_camera.enable = True
configs.sensors.base_back_camera.enable = True
robot = Robot(configs=configs)

base_left_camera_image = robot.sensors.base_left_camera.get_obs()
base_right_camera_image = robot.sensors.base_right_camera.get_obs()
base_front_camera_image = robot.sensors.base_front_camera.get_obs()
base_back_camera_image = robot.sensors.base_back_camera.get_obs()

print(f'Left camera image shape: {base_left_camera_image.shape}')
print(f'Right camera image shape: {base_right_camera_image.shape}')
print(f'Front camera image shape: {base_front_camera_image.shape}')
print(f'Back camera image shape: {base_back_camera_image.shape}')

robot.shutdown()
