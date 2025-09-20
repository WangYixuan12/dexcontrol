from dexcontrol.config.vega import get_vega_config
from dexcontrol.robot import Robot

configs = get_vega_config()
configs.sensors.lidar.enable = True
robot = Robot(configs=configs)

scan_data = robot.sensors.lidar.get_obs()

ranges = scan_data["ranges"]
angles = scan_data["angles"]
qualities = scan_data.get("qualities")

print(f"Ranges: {ranges}")
print(f"Angles: {angles}")
print(f"Qualities: {qualities}")

robot.shutdown()
