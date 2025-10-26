import numpy as np
import time
from pathlib import Path
import tyro

from yixuan_utilities.hdf5_utils import save_dict_to_hdf5

import pyzed.sl as sl
from dexcontrol.robot import Robot


class DemoApp:
    def __init__(self):
        # Create a Camera object
        self.zed = sl.Camera()
        self.bot = Robot()

        # pose of looking down
        head_pos = np.array([-0.01, -0.02078687, 0.00514872])
        torso_pos = np.array([7.7998763e-01,  1.5692255e+00, -3.4906584e-04])
        left_arm_pos = np.array([0.81562376, 0.3355099, 0.5667102, -2.0946028, 1.270783, -0.01205499, -0.00896401])
        right_arm_pos = np.array([ 0.23443438, -0.29044896,  0.04472581, -2.183742,   -1.2762475,  -0.03269351, -0.04249527])

        self.bot.set_joint_pos(
            {
                "head": head_pos,
                "torso": torso_pos,
                "left_arm": left_arm_pos,
                "right_arm": right_arm_pos,
            },
            wait_time=5.0,
        )

        # Create a dictionary to store the episode data
        self.episode_data = {
            "color": [],
            "depth": [],
            "intrinsics": [],
            "extrinsics": [],
            "confidence": [],
            "joints": {},
        }
        self.save_dir = Path("data/zed")
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.episode_id = len(list(self.save_dir.glob("*.hdf5")))

    def save_episode_data(self):
        for key in self.episode_data:
            if isinstance(self.episode_data[key], list):
                self.episode_data[key] = np.stack(self.episode_data[key])
            elif isinstance(self.episode_data[key], dict):
                for sub_key in self.episode_data[key].keys():
                    self.episode_data[key][sub_key] = np.stack(self.episode_data[key][sub_key])
        rgb_h, rgb_w = self.episode_data["color"].shape[1:3]
        depth_h, depth_w = self.episode_data["depth"].shape[1:3]
        confidence_h, confidence_w = self.episode_data["confidence"].shape[1:3]
        hdf5_config = {
            "color": {
                "chunks": (1, rgb_h, rgb_w, 3),
                "dtype": "uint8",
            },
            "depth": {
                "chunks": (1, depth_h, depth_w, 1),
                "dtype": "uint16",
            },
            "confidence": {
                "chunks": (1, confidence_h, confidence_w, 1),
                "dtype": "uint8",
            },
            "intrinsics": {},
            "extrinsics": {},
        }
        save_path = str(self.save_dir / f"episode_{self.episode_id}.hdf5")
        save_dict_to_hdf5(self.episode_data, hdf5_config, save_path)
        print(f"Saved episode {self.episode_id} to {save_path}")
        self.episode_id += 1
        self.episode_data = {
            "color": [],
            "depth": [],
            "intrinsics": [],
            "extrinsics": [],
            "confidence": [],
            "joints": {},
        }

    def connect_to_device(self):
        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.AUTO # Use HD720 or HD1200 video mode (default fps: 60)
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP # Use a right-handed Y-up coordinate system
        init_params.coordinate_units = sl.UNIT.METER  # Set units in meters
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : "+repr(err)+". Exit program.")
            exit()


        # Enable positional tracking with default parameters
        py_transform = sl.Transform()  # First create a Transform object for TrackingParameters object
        tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
        err = self.zed.enable_positional_tracking(tracking_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            print("Enable positional tracking : "+repr(err)+". Exit program.")
            self.zed.close()
            exit()

    def get_intrinsic_mat_from_coeffs(self, coeffs):
        return np.array([[coeffs.fx,         0, coeffs.tx],
                         [        0, coeffs.fy, coeffs.ty],
                         [        0,         0,         1]])

    def start_processing_stream(self):
        # Track the camera position during 1000 frames
        runtime_parameters = sl.RuntimeParameters()
        cam_info = self.zed.get_camera_information()
        calib = cam_info.camera_configuration.calibration_parameters  # rectified pair

        left_K = np.array([[calib.left_cam.fx, 0, calib.left_cam.cx],
                           [0, calib.left_cam.fy, calib.left_cam.cy],
                           [0, 0, 1]], dtype=np.float32)

        
        for _ in range(20):
            start_time = time.time()
            left_rgb_sl = sl.Mat()
            depth_sl = sl.Mat()
            confidence_sl = sl.Mat()
            zed_pose = sl.Pose()
            if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # Get the pose of the left eye
                self.zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
                
                # Display the translation and timestamp
                py_translation = sl.Translation()
                tx = zed_pose.get_translation(py_translation).get()[0]
                ty = zed_pose.get_translation(py_translation).get()[1]
                tz = zed_pose.get_translation(py_translation).get()[2]
                print("Translation: Tx: {0}, Ty: {1}, Tz {2}, Timestamp: {3}\n".format(tx, ty, tz, zed_pose.timestamp.get_milliseconds()))

                # Display the orientation quaternion
                py_orientation = sl.Orientation()
                ox = zed_pose.get_orientation(py_orientation).get()[0]
                oy = zed_pose.get_orientation(py_orientation).get()[1]
                oz = zed_pose.get_orientation(py_orientation).get()[2]
                ow = zed_pose.get_orientation(py_orientation).get()[3]
                print("Orientation: Ox: {0}, Oy: {1}, Oz {2}, Ow: {3}\n".format(ox, oy, oz, ow))
                
                # Convert to numpy
                intrinsic_mat = left_K
                t = np.array([tx, ty, tz])
                cam_mat = np.eye(4)
                py_rotation = sl.Rotation()
                for row in range(3):
                    for col in range(3):
                        cam_mat[row, col] = zed_pose.get_rotation_matrix(py_rotation)[row, col]
                cam_mat[:3, 3] = t
                cam_mat = cam_mat @ np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

                # Copy the newly arrived RGBD frame
                self.zed.retrieve_image(left_rgb_sl, sl.VIEW.LEFT)   # BGRA by default
                self.zed.retrieve_measure(depth_sl, sl.MEASURE.DEPTH)  # float32 meters
                self.zed.retrieve_measure(confidence_sl, sl.MEASURE.CONFIDENCE)  # float32 meters
                left_rgba = left_rgb_sl.get_data()     # HxWx4 uint8, LEFT
                depth   = depth_sl.get_data()        # HxW float32, aligned to LEFT
                confidence = confidence_sl.get_data().astype(np.uint8)
                left_rgb_np  = left_rgba[:, :, :3][:, :, ::-1].copy()

                joint_pos_dict = self.bot.get_joint_pos_dict(
                    component=["left_arm", "right_arm", "torso", "head"]
                )

                for key, value in joint_pos_dict.items():
                    if key not in self.episode_data["joints"].keys():
                        self.episode_data["joints"][key] = []
                    self.episode_data["joints"][key].append(value)
                self.episode_data["color"].append(left_rgb_np)
                self.episode_data["depth"].append((depth[..., None] * 1000).astype(np.uint16))
                self.episode_data["intrinsics"].append(intrinsic_mat)
                self.episode_data["extrinsics"].append(cam_mat)
                self.episode_data["confidence"].append(confidence[..., None])
            end_time = time.time()
            print(f"FPS: {1 / (end_time - start_time)}")
        self.save_episode_data()
        self.zed.close()

if __name__ == '__main__':
    app = DemoApp()
    app.connect_to_device()
    tyro.cli(app.start_processing_stream)
