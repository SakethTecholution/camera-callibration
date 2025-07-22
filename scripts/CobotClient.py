import requests
import json
import numpy as np
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


ip = "192.168.1.102"  # Server IP 
port = "8001"
timeout = 15  # Timeout in seconds

class CobotClient:
    def __init__(self, ip=ip, port=port, timeout=timeout):
        self.base_url = f"http://{ip}:{port}"
        self.timeout = timeout

    def get_current_joint_values(self):
        url = f"{self.base_url}/get_current_joint_values"
        return self.make_request('get', url)

    def get_current_pose(self):
        url = f"{self.base_url}/get_current_pose"
        return self.make_request('get', url)

    def move_to_joints(self, joint_angles):
        url = f"{self.base_url}/move_to_joints"
        return self.make_request('post', url, json=joint_angles)

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        url = f"{self.base_url}/move_to_pose"
        params = {
            "X": x,
            "Y": y,
            "Z": z,
            "A": roll,
            "B": pitch,
            "C": yaw
        }
        return self.make_request('post', url, params=params)

    def set_velocity(self, velocity):
        url = f"{self.base_url}/set_velocity"
        return self.make_request('post', url, json={"velocity": velocity})

    def set_acceleration(self, acceleration):
        url = f"{self.base_url}/set_acceleration"
        return self.make_request('post', url, json={"acceleration": acceleration})

    def stop_robot(self):
        url = f"{self.base_url}/stop_robot"
        return self.make_request('get', url)

    def set_object_coordinates(self, coordinates):
        url = f"{self.base_url}/set_object_coordinates"
        return self.make_request('post', url, json=coordinates)

    def get_home_info(self):
        url = f"{self.base_url}/"
        return self.make_request('get', url)

    def get_live_data(self):
        url = f"{self.base_url}/live_data"
        return self.make_request('get', url)

    def set_home_joints(self, joint_values):
        url = f"{self.base_url}/set_home_joints"
        return self.make_request('post', url, json=joint_values)

    def move_to_home(self):
        url = f"{self.base_url}/move_to_home"
        return self.make_request('get', url)

    def pause_robot(self):
        url = f"{self.base_url}/pause_robot"
        return self.make_request('get', url)

    def resume_robot(self):
        url = f"{self.base_url}/resume_robot"
        return self.make_request('get', url)

    def get_gripper_status(self, index=2):
        url = f"{self.base_url}/get_gripper_status/?index={index}"
        return self.make_request('get', url)

    def set_gripper_status(self, value, index=2):
        url = f"{self.base_url}/set_gripper_status/?index={index}&value={value}"
        return self.make_request('get', url)

    def get_current_rotm(self):
        url = f"{self.base_url}/get_current_rotm"
        response = self.make_request('get', url)
        if response and 'rot_matrix' in response:
            return np.array(response['rot_matrix'])
        return None

    def set_tcp_values(self, x, y, z, a, b, c, name):
        url = f"{self.base_url}/set_tcp_values"
        params = {
            "X": x,
            "Y": y,
            "Z": z,
            "A": a,
            "B": b,
            "C": c,
            "name": name
        }
        return self.make_request('post', url, params=params)

    def make_request(self, method, url, **kwargs):
        try:
            response = requests.request(method, url, timeout=self.timeout, **kwargs)
            response.raise_for_status()
            return self.extract_data(response)
        except requests.exceptions.Timeout:
            logger.error(f"Request to {url} timed out after {self.timeout} seconds.")
        except requests.exceptions.RequestException as e:
            logger.error(f"Request to {url} failed: {e}")
        return None

    def extract_data(self, response):
        try:
            return response.json()
        except json.JSONDecodeError:
            return response.text

# For Testing
if __name__ == "__main__":
    client = CobotClient()
    # print(client.get_current_joint_values())
    # print(client.get_current_rotm())
    print(client.set_tcp_values(0, 0, 0, 0, 0, 0, "tcp1"))
    print(client.get_current_pose())
    rotm = client.get_current_rotm()
    print("Rotation Matrix:", rotm)
    print("here")
    # print(client.get_current_pose())
    # print(client.move_to_joints([90, 0, 90, 0, 90, 0]))
    # print(client.set_gripper_status(value=0))
    # print(client.move_to_pose(100, -400, 601, 98, -116, -174))
    # client.set_velocity(10)
    # client.set_acceleration(5)
    # client.stop_robot()
    # client.pause_robot()
    # client.resume_robot()