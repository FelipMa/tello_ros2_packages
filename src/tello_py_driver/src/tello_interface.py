"""Library for interacting with DJI Ryze Tello drones.
"""

# coding=utf-8
import logging
import socket
import time
import threading
from typing import Optional, Union, Type, Dict
import cv2

threads_initialized = False
drones: Optional[dict] = {}
client_socket: socket.socket

class TelloException(Exception):
    pass


class Tello:
    """Python wrapper to interact with the Ryze Tello drone using the official Tello api.
    """
    # Send and receive commands, client socket
    RESPONSE_TIMEOUT = 0.5  # in seconds

    TELLO_IP = '192.168.10.1'  # Tello IP address

    # Video stream, server socket
    VS_UDP_IP = '0.0.0.0'
    VS_UDP_PORT = 11111

    CONTROL_UDP_PORT = 8889
    STATE_UDP_PORT = 8890

    # Constants for video settings
    BITRATE_AUTO = 0
    BITRATE_1MBPS = 1
    BITRATE_2MBPS = 2
    BITRATE_3MBPS = 3
    BITRATE_4MBPS = 4
    BITRATE_5MBPS = 5
    RESOLUTION_480P = 'low'
    RESOLUTION_720P = 'high'
    FPS_5 = 'low'
    FPS_15 = 'middle'
    FPS_30 = 'high'
    CAMERA_FORWARD = 0
    CAMERA_DOWNWARD = 1

    # Set up logger
    HANDLER = logging.StreamHandler()
    FORMATTER = logging.Formatter(
        '[%(levelname)s] %(filename)s - %(lineno)d - %(message)s')
    HANDLER.setFormatter(FORMATTER)

    LOGGER = logging.getLogger('tello_interface')
    LOGGER.addHandler(HANDLER)
    LOGGER.setLevel(logging.INFO)

    # Conversion functions for state protocol fields
    INT_STATE_FIELDS = (
        # Common entries
        'pitch', 'roll', 'yaw',
        'vgx', 'vgy', 'vgz',
        'templ', 'temph',
        'tof', 'h', 'bat', 'time'
    )
    FLOAT_STATE_FIELDS = ('baro', 'agx', 'agy', 'agz')

    state_field_converters: Dict[str, Union[Type[int], Type[float]]]
    state_field_converters = {key: int for key in INT_STATE_FIELDS}
    state_field_converters.update({key: float for key in FLOAT_STATE_FIELDS})

    network_interface: Optional[str] = None
    video_foward_port: int = 11112

    def __init__(self,
                 host=TELLO_IP,
                 network_interface: Optional[str] = None,
                 video_foward_port: int = 11112):
        
        Tello.network_interface = network_interface
        Tello.video_foward_port = video_foward_port

        global threads_initialized, client_socket, drones

        self.is_flying = False
        self.stream_on = False
        self.cv2_frame = None
        self.video_thread = None
        self.address = (host, Tello.CONTROL_UDP_PORT)
        self.retry_count = 3
        self.last_rc_control_timestamp = time.time()

        if not threads_initialized:
            # Build the UDP socket for sending commands, client_socket is a global variable
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if Tello.network_interface is not None:
                client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, Tello.network_interface.encode())
            client_socket.bind(("", Tello.CONTROL_UDP_PORT))

            # Run response UDP receiver on background
            response_receiver_thread = threading.Thread(target=Tello.udp_response_receiver)
            response_receiver_thread.daemon = True
            response_receiver_thread.start()

            # Run state UDP receiver on background
            state_receiver_thread = threading.Thread(target=Tello.udp_state_receiver)
            state_receiver_thread.daemon = True
            state_receiver_thread.start()

            threads_initialized = True

        drones[host] = {'responses': [], 'state': {}}

        self.LOGGER.info("Tello instance was initialized. Host: '{}'. Port: '{}'.".format(
            host, Tello.CONTROL_UDP_PORT))

    def get_own_udp_object(self):
        """Get own object from the global drones dict. This object is filled
        with responses and state information by the receiver threads.
        """
        global drones

        host = self.address[0]
        return drones[host]

    @staticmethod
    def udp_response_receiver():
        """Setup drone UDP receiver. This method listens for responses of Tello.
        Must be run from a background thread in order to not block the main thread.
        """
        while True:
            try:
                data, address = client_socket.recvfrom(1024)

                recAddress: str = address[0]
                Tello.LOGGER.debug(
                    'Data received from {} at client_socket'.format(recAddress))

                if recAddress not in drones:
                    continue

                drones[recAddress]['responses'].append(data)

            except Exception as e:
                Tello.LOGGER.error(e)
                break

    @staticmethod
    def udp_state_receiver():
        """Setup state UDP receiver. This method listens for state information from
        Tello. Must be run from a background thread in order to not block
        the main thread.
        """
        state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if Tello.network_interface is not None:
            state_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, Tello.network_interface.encode())
        state_socket.bind(("", Tello.STATE_UDP_PORT))

        while True:
            try:
                data, address = state_socket.recvfrom(1024)

                address = address[0]
                Tello.LOGGER.debug(
                    'Data received from {} at state_socket'.format(address))

                if address not in drones:
                    continue

                data = data.decode('ASCII')
                drones[address]['state'] = Tello.parse_state(data)

            except Exception as e:
                Tello.LOGGER.error(e)
                break

    @staticmethod
    def parse_state(state: str) -> Dict[str, Union[int, float, str]]:
        """Parse a state line to a dictionary
        """
        state = state.strip()
        Tello.LOGGER.debug('Raw state data: {}'.format(state))

        if state == 'ok':
            return {}

        state_dict = {}
        for field in state.split(';'):
            split = field.split(':')
            if len(split) < 2:
                continue

            key = split[0]
            value: Union[int, float, str] = split[1]

            if key in Tello.state_field_converters:
                num_type = Tello.state_field_converters[key]
                try:
                    value = num_type(value)
                except ValueError as e:
                    Tello.LOGGER.debug('Error parsing state value for {}: {} to {}'
                                       .format(key, value, num_type))
                    Tello.LOGGER.error(e)
                    continue

            state_dict[key] = value

        return state_dict

    def get_current_state(self) -> dict:
        """Call this function to attain the state of the Tello. Returns a dict
        with all fields.
        """
        return self.get_own_udp_object()['state']

    def get_state_field(self, key: str):
        """Get a specific sate field by name.
        """
        state = self.get_current_state()

        if key in state:
            return state[key]
        else:
            raise TelloException(
                'Could not get state property: {}'.format(key))

    def get_pitch(self) -> int:
        """Get pitch in degree
        Returns:
            int: pitch in degree
        """
        return self.get_state_field('pitch')

    def get_roll(self) -> int:
        """Get roll in degree
        Returns:
            int: roll in degree
        """
        return self.get_state_field('roll')

    def get_yaw(self) -> int:
        """Get yaw in degree
        Returns:
            int: yaw in degree
        """
        return self.get_state_field('yaw')

    def get_speed_x(self) -> int:
        """X-Axis Speed
        Returns:
            int: speed
        """
        return self.get_state_field('vgx')

    def get_speed_y(self) -> int:
        """Y-Axis Speed
        Returns:
            int: speed
        """
        return self.get_state_field('vgy')

    def get_speed_z(self) -> int:
        """Z-Axis Speed
        Returns:
            int: speed
        """
        return self.get_state_field('vgz')

    def get_acceleration_x(self) -> float:
        """X-Axis Acceleration
        Returns:
            float: acceleration
        """
        return self.get_state_field('agx')

    def get_acceleration_y(self) -> float:
        """Y-Axis Acceleration
        Returns:
            float: acceleration
        """
        return self.get_state_field('agy')

    def get_acceleration_z(self) -> float:
        """Z-Axis Acceleration
        Returns:
            float: acceleration
        """
        return self.get_state_field('agz')

    def get_lowest_temperature(self) -> int:
        """Get lowest temperature
        Returns:
            int: lowest temperature (°C)
        """
        return self.get_state_field('templ')

    def get_highest_temperature(self) -> int:
        """Get highest temperature
        Returns:
            float: highest temperature (°C)
        """
        return self.get_state_field('temph')

    def get_height(self) -> int:
        """Get current height in cm
        Returns:
            int: height in cm
        """
        return self.get_state_field('h')

    def get_distance_tof(self) -> int:
        """Get current distance value from TOF in cm
        Returns:
            int: TOF distance in cm
        """
        return self.get_state_field('tof')

    def get_barometer(self) -> float:
        """Get current barometer measurement in cm
        Returns:
            int: barometer measurement in cm
        """
        return self.get_state_field('baro')

    def get_flight_time(self) -> int:
        """Get the time the motors have been active in seconds
        Returns:
            int: flight time in s
        """
        return self.get_state_field('time')

    def get_battery(self) -> int:
        """Get current battery percentage
        Returns:
            int: 0-100
        """
        return self.get_state_field('bat')

    def send_command_without_return(self, command: str):
        """Send command to Tello without expecting a response.
        """
        self.LOGGER.info("Send command: '{}' (no response expected)".format(command))
        client_socket.sendto(command.encode('utf-8'), self.address)

    def send_command_with_return_non_blocking(self, command: str, timeout: int = RESPONSE_TIMEOUT) -> str:
        """Send command to Tello and wait for its response. Non blocking version.
        Return:
            bool/str: str with response text on success, False when unsuccessfull.
        """
        self.LOGGER.info("Send command: '{}'".format(command))
        client_socket.sendto(command.encode('utf-8'), self.address)
        timestamp = time.time()

        responses = self.get_own_udp_object()['responses']

        while not responses:
            if time.time() - timestamp > timeout:
                response = "Response for '{}' timed out".format(command)
                self.LOGGER.info(response)
                return response
            time.sleep(0.1)  # Sleep during send command

        first_response = responses.pop(0)  # first data from socket

        try:
            response = first_response.decode("utf-8")
        except UnicodeDecodeError as e:
            self.LOGGER.error(e)
            return "response decode error"

        response = response.rstrip("\r\n")
        self.LOGGER.info(response)
        return response

    def send_control_command_non_blocking(self, command: str, timeout: int = RESPONSE_TIMEOUT) -> bool:
        """Send control command to Tello and wait for its response. Non blocking version.
        """
        response = ""

        response = self.send_command_with_return_non_blocking(
            command, timeout=timeout)

        if 'ok' in response.lower():
            return True

        return False

    def send_read_command(self, command: str) -> str:
        """Send given command to Tello and wait for its response.
        """

        response = self.send_command_with_return_non_blocking(command)

        try:
            response = str(response)
        except TypeError as e:
            self.LOGGER.error(e)

        if any(word in response for word in ('error', 'ERROR', 'False')):
            self.raise_result_error(command, response)
            return "Error: this code should never be reached"

        return response

    def send_read_command_int(self, command: str) -> int:
        """Send given command to Tello and wait for its response.
        Parses the response to an integer
        """
        response = self.send_read_command(command)
        return int(response)

    def send_read_command_float(self, command: str) -> float:
        """Send given command to Tello and wait for its response.
        Parses the response to an integer
        """
        response = self.send_read_command(command)
        return float(response)

    def raise_result_error(self, command: str, response: str) -> bool:
        """Used to raise an error after an unsuccessful command
        """
        tries = self.retry_count
        raise TelloException("Command '{}' was unsuccessful for {} tries. Latest response:\n'{}'"
                             .format(command, tries, response))

    def connect(self, wait_for_state=True):
        """Enter SDK mode. Call this before any of the control functions.
        """
        for i in range(self.retry_count):
            res = self.send_control_command_non_blocking("command")
            if res is True:
                break
            time.sleep(0.2) # Sleep between attempts

        if res is False:
            raise TelloException(
                "Tello not connected. Tried {} times".format(self.retry_count))

        if wait_for_state:
            REPS = 20
            for i in range(REPS):
                if self.get_current_state():
                    t = i / REPS  # in seconds
                    Tello.LOGGER.debug(
                        "'.connect()' received first state packet after {} seconds".format(t))
                    break
                time.sleep(1 / REPS)

            if not self.get_current_state():
                raise TelloException(
                    'Did not receive a state packet from the Tello')

    def takeoff(self):
        """Automatic takeoff.
        """
        # Something it takes a lot of time to take off and return a succesful takeoff.
        # So we better wait. Otherwise, it would give us an error on the following calls.
        self.send_control_command_non_blocking("takeoff")
        self.is_flying = True

    def land(self):
        """Automatic landing.
        """
        self.send_control_command_non_blocking("land")
        self.is_flying = False

    def streamon(self):
        """Turn on video streaming.

        !!! Note:
            If the response is 'Unknown command' you have to update the Tello
            firmware. This can be done using the official Tello app.
        """
        self.send_control_command_non_blocking("streamon")

        self.stream_on = True

        if self.cv2_frame is None:
            self.foward_udp_video_thread = threading.Thread(target=self.foward_udp_video)
            self.foward_udp_video_thread.daemon = True
            self.foward_udp_video_thread.start()

            self.cv2_video_capture_thread = threading.Thread(target=self.cv2_video_capture)
            self.cv2_video_capture_thread.daemon = True
            self.cv2_video_capture_thread.start()

    def streamoff(self):
        """Turn off video streaming.
        """
        self.send_control_command_non_blocking("streamoff")
        self.stream_on = False

        if self.video_thread is not None:
            self.video_thread.join()
            self.video_thread = None

    def emergency(self):
        """Stop all motors immediately.
        """
        self.send_control_command_non_blocking("emergency")
        self.is_flying = False

    def move(self, direction: str, x: int):
        """Tello fly up, down, left, right, forward or back with distance x cm.
        Users would normally call one of the move_x functions instead.
        Arguments:
            direction: up, down, left, right, forward or back
            x: 20-500
        """
        self.send_control_command_non_blocking("{} {}".format(direction, x))

    def move_up(self, x: int):
        """Fly x cm up.
        Arguments:
            x: 20-500
        """
        self.move("up", x)

    def move_down(self, x: int):
        """Fly x cm down.
        Arguments:
            x: 20-500
        """
        self.move("down", x)

    def move_left(self, x: int):
        """Fly x cm left.
        Arguments:
            x: 20-500
        """
        self.move("left", x)

    def move_right(self, x: int):
        """Fly x cm right.
        Arguments:
            x: 20-500
        """
        self.move("right", x)

    def move_forward(self, x: int):
        """Fly x cm forward.
        Arguments:
            x: 20-500
        """
        self.move("forward", x)

    def move_back(self, x: int):
        """Fly x cm backwards.
        Arguments:
            x: 20-500
        """
        self.move("back", x)

    def rotate_clockwise(self, x: int):
        """Rotate x degree clockwise.
        Arguments:
            x: 1-3600
        """
        self.send_control_command_non_blocking("cw {}".format(x))

    def rotate_counter_clockwise(self, x: int):
        """Rotate x degree counter-clockwise.
        Arguments:
            x: 1-3600
        """
        self.send_control_command_non_blocking("ccw {}".format(x))

    def flip(self, direction: str):
        """Do a flip maneuver.
        Users would normally call one of the flip_x functions instead.
        Arguments:
            direction: l (left), r (right), f (forward) or b (back)
        """
        self.send_control_command_non_blocking("flip {}".format(direction))

    def flip_left(self):
        """Flip to the left.
        """
        self.flip("l")

    def flip_right(self):
        """Flip to the right.
        """
        self.flip("r")

    def flip_forward(self):
        """Flip forward.
        """
        self.flip("f")

    def flip_back(self):
        """Flip backwards.
        """
        self.flip("b")

    def go_xyz_speed(self, x: int, y: int, z: int, speed: int):
        """Fly to x y z relative to the current position.
        Speed defines the traveling speed in cm/s.
        Arguments:
            x: -500-500
            y: -500-500
            z: -500-500
            speed: 10-100
        """
        cmd = 'go {} {} {} {}'.format(x, y, z, speed)
        self.send_control_command_non_blocking(cmd)

    def curve_xyz_speed(self, x1: int, y1: int, z1: int, x2: int, y2: int, z2: int, speed: int):
        """Fly to x2 y2 z2 in a curve via x2 y2 z2. Speed defines the traveling speed in cm/s.

        - Both points are relative to the current position
        - The current position and both points must form a circle arc.
        - If the arc radius is not within the range of 0.5-10 meters, it raises an Exception
        - x1/x2, y1/y2, z1/z2 can't both be between -20-20 at the same time, but can both be 0.

        Arguments:
            x1: -500-500
            x2: -500-500
            y1: -500-500
            y2: -500-500
            z1: -500-500
            z2: -500-500
            speed: 10-60
        """
        cmd = 'curve {} {} {} {} {} {} {}'.format(
            x1, y1, z1, x2, y2, z2, speed)
        self.send_control_command_non_blocking(cmd)

    def set_speed(self, x: int):
        """Set speed to x cm/s.
        Arguments:
            x: 10-100
        """
        self.send_control_command_non_blocking("speed {}".format(x))

    def send_rc_control(self, left_right_velocity: int, forward_backward_velocity: int, up_down_velocity: int,
                        yaw_velocity: int):
        """Send RC control via four channels.
        Arguments:
            left_right_velocity: -100~100 (left/right)
            forward_backward_velocity: -100~100 (forward/backward)
            up_down_velocity: -100~100 (up/down)
            yaw_velocity: -100~100 (yaw)
        """
        def clamp100(x: int) -> int:
            return max(-100, min(100, x))

        if time.time() - self.last_rc_control_timestamp > 0.01:
            self.last_rc_control_timestamp = time.time()
            cmd = 'rc {} {} {} {}'.format(
                clamp100(left_right_velocity),
                clamp100(forward_backward_velocity),
                clamp100(up_down_velocity),
                clamp100(yaw_velocity)
            )
            self.send_control_command_non_blocking(cmd)

    def query_speed(self) -> int:
        """Query speed setting (cm/s)
        Returns:
            int: 1-100
        """
        return self.send_read_command_int('speed?')

    def query_battery(self) -> int:
        """Get current battery percentage via a query command
        Using get_battery is usually faster
        Returns:
            int: 0-100 in %
        """
        return self.send_read_command_int('battery?')

    def query_flight_time(self) -> int:
        """Query current fly time (s).
        Using get_flight_time is usually faster.
        Returns:
            int: Seconds elapsed during flight.
        """
        return self.send_read_command_int('time?')

    def query_height(self) -> int:
        """Get height in cm via a query command.
        Using get_height is usually faster
        Returns:
            int: 0-3000
        """
        return self.send_read_command_int('height?')

    def query_temperature(self) -> int:
        """Query temperature (°C).
        Using get_temperature is usually faster.
        Returns:
            int: 0-90
        """
        return self.send_read_command_int('temp?')

    def query_attitude(self) -> dict:
        """Query IMU attitude data.
        Using get_pitch, get_roll and get_yaw is usually faster.
        Returns:
            {'pitch': int, 'roll': int, 'yaw': int}
        """
        response = self.send_read_command('attitude?')
        return Tello.parse_state(response)

    def query_barometer(self) -> int:
        """Get barometer value (cm)
        Using get_barometer is usually faster.
        Returns:
            int: 0-100
        """
        baro = self.send_read_command_int('baro?')
        return baro * 100

    def query_distance_tof(self) -> float:
        """Get distance value from TOF (cm)
        Using get_distance_tof is usually faster.
        Returns:
            float: 30-1000
        """
        # example response: 801mm
        tof = self.send_read_command('tof?')
        return int(tof[:-2]) / 10

    def query_wifi_signal_noise_ratio(self) -> str:
        """Get Wi-Fi SNR
        Returns:
            str: snr
        """
        return self.send_read_command('wifi?')

    def end(self):
        """Call this method when you want to end the tello object
        """
        try:
            if self.is_flying:
                self.land()
            if self.stream_on:
                self.streamoff()
        except TelloException:
            pass

        if self.video_thread is not None:
            self.video_thread.join()
            self.video_thread = None

        host = self.address[0]
        if host in drones:
            del drones[host]

    def __del__(self):
        self.end()

    # check if this works later
    def get_frame_read(self):
        """Return last frame read from the drone.
        """

        return self.cv2_frame
    
    def foward_udp_video(self):
        """Forward udp video feed from drone to localhost.
        """
        video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if Tello.network_interface is not None:
            video_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, Tello.network_interface.encode())
        video_socket.bind(("", Tello.VS_UDP_PORT))

        while True:
            data, address = video_socket.recvfrom(2048)
            if data:
                foward_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                foward_socket.sendto(data, ("localhost", Tello.video_foward_port))
                foward_socket.close()

    def get_foward_udp_video_address(self) -> str:
        address_schema = 'udp://@{ip}:{port}'
        address = address_schema.format(ip="localhost", port=Tello.video_foward_port)
        return address

    def cv2_video_capture(self):
        """Get frame received in udp port with open cv.
        """
        while True:
            cap = cv2.VideoCapture(self.get_foward_udp_video_address())
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    Tello.LOGGER.error("Image not received, trying again in five seconds. Please check if the drone is connected")
                    time.sleep(5)
                    continue
                self.cv2_frame = frame

            cap.release()
            cv2.destroyAllWindows()
            Tello.LOGGER.error("OpenCV lost connection to drone")
            time.sleep(1)
        