#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Exercise the production node through a pseudo-terminal, without IMU hardware."""
import math
import os
import pty
import signal
import struct
import subprocess
import sys
import tempfile
import time
import tty

os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 80)
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger


def run(executable):
    rclpy.init()
    node = rclpy.create_node('check_witmotion_serial')
    samples = []
    subscription = node.create_subscription(Imu, '/data_raw', samples.append, qos_profile_sensor_data)
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    commands = bytearray()
    requested_rates = []
    requested_modes = []
    wrong_readback = False
    wrong_mode_readback = False
    calibration_mode = 0
    ignore_calibration_write = False
    ignore_calibration_read = False
    unlocked = False
    calibration_writes = []
    save_commands = []
    start_client = node.create_client(Trigger, '/witmotion_imu/start_magnetic_calibration')
    stop_client = node.create_client(Trigger, '/witmotion_imu/stop_magnetic_calibration')
    parameter_client = node.create_client(SetParameters, '/witmotion_imu/set_parameters')

    def respond_to_commands():
        nonlocal calibration_mode, unlocked
        try:
            commands.extend(os.read(master, 1024))
        except BlockingIOError:
            pass
        while len(commands) >= 5:
            command = bytes(commands[:5])
            del commands[:5]
            assert command[:2] == b'\xff\xaa', 'Unexpected command header'
            if command[2] == 0x69:
                assert command[3:] == b'\x88\xb5', 'Invalid unlock command'
                unlocked = True
            elif command[2] == 0x03:
                assert unlocked, 'Rate write was not unlocked'
                unlocked = False
                requested_rates.append(command[3] | command[4] << 8)
            elif command[2] == 0x24:
                assert unlocked and command[4] == 0, 'Mode write was not unlocked or malformed'
                unlocked = False
                assert command[3] in (0, 1), 'Unexpected attitude fusion mode'
                requested_modes.append(command[3])
            elif command[2] == 0x01:
                assert unlocked, 'Calibration write was not unlocked'
                unlocked = False
                value = command[3] | command[4] << 8
                assert value in (0, 7), 'Unexpected calibration mode'
                calibration_writes.append(value)
                if not ignore_calibration_write:
                    calibration_mode = value
            elif command[2] == 0x00:
                assert unlocked and command[3:] == bytes(2), 'Invalid save command'
                unlocked = False
                assert calibration_mode == 0, 'Settings saved before leaving calibration mode'
                save_commands.append(command)
            elif command[2] == 0x27:
                assert command[4] == 0 and requested_rates
                address = command[3]
                if address == 3:
                    code = requested_rates[-1] ^ int(wrong_readback)
                elif address == 0x24:
                    assert requested_modes, 'Mode read preceded configuration'
                    code = requested_modes[-1] ^ int(wrong_mode_readback)
                    os.write(master, b'\x55\x71\x03\x00' + struct.pack('<H', requested_rates[-1]) + bytes(14))
                else:
                    assert address == 1, 'Unexpected register read'
                    # A reply to another register must not satisfy calibration readback.
                    os.write(master, b'\x55\x71\x03\x00' + struct.pack('<H', requested_rates[-1]) + bytes(14))
                    if ignore_calibration_read:
                        continue
                    code = calibration_mode
                os.write(master, b'\x55\x71' + bytes([address, 0]) + struct.pack('<H', code) + bytes(14))
            else:
                raise AssertionError('Unexpected sensor command')
    process = None
    try:
        with tempfile.TemporaryFile(mode='w+') as log:
            arguments = [
                executable, '--ros-args', '-p', f'device:={os.ttyname(slave)}',
                '-p', 'baud_rate:=115200', '-p', 'frame_id:=imu_link', '-p', 'timeout:=1.0',
                '-p', 'use_6axis:=false',
                '-p', 'update_rate:=50.0']
            process = subprocess.Popen(arguments,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            packet = b'\x55\x61' + struct.pack('<9h', 2048, -1024, 1536, 16384, -8192, 0, 4096, -2048, 8192)
            reply = b'\x55\x71' + bytes(18)
            deadline = time.monotonic() + 8
            while not samples and time.monotonic() < deadline:
                assert process.poll() is None, 'Driver exited during discovery'
                respond_to_commands()
                # Garbage, repeated headers, an ignored reply and a split IMU packet.
                os.write(master, b'noise\x55' + reply + packet[:7])
                rclpy.spin_once(node, timeout_sec=.01)
                os.write(master, packet[7:])
                rclpy.spin_once(node, timeout_sec=.02)
            assert samples, 'No IMU message received'
            assert requested_rates == [0x08], '50 Hz sensor command mismatch'
            assert requested_modes == [0], '9-axis sensor mode command mismatch'
            message = samples[-1]
            assert message.header.frame_id == 'imu_link'
            assert message.header.stamp.sec > 0
            expected = [9.80665, -9.80665/2, 9.80665*3/4]
            for actual, value in zip([message.linear_acceleration.x, message.linear_acceleration.y,
                                      message.linear_acceleration.z], expected):
                assert math.isclose(actual, value, abs_tol=1e-10), 'Acceleration sign/scale mismatch'
            assert math.isclose(message.angular_velocity.x, math.radians(1000), abs_tol=1e-10)
            assert math.isclose(message.angular_velocity.y, math.radians(-500), abs_tol=1e-10)
            assert message.angular_velocity.z == 0
            r, p, y = math.pi/8, -math.pi/16, math.pi/4
            sr, cr, sp, cp, sy, cy = math.sin(r/2), math.cos(r/2), math.sin(p/2), math.cos(p/2), math.sin(y/2), math.cos(y/2)
            q = [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]
            actual = [message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w]
            assert math.isclose(sum(a*b for a,b in zip(actual, q)), 1, abs_tol=1e-10), 'Quaternion mismatch'
            assert all(v == 0 for v in message.orientation_covariance)

            def call_service(client, request):
                deadline = time.monotonic() + 5
                while not client.service_is_ready() and time.monotonic() < deadline:
                    respond_to_commands()
                    os.write(master, packet)
                    rclpy.spin_once(node, timeout_sec=.01)
                assert client.service_is_ready(), 'Service was not discovered'
                before = len(samples)
                future = client.call_async(request)
                while not future.done() and time.monotonic() < deadline:
                    assert process.poll() is None, 'Driver exited during service request'
                    respond_to_commands()
                    os.write(master, packet)
                    rclpy.spin_once(node, timeout_sec=.01)
                assert future.done(), 'Service did not respond'
                respond_to_commands()
                if len(samples)-before > 4:
                    stamps = [m.header.stamp.sec + m.header.stamp.nanosec*1e-9
                              for m in samples[max(0, before-1):]]
                    assert max(b-a for a,b in zip(stamps, stamps[1:])) < .15, 'Service blocked IMU reception'
                return future.result(), len(samples)-before

            request = SetParameters.Request()
            request.parameters = [Parameter(name, value=value).to_parameter_msg() for name, value in [
                ('device', '/dev/unavailable'), ('baud_rate', 9600), ('frame_id', 'changed'),
                ('timeout', 2.0), ('update_rate', 200.0), ('use_6axis', True)]]
            response, _ = call_service(parameter_client, request)
            assert len(response.results) == 6
            assert all(not result.successful for result in response.results), \
                'Startup-only parameter changes were accepted without applying them'

            assert not save_commands and not calibration_writes, 'Calibration started or saved at startup'
            response, count = call_service(start_client, Trigger.Request())
            assert response.success and calibration_mode == 7 and count > 0
            assert calibration_writes == [7] and not save_commands
            response, _ = call_service(start_client, Trigger.Request())
            assert not response.success and 'already active' in response.message
            assert calibration_writes == [7], 'Duplicate start overwrote calibration'
            ignore_calibration_write = True
            response, count = call_service(stop_client, Trigger.Request())
            assert not response.success and calibration_mode == 7 and count > 0
            assert not save_commands, 'Failed stop saved settings'
            ignore_calibration_write = False
            response, count = call_service(stop_client, Trigger.Request())
            assert response.success and calibration_mode == 0 and count > 0
            assert len(save_commands) == 1
            ignore_calibration_write = True
            response, _ = call_service(start_client, Trigger.Request())
            assert not response.success and calibration_mode == 0
            assert len(save_commands) == 1, 'Failed start saved settings'
            ignore_calibration_write = False
            ignore_calibration_read = True
            response, count = call_service(start_client, Trigger.Request())
            assert not response.success and 'readback timed out' in response.message and count > 0
            ignore_calibration_read = False
            calibration_mode = 1
            before = len(calibration_writes)
            response, _ = call_service(start_client, Trigger.Request())
            assert not response.success and 'another calibration mode' in response.message
            assert len(calibration_writes) == before
            # Stop can recover calibration started before this node instance, including retries.
            calibration_mode = 7
            response, _ = call_service(stop_client, Trigger.Request())
            assert response.success and calibration_mode == 0 and len(save_commands) == 2

            # Replaying register replies must not keep stale IMU data alive.
            deadline = time.monotonic() + 3
            while process.poll() is None and time.monotonic() < deadline:
                os.write(master, reply)
                rclpy.spin_once(node, timeout_sec=.05)
            assert process.poll() == 1, 'Missing IMU packets did not terminate the driver'
            log.seek(0)
            assert 'No complete WT901BLECL IMU packet' in log.read()
            samples.clear()
            arguments[-1] = 'update_rate:=200.0'
            wrong_readback = True
            process = subprocess.Popen(arguments,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 3
            while process.poll() is None and time.monotonic() < deadline:
                respond_to_commands()
                os.write(master, packet)
                rclpy.spin_once(node, timeout_sec=.02)
            assert process.poll() == 1 and not samples, 'Incorrect rate readback was accepted'
            log.seek(0)
            assert 'Sensor update_rate readback failed' in log.read()
            wrong_readback = False
            arguments[arguments.index('use_6axis:=false')] = 'use_6axis:=true'
            wrong_mode_readback = True
            process = subprocess.Popen(arguments,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 3
            while process.poll() is None and time.monotonic() < deadline:
                respond_to_commands()
                os.write(master, packet)
                rclpy.spin_once(node, timeout_sec=.02)
            assert process.poll() == 1 and not samples, 'Incorrect attitude mode readback was accepted'
            log.seek(0)
            assert 'Sensor use_6axis readback failed' in log.read()
            wrong_mode_readback = False
            process = subprocess.Popen(arguments,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 8
            while not samples and time.monotonic() < deadline:
                assert process.poll() is None, 'Driver exited before the disconnect check'
                respond_to_commands()
                os.write(master, packet)
                rclpy.spin_once(node, timeout_sec=.02)
            assert samples, 'Restarted driver did not publish'
            assert requested_rates == [0x08, 0x0b, 0x0b, 0x0b], '200 Hz sensor command mismatch'
            assert requested_modes == [0, 1, 1], '6-axis sensor mode command mismatch'
            os.close(master)
            master = -1
            assert process.wait(timeout=3) == 1, 'Disconnect did not terminate the driver'
            log.seek(0)
            assert 'Serial device disconnected' in log.read()
            print('6/9-axis mode/readback, calibration services/save/recovery, sensor rate, framing, SI units, attitude, timeout and disconnect passed.')
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        if master >= 0:
            os.close(master)
        os.close(slave)
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run(sys.argv[1])
