'''
Webots simulator support for Crazyflie Python client

Copyright (C) 2023 Simon D. Levy

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 51
Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
'''

# Changelog
# ---------
# Created 07-08-2023 Simon D. Levy for PR CFclient #672
# Adapted 15-08-2023 Kimberly as socket script for webots

import socket
import threading
import time
import struct
import keyboard

class WebotsConnection:

    def __init__(self, host = '127.0.0.1', port=5000):

        self.host = host
        self.port = port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)

        self.is_open = False

        self.pose = 0, 0, 0, 0, 0, 0
        self.velocities = 0, 0, 0, 0
        self.mode = 0

    def start(self):

        try:
            self.sock.connect((self.host, self.port))

        except:
            return False

        self.is_open = True

        threading.Thread(target=self._threadfun).start()

        return True

    def stop(self):

        self.is_open = False
        self.sock.close()

    def getPose(self):

        return self.pose

    def setDesiredVelocity(self, velocities):
        self.velocities = velocities

    def _threadfun(self):

        while self.is_open:

            try:

                self.pose = struct.unpack('ffffff', self.sock.recv(24))

                self.sock.send(struct.pack('ffff',
                    self.velocities[0],
                    self.velocities[1],
                    self.velocities[2],
                    self.velocities[3]))

            except:
                pass

            time.sleep(0)

# start the server
if __name__ == '__main__':
    print("Starting server")
    conn = WebotsConnection()
    conn.start()


    print("\n");

    print("====== External Socket Script Webots =======\n\n");

    # Handle keyboard interrupt
    try:
        while True:
            time.sleep(0.1)
            print(f"x: {conn.pose[0]:.2f}, y: {conn.pose[1]:.2f}, z: {conn.pose[2]:.2f}, yaw: {conn.pose[5]:.2f}")

            # get keyboard key pressed that doesn't block the loop
            if keyboard.is_pressed('up'):
                conn.setDesiredVelocity([0.5, 0, 0, 0])
            elif keyboard.is_pressed('down'):
                conn.setDesiredVelocity([-0.5, 0, 0, 0])
            elif keyboard.is_pressed('left'):
                conn.setDesiredVelocity([0, 0.5, 0, 0])
            elif keyboard.is_pressed('right'):
                conn.setDesiredVelocity([0, -0.5, 0, 0])
            elif keyboard.is_pressed('q'):
                conn.setDesiredVelocity([0, 0, 0, 0.5])
            elif keyboard.is_pressed('e'):
                conn.setDesiredVelocity([0, 0, 0, -0.5])
            elif keyboard.is_pressed('w'):
                conn.setDesiredVelocity([0, 0, 0.1, 0])
            elif keyboard.is_pressed('s'):
                conn.setDesiredVelocity([0, 0, -0.1, 0])
            else:
                conn.setDesiredVelocity([0, 0, 0, 0.0])

    except KeyboardInterrupt:
        print("Stopping server")
        conn.stop()

