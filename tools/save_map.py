#!/usr/bin/env python3
# Saves one /robot1/map OccupancyGrid to ~/vla_map.pgm + ~/vla_map.yaml (Nav2 format).

import rclpy                                     # ROS 2 client library for Python: init, spin, shutdown
from rclpy.node import Node                      # every ROS 2 participant is a Node; we subclass it
from rclpy.qos import QoSProfile                 # container holding the four QoS settings below
from rclpy.qos import QoSDurabilityPolicy        # controls whether late subscribers get the last message
from rclpy.qos import QoSReliabilityPolicy       # controls whether lost packets are retransmitted
from rclpy.qos import QoSHistoryPolicy           # controls how many old messages are buffered
from nav_msgs.msg import OccupancyGrid           # the message type slam_toolbox publishes on /robot1/map
import numpy as np                               # fast array maths: reshape the grid, recolour it, flip it
import os                                        # expands '~' and reads VLA_* paths
import sys                                       # only used to exit with a non-zero code on timeout

TOPIC = '/robot1/map'                            # the namespaced map topic (plain /map does not exist here)
OUT = os.environ.get('VLA_MAP_OUT') or os.path.join(
    os.environ.get('VLA_MAP_DIR')
    or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'maps'),
    'vla_map')
# output basename; '.pgm' and '.yaml' get appended
TIMEOUT_S = 30.0                                 # give up after 30 s instead of hanging forever


class MapSaver(Node):                            # our node: subscribe once, write two files, stop
    def __init__(self):                          # constructor: runs when the node is created
        super().__init__('map_saver_custom')     # register with ROS 2 under this node name
        qos = QoSProfile(                        # QoS must MATCH the publisher or DDS never connects us
            depth=1,                             # keep only the newest map; older ones are useless
            history=QoSHistoryPolicy.KEEP_LAST,  # "keep last N", where N is the depth above
            reliability=QoSReliabilityPolicy.RELIABLE,        # slam_toolbox publishes RELIABLE
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)   # publisher stores the last map for latecomers
        self.sub = self.create_subscription(     # create the subscription; kept in self so it stays alive
            OccupancyGrid,                       # message type we expect on the wire
            TOPIC,                               # topic name to subscribe to
            self.cb,                             # function called once per received message
            qos)                                 # the matching QoS profile built above
        self.done = False                        # flag flipped to True once the files are written

    def cb(self, msg):                           # called by ROS 2 when a map message arrives
        if self.done:                            # ignore any further maps after the first
            return                               # nothing more to do
        w = msg.info.width                       # map width in cells (163 in your case)
        h = msg.info.height                      # map height in cells (293 in your case)
        res = msg.info.resolution                # metres represented by one cell (0.05 = 5 cm)
        ox = msg.info.origin.position.x          # world X of the map's bottom-left corner (-3.5658)
        oy = msg.info.origin.position.y          # world Y of the map's bottom-left corner (-7.5728)

        data = np.array(msg.data, dtype=np.int8)  # msg.data is one flat list of w*h occupancy values
        data = data.reshape((h, w))               # fold that flat list into a 2D grid, row by row

        img = np.zeros((h, w), dtype=np.uint8)   # blank 8-bit image, same shape as the grid
        img[data == -1] = 205                    # -1 means "never seen"  -> mid grey, the Nav2 convention
        img[data == 0] = 254                     # 0 means "known free"   -> near-white
        img[data > 0] = 0                        # 1..100 means "occupied" -> black
        img = np.flipud(img)                     # ROS grids start bottom-left, images start top-left

        with open(OUT + '.pgm', 'wb') as f:      # 'wb' = write binary; a PGM is raw bytes, not text
            f.write(b'P5\n')                     # magic number: P5 = binary greyscale PGM
            f.write(b'%d %d\n' % (w, h))         # header line: width then height, in cells
            f.write(b'255\n')                    # header line: the maximum pixel value
            f.write(img.tobytes())               # the pixels themselves, one byte each, row by row

        with open(OUT + '.yaml', 'w') as f:      # 'w' = write text; the YAML is what Nav2 actually loads
            f.write('image: vla_map.pgm\n')      # relative filename of the image beside this YAML
            f.write('resolution: %f\n' % res)    # metres per pixel, so Nav2 can scale the image
            f.write('origin: [%f, %f, %f]\n' %   # where the image's bottom-left corner sits in the world
                    (ox, oy, 0.0))               # x, y, and yaw; yaw is 0 because SLAM maps are unrotated
            f.write('negate: 0\n')               # 0 = do not invert the greys (white is free, as written)
            f.write('occupied_thresh: 0.65\n')   # pixels darker than 65% occupied count as obstacles
            f.write('free_thresh: 0.25\n')       # pixels lighter than 25% occupied count as free space

        self.get_logger().info(                  # print confirmation to the terminal
            'Saved %dx%d map (%.3f m/cell) to %s.pgm / %s.yaml' % (w, h, res, OUT, OUT))
        self.done = True                         # tell the main loop below that we can stop


rclpy.init()                                     # start ROS 2 for this process
node = MapSaver()                                # build the node, which creates the subscription
node.get_logger().info('Waiting for %s ...' % TOPIC)   # tell the user we are listening
deadline = node.get_clock().now().nanoseconds + int(TIMEOUT_S * 1e9)   # absolute give-up time
while not node.done:                             # loop until the callback has written the files
    rclpy.spin_once(node, timeout_sec=0.5)       # process incoming messages for up to 0.5 s, then return
    if node.get_clock().now().nanoseconds > deadline:   # have we run past the give-up time?
        node.get_logger().error(                 # report failure rather than hanging silently
            'No map received in %.0f s - check the discovery-server env in THIS shell' % TIMEOUT_S)
        node.destroy_node()                      # release the subscription
        rclpy.shutdown()                         # stop ROS 2
        sys.exit(1)                              # non-zero exit code = failure
node.destroy_node()                              # success path: release the subscription
rclpy.shutdown()                                 # success path: stop ROS 2 cleanly
