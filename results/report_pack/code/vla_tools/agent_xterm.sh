#!/bin/bash
source /home/danyalaziz/robot_env.sh
# #64 (11 Sep, hardware only): open the agent's camera gate on colour alone.
# The OAK-D runs colour-only on the robot (depth pipeline = +7 C on the Pi and
# ~4 s frame lag); ranging comes from the LiDAR (#54). The sim never sets this.
export VLA_RGB_ONLY=1
# #64b: arrival slack 0.60 m (default 0.35) -- Nav2's 0.25 m goal tolerance on
# top of the 1.30 m stand-off otherwise leaves the robot parked without
# announcing "Arrived" (2 of 3 drives, 11 Sep). The sim never sets this.
export VLA_ARRIVE_TOL=0.50   # 15 Sep: must exceed largest stand-off 0.80 + Nav2 tolerance 0.25 - stop 0.60 = 0.45
# #64d (14 Sep): drive up to the object. Stand-off 0.45 m from its laser points
# (the closest goal Nav2 accepts: robot_radius 0.175 + inflation), ring radii
# 0.45/0.65, arrival radius = 0.45 + Nav2's 0.25 tolerance + slack 0.30 = 0.75 m.
# Defaults in the agent (1.0 / 1.00,1.30 / 0.35) are what the sim still uses.
export VLA_STOP_DISTANCE=0.60
export VLA_STANDOFFS=0.60,0.80
# #64c (14 Sep): collision-guard distance 0.35 m (default 0.70). At 0.70 the guard
# stopped every move and every drive that started near the dock or near people
# ("something solid is 0.55 m ahead"). The Create 3 is 0.34 m long; Nav2's
# inflation layer and the bumpers remain. The sim never sets this.
export VLA_MIN_FRONT_CLEAR=0.35
# #64e (15 Sep): target in the camera frame within 1.2 m (LiDAR) = arrived, stop
# and announce -- no stand-off goal needed. A person near furniture left no
# ring goal outside Nav2's inflation; every goal was refused and the agent
# rescanned instead of stopping. Default 0 (off) in the agent = the sim.
export VLA_ARRIVE_IF_SEEN_M=1.2
cd /home/danyalaziz
exec python3 -u /home/danyalaziz/vla_agent_v28.py
