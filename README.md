Hello! Team 695 Bison Robotics is proud to release the code for our 2025 robot "Goldfish".

Goldfish features:
- Dual Limelight 4s running MegaTag2
- High-frequency odometry running at 200Hz, using a NavX2 and a full Kraken drivebase for robust localization
- Fully autonomous scoring and feeder station pickup
- All mechanism control loops run entirely on onboard motor controllers

Our auto-align system utlizes a repulsion field for avoiding collisions with the reef. Other than that it's just P control on odometry x, y, and θ. Everything was done from scratch.

We also made a desmos calculator as a proof-of-concept. You can drag the purple robot around and hit play in the top-left to animate its behavior. (Note: The real robot includes acceleration limits to mitigate the jerkiness you might see in Desmos when we switch from applying repulsion to not applying repulsion)

The repulsion field only kicks in if we detect a potential collision on a straight path to the target. Here's a quick summary of how it works:
* We calculate vectors from each reef vertex to the robot's center, scaled by the inverse square of the distance
* These are summed and combined with an attraction vector from the robot to the target.

The same auto align system is used for all our autons.

https://www.desmos.com/calculator/ygvuapt0xr
https://github.com/FRCTeam695/Goldfish
