# 6DOF-Robotic-Arm

This repository is for my very first Hack Club Blueprint Grant Submission, along with anyone else who wants to watch my progress and build their own 6DOF robot arm.

## Current Progress
*As of January 8th, 2026*

![Master v6 in Fusion Environment](.PNG/Image_1.png)
*Fig.1 (Image_1)*

This is the current state of my arm. In *Fig.1*, the Forearm is a copy of the Upper Arm for visualization purposes. 

I have reached the point at which the stepper motors I am using, 28BYJ-48 Unipolar Stepper Motors with ULN2003 drivers from Elegoo (As seen in *Fig.2*), no longer supply the torque needed to lift anything more than the Upper Arm, and even then, they still skip a couple of teeth when the shoulder is extended by its full range of motion.

![28BYJ-48 w/ ULN2003 Driver](https://ae01.alicdn.com/kf/S1c7b102989e445338c556a86d37a3a39u.jpg)
*Fig.2*

Of course, on their own, the 28BYJ-48 only produces about 3.4 N · cm of torque, which is not nearly enough to support any reasonable weight. That is why they are currently being paired with two 4:1 planetary gearboxes stacked on top of oneanother, providing a 16:1 gear reduction and a subsequent output of roughly 54.4 N · cm of torque. However, still this seems to be barely enough to suffice withthe progress Ive made so far. This is why I intend to upgrade from these tiny steppers to something larger, like NEMA 17 Stepper Motors, like the ones from Adafruit that provide 20 N · cm of torque per phase at 0.35A (350mAh).
