# 5DOF-Robotic-Arm

This repository is for my very first Hack Club Blueprint Grant Submission, along with anyone else who wants to watch my progress and build their own 5DOF robot arm.

<img width="920" height="603" alt="image" src="https://github.com/user-attachments/assets/ad5df4c4-09b1-48bf-8512-93b7103acdf2" />

## Overview
* ESP32-S3
* A4988 Drivers
* Nema 17 Stepper Motors
* 25:1 Strain Wave Gearbox *Credit to @user_454693624 over on Makerworld*

## WHY?
This is the first passion project of mine that I have actually pursued. I love engineering, building computers, and 3d printing. All these hobbies have led to this monstrosity of an endeavour. This arm is meant to be modular, easily printable, and fairly inexpensive for hobbyists to build on their own and tinker with. This project has definitely challenged me in ways I couldn't imagine. I started, naive to the oncoming money pit that the thing would become. That's why I am so thankful to the amazing folks over at Blueprint and Hackclub for supporting students in their pursuit of engineering.

## How Does it Work?
This 5DOF Arm is powered by 5 NEMA 17 34mm 0.35A 12V 1.8deg Stepper Motors that you can find at many online stores, such as StepperOnline or, in my case, Adafruit. The joints in the shoulder, elbow, and wrist pitch axis all use the same gearbox and housing assemblies, making for simpler printing and a common attachment point for new structural designs. You might notice there is no end effector piece on this Arm, and that is intentional. The PCB has two servo headers that can be utilized for many different Griper or Claw style designs, or maybe a suction cup or magnet, who knows the possibilities. The PCB is also made for simple installations, just requiring 
