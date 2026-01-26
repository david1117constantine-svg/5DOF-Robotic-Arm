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

## PCB: Schematic, Footprints, and Renders

#### Schematic:
<img width="1165" height="824" alt="image" src="https://github.com/user-attachments/assets/5bfaf8ac-7df0-487d-9d19-fb7f5e813fa5" />

#### Footprint:
<img width="981" height="838" alt="image" src="https://github.com/user-attachments/assets/a67a73c5-bd7d-450b-9d9b-64fb93702eaa" />

#### Render:
<img width="995" height="834" alt="image" src="https://github.com/user-attachments/assets/20c2cea6-3edb-4c8a-bf0c-1cd59097921e" />



## BOM
| Item                                                    | Quantity   | Price ($)   |
|:--------------------------------------------------------|:-----------|:------------------|
| Self-Sourced                                            |            |                   |
| Screws - Assorted M3                                    | 2240       | Self-Sourced      |
| Bearing - 6807 2RS                                      | 5          | Self-Sourced      |
| Microcontroller - ESP32                                 | 1          | Self-Sourced      |
| Electronics (Excluding PCB)                              |            |                   |
| Stepper motor - NEMA-17 size - 200 steps/rev, 12V 350mA | 5          | 70.00             |
| Stepper Motor Driver - A4988                            | 5          | 5.12              |
| Limit Switch                                            | 3          | 0.41              |
| Buck Converter - 12v to 5v, 5A                          | 1          | 2.96              |
| Wire Connectors - JST xh 2.54 2Pin 10pcs                | 1          | 1.51              |
| Wire Connectors - JST xh 2.54 4Pin 10pcs                | 1          | 2.02              |
| Wire - General Purpose 28 Gauge Wire                    | 1          | 9.66              |
| Heat Shrink Tubing - 5mm ID, 5M                         | 1          | 1.90              |
| Power Supply - DC12V 8A                                 | 1          | 15.43             |
| Hardware                                                |            |                   |
| Extrusions - 150mm 2020 T-slot Extrusions 4pcs          | 4          | 17.98             |
| Screws - M4x10 Countersunk                              | 4          | 0.02              |
| Screws - M4x16 BHCS                                     | 8          | 0.09              |
| T-Nuts - M4 20 Series Profile                           | 12         | 0.09              |
| Heat Inserts - M3x4x5                                   | 100        | 4.18              |
| NEMA 17 Strain Wave Gearbox (OpenSource) BOM            |            |                   |
| Bearings  - 6706 2RS                                    | 5          | 4.33              |
| Bearings - 605ZZ                                        | 10         | 1.96              |
| Bearings - 4mm Balls                                    | 200        | 6.28              |
| PCB Components                                          |            |                   |
| Capacitor - 100uF 16V Capacitor                         | 5          | 0.70              |
| Capacitor - 0.1uF Capacitor                             | 5          | 0.62              |
| Capacitor - 47uF                                        | 1          | 0.90              |
| Capacitor - 1000uF                                      | 2          | 1.33              |
| Screw Terminal - 2 Pin                                  | 3          | 1.84              |
| Schottky Diode - 1N5819                                 | 2          | 0.06              |
| Female Pin Headers - 1x22                               | 2          | 0.80              |
| Female Pin Headers - 1x8                                | 10         | 0.59              |
