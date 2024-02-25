# APC: Adaptive Platoon Control Strategy at a Signalized Intersection

## Introduction

This is the code repository for the paper "APC: Adaptive Platoon Control Strategy at a Signalized Intersection". In the paper, we propose an adaptive platoon control strategy named APC for intersection scenarios to reduce travel time and fuel consumption during the passage through intersections. We first distinguish two different stages based on the status of traffic light phases. Then we develop two components for the strategy including speed trajectory planning which minimizes delay and fuel consumption at intersections and optimal length planning which ensures correct platoon maneuvers before intersection passage. We validate the proposed method on an open-source simulation platform -- VENTOS. We analyze the platooning behavior in two designed scenarios, including a big-sized platoon splitting situation and small-sized platoons merging situation.  The repository includes **the** **source code of APC strategy** and **the implementation of the two scenarios** described in the paper.

## Getting Started

The platform VENTOS is an integrated simulator based on two well-known simulators: SUMO and OMNeT++. Therefore, first, you should download and install SUMO and OMNeT++, and then import the project into the OMNeT++ IDE. 

```sh
git clone https://github.com/crashcar/APC-Adaptive-Platoon-Control-Strategy-at-a-Signalized-Intersection.git
```

Run the 'runme' script to complete the installation process.

```sh
./runme
```

Import APC project into the OMNeT++ IDE by choosing `File->Import` from the menu. Choose `General->Existing Projects into Workspace` from the upcoming dialog and proceed with `Next`. Choose `Select root directory` and select the APC folder. `VENTOS(YourPathToAPC/APC-Adaptive-Platoon-Control-Strategy-at-a-Signalized-Intersection)` should appear in the "Projects" section. Unselect `Copy project into workspace` if the VENTOS folder is already in your workspace. Click `Finish`.

The simulation configuration file is located in `YourPathToAPC/APC-Adaptive-Platoon-Control-Strategy-at-a-Signalized-Intersection/examples/intersectionPlatoon/omnetpp.ini`. This configuration file contains a total of four simulation setups using APC and manual control strategies across two scenarios. To start the simulation, click the `Run IntersectionPlatoon` button in the top menu bar.

## Simulation Results
APC in Scenario 1: the 8-vehicle platoon performing splitting maneuver at the intersection

https://github.com/crashcar/APC-Adaptive-Platoon-Control-Strategy-at-a-Signalized-Intersection/assets/56828380/7841c5b3-b508-4075-9bec-43e6f5d2f92c

APC in Scenario 2: three platoons (3/3/2-vehicle) performing merging maneuver at the intersection

https://github.com/crashcar/APC-Adaptive-Platoon-Control-Strategy-at-a-Signalized-Intersection/assets/56828380/e31677a3-51e8-4060-bee8-4b4767add4c4


