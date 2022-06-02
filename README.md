# Generate Safety-Critical Test Scenarios For ADSs in Simulator Based on Influential Traffic Factors

This project contains the implementation of CRISCO for constructing virtual scenarios to test Apollo in SVL simulator. CRISCO generates scenarios by extracted influential factors of traffic accidents. 

The generation approach requires the following dependencies to run:

	1. SVL simulator: https://www.svlsimulator.com/
	
	2. Apollo autonomous driving platform: https://github.com/ApolloAuto/apollo


# Prerequisites

* A 8-core processor and 16GB memory minimum
* Ubuntu 18.04 or later
* Python 3.8.11 or higher
* NVIDIA graphics card: NVIDIA proprietary driver (>=455.32) must be installed
* CUDA upgraded to version 11.1 to support Nvidia Ampere (30x0 series) GPUs
* Docker-CE version 19.03 and above
* NVIDIA Container Toolkit

# Requirements

Install LGSVL PythonAPI (pip3 install): https://github.com/lgsvl/PythonAPI

Other requirements: see in requirements.txt

# SVL - A Python API for SVL Simulator

Documentation is available on: https://www.svlsimulator.com/docs/

Version: SVL 2021.3

# Apollo - A high performance, flexible architecture which accelerates the development, testing, and deployment of Autonomous Vehicles

Website of Apollo: https://apollo.auto/

Installation of Apollo 6.0: https://www.svlsimulator.com/docs/system-under-test/apollo6-0-instructions/

# Run
To generate a set of concrete scenarios of one abstract scenario, execute the main() of generate_test_scenarios.py in generation_scenario directory; to stop the running, please press Ctrl+C until the program exits.

To replay the recorded safety-violation scenarios, execute the main() of reproduce_scenario.py in reproduce_safety_violation_scenarios directory(input scenario record file name)

# Customize Influential Factors
Environment: Environment file in UserDefined directory

Behavior Patterns: BehaviorPatterns file in UserDefined directory

Accident-prone Area: RoadDistrict file in UserDefined directory

