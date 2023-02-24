# rover4ws-kinematics
This python repository aims at providing useful functionalities to be used to compute the kinematics of a four wheel steering rover, which is subjected to steer lower and upper joint limits. Moreover, the package allows for multiple driving mode selection:

  * Symmetric Ackermann Steering;
  * Car-Like Ackermann Steering;
  * In-Place rotation;
  * Parallel Drive;
  * Lateral Drive;
  * Inner Ackermann;
  * Outer Ackermann;
  * General Ackermann.
  

The kinematics is computed through the ICR projection procedure.

### Package installation and Virtual environment setup
In order to being able to use this package, it is required to be installed as a python package. To do so, run:
```
python3.8 -m venv ~/p38Venv
mkdir -p ~/repositories/python
cd ~/repositories/python
git clone https://github.com/matteocaruso1993/rover4ws-kinematics
cd rover4ws-kinematics
source ~/p38Venv/bin/activate
pip install -r requirements.txt
pip install -e . 
deactivate
```
## Examples
#### Scripts
```
source ~/p38Venv/bin/activate
cd ~/repositories/python/rover4ws-kinematics
python rover4ws_kinematics/examples/test_carlike.py
```
#### Desktop App Visualization
```
source ~/p38Venv/bin/activate
python -m rover4ws_kinematics.apps
```

![Kinematics App](/rover4ws_kinematics/resources/app.png?raw=true "Kinematics app")


## Notes 
This package is used as the basis for the [kinematics ROS package](https://github.com/matteocaruso1993/ros-rover4ws-kinematics)