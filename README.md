# solo-pybullet

[![Pipeline status](https://gepgitlab.laas.fr/gepetto/solo-pybullet/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/gepetto/solo-pybullet/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/gepetto/solo-pybullet/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/gepetto/solo-pybullet/master/coverage/)

**Simulation and Controller code for Solo Quadruped**

This repository offers an environment to simulate different controllers on the Quadruped robot **Solo**.

You can implement your controller on the *controller.py* file and call your control function in the main program *main.py* by replacing the `c(...)` function in the loop.

## Installation
### Python Version
{PYTHON_VERSION}

### Add robotpkg apt repository
```bash
sudo apt install -qqy lsb-release gnupg2 curl
echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
sudo apt-get update
```

### Install Pinocchio
```bash
 sudo apt install -qqy robotpkg-py{PYTHON_VERSION}-pinocchio
```

### Configure environment variables
```bash
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python{PYTHON_VERSION}/site-packages:$PYTHONPATH # Adapt your desired python version here
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

### Install dependencies
```bash
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -cs) robotpkg
deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg
EOF
sudo apt update -qqy
sudo apt install -qqy robotpkg-py{PYTHON_VERSION}-{pinocchio,example-robot-data,qt5-gepetto-viewer-corba}
```

### Install PyBullet
```bash
pip3 install --user pybullet
```

### Install remote controller dependencies
https://blog.thea.codes/talking-to-gamepads-without-pygame/
https://medium.com/devopss-hole/python-error-undefined-symbol-hid-get-input-report-c01af1ccf1f6

## How to start the simulation
launch `gepetto-gui`, then `python3 -m solo_pybullet`
