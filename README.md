# Kalibr camera calibration

Simple fork of [kalibr](https://github.com/ethz-asl/kalibr) for monocular camera calibration. Inspired by
[this repo](https://gitlab.com/inivation/dv/dv-imu-cam-calibration).

## Build dependencies

- boost >= 1.79
- OpenCV
- fmt
- eigen
- libsuitesparse-dev
- gcc >= 10.0
- python3-dev
- python3-numpy

## Build instructions

### C++
#### 1) Clone the repo
```
git clone <path_to_repo>
cd kalibr_cam_calibration
```

#### 2) Update submodules
```
git submodule update --init --recursive
```

#### 3) Start build
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4 -s
```


### Python bindings

#### 1) Clone the repo
```
git clone <path_to_repo>
cd kalibr_cam_calibration
```

#### 2) Update submodules
```
git submodule update --init --recursive
```

#### 3) Create virtual environment
```
python3 -m venv venv
source venv/bin/activate
```

### 4) Install wheel requirements
```
pip install wheel numpy
```

#### 5) Build python wheel
```
python3 setup.py bdist_wheel
```

#### 6) Install resulting wheel via your favourite package manager

The resulting wheel should be found under build/dist/kalibr_cam_calibration-1.0.0-cpxxx-cpxxx-linux_x86_64.whl

To install the wheel using pip, for example, run:
```
pip install --force-reinstall path/to/build/dist/kalibr_cam_calibration-1.0.0-cpxxx-cpxxx-linux_x86_64.whl
```


## Usage

### C++
Use the given [sample](samples/sample_run_detector.cpp) for running monocular camera calibration on a sequence of images

### Python
Installing the python wheel allows you to use kalibr as follows
```
import kalibr_cam_calibration as kalibr
```
See [sample](python/samples/sample_run_detector.cpp) for how to use the Kalibr frontend/backend for camera calibration.