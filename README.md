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
- python3-dev (numpy is installed by pip/uv, both for the build and at runtime)

## Build instructions

### Common steps
```
git clone <path_to_repo>
cd kalibr_cam_calibration
git submodule update --init --recursive
```

### C++
```
cmake -S . -B build/release -DCMAKE_BUILD_TYPE=Release -DENABLE_SAMPLES=ON
cmake --build build/release -j$(nproc)
```
`ENABLE_SAMPLES=ON` also builds `build/release/samples/sample_run_detector`.

Optionally install the kalibr shared libraries (into `<prefix>/lib`, with `$ORIGIN` RPATH):
```
cmake --install build/release --prefix /path/to/prefix
```

### Python bindings
The package is built with [scikit-build-core](https://scikit-build-core.readthedocs.io) from
[python/pyproject.toml](python/pyproject.toml), which configures the top-level CMake project with `-DENABLE_PYTHON=ON`.
Build requirements (cmake, numpy, scikit-build-core) and the numpy runtime dependency are fetched automatically.

Install into a [uv](https://docs.astral.sh/uv/) virtual environment:
```
uv venv
source .venv/bin/activate
uv pip install -v ./python
```

Or build a wheel, e.g. to install on another machine:
```
uv build ./python --wheel -o dist
uv pip install dist/kalibr-0.1.0-cp*-linux_x86_64.whl
```

Check the installation from outside the repository:
```
cd /tmp && python -c "import kalibr"
```

Without uv, the same works with pip inside a regular venv: `pip install -v ./python`.

The wheel bundles the kalibr shared libraries (`libaslam_*.so`, `libsm_*.so`, ...) inside the `kalibr` package.
System libraries (OpenCV, Boost, SuiteSparse, fmt) are not bundled and must be installed on the target machine.

## Usage

### C++
Use the given [sample](samples/sample_run_detector.cpp) for running monocular camera calibration on a sequence of images

### Python
Installing the python wheel allows you to use kalibr as follows
```
import kalibr
```
See [sample](python/samples/sample_run_detector.py) for how to use the Kalibr frontend/backend for camera calibration.

### Camera <-> IMU calibration (Python)
`kalibr.backend.ImuCameraCalibrator` estimates the camera-IMU transformation `T_cam_imu` and their time offset.
Intrinsics come from a prior monocular calibration and stay fixed. Add the target observations from
`MarkerDetector.detectPattern` and the IMU samples as an `(N, 7)` array `[t_us, gx, gy, gz, ax, ay, az]`.
All timestamps are in microseconds and must come from the same clock.
```
calib = kalibr.backend.ImuCameraCalibrator(camera_result, kalibr.backend.ImuParameters())
for obs in observations:
    calib.addObservation(obs)
calib.addImuMeasurements(imu_data)
result = calib.calibrate()  # result.T_cam_imu (4x4), result.timeShift [s]: t_imu = t_cam + timeShift
```
