The codebase is for camera callibration which is been developed based on [Simple hand Eye Package ](https://github.com/Rooholla-KhorramBakht/SimpleHandEye)

# Camera Callibration

## About 📖

> The codebase is for camera callibration which is been developed based on [Simple hand Eye Package ](https://github.com/Rooholla-KhorramBakht/SimpleHandEye)

## Features ✨

- 📱 Eye to Base Callibration( Callibration with a fixed camera w.r.t base of manipulator)

# Development Tracking Board

## 📊 Feature Board (Q1 2024)


| Feature ID | Feature Name                    | Status         | Progress                              | Priority  | Owner   | Start Date | Target Date | Dependencies     | Notes                                               |
| ---------- | ------------------------------- | -------------- | ------------------------------------- | --------- | ------- | ---------- | ----------- | ---------------- | --------------------------------------------------- |
| CAM-001    | TCP based Camera Calibration    | 🟢 Complete    | ![100%](https://progress-bar.dev/100) | 🔴 High   | @saketh | 2024-02-01 | 2024-03-15  | None             | Need to test Camera Callibration based on TCP point |
| CAM-002    | System Dependencies Setup       | 🟡 In Progress | ![60%](https://progress-bar.dev/60)   | 🔴 High   | @saketh | 2024-03-01 | 2024-03-20  | CAM-001          |                                                     |
| CAM-003    | API for camera callibrated data | ⚪ Planned     | ![0%](https://progress-bar.dev/0)     | 🟡 Medium | TBD     | 2024-03-25 | 2024-04-15  | CAM-001, CAM-002 |                                                     |
| CAM-004    | Replay recorded poses           | ⚪ Planned     | ![0%](https://progress-bar.dev/0)     | 🟡 Medium | TBD     | 2024-04-01 | 2024-04-20  | CAM-003          |                                                     |
| CAM-005    | GUI for Automatic Callibration  | 🔵 In Design   | ![10%](https://progress-bar.dev/10)   | 🟢 Low    | @saketh | 2024-03-10 | 2024-04-30  | CAM-002          | 🎯 Completion Metrics                               |


| Module                  | Progress | Status         | Testing Coverage | Review Status |
| ----------------------- | -------- | -------------- | ---------------- | ------------- |
| TCP callibration        | 100%     | ✅ Completed   | 95%              | ✅ Approved   |
| Documenation            | 60%      | ⏳ In Progress | 45%              | 🔄 In Review  |
| GUI System              | 0%       | 📝 Planning    | -                | -             |
| API                     | 0%       | 📝 Planning    | -                | -             |
| Sampling based accuracy | 10%      | 🎨 Designing   | -                | -             |

### Recent Updates


| Date       | Type     | Change               | By      |
| ---------- | -------- | -------------------- | ------- |
| 2024-10-28 | ✨ New   | Added Readme         | @saketh |
| 2024-10-28 | 🐛 Merge | Merged LR cal branch | @saketh |
|            | 📚 Docs  |                      |         |

### Active Issues


| ID | Issue                                                             | Priority  | Assigned |
| -- | ----------------------------------------------------------------- | --------- | -------- |
| #1 | Offsets in X after callibration with high rotational angle change | 🔴 High   | @saketh  |
| #2 |                                                                   | 🟡 Medium |          |

## Installation 💻

### Prerequisites

```
Make sure pyrealsense and dependencies are installed
It is recommended to create conda environment for installation
```

### Quick Start

```bash
# clone the repository 
git clone --recursive https://github.com/SakethTecholution/camera-callibration.git
# Insall dependencies 
cd SimpleHandEye
pip install -e .
# start servor from docker container and initate robot connection 
# Run Callibration code 
python3 callibration.py

```

### Parameter Storage## Project Structure 🗂

```

├── readme.md
└── scripts # source code
    ├── callibration.py # main script 
    ├── inference.py # inference for live testing
    └── testing.py # Testing with aruco on hand
  
└── SimpleHandEye # Reference python package

```

## Developer Notes 📝

### Useful Links ( Need to be updated)

- [API Docs](docs/api.md)
- [Style Guide](docs/style.md)
- [Test Guide](docs/testing.md)

### Best Practices for Callibration

1. Make sure the TCP of the end effector is properly setted so that rotational angles are calculated from it
2. Keep max change in Roational Angles and min change in Translation to get proper callbiration values
3. Make sure always aruco is kept nearest possible to camera so that detection will be proper

## Authors 👥

- [Saketh ](https://github.com/SakethTecholution)
  - Callibration & Testing scripts
- This is code based on [Rohit](https://github.com/Rooholla-KhorramBakht) python package [SimpleHandEye](https://github.com/Rooholla-KhorramBakht/SimpleHandEye)

# Development Tracking Board

## 📊 Feature Board (Q1 2024)


| Feature ID | Feature Name                  | Status         | Progress                              | Priority  | Owner   | Start Date | Target Date | Dependencies     | Notes                                            |
| ---------- | ----------------------------- | -------------- | ------------------------------------- | --------- | ------- | ---------- | ----------- | ---------------- | ------------------------------------------------ |
| CAM-001    | TCP Camera Calibration        | 🟢 Complete    | ![100%](https://progress-bar.dev/100) | 🔴 High   | @saketh | 2024-02-01 | 2024-03-15  | None             | Successfully implemented basic TCP communication |
| CAM-002    | System Dependencies Setup     | 🟡 In Progress | ![60%](https://progress-bar.dev/60)   | 🔴 High   | @saketh | 2024-03-01 | 2024-03-20  | CAM-001          | OpenCV, PyQt integration pending                 |
| CAM-003    | GUI Automatic Calibration     | ⚪ Planned     | ![0%](https://progress-bar.dev/0)     | 🟡 Medium | TBD     | 2024-03-25 | 2024-04-15  | CAM-001, CAM-002 | Requires UI/UX review                            |
| CAM-004    | Calibration Parameter Storage | ⚪ Planned     | ![0%](https://progress-bar.dev/0)     | 🟡 Medium | TBD     | 2024-04-01 | 2024-04-20  | CAM-003          | Database design needed                           |
| CAM-005    | Error Handling System         | 🔵 In Design   | ![10%](https://progress-bar.dev/10)   | 🟢 Low    | @saketh | 2024-03-10 | 2024-04-30  | CAM-002          | Initial architecture review                      |

## 🎯 Completion Metrics


| Module            | Progress | Status         | Testing Coverage | Review Status |
| ----------------- | -------- | -------------- | ---------------- | ------------- |
| TCP Communication | 100%     | ✅ Completed   | 95%              | ✅ Approved   |
| Dependencies      | 60%      | ⏳ In Progress | 45%              | 🔄 In Review  |
| GUI System        | 0%       | 📝 Planning    | -                | -             |
| Parameter Storage | 0%       | 📝 Planning    | -                | -             |
| Error Handling    | 10%      | 🎨 Designing   | -                | -             |
