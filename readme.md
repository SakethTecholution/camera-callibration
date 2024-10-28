The codebase is for camera callibration which is been developed based on [Simple hand Eye Package ](https://github.com/Rooholla-KhorramBakht/SimpleHandEye)

# Camera Callibration

## About 📖

> The codebase is for camera callibration which is been developed based on [Simple hand Eye Package ](https://github.com/Rooholla-KhorramBakht/SimpleHandEye)
>

## Features ✨

- 📱 Eye to Base Callibration( Callibration with a fixed camera w.r.t base of manipulator)

## Development Status 📈

### Current Sprint

Sprint: #4
Duration: DD MMM - DD MMM YYYY
Progress: ██████░░░░ 30

### Feature Board


| Feature                          | Status     | Progress                              | Owner  | Target |
| -------------------------------- | ---------- | ------------------------------------- | ------ | ------ |
| TCP based Camera Callibration    | 🟢 Done    | ![100%](https://progress-bar.dev/100) | @john  | Mar 15 |
| API for Backend                  | 🟡 Ongoing | ![60%](https://progress-bar.dev/60)   | @sarah | Mar 20 |
| GUI based Atuomatic Callibration | ⚪ Planned | ![0%](https://progress-bar.dev/0)     | @mike  | Mar 30 |

### Recent Updates



| Date       | Type     | Change               | By      |
| ---------- | -------- | -------------------- | ------- |
| 2024-10-28 | ✨ New   | Added Readme         | @saketh |
| 2024-10-28 | 🐛 Merge | Merged LR cal branch | @saketh |
|            | 📚 Docs  |                      |         |

### Active Issues


| ID | Issue                           | Priority  | Assigned |
| -- | ------------------------------- | --------- | -------- |
| #1 | Offsets in X after callibration | 🔴 High   | @saketh  |
| #2 |                                 | 🟡 Medium |          |

## Installation 💻

### Prerequisites

```
Make sure pyrealsense and dependencies are installed
It is recommended to create conda environment for installation
```

### Quick Start

```bash
# clone the repository 
git clone 
# Insall dependencies 
cd SimpleHandEye
pip install -e .

```

### ## Project Structure 🗂

```

├── readme.md
└── scripts # source code
    ├── callibration.py # main script 
    ├── inference.py # inference for live testing
    └── testing.py # Testing with aruco on hand

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
