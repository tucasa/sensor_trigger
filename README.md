# sensor_trigger

## Introduction

This branch generates camera trigger pulses for the Vecow EAC-5000. Each pulse is written to a MAX9296 deserializer MFP0 register over I2C. Jetson GPIO and libgpiod are not used.

The camera driver must be in `trigger_mode=2` so the sensor listens for the MFP0 register-driven trigger. This node only produces the pulses. UART1 is left enabled.

## Requirements

- ECU: Vecow EAC-5000 (Jetson AGX Orin)
- OS: Ubuntu 20.04
- ROS2: Foxy Fitzroy
- The process user must be able to open `/dev/i2c-30` through `/dev/i2c-33` (membership in the `i2c` group)

## Installation

1. Create or change into your workspace directory, and execute the following:

    ```bash
    mkdir -p src
    git clone git@github.com:tier4/sensor_trigger.git src
    git -C src checkout feat/eac-5000-i2c-trigger
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to sensor_trigger
    ```

2. The sensor trigger node requires as close to real-time operation as possible to maintain reliable trigger timing under heavy CPU load. If this is required, it is recommended to allow thread schedule priority setting to the ROS2 user by adding the following line to `/etc/security/limits.conf`:

    ```
    <username>    -   rtprio   98
    ```

    After saving the edited file (as sudo), a reboot will be required.

    Some notes about thread scheduling:

    - If you do not make the settings in (2), the node will run but the timing thread will not be scheduled at any higher priority than other user threads and interruption may occur, resulting in high jitter in the trigger output.
    - When running in a ROS2 docker, the settings in (2) are not required as the docker user is by default the root user.

## Usage

The node writes a high/low pulse on each enabled deserializer MFP0 at the configured rate. `phase` is degrees relative to Top of Second (ToS), using the same conversion as the original GPIO node:

```text
start_nsec = interval_nsec * (phase * 10) / 3600
```

A phase of 0 starts on ToS. There is no separate time-offset parameter.

`dserN.cpu_core_id` selects the CPU core for that deserializer's trigger thread. CPU 0 is used for system interrupts, so a core greater than 0 is recommended. Give each enabled deserializer its own core. Check the available cores with `nproc --all`.

```bash
ros2 launch sensor_trigger sensor_trigger.launch.xml
```

## Deserializer mapping

The I2C bus for each camera is fixed. Cameras that share a deserializer receive the same MFP0 waveform. `frame_rate` and `pulse_width_ms` apply to every deserializer. Phase is set per deserializer.

| Deserializer | Cameras | I2C device                         | Address | Register |
| ------------ | ------- | ---------------------------------- | ------- | -------- |
| `dser0`      | 0, 1    | `/dev/i2c-30` (`tca9543@72/i2c@0`) | `0x48`  | `0x02B0` |
| `dser1`      | 2, 3    | `/dev/i2c-31` (`tca9543@72/i2c@1`) | `0x48`  | `0x02B0` |
| `dser2`      | 4, 5    | `/dev/i2c-32` (`tca9543@73/i2c@0`) | `0x48`  | `0x02B0` |
| `dser3`      | 6, 7    | `/dev/i2c-33` (`tca9543@73/i2c@1`) | `0x48`  | `0x02B0` |

MFP0 high is `0x12` and low is `0x02`. TX ID `0x06` is written to `0x02B1` when the node starts. On exit, `0x02B0`, `0x02B1`, and `0x02B2` are restored.

## Inputs / Outputs

### Input

This node does not take any inputs.

### Output

| Name                  | Type                            | Description                                              |
| --------------------- | ------------------------------- | -------------------------------------------------------- |
| `dser0/trigger_time`  | `builtin_interfaces::msg::Time` | Time the dser0 rising edge was requested (system time)   |
| `dser1/trigger_time`  | `builtin_interfaces::msg::Time` | Time the dser1 rising edge was requested (system time)   |
| `dser2/trigger_time`  | `builtin_interfaces::msg::Time` | Time the dser2 rising edge was requested (system time)   |
| `dser3/trigger_time`  | `builtin_interfaces::msg::Time` | Time the dser3 rising edge was requested (system time)   |

A disabled deserializer does not publish.

## Parameters

| Name                    | Type   | Default | Description                                      |
| ----------------------- | ------ | ------- | ------------------------------------------------ |
| `frame_rate`            | double | `10.0`  | Trigger frequency in Hz for every deserializer. Values below 1 stop the node |
| `pulse_width_ms`        | int    | `5`     | Time every MFP0 output stays high, in milliseconds |
| `dserN.phase`           | double | `0.0`   | Phase in degrees relative to ToS                 |
| `dserN.cpu_core_id`     | int    | `N + 1` | CPU core for this deserializer's trigger thread. Defaults are 1, 2, 3, and 4 |
| `dserN.enabled`         | bool   | `true`  | Enable this deserializer. `N` is 0, 1, 2, or 3   |

Core numbers are indexed from CPU 0 (CPU 1 in `htop`).

## Related Repositories

- [tier4/perception_ecu_container](https://github.com/tier4/perception_ecu_container)
  - Meta-repository containing `.repos` file to construct perception-ecu workspace
- [tier4/perception_ecu_launch](https://github.com/tier4/perception_ecu_launch.git)
- [tier4/ros2_v4l2_camera](https://github.com/tier4/ros2_v4l2_camera.git)
- [tier4/perception_ecu_individual_params](https://github.com/tier4/perception_ecu_individual_params)
- [autowarefoundation/autoware.universe](https://github.com/autowarefoundation/autoware.universe.git)
