# Multisine identification control

run a multisne experiment, each execute append the estimated frequency response in parameter `hw_name`/`ctrl_name`/frequency_response.
You can visualize the bode diagram using the node

```sh
rosrun frequency_identification plot_freq_resp.py `hw_name`/`ctrl_name`
```

## Example of configuration

```yaml
freq_est:
  type:        cnr/control/MultisineIdentificationController
  controlled_joints: all
  base_link  : "world"
  tool_link  : "tool0"
  kin_update_period: 0.002
  appenders: [file, screen]
  levels: [trace, info]
  pattern_layout: "[%5p][%d{HH:mm:ss,SSS}][%M:%L][%c] %m%n"
  file_name: "freq_est"
  active_joint: "wrist_1_joint"

  # multisine parameters:
  rampup_time: 10.0  # [seconds] transient time before and after the multisine signal.

  carrier:
    angular_frequency: 0.5       # [rad/s]            angular frequency
    amplitude: 0.1               # [rad or m]         amplitude of the carrier
    periods:   2.0               # [-]                number of repetitions of the carriers (round using std::ceil)

  input:
    max_pos: 0.05                # [rad     or m]     maximum amplitude of the signal in position (deflection from the starting point)
    max_vel: 1.0                 # [rad/s   or m/s]   maximum velocity of the signal
    max_acc: 3.0                 # [rad/s^2 or m/s^2] maximum acceleration of the signal

    min_angular_frequency: 1.0   # [rad/s]            must be high than carrier/angular_frequency
    max_angular_frequency: 100.0 # [rad/s]            must be high than input/min_angular_frequency
    harmonics_number: 10         # [-]                number of harmonics in the multisine signal

```
