# GNSS Pose Publisher for GNSS Fault Injection
## 1. Build and Run
* Launch the node: `ros2 run gnss_fault_injection gnss_fault_injection`
## 2. Usage Examples
* The GNSS fault injection node allows for the injection of noise into GNSS pose data. Use the following commands to control the noise injection.
* Start Noise Injection: `ros2 topic pub /gnss_fault_injection/command std_msgs/msg/String "data: 'start'"`
* Stop Noise Inhection: `ros2 topic pub /gnss_fault_injection/command std_msgs/msg/String "data: 'stop'"`
## Note
* Pressing `Ctrl+C` does not affect the state of noise injection.
* Only the `start` and `stop` commands activate or deactivate the noise injection.
## Testing
* Subscribe to the output topic: `ros2 topic echo /sensing/gnss/pose_with_fault_injection`
* CHeck the data published on the `/sensing/gnss/pose_with_fault_injection` topic to ensure the expected behavior.
