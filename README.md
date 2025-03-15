# PiRobot V.0 (basic)

![image](https://github.com/user-attachments/assets/e8536b8b-3085-4539-90a0-e954b36e6790)

### How to Use

1. Save the files in the appropriate structure:

1. `robot_controller.py` in the main directory
2. `control_panel.html` in a `templates` folder



2. Install required dependencies:

```plaintext
pip install wiringpi flask opencv-python psutil
```


3. Run the robot controller:

```plaintext
python robot_controller.py
```


4. Access the control panel at `http://<your_pi_ip>:5002`

### Future Development

1. **Sensor Integration**: Add classes for different sensors (ultrasonic, IR, etc.)
2. **Autonomous Navigation**: Implement path planning and obstacle avoidance
3. **Computer Vision**: Add object detection using TensorFlow Lite or similar
4. **Data Logging**: Implement data collection for training autonomous behaviors
