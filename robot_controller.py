import wiringpi as wp
from flask import Flask, render_template, Response, request, jsonify
import threading
import cv2
import time
import logging
import os
import signal
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler("robot.log")
    ]
)
logger = logging.getLogger("RobotController")

class MotorController:
    """Handles motor control operations"""
    
    def __init__(self):
        # Setup WiringPi
        wp.wiringPiSetup()
        
        # Pin definitions
        self.MOTOR_PINS = {
            "motor1": {"in1": 0, "in2": 1, "enable": 4},  # GPIO 17, 18, soft PWM 4
            "motor2": {"in1": 2, "in2": 3, "enable": 5}   # GPIO 27, 22, soft PWM 5
        }
        
        # Configure pins and initialize PWM
        for motor, pins in self.MOTOR_PINS.items():
            wp.pinMode(pins["in1"], 1)
            wp.pinMode(pins["in2"], 1)
            wp.softPwmCreate(pins["enable"], 0, 100)
            
        logger.info("Motor controller initialized")
        
    def control_motor(self, motor, direction, speed):
        """Control a specific motor"""
        pins = self.MOTOR_PINS[motor]
        wp.softPwmWrite(pins["enable"], int(speed * 100))
        
        if direction == "forward":
            wp.digitalWrite(pins["in1"], 0)
            wp.digitalWrite(pins["in2"], 1)
        elif direction == "backward":
            wp.digitalWrite(pins["in1"], 1)
            wp.digitalWrite(pins["in2"], 0)
        else:  # Stop
            wp.digitalWrite(pins["in1"], 0)
            wp.digitalWrite(pins["in2"], 0)
            
    def move(self, direction, speed):
        """Move the robot in a specific direction"""
        if direction == "forward":
            self.control_motor("motor1", "forward", speed)
            self.control_motor("motor2", "forward", speed)
            logger.debug(f"Moving forward at speed {speed}")
        elif direction == "backward":
            self.control_motor("motor1", "backward", speed)
            self.control_motor("motor2", "backward", speed)
            logger.debug(f"Moving backward at speed {speed}")
        elif direction == "left":
            self.control_motor("motor1", "backward", speed)
            self.control_motor("motor2", "forward", speed)
            logger.debug(f"Turning left at speed {speed}")
        elif direction == "right":
            self.control_motor("motor1", "forward", speed)
            self.control_motor("motor2", "backward", speed)
            logger.debug(f"Turning right at speed {speed}")
        elif direction == "stop":
            self.control_motor("motor1", "stop", 0)
            self.control_motor("motor2", "stop", 0)
            logger.debug("Stopping motors")
            
    def cleanup(self):
        """Stop motors and clean up"""
        self.move("stop", 0)
        logger.info("Motors stopped and cleaned up")


class CameraController:
    """Handles camera operations"""
    
    def __init__(self):
        self.cameras = []
        self.camera_index = 0
        self.current_camera = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.running = True
        self.detect_cameras()
        
        # Start frame capture thread
        self.capture_thread = threading.Thread(target=self._capture_frames, daemon=True)
        self.capture_thread.start()
        
        logger.info(f"Camera controller initialized. Found cameras: {self.cameras}")
        
    def detect_cameras(self):
        """Detect available cameras"""
        self.cameras = []
        for i in range(5):  # Check the first 5 camera indices
            try:
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    self.cameras.append(i)
                    cap.release()
            except Exception as e:
                logger.error(f"Error detecting camera {i}: {e}")
                
        if not self.cameras:
            logger.warning("No cameras detected!")
        
    def set_camera(self, camera_id):
        """Set the active camera"""
        if camera_id in self.cameras:
            self.camera_index = camera_id
            logger.info(f"Switched to camera {camera_id}")
            return True
        else:
            logger.warning(f"Camera {camera_id} not available")
            return False
            
    def _capture_frames(self):
        """Continuously capture frames from the current camera"""
        while self.running:
            try:
                # If camera index changed, reopen camera
                if self.current_camera is None or self.current_camera != self.camera_index:
                    if self.current_camera is not None:
                        cap = cv2.VideoCapture(self.current_camera)
                        cap.release()
                    
                    self.current_camera = self.camera_index
                    cap = cv2.VideoCapture(self.current_camera)
                    
                    if not cap.isOpened():
                        logger.error(f"Failed to open camera {self.current_camera}")
                        time.sleep(1)
                        continue
                
                # Read frame
                success, frame = cap.read()
                if not success:
                    logger.warning(f"Failed to read frame from camera {self.current_camera}")
                    time.sleep(0.1)
                    continue
                    
                # Update latest frame
                with self.frame_lock:
                    self.latest_frame = frame
                    
                time.sleep(0.03)  # ~30 FPS
                
            except Exception as e:
                logger.error(f"Error in frame capture: {e}")
                time.sleep(1)
                
    def generate_frames(self):
        """Generate frames for streaming"""
        while self.running:
            with self.frame_lock:
                if self.latest_frame is None:
                    time.sleep(0.1)
                    continue
                    
                frame = self.latest_frame.copy()
                
            try:
                _, buffer = cv2.imencode('.jpg', frame)
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            except Exception as e:
                logger.error(f"Error encoding frame: {e}")
                time.sleep(0.1)
                
    def cleanup(self):
        """Clean up camera resources"""
        self.running = False
        if self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        logger.info("Camera controller cleaned up")


class RobotWebServer:
    """Web server for robot control"""
    
    def __init__(self, motor_controller, camera_controller, host="0.0.0.0", port=5002):
        self.app = Flask(__name__)
        self.motor_controller = motor_controller
        self.camera_controller = camera_controller
        self.host = host
        self.port = port
        self.server_thread = None
        self.setup_routes()
        
        logger.info(f"Web server initialized on {host}:{port}")
        
    def setup_routes(self):
        """Set up Flask routes"""
        
        @self.app.route("/")
        def home():
            self.camera_controller.detect_cameras()  # Refresh camera list
            return render_template("control_panel.html", 
                                  cameras=self.camera_controller.cameras, 
                                  selected_camera=self.camera_controller.camera_index)
        
        @self.app.route("/control", methods=["POST"])
        def control():
            action = request.form["action"]
            speed = float(request.form["speed"])
            camera_id = request.form.get("camera_id", type=int)
            
            if camera_id is not None:
                self.camera_controller.set_camera(camera_id)
                
            self.motor_controller.move(action, speed)
            return "OK"
        
        @self.app.route("/video_feed")
        def video_feed():
            return Response(self.camera_controller.generate_frames(), 
                           mimetype='multipart/x-mixed-replace; boundary=frame')
                           
        @self.app.route("/detection_info")
        def detection_info():
            # Placeholder for future object detection functionality
            return jsonify({"info": "Object detection not yet implemented"})
            
        @self.app.route("/system_info")
        def system_info():
            # Basic system information
            try:
                import psutil
                cpu = psutil.cpu_percent()
                memory = psutil.virtual_memory().percent
                temperature = None
                
                try:
                    # Try to get Raspberry Pi temperature
                    with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                        temperature = float(f.read()) / 1000.0
                except:
                    pass
                    
                return jsonify({
                    "cpu": cpu,
                    "memory": memory,
                    "temperature": temperature,
                    "cameras": self.camera_controller.cameras,
                    "active_camera": self.camera_controller.camera_index
                })
            except Exception as e:
                logger.error(f"Error getting system info: {e}")
                return jsonify({"error": "Failed to get system information"})
        
    def start(self):
        """Start the web server in a separate thread"""
        self.server_thread = threading.Thread(
            target=lambda: self.app.run(
                host=self.host, 
                port=self.port, 
                debug=False, 
                use_reloader=False,
                threaded=True
            ),
            daemon=True
        )
        self.server_thread.start()
        logger.info(f"Web server running at http://{self.host}:{self.port}")
        
    def cleanup(self):
        """Clean up web server resources"""
        logger.info("Web server shutting down")


class Robot:
    """Main robot class that coordinates all components"""
    
    def __init__(self):
        self.motor_controller = MotorController()
        self.camera_controller = CameraController()
        self.web_server = RobotWebServer(self.motor_controller, self.camera_controller)
        
        # Set up signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        logger.info("Robot initialized")
        
    def start(self):
        """Start all robot components"""
        self.web_server.start()
        logger.info("Robot started")
        
        # Keep the main thread alive
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.cleanup()
            
    def signal_handler(self, sig, frame):
        """Handle termination signals"""
        logger.info(f"Received signal {sig}, shutting down...")
        self.cleanup()
        sys.exit(0)
        
    def cleanup(self):
        """Clean up all resources"""
        self.camera_controller.cleanup()
        self.motor_controller.cleanup()
        self.web_server.cleanup()
        logger.info("Robot shutdown complete")


if __name__ == "__main__":
    robot = Robot()
    robot.start()

