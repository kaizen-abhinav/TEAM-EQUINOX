#!/usr/bin/env python3
"""
UFLDv2 Lane Detection in CARLA Simulator - Teleop Mode
Uses CULane ResNet34 model with CARLA autopilot steering + manual throttle/brake

Controls:
    W / UP      - Accelerate
    S / DOWN    - Brake/Reverse
    SPACE       - Handbrake
    Q           - Quit

Usage:
    1. Start CARLA server: ./CarlaUE4.sh
    2. Run this script: python3 carla_lane_detection.py
"""

import torch
import cv2
import numpy as np
import time
import sys
import os

# Get the directory where this script is located
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Add CARLA Python API to path (adjust path as needed)
try:
    sys.path.append('/opt/carla-simulator/PythonAPI/carla')
    sys.path.append('/opt/carla-simulator/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg')
except IndexError:
    pass

import carla


# ============================================================================
# UFLDv2 Lane Detection Functions
# ============================================================================

def get_culane_config():
    """Return CULane ResNet34 configuration"""
    class Config:
        dataset = 'CULane'
        backbone = '34'
        num_lanes = 4
        num_row = 72
        num_col = 81
        num_cell_row = 200
        num_cell_col = 100
        train_width = 1600
        train_height = 320
        crop_ratio = 0.6
        use_aux = False
        fc_norm = True
        
        row_anchor = np.linspace(0.42, 1, num_row)
        col_anchor = np.linspace(0, 1, num_col)
    
    return Config()


def load_model(cfg, weights_path):
    """Load the UFLDv2 model with pre-trained weights"""
    from model.model_culane import parsingNet
    
    net = parsingNet(
        pretrained=False,
        backbone=cfg.backbone,
        num_grid_row=cfg.num_cell_row,
        num_cls_row=cfg.num_row,
        num_grid_col=cfg.num_cell_col,
        num_cls_col=cfg.num_col,
        num_lane_on_row=cfg.num_lanes,
        num_lane_on_col=cfg.num_lanes,
        use_aux=cfg.use_aux,
        input_height=cfg.train_height,
        input_width=cfg.train_width,
        fc_norm=cfg.fc_norm
    )
    
    state_dict = torch.load(weights_path, map_location='cpu')['model']
    compatible_state_dict = {}
    for k, v in state_dict.items():
        if 'module.' in k:
            compatible_state_dict[k[7:]] = v
        else:
            compatible_state_dict[k] = v
    
    net.load_state_dict(compatible_state_dict, strict=False)
    
    # Use half precision (FP16) to reduce GPU memory usage
    net = net.cuda().half()
    net.eval()
    
    # Clear CUDA cache
    torch.cuda.empty_cache()
    
    return net


def pred2coords(pred, row_anchor, col_anchor, local_width=1, 
                original_image_width=1280, original_image_height=720):
    """Convert model predictions to lane coordinates"""
    batch_size, num_grid_row, num_cls_row, num_lane_row = pred['loc_row'].shape

    max_indices_row = pred['loc_row'].argmax(1).cpu()
    valid_row = pred['exist_row'].argmax(1).cpu()

    pred['loc_row'] = pred['loc_row'].cpu()

    coords = []
    row_lane_idx = [1, 2]

    for i in row_lane_idx:
        tmp = []
        if valid_row[0, :, i].sum() > num_cls_row / 2:
            for k in range(valid_row.shape[1]):
                if valid_row[0, k, i]:
                    all_ind = torch.tensor(list(range(
                        max(0, max_indices_row[0, k, i] - local_width),
                        min(num_grid_row - 1, max_indices_row[0, k, i] + local_width) + 1
                    )))
                    
                    out_tmp = (pred['loc_row'][0, all_ind, k, i].softmax(0) * all_ind.float()).sum() + 0.5
                    out_tmp = out_tmp / (num_grid_row - 1) * original_image_width
                    tmp.append((int(out_tmp), int(row_anchor[k] * original_image_height)))
            coords.append(tmp)

    return coords


def preprocess_frame(frame, cfg):
    """Preprocess a frame for model inference"""
    input_height = int(cfg.train_height / cfg.crop_ratio)
    input_width = cfg.train_width
    
    resized = cv2.resize(frame, (input_width, input_height))
    crop_start = input_height - cfg.train_height
    cropped = resized[crop_start:, :, :]
    
    rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
    normalized = rgb.astype(np.float32) / 255.0
    normalized = (normalized - np.array([0.485, 0.456, 0.406])) / np.array([0.229, 0.224, 0.225])
    
    tensor = torch.from_numpy(normalized.transpose(2, 0, 1)).float()
    tensor = tensor.unsqueeze(0)
    
    return tensor


def draw_lanes(frame, coords, colors=None):
    """Draw lane detections on frame"""
    if colors is None:
        colors = [(0, 255, 0), (0, 255, 255)]
    
    for idx, lane in enumerate(coords):
        color = colors[idx % len(colors)]
        for point in lane:
            cv2.circle(frame, point, 5, color, -1)
        if len(lane) > 1:
            for i in range(len(lane) - 1):
                cv2.line(frame, lane[i], lane[i + 1], color, 2)
    
    return frame


# ============================================================================
# CARLA Setup Functions
# ============================================================================

class CarlaCameraManager:
    """Manages CARLA camera sensor and image capture"""
    
    def __init__(self, world, vehicle, width=1280, height=720):
        self.world = world
        self.vehicle = vehicle
        self.width = width
        self.height = height
        self.image = None
        self.camera = None
        
    def setup_camera(self):
        """Create and attach camera to vehicle"""
        blueprint_library = self.world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(self.width))
        camera_bp.set_attribute('image_size_y', str(self.height))
        camera_bp.set_attribute('fov', '90')
        
        camera_transform = carla.Transform(
            carla.Location(x=2.0, z=1.4),
            carla.Rotation(pitch=-10)
        )
        
        self.camera = self.world.spawn_actor(
            camera_bp, 
            camera_transform, 
            attach_to=self.vehicle
        )
        self.camera.listen(self._process_image)
        
        return self.camera
    
    def _process_image(self, image):
        """Callback for camera sensor"""
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((self.height, self.width, 4))
        self.image = array[:, :, :3]
    
    def get_image(self):
        """Get the latest camera image"""
        return self.image
    
    def destroy(self):
        """Clean up camera sensor"""
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()


class TeleopController:
    """
    Teleop controller: waypoint-based steering + manual throttle/brake
    
    Steering is calculated from CARLA waypoints (follows road).
    Throttle and brake are controlled by keyboard.
    """
    
    def __init__(self, vehicle, world):
        self.vehicle = vehicle
        self.world = world
        self.map = world.get_map()
        
        # Control state
        self.throttle = 0.0
        self.brake = 0.0
        self.hand_brake = False
        
        # Steering parameters
        self.lookahead_distance = 10.0  # meters ahead to look for waypoint
    
    def process_key(self, key):
        """Process keyboard input for throttle/brake"""
        throttle_delta = 0.15
        brake_delta = 0.3
        
        if key == ord('w') or key == 82:  # W or UP arrow
            self.throttle = min(1.0, self.throttle + throttle_delta)
            self.brake = 0.0
        elif key == ord('s') or key == 84:  # S or DOWN arrow
            self.throttle = 0.0
            self.brake = min(1.0, self.brake + brake_delta)
        elif key == ord(' '):  # SPACE - handbrake
            self.hand_brake = not self.hand_brake
        elif key == 255:  # No key pressed - gradual release
            self.throttle = max(0.0, self.throttle - 0.03)
            self.brake = max(0.0, self.brake - 0.08)
    
    def calculate_steering(self):
        """Calculate steering angle to follow road waypoints"""
        # Get vehicle transform
        vehicle_transform = self.vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_rotation = vehicle_transform.rotation
        
        # Get current waypoint
        current_waypoint = self.map.get_waypoint(vehicle_location)
        if current_waypoint is None:
            return 0.0
        
        # Get waypoint ahead
        waypoints_ahead = current_waypoint.next(self.lookahead_distance)
        if not waypoints_ahead:
            return 0.0
        
        target_waypoint = waypoints_ahead[0]
        target_location = target_waypoint.transform.location
        
        # Calculate angle to target
        dx = target_location.x - vehicle_location.x
        dy = target_location.y - vehicle_location.y
        
        # Target angle in world coordinates
        target_angle = np.arctan2(dy, dx) * 180.0 / np.pi
        
        # Vehicle's current yaw
        vehicle_yaw = vehicle_rotation.yaw
        
        # Angle difference
        angle_diff = target_angle - vehicle_yaw
        
        # Normalize to [-180, 180]
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        # Convert to steering (-1 to 1)
        # Negative angle = steer right, Positive angle = steer left
        steer = angle_diff / 45.0  # 45 degrees = full lock
        steer = np.clip(steer, -1.0, 1.0)
        
        return steer
    
    def apply_control(self):
        """Apply teleop control: waypoint steering + manual throttle/brake"""
        # Calculate steering from waypoints
        steer = self.calculate_steering()
        
        # Get current speed
        velocity = self.vehicle.get_velocity()
        speed_kmh = 3.6 * np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # Speed limit: 20 km/h
        max_speed = 20.0
        throttle = self.throttle
        if speed_kmh >= max_speed:
            throttle = 0.0  # Cut throttle when at/above speed limit
        
        # Create control
        control = carla.VehicleControl()
        control.steer = steer
        control.throttle = throttle
        control.brake = self.brake
        control.hand_brake = self.hand_brake
        
        # Apply control
        self.vehicle.apply_control(control)
        
        return control
    
    def destroy(self):
        """Cleanup"""
        pass


def spawn_vehicle(world, spawn_point=None):
    """Spawn a vehicle in the CARLA world"""
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
    
    if spawn_point is None:
        spawn_points = world.get_map().get_spawn_points()
        spawn_point = spawn_points[0] if spawn_points else carla.Transform()
    
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    return vehicle


def main():
    """Main function to run lane detection in CARLA with teleop"""
    
    # Configuration
    cfg = get_culane_config()
    weights_path = os.path.join(SCRIPT_DIR, 'weights', 'culane_res34.pth')
    
    print("=" * 60)
    print("UFLDv2 Lane Detection in CARLA - TELEOP MODE")
    print("=" * 60)
    print("Controls:")
    print("  W / UP    - Accelerate")
    print("  S / DOWN  - Brake")
    print("  SPACE     - Handbrake")
    print("  Q         - Quit")
    print("=" * 60)
    
    # Connect to CARLA FIRST (before loading PyTorch model to avoid GPU memory conflicts)
    print("\nConnecting to CARLA server...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)  # Longer timeout for map loading
    
    # Load Town04 map
    print("Loading Town04 map (this may take a moment)...")
    client.load_world('Town04')
    
    # Wait for map to fully load
    time.sleep(2.0)
    
    world = client.get_world()
    print(f"Connected to CARLA! Map: {world.get_map().name}")
    
    # Now load the lane detection model
    print(f"\nLoading model from: {weights_path}")
    model = load_model(cfg, weights_path)
    print("Model loaded successfully!")
    
    # Set synchronous mode
    settings = world.get_settings()
    original_settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    
    vehicle = None
    camera_manager = None
    teleop = None
    
    try:
        # Spawn vehicle
        print("Spawning vehicle...")
        vehicle = spawn_vehicle(world)
        print(f"Spawned: {vehicle.type_id}")
        
        # Setup camera
        camera_manager = CarlaCameraManager(world, vehicle, width=1280, height=720)
        camera_manager.setup_camera()
        print("Camera attached!")
        
        # Setup teleop controller
        teleop = TeleopController(vehicle, world)
        print("Teleop controller ready! (Autopilot steering + manual throttle/brake)")
        
        # Create display window
        cv2.namedWindow('CARLA UFLDv2 Lane Detection - TELEOP', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('CARLA UFLDv2 Lane Detection - TELEOP', 1280, 720)
        
        print("\nRunning... Press 'q' to quit")
        print("-" * 60)
        
        frame_count = 0
        fps_smooth = 0
        
        while True:
            # Advance simulation
            world.tick()
            
            # Get camera image
            image = camera_manager.get_image()
            if image is None:
                continue
            
            frame_count += 1
            start_time = time.time()
            
            # Make a copy for visualization
            frame = image.copy()
            
            # Run lane detection
            input_tensor = preprocess_frame(frame, cfg).cuda().half()
            
            with torch.no_grad():
                pred = model(input_tensor)
            
            coords = pred2coords(
                pred, 
                cfg.row_anchor, 
                cfg.col_anchor,
                original_image_width=1280,
                original_image_height=720
            )
            
            # Draw lanes
            vis_frame = draw_lanes(frame.copy(), coords)
            
            # Get vehicle info
            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            
            # Handle key presses BEFORE applying control
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\nQuitting...")
                break
            
            # Process teleop input
            teleop.process_key(key)
            
            # Apply control (autopilot steering + manual throttle/brake)
            control = teleop.apply_control()
            
            # Calculate FPS
            inference_time = time.time() - start_time
            current_fps = 1.0 / inference_time if inference_time > 0 else 0
            fps_smooth = 0.9 * fps_smooth + 0.1 * current_fps
            
            # Draw info overlay
            info_text = f"FPS: {fps_smooth:.1f} | Speed: {speed_kmh:.1f} km/h | Lanes: {len(coords)}"
            cv2.putText(vis_frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            control_text = f"Steer: {control.steer:.2f} (AUTO) | Throttle: {control.throttle:.2f} | Brake: {control.brake:.2f}"
            cv2.putText(vis_frame, control_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            mode_text = "TELEOP: Autopilot Steering + Manual Throttle/Brake"
            cv2.putText(vis_frame, mode_text, (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            
            controls_text = "W=Accel | S=Brake | SPACE=Handbrake | Q=Quit"
            cv2.putText(vis_frame, controls_text, (10, 115),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            # Display
            cv2.imshow('CARLA UFLDv2 Lane Detection - TELEOP', vis_frame)
    
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        print("\nCleaning up...")
        
        cv2.destroyAllWindows()
        
        if teleop:
            teleop.destroy()
        
        if camera_manager:
            camera_manager.destroy()
        
        if vehicle:
            vehicle.set_autopilot(False)
            vehicle.destroy()
        
        # Restore original settings
        world.apply_settings(original_settings)
        
        print("Done!")


if __name__ == '__main__':
    main()
