import numpy as np

class LaneKalmanFilter:
    """
    A 2D Kalman Filter for tracking lane polynomial coefficients.
    Assumes a 2nd degree polynomial: x = a*y^2 + b*y + c
    State vector [a, b, c, a_dot, b_dot, c_dot]
    """
    def __init__(self, dt=0.033):
        self.dt = dt
        
        # State vector: [a, b, c, da/dt, db/dt, dc/dt]
        self.x = np.zeros((6, 1))
        
        # State transition matrix
        self.F = np.array([
            [1, 0, 0, dt, 0,  0 ],
            [0, 1, 0, 0,  dt, 0 ],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0 ],
            [0, 0, 0, 0,  1,  0 ],
            [0, 0, 0, 0,  0,  1 ]
        ])
        
        # Measurement matrix (we only measure a, b, c)
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        
        # Initial Covariance Matrix
        self.P = np.eye(6) * 10.0
        
        # Process Noise Covariance
        # Tune these to trust the model more (lower values) or less (higher values)
        self.Q = np.eye(6) * 0.1
        
        # Measurement Noise Covariance
        # Tune these to trust the measurement more (lower) or less (higher)
        # We expect a bit of jitter, so we don't trust measurements perfectly
        self.R = np.eye(3) * 5.0
        
        self.initialized = False
        
    def predict(self):
        """Predict the next state."""
        if not self.initialized:
            return self.x[:3].flatten()
            
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x[:3].flatten()
        
    def update(self, measurement):
        """Update the state with a new measurement."""
        z = np.array(measurement).reshape(3, 1)
        
        if not self.initialized:
            self.x[:3] = z
            self.initialized = True
            return self.x[:3].flatten()
            
        # Kalman Gain
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # Update State
        y = z - np.dot(self.H, self.x)
        self.x = self.x + np.dot(K, y)
        
        # Update Covariance
        I = np.eye(6)
        self.P = np.dot(I - np.dot(K, self.H), self.P)
        
        return self.x[:3].flatten()

class DualLaneTracker:
    """Manages two Kalman filters for the left and right lanes."""
    def __init__(self):
        self.left_kf = LaneKalmanFilter()
        self.right_kf = LaneKalmanFilter()
        self.missing_frames = 0
        self.max_missing = 15 # Reset if we lose the lane for too long
        
    def process_lanes(self, lanes, image_height):
        """
        Takes a list of raw lane points (lists of (x,y)), fits polynomials,
        smooths them with Kalman filters, and returns smoothed points.
        """
        # 1. Identify left and right lanes based on average x position
        left_lane = None
        right_lane = None
        
        # Simple heuristic: sort by average x
        valid_lanes = [l for l in lanes if len(l) > 5]
        valid_lanes.sort(key=lambda l: np.mean([p[0] for p in l]))
        
        if len(valid_lanes) == 1:
            # Guess if it's left or right based on position
            avg_x = np.mean([p[0] for p in valid_lanes[0]])
            # Assume image width ~640 or so, middle is ~320. 
            # We'll use a rough heuristic, or just rely on the prediction for the missing one.
            if avg_x < 320:
                left_lane = valid_lanes[0]
            else:
                right_lane = valid_lanes[0]
        elif len(valid_lanes) >= 2:
            left_lane = valid_lanes[0]
            right_lane = valid_lanes[-1] # Usually the right-most
            
        smoothed_lanes = []
        
        # 2. Process Left Lane
        left_points = self._update_lane(left_lane, self.left_kf, image_height)
        if left_points is not None:
            smoothed_lanes.append(left_points)
            
        # 3. Process Right Lane
        right_points = self._update_lane(right_lane, self.right_kf, image_height)
        if right_points is not None:
            smoothed_lanes.append(right_points)
            
        # Handle resets
        if left_lane is None and right_lane is None:
            self.missing_frames += 1
            if self.missing_frames > self.max_missing:
                self.left_kf.initialized = False
                self.right_kf.initialized = False
        else:
            self.missing_frames = 0
            
        return smoothed_lanes

    def _update_lane(self, raw_lane, kf, img_h):
        # We will generate y values from bottom of screen upwards
        # CarMaker resolution is usually 640x480 or similar.
        y_eval = np.linspace(img_h * 0.4, img_h, 30) # Evaluate bottom 60% of image
        
        kf.predict() # Always predict forward
        
        if raw_lane is not None and len(raw_lane) > 3:
            # Fit polynomial: x = a*y^2 + b*y + c
            pts = np.array(raw_lane)
            x = pts[:, 0]
            y = pts[:, 1]
            try:
                poly_coeffs = np.polyfit(y, x, 2)
                # Update Kalman Filter
                smoothed_coeffs = kf.update(poly_coeffs)
            except np.linalg.LinAlgError:
                # Fallback to prediction if fit fails
                smoothed_coeffs = kf.x[:3].flatten()
        else:
            if not kf.initialized:
                return None
            # Use pure prediction
            smoothed_coeffs = kf.x[:3].flatten()
            
        # Generate smoothed points
        a, b, c = smoothed_coeffs
        x_eval = a * y_eval**2 + b * y_eval + c
        
        # Return as list of (int(x), int(y)) tuples
        return [(int(x), int(y)) for x, y in zip(x_eval, y_eval)]