"""
UFLDv2 Lane Detection Module - VRAM Optimized
(ROS2 compatible version)

This module provides a clean interface to the Ultra-Fast-Lane-Detection-v2 model.
Optimized for reduced VRAM usage and maximum inference performance.

Usage:
    from lka.ufldv2_detector import UFLDv2Detector
    
    detector = UFLDv2Detector()
    detector.load_model()
    result = detector.detect(frame)
"""

import sys
import os
import numpy as np
import cv2
import torch

# CUDA optimizations - set before any CUDA operations
torch.backends.cudnn.benchmark = True  # Optimize convolutions
torch.backends.cuda.matmul.allow_tf32 = True  # Enable TF32 for tensor cores
torch.backends.cudnn.allow_tf32 = True

# Resolve UFLDv2 repo path
# When running as a ROS2 node, __file__ points to the install/ directory.
# We'll use the environment variable if set, otherwise we'll resolve relative
# to the current working directory assuming the workspace structure.
import rclpy
from ament_index_python.packages import get_package_share_directory

try:
    # Try to find the package share directory
    pkg_dir = get_package_share_directory('lka')
    # pkg_dir is install/lka/share/lka
    # Workspace root is 4 levels up from pkg_dir: lka -> share -> lka -> install -> ws_root
    ws_root = os.path.abspath(os.path.join(pkg_dir, '..', '..', '..', '..'))
    UFLDV2_DIR_GUESS = os.path.join(ws_root, 'Ultra-Fast-Lane-Detection-v2')
except Exception:
    # Fallback to current directory logic
    UFLDV2_DIR_GUESS = os.path.abspath(os.path.join(os.getcwd(), 'Ultra-Fast-Lane-Detection-v2'))

if 'UFLDV2_DIR' in os.environ:
    UFLDV2_DIR = os.environ['UFLDV2_DIR']
else:
    UFLDV2_DIR = UFLDV2_DIR_GUESS

if not os.path.isdir(UFLDV2_DIR):
    raise RuntimeError(f"Could not find UFLDv2 repo at {UFLDV2_DIR}. Please set UFLDV2_DIR env var.")

if UFLDV2_DIR not in sys.path:
    sys.path.insert(0, UFLDV2_DIR)

# Now we can import from the UFLDv2 repo
from model.model_culane import parsingNet


class CULaneConfig:
    """CULane ResNet34 configuration"""
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


class TuSimpleConfig:
    """TuSimple ResNet34 configuration"""
    dataset = 'Tusimple'
    backbone = '34'
    num_lanes = 4
    num_row = 56
    num_col = 41
    num_cell_row = 100
    num_cell_col = 100
    train_width = 800
    train_height = 320
    crop_ratio = 0.8
    use_aux = False
    fc_norm = False
    
    row_anchor = np.linspace(160, 710, num_row) / 720
    col_anchor = np.linspace(0, 1, num_col)


class UFLDv2Detector:
    """
    UFLDv2 Lane Detection wrapper class - VRAM Optimized.
    
    Provides a clean interface for lane detection using the UFLDv2 model.
    Optimizations:
    - Reusable preprocessing buffer
    - torch.inference_mode() for efficient inference
    - cuDNN benchmark mode enabled
    - Explicit memory management
    """
    
    def __init__(self, model_type='culane', use_fp16=True, max_vram_fraction=None):
        """
        Initialize the UFLDv2 detector.
        
        Args:
            model_type: 'culane' or 'tusimple'
            use_fp16: Use half-precision (FP16) for reduced memory usage
            max_vram_fraction: Optional, limit VRAM usage (0.0-1.0)
        """
        self.model_type = model_type
        self.use_fp16 = use_fp16
        
        # Optional VRAM limit
        if max_vram_fraction is not None:
            torch.cuda.set_per_process_memory_fraction(max_vram_fraction)
        
        # Set config based on model type
        if model_type == 'culane':
            self.cfg = CULaneConfig()
            weights_name = 'culane_res34.pth'
        elif model_type == 'tusimple':
            self.cfg = TuSimpleConfig()
            weights_name = 'tusimple_res34.pth'
        else:
            raise ValueError(f"Unknown model type: {model_type}")
        
        self.weights_path = os.path.join(UFLDV2_DIR, 'weights', weights_name)
        self.model = None
        
        # Reusable preprocessing buffers (allocated on first use)
        self._input_buffer = None
        self._mean = None
        self._std = None
        
    def load_model(self):
        """Load the UFLDv2 model with pre-trained weights"""
        traced_path = os.path.join(UFLDV2_DIR, 'weights', 'ufldv2_traced.pt')
        
        if os.path.exists(traced_path):
            self.model = torch.jit.load(traced_path)
            # Move to GPU and set eval mode
            self.model = self.model.cuda()
            if self.use_fp16:
                self.model = self.model.half()
            self.model.eval()
        else:
            # Create model
            self.model = parsingNet(
                pretrained=False,
                backbone=self.cfg.backbone,
                num_grid_row=self.cfg.num_cell_row,
                num_cls_row=self.cfg.num_row,
                num_grid_col=self.cfg.num_cell_col,
                num_cls_col=self.cfg.num_col,
                num_lane_on_row=self.cfg.num_lanes,
                num_lane_on_col=self.cfg.num_lanes,
                use_aux=self.cfg.use_aux,
                input_height=self.cfg.train_height,
                input_width=self.cfg.train_width,
                fc_norm=self.cfg.fc_norm
            )
            
            # Load weights
            state_dict = torch.load(self.weights_path, map_location='cpu', weights_only=False)['model']
            compatible_state_dict = {}
            for k, v in state_dict.items():
                if 'module.' in k:
                    compatible_state_dict[k[7:]] = v
                else:
                    compatible_state_dict[k] = v
            
            self.model.load_state_dict(compatible_state_dict, strict=False)
            
            # Move to GPU with optional FP16
            if self.use_fp16:
                self.model = self.model.cuda().half()
            else:
                self.model = self.model.cuda()
            
            self.model.eval()
        
        # Pre-allocate normalization tensors on GPU
        dtype = torch.float16 if self.use_fp16 else torch.float32
        self._mean = torch.tensor([0.485, 0.456, 0.406], device='cuda', dtype=dtype).view(3, 1, 1)
        self._std = torch.tensor([0.229, 0.224, 0.225], device='cuda', dtype=dtype).view(3, 1, 1)
        
        # Clear any cached memory from model loading
        torch.cuda.empty_cache()
        
        return self
    
    def preprocess(self, frame):
        """Preprocess a frame for model inference - optimized with buffer reuse"""
        input_height = int(self.cfg.train_height / self.cfg.crop_ratio)
        input_width = self.cfg.train_width
        
        # Resize and crop using OpenCV (CPU - fast)
        resized = cv2.resize(frame, (input_width, input_height))
        crop_start = input_height - self.cfg.train_height
        cropped = resized[crop_start:, :, :]
        
        # Convert to RGB and normalize
        rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
        
        # Convert to tensor and move to GPU in one operation
        tensor = torch.from_numpy(rgb.transpose(2, 0, 1)).float().div_(255.0)
        tensor = tensor.unsqueeze(0).cuda()
        
        if self.use_fp16:
            tensor = tensor.half()
        
        # Normalize using pre-allocated tensors on GPU
        tensor = (tensor - self._mean) / self._std
        
        return tensor
    
    def _pred2coords(self, pred, original_image_width, original_image_height, local_width=1):
        """
        Convert model predictions to lane coordinates using full hybrid architecture.
        
        Uses both row anchors (horizontal scan lines) and column anchors (vertical scan lines)
        for robust lane detection on curves and varying lane orientations.
        
        Returns:
            dict with 'primary' (row anchor lanes) and 'secondary' (column anchor lanes)
        """
        batch_size, num_grid_row, num_cls_row, num_lane_row = pred['loc_row'].shape
        batch_size, num_grid_col, num_cls_col, num_lane_col = pred['loc_col'].shape

        # Row anchor processing - move to CPU once
        max_indices_row = pred['loc_row'].argmax(1).cpu()
        valid_row = pred['exist_row'].argmax(1).cpu()
        loc_row_cpu = pred['loc_row'].cpu()
        
        # Column anchor processing - move to CPU once
        max_indices_col = pred['loc_col'].argmax(1).cpu()
        valid_col = pred['exist_col'].argmax(1).cpu()
        loc_col_cpu = pred['loc_col'].cpu()

        primary_lanes = []  # Row-based lanes for display and steering
        secondary_lanes = []  # Column-based lanes for path planning
        
        # Row-based lanes (indices 1, 2) - primary inner lanes
        row_lane_idx = [1, 2]
        for i in row_lane_idx:
            tmp = []
            if valid_row[0, :, i].sum() > num_cls_row / 6:
                for k in range(valid_row.shape[1]):
                    if valid_row[0, k, i]:
                        all_ind = torch.tensor(list(range(
                            max(0, max_indices_row[0, k, i] - local_width),
                            min(num_grid_row - 1, max_indices_row[0, k, i] + local_width) + 1
                        )))
                        
                        out_tmp = (loc_row_cpu[0, all_ind, k, i].softmax(0) * all_ind.float()).sum() + 0.5
                        out_tmp = out_tmp / (num_grid_row - 1) * original_image_width
                        tmp.append((int(out_tmp), int(self.cfg.row_anchor[k] * original_image_height)))
                if len(tmp) >= 2:
                    primary_lanes.append(tmp)

        # Column-based lanes (indices 0, 3) - outer lanes / curved segments
        col_lane_idx = [0, 3]
        for i in col_lane_idx:
            tmp = []
            if valid_col[0, :, i].sum() > num_cls_col / 8:
                for k in range(valid_col.shape[1]):
                    if valid_col[0, k, i]:
                        all_ind = torch.tensor(list(range(
                            max(0, max_indices_col[0, k, i] - local_width),
                            min(num_grid_col - 1, max_indices_col[0, k, i] + local_width) + 1
                        )))
                        
                        out_tmp = (loc_col_cpu[0, all_ind, k, i].softmax(0) * all_ind.float()).sum() + 0.5
                        out_tmp = out_tmp / (num_grid_col - 1) * original_image_height
                        tmp.append((int(self.cfg.col_anchor[k] * original_image_width), int(out_tmp)))
                if len(tmp) >= 2:
                    secondary_lanes.append(tmp)

        return {
            'primary': primary_lanes,      # Row anchor lanes (for display & steering)
            'secondary': secondary_lanes,  # Column anchor lanes (for path planning)
            'all': primary_lanes + secondary_lanes  # Combined for backwards compatibility
        }
    
    def detect(self, frame):
        """
        Detect lanes in a frame using full hybrid architecture.
        
        Args:
            frame: BGR image (numpy array)
            
        Returns:
            dict with:
                'primary': List of primary lane coords (row anchors) - for display/steering
                'secondary': List of secondary lane coords (column anchors) - for path planning
                'all': Combined list of all lanes
        """
        if self.model is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")
        
        height, width = frame.shape[:2]
        
        # Preprocess
        input_tensor = self.preprocess(frame)
        
        # Inference with torch.inference_mode() for better memory efficiency
        with torch.inference_mode():
            pred = self.model(input_tensor)
        
        # Convert to coordinates using hybrid approach
        result = self._pred2coords(pred, width, height)
        
        # Explicit cleanup to prevent memory accumulation
        del input_tensor, pred
        
        return result
    
    @staticmethod
    def draw_lanes(frame, lane_result, colors=None, show_secondary=False):
        """
        Draw detected lanes on a frame.
        
        Args:
            frame: BGR image (numpy array)
            lane_result: Result from detect() - either dict or list
            colors: Optional list of BGR colors
            show_secondary: If True, also draw secondary (column anchor) lanes
            
        Returns:
            Frame with lanes drawn
        """
        if colors is None:
            colors = [(0, 255, 0), (0, 255, 255)]  # Green, Yellow for primary lanes
        
        # Handle both dict and list input for backwards compatibility
        if isinstance(lane_result, dict):
            lanes_to_draw = lane_result['primary']
            if show_secondary:
                lanes_to_draw = lane_result['all']
        else:
            lanes_to_draw = lane_result
        
        vis_frame = frame.copy()
        
        for idx, lane in enumerate(lanes_to_draw):
            color = colors[idx % len(colors)]
            for point in lane:
                cv2.circle(vis_frame, point, 5, color, -1)
            if len(lane) > 1:
                for i in range(len(lane) - 1):
                    cv2.line(vis_frame, lane[i], lane[i + 1], color, 2)
        
        return vis_frame
    
    @staticmethod
    def get_vram_usage():
        """Get current VRAM usage statistics"""
        if torch.cuda.is_available():
            allocated = torch.cuda.memory_allocated() / 1024**2  # MB
            reserved = torch.cuda.memory_reserved() / 1024**2  # MB
            return {'allocated_mb': allocated, 'reserved_mb': reserved}
        return {'allocated_mb': 0, 'reserved_mb': 0}
