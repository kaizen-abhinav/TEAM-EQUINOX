#!/usr/bin/env python3
"""
Real-time UFLDv2 Lane Detection on Video
Uses CULane ResNet34 model with OpenCV overlay
"""

import torch
import cv2
import numpy as np
import time
import os

# Get the directory where this script is located
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def get_culane_config():
    """Return CULane ResNet34 configuration as a namespace object"""
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
        
        # Row anchors: y-coordinates from 0.42 to 1.0 (CULane format)
        row_anchor = np.linspace(0.42, 1, num_row)
        col_anchor = np.linspace(0, 1, num_col)
    
    return Config()


def load_model(cfg, weights_path):
    """Load the UFLDv2 model with pre-trained weights"""
    from model.model_culane import parsingNet
    
    # Create model
    net = parsingNet(
        pretrained=False,  # We'll load our own weights
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
    
    # Load weights
    state_dict = torch.load(weights_path, map_location='cpu')['model']
    compatible_state_dict = {}
    for k, v in state_dict.items():
        if 'module.' in k:
            compatible_state_dict[k[7:]] = v
        else:
            compatible_state_dict[k] = v
    
    net.load_state_dict(compatible_state_dict, strict=False)
    net = net.cuda()
    net.eval()
    
    return net


def pred2coords(pred, row_anchor, col_anchor, local_width=1, 
                original_image_width=1640, original_image_height=590):
    """Convert model predictions to lane coordinates
    
    Only uses row-based lanes (indices 1, 2) - left and right main lanes.
    """
    batch_size, num_grid_row, num_cls_row, num_lane_row = pred['loc_row'].shape

    max_indices_row = pred['loc_row'].argmax(1).cpu()
    valid_row = pred['exist_row'].argmax(1).cpu()

    pred['loc_row'] = pred['loc_row'].cpu()

    coords = []
    row_lane_idx = [1, 2]  # Only inner lanes (green and yellow)

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
    # Resize to intermediate size (before crop)
    input_height = int(cfg.train_height / cfg.crop_ratio)  # 320/0.6 = 533
    input_width = cfg.train_width  # 1600
    
    resized = cv2.resize(frame, (input_width, input_height))
    
    # Crop from top - keep only bottom portion (train_height pixels)
    crop_start = input_height - cfg.train_height
    cropped = resized[crop_start:, :, :]  # Shape: (320, 1600, 3)
    
    # Convert BGR to RGB
    rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
    
    # Normalize using ImageNet stats
    normalized = rgb.astype(np.float32) / 255.0
    normalized = (normalized - np.array([0.485, 0.456, 0.406])) / np.array([0.229, 0.224, 0.225])
    
    # Convert to tensor: HWC -> CHW
    tensor = torch.from_numpy(normalized.transpose(2, 0, 1)).float()
    
    # Add batch dimension
    tensor = tensor.unsqueeze(0)
    
    return tensor


def draw_lanes(frame, coords, colors=None):
    """Draw lane detections on frame"""
    if colors is None:
        colors = [
            (0, 255, 0),    # Green
            (0, 255, 255),  # Yellow
            (255, 0, 0),    # Blue
            (255, 0, 255),  # Magenta
        ]
    
    for idx, lane in enumerate(coords):
        color = colors[idx % len(colors)]
        
        # Draw points
        for point in lane:
            cv2.circle(frame, point, 5, color, -1)
        
        # Draw lines connecting points
        if len(lane) > 1:
            for i in range(len(lane) - 1):
                cv2.line(frame, lane[i], lane[i + 1], color, 2)
    
    return frame


def main():
    # Configuration - use CULane ResNet34
    cfg = get_culane_config()
    weights_path = os.path.join(SCRIPT_DIR, 'weights', 'culane_res34.pth')
    video_path = os.path.join(SCRIPT_DIR, 'example.mp4')
    
    print(f"Loading model from: {weights_path}")
    print(f"Video file: {video_path}")
    print(f"Using CULane config: {cfg.train_width}x{cfg.train_height}, crop_ratio={cfg.crop_ratio}")
    
    # Load model
    model = load_model(cfg, weights_path)
    print("Model loaded successfully!")
    
    # Open video
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Cannot open video file {video_path}")
        return
    
    # Get video properties
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    print(f"Video: {frame_width}x{frame_height} @ {fps:.1f} FPS, {total_frames} frames")
    print("Press 'q' to quit, 'p' to pause/resume")
    
    # Create window
    cv2.namedWindow('UFLDv2 Lane Detection', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('UFLDv2 Lane Detection', 1280, 720)
    
    paused = False
    frame_count = 0
    fps_smooth = 0
    
    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                print("End of video. Looping...")
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            
            frame_count += 1
            start_time = time.time()
            
            # Preprocess
            input_tensor = preprocess_frame(frame, cfg).cuda()
            
            # Inference
            with torch.no_grad():
                pred = model(input_tensor)
            
            # Convert predictions to coordinates
            coords = pred2coords(
                pred, 
                cfg.row_anchor, 
                cfg.col_anchor,
                original_image_width=frame_width,
                original_image_height=frame_height
            )
            
            # Draw lanes on frame
            vis_frame = draw_lanes(frame.copy(), coords)
            
            # Calculate FPS
            inference_time = time.time() - start_time
            current_fps = 1.0 / inference_time if inference_time > 0 else 0
            fps_smooth = 0.9 * fps_smooth + 0.1 * current_fps
            
            # Add info overlay
            info_text = f"FPS: {fps_smooth:.1f} | Frame: {frame_count}/{total_frames} | Lanes: {len(coords)}"
            cv2.putText(vis_frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(vis_frame, "CULane ResNet34 | Press 'q' to quit, 'p' to pause", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Display
        cv2.imshow('UFLDv2 Lane Detection', vis_frame)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            paused = not paused
            print("Paused" if paused else "Resumed")
    
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    print("Done!")


if __name__ == '__main__':
    main()
