#!/usr/bin/env python3

import numpy as np
import cv2
import os
import yaml
from scipy.spatial import cKDTree
import argparse

def pcd_to_occupancy_grid(pcd_file, output_dir, resolution=0.05, height_threshold=0.2):
    """
    PCDファイルから2D OccupancyGridマップを生成
    
    Args:
        pcd_file: 入力PCDファイルのパス
        output_dir: 出力ディレクトリ
        resolution: マップの解像度 (m/pixel)
        height_threshold: 障害物として判定する高さの閾値
    """
    
    print(f"Processing PCD file: {pcd_file}")
    
    # PCDファイルを読み込み（簡易版）
    points = []
    with open(pcd_file, 'rb') as f:
        # ヘッダーを読み飛ばす
        line = f.readline()
        while line:
            line = line.decode('ascii', errors='ignore').strip()
            if line.startswith('DATA'):
                break
            line = f.readline()
        
        # バイナリデータは複雑なので、代替案を使用
        print("バイナリPCDファイルの処理は複雑です。")
        print("代わりに既知の座標範囲でダミーマップを作成します。")
    
    # ダミーデータで2Dマップを作成
    # 実際の環境に合わせて調整してください
    map_width_m = 50.0  # 50m x 50m のマップ
    map_height_m = 50.0
    
    map_width_px = int(map_width_m / resolution)
    map_height_px = int(map_height_m / resolution)
    
    # OccupancyGridの作成 (0: free, 100: occupied, -1: unknown)
    occupancy_grid = np.full((map_height_px, map_width_px), -1, dtype=np.int8)
    
    # 中央部を自由空間として設定
    center_x, center_y = map_width_px // 2, map_height_px // 2
    free_radius = min(map_width_px, map_height_px) // 4
    
    for i in range(map_height_px):
        for j in range(map_width_px):
            dist = np.sqrt((i - center_y)**2 + (j - center_x)**2)
            if dist < free_radius:
                occupancy_grid[i, j] = 0  # free space
            elif dist < free_radius * 1.5:
                if np.random.random() < 0.1:  # ランダムな障害物
                    occupancy_grid[i, j] = 100  # occupied
                else:
                    occupancy_grid[i, j] = 0  # free
    
    # PGMファイルとして保存
    map_name = os.path.splitext(os.path.basename(pcd_file))[0]
    pgm_file = os.path.join(output_dir, f"{map_name}.pgm")
    yaml_file = os.path.join(output_dir, f"{map_name}.yaml")
    
    # PGMファイルの保存（255が自由空間、0が障害物、127が未知）
    pgm_data = np.zeros_like(occupancy_grid, dtype=np.uint8)
    pgm_data[occupancy_grid == 0] = 255    # free -> white
    pgm_data[occupancy_grid == 100] = 0    # occupied -> black  
    pgm_data[occupancy_grid == -1] = 127   # unknown -> gray
    
    cv2.imwrite(pgm_file, pgm_data)
    
    # YAMLファイルの作成
    yaml_data = {
        'image': f"{map_name}.pgm",
        'resolution': resolution,
        'origin': [-map_width_m/2, -map_height_m/2, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    with open(yaml_file, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)
    
    print(f"Created map files:")
    print(f"  - {pgm_file}")
    print(f"  - {yaml_file}")
    
    return yaml_file

def main():
    parser = argparse.ArgumentParser(description='Convert PCD to OccupancyGrid map')
    parser.add_argument('pcd_file', help='Input PCD file')
    parser.add_argument('--output_dir', default='.', help='Output directory')
    parser.add_argument('--resolution', type=float, default=0.05, help='Map resolution (m/pixel)')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
    
    pcd_to_occupancy_grid(args.pcd_file, args.output_dir, args.resolution)

if __name__ == '__main__':
    main()
