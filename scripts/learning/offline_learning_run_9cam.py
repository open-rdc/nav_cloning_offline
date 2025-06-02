#!/usr/bin/env python3
import os
import sys
import csv
import time
import cv2
import roslib
import random
import numpy as np
import torch
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../pytorch'))
from nav_cloning_pytorch import *

class CourseFollowingLearningNode:
    def __init__(self):
        self.dl = deep_learning(n_action=1)

        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.model_num = str(sys.argv[1])
        self.pro = "20250517_12:49:45"  # データセットの識別名
        self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/'
        self.save_path = self.path + f"model/{self.pro}/model{self.model_num}.pt"
        self.ang_path = self.path + f"ang/{self.pro}"
        self.img_path = self.path + f"img/{self.pro}"
        self.loss_path =  self.path + f"loss/{self.pro}/{self.pro}.csv"

        self.data = 645  # 使用するデータ数
        self.BATCH_SIZE = 16 # バッチサイズを指定
        self.EPOCHS = 400 # エポック数を指定
        
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        os.makedirs(self.path + f"/loss/{self.pro}/", exist_ok=True)
    
    def load_images(self, index):
        shifts = {
            (1, 'center'):  0.022552916,
            (1, 'left')  : -0.0486526322458927,
            (1, 'right') :  0.0861816156644032,
            (2, 'center'): -0.289712051031864,
            (2, 'left')  : -0.252415274370429,
            (2, 'right') : -0.302315412659561,
            (3, 'center'):  0.289517402543431,
            (3, 'left')  :  0.296489666115526,
            (3, 'right') :  0.277030770948767
        }
        
        img_types = ["center", "left", "right"]
        images = []

        for lane in range(1,4): #lane1~3
            for img_type in img_types:
                angle_shift = shifts[(lane, img_type)]
                img_file = f"{self.img_path}/lane{lane}_{img_type}_{index}.jpg"
                img = cv2.imread(img_file)
                
                if img is None:
                    print(f"Warning: Failed to load {img_file}")
                    continue

                images.append((img, angle_shift))

        return images
        
    def load_angles(self):
        angles = []
        with open(self.ang_path + '/ang.csv', 'r') as f:
            for row in csv.reader(f):
                _, tar_ang = row
                angles.append(float(tar_ang))
        return angles

    def save_loss(self, loss_log):
        model_index = int(self.model_num) + 1  # 0列目はEpoch番号にするため+1
        epochs = len(loss_log)

        # 既存のloss.csvを読み込み（または初期化）
        if os.path.exists(self.loss_path):
            with open(self.loss_path, 'r') as fr:
                lines = list(csv.reader(fr))
        else:
            lines = []

        # loss_log からloss値だけを取り出す
        loss_values = [row[0] for row in loss_log]

        # 行数（エポック数）分用意（不足分は空行として補完）
        while len(lines) < epochs:
            lines.append([""] * (model_index + 1))  # 空列も補完

        # 各行（エポック）にloss値を記入
        for i in range(epochs):
            # 必要に応じて列数を拡張
            while len(lines[i]) <= model_index:
                lines[i].append("")

            lines[i][0] = str(i)  # 0列目にエポック番号
            lines[i][model_index] = loss_values[i]  # 該当モデルの列に上書き

        # 書き込み（上書きモード）
        with open(self.loss_path, 'w', newline='') as fw:
            writer = csv.writer(fw)
            writer.writerows(lines)

    def learn(self):
        ang_list = self.load_angles()

        # --- データのインデックスをシャッフル ---
        indices = list(range(self.data))
        random.shuffle(indices)

        now_dataset_num = 1
        # データセット作成（シャッフル後）
        for i in indices:
            images = self.load_images(i)
            target_ang = ang_list[i]
            for img, angle_shift in images:
                self.dl.make_dataset(img, target_ang + angle_shift)
            print(f"Model {self.model_num}, Dataset: {now_dataset_num}, Target Angle: {target_ang}")
            now_dataset_num += 1

        print(f"Dataset size: {len(self.dl.dataset)}")
        loss_log = []

        # 学習処理：エポックごとに trains() を呼ぶ
        for epoch in range(self.EPOCHS):
            start_time_epoch = time.time()
            loss = self.dl.trains(self.BATCH_SIZE)
            end_time_epoch = time.time()
            print(f"Model {self.model_num}, Epoch {epoch + 1}, Epoch time: {end_time_epoch - start_time_epoch:.4f} seconds, Loss: {loss}")
            loss_log.append([str(loss)])

        # lossの保存
        self.save_loss(loss_log)

        # モデルの保存
        self.dl.save(self.save_path)
        print(f"モデル保存完了: {self.save_path}")

        sys.exit()

if __name__ == '__main__':
    node = CourseFollowingLearningNode()
    node.learn()