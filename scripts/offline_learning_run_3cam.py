#!/usr/bin/env python3
import os
import sys
import csv
import time
import cv2
import roslib
import random
import numpy as np
from nav_cloning_pytorch import deep_learning

class CourseFollowingLearningNode:
    def __init__(self):
        self.dl = deep_learning(n_action=1)
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.model_num = str(sys.argv[1])
        self.pro = "20250419_23:59:33"  # データセットの識別名
        self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/'
        self.save_path = self.path + f"model/{self.pro}/model{self.model_num}.pt"
        self.ang_path = self.path + f"ang/{self.pro}/"
        self.img_path = self.path + f"img/{self.pro}/"
        self.loss_path =  self.path + f"loss/{self.pro}/model{self.model_num}.csv"

        self.data = 1254  # 使用するデータ数
        self.BATCH_SIZE = 16 # バッチサイズを指定
        self.EPOCHS = 100 # エポック数を指定
        
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        os.makedirs(self.path + f"/loss/{self.pro}/", exist_ok=True)

    def load_images(self, index):
        shifts = {
            "left": -0.2,
            "center": 0,
            "right": 0.2
        }

        img_types = ["left", "center", "right"]
        images = []

        for img_type in img_types:
            angle_shift = shifts[img_type]
            img_file = f"{self.img_path}{img_type}{index}.jpg"
            img = cv2.imread(img_file)
            
            if img is None:
                print(f"Warning: Failed to load {img_file}")
                continue

            images.append((img, angle_shift))

        return images
        
    def load_angles(self):
        angles = []
        with open(self.ang_path + 'ang.csv', 'r') as f:
            for row in csv.reader(f):
                _, tar_ang = row
                angles.append(float(tar_ang))
        return angles

    def learn(self):
        ang_list = self.load_angles()

        # --- データのインデックスをシャッフル ---
        indices = list(range(self.data))
        random.shuffle(indices)

        j = 1
        # データセット作成（シャッフル後）
        for i in indices:
            images = self.load_images(i)
            target_ang = ang_list[i]
            for img, angle_shift in images:
                self.dl.make_dataset(img, target_ang + angle_shift)
            print(f"Dataset: {j}")
            j += 1

        print(f"Dataset size: {len(self.dl.dataset)}")
        loss_log = []

        # 学習処理：エポックごとに trains() を呼ぶ
        for epoch in range(self.EPOCHS):
            start_time_epoch = time.time()
            loss = self.dl.trains(self.BATCH_SIZE)
            end_time_epoch = time.time()
            print(f"Epoch {epoch + 1}, Epoch time: {end_time_epoch - start_time_epoch:.4f} seconds, Loss: {loss}")
            loss_log.append([str(loss)])

        # lossの保存
        with open(self.loss_path, 'a') as fw:
            writer = csv.writer(fw, lineterminator='\n')
            writer.writerows(loss_log)

        # モデルの保存
        self.dl.save(self.save_path)
        print(f"モデル保存完了: {self.save_path}")

        sys.exit()

if __name__ == '__main__':
    node = CourseFollowingLearningNode()
    node.learn()