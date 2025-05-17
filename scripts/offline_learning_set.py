#!/usr/bin/env python3
import os
import sys
import csv
import time
import cv2
import random
import roslib
import numpy as np
from nav_cloning_pytorch import deep_learning
from skimage.transform import resize

class CourseFollowingLearningNode:
    def __init__(self):
        self.dl = deep_learning(n_action=1)
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.model_num = str(sys.argv[1])
        self.pro = "20250506_00:14:39"  # データセットの識別名

        self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/'
        self.save_path = self.path + f"model/{self.pro}/model{self.model_num}.pt"
        self.ang_path = self.path + f"ang/{self.pro}/"
        self.img_path = self.path + f"img/{self.pro}/center"
        self.loss_path =  self.path + f"loss/{self.pro}/model{self.model_num}.csv"
        
        self.data = 1695  # 使用するデータ数
        self.BATCH_SIZE = 16 # バッチサイズを指定
        self.EPOCHS = 300 # エポック数を指定
        
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        os.makedirs(self.path + f"/loss/{self.pro}/", exist_ok=True)

    def load_images(self, index, parallax):

        # 画像ファイルのパスを作成
        img_file = f"{self.img_path}{index}_{parallax}.jpg"
        img = cv2.imread(img_file)
        
        if img is None:
            print(f"Warning: Failed to load {img_file}")
            
        return img
        
    def load_angles(self):
        angles = []
        with open(self.ang_path + 'ang.csv', 'r') as f:
            for row in csv.reader(f):
                _, tar_ang = row
                angles.append(float(tar_ang))
        return angles

    def learn(self):
        ang_list = self.load_angles()
        data_entries = []

        # 全データ（画像と角度）をリストに格納
        ang_idx = 0
        for i in range(self.data):
            for parallax in ["-5", "0", "5"]:
                if parallax ==  "-5":
                    data_entries.append((i, parallax, ang_idx))
                ang_idx += 1

        # ランダムにシャッフル
        random.shuffle(data_entries)

        # シャッフル後にmake_datasetで追加
        now_dataset_num = 0
        for i, parallax, ang_idx in data_entries:
            img = self.load_images(i, parallax)
            if img is None:
                continue
            target_ang = ang_list[ang_idx]
            self.dl.make_dataset(img, target_ang)
            now_dataset_num += 1
            print(f"Model {self.model_num}, Dataset: {now_dataset_num}, Parallax: {parallax}, Target Angle: {target_ang}")
            
        loss_log = []

        for epoch in range(self.EPOCHS):
            start_time_epoch = time.time()
            loss = self.dl.trains(self.BATCH_SIZE)
            end_time_epoch = time.time()
            print(f"Model {self.model_num}, Epoch {epoch + 1}, Time: {end_time_epoch - start_time_epoch:.4f}s, Loss: {loss}")
            loss_log.append([str(loss)])

        # lossを保存
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