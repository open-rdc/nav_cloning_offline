import cv2
import numpy as np
import roslib
import math

def simulate_disparity(img_path, angle_deg=5):
    # 画像読み込み
    img = cv2.imread(img_path)
    h, w = img.shape[:2]  # 画像の高さと幅を取得

    # カメラ内部パラメータの計算
    horizontal_fov = 2.09  # 水平方向の視野角（ラジアン：約120度）
    # 焦点距離fを画素単位で計算（画像の中心から端までの距離 / tan(FOV/2)）
    f = (w / 2) / math.tan(horizontal_fov / 2)
    # 画像中心座標
    cx, cy = w / 2, h / 2

    # カメラ内部パラメータ行列Kを作成
    K = np.array([[f, 0, cx],
                  [0, f, cy],
                  [0, 0, 1]])

    # y軸回転行列R（角度angle_deg度をラジアンに変換）
    angle_rad = math.radians(angle_deg)
    R = np.array([[ math.cos(angle_rad), 0, math.sin(angle_rad)],
                  [0,                 1, 0],
                  [-math.sin(angle_rad),0, math.cos(angle_rad)]])

    # 射影変換行列Hを計算（K * R * Kの逆行列）
    H = K @ R @ np.linalg.inv(K)
    # 正規化（射影変換行列の右下の値を1に）
    H /= H[2, 2]

    # 射影変換を元画像に適用し、新しい視点画像を生成
    warped = cv2.warpPerspective(img, H, (w, h))

    return warped


# 使用例
pro = "Checker_Plate"
# パッケージのデータディレクトリパスを取得
path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/'
# 中央画像のパス
img_center = path + f"img/{pro}/g_c_plate_3s.png"

# 中央画像から左方向（-5度）に視点を傾けた画像を作成
img_left = simulate_disparity(img_center, angle_deg=-5)
# 中央画像から右方向（+5度）に視点を傾けた画像を作成
img_right = simulate_disparity(img_center, angle_deg=5)

# 左右の画像をウィンドウに表示
cv2.imshow('Left View', img_left)
cv2.imshow('Right View', img_right)
cv2.waitKey(0)
cv2.destroyAllWindows()