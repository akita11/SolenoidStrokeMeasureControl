## 測定したTon, v0, v1からPosを推定するモデルを学習させる
## - 入力 : Ton, v0, v1, Posを含む測定データのExcelファイル
## - 出力 : TensorFlowLite for Microcontrollersで読み込むモデル(model.h)

# pip install pandas scikit-learn tensorflow

import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Conv1D, Flatten
import numpy as np

# (Ton, v0, v1) -> (Pos, Temp)
# Scaler Params:
# -> x_scaled = (x - x_mean) / x_scaler
# -> y = y_scaled * y_scaler + y_mean

################################3
### 使うデータのExcelファイル
file_path = '学習用SSBH0830-01.xlsx'
### 学習回数
Nepoch = 100
################################3

# データの読み込み
file_path = '学習用CBS0730140.xlsx'
data = pd.read_excel(file_path)

# 特徴量とターゲット変数を分ける
x = data[['Ton-100', 'v0-100', 'v1-100']].values # 100Hz
#x = data[['Ton-200', 'v0-200', 'v1-200']].values # 200Hz
y = data[['Pos[mm]', 'Temp[degC]']].values

# データの標準化
scaler_x = StandardScaler()
scaler_y = StandardScaler()

x_scaled = scaler_x.fit_transform(x)
y_scaled = scaler_y.fit_transform(y)

# データをトレーニングセットとテストセットに分割する
x_train, x_test, y_train, y_test = train_test_split(x_scaled, y_scaled, test_size=0.2, random_state=42)

# データの形状を変更してCNNに適した形にする
x_train = x_train.reshape(x_train.shape[0], x_train.shape[1], 1)
x_test = x_test.reshape(x_test.shape[0], x_test.shape[1], 1)

# モデルの構築
model = Sequential()
model.add(Conv1D(32, kernel_size=2, activation='relu', input_shape=(x_train.shape[1], 1)))
model.add(Flatten())
model.add(Dense(32, activation='relu'))
model.add(Dense(2))  # 出力層は2つの出力

# モデルのコンパイル
model.compile(optimizer='adam', loss='mean_squared_error')

# モデルのトレーニング
history = model.fit(x_train, y_train, epochs=20000, validation_split=0.2)

# モデルの評価
y_pred = model.predict(x_test)
mse = np.mean((y_test - y_pred)**2, axis=0)
print("Mean Squared Error for Pos and Temp:", mse)

# スケーラーのパラメータを保存
scaler_x_params = {"mean": scaler_x.mean_, "scale": scaler_x.scale_}
scaler_y_params = {"mean": scaler_y.mean_, "scale": scaler_y.scale_}
#np.save("scaler_x_params.npy", scaler_x_params)
#np.save("scaler_y_params.npy", scaler_y_params)

print("Scaler params", scaler_x_params, scaler_y_params)

# モデルを保存
#model.save('cnn_model.h5')

# TensorFlow Liteモデルに変換
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

# TensorFlow Liteモデルを保存
#with open('model.tflite', 'wb') as f:
#    f.write(tflite_model)

# C++ヘッダーファイル形式に変換
hex_array = ','.join([f'0x{b:02x}' for b in tflite_model])
header_file_content = f"""
#ifndef MODEL_H
#define MODEL_H

// MSE(Pos, Temp): {mse}


const unsigned char g_model[] = {{{hex_array}}};
const int g_model_len = {len(tflite_model)};

const scaler_x_mean={{ {scaler_x.mean_[0]}, {scaler_x.mean_[1]}, {scaler_x.mean_[2]} }};
const scaler_x_scale={{ {scaler_x.scale_[0]}, {scaler_x.scale_[1]}, {scaler_x.scale_[2]} }};
const scaler_y_mean={{ {scaler_y.mean_[0]}, {scaler_y.mean_[1]} }};
const scaler_y_scale={{ {scaler_y.scale_[0]}, {scaler_y.scale_[1]} }};
#endif  // MODEL_H
"""
# ヘッダーファイルとして保存
with open('model.h', 'w') as f:
    f.write(header_file_content)

