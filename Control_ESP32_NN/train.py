## 測定したTon, v0, v1からPosを推定するモデルを学習させる
## - 入力 : Ton, v0, v1, Posを含む測定データのExcelファイル
## - 出力 : TensorFlowLite for Microcontrollersで読み込むモデル(model.h)

# pip install pandas scikit-learn tensorflow

import pandas as pd
import numpy as np
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Conv1D, Flatten
from tensorflow.keras.optimizers import Adam

################################3
### 使うデータのExcelファイル
file_path = '学習用SSBH0830-01.xlsx'
### 学習回数
Nepoch = 100
################################3


# データの読み込み
data = pd.read_excel(file_path)
print("data read done")

# 特徴量とターゲット変数を分ける
X = data[['Ton[ms]', 'v0', 'v1']].values
y = data['Pos[mm]'].values

# データの標準化
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

# データの形状を変更してCNNに適した形にする
X_scaled = X_scaled.reshape(X_scaled.shape[0], X_scaled.shape[1], 1)

# データをトレーニングセットとテストセットに分割する
X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.2, random_state=42)

# モデルの構築
model = Sequential()
model.add(Conv1D(64, kernel_size=2, activation='relu', input_shape=(X_train.shape[1], 1)))
model.add(Flatten())
model.add(Dense(64, activation='relu'))
model.add(Dense(1))

# モデルのコンパイル
model.compile(optimizer=Adam(learning_rate=0.001), loss='mean_squared_error')

# モデルのトレーニング
### epoch数は増やす
model.fit(X_train, y_train, epochs=Nepoch, validation_split=0.2)

# モデルの評価
y_pred = model.predict(X_test)
mse = np.mean((y_test - y_pred.flatten())**2)
print(f"Mean Squared Error: {mse}")

# モデルを保存
#model.save('my_model.h5')

# TensorFlow Liteモデルに変換
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

# TensorFlow Liteモデルを保存
#with open('model.tflite', 'wb') as f:
#    f.write(tflite_model)

#### 量子化（やらない）
# 量子化モデルの変換
#converter = tf.lite.TFLiteConverter.from_keras_model(model)
#converter.optimizations = [tf.lite.Optimize.DEFAULT]
#tflite_quant_model = converter.convert()
## 量子化モデルの保存
#with open('model_quant.tflite', 'wb') as f:
#    f.write(tflite_quant_model)

# C++ヘッダーファイル形式に変換
hex_array = ','.join([f'0x{b:02x}' for b in tflite_model])
header_file_content = f"""
#ifndef MODEL_H
#define MODEL_H

const unsigned char g_model[] = {{{hex_array}}};
const int g_model_len = {len(tflite_model)};

#endif  // MODEL_H
"""

# ヘッダーファイルとして保存
with open('model.h', 'w') as f:
    f.write(header_file_content)
