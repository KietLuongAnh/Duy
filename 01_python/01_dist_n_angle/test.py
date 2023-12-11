import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math
from scipy.optimize import least_squares
from sklearn.neighbors import LocalOutlierFactor
from scipy import stats

# Đọc file và parse data
file_path = "D:\\git\\project\\01_python\\01_dist_n_angle\\00_data\\storeDataCalculated.csv"
df = pd.read_csv(file_path)
num_rows, num_columns = df.shape

total_index = 3201

the0 = df['Degree0']
x0 = df['XCoordinate0'].to_numpy()
y0 = df['YCoordinate0'].to_numpy()
z0 = df['ZCoordinate0'].to_numpy()

the1 = df['Degree1']
x1 = df['XCoordinate1'].to_numpy()
y1 = df['YCoordinate1'].to_numpy()
z1 = df['ZCoordinate1'].to_numpy()

def shift_arr(arr ,index):
    if index >= 0:
        result = np.roll(arr, len(x0) - abs(index))
    else:
        result = np.roll(arr, index)
    return result

def filter_custom(x0, y0, z0, x1, y1, z1, thre):
    points1 = np.column_stack((x0, y0, z0))
    points2 = np.column_stack((x1, y1, z1))

    # Tính toán khoảng cách giữa các điểm
    distances = np.linalg.norm(points1 - points2, axis=1)

    # Tính z-score của khoảng cách
    z_scores = np.abs(stats.zscore(distances))

    # Thiết lập ngưỡng dựa trên z-score (ví dụ: 2 đơn vị độ lệch chuẩn)
    threshold_z_score = thre

    # Lấy chỉ số của các điểm không bị loại bỏ
    indices_to_keep = np.where(z_scores < threshold_z_score)[0]

    # Tạo lại các mảng x0, y0, x1, y1 chỉ chứa các điểm không bị loại bỏ
    return x0[indices_to_keep], y0[indices_to_keep], z0[indices_to_keep], x1[indices_to_keep], y1[indices_to_keep], z1[indices_to_keep]

# Hàm residual để tối ưu hóa tx, ty
def residual(variables, samples):
    tx, ty = variables
    x0 = samples[:, 0]
    y0 = samples[:, 1]
    x1 = samples[:, 2]
    y1 = samples[:, 3]

    x_predicted = tx + x1 * np.cos(theta_radian) - y1 * np.sin(theta_radian)
    y_predicted = ty + x1 * np.sin(theta_radian) + y1 * np.cos(theta_radian)
    return np.concatenate([(x0 - x_predicted), (y0 - y_predicted)])

theta_range = np.arange(0, 360, 1)
best_theta = 0
best_tx = 0
best_ty = 0
lowest_residual = float('inf')

for theta_degree in theta_range:
    theta_radian = np.radians(theta_degree)
    cos_theta = np.cos(theta_radian)
    sin_theta = np.sin(theta_radian)

    index_io = math.floor(theta_degree * 3201 / 360)

    x0_test = shift_arr(x0, index_io)
    y0_test = shift_arr(y0, index_io)
    z0_test = shift_arr(z0, index_io)
    x1_test = shift_arr(x1, index_io)
    y1_test = shift_arr(y1, index_io)
    z1_test = shift_arr(z1, index_io)

    x0_test, y0_test, z0_test, x1_test, y1_test, z1_test = filter_custom(x0_test,
                                                                         y0_test,
                                                                         z0_test,
                                                                         x1_test,
                                                                         y1_test,
                                                                         z1_test,
                                                                         1)

    samples = np.column_stack((x0_test, y0_test, x1_test, y1_test))

    # Tìm tx, ty tương ứng với theta hiện tại
    variables_init = np.array([0, 0])  # Giả định ban đầu cho tx, ty
    result = least_squares(residual, variables_init, args=(samples,))

    tx, ty = result.x
    # Tính residual cho tx, ty hiện tại
    current_residual = result.cost

    # So sánh với residual tốt nhất hiện tại để cập nhật nếu cần
    if current_residual < lowest_residual:
        lowest_residual = current_residual
        best_theta = theta_degree
        best_tx = tx
        best_ty = ty

# Kết quả cuối cùng
print(f"Best theta: {best_theta} degrees")
print(f"Best tx   : {best_tx}")
print(f"Best ty   : {best_ty}")