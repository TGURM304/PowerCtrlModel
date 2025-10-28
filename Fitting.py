import numpy as np

# 填入数据 (电流A, 转速rpm, 功率W)
data = [
    # 空载
    (-0.138, -3200, 4.17),
    (-0.151, -1600, 2.04),
    (-0.143, -800, 1.20),
    (-0.133, -400, 0.864),
    (-0.134, -200, 0.792),
    (-0.164, -100, 0.744),
    (-0.390, -50, 0.793),
    (-0.640, -20, 0.817),
    (0.0, 0.0, 0.721),
    (0.640, 20, 0.817),
    (0.410, 50, 0.793),
    (0.156, 100, 0.745),
    (0.136, 200, 0.793),
    (0.139, 400, 0.912),
    (0.150, 800, 1.22),
    (0.158, 1600, 2.06),
    (0.153, 3200, 4.24),
    # 带负载
    (3.05, 329, 4.76),
    (7.50, 58, 11.76),
    (12.50, 41.70, 32.81),
    (14.1, 105.5, 42.70),
    (6.75, 1020, 25.5),
    (16.4, 35, 52.87),
    (7.98, 418, 19.58),
    (4.56, 502, 9.53),
    (9.32, 106, 18.52),
    (7.07, 194, 12.88),
    (4.01, 49, 3.92),
    (3.20, 116, 3.37)
]

# 提取变量
I = np.array([x[0] for x in data])
W = np.array([x[1] for x in data])
y = np.array([x[2] for x in data])

# 构建设计矩阵 X: [1, I, W, I*W, I², W²]
X = np.column_stack([np.ones_like(I), I, W, I*W, I**2, W**2])

# 最小二乘求解
coeffs, residuals, _, _ = np.linalg.lstsq(X, y, rcond=None)

# 输出系数
print("拟合系数:")
print(f"K0 = {coeffs[0]:.6f}")
print(f"K1 = {coeffs[1]:.6f}")
print(f"K2 = {coeffs[2]:.6f}")
print(f"K3 = {coeffs[3]:.6f}")
print(f"K4 = {coeffs[4]:.6f}")
print(f"K5 = {coeffs[5]:.6f}\n")

# 计算预测值和残差
y_pred = X @ coeffs  # 矩阵乘法计算预测值
residuals = y - y_pred  # 残差 = 实际值 - 预测值

# 模型拟合评价指标
ss_total = np.sum((y - np.mean(y))** 2)  # 总平方和
ss_residual = np.sum(residuals **2)  # 残差平方和
r_squared = 1 - (ss_residual / ss_total)  # R²值
mse = np.mean(residuals** 2)  # 均方误差

print("模型拟合评价:")
print(f"R²值: {r_squared:.6f} (越接近1表示拟合越好)")
print(f"均方误差(MSE): {mse:.6f} (越小表示拟合越好)")
print(f"残差平方和: {ss_residual:.6f}\n")

# 检测异常值（残差绝对值超过3倍标准差的视为异常）
residual_std = np.std(residuals)  # 残差标准差
threshold = 3 * residual_std  # 异常值判断阈值
outlier_indices = np.where(np.abs(residuals) > threshold)[0]

print("可能的异常数据（残差过大）:")
if len(outlier_indices) == 0:
    print("未检测到明显异常数据")
else:
    print(f"检测到 {len(outlier_indices)} 个异常数据（残差绝对值 > {threshold:.6f}）:")
    for idx in outlier_indices:
        print(f"数据点 {idx+1}: 电流={I[idx]:.3f}A, 转速={W[idx]:.3f}rpm, 实际功率={y[idx]:.3f}W, 预测功率={y_pred[idx]:.3f}W, 残差={residuals[idx]:.3f}W")