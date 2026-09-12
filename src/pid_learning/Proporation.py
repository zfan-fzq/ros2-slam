import numpy as np
import matplotlib.pyplot as plt


def proportional(theta, target_theta, Kp):
    error = target_theta - theta
    omega = Kp * error

    return omega


# =========================
# 基本参数
# =========================

theta_start = np.deg2rad(20.0)
target_theta = np.deg2rad(60.0)

dt = 0.1
steps = 100

Kp_list = [5, 10, 15, 20, 25]


# =========================
# 不同 Kp 分别仿真
# =========================

for Kp in Kp_list:

    theta = theta_start

    time_history = []
    theta_history = []

    for i in range(steps):

        # P 控制器
        omega = proportional(
            theta,
            target_theta,
            Kp
        )

        # 机器人运动
        theta = theta + omega * dt

        # 保存数据
        time_history.append(i * dt)
        theta_history.append(
            np.rad2deg(theta)
        )

    # 画这一组 Kp 的曲线
    plt.plot(
        time_history,
        theta_history,
        label=f"Kp = {Kp}"
    )


# =========================
# 目标角度
# =========================

plt.axhline(
    np.rad2deg(target_theta),
    linestyle="--",
    label="Target = 60 deg"
)

plt.xlabel("Time (s)")
plt.ylabel("Theta (deg)")

plt.title("P Control with Different Kp")

plt.legend()
plt.grid()

plt.show()