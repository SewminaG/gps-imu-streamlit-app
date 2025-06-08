import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def run_ukf(df, include_attitude=True, plot=True):
        # --- Convert to meters and smooth GPS ---
    df['x'] = df['longitude_dif(cm)'] / 100.0
    df['y'] = df['latitude_dif(cm)'] / 100.0

    # Remove outliers (Median Absolute Deviation)
    def remove_outliers(series, threshold=3.5):
        median = np.median(series)
        diff = np.abs(series - median)
        mad = np.median(diff)
        z = 0.6745 * diff / mad if mad else 0
        return np.where(z < threshold, series, np.nan)

    df['x'] = remove_outliers(df['x'])
    df['y'] = remove_outliers(df['y'])
    df['x'] = pd.Series(df['x']).interpolate()
    df['y'] = pd.Series(df['y']).interpolate()

    # --- Smooth accelerometer data ---
    df['xacc'] = df['xacc'].rolling(5, center=True).mean().bfill().ffill()
    df['yacc'] = df['yacc'].rolling(5, center=True).mean().bfill().ffill()

    # Extract values
    timestamps = df['Timestamp_seconds'].values
    yaw = np.deg2rad(df['yaw'].values)
    acc_x = df['xacc'].values
    acc_y = df['yacc'].values
    meas_x = df['x'].values
    meas_y = df['y'].values

    # UKF parameters
    n = 5
    alpha, kappa, beta = 0.1, 0, 2.0
    lambda_ = alpha**2 * (n + kappa) - n
    gamma = np.sqrt(n + lambda_)

    # Robust velocity init
    vx0 = np.mean((meas_x[1:5] - meas_x[0:4]) / (timestamps[1:5] - timestamps[0:4]))
    vy0 = np.mean((meas_y[1:5] - meas_y[0:4]) / (timestamps[1:5] - timestamps[0:4]))
    x = np.array([meas_x[0], meas_y[0], vx0, vy0, yaw[0]])

    # Covariances
    P = np.eye(n)
    Q = np.diag([0.01, 0.01, 0.5, 0.5, 0.001])  # more noise on velocity
    R = np.diag([1.5, 1.5, np.deg2rad(5.0)]) if include_attitude else np.diag([1.5, 1.5])  # less trust on yaw

    Wm = np.full(2 * n + 1, 1 / (2 * (n + lambda_)))
    Wc = Wm.copy()
    Wm[0] = lambda_ / (n + lambda_)
    Wc[0] = lambda_ / (n + lambda_) + (1 - alpha**2 + beta)

    def normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def fx(x, dt, ax, ay):
        px, py, vx, vy, theta = x
        ax_g = ax * np.cos(theta) - ay * np.sin(theta)
        ay_g = ax * np.sin(theta) + ay * np.cos(theta)
        vx += ax_g * dt
        vy += ay_g * dt
        px += vx * dt
        py += vy * dt
        return np.array([px, py, vx, vy, theta])

    def hx(x):
        return np.array([x[0], x[1], x[4]]) if include_attitude else np.array([x[0], x[1]])

    # UKF Loop
    filtered = []
    for i in range(0, len(df)):
        dt = timestamps[i] - timestamps[i-1]
        ax, ay = acc_x[i], acc_y[i]
        z = np.array([meas_x[i], meas_y[i], yaw[i]]) if include_attitude else np.array([meas_x[i], meas_y[i]])

        sqrt_P = np.linalg.cholesky(P)
        sigma_pts = [x]
        for j in range(n):
            sigma_pts.append(x + gamma * sqrt_P[:, j])
            sigma_pts.append(x - gamma * sqrt_P[:, j])
        sigma_pts = np.array(sigma_pts)

        sigma_pred = np.array([fx(sp, dt, ax, ay) for sp in sigma_pts])
        x_pred = np.sum(Wm[:, None] * sigma_pred, axis=0)
        P_pred = Q.copy()
        for j in range(2 * n + 1):
            dx = sigma_pred[j] - x_pred
            dx[4] = normalize_angle(dx[4])
            P_pred += Wc[j] * np.outer(dx, dx)

        Z_sigma = np.array([hx(sp) for sp in sigma_pred])
        z_pred = np.sum(Wm[:, None] * Z_sigma, axis=0)
        S = R.copy()
        for j in range(2 * n + 1):
            dz = Z_sigma[j] - z_pred
            if include_attitude: dz[2] = normalize_angle(dz[2])
            S += Wc[j] * np.outer(dz, dz)

        Cxz = np.zeros((n, len(z)))
        for j in range(2 * n + 1):
            dx = sigma_pred[j] - x_pred
            dx[4] = normalize_angle(dx[4])
            dz = Z_sigma[j] - z_pred
            if include_attitude: dz[2] = normalize_angle(dz[2])
            Cxz += Wc[j] * np.outer(dx, dz)

        K = Cxz @ np.linalg.inv(S)
        innovation = z - z_pred
        if include_attitude: innovation[2] = normalize_angle(innovation[2])
        x = x_pred + K @ innovation
        P = P_pred - K @ S @ K.T
        filtered.append(x.copy())

    # Output DataFrame
    results_df = pd.DataFrame(filtered, columns=['x', 'y', 'vx', 'vy', 'theta'])
    results_df['time'] = timestamps[0:len(filtered)+1]
    results_df['Latitude'] = results_df['y']
    results_df['Longitude'] = results_df['x']
    
    if plot:
        fig, axs = plt.subplots(1, 2, figsize=(12, 5))

        axs[0].plot(timestamps, df['latitude_dif(cm)'], 'r-', label='Raw Latitude')
        axs[0].plot(results_df['time'], results_df['Latitude'] * 100, 'b-', label='UKF Latitude')
        axs[0].set_title(f'Latitude ({ "Scenario I" if include_attitude else "Scenario II" })')
        axs[0].set_ylabel('Latitude (cm)')
        axs[0].legend()
        axs[0].grid(True)

        axs[1].plot(timestamps, df['longitude_dif(cm)'], 'r-', label='Raw Longitude')
        axs[1].plot(results_df['time'], results_df['Longitude'] * 100, 'b-', label='UKF Longitude')
        axs[1].set_title(f'Longitude ({ "Scenario I" if include_attitude else "Scenario II" })')
        axs[1].set_ylabel('Longitude (cm)')
        axs[1].legend()
        axs[1].grid(True)

        plt.tight_layout()
        return results_df, fig
    else:
        return results_df, None
