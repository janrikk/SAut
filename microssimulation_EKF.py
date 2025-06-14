#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


DT = 0.1  # sampling time
Q = np.diag([0.005, 0.005, np.deg2rad(1.0)]) ** 2 
R = np.diag([0.5, np.deg2rad(5.0)]) ** 2 

LANDMARKS = np.array([
    [0.0, 0.0],      # aruco_0
    [0.9, 2.7],      # aruco_1
    [-3.3, 2.4],     # aruco_2
    [-3.6, -1.5],    # aruco_3
    [-1.5, -0.6]     # aruco_4
])

# caminho
WAYPOINTS = np.array([
    [0.0, 0.0], [0.9, 2.7], [-3.3, 2.4], [-3.6, -1.5],
    [-3.3, 2.4], [0.9, 2.7], [-1.5, -0.6]
])

def pi_to_pi(angle):
    """Normalize angle to [-π, π]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def motion_model(x, u):
    """Unicycle motion model"""
    x_new = x.copy()
    x_new[0] += u[0] * np.cos(x[2]) * DT  # posição x
    x_new[1] += u[0] * np.sin(x[2]) * DT  # posição y
    x_new[2] += u[1] * DT                 
    x_new[2] = pi_to_pi(x_new[2])        

def observation_model(x, landmark):
    """Range and bearing measurement model"""
    dx = landmark[0] - x[0]
    dy = landmark[1] - x[1]
    r = np.hypot(dx, dy)                          # range
    phi = np.arctan2(dy, dx) - x[2]               # bearing
    return np.array([r, pi_to_pi(phi)])

def compute_jacobians(x, u):
    """Compute motion model Jacobians"""
    theta = x[2]
    G = np.eye(3)
    G[0, 2] = -u[0] * np.sin(theta) * DT
    G[1, 2] = u[0] * np.cos(theta) * DT
    return G

def compute_control(xEst, wp, v=0.7):
    """Compute control inputs to reach waypoint"""
    dx = wp[0] - xEst[0]
    dy = wp[1] - xEst[1]
    desired_theta = np.arctan2(dy, dx)
    omega = pi_to_pi(desired_theta - xEst[2]) / DT
    return np.array([v, omega])

def is_landmark_visible(x, lm, max_range=5.0, fov_deg=180):
    """Check if landmark is within detection range and FOV"""
    dx = lm[0] - x[0]
    dy = lm[1] - x[1]
    r = np.hypot(dx, dy)
    if r > max_range:
        return False
    bearing = pi_to_pi(np.arctan2(dy, dx) - x[2])
    return abs(bearing) < np.deg2rad(fov_deg / 2)

def initialize_landmark(xEst, PEst, z, landmark_id):
    """Initialize new landmark in state vector"""
    r, phi = z
    lmx = xEst[0] + r * np.cos(xEst[2] + phi)
    lmy = xEst[1] + r * np.sin(xEst[2] + phi)

    new_state = np.append(xEst, [lmx, lmy])
    n = len(new_state)
    new_cov = np.zeros((n, n))
    new_cov[:-2, :-2] = PEst
    new_cov[-2:, -2:] = np.eye(2) * 10.0
    return new_state, new_cov

def observation_jacobian(xEst, landmark_idx):
    """Compute observation model Jacobian"""
    lm_start = 3 + 2 * landmark_idx
    dx = xEst[lm_start] - xEst[0]
    dy = xEst[lm_start + 1] - xEst[1]
    q = dx**2 + dy**2
    sqrt_q = np.sqrt(q)

    H = np.zeros((2, len(xEst)))
    H[0, 0] = -dx / sqrt_q
    H[0, 1] = -dy / sqrt_q
    H[1, 0] = dy / q
    H[1, 1] = -dx / q
    H[1, 2] = -1.0
    H[0, lm_start] = dx / sqrt_q
    H[0, lm_start + 1] = dy / sqrt_q
    H[1, lm_start] = -dy / q
    H[1, lm_start + 1] = dx / q
    return H

def covariance_ellipse(cov, nstd=2):
    """Compute parameters for covariance ellipse"""
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
    width, height = 2 * nstd * np.sqrt(vals)
    return width, height, theta

def closest_point_distance(set_A, set_B):
    """
    Compute closest point distances between two sets of points.
    Returns: (distances from A to B, distances from B to A)
    """
    dists_A_to_B = [np.min(np.linalg.norm(a - set_B, axis=1)) for a in set_A]
    dists_B_to_A = [np.min(np.linalg.norm(b - set_A, axis=1)) for b in set_B]
    return np.array(dists_A_to_B), np.array(dists_B_to_A)

def calculate_localization_cpd(true_traj, est_traj):
    """Compute CPD metrics for trajectory validation"""
    cpd_true_to_est, cpd_est_to_true = closest_point_distance(true_traj[:, :2], est_traj[:, :2])
    
    return {
        'mean_cpd': np.mean(cpd_true_to_est),
        'max_cpd': np.max(cpd_true_to_est),
        'symmetric_cpd': 0.5 * (np.mean(cpd_true_to_est) + np.mean(cpd_est_to_true))
    }

def calculate_mapping_cpd(true_landmarks, est_landmarks):
    """Compute CPD metrics for landmark validation"""
    cpd_true_to_est, cpd_est_to_true = closest_point_distance(true_landmarks, est_landmarks)
    
    return {
        'landmark_mean_cpd': np.mean(cpd_true_to_est),
        'landmark_max_cpd': np.max(cpd_true_to_est),
        'landmark_symmetric_cpd': 0.5 * (np.mean(cpd_true_to_est) + np.mean(cpd_est_to_true)),
        'missed_landmarks': len(true_landmarks) - len(est_landmarks)  # Count undetected landmarks
    }

def main():
    xTrue = np.array([0.0, 0.0, 0.0])  
    xEst = np.array([0.0, 0.0, 0.0])   
    PEst = np.eye(3)                    


    hxTrue, hxEst = [], []
    known_landmarks = {}
    current_wp = 0

    #EKF-SLAM
    for step in range(1000):
        wp = WAYPOINTS[current_wp]
        if np.hypot(wp[0] - xEst[0], wp[1] - xEst[1]) < 0.3:
            current_wp = (current_wp + 1) % len(WAYPOINTS)

        # Control and motion prediction
        u = compute_control(xEst, wp)
        u_noisy = u + np.random.multivariate_normal([0, 0], Q[:2, :2])
        xTrue = motion_model(xTrue, u)
        xPred = motion_model(xEst[:3], u_noisy)

        # Covariance prediction
        G = compute_jacobians(xEst[:3], u_noisy)
        G_full = np.eye(len(xEst))
        G_full[:3, :3] = G
        PEst = G_full @ PEst @ G_full.T
        PEst[:3, :3] += Q
        xEst[:3] = xPred

        # Landmark observations
        for lm_id, lm in enumerate(LANDMARKS):
            if not is_landmark_visible(xTrue, lm):
                continue

            z_true = observation_model(xTrue, lm)
            z = z_true + np.random.multivariate_normal([0, 0], R)

            if lm_id not in known_landmarks:
                xEst, PEst = initialize_landmark(xEst, PEst, z, lm_id)
                known_landmarks[lm_id] = len(known_landmarks)
                continue

            # EKF update
            idx = known_landmarks[lm_id]
            H = observation_jacobian(xEst, idx)
            lm_pred = xEst[3 + 2 * idx : 3 + 2 * idx + 2]
            z_pred = observation_model(xEst[:3], lm_pred)

            y = z - z_pred
            y[1] = pi_to_pi(y[1])
            S = H @ PEst @ H.T + R
            K = PEst @ H.T @ np.linalg.inv(S)
            xEst += K @ y
            PEst = (np.eye(len(xEst)) - K @ H) @ PEst + 1e-6 * np.eye(len(xEst))

        # Store history
        hxTrue.append(xTrue.copy())
        hxEst.append(xEst.copy())

    # --- Visualization ---
    hxTrue = np.array(hxTrue)[:, :2]  # Only need x,y for CPD
    hxEst = np.array([x[:2] for x in hxEst])
    est_landmarks = np.array([xEst[3 + 2*i : 3 + 2*i + 2] for i in range(len(known_landmarks))])

    # Calculate CPD metrics
    loc_metrics = calculate_localization_cpd(hxTrue, hxEst)
    map_metrics = calculate_mapping_cpd(LANDMARKS, est_landmarks)
    
    # Print results
    print("\n=== Localization CPD Metrics ===")
    print(f"Mean CPD: {loc_metrics['mean_cpd']:.3f} m")
    print(f"Max CPD: {loc_metrics['max_cpd']:.3f} m")
    print(f"Symmetric CPD: {loc_metrics['symmetric_cpd']:.3f} m")
    
    print("\n=== Mapping CPD Metrics ===")
    print(f"Landmark Mean CPD: {map_metrics['landmark_mean_cpd']:.3f} m")
    print(f"Landmark Max CPD: {map_metrics['landmark_max_cpd']:.3f} m") 
    print(f"Undetected Landmarks: {map_metrics['missed_landmarks']}/{len(LANDMARKS)}")

    # Plot 1: Trajectory and Landmarks
    plt.figure(1, figsize=(10, 8))
    plt.plot(hxTrue[:, 0], hxTrue[:, 1], 'b-', label='True Trajectory', alpha=0.6)
    plt.plot(hxEst[:, 0], hxEst[:, 1], 'r--', label='Estimated Trajectory', alpha=0.7)
    plt.scatter(LANDMARKS[:, 0], LANDMARKS[:, 1], c='green', marker='X', s=100, label='True Landmarks')
    plt.scatter(est_landmarks[:, 0], est_landmarks[:, 1], c='purple', edgecolors='k', marker='o', label='Estimated Landmarks')

    # Add 2σ covariance ellipses
    for i in range(len(known_landmarks)):
        idx = 3 + 2*i
        cov = PEst[idx:idx+2, idx:idx+2]
        width, height, angle = covariance_ellipse(cov, nstd=2)
        ell = Ellipse(xy=(xEst[idx], xEst[idx+1]),
                     width=width, height=height,
                     angle=angle, color='purple', alpha=0.2)
        plt.gca().add_patch(ell)

    for i, (x, y) in enumerate(LANDMARKS):
        plt.text(x + 0.1, y + 0.1, f'LM{i}', fontsize=9)

    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.title("EKF-SLAM: Trajectory and Landmark Uncertainty (2σ)")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.tight_layout()

    # Plot 2: Robot Pose Correlations
    plt.figure(2, figsize=(6, 5))
    diag = np.sqrt(np.diag(PEst))
    corr_matrix = PEst / np.outer(diag, diag)
    
    plt.imshow(corr_matrix[:3, :3], cmap='coolwarm', vmin=-1, vmax=1)
    plt.colorbar(label='Correlation Coefficient')
    plt.xticks([0, 1, 2], ['x', 'y', 'θ'])
    plt.yticks([0, 1, 2], ['x', 'y', 'θ'])
    plt.title("Robot Pose State Correlations")
    plt.tight_layout()

    # Plot 3: Landmark Correlations (if landmarks exist)
    if len(known_landmarks) > 0:
        plt.figure(3, figsize=(8, 6))
        landmark_corr = corr_matrix[3:, 3:]
        
        plt.imshow(landmark_corr, cmap='coolwarm', vmin=-1, vmax=1)
        plt.colorbar(label='Correlation Coefficient')
        
        n_landmarks = len(known_landmarks)
        xticks = np.arange(0, 2*n_landmarks, 2)
        plt.xticks(xticks, [f'LM{i}' for i in range(n_landmarks)])
        plt.yticks(xticks, [f'LM{i}' for i in range(n_landmarks)])
        
        plt.title("Landmark Position Correlations")
        plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()