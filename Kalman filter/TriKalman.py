import numpy as np

class KalmanFilter3D:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise):
        # Initial state estimate [x, y, z, vx, vy, vz]
        self.state = initial_state

        # Initial covariance matrix
        self.covariance = initial_covariance

        # Process noise covariance matrix
        self.Q = process_noise

        # Measurement noise covariance matrix
        self.R = measurement_noise

        # State transition matrix (constant velocity model)
        self.A = np.array([[1, 0, 0, 1, 0, 0],
                          [0, 1, 0, 0, 1, 0],
                          [0, 0, 1, 0, 0, 1],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 1]])

        # Measurement matrix (we can measure all state variables)
        self.H = np.eye(6)

    def predict(self):
        # Predict the next state
        self.state = np.dot(self.A, self.state)
        self.covariance = np.dot(np.dot(self.A, self.covariance), self.A.T) + self.Q

    def update(self, measurement):
        # Update the state estimate based on the measurement
        residual = measurement - np.dot(self.H, self.state)
        S = np.dot(np.dot(self.H, self.covariance), self.H.T) + self.R
        K = np.dot(np.dot(self.covariance, self.H.T), np.linalg.inv(S))

        # Update state and covariance
        self.state = self.state + np.dot(K, residual)
        self.covariance = self.covariance - np.dot(np.dot(K, self.H), self.covariance)

if __name__ == "__main__":
    # Initial state estimate [x, y, z, vx, vy, vz]
    initial_state = np.array([0, 0, 0, 1, 1, 1])

    # Initial covariance matrix
    initial_covariance = np.eye(6)

    # Process noise covariance matrix
    process_noise = np.eye(6) * 0.01

    # Measurement noise covariance matrix
    measurement_noise = np.eye(6) * 0.1

    # Create a Kalman filter instance
    kf = KalmanFilter3D(initial_state, initial_covariance, process_noise, measurement_noise)

    # Simulate measurements
    measurements = np.array([1, 1, 1, 2, 2, 2])

    for measurement in measurements:
        # Predict and update the Kalman filter
        kf.predict()
        kf.update(measurement)

        print("Estimated state:", kf.state)