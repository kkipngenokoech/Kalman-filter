class Kalman:
    def __init(self, A, B, H, Q, R, P, x):
        '''
        A - State transition matrix - predict next state => it describes how the state of the system changes from one time step to the next
        B - Control matrix = this is an optional component that accounts for the control input to the system (it accounts for external factors that influence the system's state)
        H - Observation matrix/ measurement matrix: this matrix maps the true state space to the observed space (it describes how the state of the system is mapped to the measurement space) we are mapping what we are observing and the variables we want to measure
        Q - Process noise covariance : accounts for the uncertainty in the model itself and the randomness in the system
        R - Measurement noise covariance: accounts for the uncertainty in the measurements/ noise in the observations
        P - Estimation error covariance = it describes the uncertainty in the state estimate (how much the filter trusts its current estimate)
        x - Initial state estimate : it is the initial guess of the state of the system
        '''
        self.A = A
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        self.P = P
        self.x = x
        
    '''
    kalman filter has two steps:
    1. Predict : predict the next state of the system
    2. Update : update the state estimate based on the measurement
    
    Predict:
    x = A*x + B*u
    P = A*P*A^T + Q
    
    Update:
    K = P*H^T*(H*P*H^T + R)^-1
    x = x + K*(z - H*x)
    P = (I - K*H)*P
    
    where:
    x = state estimate
    P = state covariance
    K = Kalman gain
    z = measurement
    u = control input
    I = identity matrix
    '''
    
    def predict(self, u):
        '''
        what we are doing here is predicting the next state of the system using the state transition matrix A and the control matrix B. We are ,ultiplying the state transition matrix A by the current state of the system x and the control matrix B by the control input u. We are also updating the state covariance matrix P by multiplying the state transition matrix A by the current state covariance matrix P and the transpose of the state transition matrix A. We are also adding the process noise covariance Q to the state covariance matrix P
        '''
        # Predict the next state of the system
        self.x = self.A * self.x + self.B * u
        self.P = self.A * self.P * self.A.T + self.Q
        return self.x