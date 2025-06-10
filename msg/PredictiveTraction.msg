# Predictive traction control using Model Predictive Control

uint64 timestamp               # Time since system start (microseconds)

bool predictive_active         # True if predictive control is active
bool mpc_converged            # True if MPC optimization converged
float32 prediction_horizon_s   # Prediction horizon in seconds

float32 predicted_slip_front   # Predicted front wheel slip ratio
float32 predicted_slip_rear    # Predicted rear wheel slip ratio
float32 optimal_torque_front   # Optimal front wheel torque from MPC
float32 optimal_torque_rear    # Optimal rear wheel torque from MPC

float32 cost_function_value    # Current MPC cost function value
float32 slip_tracking_error    # RMS slip tracking error
float32 torque_smoothness      # Torque smoothness metric

uint8 mpc_status              # MPC solver status (0=converged, 1=max_iter, 2=failed)
uint16 solver_iterations      # Number of solver iterations used
float32 solve_time_ms         # Time to solve MPC problem in milliseconds

float32[10] predicted_trajectory_front # Predicted front wheel slip trajectory
float32[10] predicted_trajectory_rear  # Predicted rear wheel slip trajectory
