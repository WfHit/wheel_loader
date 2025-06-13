/**
 * @file predictive_traction_params.c
 *
 * Parameters for PredictiveTractionControl module
 */

/**
 * Slip Warning Threshold
 *
 * Slip ratio threshold for issuing warnings
 *
 * @min 0.01
 * @max 0.50
 * @decimal 2
 * @increment 0.01
 * @reboot_required false
 * @group Predictive Traction Control
 */
PARAM_DEFINE_FLOAT(PTC_SLIP_WARN, 0.15f);

/**
 * Slip Critical Threshold
 *
 * Slip ratio threshold for critical situations
 *
 * @min 0.05
 * @max 0.80
 * @decimal 2
 * @increment 0.01
 * @reboot_required false
 * @group Predictive Traction Control
 */
PARAM_DEFINE_FLOAT(PTC_SLIP_CRIT, 0.30f);

/**
 * Prediction Horizon
 *
 * Time horizon for predictive control
 *
 * @unit ms
 * @min 100
 * @max 5000
 * @decimal 0
 * @increment 100
 * @reboot_required false
 * @group Predictive Traction Control
 */
PARAM_DEFINE_FLOAT(PTC_PRED_HOR, 1000.0f);

/**
 * Slip Weight
 *
 * Weight for slip penalty in cost function
 *
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 * @reboot_required false
 * @group Predictive Traction Control
 */
PARAM_DEFINE_FLOAT(PTC_WEIGHT_SLIP, 1.0f);

/**
 * Stability Weight
 *
 * Weight for stability penalty in cost function
 *
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 * @reboot_required false
 * @group Predictive Traction Control
 */
PARAM_DEFINE_FLOAT(PTC_WEIGHT_STAB, 1.0f);

/**
 * Control Weight
 *
 * Weight for control effort penalty in cost function
 *
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 * @reboot_required false
 * @group Predictive Traction Control
 */
PARAM_DEFINE_FLOAT(PTC_WEIGHT_CTRL, 0.1f);

/**
 * Maximum Torque Rate
 *
 * Maximum rate of change for torque commands
 *
 * @min 10.0
 * @max 1000.0
 * @decimal 0
 * @increment 10.0
 * @reboot_required false
 * @group Predictive Traction Control
 */
PARAM_DEFINE_FLOAT(PTC_MAX_TQ_RATE, 200.0f);

/**
 * Maximum Steering Rate
 *
 * Maximum rate of change for steering commands
 *
 * @unit rad/s
 * @min 0.09
 * @max 3.14
 * @decimal 2
 * @increment 0.09
 * @reboot_required false
 * @group Predictive Traction Control
 */
PARAM_DEFINE_FLOAT(PTC_MAX_ST_RATE, 0.785f);

/**
 * Learning Enable
 *
 * Enable adaptive learning for terrain parameters
 *
 * @boolean
 * @reboot_required false
 * @group Predictive Traction Control
 */
PARAM_DEFINE_INT32(PTC_LEARNING_EN, 1);

/**
 * Adaptation Rate
 *
 * Rate of adaptation for learning algorithms
 *
 * @min 0.001
 * @max 0.1
 * @decimal 3
 * @increment 0.001
 * @reboot_required false
 * @group Predictive Traction Control
 */
PARAM_DEFINE_FLOAT(PTC_ADAPT_RATE, 0.01f);

/**
 * Maximum Iterations
 *
 * Maximum iterations for optimization solver
 *
 * @min 5
 * @max 100
 * @decimal 0
 * @increment 1
 * @reboot_required false
 * @group Predictive Traction Control
 */
PARAM_DEFINE_INT32(PTC_MAX_ITER, 20);

/**
 * Convergence Tolerance
 *
 * Tolerance for optimization convergence
 *
 * @min 1e-6
 * @max 1e-2
 * @decimal 6
 * @increment 1e-6
 * @reboot_required false
 * @group Predictive Traction Control
 */
PARAM_DEFINE_FLOAT(PTC_CONV_TOL, 1e-4f);
