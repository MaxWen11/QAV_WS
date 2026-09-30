Header header

float64 des_v_x
float64 des_v_y
float64 des_v_z

float64 fb_a_x
float64 fb_a_y
float64 fb_a_z

float64 des_a_x
float64 des_a_y
float64 des_a_z

float64 des_q_x
float64 des_q_y
float64 des_q_z
float64 des_q_w

float64 des_thr
float64 hover_percentage
float64 thr_scale_compensate
float64 voltage

float64 err_axisang_x
float64 err_axisang_y
float64 err_axisang_z
float64 err_axisang_ang

float64 fb_rate_x
float64 fb_rate_y
float64 fb_rate_z

float64 exec_err_axisang_x
float64 exec_err_axisang_y
float64 exec_err_axisang_z
float64 exec_err_axisang_ang

float64 fb_axisang_x
float64 fb_axisang_y
float64 fb_axisang_z
float64 fb_axisang_ang

float64 des_p_x
float64 des_p_y
float64 des_p_z

float64 eso_vel_x
float64 eso_vel_y
float64 eso_vel_z

float64 real_x
float64 real_y
float64 real_z

float64 real_vx
float64 real_vy
float64 real_vz

# UADL online diagnostics. des_a_* is the executed net inertial acceleration
# reconstructed from the final attitude/thrust command; fb_a_* is the measured
# net inertial acceleration (IMU specific force rotated into the world frame).
bool controller_valid
bool used_backup
bool gp_sample_inserted
bool gp_sample_rejected
bool envelope_saturated
bool inside_analysis_domain
string online_status
float64 cycle_ms
float64[3] eta
float64[3] gp_f
float64[3] gp_g
float64[3] gp_cov_fg
float64[3] response_sigma
float64[3] response_error_bound
float64[3] residual_envelope
float64[3] command_error
float64[3] nominal_position_error
float64[3] tube_position_radius
float64[3] tube_velocity_radius
float64[3] input_reserve
float64[3] tracking_slack
uint32[3] gp_window_size
