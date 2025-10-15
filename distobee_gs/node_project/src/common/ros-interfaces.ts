// Standard ROS types

export type Time = {
  sec?: number;
  nanosec?: number;
};

export type Header = {
  stamp?: Time;
  frame_id?: string;
};

export type NavSatStatus = {
  status?: number;
  service?: number;
};

export type NavSatFix = {
  header?: Header;
  status?: NavSatStatus;
  latitude?: number;
  longitude?: number;
  altitude?: number;
  position_covariance?: number[];
};

export type Quaternion = {
  x?: number;
  y?: number;
  z?: number;
  w?: number;
};

export type Vector3 = {
  x?: number;
  y?: number;
  z?: number;
};

export type Imu = {
  header?: Header;
  orientation?: Quaternion;
  orientation_covariance?: number[];
  angular_velocity?: Vector3;
  angular_velocity_covariance?: number[];
  linear_acceleration?: Vector3;
  linear_acceleration_covariance?: number[];
};

export type Point = {
  x?: number;
  y?: number;
  z?: number;
};

export type PointStamped = {
  header?: Header;
  point?: Point;
};

export type GeoPoint = {
  latitude?: number;
  longitude?: number;
  altitude?: number;
};

export type Twist = {
  linear?: Vector3;
  angular?: Vector3;
};

export type Bool = {
  data?: boolean;
};

export type Pose = {
  position?: Point;
  orientation?: Quaternion;
};

export type PoseWithCovariance = {
  pose?: Pose;
  covariance?: number[];
};

export type TwistWithCovariance = {
  twist?: Twist;
  covariance?: number[];
};

export type Odometry = {
  header?: Header;
  pose?: PoseWithCovariance;
  twist?: TwistWithCovariance;
};

// std_srvs/SetBool
export type SetBoolRequest = {
  data?: boolean;
};
export type SetBoolResponse = {
  success?: boolean;
  message?: string;
};

// std_srvs/Empty
export type EmptyRequest = {};
export type EmptyResponse = {};

export type ColorRGBA = {
  r?: number;
  g?: number;
  b?: number;
  a?: number;
};

// distobee_interfaces
export type WheelStates = {
  back_left_velocity?: number;
  back_right_velocity?: number;
  front_left_angle?: number;
  front_right_angle?: number;
};

export type WheelTelemetry = {
  state?: number;
};

// odrive_can/AxisState service
export type AxisStateRequest = {
  axis_requested_state?: number;
};

export type AxisStateResponse = {
  active_errors?: number;
  axis_state?: number;
  procedure_result?: number;
};

export type PipeStates = {
  left?: boolean;
  right?: boolean;
};

export type SetFeedRequest = {
  feed?: number;
  camera?: number;
  channel?: number;
  power?: number;
};
// Response is empty.

export type ControllerStatus = {
  pos_estimate?: number;
  vel_estimate?: number;
  torque_target?: number;
  torque_estimate?: number;
  iq_setpoint?: number;
  iq_measured?: number;
  active_errors?: number;
  axis_state?: number;
  procedure_result?: number;
  trajectory_done_flag?: boolean;
};

export type ControlMessage = {
  control_mode?: number;
  input_mode?: number;
  input_pos?: number;
  input_vel?: number;
  input_torque?: number;
};
