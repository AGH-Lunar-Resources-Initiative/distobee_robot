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

export type ColorRGBA = {
  r?: number;
  g?: number;
  b?: number;
  a?: number;
};

// distobee_interfaces

export type WheelState = {
  velocity?: number;
  angle?: number;
};
export type WheelStates = {
  front_left?: WheelState;
  front_right?: WheelState;
  back_left?: WheelState;
  back_right?: WheelState;
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
