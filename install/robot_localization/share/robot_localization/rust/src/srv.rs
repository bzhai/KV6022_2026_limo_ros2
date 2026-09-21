#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to robot_localization__srv__FromLL_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FromLL_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ll_point: geographic_msgs::msg::GeoPoint,

}



impl Default for FromLL_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::FromLL_Request::default())
  }
}

impl rosidl_runtime_rs::Message for FromLL_Request {
  type RmwMsg = super::srv::rmw::FromLL_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ll_point: geographic_msgs::msg::GeoPoint::into_rmw_message(std::borrow::Cow::Owned(msg.ll_point)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ll_point: geographic_msgs::msg::GeoPoint::into_rmw_message(std::borrow::Cow::Borrowed(&msg.ll_point)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ll_point: geographic_msgs::msg::GeoPoint::from_rmw_message(msg.ll_point),
    }
  }
}


// Corresponds to robot_localization__srv__FromLL_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FromLL_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub map_point: geometry_msgs::msg::Point,

}



impl Default for FromLL_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::FromLL_Response::default())
  }
}

impl rosidl_runtime_rs::Message for FromLL_Response {
  type RmwMsg = super::srv::rmw::FromLL_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_point: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.map_point)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_point: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.map_point)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map_point: geometry_msgs::msg::Point::from_rmw_message(msg.map_point),
    }
  }
}


// Corresponds to robot_localization__srv__GetState_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetState_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub time_stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub frame_id: std::string::String,

}



impl Default for GetState_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetState_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetState_Request {
  type RmwMsg = super::srv::rmw::GetState_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.time_stamp)).into_owned(),
        frame_id: msg.frame_id.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_stamp)).into_owned(),
        frame_id: msg.frame_id.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      time_stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.time_stamp),
      frame_id: msg.frame_id.to_string(),
    }
  }
}


// Corresponds to robot_localization__srv__GetState_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetState_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub state: [f64; 15],

    /// Covariance matrix in row-major order
    #[cfg_attr(feature = "serde", serde(with = "serde_big_array::BigArray"))]
    pub covariance: [f64; 225],

}



impl Default for GetState_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetState_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetState_Response {
  type RmwMsg = super::srv::rmw::GetState_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        state: msg.state,
        covariance: msg.covariance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        state: msg.state,
        covariance: msg.covariance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      state: msg.state,
      covariance: msg.covariance,
    }
  }
}


// Corresponds to robot_localization__srv__SetDatum_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetDatum_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub geo_pose: geographic_msgs::msg::GeoPose,

}



impl Default for SetDatum_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetDatum_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetDatum_Request {
  type RmwMsg = super::srv::rmw::SetDatum_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        geo_pose: geographic_msgs::msg::GeoPose::into_rmw_message(std::borrow::Cow::Owned(msg.geo_pose)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        geo_pose: geographic_msgs::msg::GeoPose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.geo_pose)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      geo_pose: geographic_msgs::msg::GeoPose::from_rmw_message(msg.geo_pose),
    }
  }
}


// Corresponds to robot_localization__srv__SetDatum_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetDatum_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for SetDatum_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetDatum_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetDatum_Response {
  type RmwMsg = super::srv::rmw::SetDatum_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to robot_localization__srv__SetPose_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPose_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::PoseWithCovarianceStamped,

}



impl Default for SetPose_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetPose_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetPose_Request {
  type RmwMsg = super::srv::rmw::SetPose_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::PoseWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose: geometry_msgs::msg::PoseWithCovarianceStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose: geometry_msgs::msg::PoseWithCovarianceStamped::from_rmw_message(msg.pose),
    }
  }
}


// Corresponds to robot_localization__srv__SetPose_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPose_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for SetPose_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetPose_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetPose_Response {
  type RmwMsg = super::srv::rmw::SetPose_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to robot_localization__srv__ToggleFilterProcessing_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ToggleFilterProcessing_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub on: bool,

}



impl Default for ToggleFilterProcessing_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ToggleFilterProcessing_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ToggleFilterProcessing_Request {
  type RmwMsg = super::srv::rmw::ToggleFilterProcessing_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        on: msg.on,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      on: msg.on,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      on: msg.on,
    }
  }
}


// Corresponds to robot_localization__srv__ToggleFilterProcessing_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ToggleFilterProcessing_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: bool,

}



impl Default for ToggleFilterProcessing_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ToggleFilterProcessing_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ToggleFilterProcessing_Response {
  type RmwMsg = super::srv::rmw::ToggleFilterProcessing_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        status: msg.status,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      status: msg.status,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      status: msg.status,
    }
  }
}


// Corresponds to robot_localization__srv__ToLL_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ToLL_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub map_point: geometry_msgs::msg::Point,

}



impl Default for ToLL_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ToLL_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ToLL_Request {
  type RmwMsg = super::srv::rmw::ToLL_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_point: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.map_point)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        map_point: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.map_point)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      map_point: geometry_msgs::msg::Point::from_rmw_message(msg.map_point),
    }
  }
}


// Corresponds to robot_localization__srv__ToLL_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ToLL_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ll_point: geographic_msgs::msg::GeoPoint,

}



impl Default for ToLL_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ToLL_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ToLL_Response {
  type RmwMsg = super::srv::rmw::ToLL_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ll_point: geographic_msgs::msg::GeoPoint::into_rmw_message(std::borrow::Cow::Owned(msg.ll_point)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ll_point: geographic_msgs::msg::GeoPoint::into_rmw_message(std::borrow::Cow::Borrowed(&msg.ll_point)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ll_point: geographic_msgs::msg::GeoPoint::from_rmw_message(msg.ll_point),
    }
  }
}






#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_localization__srv__FromLL() -> *const std::ffi::c_void;
}

// Corresponds to robot_localization__srv__FromLL
#[allow(missing_docs, non_camel_case_types)]
pub struct FromLL;

impl rosidl_runtime_rs::Service for FromLL {
    type Request = FromLL_Request;
    type Response = FromLL_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_localization__srv__FromLL() }
    }
}




#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_localization__srv__GetState() -> *const std::ffi::c_void;
}

// Corresponds to robot_localization__srv__GetState
#[allow(missing_docs, non_camel_case_types)]
pub struct GetState;

impl rosidl_runtime_rs::Service for GetState {
    type Request = GetState_Request;
    type Response = GetState_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_localization__srv__GetState() }
    }
}




#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_localization__srv__SetDatum() -> *const std::ffi::c_void;
}

// Corresponds to robot_localization__srv__SetDatum
#[allow(missing_docs, non_camel_case_types)]
pub struct SetDatum;

impl rosidl_runtime_rs::Service for SetDatum {
    type Request = SetDatum_Request;
    type Response = SetDatum_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_localization__srv__SetDatum() }
    }
}




#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_localization__srv__SetPose() -> *const std::ffi::c_void;
}

// Corresponds to robot_localization__srv__SetPose
#[allow(missing_docs, non_camel_case_types)]
pub struct SetPose;

impl rosidl_runtime_rs::Service for SetPose {
    type Request = SetPose_Request;
    type Response = SetPose_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_localization__srv__SetPose() }
    }
}




#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_localization__srv__ToggleFilterProcessing() -> *const std::ffi::c_void;
}

// Corresponds to robot_localization__srv__ToggleFilterProcessing
#[allow(missing_docs, non_camel_case_types)]
pub struct ToggleFilterProcessing;

impl rosidl_runtime_rs::Service for ToggleFilterProcessing {
    type Request = ToggleFilterProcessing_Request;
    type Response = ToggleFilterProcessing_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_localization__srv__ToggleFilterProcessing() }
    }
}




#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_localization__srv__ToLL() -> *const std::ffi::c_void;
}

// Corresponds to robot_localization__srv__ToLL
#[allow(missing_docs, non_camel_case_types)]
pub struct ToLL;

impl rosidl_runtime_rs::Service for ToLL {
    type Request = ToLL_Request;
    type Response = ToLL_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_localization__srv__ToLL() }
    }
}


