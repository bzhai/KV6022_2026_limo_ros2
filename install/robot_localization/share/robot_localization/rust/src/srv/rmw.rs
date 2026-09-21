#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__FromLL_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_localization__rosidl_generator_c")]
extern "C" {
    fn robot_localization__srv__FromLL_Request__init(msg: *mut FromLL_Request) -> bool;
    fn robot_localization__srv__FromLL_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FromLL_Request>, size: usize) -> bool;
    fn robot_localization__srv__FromLL_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FromLL_Request>);
    fn robot_localization__srv__FromLL_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FromLL_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FromLL_Request>) -> bool;
}

// Corresponds to robot_localization__srv__FromLL_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FromLL_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ll_point: geographic_msgs::msg::rmw::GeoPoint,

}



impl Default for FromLL_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_localization__srv__FromLL_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_localization__srv__FromLL_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FromLL_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__FromLL_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__FromLL_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__FromLL_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FromLL_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FromLL_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_localization/srv/FromLL_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__FromLL_Request() }
  }
}


#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__FromLL_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_localization__rosidl_generator_c")]
extern "C" {
    fn robot_localization__srv__FromLL_Response__init(msg: *mut FromLL_Response) -> bool;
    fn robot_localization__srv__FromLL_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FromLL_Response>, size: usize) -> bool;
    fn robot_localization__srv__FromLL_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FromLL_Response>);
    fn robot_localization__srv__FromLL_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FromLL_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FromLL_Response>) -> bool;
}

// Corresponds to robot_localization__srv__FromLL_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FromLL_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub map_point: geometry_msgs::msg::rmw::Point,

}



impl Default for FromLL_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_localization__srv__FromLL_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_localization__srv__FromLL_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FromLL_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__FromLL_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__FromLL_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__FromLL_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FromLL_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FromLL_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_localization/srv/FromLL_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__FromLL_Response() }
  }
}


#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__GetState_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_localization__rosidl_generator_c")]
extern "C" {
    fn robot_localization__srv__GetState_Request__init(msg: *mut GetState_Request) -> bool;
    fn robot_localization__srv__GetState_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetState_Request>, size: usize) -> bool;
    fn robot_localization__srv__GetState_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetState_Request>);
    fn robot_localization__srv__GetState_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetState_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetState_Request>) -> bool;
}

// Corresponds to robot_localization__srv__GetState_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetState_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub time_stamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub frame_id: rosidl_runtime_rs::String,

}



impl Default for GetState_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_localization__srv__GetState_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_localization__srv__GetState_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetState_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__GetState_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__GetState_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__GetState_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetState_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetState_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_localization/srv/GetState_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__GetState_Request() }
  }
}


#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__GetState_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_localization__rosidl_generator_c")]
extern "C" {
    fn robot_localization__srv__GetState_Response__init(msg: *mut GetState_Response) -> bool;
    fn robot_localization__srv__GetState_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetState_Response>, size: usize) -> bool;
    fn robot_localization__srv__GetState_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetState_Response>);
    fn robot_localization__srv__GetState_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetState_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetState_Response>) -> bool;
}

// Corresponds to robot_localization__srv__GetState_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_localization__srv__GetState_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_localization__srv__GetState_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetState_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__GetState_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__GetState_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__GetState_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetState_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetState_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_localization/srv/GetState_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__GetState_Response() }
  }
}


#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__SetDatum_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_localization__rosidl_generator_c")]
extern "C" {
    fn robot_localization__srv__SetDatum_Request__init(msg: *mut SetDatum_Request) -> bool;
    fn robot_localization__srv__SetDatum_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetDatum_Request>, size: usize) -> bool;
    fn robot_localization__srv__SetDatum_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetDatum_Request>);
    fn robot_localization__srv__SetDatum_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetDatum_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetDatum_Request>) -> bool;
}

// Corresponds to robot_localization__srv__SetDatum_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetDatum_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub geo_pose: geographic_msgs::msg::rmw::GeoPose,

}



impl Default for SetDatum_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_localization__srv__SetDatum_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_localization__srv__SetDatum_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetDatum_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__SetDatum_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__SetDatum_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__SetDatum_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetDatum_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetDatum_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_localization/srv/SetDatum_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__SetDatum_Request() }
  }
}


#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__SetDatum_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_localization__rosidl_generator_c")]
extern "C" {
    fn robot_localization__srv__SetDatum_Response__init(msg: *mut SetDatum_Response) -> bool;
    fn robot_localization__srv__SetDatum_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetDatum_Response>, size: usize) -> bool;
    fn robot_localization__srv__SetDatum_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetDatum_Response>);
    fn robot_localization__srv__SetDatum_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetDatum_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetDatum_Response>) -> bool;
}

// Corresponds to robot_localization__srv__SetDatum_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetDatum_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for SetDatum_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_localization__srv__SetDatum_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_localization__srv__SetDatum_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetDatum_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__SetDatum_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__SetDatum_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__SetDatum_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetDatum_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetDatum_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_localization/srv/SetDatum_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__SetDatum_Response() }
  }
}


#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__SetPose_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_localization__rosidl_generator_c")]
extern "C" {
    fn robot_localization__srv__SetPose_Request__init(msg: *mut SetPose_Request) -> bool;
    fn robot_localization__srv__SetPose_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetPose_Request>, size: usize) -> bool;
    fn robot_localization__srv__SetPose_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetPose_Request>);
    fn robot_localization__srv__SetPose_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetPose_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetPose_Request>) -> bool;
}

// Corresponds to robot_localization__srv__SetPose_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPose_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose: geometry_msgs::msg::rmw::PoseWithCovarianceStamped,

}



impl Default for SetPose_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_localization__srv__SetPose_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_localization__srv__SetPose_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetPose_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__SetPose_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__SetPose_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__SetPose_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetPose_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetPose_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_localization/srv/SetPose_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__SetPose_Request() }
  }
}


#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__SetPose_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_localization__rosidl_generator_c")]
extern "C" {
    fn robot_localization__srv__SetPose_Response__init(msg: *mut SetPose_Response) -> bool;
    fn robot_localization__srv__SetPose_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetPose_Response>, size: usize) -> bool;
    fn robot_localization__srv__SetPose_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetPose_Response>);
    fn robot_localization__srv__SetPose_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetPose_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetPose_Response>) -> bool;
}

// Corresponds to robot_localization__srv__SetPose_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPose_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for SetPose_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_localization__srv__SetPose_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_localization__srv__SetPose_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetPose_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__SetPose_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__SetPose_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__SetPose_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetPose_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetPose_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_localization/srv/SetPose_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__SetPose_Response() }
  }
}


#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__ToggleFilterProcessing_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_localization__rosidl_generator_c")]
extern "C" {
    fn robot_localization__srv__ToggleFilterProcessing_Request__init(msg: *mut ToggleFilterProcessing_Request) -> bool;
    fn robot_localization__srv__ToggleFilterProcessing_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ToggleFilterProcessing_Request>, size: usize) -> bool;
    fn robot_localization__srv__ToggleFilterProcessing_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ToggleFilterProcessing_Request>);
    fn robot_localization__srv__ToggleFilterProcessing_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ToggleFilterProcessing_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ToggleFilterProcessing_Request>) -> bool;
}

// Corresponds to robot_localization__srv__ToggleFilterProcessing_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ToggleFilterProcessing_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub on: bool,

}



impl Default for ToggleFilterProcessing_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_localization__srv__ToggleFilterProcessing_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_localization__srv__ToggleFilterProcessing_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ToggleFilterProcessing_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__ToggleFilterProcessing_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__ToggleFilterProcessing_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__ToggleFilterProcessing_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ToggleFilterProcessing_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ToggleFilterProcessing_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_localization/srv/ToggleFilterProcessing_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__ToggleFilterProcessing_Request() }
  }
}


#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__ToggleFilterProcessing_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_localization__rosidl_generator_c")]
extern "C" {
    fn robot_localization__srv__ToggleFilterProcessing_Response__init(msg: *mut ToggleFilterProcessing_Response) -> bool;
    fn robot_localization__srv__ToggleFilterProcessing_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ToggleFilterProcessing_Response>, size: usize) -> bool;
    fn robot_localization__srv__ToggleFilterProcessing_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ToggleFilterProcessing_Response>);
    fn robot_localization__srv__ToggleFilterProcessing_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ToggleFilterProcessing_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ToggleFilterProcessing_Response>) -> bool;
}

// Corresponds to robot_localization__srv__ToggleFilterProcessing_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ToggleFilterProcessing_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: bool,

}



impl Default for ToggleFilterProcessing_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_localization__srv__ToggleFilterProcessing_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_localization__srv__ToggleFilterProcessing_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ToggleFilterProcessing_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__ToggleFilterProcessing_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__ToggleFilterProcessing_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__ToggleFilterProcessing_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ToggleFilterProcessing_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ToggleFilterProcessing_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_localization/srv/ToggleFilterProcessing_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__ToggleFilterProcessing_Response() }
  }
}


#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__ToLL_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_localization__rosidl_generator_c")]
extern "C" {
    fn robot_localization__srv__ToLL_Request__init(msg: *mut ToLL_Request) -> bool;
    fn robot_localization__srv__ToLL_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ToLL_Request>, size: usize) -> bool;
    fn robot_localization__srv__ToLL_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ToLL_Request>);
    fn robot_localization__srv__ToLL_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ToLL_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ToLL_Request>) -> bool;
}

// Corresponds to robot_localization__srv__ToLL_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ToLL_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub map_point: geometry_msgs::msg::rmw::Point,

}



impl Default for ToLL_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_localization__srv__ToLL_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_localization__srv__ToLL_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ToLL_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__ToLL_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__ToLL_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__ToLL_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ToLL_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ToLL_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_localization/srv/ToLL_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__ToLL_Request() }
  }
}


#[link(name = "robot_localization__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__ToLL_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_localization__rosidl_generator_c")]
extern "C" {
    fn robot_localization__srv__ToLL_Response__init(msg: *mut ToLL_Response) -> bool;
    fn robot_localization__srv__ToLL_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ToLL_Response>, size: usize) -> bool;
    fn robot_localization__srv__ToLL_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ToLL_Response>);
    fn robot_localization__srv__ToLL_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ToLL_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ToLL_Response>) -> bool;
}

// Corresponds to robot_localization__srv__ToLL_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ToLL_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ll_point: geographic_msgs::msg::rmw::GeoPoint,

}



impl Default for ToLL_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_localization__srv__ToLL_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_localization__srv__ToLL_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ToLL_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__ToLL_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__ToLL_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_localization__srv__ToLL_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ToLL_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ToLL_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_localization/srv/ToLL_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_localization__srv__ToLL_Response() }
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


