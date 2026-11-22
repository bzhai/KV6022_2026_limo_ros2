#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "limo_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__limo_msgs__msg__LimoStatus() -> *const std::ffi::c_void;
}

#[link(name = "limo_msgs__rosidl_generator_c")]
extern "C" {
    fn limo_msgs__msg__LimoStatus__init(msg: *mut LimoStatus) -> bool;
    fn limo_msgs__msg__LimoStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LimoStatus>, size: usize) -> bool;
    fn limo_msgs__msg__LimoStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LimoStatus>);
    fn limo_msgs__msg__LimoStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LimoStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<LimoStatus>) -> bool;
}

// Corresponds to limo_msgs__msg__LimoStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LimoStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vehicle_state: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub control_mode: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub battery_voltage: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error_code: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub motion_mode: u8,

}



impl Default for LimoStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !limo_msgs__msg__LimoStatus__init(&mut msg as *mut _) {
        panic!("Call to limo_msgs__msg__LimoStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LimoStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { limo_msgs__msg__LimoStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { limo_msgs__msg__LimoStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { limo_msgs__msg__LimoStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LimoStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LimoStatus where Self: Sized {
  const TYPE_NAME: &'static str = "limo_msgs/msg/LimoStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__limo_msgs__msg__LimoStatus() }
  }
}


