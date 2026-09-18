#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to limo_msgs__msg__LimoStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LimoStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LimoStatus::default())
  }
}

impl rosidl_runtime_rs::Message for LimoStatus {
  type RmwMsg = super::msg::rmw::LimoStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        vehicle_state: msg.vehicle_state,
        control_mode: msg.control_mode,
        battery_voltage: msg.battery_voltage,
        error_code: msg.error_code,
        motion_mode: msg.motion_mode,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      vehicle_state: msg.vehicle_state,
      control_mode: msg.control_mode,
      battery_voltage: msg.battery_voltage,
      error_code: msg.error_code,
      motion_mode: msg.motion_mode,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      vehicle_state: msg.vehicle_state,
      control_mode: msg.control_mode,
      battery_voltage: msg.battery_voltage,
      error_code: msg.error_code,
      motion_mode: msg.motion_mode,
    }
  }
}


