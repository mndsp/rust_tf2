use crate::time::Time;
use std::cmp::Ordering;
use tf2_msgs;
use geometry_msgs;
use std_msgs;

/**
 * NewType pattern on geometry_msgs::TransformStamped
 */
#[derive(Debug, Clone, PartialEq)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: String,
    pub transform: Transform,
}

impl Eq for TransformStamped {}

impl Ord for TransformStamped {
    fn cmp(&self, other: &TransformStamped) -> Ordering {
        self.header.stamp.cmp(&other.header.stamp)
    }
}

impl PartialOrd for TransformStamped {
    fn partial_cmp(&self, other: &TransformStamped) -> Option<Ordering> {
        Some(self.header.stamp.cmp(&other.header.stamp))
    }
}

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Header {
    pub seq: u32,
    pub frame_id: String,
    pub stamp: Time,
}

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TFMessage {
    pub transforms: Vec<TransformStamped>,
}

impl From<std_msgs::msg::Header> for Header {
    fn from(msg: std_msgs::msg::Header) -> Self {
        Self {
            seq: 0,
            frame_id: msg.frame_id,
            stamp: Time::from(msg.stamp),
        }
    }
}

impl From<geometry_msgs::msg::Vector3> for Vector3 {
    fn from(msg: geometry_msgs::msg::Vector3) -> Self {
        Self {
            x: msg.x,
            y: msg.y,
            z: msg.z,
        }
    }
}

impl From<geometry_msgs::msg::Quaternion> for Quaternion {
    fn from(msg: geometry_msgs::msg::Quaternion) -> Self {
        Self {
            x: msg.x,
            y: msg.y,
            z: msg.z,
            w: msg.w,
        }
    }
}

impl From<geometry_msgs::msg::Transform> for Transform {
    fn from(msg: geometry_msgs::msg::Transform) -> Self {
        Self {
            translation: Vector3::from(msg.translation),
            rotation: Quaternion::from(msg.rotation),
        }
    }
}

impl From<geometry_msgs::msg::TransformStamped> for TransformStamped {
    fn from(msg: geometry_msgs::msg::TransformStamped) -> Self {
        Self {
            header: Header::from(msg.header),
            child_frame_id: msg.child_frame_id,
            transform: Transform::from(msg.transform),
        }
    }
}

impl From<tf2_msgs::msg::TFMessage> for TFMessage {
    fn from(msg: tf2_msgs::msg::TFMessage) -> Self {
        Self {
            transforms: msg.transforms
                .into_iter()
                .map(TransformStamped::from)
                .collect(),
        }
    }
}
