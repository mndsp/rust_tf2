use crate::time::Time;
use std::cmp::Ordering;

pub trait MessageConverter<Output = Self> {
    type MessageType;

    fn from_msg(msg: Self::MessageType) -> Output;
    fn to_msg(output: Output) -> Self::MessageType;
}

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
