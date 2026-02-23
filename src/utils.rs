use crate::msg;
use crate::time::{Duration, Time};
use crate::transforms;

/// Calculates the inverse of a ros transform
pub fn get_inverse(transform: msg::TransformStamped) -> msg::TransformStamped {
    //let m_transform = to_transform(transform);
    let inverse = transforms::invert_transform(transform.transform);

    let inv = msg::TransformStamped {
        child_frame_id: transform.header.frame_id,
        header: msg::Header {
            frame_id: transform.child_frame_id,
            stamp: transform.header.stamp,
            seq: transform.header.seq,
        },
        transform: msg::Transform {
            rotation: msg::Quaternion {
                x: inverse.rotation.x,
                y: inverse.rotation.y,
                z: inverse.rotation.z,
                w: inverse.rotation.w,
            },
            translation: msg::Vector3 {
                x: inverse.translation.x,
                y: inverse.translation.y,
                z: inverse.translation.z,
            },
        },
    };
    inv
}

pub fn to_transform_stamped(
    transform: msg::Transform,
    from: std::string::String,
    to: std::string::String,
    time: Time,
) -> msg::TransformStamped {
    let transform_stamped_msg = msg::TransformStamped {
        child_frame_id: to,
        header: msg::Header {
            frame_id: from,
            stamp: time,
            seq: 0,
        },
        transform: transform,
    };
    transform_stamped_msg
}

pub fn get_nanos(dur: Duration) -> i64 {
    i64::from(dur.sec) * 1_000_000_000 + i64::from(dur.nsec)
}
