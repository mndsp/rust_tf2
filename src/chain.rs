use crate::core::TfError;
use crate::msg;
use crate::time::Time;
use crate::transforms;
use crate::utils::{get_nanos, to_transform_stamped};

#[derive(Clone, Debug)]
pub struct TfIndividualTransformChain {
    buffer_size: usize,
    static_tf: bool,
    //TODO:  Implement a circular buffer. Current method is slowww.
    transform_chain: Vec<msg::TransformStamped>,
}

impl TfIndividualTransformChain {
    pub fn new(static_tf: bool) -> Self {
        return TfIndividualTransformChain {
            buffer_size: 100,
            transform_chain: Vec::new(),
            static_tf: static_tf,
        };
    }

    pub fn add_to_buffer(&mut self, msg: msg::TransformStamped) {
        let res = self.transform_chain.binary_search(&msg);

        match res {
            Ok(x) => self.transform_chain.insert(x, msg),
            Err(x) => self.transform_chain.insert(x, msg),
        }

        if self.transform_chain.len() > self.buffer_size {
            self.transform_chain.remove(0);
        }
    }

    pub fn get_closest_transform(&self, time: Time) -> Result<msg::TransformStamped, TfError> {
        if self.static_tf {
            return Ok(self
                .transform_chain
                .get(self.transform_chain.len() - 1)
                .unwrap()
                .clone());
        }

        let res = msg::TransformStamped {
            child_frame_id: "".to_string(),
            header: msg::Header {
                frame_id: "".to_string(),
                stamp: time,
                seq: 1,
            },
            transform: msg::Transform {
                rotation: msg::Quaternion {
                    x: 0f64,
                    y: 0f64,
                    z: 0f64,
                    w: 1f64,
                },
                translation: msg::Vector3 {
                    x: 0f64,
                    y: 0f64,
                    z: 0f64,
                },
            },
        };

        let res = self.transform_chain.binary_search(&res);
        match res {
            Ok(x) => return Ok(self.transform_chain.get(x).unwrap().clone()),
            Err(x) => {
                if x == 0 {
                    return Err(TfError::AttemptedLookupInPast);
                }
                if x >= self.transform_chain.len() {
                    return Err(TfError::AttemptedLookUpInFuture);
                }
                let tf1 = self.transform_chain.get(x - 1).unwrap().clone();
                let tf2 = self.transform_chain.get(x).unwrap().clone();
                let time1 = self.transform_chain.get(x - 1).unwrap().header.stamp;
                let time2 = self.transform_chain.get(x).unwrap().header.stamp;
                let header = self.transform_chain.get(x).unwrap().header.clone();
                let child_frame = self.transform_chain.get(x).unwrap().child_frame_id.clone();
                let total_duration = get_nanos(time2 - time1) as f64;
                let desired_duration = get_nanos(time - time1) as f64;
                let weight = 1.0 - desired_duration / total_duration;
                let final_tf = transforms::interpolate(tf1.transform, tf2.transform, weight);
                let ros_msg = to_transform_stamped(final_tf, header.frame_id, child_frame, time);
                Ok(ros_msg)
            }
        }
    }
}
