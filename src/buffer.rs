use crate::time::{Duration, Time};
use std::collections::HashMap;
use std::collections::HashSet;
use std::collections::VecDeque;

// use rosrust;

use crate::chain::TfIndividualTransformChain;
use crate::core::{TfError, TransformInterface, TransformWithTimeInterface};
use crate::graph::TfGraphNode;
use crate::msg;
use crate::transforms;
use crate::utils::{get_inverse, to_transform_stamped};

const _DEFAULT_CACHE_TIME: i32 = 10;
const _MAX_GRAPH_DEPTH: u32 = 1000;

#[derive(Clone, Debug)]
pub struct TfBuffer {
    child_transform_index: HashMap<String, HashSet<String>>,
    transform_data: HashMap<TfGraphNode, TfIndividualTransformChain>,
}

impl TfBuffer {
    pub fn new() -> TfBuffer {
        TfBuffer {
            child_transform_index: HashMap::new(),
            transform_data: HashMap::new(),
        }
    }

    pub fn handle_incoming_transforms(&mut self, transforms: msg::TFMessage, static_tf: bool) {
        for transform in transforms.transforms {
            let inverse_transform = get_inverse(transform.clone());
            self.add_transform(transform, static_tf);
            self.add_transform(inverse_transform, static_tf);
        }
    }

    fn add_transform(&mut self, transform: msg::TransformStamped, static_tf: bool) {
        //TODO: Detect is new transform will create a loop
        if self
            .child_transform_index
            .contains_key(&transform.header.frame_id)
        {
            let res = self
                .child_transform_index
                .get_mut(&transform.header.frame_id.clone())
                .unwrap();
            res.insert(transform.child_frame_id.clone());
        } else {
            self.child_transform_index
                .insert(transform.header.frame_id.clone(), HashSet::new());
            let res = self
                .child_transform_index
                .get_mut(&transform.header.frame_id.clone())
                .unwrap();
            res.insert(transform.child_frame_id.clone());
        }

        let key = TfGraphNode {
            child: transform.child_frame_id.clone(),
            parent: transform.header.frame_id.clone(),
        };

        if self.transform_data.contains_key(&key) {
            let data = self.transform_data.get_mut(&key).unwrap();
            data.add_to_buffer(transform.clone());
        } else {
            let mut data = TfIndividualTransformChain::new(static_tf);
            data.add_to_buffer(transform.clone());
            self.transform_data.insert(key, data);
        }
    }

    /// Retrieves the transform path
    fn retrieve_transform_path(&self, from: String, to: String) -> Result<Vec<String>, TfError> {
        let mut res = vec![];
        let mut frontier: VecDeque<String> = VecDeque::new();
        let mut visited: HashSet<String> = HashSet::new();
        let mut parents: HashMap<String, String> = HashMap::new();
        visited.insert(from.clone());
        frontier.push_front(from.clone());

        while !frontier.is_empty() {
            let current_node = frontier.pop_front().unwrap();
            if current_node == to {
                break;
            }
            let children = self.child_transform_index.get(&current_node);
            match children {
                Some(children) => {
                    for v in children {
                        if visited.contains(&v.to_string()) {
                            continue;
                        }
                        parents.insert(v.to_string(), current_node.clone());
                        frontier.push_front(v.to_string());
                        visited.insert(v.to_string());
                    }
                }
                None => {}
            }
        }
        let mut r = to;
        while r != from {
            res.push(r.clone());
            let parent = parents.get(&r);

            match parent {
                Some(x) => r = x.to_string(),
                None => return Err(TfError::CouldNotFindTransform),
            }
        }
        res.reverse();
        Ok(res)
    }
}

impl TransformInterface for TfBuffer {
    /// Looks up a transform within the tree at a given time.
    fn lookup_transform(
        &self,
        source_frame: &str,
        target_frame: &str,
        time: Time,
    ) -> Result<msg::TransformStamped, TfError> {
        let source_frame = source_frame.to_string();
        let target_frame = target_frame.to_string();
        let path = self.retrieve_transform_path(source_frame.clone(), target_frame.clone())?;

        let mut tflist = Vec::<msg::Transform>::new();
        let mut first = source_frame.clone();
        for intermediate in path {
            let node = TfGraphNode {
                child: intermediate.clone(),
                parent: first.clone(),
            };
            let time_cache = self.transform_data.get(&node).unwrap();
            let transform = time_cache.get_closest_transform(time)?.transform.clone();
            let tf = msg::Transform {
                rotation: msg::Quaternion {
                    x: transform.rotation.x,
                    y: transform.rotation.y,
                    z: transform.rotation.z,
                    w: transform.rotation.w,
                },
                translation: msg::Vector3 {
                    x: transform.translation.x,
                    y: transform.translation.y,
                    z: transform.translation.z,
                },
            };
            tflist.push(tf);
            first = intermediate.clone();
        }
        let final_tf = transforms::chain_transforms(tflist);
        let msg = msg::TransformStamped {
            child_frame_id: target_frame.clone(),
            header: msg::Header {
                frame_id: source_frame.clone(),
                stamp: time,
                seq: 1,
            },
            transform: msg::Transform {
                rotation: msg::Quaternion {
                    x: final_tf.rotation.x,
                    y: final_tf.rotation.y,
                    z: final_tf.rotation.z,
                    w: final_tf.rotation.w,
                },
                translation: msg::Vector3 {
                    x: final_tf.translation.x,
                    y: final_tf.translation.y,
                    z: final_tf.rotation.z,
                },
            },
        };
        Ok(msg)
    }

    // TODO(MathuxNY-73) implement those methods
    fn can_transform(
        &self,
        _target_frame: &str,
        _source_frame: &str,
        _time: Time,
        _timeout: Duration,
    ) -> Result<bool, TfError> {
        todo!()
    }

    fn transform_to_output<'a, T>(
        &self,
        _input: &'a T,
        _output: &'a T,
        _target_frame: &str,
        _timeout: Option<Duration>,
    ) -> &'a T {
        todo!()
    }
    fn transform_from_input<T>(&self, _input: T, _target: &str, _timeout: Option<Duration>) -> T {
        todo!()
    }
}

impl TransformWithTimeInterface for TfBuffer {
    fn lookup_transform_with_time_travel(
        &self,
        target_frame: &str,
        target_time: Time,
        source_frame: &str,
        source_time: Time,
        fixed_frame: &str,
        _timeout: Duration,
    ) -> Result<msg::TransformStamped, TfError> {
        let source_tf = self.lookup_transform(source_frame, fixed_frame, source_time)?;
        let target_tf = self.lookup_transform(target_frame, fixed_frame, target_time)?;

        let transforms = transforms::invert_transform(source_tf.transform); // TODO same here it seems that source_tf could be moved.

        let result = transforms::chain_transforms(vec![target_tf.transform, transforms]);

        Ok(to_transform_stamped(
            result,
            source_frame.to_string(),
            target_frame.to_string(),
            source_time,
        ))
    }

    fn can_transform_with_time_travel(
        &self,
        _target_frame: &str,
        _target_time: Time,
        _source_frame: &str,
        _source_time: Time,
        _fixed_frame: &str,
        _timeout: Duration,
    ) -> Result<bool, TfError> {
        todo!()
    }
}

#[cfg(test)]
mod test {
    use super::*;
    /// This function builds a tree consisting of the following items:
    /// * a world coordinate frame
    /// * an item in the world frame at (1,0,0)
    /// * base_link of a robot starting at (0,0,0) and progressing at (0,t,0) where t is time in seconds
    /// * a camera which is (0.5, 0, 0) from the base_link
    fn build_test_tree(buffer: &mut TfBuffer, time: f64) {
        let nsecs = ((time - ((time.floor() as i64) as f64)) * 1E9) as u32;

        let world_to_item = msg::TransformStamped {
            child_frame_id: "item".to_string(),
            header: msg::Header {
                frame_id: "world".to_string(),
                stamp: Time {
                    sec: time.floor() as u32,
                    nsec: nsecs,
                },
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
                    x: 1f64,
                    y: 0f64,
                    z: 0f64,
                },
            },
        };
        let world_to_item_inverse = get_inverse(world_to_item.clone());
        buffer.add_transform(world_to_item, true);
        buffer.add_transform(world_to_item_inverse, true);

        let world_to_base_link = msg::TransformStamped {
            child_frame_id: "base_link".to_string(),
            header: msg::Header {
                frame_id: "world".to_string(),
                stamp: Time {
                    sec: time.floor() as u32,
                    nsec: nsecs,
                },
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
                    y: time,
                    z: 0f64,
                },
            },
        };
        let world_to_base_link_inv = get_inverse(world_to_base_link.clone());
        buffer.add_transform(world_to_base_link, false);
        buffer.add_transform(world_to_base_link_inv, false);

        let base_link_to_camera = msg::TransformStamped {
            child_frame_id: "camera".to_string(),
            header: msg::Header {
                frame_id: "base_link".to_string(),
                stamp: Time {
                    sec: time.floor() as u32,
                    nsec: nsecs,
                },
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
                    x: 0.5f64,
                    y: 0f64,
                    z: 0f64,
                },
            },
        };
        let base_link_to_camera_inv = get_inverse(base_link_to_camera.clone());
        buffer.add_transform(base_link_to_camera, true);
        buffer.add_transform(base_link_to_camera_inv, true);
    }

    /// Tests a basic lookup
    #[test]
    fn test_basic_tf_lookup() {
        let mut tf_buffer = TfBuffer::new();
        build_test_tree(&mut tf_buffer, 0f64);
        let res = tf_buffer.lookup_transform("camera", "item", Time { sec: 0, nsec: 0 });
        let expected = msg::TransformStamped {
            child_frame_id: "item".to_string(),
            header: msg::Header {
                frame_id: "camera".to_string(),
                stamp: Time { sec: 0, nsec: 0 },
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
                    x: 0.5f64,
                    y: 0f64,
                    z: 0f64,
                },
            },
        };
        assert_eq!(res.unwrap(), expected);
    }

    /// Tests an interpolated lookup.
    #[test]
    fn test_basic_tf_interpolation() {
        let mut tf_buffer = TfBuffer::new();
        build_test_tree(&mut tf_buffer, 0f64);
        build_test_tree(&mut tf_buffer, 1f64);
        let res = tf_buffer.lookup_transform(
            "camera",
            "item",
            Time {
                sec: 0,
                nsec: 700_000_000,
            },
        );
        let expected = msg::TransformStamped {
            child_frame_id: "item".to_string(),
            header: msg::Header {
                frame_id: "camera".to_string(),
                stamp: Time {
                    sec: 0,
                    nsec: 700_000_000,
                },
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
                    x: 0.5f64,
                    y: -0.7f64,
                    z: 0f64,
                },
            },
        };
        assert_eq!(res.unwrap(), expected);
    }

    /// Tests an interpolated lookup.
    #[test]
    fn test_basic_tf_timetravel() {
        let mut tf_buffer = TfBuffer::new();
        build_test_tree(&mut tf_buffer, 0f64);
        build_test_tree(&mut tf_buffer, 1f64);
        let res = tf_buffer.lookup_transform_with_time_travel(
            "camera",
            Time {
                sec: 0,
                nsec: 400_000_000,
            },
            "camera",
            Time {
                sec: 0,
                nsec: 700_000_000,
            },
            "item",
            Duration {
                sec: 0,
                nsec: 700_000_000,
            },
        );
        let expected = msg::TransformStamped {
            child_frame_id: "camera".to_string(),
            header: msg::Header {
                frame_id: "camera".to_string(),
                stamp: Time {
                    sec: 0,
                    nsec: 700_000_000,
                },
                seq: 0,
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
                    y: 0.3f64,
                    z: 0f64,
                },
            },
        };
        assert_approx_eq(res.unwrap(), expected);
    }

    fn assert_approx_eq(msg1: msg::TransformStamped, msg2: msg::TransformStamped) {
        assert_eq!(msg1.header, msg2.header);
        assert_eq!(msg1.child_frame_id, msg2.child_frame_id);

        assert!((msg1.transform.rotation.x - msg2.transform.rotation.x).abs() < 1e-9);
        assert!((msg1.transform.rotation.y - msg2.transform.rotation.y).abs() < 1e-9);
        assert!((msg1.transform.rotation.z - msg2.transform.rotation.z).abs() < 1e-9);
        assert!((msg1.transform.rotation.w - msg2.transform.rotation.w).abs() < 1e-9);

        assert!((msg1.transform.translation.x - msg2.transform.translation.x).abs() < 1e-9);
        assert!((msg1.transform.translation.y - msg2.transform.translation.y).abs() < 1e-9);
        assert!((msg1.transform.translation.z - msg2.transform.translation.z).abs() < 1e-9);
    }
}
