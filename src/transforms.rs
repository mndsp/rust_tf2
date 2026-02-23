use nalgebra::base;
use nalgebra::geometry;

use crate::msg;

// TODO find a better localization for this helper method
fn from_na_vector_to_vector_msg(vector: base::Vector3<f64>) -> msg::Vector3 {
    msg::Vector3 {
        x: vector[0],
        y: vector[1],
        z: vector[2],
    }
}

// TODO find a better localization for this helper method
fn from_vector_msg_to_na_vector(vector: msg::Vector3) -> base::Vector3<f64> {
    let msg::Vector3 { x, y, z } = vector;
    base::Vector3::new(x, y, z)
}

// TODO find a better localization for this helper method
fn from_quaternion_msg_to_na_quarternion(
    quaternion: msg::Quaternion,
) -> geometry::UnitQuaternion<f64> {
    let msg::Quaternion { x, y, z, w } = quaternion;
    geometry::UnitQuaternion::new_normalize(geometry::Quaternion::new(w, x, y, z))
}

// TODO find a better localization for this helper method
fn from_na_quaternion_to_quaternion_msg(quaternion: geometry::Quaternion<f64>) -> msg::Quaternion {
    msg::Quaternion {
        x: quaternion.coords[0],
        y: quaternion.coords[1],
        z: quaternion.coords[2],
        w: quaternion.coords[3],
    }
}

///Converts a transform from xyz translation + quaternion format to an SE3 matrix
pub fn isometry_from_transform_msg(transform: msg::Transform) -> geometry::Isometry3<f64> {
    let qt = from_quaternion_msg_to_na_quarternion(transform.rotation);
    let trans = from_vector_msg_to_na_vector(transform.translation);

    geometry::Isometry3::new(trans, qt.scaled_axis())
}

///Converts an SE3 matrix to a Transform
pub fn transform_msg_from_isometry(isometry: geometry::Isometry3<f64>) -> msg::Transform {
    msg::Transform {
        translation: from_na_vector_to_vector_msg(isometry.translation.vector),
        rotation: from_na_quaternion_to_quaternion_msg(*isometry.rotation),
    }
}

///Get the inverse transform
pub fn invert_transform(transform: msg::Transform) -> msg::Transform {
    let isometry = isometry_from_transform_msg(transform);
    let inverse_isometry = isometry.inverse();
    transform_msg_from_isometry(inverse_isometry)
}

///Chain multiple transforms together. Takes in a vector of transforms. The vector should be in order of desired transformations
pub fn chain_transforms(transforms: Vec<msg::Transform>) -> msg::Transform {
    let final_transform_opt = transforms
        .into_iter()
        .map(isometry_from_transform_msg)
        .reduce(|tf1, tf2| tf1 * tf2);

    if let Some(final_transform) = final_transform_opt {
        transform_msg_from_isometry(final_transform)
    } else {
        panic!("No transforms to chain")
    }
}

pub fn interpolate(t1: msg::Transform, t2: msg::Transform, weight: f64) -> msg::Transform {
    let iso1 = isometry_from_transform_msg(t1);
    let iso2 = isometry_from_transform_msg(t2);
    let res = iso1.try_lerp_slerp(&iso2, 1f64 - weight, 1e-9);
    match res {
        Some(iso) => transform_msg_from_isometry(iso),
        None => {
            let translation_lerp = iso1
                .translation
                .vector
                .lerp(&iso2.translation.vector, 1f64 - weight);
            if weight > 0.5 {
                msg::Transform {
                    translation: from_na_vector_to_vector_msg(translation_lerp),
                    rotation: from_na_quaternion_to_quaternion_msg(*iso1.rotation),
                }
            } else {
                msg::Transform {
                    translation: from_na_vector_to_vector_msg(translation_lerp),
                    rotation: from_na_quaternion_to_quaternion_msg(*iso2.rotation),
                }
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_basic_translation_chaining() {
        let tf1 = msg::Transform {
            translation: msg::Vector3 {
                x: 1f64,
                y: 1f64,
                z: 0f64,
            },
            rotation: msg::Quaternion {
                x: 0f64,
                y: 0f64,
                z: 0f64,
                w: 1f64,
            },
        };
        let expected_tf = msg::Transform {
            translation: msg::Vector3 {
                x: 2f64,
                y: 2f64,
                z: 0f64,
            },
            rotation: msg::Quaternion {
                x: 0f64,
                y: 0f64,
                z: 0f64,
                w: 1f64,
            },
        };
        let transform_chain = vec![tf1.clone(), tf1];
        let res = chain_transforms(transform_chain);
        assert_eq!(res, expected_tf);
    }

    #[test]
    fn test_basic_interpolation() {
        let tf1 = msg::Transform {
            translation: msg::Vector3 {
                x: 1f64,
                y: 1f64,
                z: 0f64,
            },
            rotation: msg::Quaternion {
                x: 0f64,
                y: 0f64,
                z: 0f64,
                w: 1f64,
            },
        };
        let tf2 = msg::Transform {
            translation: msg::Vector3 {
                x: 2f64,
                y: 2f64,
                z: 0f64,
            },
            rotation: msg::Quaternion {
                x: 0f64,
                y: 0f64,
                z: 0f64,
                w: 1f64,
            },
        };
        let expected = msg::Transform {
            translation: msg::Vector3 {
                x: 1.5f64,
                y: 1.5f64,
                z: 0f64,
            },
            rotation: msg::Quaternion {
                x: 0f64,
                y: 0f64,
                z: 0f64,
                w: 1f64,
            },
        };
        assert_eq!(interpolate(tf1, tf2, 0.5), expected);
    }
}
