#![cfg(all(feature = "std", feature = "fixed"))]

use fixed::{
    traits::{Fixed, ToFixed},
    types::I32F32,
};

use proptest::prelude::*;

use guv::std::fixed::calculate_h;

use guv::Number;

mod strategies;

//
// property-based tests
//

proptest! {
    //
    // calculate_h tests
    //

    #[cfg(feature = "fixed")]
    #[test]
    fn calculate_h_returns_number_of_seconds_elapsed_fixed64(
        (before, after) in strategies::ordered_system_times()
    ) {
        let h_calc = calculate_h::<I32F32>(after, before)
            .expect("calculate_h should succeed when measurement_time happens after last_update_time");
        let h: I32F32 = after.duration_since(before)
            .expect("before should come before after")
            .as_secs_f64()
            .to_fixed();
        let abs_diff = (h - h_calc).abs();

        assert!(
            abs_diff <= 1000 * I32F32::DELTA,
            "delta: {:?}, h_calc: {:?}, h: {:?}, abs_diff: {:?}",
            I32F32::DELTA,
            h_calc,
            h,
            abs_diff
        )
    }

    #[cfg(feature = "fixed")]
    #[test]
    fn calculate_h_returns_err_when_measurement_time_before_last_update_time_fixed64(
        (before, after) in strategies::ordered_system_times()
    ) {
        calculate_h::<I32F32>(before, after)
            .expect_err("calculate_h should fail when measurement_time happens before last_update_time");
    }

    //
    // PidController tests
    //

    #[cfg(feature = "fixed")]
    #[test]
    fn trivial_update_should_not_fail_fixed64(
        pid_controller in strategies::default_pid_controllers::<I32F32>()
    ) {
        let mut pid_controller = pid_controller
            .expect("constructor should not error");
        assert_eq!(
            pid_controller
                .update(
                    I32F32::ZERO,
                    I32F32::ZERO,
                    I32F32::checked_from_num(0.001)
                        .expect("0.001 must be representable as I32F32"),
                    I32F32::ZERO,
                    I32F32::ZERO
                ),
            I32F32::ZERO
        );

        assert_eq!(pid_controller.control_output(), 0.0);
    }
}

//
// visual tests
//

#[cfg(feature = "fixed")]
fn process<T: Number + Fixed>(u: T, y: T, t: T) -> T {
    let two = T::one() + T::one();

    T::from_num(-0.105) * y * t + T::from_num(0.105) * u * (t + two) + (two * T::pi() * t).cos()
}

#[cfg(feature = "fixed")]
#[test]
fn test_process() {
    let result = process::<I32F32>(I32F32::ZERO, I32F32::ZERO, I32F32::ZERO);
    let abs_diff = (I32F32::ONE - result).abs();
    assert!(
        abs_diff <= 2 * I32F32::DELTA,
        "delta: {:?}, result: {:?}, abs_diff: {:?}",
        I32F32::DELTA,
        result,
        abs_diff
    );
}
