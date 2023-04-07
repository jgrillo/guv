#![cfg(all(feature = "std", feature = "fixed"))]

use std::assert_eq;

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
        let result = pid_controller.update(
            I32F32::ZERO,
            I32F32::ZERO,
            I32F32::checked_from_num(0.1)
                .expect("0.1 must be representable as I32F32"),
            I32F32::ZERO,
            I32F32::ZERO
        );

        assert_eq!(result, I32F32::ZERO);
        assert_eq!(pid_controller.control_output(), 0.0);
    }

    #[cfg(feature = "fixed")]
    #[test]
    fn in_place_constants_update_should_not_fail_fixed64(
        proportional_gain in strategies::bounded_numbers(
            <I32F32 as Number>::zero(), <I32F32 as Number>::max_value()
        ),
        integral_time_constant in strategies::bounded_numbers(
            <I32F32 as Number>::epsilon(), <I32F32 as Number>::max_value()
        ),
        derivative_time_constant in strategies::bounded_numbers(
            <I32F32 as Number>::epsilon(), <I32F32 as Number>::max_value()
        ),
        set_point_coefficient in strategies::bounded_numbers(
            <I32F32 as Number>::zero(), <I32F32 as Number>::max_value()
        ),
        pid_controller in strategies::default_pid_controllers::<I32F32>(),
    ) {
        let mut pid_controller = pid_controller
            .expect("constructor should not error");

        let zero = <I32F32 as Number>::zero();

        assert_eq!(
            pid_controller
                .update(zero, zero, I32F32::from_num(0.001), zero, zero),
            zero
        );

        pid_controller
            .update_constants_mut(
                proportional_gain,
                integral_time_constant,
                derivative_time_constant,
                set_point_coefficient,
            )
            .expect("updating constants should not fail");

        assert_eq!(
            pid_controller
                .update(zero, zero, I32F32::from_num(0.001), zero, zero),
            zero
        );

        assert_eq!(pid_controller.control_output(), zero);
    }

    #[cfg(feature = "fixed")]
    #[test]
    fn copy_constants_update_should_not_fail_fixed64(
        proportional_gain in strategies::bounded_numbers(
            <I32F32 as Number>::zero(), <I32F32 as Number>::max_value()
        ),
        integral_time_constant in strategies::bounded_numbers(
            <I32F32 as Number>::epsilon(), <I32F32 as Number>::max_value()
        ),
        derivative_time_constant in strategies::bounded_numbers(
            <I32F32 as Number>::epsilon(), <I32F32 as Number>::max_value()
        ),
        set_point_coefficient in strategies::bounded_numbers(
            <I32F32 as Number>::zero(), <I32F32 as Number>::max_value()
        ),
        pid_controller in strategies::default_pid_controllers::<I32F32>(),
    ) {
        let mut pid_controller = pid_controller
            .expect("constructor should not error");

        let zero = <I32F32 as Number>::zero();

        assert_eq!(
            pid_controller
                .update(zero, zero, I32F32::from_num(0.001), zero, zero),
            zero
        );

        let mut another_pid_controller = pid_controller
            .update_constants(
                proportional_gain,
                integral_time_constant,
                derivative_time_constant,
                set_point_coefficient,
            )
            .expect("updating constants should not fail");

        let one = <I32F32 as Number>::one();
        let eps = <I32F32 as Number>::epsilon();
        let one_k = I32F32::from_num(1000);

        another_pid_controller
            .update(one_k, one_k, eps, -one, one);

        assert_eq!(pid_controller.control_output(), zero);
    }
}

//
// visual tests
//

#[cfg(feature = "fixed")]
fn process<T: Number + Fixed>(u: T, y: T, t: T) -> T {
    let two = T::one().safe_add(T::one());
    // -0.105 * y * t + 0.105 * u * (t + 2.0) + (2.0 * T::pi() * t).cos()
    <T as Fixed>::from_num(-0.105)
        .safe_mul(y)
        .safe_mul(t)
        .safe_add(
            <T as Fixed>::from_num(0.105)
                .safe_mul(u)
                .safe_mul(t.safe_add(two)),
        )
        .safe_add(two.safe_mul(T::pi()).safe_mul(t).cos())
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
