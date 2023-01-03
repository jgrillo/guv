#![cfg(feature = "std")]
use std::{
    fmt::Debug,
    time::{Duration, SystemTime},
};

#[cfg(feature = "fixed")]
use fixed::{
    traits::{LosslessTryFrom, LossyInto},
    types::I32F32,
};

use plotters::prelude::*;
use proptest::prelude::*;
use proptest_arbitrary_interop::{arb, ArbInterop};

#[cfg(feature = "float")]
use guv::std::real::calculate_h;

#[cfg(feature = "fixed")]
use guv::std::fixed::calculate_h;

use guv::{Number, PidController, PidControllerError};

//
// proptest strategies
//

/// This strategy generates real numbers of type `T` on the interval
/// `(T::epsilon(), T::max_value()]`
fn positive_nonzero_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop,
{
    arb::<T>().prop_map(|n| {
        let eps = T::epsilon();
        let n_abs = n.abs();

        if n_abs <= eps {
            n_abs + eps
        } else {
            n_abs
        }
    })
}

/// This strategy generates real numbers of type `T` on the interval
/// `(T::epsilon(), 1 / T::epsilon()]`
fn epsilon_epsilon_inverse_bounded_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop,
{
    positive_nonzero_numbers::<T>().prop_map(|n| {
        let one = T::one();
        let two = one + one;
        let eps = T::epsilon();
        let max = one / eps;

        if n > max {
            let val = max * ((n.sin() + one) / two);

            if val > eps {
                val
            } else {
                eps
            }
        } else {
            n
        }
    })
}

/// This strategy generates real numbers of type `T` on the interval
/// `[T::zero(), 1 / T::epsilon()]`
fn zero_epsilon_inverse_bounded_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop,
{
    arb::<T>().prop_map(|n| {
        let one = T::one();
        let two = one + one;
        let max = one / T::epsilon();

        max * ((n.sin() + one) / two)
    })
}

fn negative_epsilon_inverse_epsilon_inverse_bounded_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop,
{
    arb::<T>().prop_map(|n| (T::one() / T::epsilon()) * n.cos())
}

/// This strategy generates `PidController<T>` structs whose configuration
/// parameters are themselves individually populated by the given argument
/// strategies.
fn pid_controllers<T>(
    proportional_gains: impl Strategy<Value = T>,
    integral_time_constants: impl Strategy<Value = T>,
    derivative_time_constants: impl Strategy<Value = T>,
    set_point_coefficients: impl Strategy<Value = T>,
    initial_controller_outputs: impl Strategy<Value = T>,
) -> impl Strategy<Value = Result<PidController<T>, PidControllerError>>
where
    T: Number + Debug,
{
    (
        proportional_gains,
        integral_time_constants,
        derivative_time_constants,
        set_point_coefficients,
        initial_controller_outputs,
    )
        .prop_map(
            |(
                proportional_gain,
                integral_time_constant,
                derivative_time_constant,
                set_point_coefficient,
                initial_controller_output,
            )| {
                PidController::new(
                    proportional_gain,
                    integral_time_constant,
                    derivative_time_constant,
                    set_point_coefficient,
                    initial_controller_output,
                )
            },
        )
}

/// This strategy generates `PidController<T>` structs whose configuration
/// parameters cover the entire permissible domain.
fn default_pid_controllers<T>(
) -> impl Strategy<Value = Result<PidController<T>, PidControllerError>>
where
    T: Number + ArbInterop,
{
    pid_controllers(
        zero_epsilon_inverse_bounded_numbers(),
        epsilon_epsilon_inverse_bounded_numbers(),
        epsilon_epsilon_inverse_bounded_numbers(),
        zero_epsilon_inverse_bounded_numbers(),
        negative_epsilon_inverse_epsilon_inverse_bounded_numbers(),
    )
}

/// This strategy generates ordered pairs of timestamps `(before, after)`
/// where `before` is guaranteed to be earlier than `after`.
#[cfg(feature = "std")]
fn ordered_system_times() -> impl Strategy<Value = (SystemTime, SystemTime)> {
    (any::<SystemTime>(), any::<i32>(), 0u32..1_000_000_000u32).prop_map(|(time, delta, nanos)| {
        (
            time,
            time + Duration::new(delta.unsigned_abs() as u64, nanos),
        )
    })
}

//
// property-based tests
//

proptest! {
    //
    // calculate_h tests
    //

    #[cfg(feature = "float")]
    #[test]
    fn calculate_h_returns_number_of_seconds_elapsed_f64(
        (before, after) in ordered_system_times()
    ) {
        assert_eq!(
            calculate_h::<f64>(after, before)
                .expect("calculate_h should succeed when measurement_time happens after last_update_time"),
            after.duration_since(before).expect("before should come before after").as_secs_f64()
        )
    }

    #[cfg(feature = "float")]
    #[test]
    fn calculate_h_returns_number_of_seconds_elapsed_f32(
        (before, after) in ordered_system_times()
    ) {
        assert_eq!(
            calculate_h::<f32>(after, before)
                .expect("calculate_h should succeed when measurement_time happens after last_update_time"),
            after.duration_since(before).expect("before should come before after").as_secs_f32()
        )
    }

    #[cfg(feature = "fixed")]
    #[test]
    fn calculate_h_returns_number_of_seconds_elapsed_fixed64(
        (before, after) in ordered_system_times()
    ) {
        let delta = LossyInto::<f64>::lossy_into(I32F32::DELTA);
        let h_calc = LossyInto::<f64>::lossy_into(
            calculate_h::<I32F32>(after, before)
                .expect("calculate_h should succeed when measurement_time happens after last_update_time")
        );
        let h = after.duration_since(before)
            .expect("before should come before after")
            .as_secs_f64();
        let abs_diff = (h - h_calc).abs();

        assert!(
            abs_diff <= 2.0 * delta,
            "delta: {:?}, h_calc: {:?}, h: {:?}, abs_diff: {:?}",
            delta, h_calc, h, abs_diff
        )
    }

    #[cfg(feature = "float")]
    #[test]
    fn calculate_h_returns_err_when_measurement_time_before_last_update_time_f64(
        (before, after) in ordered_system_times()
    ) {
        calculate_h::<f64>(before, after)
            .expect_err("calculate_h should fail when measurement_time happens before last_update_time");
    }

    #[cfg(feature = "float")]
    #[test]
    fn calculate_h_returns_err_when_measurement_time_before_last_update_time_f32(
        (before, after) in ordered_system_times()
    ) {
        calculate_h::<f32>(before, after)
            .expect_err("calculate_h should fail when measurement_time happens before last_update_time");
    }

    #[cfg(feature = "fixed")]
    #[test]
    fn calculate_h_returns_err_when_measurement_time_before_last_update_time_fixed64(
        (before, after) in ordered_system_times()
    ) {
        calculate_h::<I32F32>(before, after)
            .expect_err("calculate_h should fail when measurement_time happens before last_update_time");
    }

    //
    // PidController tests
    //

    #[cfg(feature = "float")]
    #[test]
    fn trivial_update_should_not_fail_f64(
        pid_controller in default_pid_controllers::<f64>()
    ) {
        let mut pid_controller = pid_controller
            .expect("constructor should not error");

        assert_eq!(
            pid_controller
                .update(0.0, 0.0, 0.001, 0.0, 0.0),
            0.0
        );

        assert_eq!(pid_controller.control_output(), 0.0);
    }

    #[cfg(feature = "float")]
    #[test]
    fn trivial_update_should_not_fail_f32(
        pid_controller in default_pid_controllers::<f32>()
    ) {
        let mut pid_controller = pid_controller
            .expect("constructor should not error");
        assert_eq!(
            pid_controller
                .update(0.0, 0.0, 0.001, 0.0, 0.0),
            0.0
        );

        assert_eq!(pid_controller.control_output(), 0.0);
    }

    #[cfg(feature = "fixed")]
    #[test]
    fn trivial_update_should_not_fail_fixed64(
        pid_controller in default_pid_controllers::<I32F32>()
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

    #[cfg(feature = "float")]
    #[test]
    fn in_place_constants_update_should_not_fail_f64(
        proportional_gain in zero_epsilon_inverse_bounded_numbers(),
        integral_time_constant in epsilon_epsilon_inverse_bounded_numbers(),
        derivative_time_constant in epsilon_epsilon_inverse_bounded_numbers(),
        set_point_coefficient in zero_epsilon_inverse_bounded_numbers(),
        pid_controller in default_pid_controllers::<f64>(),
    ) {
        let mut pid_controller = pid_controller
            .expect("constructor should not error");

        assert_eq!(
            pid_controller
                .update(0.0, 0.0, 0.001, 0.0, 0.0),
            0.0
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
                .update(0.0, 0.0, 0.001, 0.0, 0.0),
            0.0
        );

        assert_eq!(pid_controller.control_output(), 0.0);
    }

    #[cfg(feature = "float")]
    #[test]
    fn in_place_constants_update_should_not_fail_f32(
        proportional_gain in zero_epsilon_inverse_bounded_numbers(),
        integral_time_constant in epsilon_epsilon_inverse_bounded_numbers(),
        derivative_time_constant in epsilon_epsilon_inverse_bounded_numbers(),
        set_point_coefficient in zero_epsilon_inverse_bounded_numbers(),
        pid_controller in default_pid_controllers::<f32>(),
    ) {
        let mut pid_controller = pid_controller
            .expect("constructor should not error");

        assert_eq!(
            pid_controller
                .update(0.0, 0.0, 0.001, 0.0, 0.0),
            0.0
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
                .update(0.0, 0.0, 0.001, 0.0, 0.0),
            0.0
        );

        assert_eq!(pid_controller.control_output(), 0.0);
    }

    #[cfg(feature = "float")]
    #[test]
    fn copy_constants_update_should_not_fail_f64(
        proportional_gain in zero_epsilon_inverse_bounded_numbers(),
        integral_time_constant in epsilon_epsilon_inverse_bounded_numbers(),
        derivative_time_constant in epsilon_epsilon_inverse_bounded_numbers(),
        set_point_coefficient in zero_epsilon_inverse_bounded_numbers(),
        pid_controller in default_pid_controllers::<f64>(),
    ) {
        let mut pid_controller = pid_controller
            .expect("constructor should not error");

        assert_eq!(
            pid_controller
                .update(0.0, 0.0, 0.001, 0.0, 0.0),
            0.0
        );

        let mut another_pid_controller = pid_controller
            .update_constants(
                proportional_gain,
                integral_time_constant,
                derivative_time_constant,
                set_point_coefficient,
            )
            .expect("updating constants should not fail");

        assert_eq!(
            another_pid_controller
                .update(1000.0, 1000.0, std::f64::EPSILON, -1.0, 1.0)
                .abs(),
            1.0
        );

        assert_eq!(pid_controller.control_output(), 0.0);
        assert_eq!(another_pid_controller.control_output().abs(), 1.0);
    }

    #[cfg(feature = "float")]
    #[test]
    fn copy_constants_update_should_not_fail_f32(
        proportional_gain in zero_epsilon_inverse_bounded_numbers(),
        integral_time_constant in epsilon_epsilon_inverse_bounded_numbers(),
        derivative_time_constant in epsilon_epsilon_inverse_bounded_numbers(),
        set_point_coefficient in zero_epsilon_inverse_bounded_numbers(),
        pid_controller in default_pid_controllers::<f32>(),
    ) {
        let mut pid_controller = pid_controller
            .expect("constructor should not error");

        assert_eq!(
            pid_controller
                .update(0.0, 0.0, 0.001, 0.0, 0.0),
            0.0
        );

        let mut another_pid_controller = pid_controller
            .update_constants(
                proportional_gain,
                integral_time_constant,
                derivative_time_constant,
                set_point_coefficient,
            )
            .expect("updating constants should not fail");

        assert_eq!(
            another_pid_controller
                .update(1000.0, 1000.0, std::f32::EPSILON, -1.0, 1.0)
                .abs(),
            1.0
        );

        assert_eq!(pid_controller.control_output(), 0.0);
        assert_eq!(another_pid_controller.control_output().abs(), 1.0);
    }
}

//
// visual tests
//

#[cfg(feature = "float")]
fn process<T: Number + num_traits::cast::FromPrimitive>(u: T, y: T, t: T) -> T {
    let two = T::one() + T::one();

    T::from_f32(-0.105).unwrap() * y * t
        + T::from_f32(0.105).unwrap() * u * (t + two)
        + (two * T::pi() * t).cos()
}

#[cfg(feature = "fixed")]
fn process<T: Number + LosslessTryFrom<f32>>(u: T, y: T, t: T) -> T {
    let two = T::one() + T::one();

    T::lossless_try_from(-0.105).unwrap() * y * t
        + T::lossless_try_from(0.105).unwrap() * u * (t + two)
        + (two * T::pi() * t).cos()
}

#[cfg(feature = "float")]
#[test]
fn test_plot_f64() {
    let root = SVGBackend::new("test-output/1_f64.svg", (640, 480)).into_drawing_area();
    root.fill(&WHITE).expect("background fill should not error");
    let root = root.margin(10, 10, 10, 10);

    let mut chart = ChartBuilder::on(&root)
        .caption(
            "PidController impulse response",
            ("sans-serif", 40).into_font(),
        )
        .x_label_area_size(20)
        .y_label_area_size(40)
        .build_cartesian_2d(0f64..10f64, -10f64..10f64)
        .expect("building the chart should not error");

    chart
        .configure_mesh()
        .x_labels(5)
        .y_labels(5)
        .y_label_formatter(&|y| format!("{:.3}", y))
        .draw()
        .expect("configuring mesh should not fail");

    let mut pid_controller = PidController::<f64>::new(0.05, 0.01, 0.0001, 0.01, 1.0)
        .expect("constructing pid_controller should not fail");

    let h = 0.01; // 10ms

    let mut set_points: Vec<(f64, f64)> = Vec::with_capacity(1000);
    let mut control_outputs: Vec<(f64, f64)> = Vec::with_capacity(1000);
    let mut process_measurements: Vec<(f64, f64)> = Vec::with_capacity(1000);
    let mut previous_process_measurement = 0.0;
    let mut previous_control_output = 0.0;
    let mut t = 0.0;

    for n in 0u64..1000u64 {
        let set_point = if n < 100 {
            0.0
        } else if n > 600 {
            -3.0
        } else {
            3.0
        };

        set_points.push((t, set_point));

        let process_measurement = process(previous_control_output, previous_process_measurement, t);
        process_measurements.push((t, process_measurement));
        previous_process_measurement = process_measurement;

        let control_output = pid_controller.update(set_point, process_measurement, h, -7.0, 7.0);
        control_outputs.push((t, control_output));
        previous_control_output = control_output;

        t += h;
    }

    chart
        .draw_series(LineSeries::new(set_points, BLUE))
        .expect("set points should be plottable")
        .label("set point")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));
    chart
        .draw_series(LineSeries::new(control_outputs, GREEN))
        .expect("control outputs should be plottable")
        .label("control output")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &GREEN));
    chart
        .draw_series(LineSeries::new(process_measurements, RED))
        .expect("process measurements should be plottable")
        .label("process measurement")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()
        .expect("drawing the legend should not fail");

    root.present().expect("presentation should not fail")
}

#[cfg(feature = "float")]
#[test]
fn test_plot_f32() {
    let root = SVGBackend::new("test-output/1_f32.svg", (640, 480)).into_drawing_area();
    root.fill(&WHITE).expect("background fill should not error");
    let root = root.margin(10, 10, 10, 10);

    let mut chart = ChartBuilder::on(&root)
        .caption(
            "PidController impulse response",
            ("sans-serif", 40).into_font(),
        )
        .x_label_area_size(20)
        .y_label_area_size(40)
        .build_cartesian_2d(0f32..10f32, -10f32..10f32)
        .expect("building the chart should not error");

    chart
        .configure_mesh()
        .x_labels(5)
        .y_labels(5)
        .y_label_formatter(&|y| format!("{:.3}", y))
        .draw()
        .expect("configuring mesh should not fail");

    let mut pid_controller = PidController::<f32>::new(0.05, 0.01, 0.0001, 0.01, 1.0)
        .expect("constructing pid_controller should not fail");

    let h = 0.01; // 10ms

    let mut set_points: Vec<(f32, f32)> = Vec::with_capacity(1000);
    let mut control_outputs: Vec<(f32, f32)> = Vec::with_capacity(1000);
    let mut process_measurements: Vec<(f32, f32)> = Vec::with_capacity(1000);
    let mut previous_process_measurement = 0.0f32;
    let mut previous_control_output = 0.0f32;
    let mut t = 0.0f32;

    for n in 0u32..1000u32 {
        let set_point = if n < 100 {
            0.0f32
        } else if n > 600 {
            -3.0f32
        } else {
            3.0f32
        };

        set_points.push((t, set_point));

        let process_measurement = process(previous_control_output, previous_process_measurement, t);
        process_measurements.push((t, process_measurement));
        previous_process_measurement = process_measurement;

        let control_output = pid_controller.update(set_point, process_measurement, h, -7.0, 7.0);
        control_outputs.push((t, control_output));
        previous_control_output = control_output;

        t += h;
    }

    chart
        .draw_series(LineSeries::new(set_points, BLUE))
        .expect("set points should be plottable")
        .label("set point")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));
    chart
        .draw_series(LineSeries::new(control_outputs, GREEN))
        .expect("control outputs should be plottable")
        .label("control output")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &GREEN));
    chart
        .draw_series(LineSeries::new(process_measurements, RED))
        .expect("process measurements should be plottable")
        .label("process measurement")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()
        .expect("drawing the legend should not fail");

    root.present().expect("presentation should not fail")
}
