#![cfg(feature = "std")]

use std::{
    fmt::Debug,
    time::{Duration, SystemTime},
};

#[cfg(all(feature = "float", not(feature = "fixed")))]
use num_traits::float::Float;

#[cfg(all(feature = "fixed", not(feature = "float")))]
use fixed::traits::Fixed;

use proptest::prelude::*;
use proptest_arbitrary_interop::{arb, ArbInterop};

use guv::{Number, PidController, PidControllerError};

//
// proptest strategies
//

#[cfg(all(feature = "float", not(feature = "fixed")))]
fn make_behave<T>(n: T) -> T
where
    T: Number + ArbInterop + Float + Debug,
{
    if Float::is_nan(n) {
        <T as Number>::zero()
    } else if Float::is_infinite(n) {
        if <T as Float>::is_sign_positive(n) {
            <T as Float>::max_value()
        } else {
            <T as Float>::min_value()
        }
    } else {
        n
    }
}

#[cfg(all(feature = "float", not(feature = "fixed")))]
pub fn well_behaved_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Float + Debug,
{
    arb::<T>().prop_map(make_behave)
}

#[cfg(all(feature = "fixed", not(feature = "float")))]
pub fn well_behaved_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Fixed + Debug,
{
    arb::<T>()
}

/// This strategy generates real numbers of type `T` on the interval
/// `(T::epsilon(), T::max_value()]`
#[cfg(all(feature = "float", not(feature = "fixed")))]
pub fn positive_nonzero_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Float + Debug,
{
    well_behaved_numbers()
        .prop_map(|n: T| {
            let eps = <T as Number>::epsilon();
            let n_abs = n.abs();

            if n_abs <= eps {
                n_abs + eps
            } else {
                n_abs
            }
        })
        .prop_map(make_behave)
}

/// This strategy generates real numbers of type `T` on the interval
/// `(T::epsilon(), T::max_value()]`
#[cfg(all(feature = "fixed", not(feature = "float")))]
pub fn positive_nonzero_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Fixed + Debug,
{
    well_behaved_numbers().prop_map(|n: T| {
        let eps = <T as Number>::epsilon();
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
#[cfg(all(feature = "float", not(feature = "fixed")))]
pub fn epsilon_epsilon_inverse_bounded_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Float + Debug,
{
    positive_nonzero_numbers::<T>()
        .prop_map(|n: T| {
            let one = <T as Number>::one();
            let two = one + one;
            let eps = <T as Number>::epsilon();
            let max = one / (two * two * eps); // FIXME: more rigor

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
        .prop_map(make_behave)
}

/// This strategy generates real numbers of type `T` on the interval
/// `(T::epsilon(), 1 / T::epsilon()]`
#[cfg(all(feature = "fixed", not(feature = "float")))]
pub fn epsilon_epsilon_inverse_bounded_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Fixed + Debug,
{
    positive_nonzero_numbers::<T>().prop_map(|n: T| {
        let one = <T as Number>::one();
        let two = one + one;
        let eps = <T as Number>::epsilon();
        let max = (two * two * T::epsilon()).recip(); // FIXME: more rigor

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
#[cfg(all(feature = "float", not(feature = "fixed")))]
pub fn zero_epsilon_inverse_bounded_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Float + Debug,
{
    well_behaved_numbers()
        .prop_map(|n: T| {
            let one = <T as Number>::one();
            let two = one + one;
            let max = one / (two * two * <T as Number>::epsilon()); // FIXME: more rigor

            max * ((n.sin() + one) / two)
        })
        .prop_map(make_behave)
}

/// This strategy generates real numbers of type `T` on the interval
/// `[T::zero(), 1 / T::epsilon()]`
#[cfg(all(feature = "fixed", not(feature = "float")))]
pub fn zero_epsilon_inverse_bounded_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Fixed + Debug,
{
    well_behaved_numbers().prop_map(|n: T| {
        let one = <T as Number>::one();
        let two = one + one;
        let max = (two * two * T::epsilon()).recip(); // FIXME: more rigor

        max * ((n.sin() + one) / two)
    })
}

#[cfg(all(feature = "float", not(feature = "fixed")))]
pub fn negative_epsilon_inverse_epsilon_inverse_bounded_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Float + Debug,
{
    arb::<T>()
        .prop_map(|n| {
            let one = <T as Number>::one();
            let two = one + one;
            let max = one / (two * two * <T as Number>::epsilon()); // FIXME: more rigor
            max * n.cos()
        })
        .prop_map(make_behave)
}

#[cfg(all(feature = "fixed", not(feature = "float")))]
pub fn negative_epsilon_inverse_epsilon_inverse_bounded_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Fixed + Debug,
{
    arb::<T>().prop_map(|n| {
        let one = <T as Number>::one();
        let two = one + one;
        let max = (two * two * T::epsilon()).recip(); // FIXME: more rigor
        max * n.cos()
    })
}

/// This strategy generates `PidController<T>` structs whose configuration
/// parameters are themselves individually populated by the given argument
/// strategies.
#[cfg(all(feature = "float", not(feature = "fixed")))]
pub fn pid_controllers<T>(
    proportional_gains: impl Strategy<Value = T>,
    integral_time_constants: impl Strategy<Value = T>,
    derivative_time_constants: impl Strategy<Value = T>,
    set_point_coefficients: impl Strategy<Value = T>,
    initial_controller_outputs: impl Strategy<Value = T>,
) -> impl Strategy<Value = Result<PidController<T>, PidControllerError>>
where
    T: Number + ArbInterop + Float + Debug,
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
/// parameters are themselves individually populated by the given argument
/// strategies.
#[cfg(all(feature = "fixed", not(feature = "float")))]
pub fn pid_controllers<T>(
    proportional_gains: impl Strategy<Value = T>,
    integral_time_constants: impl Strategy<Value = T>,
    derivative_time_constants: impl Strategy<Value = T>,
    set_point_coefficients: impl Strategy<Value = T>,
    initial_controller_outputs: impl Strategy<Value = T>,
) -> impl Strategy<Value = Result<PidController<T>, PidControllerError>>
where
    T: Number + ArbInterop + Fixed + Debug,
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
#[cfg(all(feature = "float", not(feature = "fixed")))]
pub fn default_pid_controllers<T>(
) -> impl Strategy<Value = Result<PidController<T>, PidControllerError>>
where
    T: Number + ArbInterop + Float + Debug,
{
    pid_controllers(
        zero_epsilon_inverse_bounded_numbers(),
        epsilon_epsilon_inverse_bounded_numbers(),
        epsilon_epsilon_inverse_bounded_numbers(),
        zero_epsilon_inverse_bounded_numbers(),
        negative_epsilon_inverse_epsilon_inverse_bounded_numbers(),
    )
}

/// This strategy generates `PidController<T>` structs whose configuration
/// parameters cover the entire permissible domain.
#[cfg(all(feature = "fixed", not(feature = "float")))]
pub fn default_pid_controllers<T>(
) -> impl Strategy<Value = Result<PidController<T>, PidControllerError>>
where
    T: Number + ArbInterop + Fixed + Debug,
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
#[cfg(all(feature = "float", not(feature = "fixed")))]
pub fn ordered_system_times() -> impl Strategy<Value = (SystemTime, SystemTime)> {
    (any::<SystemTime>(), any::<i32>(), 0u32..1_000_000_000u32).prop_map(|(time, delta, nanos)| {
        (
            time,
            time + Duration::new(delta.unsigned_abs() as u64, nanos),
        )
    })
}

/// This strategy generates ordered pairs of timestamps `(before, after)`
/// where `before` is guaranteed to be earlier than `after`.
#[cfg(all(feature = "fixed", not(feature = "float")))]
pub fn ordered_system_times() -> impl Strategy<Value = (SystemTime, SystemTime)> {
    (any::<SystemTime>(), any::<i32>(), 0u32..1_000_000_000u32).prop_map(|(time, delta, nanos)| {
        (
            time,
            time + Duration::new(delta.unsigned_abs() as u64, nanos),
        )
    })
}
