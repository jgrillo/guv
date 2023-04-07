#![cfg(feature = "std")]

use std::{
    fmt::Debug,
    time::{Duration, SystemTime},
};

#[cfg(all(feature = "float", not(feature = "fixed")))]
use num_traits::float::Float;

#[cfg(all(feature = "fixed", not(feature = "float")))]
use fixed::{traits::Fixed, types::I32F32};

use proptest::prelude::*;
use proptest_arbitrary_interop::{arb, ArbInterop};

use guv::{Number, PidController, PidControllerError};

//
// proptest strategies
//

/// Constrain the input `n` to the interval `[-T::max_value(), T::max_value()]`
/// and map `NaN` values to zero. This makes sure `NaN` and `Inf` are never
/// returned.
#[cfg(all(feature = "float", not(feature = "fixed")))]
fn make_behave<T>(n: T) -> T
where
    T: Number + ArbInterop + Float + Debug,
{
    let max = <T as Number>::max_value();
    if Float::is_nan(n) {
        <T as Number>::zero()
    } else if Float::is_infinite(n) {
        if <T as Float>::is_sign_positive(n) {
            max
        } else {
            -max
        }
    } else {
        n.clamp(-max, max)
    }
}

/// This strategy generates arbitrary floating point numbers on the interval
/// `[T::min_value(), T::max_value()]` and ensures no `NaN` or `Inf` values are
/// present.
#[cfg(all(feature = "float", not(feature = "fixed")))]
pub fn well_behaved_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Float + Debug,
{
    arb::<T>().prop_map(make_behave)
}

/// This strategy generates arbitrary fixed point numbers.
#[cfg(all(feature = "fixed", not(feature = "float")))]
pub fn well_behaved_numbers<T>() -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Fixed + Debug,
{
    let max = <T as Number>::max_value();
    arb::<T>().prop_map(move |n| n.clamp(-max, max))
}

/// Constrain the input `n` to the interval `(low, high]`
#[cfg(all(feature = "float", not(feature = "fixed")))]
fn bound<T>(low: T, high: T, n: T) -> T
where
    T: Number + ArbInterop + Float + Debug,
{
    let two = <T as Number>::one() + <T as Number>::one();
    let max = <T as Number>::max_value();
    // low + ((high - low) / (2.0 * max)) * (n + max)
    low.safe_add(
        high.safe_sub(low)
            .safe_div(two.safe_mul(max))
            .safe_mul(n.safe_add(max)),
    )
}

/// Constrain the input `n` to the interval `(low, high]`
#[cfg(all(feature = "fixed", not(feature = "float")))]
fn bound<T>(low: T, high: T, n: T) -> T
where
    T: Number + ArbInterop + Fixed + Debug,
{
    let two = <T as Number>::one() + <T as Number>::one();
    let max = <T as Number>::max_value();
    // low + ((high - low) / (2.0 * max)) * (n + max)
    low.safe_add(
        high.safe_sub(low)
            .safe_div(two.safe_mul(max))
            .safe_mul(n.safe_add(max)),
    )
}

/// This strategy generates real numbers of type `T` on the interval
/// `(low, high]`
#[cfg(all(feature = "float", not(feature = "fixed")))]
pub fn bounded_numbers<T>(low: T, high: T) -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Float + Debug,
{
    well_behaved_numbers().prop_map(move |n: T| bound(low, high, n))
}

/// This strategy generates real numbers of type `T` on the interval
/// `(low, high]`
#[cfg(all(feature = "fixed", not(feature = "float")))]
pub fn bounded_numbers<T>(low: T, high: T) -> impl Strategy<Value = T>
where
    T: Number + ArbInterop + Fixed + Debug,
{
    well_behaved_numbers().prop_map(move |n: T| bound(low, high, n))
}

/// This strategy generates tuples of numbers `(low, high)` where high > low and
/// low + high < T::max_value().
#[cfg(all(feature = "float", not(feature = "fixed")))]
fn ordered_pairs<T>() -> impl Strategy<Value = (T, T)>
where
    T: Number + ArbInterop + Float + Debug,
{
    (well_behaved_numbers::<T>(), well_behaved_numbers::<T>()).prop_map(|(m, n)| {
        let max = <T as Number>::max_value();
        let one = <T as Number>::one();
        let low = bound(-max, max, m);
        let high = bound(-max, max, n);
        if high > low {
            (low, high)
        } else if high == low {
            (low.safe_sub(one), high.safe_add(one))
        } else {
            (high, low)
        }
    })
}

/// This strategy generates tuples of numbers `(low, high)` where high > low and
/// low + high < T::MAX.
#[cfg(all(feature = "fixed", not(feature = "float")))]
fn ordered_pairs<T>() -> impl Strategy<Value = (T, T)>
where
    T: Number + ArbInterop + Fixed + Debug,
{
    (well_behaved_numbers::<T>(), well_behaved_numbers::<T>()).prop_map(|(m, n)| {
        let max = <T as Number>::max_value();
        let one = <T as Number>::one();
        let low = bound(-max, max, m);
        let high = bound(-max, max, n);
        if high > low {
            (low, high)
        } else if high == low {
            (low.safe_sub(one), high.safe_add(one))
        } else {
            (high, low)
        }
    })
}

#[cfg(all(feature = "float", not(feature = "fixed")))]
fn arbitrary_bounded_numbers<T>() -> impl Strategy<Value = (T, T, T)>
where
    T: Number + ArbInterop + Float + Debug,
{
    ordered_pairs().prop_flat_map(|(low, high)| (Just(low), Just(high), bounded_numbers(low, high)))
}

#[cfg(all(feature = "fixed", not(feature = "float")))]
fn arbitrary_bounded_numbers<T>() -> impl Strategy<Value = (T, T, T)>
where
    T: Number + ArbInterop + Fixed + Debug,
{
    ordered_pairs().prop_flat_map(|(low, high)| (Just(low), Just(high), bounded_numbers(low, high)))
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
) -> impl Strategy<Value = Result<PidController<T>, PidControllerError<T>>>
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
) -> impl Strategy<Value = Result<PidController<T>, PidControllerError<T>>>
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
) -> impl Strategy<Value = Result<PidController<T>, PidControllerError<T>>>
where
    T: Number + ArbInterop + Float + Debug,
{
    let zero = <T as Number>::zero();
    let eps = <T as Number>::epsilon();
    let max = <T as Number>::max_value();
    pid_controllers(
        bounded_numbers(zero, max),
        bounded_numbers(eps, max),
        bounded_numbers(eps, max),
        bounded_numbers(zero, max),
        bounded_numbers(-max, max),
    )
}

/// This strategy generates `PidController<T>` structs whose configuration
/// parameters cover the entire permissible domain.
#[cfg(all(feature = "fixed", not(feature = "float")))]
pub fn default_pid_controllers<T>(
) -> impl Strategy<Value = Result<PidController<T>, PidControllerError<T>>>
where
    T: Number + ArbInterop + Fixed + Debug,
{
    let zero = <T as Number>::zero();
    let eps = <T as Number>::epsilon();
    let max = <T as Number>::max_value();
    pid_controllers(
        bounded_numbers(zero, max),
        bounded_numbers(eps, max),
        bounded_numbers(eps, max),
        bounded_numbers(zero, max),
        bounded_numbers(-max, max),
    )
}

/// This strategy generates ordered pairs of timestamps `(before, after)`
/// where `before` is guaranteed to be earlier than `after`.
#[cfg(all(feature = "float", not(feature = "fixed")))]
pub fn ordered_system_times() -> impl Strategy<Value = (SystemTime, SystemTime)> {
    (any::<SystemTime>(), any::<i32>(), 1u32..1_000_000_000u32).prop_map(|(time, delta, nanos)| {
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
    (any::<SystemTime>(), any::<i32>(), 1u32..1_000_000_000u32).prop_map(|(time, delta, nanos)| {
        (
            time,
            time + Duration::new(delta.unsigned_abs() as u64, nanos),
        )
    })
}

//
// strategies tests
//
mod test {
    use super::*;

    proptest! {

        #[cfg(all(feature = "float", not(feature = "fixed")))]
        #[test]
        fn bound_bounds_numbers(
            (low, high) in ordered_pairs::<f64>(),
            n in well_behaved_numbers::<f64>(),
        ) {
            let bounded = bound(low, high, n);

            assert!(
                bounded >= low,
                "low: {:?} high: {:?} n: {:?} bounded: {:?} bounded must be >= low",
                low, high, n, bounded,
            );
            assert!(
                bounded <= high,
                "low: {:?} high: {:?} n: {:?} bounded: {:?} bounded must be <= high",
                low, high, n, bounded,
            );
        }

        #[cfg(all(feature = "fixed", not(feature = "float")))]
        #[test]
        fn bound_bounds_numbers(
            (low, high) in ordered_pairs::<I32F32>(),
            n in well_behaved_numbers::<I32F32>(),
        ) {
            let bounded = bound(low, high, n);

            assert!(
                bounded >= low,
                "low: {:?} high: {:?} n: {:?} bounded: {:?} bounded must be >= low",
                low, high, n, bounded,
            );
            assert!(
                bounded <= high,
                "low: {:?} high: {:?} n: {:?} bounded: {:?} bounded must be <= high",
                low, high, n, bounded,
            );
        }

        #[cfg(all(feature = "float", not(feature = "fixed")))]
        #[test]
        fn well_behaved_numbers_behave_well(
            n in well_behaved_numbers::<f64>(),
        ) {
            assert!(<f64 as Float>::is_finite(n));
            assert!(!<f64 as Float>::is_nan(n));
            assert!(!<f64 as Float>::is_infinite(n));
        }

        #[cfg(all(feature = "float", not(feature = "fixed")))]
        #[test]
        fn ordered_pairs_generates_ordered_numbers(
            (low, high) in ordered_pairs::<f64>(),
        ) {
            assert!(
                low < high,
                "low: {:?} high: {:?} low must be < high",
                low,
                high
            )
        }

        #[cfg(all(feature = "fixed", not(feature = "float")))]
        #[test]
        fn ordered_pairs_generates_ordered_numbers(
            (low, high) in ordered_pairs::<I32F32>(),
        ) {
            assert!(
                low < high,
                "low: {:?} high: {:?} low must be < high",
                low,
                high
            )
        }

        #[cfg(all(feature = "float", not(feature = "fixed")))]
        #[test]
        fn bounded_numbers_generates_numbers_on_interval(
            (low, high, n) in arbitrary_bounded_numbers::<f64>(),
        ) {
            assert!(
                n >= low,
                "low: {:?} high: {:?} n: {:?} n must be >= low",
                low,
                high,
                n
            );
            assert!(
                n <= high,
                "low: {:?} high: {:?} n: {:?} n must be <= high",
                low,
                high,
                n
            );
        }

        #[cfg(all(feature = "fixed", not(feature = "float")))]
        #[test]
        fn bounded_numbers_generates_numbers_on_interval(
            (low, high, n) in arbitrary_bounded_numbers::<I32F32>(),
        ) {
            assert!(
                n >= low,
                "low: {:?} high: {:?} n: {:?} n must be >= low",
                low,
                high,
                n
            );
            assert!(
                n <= high,
                "low: {:?} high: {:?} n: {:?} n must be <= high",
                low,
                high,
                n
            );
        }
    }
}
