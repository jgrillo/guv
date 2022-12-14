//! This crate is experimental and should not be used.

use std::{
    fmt::Debug,
    time::{SystemTime, SystemTimeError},
};

use thiserror::Error;

#[derive(Clone, Debug, Error)]
#[non_exhaustive]
pub enum PidControllerError {
    #[error("error computing elapsed time {0}")]
    SystemTime(#[from] SystemTimeError),

    #[error("numeric error {0}")]
    Numeric(&'static str),
}

/// This implementation of a PID controller comes from
///
/// ```text
/// Åström, K. J., & Hägglund, T. (1988).
/// Automatic Tuning of PID Controllers.
/// Instrument Society of America (ISA).
/// ISBN 1-55617-081-5
/// ```
///
/// While I've used English natural language names for the public API, I've kept
/// the mathematical symbols (at least within reason) faithful to their typeset
/// representation. This code is meant to be read alongside a copy of the book!
/// I found that keeping these variable names closely matched to the mathematics
/// aids understanding by reducing the amount of mental indirection, making
/// implementation errors easier to spot.
///
/// If you have any questions about this code feel free to reach out:
///
/// ```text
/// Jesse C. Grillo
/// jgrillo@protonmail.com
/// ```
/// \
/// # A note on numerical precision:
///
/// This controller is parametrized by a type `T` which can take either `f32` or
/// `f64` values. If you're using the 32 bit version of this controller, make
/// sure your update measurements are taken at least 1 microsecond apart to
/// ensure the calculations remain accurate. If you're using the 64 bit version,
/// make sure your measurements are taken at least 1 nanosecond apart.
#[allow(non_snake_case)]
#[derive(Clone, Debug)]
pub struct PidController<T>
where
    T: num_traits::real::Real + Debug,
{
    K: T,
    T_i: T,
    T_d: T,
    T_t: T,
    N: T,
    b: T,
    P_k1: T,
    I_k1: T,
    D_k1: T,
    r_k1: T,
    y_k1: T,
    y_k2: T,
    t_k1: SystemTime,
}

#[allow(non_snake_case)]
impl<T> PidController<T>
where
    T: num_traits::real::Real + Debug,
{
    /// Construct a new PidController.
    /// \
    /// # Arguments:
    /// \
    /// - `proportional_gain` -- The output the controller is multiplied by this
    ///   factor.
    /// \
    /// - `integral_time_constant` -- The time required for the integral term to
    ///   "catch up to" the proportional term in the face of an instantaneous
    ///   jump in controller error. Must greater than zero.
    /// \
    /// - `derivative_time_constant` -- The time required for the proportional
    ///   term to "catch up to" the derivative term if the error starts at zero
    ///   and increases at a fixed rate. Must be greater than zero.
    /// \
    /// - `tracking_time_constant` -- The time required for the integral term to
    ///   "reset". This is necessary to prevent integral wind-up. Must be
    ///   greater than zero.
    /// \
    /// - `derivative_gain_limit` -- This term mitigates the effects of high
    ///   frequency noise in the derivative term by limiting the gain, which in
    ///   turn limits the high frequency noise amplification factor. Must be
    ///   greater than zero.
    /// \
    /// - `set_point_coefficient` -- This term determines how the controller
    ///   reacts to a change in the setpoint.
    pub fn new(
        proportional_gain: T,
        integral_time_constant: T,
        derivative_time_constant: T,
        tracking_time_constant: T,
        derivative_gain_limit: T,
        set_point_coefficient: T,
    ) -> Result<Self, PidControllerError> {
        if integral_time_constant <= T::zero() {
            return Err(PidControllerError::Numeric(
                "integral_time_constant must be greater than zero",
            ));
        }

        if derivative_time_constant <= T::zero() {
            return Err(PidControllerError::Numeric(
                "derivative_time_constant must be greater than zero",
            ));
        }

        if tracking_time_constant <= T::zero() {
            return Err(PidControllerError::Numeric(
                "tracking_time_constant must be greater than zero",
            ));
        }

        if derivative_gain_limit <= T::zero() {
            return Err(PidControllerError::Numeric(
                "derivative_gain_limit must be greater than zero",
            ));
        }

        Ok(Self {
            K: proportional_gain,
            T_i: integral_time_constant,
            T_d: derivative_time_constant,
            T_t: tracking_time_constant,
            N: derivative_gain_limit,
            b: set_point_coefficient,
            P_k1: T::zero(),
            I_k1: T::zero(),
            D_k1: T::zero(),
            r_k1: T::zero(),
            y_k1: T::zero(),
            y_k2: T::zero(),
            t_k1: SystemTime::now(),
        })
    }

    /// Updates this controller's state according to the given
    /// parameters. Returns the updated control output, or a
    /// `PidControllerError` if the `measurement_time` is in the future relative
    /// to the system clock.
    /// \
    /// # Arguments:
    /// \
    /// - `set_point` -- the desired value for the process variable
    /// \
    /// - `process_measurement` -- the measured process variable value
    /// \
    /// - `measurement_time` -- the time when the `process_measurement` was made
    /// \
    /// - `lower_saturation_limit` -- the minimum value the controller can take
    /// \
    /// - `upper_saturation_limit` -- the maximum value the controller can take
    pub fn update(
        &mut self,
        set_point: T,
        process_measurement: T,
        measurement_time: SystemTime,
        lower_saturation_limit: T,
        upper_saturation_limit: T,
    ) -> Result<T, PidControllerError> {
        let h = calculate_h(measurement_time, self.last_update_time())?;

        self.update_state(
            h,
            set_point,
            process_measurement,
            lower_saturation_limit,
            upper_saturation_limit,
        )
    }

    /// Returns the most recently computed control output. If `.update(..)` has
    /// not been called yet this will return zero.
    pub fn control_output(&self) -> T {
        self.P_k1 + self.I_k1 + self.D_k1
    }

    /// Returns the last time this controller was updated
    pub fn last_update_time(&self) -> SystemTime {
        self.t_k1
    }

    fn calculate_P(&self, r_k: T, y_k: T) -> T {
        self.P_k1 + self.K * (self.b * r_k - y_k - self.b * self.r_k1 + self.y_k1)
    }

    fn calculate_I(&self, h: T, r_k: T, y_k: T, u: T, v: T) -> T {
        self.I_k1 + (self.K * h / self.T_i) * (r_k - y_k) + (h / self.T_t) * (u - v)
    }

    fn calculate_D(&self, h: T, y_k: T) -> Result<T, PidControllerError> {
        let two = T::from(2.0).ok_or(PidControllerError::Numeric(
            "2.0 must be representable by T",
        ))?;

        let (a_i, b_i) = if self.T_d < (self.N * h) / two {
            // use backward difference
            let a_i = self.T_d / (self.T_d + self.N * h);
            let b_i = -T::one() * self.K * self.T_d * self.N / (self.T_d + h * self.N);

            (a_i, b_i)
        } else {
            // use Tustin's approximation
            let a_i = (two * self.T_d - h * self.N) / (two * self.T_d + h * self.N);
            let b_i = -two * self.K * self.N * self.T_d / (two * self.T_d + h * self.N);

            (a_i, b_i)
        };

        if T::one() - a_i < two * T::epsilon() {
            panic!("a_i is too close to 1.0, this will cause NaN on L228"); // FIXME: figure this out
        }

        Ok(self.D_k1 + (b_i / (T::one() - a_i)) * (y_k - two * self.y_k1 + self.y_k2))
    }

    fn update_state(
        &mut self,
        h: T,
        r_k: T,
        y_k: T,
        u_low: T,
        u_high: T,
    ) -> Result<T, PidControllerError> {
        self.P_k1 = self.calculate_P(r_k, y_k);
        self.D_k1 = self.calculate_D(h, y_k)?;

        let v = self.control_output();

        // simulate saturating the output -- e.g. an actuator may have a
        // prescribed range beyond which it will not travel no matter the signal
        // it's given.
        let u = num_traits::clamp(v, u_low, u_high);

        self.I_k1 = self.calculate_I(h, r_k, y_k, u, v);

        self.r_k1 = r_k;
        self.y_k2 = self.y_k1;
        self.y_k1 = y_k;

        Ok(self.control_output())
    }
}

/// Compute the number of seconds that has elapsed between `last_update_time`
/// and `measurement_time`. Returns a `PidControllerError` if `last_update_time`
/// occurred after `measurement_time`, or if there is a problem representing a
/// number as `T`.
fn calculate_h<T>(
    measurement_time: SystemTime,
    last_update_time: SystemTime,
) -> Result<T, PidControllerError>
where
    T: num_traits::real::Real,
{
    let duration = measurement_time.duration_since(last_update_time)?;

    Ok(
        T::from(duration.as_secs()).ok_or(PidControllerError::Numeric(
            "duration's seconds part must be representable as T",
        ))? + (T::from(duration.subsec_nanos()).ok_or(PidControllerError::Numeric(
            "duration's nanoseconds part must be representable as T",
        ))? / T::from(1_000_000_000).ok_or(PidControllerError::Numeric(
            "1_000_000_000 must be representable as T",
        ))?),
    )
}

#[cfg(test)]
mod tests {
    use std::{
        fmt::Debug,
        time::{Duration, SystemTime},
    };

    use proptest::prelude::*;

    use crate::{PidController, PidControllerError};

    use super::calculate_h;

    //
    // proptest strategies
    //

    fn pid_controllers<T>(
        proportional_gains: impl Strategy<Value = T>,
        integral_time_constants: impl Strategy<Value = T>,
        derivative_time_constants: impl Strategy<Value = T>,
        tracking_time_constants: impl Strategy<Value = T>,
        derivative_gain_limits: impl Strategy<Value = T>,
        set_point_coefficients: impl Strategy<Value = T>,
    ) -> impl Strategy<Value = Result<PidController<T>, PidControllerError>>
    where
        T: num_traits::real::Real + Debug,
    {
        (
            proportional_gains,
            integral_time_constants,
            derivative_time_constants,
            tracking_time_constants,
            derivative_gain_limits,
            set_point_coefficients,
        )
            .prop_map(
                |(
                    proportional_gain,
                    integral_time_constant,
                    derivative_time_constant,
                    tracking_time_constant,
                    derivative_gain_limit,
                    set_point_coefficient,
                )| {
                    PidController::new(
                        proportional_gain,
                        integral_time_constant,
                        derivative_time_constant,
                        tracking_time_constant,
                        derivative_gain_limit,
                        set_point_coefficient,
                    )
                },
            )
    }

    fn positive_nonzero_numbers<T>() -> impl Strategy<Value = T>
    where
        T: num_traits::real::Real + Arbitrary,
    {
        any::<T>().prop_map(|n| {
            if n.is_zero() {
                n.abs() + T::epsilon()
            } else {
                n.abs()
            }
        })
    }

    fn ordered_system_times() -> impl Strategy<Value = (SystemTime, SystemTime)> {
        (any::<SystemTime>(), any::<i32>(), 0u32..1_000_000_000u32).prop_map(
            |(time, delta, nanos)| {
                (
                    time,
                    time + Duration::new(delta.unsigned_abs() as u64, nanos),
                )
            },
        )
    }

    //
    // property-based tests
    //

    proptest! {
        //
        // calculate_h tests
        //

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

        #[test]
        fn calculate_h_returns_err_when_measurement_time_before_last_update_time_f64(
            (before, after) in ordered_system_times()
        ) {
            calculate_h::<f64>(before, after)
                .expect_err("calculate_h should fail when measurement_time happens before last_update_time");
        }

        #[test]
        fn calculate_h_returns_err_when_measurement_time_before_last_update_time_f32(
            (before, after) in ordered_system_times()
        ) {
            calculate_h::<f32>(before, after)
                .expect_err("calculate_h should fail when measurement_time happens before last_update_time");
        }

        //
        // PidController tests
        //

        #[test]
        fn control_output_should_be_zero_initially_f64(
            pid_controller in pid_controllers::<f64>(
                any::<f64>(),
                positive_nonzero_numbers::<f64>(),
                positive_nonzero_numbers::<f64>(),
                positive_nonzero_numbers::<f64>(),
                positive_nonzero_numbers::<f64>(),
                any::<f64>()
            )
        ) {
            assert_eq!(
                pid_controller
                    .expect("constructor should not error")
                    .control_output(),
                0.0
            );
        }

        #[test]
        fn control_output_should_be_zero_initially_f32(
            pid_controller in pid_controllers::<f32>(
                any::<f32>(),
                positive_nonzero_numbers::<f32>(),
                positive_nonzero_numbers::<f32>(),
                positive_nonzero_numbers::<f32>(),
                positive_nonzero_numbers::<f32>(),
                any::<f32>()
            )
        ) {
            assert_eq!(
                pid_controller
                    .expect("constructor should not error")
                    .control_output(),
                0.0
            );
        }

        #[test]
        fn update_should_not_fail_f64(
            pid_controller in pid_controllers::<f64>(
                any::<f64>(),
                positive_nonzero_numbers::<f64>(),
                positive_nonzero_numbers::<f64>(),
                positive_nonzero_numbers::<f64>(),
                positive_nonzero_numbers::<f64>(),
                any::<f64>(),
            ),
        ) {
            std::thread::sleep(Duration::from_millis(10));
            assert_eq!(
                pid_controller
                    .expect("constructor should not error")
                    .update(0.0, 0.0, SystemTime::now(), 0.0, 0.0)
                    .expect("update should not fail"),
                0.0
            );
        }

        #[test]
        fn update_should_not_fail_f32(
            pid_controller in pid_controllers::<f32>(
                any::<f32>(),
                positive_nonzero_numbers::<f32>(),
                positive_nonzero_numbers::<f32>(),
                positive_nonzero_numbers::<f32>(),
                positive_nonzero_numbers::<f32>(),
                any::<f32>(),
            ),
        ) {
            assert_eq!(
                pid_controller
                    .expect("constructor should not error")
                    .update(0.0, 0.0, SystemTime::now(), 0.0, 0.0)
                    .expect("update should not fail"),
                0.0
            );
        }
    }
}
