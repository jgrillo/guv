//! This crate is experimental and should not be used.

#![cfg_attr(not(feature = "std"), no_std)]

use core::{
    fmt::{Debug, Display},
    result::Result,
};

#[derive(Clone, Debug)]
#[non_exhaustive]
pub enum PidControllerError {
    Numeric(&'static str),
    InvalidParameter(&'static str),
}

impl Display for PidControllerError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            PidControllerError::Numeric(s) => f.write_fmt(format_args!("numeric error {0}", s)),
            PidControllerError::InvalidParameter(s) => {
                f.write_fmt(format_args!("invalid parameter {0}", s))
            }
        }
    }
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
#[allow(non_snake_case)]
#[derive(Clone, Debug)]
pub struct PidController<T>
where
    T: num_traits::real::Real + Debug,
{
    K: T,
    T_i: T,
    T_d: T,
    b: T,
    two: T,
    u: T,
    r_1: T,
    y_1: T,
    y_2: T,
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
    ///   factor. Must be in `[T::zero(), 1 / T::epsilon()]`.
    /// \
    /// - `integral_time_constant` -- The time required for the integral term to
    ///   "catch up to" the proportional term in the face of an instantaneous
    ///   jump in controller error. Must be in `(T::epsilon(), 1 / T::epsilon()]`.
    /// \
    /// - `derivative_time_constant` -- The time required for the proportional
    ///   term to "catch up to" the derivative term if the error starts at zero
    ///   and increases at a fixed rate. Must be in `(T::epsilon(), 1 / T::epsilon()]`.
    /// \
    /// - `set_point_coefficient` -- This term determines how the controller
    ///   reacts to a change in the setpoint. Must be in
    ///   `[T::zero(), 1 / T::epsilon()]`.
    pub fn new(
        proportional_gain: T,
        integral_time_constant: T,
        derivative_time_constant: T,
        set_point_coefficient: T,
    ) -> Result<Self, PidControllerError> {
        let zero = T::zero();
        let eps = T::epsilon();
        let max = T::one() / eps;
        let two = T::from(2.0).ok_or(PidControllerError::Numeric(
            "2.0 must be representable by T",
        ))?;

        if proportional_gain < zero || proportional_gain > max {
            return Err(PidControllerError::InvalidParameter(
                "proportional_gain must be in [T::zero(), 1 / T::epsilon()]",
            ));
        }

        if integral_time_constant < eps || integral_time_constant > max {
            return Err(PidControllerError::InvalidParameter(
                "integral_time_constant must be in (T::epsilon(), 1 / T::epsilon()]",
            ));
        }

        if derivative_time_constant < eps || derivative_time_constant > max {
            return Err(PidControllerError::InvalidParameter(
                "derivative_time_constant must be in (T::epsilon(), 1 / T::epsilon()]",
            ));
        }

        if set_point_coefficient < zero || set_point_coefficient > max {
            return Err(PidControllerError::InvalidParameter(
                "set_point_coefficient must be in [T::zero(), 1 / T::epsilon()]",
            ));
        }

        Ok(Self {
            K: proportional_gain,
            T_i: integral_time_constant,
            T_d: derivative_time_constant,
            b: set_point_coefficient,
            two,
            u: zero,
            r_1: zero,
            y_1: zero,
            y_2: zero,
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
    /// - `measurement_time_delta` -- the interval of time between the previous
    ///   and current `process_measurement`
    /// \
    /// - `lower_saturation_limit` -- the minimum value the controller can take
    /// \
    /// - `upper_saturation_limit` -- the maximum value the controller can take
    pub fn update(
        &mut self,
        set_point: T,
        process_measurement: T,
        measurement_time_delta: T,
        lower_saturation_limit: T,
        upper_saturation_limit: T,
    ) -> T {
        self.update_state(
            measurement_time_delta,
            set_point,
            process_measurement,
            lower_saturation_limit,
            upper_saturation_limit,
        )
    }

    fn delta_P(&self, r: T, y: T) -> T {
        self.K * (self.b * r - y - self.b * self.r_1 + self.y_1)
    }

    fn delta_I(&self, h: T, r: T, y: T) -> T {
        (self.K * h / self.T_i) * (r - y)
    }

    fn delta_D(&self, h: T, y: T) -> T {
        ((-self.K * self.T_d) / h) * (y - self.two * self.y_1 + self.y_2)
    }

    fn update_state(&mut self, h: T, r: T, y: T, u_low: T, u_high: T) -> T {
        let delta_P = self.delta_P(r, y);
        let delta_I = self.delta_I(h, r, y);
        let delta_D = self.delta_D(h, y);

        let delta_v = delta_P + delta_I + delta_D;

        // simulate saturating the output -- e.g. an actuator may have a
        // prescribed range beyond which it will not travel no matter the signal
        // it's given.
        self.u = num_traits::clamp(self.u + delta_v, u_low, u_high);

        self.y_2 = self.y_1;
        self.r_1 = r;
        self.y_1 = y;

        self.u
    }
}

#[cfg(feature = "std")]
pub mod std {
    use std::time::{SystemTime, SystemTimeError};

    use crate::PidControllerError;

    impl From<SystemTimeError> for PidControllerError {
        fn from(e: SystemTimeError) -> Self {
            let msg = if let Some(msg) = format_args!("measurement time was before last update time: {:?}", e.duration()).as_str() {
                msg
            } else {
                "measurement time was before last update time"
            };

            Self::InvalidParameter(msg)
        }
    }

    /// Compute the number of seconds that has elapsed between
    /// `last_update_time` and `measurement_time`. Returns a
    /// `PidControllerError` if `last_update_time` occurred after
    /// `measurement_time`, or if there is a problem representing a number as
    /// `T`.
    pub fn calculate_h<T>(
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
}
