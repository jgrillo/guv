//! This crate is experimental and should not be used.

#![cfg_attr(not(feature = "std"), no_std)]

use core::{fmt::Debug, ops::Neg, result::Result};

/// This trait allows `PidController<T: Number>` to be defined abstractly over
/// the underlying number type. Blanket implementations are (optionally)
/// provided for the following types:
/// \
/// - `num_traits::float::Float + num_traits::FloatConst` if compiled with feature
///   `"float"`
/// \
/// - `fixed::traits::FixedSigned + cordic::CordicNumber` if compiled with
///   feature `"fixed"`
/// \
/// **N.B.:** while these features aren't mutually exclusive--the crate will
/// compile with them both activated--you'll have to supply your `PidController`
/// with a type `T` which implements both `num_traits::float::Float` and
/// `fixed::traits::FixedSigned`. If you accomplish this, please let me know!
pub trait Number
where
    Self: Copy + PartialOrd + Neg<Output = Self>,
{
    fn zero() -> Self;

    fn one() -> Self;

    fn epsilon() -> Self;

    fn max_value() -> Self;

    fn pi() -> Self;

    fn abs(&self) -> Self;

    fn clamp(&self, min: Self, max: Self) -> Self;

    fn safe_add(&self, rhs: Self) -> Self;

    fn safe_sub(&self, rhs: Self) -> Self;

    fn safe_mul(&self, rhs: Self) -> Self;

    fn safe_div(&self, rhs: Self) -> Self;

    fn sin(&self) -> Self;

    fn cos(&self) -> Self;
}

// we use this impl when "float" but not "fixed"
#[cfg(all(feature = "float", not(feature = "fixed")))]
impl<T> Number for T
where
    T: num_traits::float::Float + num_traits::FloatConst,
{
    fn zero() -> T {
        T::zero()
    }

    fn one() -> T {
        T::one()
    }

    fn epsilon() -> T {
        T::epsilon()
    }

    fn max_value() -> T {
        Self::one().safe_div(Self::epsilon())
    }

    fn pi() -> T {
        T::PI()
    }

    fn abs(&self) -> T {
        T::abs(*self)
    }

    fn clamp(&self, min: T, max: T) -> T {
        if self < &min {
            min
        } else if self > &max {
            max
        } else {
            *self
        }
    }

    fn safe_add(&self, rhs: Self) -> Self {
        let max = Self::max_value();
        let res = *self + rhs;
        if <Self as num_traits::float::Float>::is_finite(res) {
            res.clamp(-max, max)
        } else {
            max
        }
    }

    fn safe_sub(&self, rhs: Self) -> Self {
        let max = Self::max_value();
        let res = *self - rhs;
        if <Self as num_traits::float::Float>::is_finite(res) {
            res.clamp(-max, max)
        } else {
            -max
        }
    }

    fn safe_mul(&self, rhs: Self) -> Self {
        let max = Self::max_value();
        let res = *self * rhs;
        if <Self as num_traits::float::Float>::is_finite(res) {
            res.clamp(-max, max)
        } else {
            max
        }
    }

    fn safe_div(&self, rhs: Self) -> Self {
        let max = Self::max_value();
        if rhs == <Self as Number>::zero() {
            max
        } else {
            let res = *self / rhs;
            if <Self as num_traits::float::Float>::is_finite(res) {
                res.clamp(-max, max)
            } else if self.safe_mul(rhs) > <Self as Number>::zero() {
                // overflow
                max
            } else {
                // underflow
                -max
            }
        }
    }

    fn sin(&self) -> T {
        T::sin(*self)
    }

    fn cos(&self) -> T {
        T::cos(*self)
    }
}

// we use this impl when "fixed" but not "float"
#[cfg(all(feature = "fixed", not(feature = "float")))]
impl<T> Number for T
where
    T: fixed::traits::FixedSigned
        + cordic::CordicNumber
        + num_traits::SaturatingAdd
        + num_traits::SaturatingSub
        + num_traits::SaturatingMul
        + num_traits::CheckedDiv,
{
    fn zero() -> T {
        cordic::CordicNumber::zero()
    }

    fn one() -> T {
        cordic::CordicNumber::one()
    }

    fn epsilon() -> T {
        T::DELTA
    }

    fn max_value() -> T {
        T::saturating_recip(Self::epsilon())
    }

    fn pi() -> T {
        cordic::CordicNumber::pi()
    }

    fn abs(&self) -> T {
        T::abs(*self)
    }

    fn clamp(&self, min: T, max: T) -> T {
        if self < &min {
            min
        } else if self > &max {
            max
        } else {
            *self
        }
    }

    fn safe_add(&self, rhs: Self) -> Self {
        let max = Self::max_value();
        self.saturating_add(&rhs).clamp(-max, max)
    }

    fn safe_sub(&self, rhs: Self) -> Self {
        let max = Self::max_value();
        self.saturating_sub(&rhs).clamp(-max, max)
    }

    fn safe_mul(&self, rhs: Self) -> Self {
        let max = Self::max_value();
        self.saturating_mul(&rhs).clamp(-max, max)
    }

    fn safe_div(&self, rhs: Self) -> Self {
        let max = Self::max_value();
        if let Some(retval) = self.checked_div(&rhs) {
            retval.clamp(-max, max)
        } else if self.safe_mul(rhs) >= <Self as Number>::zero() {
            // overflow or divide-by-zero
            max
        } else {
            // underflow
            -max
        }
    }

    fn sin(&self) -> T {
        cordic::sin(*self)
    }

    fn cos(&self) -> T {
        cordic::cos(*self)
    }
}

// We use this impl when both "float" and "fixed". This mostly just exists to
// defeat the orphan rule. There is no such T which implements
// both FixedSigned and Float.
#[cfg(all(features = "fixed", feature = "float"))]
impl<T> Number for T
where
    T: fixed::traits::FixedSigned + num_traits::float::Float,
{
    fn zero() -> T {
        unimplemented!()
    }

    fn one() -> T {
        unimplemented!()
    }

    fn epsilon() -> T {
        unimplemented!()
    }

    fn max_value() -> T {
        unimplemented!()
    }

    fn pi() -> T {
        unimplemented!()
    }

    fn abs(&self) -> T {
        unimplemented!()
    }

    fn clamp(&self, min: T, max: T) -> T {
        unimplemented!()
    }

    fn safe_add(&self, rhs: Self) -> Self {
        unimplemented!()
    }

    fn safe_sub(&self, rhs: Self) -> Self {
        unimplemented!()
    }

    fn safe_mul(&self, rhs: Self) -> Self {
        unimplemented!()
    }

    fn safe_div(&self, rhs: Self) -> Self {
        unimplemented!()
    }

    fn sin(&self) -> T {
        unimplemented!()
    }

    fn cos(&self) -> T {
        unimplemented!()
    }
}

#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub enum Parameter {
    ProportionalGain,
    IntegralTimeConstant,
    DerivativeTimeConstant,
    SetPointCoefficient,
    InitialControllerOutput,
}

#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub enum PidControllerError<T>
where
    T: Number,
{
    UnrepresentableNumber,
    ParameterOutOfBounds {
        parameter: Parameter,
        value: T,
        lower_bound: T,
        upper_bound: T,
    },
    InvalidTimestamp,
}

fn check_constants<T>(
    proportional_gain: T,
    integral_time_constant: T,
    derivative_time_constant: T,
    set_point_coefficient: T,
    initial_controller_output: Option<T>,
) -> Result<(), PidControllerError<T>>
where
    T: Number,
{
    let zero = T::zero();
    let eps = T::epsilon();
    let max = T::max_value();

    if proportional_gain < zero || proportional_gain > max {
        return Err(PidControllerError::ParameterOutOfBounds {
            parameter: Parameter::ProportionalGain,
            value: proportional_gain,
            lower_bound: zero,
            upper_bound: max,
        });
    }

    if integral_time_constant < eps || integral_time_constant > max {
        return Err(PidControllerError::ParameterOutOfBounds {
            parameter: Parameter::IntegralTimeConstant,
            value: integral_time_constant,
            lower_bound: eps,
            upper_bound: max,
        });
    }

    if derivative_time_constant < eps || derivative_time_constant > max {
        return Err(PidControllerError::ParameterOutOfBounds {
            parameter: Parameter::DerivativeTimeConstant,
            value: derivative_time_constant,
            lower_bound: eps,
            upper_bound: max,
        });
    }

    if set_point_coefficient < zero || set_point_coefficient > max {
        return Err(PidControllerError::ParameterOutOfBounds {
            parameter: Parameter::SetPointCoefficient,
            value: set_point_coefficient,
            lower_bound: zero,
            upper_bound: max,
        });
    }

    if let Some(u0) = initial_controller_output {
        if u0 < -max || u0 > max {
            return Err(PidControllerError::ParameterOutOfBounds {
                parameter: Parameter::InitialControllerOutput,
                value: u0,
                lower_bound: -max,
                upper_bound: max,
            });
        }
    }

    Ok(())
}

/// This implementation of a PID controller comes from
///
/// ```text
/// Åström, K. J., & Hägglund, T. (1988).
/// Automatic Tuning of PID Controllers.
/// Instrument Society of America (ISA).
/// ISBN 1-55617-081-5
/// ```
#[allow(non_snake_case)]
#[derive(Clone, Copy, Debug)]
pub struct PidController<T>
where
    T: Number + Debug,
{
    K: T,
    T_i: T,
    T_d: T,
    b: T,
    u: T,
    r_1: T,
    y_1: T,
    y_2: T,
}

#[allow(non_snake_case)]
impl<T> PidController<T>
where
    T: Number + Debug,
{
    /// Construct a new PidController.
    /// \
    /// # Arguments:
    /// \
    /// - `proportional_gain` -- The output the controller is multiplied by this
    ///   factor. Must be in `[T::zero(), T::max_value()]`.
    /// \
    /// - `integral_time_constant` -- The time required for the integral term to
    ///   "catch up to" the proportional term in the face of an instantaneous
    ///   jump in controller error. Must be in `[T::epsilon(), T::max_value()]`.
    /// \
    /// - `derivative_time_constant` -- The time required for the proportional
    ///   term to "catch up to" the derivative term if the error starts at zero
    ///   and increases at a fixed rate. Must be in `[T::epsilon(), T::max_value()]`.
    /// \
    /// - `set_point_coefficient` -- This term determines how the controller
    ///   reacts to a change in the setpoint. Must be in
    ///   `[T::zero(), T::max_value()]`.
    /// \
    /// - `initial_controller_output` -- The controller will return this value
    ///   until it is updated. Must be in `[-T::max_value(), T::max_value()]`.
    pub fn new(
        proportional_gain: T,
        integral_time_constant: T,
        derivative_time_constant: T,
        set_point_coefficient: T,
        initial_controller_output: T,
    ) -> Result<Self, PidControllerError<T>> {
        check_constants(
            proportional_gain,
            integral_time_constant,
            derivative_time_constant,
            set_point_coefficient,
            Some(initial_controller_output),
        )?;

        let zero = T::zero();

        Ok(Self {
            K: proportional_gain,
            T_i: integral_time_constant,
            T_d: derivative_time_constant,
            b: set_point_coefficient,
            u: initial_controller_output,
            r_1: zero,
            y_1: zero,
            y_2: zero,
        })
    }

    /// Constructs a new PidController whose internal state is copied from this
    /// one except for the provided constants. The new PidController is
    /// effectively an updated copy of this one where only the provided
    /// constants have changed.
    /// \
    /// # Arguments:
    /// \
    /// - `proportional_gain` -- The output of the controller is multiplied by
    ///   this factor. Must be in `[T::zero(), T::max_value()]`.
    /// \
    /// - `integral_time_constant` -- The time required for the integral term to
    ///   "catch up to" the proportional term in the face of an instantaneous
    ///   jump in controller error. Must be in `[T::epsilon(), T::max_value()]`.
    /// \
    /// - `derivative_time_constant` -- The time required for the proportional
    ///   term to "catch up to" the derivative term if the error starts at zero
    ///   and increases at a fixed rate. Must be in `[T::epsilon(), T::max_value()]`.
    /// \
    /// - `set_point_coefficient` -- This term determines how the controller
    ///   reacts to a change in the setpoint. Must be in `[T::zero(), T::max_value()]`.
    pub fn update_constants(
        &self,
        proportional_gain: T,
        integral_time_constant: T,
        derivative_time_constant: T,
        set_point_coefficient: T,
    ) -> Result<Self, PidControllerError<T>> {
        check_constants(
            proportional_gain,
            integral_time_constant,
            derivative_time_constant,
            set_point_coefficient,
            None,
        )?;

        Ok(Self {
            K: proportional_gain,
            T_i: integral_time_constant,
            T_d: derivative_time_constant,
            b: set_point_coefficient,
            u: self.u,
            r_1: self.r_1,
            y_1: self.y_1,
            y_2: self.y_2,
        })
    }

    /// Update this PidController's internal state with the provided constants.
    /// \
    /// # Arguments:
    /// \
    /// - `proportional_gain` -- The output the controller is multiplied by this
    ///   factor. Must be in `[T::zero(), T::max_value()]`.
    /// \
    /// - `integral_time_constant` -- The time required for the integral term to
    ///   "catch up to" the proportional term in the face of an instantaneous
    ///   jump in controller error. Must be in `(T::epsilon(), T::max_value()]`.
    /// \
    /// - `derivative_time_constant` -- The time required for the proportional
    ///   term to "catch up to" the derivative term if the error starts at zero
    ///   and increases at a fixed rate. Must be in `(T::epsilon(), T::max_value()]`.
    /// \
    /// - `set_point_coefficient` -- This term determines how the controller
    ///   reacts to a change in the setpoint. Must be in `[T::zero(), T::max_value()]`.
    pub fn update_constants_mut(
        &mut self,
        proportional_gain: T,
        integral_time_constant: T,
        derivative_time_constant: T,
        set_point_coefficient: T,
    ) -> Result<(), PidControllerError<T>> {
        check_constants(
            proportional_gain,
            integral_time_constant,
            derivative_time_constant,
            set_point_coefficient,
            None,
        )?;

        self.K = proportional_gain;
        self.T_i = integral_time_constant;
        self.T_d = derivative_time_constant;
        self.b = set_point_coefficient;

        Ok(())
    }

    /// Returns the most recently computed control output
    pub fn control_output(&self) -> T {
        self.u
    }

    /// Updates this controller's state according to the given
    /// parameters. Returns the updated control output.
    /// \
    /// # Arguments:
    /// \
    /// - `set_point` -- the desired value for the process variable
    /// \
    /// - `process_measurement` -- the measured process variable value
    /// \
    /// - `measurement_time_interval` -- the elapsed time between the previous
    ///   and current `process_measurement`
    /// \
    /// - `lower_saturation_limit` -- the minimum value the controller can take
    /// \
    /// - `upper_saturation_limit` -- the maximum value the controller can take
    pub fn update(
        &mut self,
        set_point: T,
        process_measurement: T,
        measurement_time_interval: T,
        lower_saturation_limit: T,
        upper_saturation_limit: T,
    ) -> T {
        self.update_state(
            measurement_time_interval,
            set_point,
            process_measurement,
            lower_saturation_limit,
            upper_saturation_limit,
        )
    }

    fn delta_P(&self, r: T, y: T) -> T {
        // self.K * (self.b * r - y - self.b * self.r_1 + self.y_1)
        self.K.safe_mul(
            self.b
                .safe_mul(r)
                .safe_sub(y)
                .safe_sub(self.b.safe_mul(self.r_1))
                .safe_add(self.y_1),
        )
    }

    fn delta_I(&self, h: T, r: T, y: T) -> T {
        // (self.K * h / self.T_i) * (r - y)
        self.K
            .safe_mul(h)
            .safe_div(self.T_i)
            .safe_mul(r.safe_sub(y))
    }

    fn delta_D(&self, h: T, y: T) -> T {
        let two = T::one().safe_add(T::one());
        // ((-self.K * self.T_d) / h) * (y - two * self.y_1 + self.y_2)
        -self
            .K
            .safe_mul(self.T_d)
            .safe_div(h)
            .safe_mul(y.safe_sub(two.safe_mul(self.y_1)).safe_add(self.y_2))
    }

    fn update_state(&mut self, h: T, r: T, y: T, u_low: T, u_high: T) -> T {
        let delta_P = self.delta_P(r, y);
        let delta_I = self.delta_I(h, r, y);
        let delta_D = self.delta_D(h, y);

        // delta_P + delta_I + delta_D
        let delta_v = delta_P.safe_add(delta_I).safe_add(delta_D);

        // simulate saturating the output -- e.g. an actuator may have a
        // prescribed range beyond which it will not travel no matter the signal
        // it's given.
        self.u = self.u.safe_add(delta_v).clamp(u_low, u_high);

        self.y_2 = self.y_1;
        self.r_1 = r;
        self.y_1 = y;

        self.u
    }
}

#[cfg(feature = "std")]
pub mod std {
    use crate::{Number, PidControllerError};
    use std::time::SystemTimeError;

    impl<T> From<SystemTimeError> for PidControllerError<T>
    where
        T: Number,
    {
        fn from(_: SystemTimeError) -> Self {
            Self::InvalidTimestamp
        }
    }

    #[cfg(feature = "fixed")]
    pub mod fixed {
        use super::*;
        use ::fixed::traits::{FixedSigned, LosslessTryFrom};
        use std::time::SystemTime;

        /// Compute the number of seconds that has elapsed between
        /// `last_update_time` and `measurement_time`. Returns a
        /// `PidControllerError` if `last_update_time` occurred after
        /// `measurement_time`, or if there is a problem representing a number
        /// as `T`.
        pub fn calculate_h<T>(
            measurement_time: SystemTime,
            last_update_time: SystemTime,
        ) -> Result<T, PidControllerError<T>>
        where
            T: Number
                + FixedSigned
                + LosslessTryFrom<u64>
                + LosslessTryFrom<u32>
                + cordic::CordicNumber,
        {
            let duration = measurement_time.duration_since(last_update_time)?;

            Ok(T::lossless_try_from(duration.as_secs())
                .ok_or(PidControllerError::UnrepresentableNumber)?
                .safe_add(
                    T::lossless_try_from(duration.subsec_nanos())
                        .ok_or(PidControllerError::UnrepresentableNumber)?
                        .safe_div(
                            T::lossless_try_from(1_000_000_000_u32)
                                .ok_or(PidControllerError::UnrepresentableNumber)?,
                        ),
                ))
        }
    }

    #[cfg(feature = "float")]
    pub mod float {
        use super::*;
        use std::time::SystemTime;

        /// Compute the number of seconds that has elapsed between
        /// `last_update_time` and `measurement_time`. Returns a
        /// `PidControllerError` if `last_update_time` occurred after
        /// `measurement_time`, or if there is a problem representing a number
        /// as `T`.
        pub fn calculate_h<T>(
            measurement_time: SystemTime,
            last_update_time: SystemTime,
        ) -> Result<T, PidControllerError<T>>
        where
            T: Number + num_traits::cast::FromPrimitive,
        {
            let duration = measurement_time.duration_since(last_update_time)?;

            Ok(T::from_u64(duration.as_secs())
                .ok_or(PidControllerError::UnrepresentableNumber)?
                .safe_add(
                    T::from_u32(duration.subsec_nanos())
                        .ok_or(PidControllerError::UnrepresentableNumber)?
                        .safe_div(
                            T::from_u32(1_000_000_000_u32)
                                .ok_or(PidControllerError::UnrepresentableNumber)?,
                        ),
                ))
        }
    }
}
