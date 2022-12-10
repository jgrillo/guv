# guv

This crate implements a PID controller as described in

``` text
Åström, K. J., & Hägglund, T. (1988).
Automatic Tuning of PID Controllers.
Instrument Society of America (ISA).
ISBN 1-55617-081-5
```

**This crate is currently experimental and should not be used yet.**

## TODO

- [ ] robust test suite
- [ ] automatic tuning

## Credits

This project steals from [pid-rs](https://github.com/braincore/pid-rs) the idea
of being generic over a real number type.

This crate [got its start](https://github.com/grapl-security/grapl/pull/2108) at
Grapl, Inc.
