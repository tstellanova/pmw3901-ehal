# pmw3901-ehal

Rust no_std embedded hal driver for the
pmw3901 optical flow sensor hardware.


## Notes
- The only supported mode for debugging is RTT with the `rttdebug` feature. 
This is because common flight controller hardware that uses the sensor
only provide a SWD interface (no easy ITM solution).


## License
BSD-3-Clause: see LICENSE file


## Status

Work in progress

- [ ] support for SPI interface
- [ ] example using PX4-based flight controller embedded hardware



