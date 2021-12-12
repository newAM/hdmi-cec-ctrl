# hdmi-cec-ctrl

A weekend project to turn my TV on and off via HDMI CEC.

## Why?

I previously had a raspberry pi to do this, but this was both slow (seconds to power on/off), and unreliable. `libcec` updates would often break the basic funtionaly I relied upon (power on, power off).

## Status

This is not high-quality code, this was developed with haste in mind, not quality; use at your own risk.

## Hardware

* [NUCLEO-H743ZI](https://www.st.com/en/evaluation-tools/nucleo-h743zi.html)
* [HDMI Plug Breakout Board](https://www.adafruit.com/product/3119)
