# Miyoo Mini (SSD202D) Linux 6.5 - bring-up notes

## TODO / known issues

- **BACH audio: duplicate debugfs directory warning.** The `msc313-bach`
  driver prints on probe:

      debugfs: Directory '1f2a0400.bach' with parent 'msc313-bach' already present!

  (twice). Harmless - the card still registers as ASoC card #0 - but the
  driver is creating its debugfs dir more than once. Fix later:
  `sound/soc/mstar/msc313-bach.c`, probe path around the debugfs setup.

- **Backlight dimming.** Only the top (max) PWM level currently lights the
  panel; the LED-boost driver seems to want a different PWM frequency/duty
  response. Parked with `default-brightness-level = <7>` (max) in
  `arch/arm/boot/dts/sigmastar/mstar-infinity2m-ssd202d-miyoo-mini.dts` so the
  screen is visible. Needs proper tuning of the PWM period / brightness curve.
