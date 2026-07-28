# Miyoo Mini (SSD202D) Linux 6.5 - bring-up notes

## Display / timing findings (2026-07-28)

- **QEMU generic-timer ran ~10x too fast (fixed).** The vendor DT pins the
  arm,armv7-timer at 6 MHz (`mstar-v7.dtsi`), but QEMU defaulted the cortex-a7
  generic timer to the 62.5 MHz back-compat rate, so guest time (and the DRM
  60 Hz vblank waits) ran ~10x fast and `drm_atomic_helper_wait_for_vblanks()`
  timed out on modesets. Fixed in the QEMU model (`hw/arm/mstarv7.c`: pin CPU
  `cntfrq` to 6 MHz). Verified with a `DRM_IOCTL_WAIT_VBLANK` probe: was
  ~5 Hz guest-perceived, now ~52 Hz.

- **Panel is mounted upside down; nothing in the pipe compensates.** The GOP
  scan-out can't flip in hardware on SSD202D (`mstar_gop.c`: flip bits not
  writable) and the kernel doesn't flip fbcon either. The DT `panel@0
  rotation = <180>` only sets the informational DRM "panel orientation"
  property. So userspace (the compositor) must compensate. DirectFB2's drmkms
  module now does (see its rotationfixes branch / the br2directfb2 patch): it
  flips the scan-out 180 for a "Upside Down" panel, covering full-screen SDL
  primaries too. fbcon, a plain DirectFB primary and chocolate-doom all come
  out upright and consistent.

- **CMA bumped to 16 MiB.** The shared 4 MiB `linux,cma` pool was too small
  for a double-buffered 640x480 primary + fbcon + DirectFB's rotation scan-out
  buffer (allocations failed with -ENOMEM). Overridden in the board DT.

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
