# Update period

A sensor adapter measures at its own period, on whole physics steps. A host that stands the adapter for a real
sensor, for example a chip whose firmware programs an output data rate, compares the two and, where the adapter
allows it, changes the adapter's period to the chip's. These calls are optional: an adapter that does not implement
them keeps the defaults of the interface, and a host then can not compare or change its period.

Requirements are numbered `PER-<3 digits>`.

## Reading the period

- **PER-010** `update_period_ns()` returns true and the period the adapter measures at now [ns], or false when the
  adapter does not say. The period is a whole number of physics steps (`physics_step_ns`); an adapter that measures
  on every step returns `physics_step_ns`.
- **PER-020** For an IMU, `part_update_period_ns(ImuPart::gyro | ImuPart::accel)` returns each part's period. An
  adapter with one period for both parts may leave the default, which returns `update_period_ns()`.
- **PER-030** Reading the period does not change the adapter's state.

## Changing the period

- **PER-110** `set_update_period_ns(period)` returns true when the adapter measures every `period` from the next
  `step()` on, on the multiples of `period` counted from time 0 (`sim_time_ns % period == 0`).
- **PER-120** It returns false with a reason in `error` when the adapter does not support the change, or refuses
  the period: zero, shorter than the physics step, or not a whole number of physics steps. The adapter then measures
  as before.
- **PER-130** For an IMU, `set_part_update_period_ns(part, period)` changes one part. The default accepts only the
  period both parts already have.
- **PER-140** What a change does to the rest of the adapter's state (delay history, drift, filters, random streams)
  is the adapter's choice, and its documentation says it. An error model whose noise depends on the period (a noise
  density) uses the new period from the change on.
- **PER-150** `reset()` returns the period to the one of the adapter's configuration.

## What a host does with it

A host that knows the sensor's own period, a chip's output data rate as its firmware programmed it, compares it with
`update_period_ns()` when the period is settled (the chip starts producing data) and every time it changes. When
they differ it calls `set_update_period_ns()`; when that is refused, or the adapter does not say its period, it
reports the difference as an error. A sensor period shorter than the physics step is an error in any case.
