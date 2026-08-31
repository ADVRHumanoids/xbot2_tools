# xbot2_tools
Collection of utilities for the XBot2 framework

## Internal-stress removal plugin

`internal_stress_removal_plugin` makes the active joint-position reference
compatible with the measured position of selected contact links. At start it
reads the link-side joint encoders, stores the Cartesian translation of every
configured URDF link, and computes the closest feasible joint reference. It
then holds that projected reference while it is active. Contact orientation is
not constrained.

```yaml
xbotcore_plugins:
  remove_internal_stress:
    type: internal_stress_removal_plugin
    thread: rt_main
    params:
      contact_links: [lf_foot, rf_foot, lh_foot, rh_foot]
      # Optional solver settings:
      # max_iterations: 20
      # tolerance: 1.0e-5  # metres
      # damping: 1.0e-8
      # max_step: 0.1      # radians per iteration
      # max_reference_rate: 1.0  # radians per second
```

The plugin acquires position control for all one-degree-of-freedom joints, so
start it only after the controller that supplied the reference has released
those resources. The projected target is approached at `max_reference_rate`,
so it does not cause a single-cycle position-reference discontinuity.
