# Repository source and modelling boundary

Repository: https://github.com/gt-marine-robotics-group/Pontus

Pinned commit: `6b9513851edb717567bb1a9e555f27a521ec036d`

Source inspected on 28 September 2026.

1. [Vehicle model](https://github.com/gt-marine-robotics-group/Pontus/blob/6b9513851edb717567bb1a9e555f27a521ec036d/pontus_description/models/pontus/pontus.xacro)
   gives density, geometry, the sim=true base-link mass rule, xDotU=0,
   xU=-5, xUabsU=-6.2282 and the 2.6 N minimum-thrust setting. Its hydrodynamic
   block contains a TODO to obtain actual values. No hardware identification is claimed.
2. [Velocity controller](https://github.com/gt-marine-robotics-group/Pontus/blob/6b9513851edb717567bb1a9e555f27a521ec036d/pontus_controller/pontus_controller/velocity_controller.py)
   uses /cmd_vel_fused and /pontus/odometry, and adds feedforward to PID before
   publishing /cmd_accel. The study replaces this variable feedforward by a
   constant trim bias and derives new PI gains; it does not copy the existing gains.
3. [Thruster controller](https://github.com/gt-marine-robotics-group/Pontus/blob/6b9513851edb717567bb1a9e555f27a521ec036d/pontus_controller/pontus_controller/thruster_controller.py)
   maps acceleration through mass/inertia and a pseudoinverse allocator, scales
   large requests, and zeroes forces at or below the deadzone. The study omits
   these actuator effects and uses an ideal equivalent net force.
4. [Thruster components](https://github.com/gt-marine-robotics-group/Pontus/blob/6b9513851edb717567bb1a9e555f27a521ec036d/pontus_description/models/pontus/components/thrusters.xacro)
   give 0.01 kg per propeller link. The reduced model uses only the published
   base-link mass parameter, neglecting these additional inertias.

The operating speed of 0.4 m/s, reference sequence, response-rate candidates,
ideal feedback and isolated surge motion are study choices. The nonlinear
equation is a Newtonian one-axis reduction using these source parameters,
not an equation copied verbatim from the repository.

The results are for a continuous-time reduced model using published settings.
They should not be represented as a simulation of the configured full AUV.
