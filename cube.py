"""Cube tipping model and iterative reaction-wheel sizing."""

import math
from dataclasses import dataclass

from flywheel import Flywheel
from motor import Motor


@dataclass(frozen=True)
class SizingResult:
    """Final values produced by the cube sizing calculation."""

    converged: bool
    iterations: int
    total_mass: float
    fixed_mass: float
    flywheel_mass: float
    total_flywheel_mass: float
    flywheel_thickness: float
    cube_inertia: float
    potential_energy: float
    cube_angular_velocity: float
    minimum_momentum: float
    design_momentum: float
    momentum_per_wheel: float
    required_flywheel_inertia: float
    motor_can_reach_target_rpm: bool
    motor_speed_margin_rpm: float
    motor_ideal_acceleration_time: float


class Cube(Flywheel, Motor):
    """Cube that inherits flywheel and motor calculations."""

    def __init__(self, side=0.150, initial_mass=1.7,
                 number_of_wheels=3, active_wheels=1,
                 structure_mass=0.350, mass=None, motor_mass=None,
                 nominal_torque=None, nominal_rpm=None, voltage=None,
                 nominal_current=None, efficiency=None, max_rpm=None,
                 motor_nominal_torque=None, motor_nominal_rpm=None,
                 motor_voltage=None, motor_nominal_current=None,
                 motor_efficiency=None, motor_max_rpm=None,
                 esc_mass_each=0.025, battery_mass=0.300,
                 electronics_mass=0.080, mechanical_hardware_mass=0.050,
                 misc_mass=0.050, com_height_factor=0.85, gravity=9.81,
                 safety_factor=1.30, transfer_efficiency=0.85,
                 tolerance=1e-6, max_iterations=100, **flywheel_parameters):
        motor_defaults = Motor()
        if motor_nominal_torque is not None:
            nominal_torque = motor_nominal_torque
        if motor_nominal_rpm is not None:
            nominal_rpm = motor_nominal_rpm
        if motor_voltage is not None:
            voltage = motor_voltage
        if motor_nominal_current is not None:
            nominal_current = motor_nominal_current
        if motor_efficiency is not None:
            efficiency = motor_efficiency
        if motor_max_rpm is not None:
            max_rpm = motor_max_rpm
        if motor_mass is not None:
            mass = motor_mass

        Flywheel.__init__(self, **flywheel_parameters)
        Motor.__init__(
            self,
            nominal_torque=(motor_defaults.nominal_torque
                            if nominal_torque is None else nominal_torque),
            nominal_rpm=(motor_defaults.nominal_rpm
                         if nominal_rpm is None else nominal_rpm),
            mass=(motor_defaults.mass if mass is None else mass),
            voltage=(motor_defaults.voltage if voltage is None else voltage),
            nominal_current=(motor_defaults.nominal_current
                             if nominal_current is None else nominal_current),
            efficiency=(motor_defaults.efficiency
                        if efficiency is None else efficiency),
            max_rpm=(motor_defaults.max_rpm if max_rpm is None else max_rpm),
        )

        if side <= 0 or initial_mass <= 0:
            raise ValueError("side and initial_mass must be positive")
        if number_of_wheels <= 0 or active_wheels <= 0:
            raise ValueError("wheel counts must be positive")
        if active_wheels > number_of_wheels:
            raise ValueError("active_wheels cannot exceed number_of_wheels")
        if not 0 < com_height_factor <= 1:
            raise ValueError("com_height_factor must be in (0, 1]")
        if safety_factor <= 0 or not 0 < transfer_efficiency <= 1:
            raise ValueError("invalid design factors")

        self.side = side
        self.initial_mass = initial_mass
        self.number_of_wheels = number_of_wheels
        self.active_wheels = active_wheels
        self.gravity = gravity
        self.safety_factor = safety_factor
        self.transfer_efficiency = transfer_efficiency
        self.tolerance = tolerance
        self.max_iterations = max_iterations

        self.com_x = side / 2.0
        self.com_y = self.com_x * com_height_factor
        self.com_radius = math.hypot(self.com_x, self.com_y)
        self.delta_height = self.com_radius - self.com_y

        self.fixed_mass = (
            structure_mass
            + number_of_wheels * self.mass
            + number_of_wheels * esc_mass_each
            + battery_mass
            + electronics_mass
            + mechanical_hardware_mass
            + misc_mass
        )

    def inertia_about_edge(self, mass):
        """Return cube inertia about the tipping edge."""
        inertia_at_com = mass * self.side**2 / 6.0
        return inertia_at_com + mass * self.com_radius**2

    def tipping_calculations(self, total_mass):
        """Calculate energy, velocity, and momentum for a cube mass."""
        cube_inertia = self.inertia_about_edge(total_mass)
        potential_energy = total_mass * self.gravity * self.delta_height
        cube_angular_velocity = math.sqrt(
            2.0 * potential_energy / cube_inertia
        )
        minimum_momentum = cube_inertia * cube_angular_velocity
        design_momentum = (
            minimum_momentum * self.safety_factor / self.transfer_efficiency
        )
        momentum_per_wheel = design_momentum / self.active_wheels
        required_inertia = momentum_per_wheel / self.omega
        motor_drive = self.can_drive(required_inertia, self.rpm)
        return {
            "cube_inertia": cube_inertia,
            "potential_energy": potential_energy,
            "cube_angular_velocity": cube_angular_velocity,
            "minimum_momentum": minimum_momentum,
            "design_momentum": design_momentum,
            "momentum_per_wheel": momentum_per_wheel,
            "required_flywheel_inertia": required_inertia,
            "motor_can_reach_target_rpm": motor_drive["can_reach"],
            "motor_speed_margin_rpm": motor_drive["speed_margin_rpm"],
            "motor_ideal_acceleration_time": motor_drive[
                "ideal_acceleration_time"
            ],
        }

    def size(self, print_iterations=False):
        """Iteratively size each wheel and return a :class:`SizingResult`."""
        total_mass = self.initial_mass
        converged = False
        wheel_mass = 0.0
        iteration = 0

        for iteration in range(1, self.max_iterations + 1):
            calculations = self.tipping_calculations(total_mass)
            wheel_mass = self.mass_from_inertia(
                calculations["required_flywheel_inertia"]
            )
            new_total_mass = self.fixed_mass + self.number_of_wheels * wheel_mass
            difference = abs(new_total_mass - total_mass)

            if print_iterations:
                print(
                    f"{iteration:9d} | {total_mass:11.6f} | "
                    f"{wheel_mass:11.6f} | {new_total_mass:11.6f} | "
                    f"{difference:11.6f}"
                )

            total_mass = new_total_mass
            if difference < self.tolerance:
                converged = True
                break

        calculations = self.tipping_calculations(total_mass)
        total_flywheel_mass = self.number_of_wheels * wheel_mass
        return SizingResult(
            converged=converged,
            iterations=iteration,
            total_mass=total_mass,
            fixed_mass=self.fixed_mass,
            flywheel_mass=wheel_mass,
            total_flywheel_mass=total_flywheel_mass,
            flywheel_thickness=self.thickness_from_mass(wheel_mass),
            **calculations,
        )



