"""Motor specifications and flywheel drive calculations."""

import math


class Motor:
    """Motor model with nominal operating limits."""

    def __init__(self, nominal_torque=0.0536, nominal_rpm=10200.0,
                 mass=0.170, voltage=24.0, nominal_current=2.83,
                 efficiency=0.85, max_rpm=None):
        if nominal_torque <= 0 or nominal_rpm <= 0 or mass <= 0:
            raise ValueError("torque, speed, and mass must be positive")
        if voltage <= 0 or nominal_current <= 0:
            raise ValueError("voltage and current must be positive")
        if not 0 < efficiency <= 1:
            raise ValueError("efficiency must be in (0, 1]")
        if max_rpm is not None and max_rpm < nominal_rpm:
            raise ValueError("max_rpm cannot be below nominal_rpm")

        self.nominal_torque = nominal_torque
        self.nominal_rpm = nominal_rpm
        self.mass = mass
        self.voltage = voltage
        self.nominal_current = nominal_current
        self.efficiency = efficiency
        self.max_rpm = max_rpm if max_rpm is not None else nominal_rpm

    @property
    def nominal_omega(self):
        """Nominal motor speed in rad/s."""
        return self.nominal_rpm * 2.0 * math.pi / 60.0

    @property
    def nominal_power(self):
        """Mechanical power at nominal torque and speed in watts."""
        return self.nominal_torque * self.nominal_omega

    @property
    def electrical_power(self):
        """Approximate electrical input power in watts."""
        return self.voltage * self.nominal_current

    def speed_capability(self, target_rpm):
        """Return speed margin, ratio, and target-speed feasibility."""
        if target_rpm < 0:
            raise ValueError("target_rpm cannot be negative")
        return {
            "target_rpm": target_rpm,
            "speed_margin_rpm": self.nominal_rpm - target_rpm,
            "speed_ratio": target_rpm / self.nominal_rpm,
            "can_reach": target_rpm <= self.nominal_rpm,
        }

    def ideal_acceleration_time(self, inertia, target_rpm):
        """Estimate ideal acceleration time using nominal torque."""
        if inertia < 0 or target_rpm < 0:
            raise ValueError("inertia and target_rpm cannot be negative")
        target_omega = target_rpm * 2.0 * math.pi / 60.0
        return inertia * target_omega / self.nominal_torque

    def can_drive(self, inertia, target_rpm):
        """Return target-speed feasibility and ideal acceleration time."""
        result = self.speed_capability(target_rpm)
        result["ideal_acceleration_time"] = self.ideal_acceleration_time(
            inertia, target_rpm
        )
        result["nominal_torque"] = self.nominal_torque
        return result