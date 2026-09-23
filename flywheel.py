"""Reaction-wheel geometry and momentum calculations."""

import math


class Flywheel:
    """Annular flywheel model using SI units."""

    def __init__(self, density=2700.0, outer_radius=0.065,
                 inner_radius=0.045, rpm=7000.0):
        if density <= 0:
            raise ValueError("density must be positive")
        if outer_radius <= inner_radius or inner_radius < 0:
            raise ValueError("outer_radius must be greater than inner_radius")
        if rpm <= 0:
            raise ValueError("rpm must be positive")

        self.density = density
        self.outer_radius = outer_radius
        self.inner_radius = inner_radius
        self.rpm = rpm

    @property
    def omega(self):
        """Flywheel angular velocity in rad/s."""
        return self.rpm * 2.0 * math.pi / 60.0

    def inertia_from_mass(self, mass):
        """Return annular flywheel inertia for a given mass."""
        if mass < 0:
            raise ValueError("mass cannot be negative")
        return 0.5 * mass * (self.outer_radius**2 + self.inner_radius**2)

    def mass_from_inertia(self, inertia):
        """Return the mass required to provide a target inertia."""
        if inertia < 0:
            raise ValueError("inertia cannot be negative")
        return 2.0 * inertia / (self.outer_radius**2 + self.inner_radius**2)

    def thickness_from_mass(self, mass):
        """Return annular flywheel thickness for a given mass."""
        if mass < 0:
            raise ValueError("mass cannot be negative")
        volume_factor = math.pi * (self.outer_radius**2 - self.inner_radius**2)
        return mass / (self.density * volume_factor)

    def momentum_from_mass(self, mass):
        """Return angular momentum at the configured speed."""
        return self.inertia_from_mass(mass) * self.omega

    def mass_for_momentum(self, momentum):
        """Return flywheel mass required for a target angular momentum."""
        if momentum < 0:
            raise ValueError("momentum cannot be negative")
        return self.mass_from_inertia(momentum / self.omega)

    def maximum_principal_stress(self, mass):
        """Return maximum principal stress for a given flywheel mass."""
        if mass < 0:
            raise ValueError("mass cannot be negative")
        return (self.density * self.omega**2 *
                (3.3 * self.outer_radius**2 + 0.7 * self.inner_radius**2) /
                (4.0 ))

    def angular_momentum(self, inertia):
        """Return angular momentum for a given flywheel inertia."""
        if inertia < 0:
            raise ValueError("inertia cannot be negative")
        return inertia * self.omega
