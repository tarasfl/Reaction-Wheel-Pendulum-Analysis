"""Run the reaction-wheel sizing calculation."""

from cube import Cube


def print_report(cube, result):
    print("\n" + "=" * 70)
    print("REACTION WHEEL ITERATIVE SIZING")
    print("=" * 70)
    print("\nFIXED COMPONENT MASSES")
    print("-" * 70)
    print(f"Fixed mass:                {result.fixed_mass:.4f} kg")
    print(f"Number of wheels:          {cube.number_of_wheels}")
    print(f"Active wheels:             {cube.active_wheels}")
    print(f"Motor mass each:           {cube.mass:.3f} kg")
    print(f"Motor nominal torque:      {cube.nominal_torque:.3f} N*m")
    print(f"Motor nominal speed:       {cube.nominal_rpm:.0f} RPM")
    print(f"Motor nominal power:       {cube.nominal_power:.1f} W")
    print(f"Motor electrical power:    {cube.electrical_power:.1f} W")

    print("\nCUBE GEOMETRY")
    print(f"Cube side:                 {cube.side * 1000:.1f} mm")
    print(f"COM horizontal position:   {cube.com_x * 1000:.2f} mm")
    print(f"COM vertical position:     {cube.com_y * 1000:.2f} mm")
    print(f"COM radius from edge:      {cube.com_radius * 1000:.2f} mm")
    print(f"Required COM rise:         {cube.delta_height * 1000:.2f} mm")

    print("\nITERATION")
    print("-" * 70)
    print(
        f"{'Iteration':>9} | {'Mass guess':>11} | "
        f"{'Wheel mass':>11} | {'New mass':>11} | {'Difference':>11}"
    )
    print("-" * 70)
    cube.size(print_iterations=True)

    print("\n" + "=" * 70)
    print("CALCULATION CONVERGED" if result.converged else "WARNING: DID NOT CONVERGE")
    print("=" * 70)
    print(f"Final total cube mass:     {result.total_mass:.4f} kg")
    print(f"Single flywheel mass:      {result.flywheel_mass * 1000:.1f} g")
    print(f"Total flywheel mass:       {result.total_flywheel_mass:.4f} kg")
    print(f"Outer diameter:            {2 * cube.outer_radius * 1000:.1f} mm")
    print(f"Inner diameter:            {2 * cube.inner_radius * 1000:.1f} mm")
    print(f"Required thickness:        {result.flywheel_thickness * 1000:.2f} mm")
    print(f"Flywheel speed:            {cube.rpm:.0f} RPM")
    print(f"Flywheel angular velocity: {cube.omega:.2f} rad/s")
    print(f"Cube moment of inertia:    {result.cube_inertia:.6f} kg*m^2")
    print(f"Potential energy barrier:  {result.potential_energy:.4f} J")
    print(f"Required cube velocity:    {result.cube_angular_velocity:.4f} rad/s")
    print(f"Minimum momentum:          {result.minimum_momentum:.4f} N*m*s")
    print(f"Design momentum:           {result.design_momentum:.4f} N*m*s")
    print(f"Momentum per active wheel: {result.momentum_per_wheel:.4f} N*m*s")
    print(f"Required wheel inertia:    {result.required_flywheel_inertia:.8f} kg*m^2")
    print(f"Motor reaches target RPM:  {'YES' if result.motor_can_reach_target_rpm else 'NO'}")
    print(f"Motor speed margin:        {result.motor_speed_margin_rpm:.0f} RPM")
    print(f"Ideal acceleration time:   {result.motor_ideal_acceleration_time:.4f} s")


if __name__ == "__main__":
    cube = Cube()
    result = cube.size()
    print_report(cube, result)