"""
Otto Robot Driver - Direct control interface for Otto fluid handling robot
"""

import time
from typing import Optional, Tuple

import ivoryos


class Otto:
    """Driver class for Otto fluid handling robot with direct action execution."""

    def __init__(self,
                 start_x: float = 14.5,
                 tray2_x: float = 119.2,
                 start_y: float = 63.0,
                 step_y: float = 18.9,
                 z_up: float = 27.0,
                 z_down: float = 6.0,
                 refill_z_baseline: float = 60.0,
                 refill_step: float = 2.0,
                 refill_speed: float = 50.0,
                 dwell_time: float = 30.0):
        """
        Initialize Otto robot with calibration parameters.

        Args:
            start_x: X position for tray 1 (mm)
            tray2_x: X position for tray 2 (mm)
            start_y: Starting Y position (mm)
            step_y: Y increment between samples (mm)
            z_up: Z position when needles are up (mm)
            z_down: Z position when needles are in vials (mm)
            refill_z_baseline: Initial plunger position (mm)
            refill_step: Refill volume step size (mm)
            refill_speed: Plunger push speed (mm/min)
            dwell_time: Sample aspiration time (seconds)
        """
        # Calibration parameters
        self.start_x = start_x
        self.tray2_x = tray2_x
        self.start_y = start_y
        self.step_y = step_y
        self.z_up = z_up
        self.z_down = z_down

        # Refill parameters
        self.refill_z_baseline = refill_z_baseline
        self.refill_step = refill_step
        self.refill_speed = refill_speed
        self.current_refill_z = refill_z_baseline

        # Timing parameters
        self.dwell_time = dwell_time

        # Speed parameters
        self.default_speed_z = 500
        self.default_speed_xy = 3000
        self.needle_speed = 500
        self.right_most = 100

        # Sample tracking
        self.sample_count = 0

    def home(self) -> None:
        """Home all axes (G28)."""
        print("🏠 Homing Otto...")
        self._move_z(self.z_up, self.default_speed_z)
        self._move_xy(0, 0, self.default_speed_xy)
        # Execute G28 command
        self._execute_gcode("G28")
        print("✓ Homing complete")

    def _move_z(self, z: float, speed: Optional[float] = None) -> None:
        """Move Z axis to position."""
        speed = speed if speed else self.needle_speed
        gcode = f"G0 Z{z} F{speed}"
        self._execute_gcode(gcode)

    def _move_xy(self, x: float, y: float, speed: Optional[float] = None) -> None:
        """Move XY axes to position."""
        speed = speed if speed else self.default_speed_xy
        gcode = f"G0 X{x} Y{y} F{speed}"
        self._execute_gcode(gcode)

    def _execute_gcode(self, gcode: str) -> None:
        """
        Execute G-code command on robot.
        TODO: Replace with actual robot communication.
        """
        print(f"  → {gcode}")
        # Here you would send the G-code to the robot via serial/USB/network
        # Example: self.serial_port.write(f"{gcode}\n".encode())

    def _needles_right(self) -> None:
        """Move needles to the right position."""
        self._move_z(self.z_up, self.default_speed_z)
        self._move_xy(self.right_most, 0, self.default_speed_xy)

    def _calculate_sample_position(self, sample_index: int) -> Tuple[float, float]:
        """Calculate XY position for a given sample index (0-based)."""
        if sample_index < 10:
            # Tray 1
            x = self.start_x
            y = self.start_y + (sample_index + 1) * self.step_y
        else:
            # Tray 2
            x = self.tray2_x
            y = self.start_y + (sample_index - 9) * self.step_y
        return x, y

    def take_sample(self, sample_index: Optional[int] = None,
                    x: Optional[float] = None,
                    y: Optional[float] = None) -> None:
        """
        Take a sample at the specified position or index.

        Args:
            sample_index: Sample number (0-based). If provided, calculates position automatically.
            x: Manual X position (mm). Used if sample_index is None.
            y: Manual Y position (mm). Used if sample_index is None.
        """
        # Determine position
        if sample_index is not None:
            x, y = self._calculate_sample_position(sample_index)
            display_num = sample_index + 1
        else:
            if x is None or y is None:
                raise ValueError("Must provide either sample_index or both x and y")
            display_num = "manual"

        print(f"\n💧 Taking sample #{display_num} at ({x:.1f}, {y:.1f})...")

        # Sampling routine
        self.home()
        self._move_z(self.z_up, self.default_speed_z)
        self._move_xy(x, y, self.default_speed_xy)
        self._move_z(self.z_down, self.default_speed_z)

        # Aspirate
        print(f"  ⏱ Aspirating for {self.dwell_time}s...")
        time.sleep(self.dwell_time)

        self.sample_count += 1
        print(f"✓ Sample #{display_num} complete")

    def refill(self) -> None:
        """Perform refill operation."""
        print("\n🔄 Refilling...")

        # Calculate positions
        braking_z = self.current_refill_z - 2
        self.current_refill_z += self.refill_step

        # Refill routine
        self.home()
        self._needles_right()
        self._move_z(braking_z, self.default_speed_z)
        self._move_z(self.current_refill_z, self.refill_speed)

        print(f"✓ Refill complete (Z={self.current_refill_z:.1f}mm)")

    def prime_refill_line(self) -> None:
        """Prime the refill line by calibrating Z position."""
        print("\n🔧 Priming refill line...")
        self._execute_gcode("G90")  # Absolute coordinates
        self.home()
        self._needles_right()

        braking_z = 58
        self._move_z(braking_z, self.default_speed_z)
        self._move_z(self.refill_z_baseline, self.refill_speed)

        self.home()
        print("✓ Priming complete")

    def run_sampling_sequence(self,
                              num_samples: int,
                              interval: float = 3600.0,
                              skip_refill: bool = False) -> None:
        """
        Run automated sampling sequence.

        Args:
            num_samples: Number of samples to take (max 20)
            interval: Time between samples in seconds
            skip_refill: If True, skip refill operations
        """
        if num_samples > 20:
            raise ValueError("Maximum 20 samples allowed")

        print(f"\n🚀 Starting sampling sequence: {num_samples} samples")
        print(f"   Interval: {interval}s, Refill: {'disabled' if skip_refill else 'enabled'}\n")

        for i in range(num_samples):
            # Wait interval (except before first sample)
            if i > 0:
                print(f"\n⏳ Waiting {interval}s...")
                time.sleep(interval)

            # Take sample
            self.take_sample(sample_index=i)

            # Refill
            if not skip_refill:
                self.refill()

        print("\n✅ Sampling sequence complete!")
        self.home()

    def reset_refill_counter(self) -> None:
        """Reset refill Z position to baseline."""
        self.current_refill_z = self.refill_z_baseline
        print(f"↺ Refill counter reset to {self.refill_z_baseline}mm")


# Example usage
if __name__ == "__main__":
    # Initialize Otto with default calibration (build 0884)
    otto = Otto()

    # Example 1: Home the robot
    otto.home()

    # Example 2: Prime refill line
    otto.prime_refill_line()

    # Example 3: Take a single sample
    otto.take_sample(sample_index=0)  # First sample position

    # Example 4: Manual sample at specific position
    otto.take_sample(x=20.0, y=80.0)

    # Example 5: Run full sampling sequence
    # otto.run_sampling_sequence(
    #     num_samples=5,
    #     interval=60.0,  # 60 seconds between samples
    #     skip_refill=False
    # )
    ivoryos.run(__name__)