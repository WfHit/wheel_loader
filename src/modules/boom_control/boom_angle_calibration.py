#!/usr/bin/env python3
"""
Boom Angle Calibration Script

This script calibrates the angle of boom AD relative to ground using:
- Measured cylinder length l1 (actuator position)
- Measured height l2 (point B height above ground)
- Known linkage geometry l3, l4, l5, l6

The goal is to establish the relationship between actuator positions and boom angle.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares, minimize
from dataclasses import dataclass
from typing import List, Tuple
import json

@dataclass
class LinkageGeometry:
    """Fixed linkage geometry parameters"""
    l3: float = 0.0  # Length AB (will be set later)
    l4: float = 0.0  # Length BC (will be set later)
    l5: float = 0.0  # Length CD (will be set later)
    l6: float = 0.0  # Length AD (boom length, will be set later)

    # Fixed pivot positions (to be determined from measurements)
    A_x: float = 0.0  # Point A X coordinate
    A_y: float = 0.0  # Point A Y coordinate
    D_x: float = 0.0  # Point D X coordinate
    D_y: float = 0.0  # Point D Y coordinate

@dataclass
class MeasurementPoint:
    """Single measurement point with cylinder length and B height"""
    l1: float  # Cylinder length (actuator position)
    l2: float  # Height of point B above ground
    boom_angle: float = None  # Calculated boom angle (to be filled)

class BoomAngleCalibrator:
    """Calibrates boom angle from actuator measurements"""

    def __init__(self, geometry: LinkageGeometry):
        self.geometry = geometry
        self.measurements: List[MeasurementPoint] = []
        self.calibration_params = None

    def add_measurement(self, l1: float, l2: float):
        """Add a measurement pair (cylinder length, B height)"""
        measurement = MeasurementPoint(l1=l1, l2=l2)
        self.measurements.append(measurement)
        print(f"Added measurement: l1={l1:.3f}, l2={l2:.3f}")

    def load_measurements_from_file(self, filename: str):
        """Load measurements from CSV file"""
        try:
            data = np.loadtxt(filename, delimiter=',', skiprows=1)
            for row in data:
                self.add_measurement(row[0], row[1])
            print(f"Loaded {len(data)} measurements from {filename}")
        except Exception as e:
            print(f"Error loading measurements: {e}")

    def save_measurements_to_file(self, filename: str):
        """Save measurements to CSV file"""
        with open(filename, 'w') as f:
            f.write("l1_cylinder_length,l2_b_height,boom_angle_deg\n")
            for m in self.measurements:
                angle_deg = np.degrees(m.boom_angle) if m.boom_angle is not None else 0
                f.write(f"{m.l1:.6f},{m.l2:.6f},{angle_deg:.6f}\n")

    def solve_forward_kinematics(self, l1: float) -> Tuple[float, float, float, float, float]:
        """
        Solve forward kinematics to find point positions given actuator length l1

        Returns: (B_x, B_y, C_x, C_y, boom_angle)
        """
        # This is a simplified model - needs geometry parameters
        # For now, return placeholder values
        # TODO: Implement actual forward kinematics based on your specific geometry

        A_x, A_y = self.geometry.A_x, self.geometry.A_y
        D_x, D_y = self.geometry.D_x, self.geometry.D_y

        # Calculate boom angle from A to D
        boom_angle = np.arctan2(D_y - A_y, D_x - A_x)

        # Placeholder calculations - replace with actual kinematics
        B_x = A_x + self.geometry.l3 * np.cos(boom_angle + 0.1)  # Approximation
        B_y = A_y + self.geometry.l3 * np.sin(boom_angle + 0.1)  # Approximation

        C_x = B_x + self.geometry.l4 * np.cos(boom_angle - 0.2)  # Approximation
        C_y = B_y + self.geometry.l4 * np.sin(boom_angle - 0.2)  # Approximation

        return B_x, B_y, C_x, C_y, boom_angle

    def calculate_boom_angle_from_measurements(self, l1: float, l2: float) -> float:
        """
        Calculate boom angle from measured l1 (cylinder) and l2 (B height)

        This uses the constraint that point B height is l2
        """
        # Method 1: Use geometric relationships
        # Since we know B height (l2) and can calculate B position from l1,
        # we can determine the boom orientation

        # For now, use a simplified relationship
        # TODO: Implement exact geometric solution

        # Placeholder calculation - replace with actual geometry
        # This assumes some linear relationship for initial calibration
        normalized_l1 = (l1 - 500) / 1000  # Normalize cylinder length
        normalized_l2 = (l2 - 100) / 500   # Normalize height

        # Simple approximation - replace with actual calculations
        boom_angle = normalized_l1 * 0.5 + normalized_l2 * 0.3

        return boom_angle

    def calibrate_geometry_parameters(self):
        """
        Calibrate unknown geometry parameters using measurement data
        """
        if len(self.measurements) < 3:
            print("Need at least 3 measurements for calibration")
            return False

        def objective_function(params):
            """Objective function for parameter optimization"""
            # Unpack parameters to calibrate
            l3, l4, l5, A_x, A_y, D_x, D_y = params

            # Update geometry
            self.geometry.l3 = l3
            self.geometry.l4 = l4
            self.geometry.l5 = l5
            self.geometry.A_x = A_x
            self.geometry.A_y = A_y
            self.geometry.D_x = D_x
            self.geometry.D_y = D_y

            error = 0.0
            for measurement in self.measurements:
                # Calculate predicted B position from kinematics
                B_x, B_y, C_x, C_y, boom_angle = self.solve_forward_kinematics(measurement.l1)

                # Error in B height prediction
                height_error = (B_y - measurement.l2) ** 2

                # Additional geometric constraints can be added here
                error += height_error

            return error

        # Initial parameter guess
        initial_params = [1000, 800, 600, 0, 0, 2000, 500]  # l3, l4, l5, A_x, A_y, D_x, D_y

        # Bounds for parameters (adjust based on your system)
        bounds = [
            (100, 5000),   # l3
            (100, 5000),   # l4
            (100, 5000),   # l5
            (-1000, 1000), # A_x
            (-1000, 1000), # A_y
            (100, 5000),   # D_x
            (-1000, 1000), # D_y
        ]

        result = minimize(objective_function, initial_params, bounds=bounds, method='L-BFGS-B')

        if result.success:
            l3, l4, l5, A_x, A_y, D_x, D_y = result.x
            self.geometry.l3 = l3
            self.geometry.l4 = l4
            self.geometry.l5 = l5
            self.geometry.A_x = A_x
            self.geometry.A_y = A_y
            self.geometry.D_x = D_x
            self.geometry.D_y = D_y

            print("Calibration successful!")
            print(f"Calibrated parameters:")
            print(f"  l3 = {l3:.3f}")
            print(f"  l4 = {l4:.3f}")
            print(f"  l5 = {l5:.3f}")
            print(f"  A = ({A_x:.3f}, {A_y:.3f})")
            print(f"  D = ({D_x:.3f}, {D_y:.3f})")

            return True
        else:
            print("Calibration failed:", result.message)
            return False

    def calculate_boom_angles(self):
        """Calculate boom angles for all measurements"""
        for measurement in self.measurements:
            measurement.boom_angle = self.calculate_boom_angle_from_measurements(
                measurement.l1, measurement.l2
            )

    def fit_polynomial_model(self, degree: int = 3):
        """
        Fit polynomial model: boom_angle = f(l1, l2)
        """
        if len(self.measurements) < degree + 1:
            print(f"Need at least {degree + 1} measurements for degree {degree} polynomial")
            return None

        # Calculate boom angles first
        self.calculate_boom_angles()

        # Prepare data
        l1_data = np.array([m.l1 for m in self.measurements])
        l2_data = np.array([m.l2 for m in self.measurements])
        angle_data = np.array([m.boom_angle for m in self.measurements])

        # Create feature matrix for polynomial fitting
        # Include l1, l2, l1^2, l2^2, l1*l2, etc.
        features = []
        feature_names = []

        for i in range(degree + 1):
            for j in range(degree + 1 - i):
                if i + j <= degree:
                    feature = (l1_data ** i) * (l2_data ** j)
                    features.append(feature)
                    feature_names.append(f"l1^{i} * l2^{j}")

        X = np.column_stack(features)

        # Fit polynomial
        coefficients = np.linalg.lstsq(X, angle_data, rcond=None)[0]

        self.calibration_params = {
            'coefficients': coefficients.tolist(),
            'feature_names': feature_names,
            'degree': degree
        }

        # Calculate R-squared
        predicted = X @ coefficients
        ss_res = np.sum((angle_data - predicted) ** 2)
        ss_tot = np.sum((angle_data - np.mean(angle_data)) ** 2)
        r_squared = 1 - (ss_res / ss_tot)

        print(f"Polynomial fit (degree {degree}) R² = {r_squared:.4f}")

        return coefficients, feature_names

    def predict_boom_angle(self, l1: float, l2: float) -> float:
        """Predict boom angle using calibrated model"""
        if self.calibration_params is None:
            print("Model not calibrated yet. Run fit_polynomial_model() first.")
            return None

        coefficients = np.array(self.calibration_params['coefficients'])
        degree = self.calibration_params['degree']

        # Generate features for prediction
        features = []
        for i in range(degree + 1):
            for j in range(degree + 1 - i):
                if i + j <= degree:
                    feature = (l1 ** i) * (l2 ** j)
                    features.append(feature)

        prediction = np.dot(features, coefficients)
        return prediction

    def plot_results(self):
        """Plot calibration results"""
        if not self.measurements:
            print("No measurements to plot")
            return

        # Calculate angles if not done
        self.calculate_boom_angles()

        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        l1_data = [m.l1 for m in self.measurements]
        l2_data = [m.l2 for m in self.measurements]
        angle_data = [np.degrees(m.boom_angle) for m in self.measurements]

        # Plot l1 vs boom angle
        axes[0, 0].scatter(l1_data, angle_data)
        axes[0, 0].set_xlabel('Cylinder Length l1')
        axes[0, 0].set_ylabel('Boom Angle (degrees)')
        axes[0, 0].set_title('Boom Angle vs Cylinder Length')
        axes[0, 0].grid(True)

        # Plot l2 vs boom angle
        axes[0, 1].scatter(l2_data, angle_data)
        axes[0, 1].set_xlabel('Point B Height l2')
        axes[0, 1].set_ylabel('Boom Angle (degrees)')
        axes[0, 1].set_title('Boom Angle vs B Height')
        axes[0, 1].grid(True)

        # Plot 3D relationship
        ax = fig.add_subplot(2, 2, 3, projection='3d')
        ax.scatter(l1_data, l2_data, angle_data)
        ax.set_xlabel('Cylinder Length l1')
        ax.set_ylabel('Point B Height l2')
        ax.set_zlabel('Boom Angle (degrees)')
        ax.set_title('3D Relationship')

        # Plot residuals if model is fitted
        if self.calibration_params is not None:
            predicted_angles = []
            for m in self.measurements:
                pred = self.predict_boom_angle(m.l1, m.l2)
                predicted_angles.append(np.degrees(pred))

            residuals = np.array(angle_data) - np.array(predicted_angles)
            axes[1, 1].scatter(predicted_angles, residuals)
            axes[1, 1].axhline(y=0, color='r', linestyle='--')
            axes[1, 1].set_xlabel('Predicted Angle (degrees)')
            axes[1, 1].set_ylabel('Residuals (degrees)')
            axes[1, 1].set_title('Model Residuals')
            axes[1, 1].grid(True)

        plt.tight_layout()
        plt.show()

    def save_calibration(self, filename: str):
        """Save calibration parameters to JSON file"""
        if self.calibration_params is None:
            print("No calibration to save")
            return

        calibration_data = {
            'geometry': {
                'l3': self.geometry.l3,
                'l4': self.geometry.l4,
                'l5': self.geometry.l5,
                'l6': self.geometry.l6,
                'A_x': self.geometry.A_x,
                'A_y': self.geometry.A_y,
                'D_x': self.geometry.D_x,
                'D_y': self.geometry.D_y,
            },
            'calibration_params': self.calibration_params,
            'num_measurements': len(self.measurements)
        }

        with open(filename, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        print(f"Calibration saved to {filename}")

    def load_calibration(self, filename: str):
        """Load calibration parameters from JSON file"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)

            # Load geometry
            geo = data['geometry']
            self.geometry.l3 = geo['l3']
            self.geometry.l4 = geo['l4']
            self.geometry.l5 = geo['l5']
            self.geometry.l6 = geo['l6']
            self.geometry.A_x = geo['A_x']
            self.geometry.A_y = geo['A_y']
            self.geometry.D_x = geo['D_x']
            self.geometry.D_y = geo['D_y']

            # Load calibration parameters
            self.calibration_params = data['calibration_params']

            print(f"Calibration loaded from {filename}")
            return True
        except Exception as e:
            print(f"Error loading calibration: {e}")
            return False

def main():
    """Example usage of the boom angle calibrator"""

    # Create geometry object (parameters will be set later)
    geometry = LinkageGeometry()

    # Create calibrator
    calibrator = BoomAngleCalibrator(geometry)

    print("Boom Angle Calibration Tool")
    print("=" * 50)

    while True:
        print("\nOptions:")
        print("1. Add measurement (l1, l2)")
        print("2. Load measurements from CSV")
        print("3. Set geometry parameters")
        print("4. Calibrate model")
        print("5. Plot results")
        print("6. Save calibration")
        print("7. Load calibration")
        print("8. Test prediction")
        print("9. Exit")

        choice = input("\nSelect option: ").strip()

        if choice == '1':
            try:
                l1 = float(input("Enter cylinder length l1: "))
                l2 = float(input("Enter point B height l2: "))
                calibrator.add_measurement(l1, l2)
            except ValueError:
                print("Invalid input. Please enter numbers.")

        elif choice == '2':
            filename = input("Enter CSV filename: ").strip()
            calibrator.load_measurements_from_file(filename)

        elif choice == '3':
            print("Enter geometry parameters (press Enter to skip):")
            try:
                l3 = input(f"l3 (current: {geometry.l3}): ").strip()
                if l3: geometry.l3 = float(l3)

                l4 = input(f"l4 (current: {geometry.l4}): ").strip()
                if l4: geometry.l4 = float(l4)

                l5 = input(f"l5 (current: {geometry.l5}): ").strip()
                if l5: geometry.l5 = float(l5)

                l6 = input(f"l6 (current: {geometry.l6}): ").strip()
                if l6: geometry.l6 = float(l6)

                print("Geometry parameters updated")
            except ValueError:
                print("Invalid input")

        elif choice == '4':
            if len(calibrator.measurements) < 3:
                print("Need at least 3 measurements for calibration")
            else:
                degree = int(input("Enter polynomial degree (3): ") or "3")
                calibrator.fit_polynomial_model(degree)

        elif choice == '5':
            calibrator.plot_results()

        elif choice == '6':
            filename = input("Enter filename to save calibration: ").strip()
            calibrator.save_calibration(filename)

        elif choice == '7':
            filename = input("Enter filename to load calibration: ").strip()
            calibrator.load_calibration(filename)

        elif choice == '8':
            if calibrator.calibration_params is None:
                print("No calibration model available")
            else:
                try:
                    l1 = float(input("Enter test l1: "))
                    l2 = float(input("Enter test l2: "))
                    angle = calibrator.predict_boom_angle(l1, l2)
                    print(f"Predicted boom angle: {np.degrees(angle):.2f} degrees")
                except ValueError:
                    print("Invalid input")

        elif choice == '9':
            break
        else:
            print("Invalid option")

if __name__ == "__main__":
    main()
