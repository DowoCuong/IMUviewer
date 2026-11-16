import math
import random
import unittest

from imu_desktop.core.alignment import JointFrame
from imu_desktop.core.angles import JointKinematics
from imu_desktop.core.calibration import functional_calibration, neutral_calibration
from imu_desktop.core.math3d import (IDENTITY, axis_angle, cardan_zxy, conjugate,
    distance_deg, from_matrix, matrix, mean_quaternion, multiply, normalize, slerp)


def compose(a, b, c):
    return multiply(axis_angle((0, 0, 1), a), multiply(axis_angle((1, 0, 0), b), axis_angle((0, 1, 0), c)))


class QuaternionTests(unittest.TestCase):
    def test_cardan_components_against_independent_matrix_construction(self):
        rng = random.Random(442)
        def matmul(a, b):
            return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        for _ in range(300):
            angles = (rng.uniform(-170, 170), rng.uniform(-70, 70), rng.uniform(-170, 170))
            a, b, c = map(math.radians, angles)
            rz = [[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]]
            rx = [[1, 0, 0], [0, math.cos(b), -math.sin(b)], [0, math.sin(b), math.cos(b)]]
            ry = [[math.cos(c), 0, math.sin(c)], [0, 1, 0], [-math.sin(c), 0, math.cos(c)]]
            q = from_matrix(matmul(matmul(rz, rx), ry))
            for actual, expected in zip(cardan_zxy(q), angles):
                self.assertAlmostEqual(actual, expected, places=8)

    def test_sign_flip_and_small_angle_precision(self):
        a = axis_angle((1, 2, 3), .000001)
        self.assertAlmostEqual(distance_deg(IDENTITY, a), .000001, places=12)
        self.assertAlmostEqual(distance_deg(a, tuple(-v for v in a)), 0)
        self.assertAlmostEqual(distance_deg(slerp(a, tuple(-v for v in a), .5), a), 0)

    def test_slerp_midpoint_and_no_extrapolation(self):
        self.assertAlmostEqual(distance_deg(IDENTITY, slerp(IDENTITY, axis_angle((0, 0, 1), 60), .5)), 30)
        with self.assertRaises(ValueError):
            slerp(IDENTITY, IDENTITY, 1.1)

    def test_zero_and_nonfinite_quaternion_rejected(self):
        for q in ((0, 0, 0, 0), (math.nan, 0, 0, 1), (1, 2, 3)):
            with self.assertRaises(ValueError):
                normalize(q)

    def test_quaternion_mean_handles_antipodal_samples(self):
        q = axis_angle((1, 2, 3), 175)
        self.assertAlmostEqual(distance_deg(mean_quaternion([q, tuple(-v for v in q)]*20), q), 0)

    def test_rotation_matrix_roundtrip_at_180(self):
        for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 2, 3)):
            q = axis_angle(axis, 180)
            self.assertLess(distance_deg(from_matrix(matrix(q)), q), 1e-10)

    def test_joint_singularity_is_explicit(self):
        with self.assertRaises(ValueError):
            cardan_zxy(compose(30, 90, 20))

    def test_near_90_does_not_snap_real_angles_to_90(self):
        # Legacy archive used abs(sin(angle)) >= .999 and corrupted 87.44..90.
        for angle in (87.5, 88, 88.4, 89.9, -87.5, -89.9):
            actual = cardan_zxy(compose(17, angle, -12))
            for value, expected in zip(actual, (17, angle, -12)):
                self.assertAlmostEqual(value, expected, places=8)


class KinematicsTests(unittest.TestCase):
    def test_common_global_motion_and_arbitrary_sensor_mounting_cancel(self):
        rng = random.Random(21)
        engine = JointKinematics()
        mounts = {n: axis_angle((rng.random(), rng.random(), rng.random()), rng.uniform(-180, 180)) for n in range(1, 5)}
        engine.sensor_to_segment = {n: conjugate(q) for n, q in mounts.items()}
        for _ in range(100):
            angles = [(rng.uniform(-25, 80), rng.uniform(-20, 20), rng.uniform(-20, 20)) for _ in range(3)]
            global_q = axis_angle((rng.random(), rng.random(), rng.random()), rng.uniform(-180, 180))
            segments = {1: global_q}
            for n, values in enumerate(angles, 2):
                segments[n] = multiply(segments[n-1], compose(*values))
            sensors = {n: multiply(segments[n], mounts[n]) for n in segments}
            result = engine.compute(sensors)
            for joint, expected in zip(("hip", "knee", "ankle"), angles):
                for component, value, sign in zip(("flexion", "abduction", "rotation"), expected, engine.signs[joint]):
                    self.assertAlmostEqual(result[f"{joint}_{component}"], value*sign, places=8)

    def test_all_joint_signs_and_neutral_ankle(self):
        engine = JointKinematics()
        result = engine.compute({1: IDENTITY, 2: compose(-10, 0, 0), 3: compose(-30, 0, 0), 4: compose(-45, 0, 0)})
        self.assertAlmostEqual(result["hip"], -10)
        self.assertAlmostEqual(result["knee"], 20)
        self.assertAlmostEqual(result["ankle"], -15)
        self.assertAlmostEqual(engine.compute({n: IDENTITY for n in range(1, 5)})["ankle"], 0)

    def test_neutral_pose_zeroes_arbitrary_mounts_and_profile_roundtrips(self):
        engine = JointKinematics()
        sensors = {n: axis_angle((1, n, n+1), n*30) for n in range(1, 5)}
        engine.calibrate_neutral(sensors)
        result = engine.compute(sensors)
        for value in result.values():
            self.assertAlmostEqual(value, 0, places=10)
        other = JointKinematics()
        other.load_profile(engine.profile())
        restored = other.compute(sensors)
        for key in result:
            self.assertAlmostEqual(restored[key], result[key], places=10)

    def test_missing_sensor_and_bad_profile_fail(self):
        engine = JointKinematics()
        with self.assertRaises(ValueError):
            engine.compute({1: IDENTITY})
        with self.assertRaises(ValueError):
            engine.load_profile({"schema": 99})

    def test_neutral_requires_static_gap_free_capture(self):
        frames = [JointFrame(i*10000, {n: IDENTITY for n in range(1, 5)}, 20, 10000, "test") for i in range(251)]
        engine = JointKinematics()
        report = neutral_calibration(engine, frames)
        self.assertEqual(report["duration_s"], 2.5)
        frames[100] = JointFrame(1_000_000, {n: compose(5, 0, 0) for n in range(1, 5)}, 20, 10000, "test")
        with self.assertRaises(ValueError):
            neutral_calibration(engine, frames)

    def test_functional_axis_recovers_heading_without_constraining_gait(self):
        engine = JointKinematics()
        upright = axis_angle((1, 0, 0), 90)
        engine.calibrate_neutral({n: upright for n in range(1, 5)})
        basis = axis_angle((0, 1, 0), 35)
        frames = []
        for i in range(501):
            angle = 30*math.sin(2*math.pi*i/200)
            local = multiply(multiply(basis, compose(angle, 0, 0)), conjugate(basis))
            frames.append(JointFrame(i*10000, {1: upright, 2: multiply(upright, local),
                3: multiply(upright, local), 4: multiply(upright, local)}, 20, 10000, "test"))
        report = functional_calibration(engine, "hip", frames)
        self.assertGreater(report["concentration"], .999)
        pose = multiply(multiply(basis, compose(40, 8, -6)), conjugate(basis))
        result = engine.compute({1: upright, 2: multiply(upright, pose), 3: upright, 4: upright})
        self.assertAlmostEqual(result["hip_flexion"], 40, places=6)
        self.assertAlmostEqual(result["hip_abduction"], 8, places=6)
        self.assertAlmostEqual(result["hip_rotation"], -6, places=6)

    def test_functional_axis_keeps_obliquity_instead_of_creating_cross_talk(self):
        engine = JointKinematics()
        upright = axis_angle((1, 0, 0), 90)
        engine.calibrate_neutral({n: upright for n in range(1, 5)})
        basis = multiply(axis_angle((0, 1, 0), 35), axis_angle((1, 0, 0), 15))
        frames = []
        for i in range(501):
            local = multiply(multiply(basis, compose(30*math.sin(2*math.pi*i/200), 0, 0)), conjugate(basis))
            frames.append(JointFrame(i*10000, {1: upright, 2: multiply(upright, local),
                3: multiply(upright, local), 4: multiply(upright, local)}, 20, 10000, "test"))
        functional_calibration(engine, "hip", frames)
        local = multiply(multiply(basis, compose(65, 0, 0)), conjugate(basis))
        result = engine.compute({1: upright, 2: multiply(upright, local), 3: upright, 4: upright})
        self.assertAlmostEqual(result["hip_flexion"], 65, places=6)
        self.assertAlmostEqual(result["hip_abduction"], 0, places=6)
        self.assertAlmostEqual(result["hip_rotation"], 0, places=6)


if __name__ == "__main__":
    unittest.main()
