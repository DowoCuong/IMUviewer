"""Operational upper-limb frames, not a claim of anatomical ISB compliance.

q_AB maps B vectors into A; intrinsic products are Rz(f) Rx(d) Ry(r),
and Ry(plane) Rx(elevation) Ry(axial). See UPPER_LIMB.md for signs/poses.
"""
from __future__ import annotations
import math
from .math3d import (IDENTITY, conjugate, multiply, normalize, matrix, axis_angle)

JOINTS = {'shoulder': (1, 2), 'elbow': (2, 3), 'wrist': (3, 4)}
COLUMNS = ('shoulder_plane', 'shoulder_elevation', 'shoulder_axial',
           'elbow_flexion', 'elbow_supination', 'wrist_flexion', 'wrist_radial')
DIAGNOSTIC = ('elbow_carrying', 'wrist_axial_residual')
MANEUVERS = {
    'shoulder_flexion': ('shoulder', 'z'), 'shoulder_abduction': ('shoulder', 'x'),
    'elbow_flexion': ('elbow', 'z'), 'elbow_supination': ('elbow', 'y'),
    'wrist_flexion': ('wrist', 'z'), 'wrist_radial': ('wrist', 'x')}


def decompose(q, sequence, threshold=0.02):
    if not math.isfinite(threshold) or not 0 < threshold < 1:
        raise ValueError('Ngưỡng kỳ dị không hợp lệ')
    r = matrix(q)
    if sequence == 'YXY':
        middle = math.acos(max(-1., min(1., r[1][1])))
        condition = abs(math.sin(middle))
        first = math.atan2(r[0][1], r[2][1])
        last = math.atan2(r[1][0], -r[1][2])
    elif sequence == 'ZXY':
        middle = math.asin(max(-1., min(1., r[2][1])))
        condition = abs(math.cos(middle))
        first = math.atan2(-r[0][1], r[1][1])
        last = math.atan2(-r[2][0], r[2][2])
    else:
        raise ValueError('Chuỗi quay không được hỗ trợ')
    return ((None if condition < threshold else math.degrees(first)), math.degrees(middle),
            (None if condition < threshold else math.degrees(last))), condition


class UpperLimbKinematics:
    model_id = 'upper_limb_v1'
    def __init__(self, side='right'):
        if side not in ('left', 'right'):
            raise ValueError('Chọn tay trái hoặc phải')
        self.side = side
        self.reset()

    def reset(self):
        self.sensor_to_segment = {}
        self.parent_frames = {j: IDENTITY for j in JOINTS}
        self.child_frames = {j: IDENTITY for j in JOINTS}
        self.calibration_kind = 'uncalibrated'
        self.calibration_report = {}
        self.validation = {}
        self.singularity_threshold = .02

    @property
    def calibrated(self):
        return set(self.sensor_to_segment) == {1, 2, 3, 4}

    def segments(self, sensors):
        return {n: normalize(multiply(normalize(q), self.sensor_to_segment.get(n, IDENTITY)))
                for n, q in sensors.items()}

    def compute_detailed(self, sensors):
        segments = self.segments(sensors)
        angles, quality, relative, conditioning = {}, {}, {}, {}
        for joint, (parent, child) in JOINTS.items():
            keys = {'shoulder': COLUMNS[:3], 'elbow': ('elbow_flexion', 'elbow_carrying', 'elbow_supination'),
                    'wrist': ('wrist_flexion', 'wrist_radial', 'wrist_axial_residual')}[joint]
            if parent not in segments or child not in segments:
                for key in keys:
                    angles[key], quality[key] = None, 'missing_pair'
                continue
            q = normalize(multiply(conjugate(segments[parent]), segments[child]))
            q = normalize(multiply(multiply(conjugate(self.parent_frames[joint]), q), self.child_frames[joint]))
            relative[joint] = q
            values, conditioning[joint] = decompose(q, 'YXY' if joint == 'shoulder' else 'ZXY', self.singularity_threshold)
            s = 1 if self.side == 'right' else -1
            signs = (1, 1, 1) if joint == 'shoulder' else ((1, s, -s) if joint == 'elbow' else (1, -s, -s))
            for key, value, sign in zip(keys, values, signs):
                angles[key] = None if value is None else value*sign
                quality[key] = 'near_singularity' if value is None else ('valid' if self.calibrated else 'uncalibrated')
        return {'angles': angles, 'quality': quality, 'relative': relative, 'conditioning': conditioning}

    def profile(self):
        return {'schema_version': 1, 'model_id': self.model_id, 'side': self.side,
                'node_mapping': {'1':'thorax','2':'upper_arm','3':'forearm','4':'hand'},
                'quaternion_convention': 'Hamilton_wxyz_qAB_maps_B_to_A',
                'segment_frames': 'x anterior, y superior/proximal, z right; anatomical reference pose',
                'sensor_to_segment': {str(n):q for n,q in self.sensor_to_segment.items()},
                'joint_frames': {'parent': self.parent_frames, 'child': self.child_frames},
                'angle_conventions': {'shoulder': 'intrinsic_YXY_raw_plane_axial',
                                      'elbow': 'intrinsic_ZXY_FE_carrying_PS', 'wrist': 'intrinsic_ZXY_FE_RU_residual'},
                'reference_pose': 'arms_down_elbows_extended_palms_anterior',
                'reference_kind': 'operational_pose_not_verified_bony_landmarks',
                'kind': self.calibration_kind, 'calibration_report': self.calibration_report,
                'validation': self.validation, 'singularity_threshold': self.singularity_threshold}

    def load_profile(self, p):
        if (p.get('schema_version') != 1 or p.get('model_id') != self.model_id or p.get('side') != self.side
            or p.get('quaternion_convention') != 'Hamilton_wxyz_qAB_maps_B_to_A'
            or p.get('reference_pose') != 'arms_down_elbows_extended_palms_anterior'
            or p.get('node_mapping') != self.profile()['node_mapping']
            or p.get('angle_conventions') != self.profile()['angle_conventions']):
            raise ValueError('Profile khác mô hình, bên tay hoặc quy ước')
        offsets = {int(n):normalize(q) for n,q in p['sensor_to_segment'].items()}
        if set(offsets) != {1,2,3,4}:
            raise ValueError('Profile cần đủ bốn node')
        parent = {j:normalize(p['joint_frames']['parent'][j]) for j in JOINTS}
        child = {j:normalize(p['joint_frames']['child'][j]) for j in JOINTS}
        threshold = float(p['singularity_threshold'])
        decompose(IDENTITY, 'YXY', threshold)
        if not isinstance(p.get('calibration_report'),dict) or not isinstance(p.get('validation'),dict):
            raise ValueError('Profile thiếu báo cáo hiệu chuẩn/kiểm tra')
        if any(not isinstance(v,dict) for v in p['validation'].values()):
            raise ValueError('Báo cáo kiểm tra không hợp lệ')
        self.sensor_to_segment, self.parent_frames, self.child_frames = offsets, parent, child
        self.singularity_threshold = threshold
        self.calibration_kind = str(p.get('kind', 'neutral_reference'))
        self.calibration_report = p.get('calibration_report', {})
        self.validation = p.get('validation', {})
