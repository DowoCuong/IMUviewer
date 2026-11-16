"""Two independent functional directions per joint, with held-out validation."""
import math
from .math3d import (axis_angle, conjugate, multiply, normalize, mean_quaternion,
                     distance_deg, angular_velocity, rotate, principal_axis, dot, cross, unit, from_matrix)
from .upper_limb import JOINTS, MANEUVERS


def check_frames(frames, duration=2.):
    if len(frames) < 50 or (frames[-1].time_us-frames[0].time_us)/1e6 < duration:
        raise ValueError('Ghi ít nhất 2 giây liên tục')
    if any(not 0 < b.time_us-a.time_us <= 30000 for a,b in zip(frames, frames[1:])):
        raise ValueError('Dữ liệu hiệu chuẩn bị gián đoạn')


def reference(engine, frames):
    check_frames(frames)
    means = {n:mean_quaternion(f.orientations[n] for f in frames) for n in range(1,5)}
    spread = {n:max(distance_deg(means[n], f.orientations[n]) for f in frames) for n in means}
    if max(spread.values()) > 1.:
        raise ValueError('Giữ yên trong tư thế chuẩn; dao động vượt 1 độ')
    # Explicit operational pose, not an estimate of individual bony geometry.
    world_segment = axis_angle((1,0,0), 90)
    engine.reset()
    engine.sensor_to_segment = {n:normalize(multiply(conjugate(q), world_segment)) for n,q in means.items()}
    engine.calibration_kind = 'neutral_reference'
    engine.calibration_report['reference'] = {'spread_deg':spread, 'samples':len(frames),
        'duration_s':(frames[-1].time_us-frames[0].time_us)/1e6}
    return engine.calibration_report['reference']


def fit_axis(engine, maneuver, frames):
    if not engine.calibrated or maneuver not in MANEUVERS:
        raise ValueError('Lấy mốc tư thế chuẩn trước')
    check_frames(frames)
    joint, _ = MANEUVERS[maneuver]
    parent, child = JOINTS[joint]
    vectors = {'parent':[], 'child':[]}
    amplitude = 0.
    first = engine.segments(frames[0].orientations)
    first_rel = multiply(conjugate(first[parent]), first[child])
    # PS must be captured with elbow extended in the reference configuration.
    if maneuver == 'elbow_supination':
        if abs(engine.compute_detailed(frames[0].orientations)['angles']['elbow_flexion'] or 0) > 15:
            raise ValueError('Ghi sấp/ngửa khi khuỷu gần duỗi như tư thế chuẩn')
    for a,b in zip(frames, frames[1:]):
        dt = (b.time_us-a.time_us)/1e6
        sa,sb = engine.segments(a.orientations), engine.segments(b.orientations)
        wp = rotate(sa[parent], angular_velocity(sa[parent], sb[parent], dt))
        wc = rotate(sa[child], angular_velocity(sa[child], sb[child], dt))
        delta = tuple(v-u for u,v in zip(wp,wc))
        qrel = multiply(conjugate(sa[parent]), sa[child])
        amplitude = max(amplitude, distance_deg(first_rel,qrel))
        if math.hypot(*delta) >= math.radians(10):
            for role,node in (('parent',parent),('child',child)):
                vectors[role].append(rotate(conjugate(sa[node]),delta))
    if min(map(len, vectors.values())) < 30 or amplitude < 15:
        raise ValueError('Biên độ hoặc số mẫu chuyển động chưa đủ (tối thiểu 15 độ)')
    report = {'samples':len(frames), 'start_us':frames[0].time_us,'end_us':frames[-1].time_us,
              'duration_s':(frames[-1].time_us-frames[0].time_us)/1e6,
              'amplitude_deg':amplitude, 'axes':{}, 'concentration':{}}
    for role,values in vectors.items():
        axis, concentration = principal_axis(values)
        if concentration < .95:
            raise ValueError('Chuyển động chưa tập trung quanh một trục')
        if dot(axis,tuple(sum(v[k] for v in values[:10]) for k in range(3))) < 0:
            axis=tuple(-v for v in axis)
        report['axes'][role], report['concentration'][role] = list(axis), concentration
    return report


def signed_direction(engine, maneuver, axis):
    _, name = MANEUVERS[maneuver]
    s = 1 if engine.side == 'right' else -1
    sign = {'shoulder_flexion':1, 'shoulder_abduction':-s, 'elbow_flexion':1,
            'elbow_supination':-s, 'wrist_flexion':1, 'wrist_radial':-s}[maneuver]
    return name, tuple(v*sign for v in axis)


def capture(engine, maneuver, frames, validate=False):
    report = fit_axis(engine, maneuver, frames)
    if validate:
        fitted = engine.calibration_report.get(maneuver)
        if fitted is None:
            raise ValueError('Ghi trục trước rồi mới kiểm tra bằng chuyển động mới')
        if report['start_us'] <= fitted['end_us']:
            raise ValueError('Kiểm tra độc lập cần cửa sổ mới sau lần ghi trục')
        errors = {role:math.degrees(math.acos(max(-1.,min(1.,dot(fitted['axes'][role],report['axes'][role])))))
                  for role in ('parent','child')}
        report['axis_repeat_error_deg'] = errors
        report['accepted'] = max(errors.values()) <= 5.
        engine.validation[maneuver] = report
        return report
    engine.calibration_report[maneuver] = report
    # Refit invalidates all validation of the affected joint.
    joint,_ = MANEUVERS[maneuver]
    names = [m for m,(j,_) in MANEUVERS.items() if j==joint]
    for m in names:
        engine.validation.pop(m,None)
    if all(m in engine.calibration_report for m in names):
        candidates = {}
        residuals = {}
        for role in ('parent','child'):
            dirs = dict(signed_direction(engine,m,engine.calibration_report[m]['axes'][role]) for m in names)
            z=unit(dirs['z'])
            other='y' if 'y' in dirs else 'x'
            v=unit(dirs[other])
            independence=math.hypot(*cross(z,v))
            if independence < .7:
                raise ValueError('Hai hướng gần đồng tuyến; ghi lại hiệu chuẩn')
            perpendicular=unit(tuple(v[i]-dot(v,z)*z[i] for i in range(3)))
            if other=='y':
                y=perpendicular; x=unit(cross(y,z))
            else:
                x=perpendicular; y=unit(cross(z,x))
            candidates[role]=from_matrix(tuple(tuple(v[i] for v in (x,y,z)) for i in range(3)))
            residuals[role]={'independence':independence,
                             'orthogonalization_deg':math.degrees(math.acos(max(-1.,min(1.,dot(v,perpendicular)))))}
        engine.parent_frames[joint]=candidates['parent']
        engine.child_frames[joint]=candidates['child']
        engine.calibration_report[joint+'_frame']=residuals
    ready=all(m in engine.calibration_report for m in MANEUVERS)
    engine.calibration_kind='functional' if ready else 'neutral_reference'
    return report


def accepted(engine):
    return engine.calibration_kind=='functional' and all(engine.validation.get(m,{}).get('accepted') is True for m in MANEUVERS)
