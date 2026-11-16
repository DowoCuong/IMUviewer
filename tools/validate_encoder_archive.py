"""Audit the legacy three-pair encoder archive without repairing data from reference.

python tools/validate_encoder_archive.py ARCHIVE --signs=-1,-1,1 --out OUTPUT
Signs declare the physical direction convention; they are never fitted. All
Encoder_Valid=1 finite pairs remain in primary metrics, including stale reference
values. Age-limited metrics are a separate sensitivity check. Originals are read
only. Quaternion replay reproduces legacy ZYX pitch, not anatomical gait angles.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from imu_desktop.core.math3d import conjugate, matrix, multiply, normalize
from imu_desktop.validate import metrics


def quaternion_pitch(row: dict) -> float:
    parent = normalize(tuple(float(row['Parent_Q_'+axis]) for axis in 'WXYZ'))
    child = normalize(tuple(float(row['Child_Q_'+axis]) for axis in 'WXYZ'))
    sine = -matrix(multiply(conjugate(parent), child))[2][0]
    return math.degrees(math.asin(max(-1.0, min(1.0, sine))))


def analyze_pair(path: Path, sign: int, max_age_ms: float = 20) -> tuple[dict, list[dict]]:
    if sign not in (-1, 1) or not math.isfinite(max_age_ms) or max_age_ms <= 0:
        raise ValueError('Declare sign +/-1 and a positive reference age threshold')
    with path.open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'No measurement rows: {path}')
    pairs, rejected = [], Counter()
    origin = None
    for row in rows:
        if row['Encoder_Valid'] != '1':
            rejected['encoder_invalid'] += 1
            continue
        try:
            stored = float(row['Relative_Angle'])
            zeroed = float(row['Relative_Angle_Zeroed'])
            reference = float(row['Encoder_Angle_Zeroed'])
            age = float(row['Encoder_Age_ms'])
            original_reference = float(row['Encoder_Angle'])
            pitch = quaternion_pitch(row)
            if not all(math.isfinite(v) for v in (stored, zeroed, reference, age, original_reference, pitch)) or age < 0:
                raise ValueError('Invalid paired measurement')
            timestamp = datetime.fromisoformat(row['Timestamp'])
        except (ValueError, KeyError, TypeError):
            rejected['malformed_valid_pair'] += 1
            continue
        if origin is None:
            origin = timestamp
        pairs.append({'t_ms': (timestamp-origin).total_seconds()*1000,
                      'measured': sign*zeroed, 'encoder': reference,
                      'stored_relative_deg': stored, 'stored_zeroed_deg': zeroed,
                      'stored_reference_deg': original_reference, 'quaternion_pitch_deg': pitch,
                      'encoder_age_ms': age, 'master_seq': row['Master_Seq'],
                      'encoder_timestamp': row['Encoder_Timestamp']})
    if len(pairs) < 2:
        raise ValueError(f'Insufficient paired measurements: {path}')
    first_pitch = pairs[0]['quaternion_pitch_deg']
    for pair in pairs:
        pair['quaternion_zeroed_measured'] = sign*(pair['quaternion_pitch_deg']-first_pitch)
    fresh = [p for p in pairs if p['encoder_age_ms'] <= max_age_ms]
    snapped = [p for p in pairs if abs(p['stored_relative_deg']) == 90 and
               abs(p['quaternion_pitch_deg']-p['stored_relative_deg']) > .01]
    def score(items, column):
        return metrics([p[column] for p in items], [p['encoder'] for p in items]) if len(items) >= 2 else None
    report = {
        'source': str(path.resolve()), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'source_rows': len(rows), 'paired_rows': len(pairs), 'rejected': dict(rejected),
        'parent_child': sorted({(r['Parent_IMU'], r['Child_IMU']) for r in rows}),
        'declared_direction_sign': sign,
        'duration_s': (pairs[-1]['t_ms']-pairs[0]['t_ms'])/1000,
        'timestamp_basis': 'Legacy host wall time; no synchronized IMU acquisition epochs available',
        'non_increasing_host_timestamps': sum(b['t_ms'] <= a['t_ms'] for a, b in zip(pairs, pairs[1:])),
        'primary_stored_zeroed': score(pairs, 'measured'),
        'literal_stored_without_direction_mapping': metrics(
            [p['stored_zeroed_deg'] for p in pairs], [p['encoder'] for p in pairs]),
        'reference_age_ms': {'median': statistics.median(p['encoder_age_ms'] for p in pairs),
                             'max': max(p['encoder_age_ms'] for p in pairs),
                             'over_30ms': sum(p['encoder_age_ms'] > 30 for p in pairs)},
        'age_sensitivity': {'max_age_ms': max_age_ms, 'excluded_count': len(pairs)-len(fresh),
                            'metrics': score(fresh, 'measured')},
        'quaternion_replay': {
            'method': 'normalize both quaternions; inv(parent)*child; unclipped ZYX pitch; first paired IMU pose zero',
            'scope': 'Retrospective legacy scalar pitch; does not validate the new anatomical/clock pipeline',
            'metrics': score(pairs, 'quaternion_zeroed_measured'),
            'age_limited_metrics': score(fresh, 'quaternion_zeroed_measured'),
            'stored_pitch_difference': metrics([p['quaternion_pitch_deg'] for p in pairs],
                                               [p['stored_relative_deg'] for p in pairs]),
            'premature_90_degree_samples': len(snapped),
            'actual_pitch_range_for_snapped_samples':
                [min(p['quaternion_pitch_deg'] for p in snapped), max(p['quaternion_pitch_deg'] for p in snapped)] if snapped else None},
        'processing': 'No smoothing, clipping, encoder-derived replacement, lag fit, fitted offset, or time warping',
        'zeroing': 'Primary retains the original first-pose zeroed columns; zeroing is already present in this archive',
        'limits': 'Encoder resolution/calibration, mechanical alignment and acquisition timing are unverified; single-axis bench data do not establish all-axis or worn-sensor accuracy'}
    return report, pairs


def plot_report(reports: list[dict], datasets: list[list[dict]], output: Path) -> None:
    # Optional offline reporting dependency; never used by the measurement app.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, len(reports), figsize=(17, 7), squeeze=False)
    for column, (report, pairs) in enumerate(zip(reports, datasets)):
        times = [p['t_ms']/1000 for p in pairs]
        measured = [p['measured'] for p in pairs]
        reference = [p['encoder'] for p in pairs]
        top, bottom = axes[:, column]
        top.plot(times, measured, label='Stored IMU (zero + declared sign)', linewidth=1)
        top.plot(times, reference, '--', label='Encoder', linewidth=1)
        bottom.plot(times, [a-b for a, b in zip(measured, reference)], linewidth=.7, label='Stored IMU error')
        if report['quaternion_replay']['premature_90_degree_samples']:
            replay = [p['quaternion_zeroed_measured'] for p in pairs]
            top.plot(times, replay, label='Quaternion-only pitch replay', linewidth=.8, alpha=.8)
            bottom.plot(times, [a-b for a, b in zip(replay, reference)], linewidth=.7, label='Quaternion replay error')
        stale = [p for p in pairs if p['encoder_age_ms'] > report['age_sensitivity']['max_age_ms']]
        if stale:
            top.scatter([p['t_ms']/1000 for p in stale], [p['encoder'] for p in stale],
                        color='crimson', s=5, label='Reference age above threshold', zorder=4)
        top.set_title(f"{report['pair_folder']} | stored RMSE {report['primary_stored_zeroed']['rmse_deg']:.3f} deg")
        for ax in (top, bottom):
            ax.grid(alpha=.25)
            ax.set_xlabel('Host elapsed time (s)')
            ax.legend(fontsize=7, loc='upper right')
        top.set_ylabel('Angle (deg)')
        bottom.set_ylabel('Signed error (deg)')
        bottom.axhline(0, color='gray', linewidth=.5)
    fig.suptitle('Historical encoder validation: all valid pairs retained; no encoder-derived peak repair', fontsize=14)
    fig.text(.5, .01, 'First-pose zeroing retained. No fitted lag/offset. Timing and encoder uncertainty unverified; not an in-vivo validation.', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .04, 1, .95))
    fig.savefig(output/'encoder-validation.png', dpi=180)
    fig.savefig(output/'encoder-validation.svg')
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--signs', required=True, help='Explicit signs for imu12,imu23,imu34, e.g. --signs=-1,-1,1')
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--max-age-ms', default=20, type=float)
    parser.add_argument('--plot', action='store_true', help='Also save PNG/SVG using optional matplotlib')
    args = parser.parse_args()
    signs = [int(value) for value in args.signs.split(',')]
    if len(signs) != 3 or any(sign not in (-1, 1) for sign in signs):
        parser.error('Exactly three signs +/-1 are required')
    args.out.mkdir(parents=True, exist_ok=True)
    reports, datasets = [], []
    for folder, sign in zip(('imu12', 'imu23', 'imu34'), signs):
        candidates, empty = [], []
        for path in sorted((args.archive/folder).glob('imu_pair_relative_*.csv')):
            with path.open(encoding='utf-8-sig', newline='') as stream:
                has_rows = next(csv.DictReader(stream), None) is not None
            (candidates if has_rows else empty).append(path)
        if len(candidates) != 1:
            parser.error(f'{folder}: expected exactly one nonempty pair CSV, found {len(candidates)}; choose an unambiguous archive')
        report, pairs = analyze_pair(candidates[0], sign, args.max_age_ms)
        report['pair_folder'] = folder
        report['header_only_files'] = [str(p) for p in empty]
        output = args.out/(folder+'-paired.csv')
        with output.open('w', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(pairs[0]))
            writer.writeheader()
            writer.writerows(pairs)
        report['paired_export'] = str(output.resolve())
        reports.append(report)
        datasets.append(pairs)
        print(folder, f"stored RMSE={report['primary_stored_zeroed']['rmse_deg']:.6f} deg; "
              f"quaternion replay={report['quaternion_replay']['metrics']['rmse_deg']:.6f} deg")
    result = {'archive': str(args.archive.resolve()), 'trials': reports,
              'accuracy_claim': 'Historical bench reanalysis only. New software/firmware physical accuracy requires a new independent measurement.'}
    (args.out/'encoder-report.json').write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')
    if args.plot:
        plot_report(reports, datasets, args.out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
