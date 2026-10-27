#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Estimate classic CAN data-frame utilization from candump or normalized CSV.

CSV columns: time_s,can_id,extended,dlc (IDs are hexadecimal; extended is 0 or 1).
Use a loss-free, single-bus capture. The bounds include three intermission bits,
exclude error frames/retransmissions missing from the capture, and do not measure latency.
"""
import argparse
import csv
from collections import Counter
from pathlib import Path
import re


def frame_bits(extended, dlc):
    if not 0 <= dlc <= 8:
        raise ValueError('Only classic CAN data frames with DLC 0..8 are supported')
    base = (67 if extended else 47) + 8 * dlc
    stuffed_region = (54 if extended else 34) + 8 * dlc
    return base, base + (stuffed_region - 1) // 4


def read_frames(path):
    lines = path.read_text(encoding='utf-8-sig').splitlines()
    if not lines:
        raise ValueError('Empty capture')
    if lines[0].startswith('time_s,'):
        for row in csv.DictReader(lines):
            identifier = int(row['can_id'], 16)
            extended = bool(int(row['extended']))
            if identifier > (0x1FFFFFFF if extended else 0x7FF):
                raise ValueError('Identifier outside the selected frame format')
            yield float(row['time_s']), identifier, extended, int(row['dlc'])
    else:
        pattern = re.compile(r'^\((\d+(?:\.\d+)?)\)\s+(\S+)\s+([0-9A-Fa-f]{3,8})#([0-9A-Fa-f]*)$')
        bus = None
        for line in lines:
            if not line.strip():
                continue
            match = pattern.fullmatch(line.strip())
            if match is None or len(match[4]) % 2:
                raise ValueError('Expected classic CAN data lines from candump -L; FD/RTR/errors are not supported')
            if bus is not None and bus != match[2]:
                raise ValueError('Capture contains multiple buses; select one bus before estimating load')
            bus = match[2]
            yield float(match[1]), int(match[3], 16), len(match[3]) > 3, len(match[4]) // 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--bitrate', type=int, default=500000)
    parser.add_argument('--duration', type=float, help='Actual capture duration, including leading/trailing idle time')
    args = parser.parse_args()
    frames = list(read_frames(args.capture))
    if not frames or args.bitrate <= 0:
        parser.error('Need frames and a positive bitrate')
    start = frames[0][0]
    if any(frames[i][0] > frames[i + 1][0] for i in range(len(frames) - 1)):
        parser.error('Capture timestamps must be nondecreasing')
    observed = frames[-1][0] - start
    duration = args.duration if args.duration is not None else observed
    if duration <= 0 or duration < observed:
        parser.error('Duration must be positive and at least the observed timestamp span')
    counts = Counter((identifier, extended, dlc) for _, identifier, extended, dlc in frames)
    total_min = total_max = 0
    buckets = Counter()
    for timestamp, identifier, extended, dlc in frames:
        lo, hi = frame_bits(extended, dlc)
        total_min += lo
        total_max += hi
        buckets[int(timestamp - start)] += hi
    print(f'Frames: {len(frames)}, duration: {duration:.6f} s, bitrate: {args.bitrate}')
    print(f'Data-frame load bounds: {100 * total_min / duration / args.bitrate:.2f}%'
          f' .. {100 * total_max / duration / args.bitrate:.2f}%')
    full_buckets = [bits for second, bits in buckets.items() if second + 1 <= observed]
    if full_buckets:
        print(f'Peak complete 1-second bucket, conservative bits: {100 * max(full_buckets) / args.bitrate:.2f}%')
    print('Format ID DLC Frames Rate_Hz')
    for (identifier, extended, dlc), count in sorted(counts.items()):
        print(f'{"EXT" if extended else "STD"} {identifier:08X} {dlc} {count} {count / duration:.2f}')
    print('Excludes uncaptured errors/retries. Check adapter drops, CAN error counters and command gaps separately.')


if __name__ == '__main__':
    main()
