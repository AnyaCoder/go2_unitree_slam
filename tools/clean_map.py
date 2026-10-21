#!/usr/bin/env python3
"""Clean a LIO-SAM GlobalMap.pcd: voxel downsample + SOR + ROR + height clip.

Removes the classic "points floating in the air" (dynamic objects, self
returns, bad loop-closure ghosts) that make RViz clicks land on nothing.

usage:
  ./clean_map.py in.pcd out.pcd [--voxel 0.03] [--sor-k 20] [--sor-std 1.0]
                 [--ror-r 0.15] [--ror-min 2] [--zmin -0.6] [--zmax 2.2]
                 [--clip-origin 0.5]
"""
import argparse
import struct
import sys
import time

import numpy as np
from scipy.spatial import cKDTree


def read_pcd(path):
    with open(path, 'rb') as f:
        hdr = b''
        while not hdr.endswith(b'DATA binary\n'):
            c = f.read(1)
            if not c:
                raise RuntimeError('not a binary pcd')
            hdr += c
        fields, sizes = None, None
        n = 0
        for line in hdr.decode(errors='replace').splitlines():
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('SIZE'):
                sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith('POINTS'):
                n = int(line.split()[1])
        if fields[:3] != ['x', 'y', 'z']:
            raise RuntimeError('unsupported fields: %s' % fields)
        step = sum(sizes)
        raw = f.read(n * step)
    arr = np.frombuffer(raw, dtype=np.uint8).reshape(n, step)
    pts = np.empty((n, 3), dtype=np.float32)
    off = 0
    for i in range(3):
        pts[:, i] = arr[:, off:off + 4].copy().view(np.float32).ravel()
        off += 4
    return pts


def write_pcd(path, pts):
    n = len(pts)
    hdr = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
           "COUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
           "POINTS %d\nDATA binary\n" % (n, n))
    with open(path, 'wb') as f:
        f.write(hdr.encode())
        f.write(np.ascontiguousarray(pts, dtype=np.float32).tobytes())


def voxel(pts, size):
    keys = np.floor(pts / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def sor(pts, k, std_ratio, chunk=200000):
    tree = cKDTree(pts)
    keep = np.ones(len(pts), dtype=bool)
    dists = np.empty(len(pts), dtype=np.float32)
    for i in range(0, len(pts), chunk):
        d, _ = tree.query(pts[i:i + chunk], k=k + 1, workers=-1)
        dists[i:i + chunk] = d[:, 1:].mean(axis=1)
    mu, sd = dists.mean(), dists.std()
    keep = dists < mu + std_ratio * sd
    return pts[keep], keep, (mu, sd)


def ror(pts, r, min_nb, chunk=200000):
    tree = cKDTree(pts)
    keep = np.ones(len(pts), dtype=bool)
    for i in range(0, len(pts), chunk):
        cnt = tree.query_ball_point(pts[i:i + chunk], r, return_length=True, workers=-1)
        keep[i:i + chunk] = cnt >= min_nb + 1
    return pts[keep], keep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--voxel', type=float, default=0.03)
    ap.add_argument('--sor-k', type=int, default=20)
    ap.add_argument('--sor-std', type=float, default=1.0)
    ap.add_argument('--ror-r', type=float, default=0.15)
    ap.add_argument('--ror-min', type=int, default=2)
    ap.add_argument('--clip-origin', type=float, default=0.0,
                    help='删掉原点 xy 半径 R 内的点：清掉建图时"狗自己/旁边的人"打在起点上的点')
    ap.add_argument('--zmin', type=float, default=-0.6)
    ap.add_argument('--zmax', type=float, default=2.2)
    a = ap.parse_args()

    t0 = time.time()
    pts = read_pcd(a.src)
    print('input      : %d' % len(pts))

    m = (pts[:, 2] > a.zmin) & (pts[:, 2] < a.zmax)
    pts = pts[m]
    print('height clip: %d  (kept z in %.2f..%.2f)' % (len(pts), a.zmin, a.zmax))

    if a.clip_origin > 0:
        d = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[d >= a.clip_origin]
        print('clip origin: %d  (r >= %.2f m)' % (len(pts), a.clip_origin))

    if a.voxel > 0:
        pts = voxel(pts, a.voxel)
        print('voxel %.3f : %d' % (a.voxel, len(pts)))

    if a.sor_k > 0:
        pts, keep, (mu, sd) = sor(pts, a.sor_k, a.sor_std)
        print('SOR        : %d kept (mean nb dist %.4f m, sd %.4f)' % (len(pts), mu, sd))

    if a.ror_r > 0:
        pts, keep = ror(pts, a.ror_r, a.ror_min)
        print('ROR        : %d kept (r=%.2f m, min nb=%d)' % (len(pts), a.ror_r, a.ror_min))

    write_pcd(a.dst, pts)
    print('output     : %d -> %s  (%.1f s)' % (len(pts), a.dst, time.time() - t0))


if __name__ == '__main__':
    main()
