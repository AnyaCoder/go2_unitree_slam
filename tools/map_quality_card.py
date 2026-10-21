#!/usr/bin/env python3
"""给一张点云地图打"质量卡"，判断它离仿真级（Unity/Gazebo）有多远。

    ./map_quality_card.py GlobalMap.pcd [--band -0.3 1.2] [--sub 200000]

输出 6 个指标，每个都直接对应规划链上的一个阈值（见 --help 末尾）：

  1. spacing p50/p90   最近邻间距 —— 决定多远之后墙面开始"断层"
  2. thickness p50/p90 局部平面拟合厚度(RMS) —— 决定障碍边界有多模糊
  3. thin ratio        thickness > 5 cm 的点占比 —— 重影/拖尾
  4. fill3x3 @5cm      5 cm 栅格里 8 邻域被占满的比例 —— 决定 terrain_analysis 的空地是否连续
  5. pinhole @5cm      被邻居包围的空洞比例 —— 决定栅格图上会不会出现假缝隙
  6. outlier SOR/ROR   统计离群点比例 —— 决定 RViz 点击会不会点到空气

只用 numpy+scipy，百万点约 1 分钟。
"""
import argparse
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
                raise RuntimeError('不是 binary PCD: %s' % path)
            hdr += c
        fields = sizes = None
        n = 0
        for line in hdr.decode(errors='replace').splitlines():
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('SIZE'):
                sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith('POINTS'):
                n = int(line.split()[1])
        if fields[:3] != ['x', 'y', 'z']:
            raise RuntimeError('不支持的字段: %s' % fields)
        step = sum(sizes)
        raw = f.read(n * step)
    arr = np.frombuffer(raw, dtype=np.uint8).reshape(n, step)
    pts = np.empty((n, 3), dtype=np.float32)
    for i in range(3):
        pts[:, i] = arr[:, 4 * i:4 * i + 4].copy().view(np.float32).ravel()
    return pts


def read_ply(path):
    """只读 binary_little_endian PLY 的 x/y/z，其它属性按步长跳过。"""
    sizes = {'char': 1, 'uchar': 1, 'int8': 1, 'uint8': 1,
             'short': 2, 'ushort': 2, 'int16': 2, 'uint16': 2,
             'int': 4, 'uint': 4, 'int32': 4, 'uint32': 4, 'float': 4, 'float32': 4,
             'double': 8, 'float64': 8}
    with open(path, 'rb') as f:
        hdr = b''
        while not hdr.endswith(b'end_header\n'):
            c = f.read(1)
            if not c:
                raise RuntimeError('bad ply header')
            hdr += c
        lines = hdr.decode(errors='replace').splitlines()
        if not any('binary_little_endian' in l for l in lines):
            raise RuntimeError('只支持 binary_little_endian PLY')
        n, props, in_v = 0, [], False
        for l in lines:
            t = l.split()
            if not t:
                continue
            if t[0] == 'element':
                in_v = t[1] == 'vertex'
                if in_v:
                    n = int(t[2])
            elif t[0] == 'property' and in_v:
                props.append((t[2], sizes[t[1]]))
        off, step, xyz = 0, 0, []
        for name, sz in props:
            if name in ('x', 'y', 'z'):
                xyz.append((off, sz))
            off += sz
            step += sz
        raw = np.frombuffer(f.read(n * step), dtype=np.uint8).reshape(n, step)
    pts = np.empty((n, 3), dtype=np.float32)
    for i, (o, sz) in enumerate(sorted(xyz, key=lambda t: t[0])):
        pts[:, i] = raw[:, o:o + sz].copy().view(np.float32 if sz == 4 else np.float64).ravel()
    return pts


def load_any(path):
    return read_ply(path) if path.lower().endswith('.ply') else read_pcd(path)


def spacing(pts, sample):
    tree = cKDTree(pts)
    d, _ = tree.query(pts[sample], k=2, workers=-1)
    nn = d[:, 1]
    return tree, nn


def thickness(pts, sample, k=16):
    tree = cKDTree(pts)
    nb = pts[tree.query(pts[sample], k=k, workers=-1)[1]]        # (n,k,3)
    nb = nb - nb.mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', nb, nb) / k
    lam = np.linalg.eigvalsh(cov)                                 # 升序
    sig = np.sqrt(np.clip(lam[:, 0], 0, None))                    # 最小方向标准差 = 面厚
    return sig


def floor_z(pts):
    return float(np.percentile(pts[:, 2], 2))


def grid_stats(pts, res=0.05, band=None, ring=1):
    if band is None:
        g = floor_z(pts)
        band = (g - 0.10, g + 1.20)
    sel = pts[(pts[:, 2] > band[0]) & (pts[:, 2] < band[1])]
    ij = np.floor(sel[:, :2] / res).astype(np.int64)
    ij -= ij.min(axis=0)
    shape = ij.max(axis=0) + 1
    occ = np.zeros(shape, dtype=bool)
    occ[ij[:, 0], ij[:, 1]] = True
    p = np.pad(occ, ring)
    nb = np.zeros(shape, dtype=np.int16)
    for dx in range(-ring, ring + 1):
        for dy in range(-ring, ring + 1):
            if dx == 0 and dy == 0:
                continue
            nb += p[ring + dx:ring + dx + shape[0], ring + dy:ring + dy + shape[1]]
    full = (nb == 8)[occ]
    hole = (nb >= 7)[~occ]
    return (full.mean(), hole.sum() / max(occ.sum(), 1), occ.sum(),
            float(shape[0] * shape[1]))


def ground_fit(pts):
    z = pts[:, 2]
    thr = np.percentile(z, 10)
    low = pts[z < thr]
    A = np.c_[low[:, 0], low[:, 1], np.ones(len(low))]
    coef, *_ = np.linalg.lstsq(A, low[:, 2], rcond=None)
    res = A @ coef - low[:, 2]
    tilt = np.degrees(np.arctan(np.hypot(coef[0], coef[1])))
    return float(np.sqrt((res ** 2).mean())), tilt


def outliers(pts, sub, tree=None):
    tree = tree or cKDTree(pts)
    s = pts[sub]
    d, _ = tree.query(s, k=21, workers=-1)
    md = d[:, 1:].mean(axis=1)
    sor = float((md > md.mean() + md.std()).mean())
    cnt = tree.query_ball_point(s, 0.10, return_length=True, workers=-1)
    ror = float((cnt < 2).mean())
    return sor, ror


def nav_readiness(pts, res=0.05, robot_radius=0.25, origin=(0.0, 0.0)):
    """把地图当成规划器看到的样子：5cm 占用栅格 + 机器人半径膨胀，看空地连不连。"""
    from scipy.ndimage import label, binary_dilation
    floor = floor_z(pts)
    sel = pts[(pts[:, 2] > floor + 0.10) & (pts[:, 2] < floor + 1.20)]
    x0, y0 = float(pts[:, 0].min()), float(pts[:, 1].min())
    w = int((pts[:, 0].max() - x0) / res) + 1
    h = int((pts[:, 1].max() - y0) / res) + 1
    cnt = np.zeros((h, w), np.int32)
    ix = np.clip(((sel[:, 0] - x0) / res).astype(np.int32), 0, w - 1)
    iy = np.clip(((sel[:, 1] - y0) / res).astype(np.int32), 0, h - 1)
    np.add.at(cnt, (iy, ix), 1)
    occ = cnt >= 2
    r = max(1, int(round(robot_radius / res)))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    disk = (xx * xx + yy * yy) <= r * r
    occ = binary_dilation(occ, disk)
    free = ~occ
    lab, n = label(free)
    sizes = np.bincount(lab.ravel())[1:]
    areas = np.sort(sizes) * res * res
    oi = int((origin[1] - y0) / res), int((origin[0] - x0) / res)
    l0 = lab[oi] if 0 <= oi[0] < h and 0 <= oi[1] < w else 0
    start_area = float(sizes[l0 - 1] * res * res) if l0 > 0 else 0.0
    big = [a for a in areas[::-1] if a >= 1.0][:5]
    return start_area, big, float(occ.mean()), int(n)


def fmt(x, unit=''):
    return ('%.1f%s' % (1000 * x, 'mm')) if x < 1 else ('%.2f%s' % (x, unit))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pcd', nargs='+')
    ap.add_argument('--band', nargs=2, type=float, default=None,
                    help='机器人关心的 z 带；默认从地面往上 0~1.2 m 自动取')
    ap.add_argument('--sub', type=int, default=200000, help='统计采样点数')
    ap.add_argument('--crop', nargs=3, type=float, metavar=('X', 'Y', 'SIZE'),
                    help='只统计以 (X,Y) 为中心、边长 SIZE 的方框（比不同大小的场景时用）')
    a = ap.parse_args()

    rng = np.random.default_rng(0)
    rows = []
    for path in a.pcd:
        t0 = time.time()
        pts = load_any(path)
        if a.crop:
            cx, cy, sz = a.crop
            m = ((np.abs(pts[:, 0] - cx) < sz / 2) & (np.abs(pts[:, 1] - cy) < sz / 2))
            pts = pts[m]
        sub = rng.choice(len(pts), size=min(a.sub, len(pts)), replace=False)
        tree, nn = spacing(pts, sub)
        sig = thickness(pts, sub)
        band = None if a.band is None else tuple(a.band)
        fill, hole, occ, cells = grid_stats(pts, band=band)
        g_rms, g_tilt = ground_fit(pts[sub])
        sor, ror = outliers(pts, sub, tree)
        rows.append(dict(name=path.split('/')[-1], n=len(pts),
                         nn50=np.percentile(nn, 50), nn90=np.percentile(nn, 90),
                         sig50=np.percentile(sig, 50), sig90=np.percentile(sig, 90),
                         thin=float((sig > 0.05).mean()),
                         fill=fill, hole=hole, occ=int(occ), cells=int(cells),
                         g_rms=g_rms, g_tilt=g_tilt, sor=sor, ror=ror,
                         sec=time.time() - t0))
    for r in rows:
        print('\n== %s  %d 点  (%.1fs)' % (r['name'], r['n'], r['sec']))
        print('  1 间距      p50 %-8s p90 %s' % (fmt(r['nn50']), fmt(r['nn90'])))
        print('  2 面厚      p50 %-8s p90 %s   (>5cm 占比 %.1f%%)'
              % (fmt(r['sig50']), fmt(r['sig90']), 100 * r['thin']))
        print('  3 栅格@5cm  已占 %d / %d 格   3x3 填满率 %.2f   针孔率 %.2f%%'
              % (r['occ'], r['cells'], r['fill'], 100 * r['hole']))
        print('  4 地面      RMS %-8s 倾角 %.2f°'
              % (fmt(r['g_rms']), r['g_tilt']))
        print('  5 离群      SOR %.2f%%   ROR %.2f%%' % (100 * r['sor'], 100 * r['ror']))
        sa, big, occr, ncomp = nav_readiness(pts)
        print('  6 可导航    起点所在空地 %.1f m²   障碍占比 %.1f%%   自由块 %d 个'
              % (sa, 100 * occr, ncomp))
        print('              最大空地块(m²): %s' % ', '.join('%.1f' % a for a in big))
    return 0


if __name__ == '__main__':
    sys.exit(main())
