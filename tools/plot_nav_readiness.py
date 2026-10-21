#!/usr/bin/env python3
"""把地图画成"规划器视角"：左=全屋 5cm 占用栅格（按连通块着色），右=起点附近放大。

    ./plot_nav_readiness.py GlobalMap_clean.pcd out.png [--radius 0.25]
"""
import argparse
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import label, binary_dilation

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from map_quality_card import read_pcd, floor_z


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd')
    ap.add_argument('out')
    ap.add_argument('--radius', type=float, default=0.25)
    ap.add_argument('--res', type=float, default=0.05)
    a = ap.parse_args()

    pts = read_pcd(a.pcd)
    fl = floor_z(pts)
    sel = pts[(pts[:, 2] > fl + 0.10) & (pts[:, 2] < fl + 1.20)]
    x0, y0 = pts[:, 0].min(), pts[:, 1].min()
    w = int((pts[:, 0].max() - x0) / a.res) + 1
    h = int((pts[:, 1].max() - y0) / a.res) + 1
    cnt = np.zeros((h, w), np.int32)
    ix = np.clip(((sel[:, 0] - x0) / a.res).astype(int), 0, w - 1)
    iy = np.clip(((sel[:, 1] - y0) / a.res).astype(int), 0, h - 1)
    np.add.at(cnt, (iy, ix), 1)
    occ = cnt >= 2
    r = max(1, int(round(a.radius / a.res)))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    occ_i = binary_dilation(occ, (xx * xx + yy * yy) <= r * r)
    lab, n = label(~occ_i)

    fig, ax = plt.subplots(1, 2, figsize=(16, 7))
    ax[0].imshow(lab, origin='lower', cmap='tab20', interpolation='nearest',
                 extent=[x0, x0 + w * a.res, y0, y0 + h * a.res])
    ax[0].imshow(np.ma.masked_where(~occ_i, occ_i), origin='lower', cmap='gray_r',
                 alpha=0.35, interpolation='nearest',
                 extent=[x0, x0 + w * a.res, y0, y0 + h * a.res])
    ax[0].plot(0, 0, 'r*', ms=18, label='map origin / dog start')
    ax[0].set_title('%s  free blocks=%d  r=%.2fm' % (a.pcd.split('/')[-1], n, a.radius))
    ax[0].legend(loc='upper right')
    ax[0].set_xlabel('x [m]'); ax[0].set_ylabel('y [m]')

    m = 2.0
    c = sel[np.hypot(sel[:, 0], sel[:, 1]) < m]
    sc = ax[1].scatter(c[:, 0], c[:, 1], c=c[:, 2] - fl, s=3, cmap='jet')
    plt.colorbar(sc, ax=ax[1], label='height above floor [m]')
    for rad, col in ((0.20, 'lime'), (0.35, 'orange'), (0.6, 'red')):
        ax[1].add_patch(plt.Circle((0, 0), rad, fill=False, color=col, lw=1.5, ls='--'))
    ax[1].plot(0, 0, 'r*', ms=15)
    ax[1].set_xlim(-m, m); ax[1].set_ylim(-m, m); ax[1].set_aspect('equal')
    ax[1].grid(alpha=0.3)
    ax[1].set_title('origin zoom: %.0f pts within 0.25m, %.0f within 0.35m'
                    % ((np.hypot(c[:, 0], c[:, 1]) < .25).sum(),
                       (np.hypot(c[:, 0], c[:, 1]) < .35).sum()))
    ax[1].set_xlabel('x [m]'); ax[1].set_ylabel('y [m]')
    fig.tight_layout()
    fig.savefig(a.out, dpi=110)
    print('saved', a.out)


if __name__ == '__main__':
    main()
