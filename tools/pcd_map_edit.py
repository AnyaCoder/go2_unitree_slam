#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Exchange a PCD map with a 2-D ROS PGM/YAML editor.

Workflow:
  1. export  PCD -> map.pgm + map.yaml
  2. edit map.pgm/map.yaml in the browser editor
  3. apply   edited map -> edited PCD

The apply step uses this rule:
  FREE    : delete every point above the floor band in that grid cell
            (walls, furniture, people, ceiling ...)
  BLOCKED : keep existing points; with --add-blocked, also insert a sparse
            vertical marker column where no obstacle points existed
  UNKNOWN : keep the original points

This edited PCD is for viewing / goal selection only.  Do not use it for
LIO-SAM relocalization; relocalization must use the original GlobalMap.pcd.

usage:
  ./pcd_map_edit.py export GlobalMap_clean.pcd -o map_edit
  ./pcd_map_edit.py apply  GlobalMap_clean.pcd \
      --yaml map_edit/map.yaml --out GlobalMap_edited.pcd
"""
import argparse
import io
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_quality_card import floor_z

UNKNOWN, FREE, BLOCKED = 0, 1, 2


def read_pcd_xyzi(path):
    with open(path, 'rb') as f:
        header = b''
        while not header.endswith(b'DATA binary\n'):
            line = f.readline()
            if not line:
                raise RuntimeError('PCD 缺少 DATA binary 行: %s' % path)
            header += line
        lines = header.decode('ascii', 'ignore').splitlines()
        fields = sizes = types = counts = None
        n = 0
        for line in lines:
            t = line.split()
            if not t:
                continue
            if t[0] == 'FIELDS':
                fields = t[1:]
            elif t[0] == 'SIZE':
                sizes = [int(v) for v in t[1:]]
            elif t[0] == 'TYPE':
                types = t[1:]
            elif t[0] == 'COUNT':
                counts = [int(v) for v in t[1:]]
            elif t[0] == 'POINTS':
                n = int(t[1])
        if not fields or n <= 0:
            raise RuntimeError('PCD 头不完整: %s' % path)
        if counts is None:
            counts = [1] * len(fields)
        dtype = []
        for name, size, typ, count in zip(fields, sizes, types, counts):
            code = {'F': 'f', 'U': 'u', 'I': 'i', 'C': 'i'}[typ] + str(size)
            dtype.append((name, '<' + code, (count,)) if count > 1 else
                         (name, '<' + code))
        raw = f.read()
    arr = np.frombuffer(raw, dtype=np.dtype(dtype), count=n)
    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float32)
    if 'intensity' in fields:
        inten = np.asarray(arr['intensity'], dtype=np.float32)
    else:
        inten = np.zeros(n, dtype=np.float32)
    return xyz, inten


def write_pcd_xyzrgb(path, xyz, rgb):
    xyz = np.ascontiguousarray(xyz, dtype=np.float32)
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    n = len(xyz)
    r = rgb[:, 0].astype(np.uint32)
    g = rgb[:, 1].astype(np.uint32)
    b = rgb[:, 2].astype(np.uint32)
    packed = ((r << 16) | (g << 8) | b).astype(np.uint32).view(np.float32)
    data = np.empty((n, 4), dtype=np.float32)
    data[:, :3] = xyz
    data[:, 3] = packed
    header = ("# .PCD v0.7\nVERSION 0.7\n"
              "FIELDS x y z rgb\nSIZE 4 4 4 4\n"
              "TYPE F F F F\nCOUNT 1 1 1 1\n"
              "WIDTH %d\nHEIGHT 1\n"
              "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n" % (n, n))
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(data.tobytes())


def write_pcd_xyzi(path, xyz, inten=None):
    xyz = np.ascontiguousarray(xyz, dtype=np.float32)
    if inten is None:
        inten = np.zeros(len(xyz), dtype=np.float32)
    inten = np.ascontiguousarray(inten, dtype=np.float32)
    n = len(xyz)
    data = np.empty((n, 4), dtype=np.float32)
    data[:, :3] = xyz
    data[:, 3] = inten
    header = ("# .PCD v0.7\nVERSION 0.7\n"
              "FIELDS x y z intensity\nSIZE 4 4 4 4\n"
              "TYPE F F F F\nCOUNT 1 1 1 1\n"
              "WIDTH %d\nHEIGHT 1\n"
              "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n" % (n, n))
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(data.tobytes())


def _next_token(data, pos):
    n = len(data)
    while pos < n:
        while pos < n and data[pos:pos + 1].isspace():
            pos += 1
        if pos < n and data[pos:pos + 1] == b'#':
            while pos < n and data[pos:pos + 1] not in (b'\n', b'\r'):
                pos += 1
            continue
        break
    start = pos
    while pos < n and not data[pos:pos + 1].isspace():
        pos += 1
    return data[start:pos], pos


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    pos = 0
    toks = []
    while len(toks) < 4:
        tok, pos = _next_token(data, pos)
        if not tok:
            raise RuntimeError('PGM 头不完整: %s' % path)
        toks.append(tok)
    magic, w, h, maxv = toks[0], int(toks[1]), int(toks[2]), int(toks[3])
    while pos < len(data) and data[pos:pos + 1].isspace():
        pos += 1
    if magic == b'P5':
        if maxv < 256:
            arr = np.frombuffer(data[pos:pos + w * h], dtype=np.uint8).reshape(h, w).copy()
        else:
            arr = np.frombuffer(data[pos:pos + w * h * 2], dtype='>u2').reshape(h, w).copy()
            arr = (arr.astype(np.float32) * 255.0 / maxv).round().astype(np.uint8)
    elif magic == b'P2':
        vals = np.fromstring(data[pos:].decode('ascii', 'ignore'), sep=' ', dtype=np.int32)
        arr = (vals[:w * h].reshape(h, w).astype(np.float32) * 255.0 / maxv).round().astype(np.uint8)
    else:
        raise RuntimeError('只支持 P2/P5 PGM: %s' % path)
    return arr


def write_pgm(path, pixels):
    pixels = np.ascontiguousarray(pixels, dtype=np.uint8)
    h, w = pixels.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(pixels.tobytes())


def read_yaml(path):
    try:
        import yaml
        with open(path) as f:
            return yaml.safe_load(f)
    except Exception:
        vals = {}
        with open(path) as f:
            for line in f:
                if ':' not in line:
                    continue
                k, v = line.split(':', 1)
                k, v = k.strip(), v.strip()
                if k == 'resolution':
                    vals[k] = float(v)
                elif k == 'origin':
                    vals[k] = [float(x) for x in v.strip('[]').split(',')]
                elif k == 'image':
                    vals[k] = v.strip().strip('"').strip("'")
        return vals


def write_yaml(path, image, res, x0, y0):
    text = ("image: %s\nresolution: %.9f\n"
            "origin: [%.9f, %.9f, 0.0]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n"
            % (image, res, x0, y0))
    with open(path, 'w') as f:
        f.write(text)


def grid_shape(pts, res):
    x0, y0 = float(pts[:, 0].min()), float(pts[:, 1].min())
    w = int((pts[:, 0].max() - x0) / res) + 1
    h = int((pts[:, 1].max() - y0) / res) + 1
    return x0, y0, w, h


def count_cells(pts, mask, res, x0, y0, w, h):
    g = np.zeros((h, w), np.int32)
    q = pts[mask]
    ix = np.clip(((q[:, 0] - x0) / res).astype(np.int32), 0, w - 1)
    iy = np.clip(((q[:, 1] - y0) / res).astype(np.int32), 0, h - 1)
    np.add.at(g, (iy, ix), 1)
    return g


def build_state(pts, res, floor_band, zmax):
    floor = float(floor_z(pts))
    x0, y0, w, h = grid_shape(pts, res)
    floor_cnt = count_cells(pts, (pts[:, 2] > floor - floor_band) &
                            (pts[:, 2] < floor + floor_band), res, x0, y0, w, h)
    obs_cnt = count_cells(pts, (pts[:, 2] >= floor + floor_band) &
                          (pts[:, 2] < floor + zmax), res, x0, y0, w, h)
    state = np.zeros((h, w), dtype=np.int8)
    state[floor_cnt >= 1] = FREE
    state[obs_cnt >= 1] = BLOCKED
    return state, dict(res=float(res), x0=x0, y0=y0, w=int(w), h=int(h),
                       floor=floor, floor_band=float(floor_band), zmax=float(zmax))


def state_to_pgm(state):
    # ROS map coordinates: image row 0 is the top (= max y).
    img = np.full(state.shape, 205, dtype=np.uint8)  # unknown
    img[state == FREE] = 254
    img[state == BLOCKED] = 0
    return np.flipud(img)


def pgm_to_state(pgm, yaml):
    occ = (255.0 - pgm.astype(np.float32)) / 255.0
    if float(yaml.get('negate', 0) or 0):
        occ = 1.0 - occ
    free_th = float(yaml.get('free_thresh', 0.196))
    occ_th = float(yaml.get('occupied_thresh', 0.65))
    state = np.full(pgm.shape, UNKNOWN, dtype=np.int8)
    state[occ < free_th] = FREE
    state[occ > occ_th] = BLOCKED
    return np.flipud(state)


def save_preview(path, state, meta, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rgb = np.ones(state.shape + (3,), dtype=float) * 0.96
    rgb[state == FREE] = [0.20, 0.72, 0.32]
    rgb[state == BLOCKED] = [0.16, 0.16, 0.16]
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(rgb, origin='lower', interpolation='nearest',
              extent=[meta['x0'], meta['x0'] + meta['w'] * meta['res'],
                      meta['y0'], meta['y0'] + meta['h'] * meta['res']])
    ax.plot(0, 0, 'r*', ms=16)
    ax.set_aspect('equal')
    ax.grid(color='0.75', lw=0.35, alpha=0.5)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def cmd_export(a):
    pts, _ = read_pcd_xyzi(a.pcd)
    state, meta = build_state(pts, a.res, a.floor_band, a.zmax)
    os.makedirs(a.out, exist_ok=True)
    pgm = os.path.join(a.out, 'map.pgm')
    yml = os.path.join(a.out, 'map.yaml')
    write_pgm(pgm, state_to_pgm(state))
    write_yaml(yml, 'map.pgm', meta['res'], meta['x0'], meta['y0'])
    with open(os.path.join(a.out, 'map_meta.json'), 'w') as f:
        json.dump(dict(source=os.path.abspath(a.pcd), **meta), f, indent=2)
    save_preview(os.path.join(a.out, 'map_preview.png'), state, meta,
                 'auto: free %.1f m2' % (int((state == FREE).sum()) * meta['res'] ** 2))
    print('exported %s' % pgm)
    print('exported %s' % yml)
    print('free %.1f m2, blocked %d cells, unknown %d cells' %
          (int((state == FREE).sum()) * meta['res'] ** 2,
           int((state == BLOCKED).sum()), int((state == UNKNOWN).sum())))
    print('open https://gyropalm.github.io/ROS-SLAM-Map-Editor/editor.html')
    print('drag both files into the browser, edit, then download map_edited.*')


def cmd_apply(a):
    pts, inten = read_pcd_xyzi(a.pcd)
    yml = read_yaml(a.yaml)
    pgm_path = a.pgm or os.path.join(os.path.dirname(a.yaml), yml.get('image', 'map.pgm'))
    pgm = read_pgm(pgm_path)
    state = pgm_to_state(pgm, yml)
    res = float(yml['resolution'])
    x0, y0 = float(yml['origin'][0]), float(yml['origin'][1])
    h, w = state.shape
    floor = float(floor_z(pts))
    band = float(a.floor_band)
    obs_top = float(a.zmax)

    ix = np.floor((pts[:, 0] - x0) / res).astype(np.int32)
    iy = np.floor((pts[:, 1] - y0) / res).astype(np.int32)
    inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    cell_state = np.full(len(pts), UNKNOWN, dtype=np.int8)
    cell_state[inside] = state[iy[inside], ix[inside]]

    # FREE means: clear the entire vertical column above the floor band.
    above_floor = pts[:, 2] > floor + band
    remove = (cell_state == FREE) & above_floor
    keep = ~remove

    # For BLOCKED cells, add a sparse marker column where the map had no obstacle.
    add_xyz = []
    add_int = []
    if a.add_blocked:
        obs_cnt = count_cells(pts, (pts[:, 2] >= floor + band) &
                              (pts[:, 2] < floor + obs_top), res, x0, y0, w, h)
        add_y, add_x = np.nonzero((state == BLOCKED) & (obs_cnt < 2))
        zs = floor + np.arange(0.25, max(0.26, a.block_height), 0.25)
        for yy, xx in zip(add_y, add_x):
            wx = x0 + (xx + 0.5) * res
            wy = y0 + (yy + 0.5) * res
            for zz in zs:
                add_xyz.append((wx, wy, float(zz)))
                add_int.append(-1.0)

    out_xyz = pts[keep]
    out_int = inten[keep]
    if add_xyz:
        out_xyz = np.vstack([out_xyz, np.asarray(add_xyz, dtype=np.float32)])
        out_int = np.concatenate([out_int, np.asarray(add_int, dtype=np.float32)])

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    write_pcd_xyzi(a.out, out_xyz, out_int)
    stats = dict(source=os.path.abspath(a.pcd), yaml=os.path.abspath(a.yaml),
                 pgm=os.path.abspath(pgm_path), output=os.path.abspath(a.out),
                 original_points=len(pts), removed_points=int(remove.sum()),
                 added_marker_points=len(add_xyz), final_points=len(out_xyz),
                 free_cells=int((state == FREE).sum()),
                 blocked_cells=int((state == BLOCKED).sum()),
                 unknown_cells=int((state == UNKNOWN).sum()))
    with open(os.path.splitext(a.out)[0] + '_apply.json', 'w') as f:
        json.dump(stats, f, indent=2)
    print('wrote %s' % a.out)
    print('removed %d points; added %d marker points; final %d points' %
          (remove.sum(), len(add_xyz), len(out_xyz)))


def cmd_color_ground(a):
    xyz, _ = read_pcd_xyzi(a.pcd)
    yml = read_yaml(a.yaml)
    pgm_path = a.pgm or os.path.join(os.path.dirname(a.yaml), yml.get('image', 'map.pgm'))
    state = pgm_to_state(read_pgm(pgm_path), yml)
    res = float(yml['resolution'])
    x0, y0 = float(yml['origin'][0]), float(yml['origin'][1])
    h, w = state.shape
    floor = float(floor_z(xyz))
    ix = np.floor((xyz[:, 0] - x0) / res).astype(np.int32)
    iy = np.floor((xyz[:, 1] - y0) / res).astype(np.int32)
    inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    free = np.zeros(len(xyz), dtype=bool)
    free[inside] = state[iy[inside], ix[inside]] == FREE
    ground = xyz[:, 2] <= floor + a.floor_band
    walkable = free & ground
    rgb = np.full((len(xyz), 3), 170, dtype=np.uint8)
    rgb[walkable] = [0, 255, 0]
    green_xyz = xyz[walkable].copy()

    # Sparse lidar floor returns are hard to see in 3D.  Add one green point
    # per FREE 2-D cell; this is a display layer, never a localization map.
    added = 0
    if a.fill:
        fy, fx = np.nonzero(state == FREE)
        cell_min = np.full((h, w), np.inf, dtype=np.float32)
        np.minimum.at(cell_min, (iy[inside], ix[inside]), xyz[inside, 2])
        zz = cell_min[fy, fx]
        zz = np.where(np.isfinite(zz), zz + 0.02, floor + 0.02)
        add_xyz = np.column_stack([
            x0 + (fx + 0.5) * res,
            y0 + (fy + 0.5) * res,
            zz]).astype(np.float32)
        xyz = np.vstack([xyz, add_xyz])
        rgb = np.vstack([rgb, np.tile(np.array([[0, 255, 0]], dtype=np.uint8),
                                      (len(add_xyz), 1))])
        green_xyz = np.vstack([green_xyz, add_xyz])
        added = len(add_xyz)

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    write_pcd_xyzrgb(a.out, xyz, rgb)
    if a.walkable_out:
        os.makedirs(os.path.dirname(os.path.abspath(a.walkable_out)), exist_ok=True)
        write_pcd_xyzi(a.walkable_out, green_xyz)
        print('wrote %s (%d green points)' % (a.walkable_out, len(green_xyz)))
    print('wrote %s' % a.out)
    print('green real ground: %d, added green fill: %d, total: %d' %
          (walkable.sum(), added, len(xyz)))


def cmd_crop_z(a):
    xyz, inten = read_pcd_xyzi(a.pcd)
    keep = np.ones(len(xyz), dtype=bool)
    if a.zmin is not None:
        keep &= xyz[:, 2] >= a.zmin
    if a.zmax is not None:
        keep &= xyz[:, 2] <= a.zmax
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    write_pcd_xyzi(a.out, xyz[keep], inten[keep])
    print('wrote %s' % a.out)
    print('kept %d / %d points (z %.3f .. %.3f)' %
          (keep.sum(), len(xyz), xyz[keep, 2].min() if keep.any() else 0,
           xyz[keep, 2].max() if keep.any() else 0))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    e = sub.add_parser('export', help='PCD -> map.pgm + map.yaml')
    e.add_argument('pcd')
    e.add_argument('-o', '--out', default='map_edit')
    e.add_argument('--res', type=float, default=0.05)
    e.add_argument('--floor-band', type=float, default=0.12)
    e.add_argument('--zmax', type=float, default=1.20)
    e.set_defaults(func=cmd_export)

    a = sub.add_parser('apply', help='edited map -> edited PCD')
    a.add_argument('pcd')
    a.add_argument('--yaml', required=True)
    a.add_argument('--pgm', default=None)
    a.add_argument('--out', default='GlobalMap_edited.pcd')
    a.add_argument('--floor-band', type=float, default=0.12)
    a.add_argument('--zmax', type=float, default=1.20)
    a.add_argument('--add-blocked', action='store_true',
                   help='insert synthetic vertical marker points for blocked cells')
    a.add_argument('--block-height', type=float, default=1.20)
    a.set_defaults(func=cmd_apply)

    g = sub.add_parser('color-ground', help='paint FREE-cell ground points green')
    g.add_argument('pcd')
    g.add_argument('--yaml', required=True)
    g.add_argument('--pgm', default=None)
    g.add_argument('--out', required=True)
    g.add_argument('--walkable-out', default=None,
                   help='optional XYZ PCD containing only the green floor points')
    g.add_argument('--floor-band', type=float, default=0.18)
    g.add_argument('--no-fill', dest='fill', action='store_false',
                   help='do not add the 5 cm green floor visualization grid')
    g.set_defaults(func=cmd_color_ground)

    c = sub.add_parser('crop-z', help='remove points outside a z range (e.g. ceiling)')
    c.add_argument('pcd')
    c.add_argument('-o', '--out', required=True)
    c.add_argument('--zmin', type=float, default=None)
    c.add_argument('--zmax', type=float, default=None)
    c.set_defaults(func=cmd_crop_z)

    args = ap.parse_args()
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
