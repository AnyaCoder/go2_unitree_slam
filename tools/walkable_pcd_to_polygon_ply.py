#!/usr/bin/env python3
"""Convert a binary walkable-floor PCD into FAR boundary-handler polygons.

The output is the native FAR boundary PLY format: x y z poly_index.
Each contour becomes one closed polygon; repeated first vertex is omitted.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def load_xyz(path):
    with open(path, 'rb') as stream:
        header = {}
        while True:
            line = stream.readline().decode('ascii', 'replace').strip()
            if not line or line.startswith('#'):
                continue
            key, *values = line.split()
            header[key] = values
            if key == 'DATA':
                break
        if header.get('DATA', [''])[0] != 'binary':
            raise ValueError('only binary PCD is supported')
        fields = header['FIELDS']
        sizes = [int(value) for value in header['SIZE']]
        types = header['TYPE']
        counts = [int(value) for value in header['COUNT']]
        points = int(header['POINTS'][0])
        kind = {'F': 'f', 'I': 'i', 'U': 'u'}
        dtype = np.dtype([
            (name, kind[typ] + str(size), (count,))
            for name, size, typ, count in zip(fields, sizes, types, counts)
        ])
        table = np.frombuffer(stream.read(dtype.itemsize * points), dtype=dtype,
                              count=points)
    return np.column_stack((table['x'][:, 0], table['y'][:, 0],
                            table['z'][:, 0])).astype(np.float64)


def signed_area(points):
    x = points[:, 0]
    y = points[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def polygonize(points, resolution, close_cells, epsilon, min_area):
    x0 = np.floor(points[:, 0].min() / resolution) * resolution - 2 * resolution
    y0 = np.floor(points[:, 1].min() / resolution) * resolution - 2 * resolution
    ix = np.floor((points[:, 0] - x0) / resolution).astype(np.int32)
    iy = np.floor((points[:, 1] - y0) / resolution).astype(np.int32)
    nx = int(ix.max()) + 4
    ny = int(iy.max()) + 4
    mask = np.zeros((ny, nx), dtype=np.uint8)
    mask[iy, ix] = 255

    kernel = np.ones((close_cells, close_cells), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    keep = np.zeros(count, dtype=bool)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= int(round(min_area / (resolution ** 2)))
    mask = np.where(keep[labels], 255, 0).astype(np.uint8)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP,
                                           cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        raise RuntimeError('no free-space contour found')

    polygons = []
    for index, contour in enumerate(contours):
        contour = cv2.approxPolyDP(contour, epsilon / resolution, True)
        if len(contour) < 3:
            continue
        xy = contour.reshape(-1, 2).astype(np.float64)
        world = np.column_stack((x0 + (xy[:, 0] + 0.5) * resolution,
                                 y0 + (xy[:, 1] + 0.5) * resolution))
        area = signed_area(world)
        if abs(area) < min_area:
            continue
        if area > 0:
            world = world[::-1]
            area = -area
        polygons.append({
            'points': world,
            'area': abs(area),
            'outer': int(hierarchy[0, index, 3]) == -1,
        })
    polygons.sort(key=lambda item: item['area'], reverse=True)
    return mask, x0, y0, polygons


def farthest_free_point(mask, x0, y0, resolution, points):
    distance = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    row, col = np.unravel_index(int(np.argmax(distance)), distance.shape)
    center = np.array([x0 + (col + 0.5) * resolution,
                       y0 + (row + 0.5) * resolution])
    nearest = points[np.argmin(np.linalg.norm(points[:, :2] - center, axis=1))]
    return nearest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('walkable_pcd')
    parser.add_argument('boundary_ply')
    parser.add_argument('trajectory_txt')
    parser.add_argument('--resolution', type=float, default=0.10)
    parser.add_argument('--close-cells', type=int, default=3)
    parser.add_argument('--epsilon', type=float, default=0.08)
    parser.add_argument('--min-area', type=float, default=0.25)
    parser.add_argument('--summary-json', default='')
    args = parser.parse_args()

    points = load_xyz(args.walkable_pcd)
    mask, x0, y0, polygons = polygonize(
        points, args.resolution, args.close_cells, args.epsilon, args.min_area)
    if not polygons:
        raise RuntimeError('no polygon survived filtering')

    z = float(np.median(points[:, 2])) + 0.05
    rows = []
    for poly_index, polygon in enumerate(polygons):
        for x, y in polygon['points']:
            rows.append((x, y, z, poly_index))

    output = Path(args.boundary_ply)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', encoding='ascii') as stream:
        stream.write('ply\nformat ascii 1.0\n')
        stream.write('element vertex %d\n' % len(rows))
        stream.write('property float x\nproperty float y\nproperty float z\n')
        stream.write('property float poly_index\nend_header\n')
        for x, y, pz, poly_index in rows:
            stream.write('%.6f %.6f %.6f %d\n' % (x, y, pz, poly_index))

    free_point = farthest_free_point(mask, x0, y0, args.resolution, points)
    trajectory = Path(args.trajectory_txt)
    trajectory.parent.mkdir(parents=True, exist_ok=True)
    trajectory.write_text('%.6f %.6f %.6f\n' % tuple(free_point), encoding='ascii')

    summary = {
        'walkable_pcd': str(Path(args.walkable_pcd).resolve()),
        'boundary_ply': str(output.resolve()),
        'trajectory_txt': str(trajectory.resolve()),
        'vertices': len(rows),
        'polygons': len(polygons),
        'outer_polygons': sum(p['outer'] for p in polygons),
        'polygon_vertices': [len(p['points']) for p in polygons],
        'polygon_areas': [round(p['area'], 4) for p in polygons],
        'free_point': [round(float(v), 6) for v in free_point],
        'plane_z': round(z, 6),
    }
    if args.summary_json:
        Path(args.summary_json).write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
