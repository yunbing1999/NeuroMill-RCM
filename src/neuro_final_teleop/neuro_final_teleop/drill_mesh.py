"""Display-only continuous neck/shaft surface; dimensions in metres."""
import math
import struct
from pathlib import Path
import numpy as np


def write_drill_mesh(path, yaw_deg, sensor_height, shaft_length, diameter):
    yaw = math.radians(yaw_deg)
    h = np.array([.033638*math.sin(yaw), -.033638*math.cos(yaw),
                  sensor_height+.062247])
    u = np.array([-math.sin(math.radians(20))*math.sin(yaw),
                  math.sin(math.radians(20))*math.cos(yaw), math.cos(math.radians(20))])
    r, p, y = np.radians([3.52, 2.74, 6.27])
    d = np.array([math.cos(y)*math.sin(p)*math.cos(r)+math.sin(y)*math.sin(r),
                  math.sin(y)*math.sin(p)*math.cos(r)-math.cos(y)*math.sin(r),
                  math.cos(p)*math.cos(r)])
    tip = np.array([-.03687, -.00758, .24361])
    start = h-.012*u
    end = tip-.8*shaft_length*d
    length = np.linalg.norm(end-start)
    centers, tangents, radii = [], [], []
    for t in np.linspace(0, 1, 101):
        centers.append((2*t**3-3*t*t+1)*start+(t**3-2*t*t+t)*length*u
                       +(-2*t**3+3*t*t)*end+(t**3-t*t)*length*d)
        tangents.append((6*t*t-6*t)*start+(3*t*t-4*t+1)*length*u
                        +(-6*t*t+6*t)*end+(3*t*t-2*t)*length*d)
        blend = 3*t*t-2*t**3
        radii.append(.0065*(1-blend)+diameter/2*blend)
    # Continue the same surface to TCP; no overlapping separate cylinder.
    for t in np.linspace(0, 1, 41)[1:]:
        centers.append(end+t*(tip-end))
        tangents.append(d)
        radii.append(diameter/2)
    rings = []
    for center, tangent, radius in zip(centers, tangents, radii):
        tangent = tangent/np.linalg.norm(tangent)
        normal = np.cross([0, 1, 0], tangent)
        if np.linalg.norm(normal) < 1e-8:
            normal = np.cross([1, 0, 0], tangent)
        normal /= np.linalg.norm(normal)
        binormal = np.cross(tangent, normal)
        rings.append([center+radius*(math.cos(a)*normal+math.sin(a)*binormal)
                      for a in np.linspace(0, 2*math.pi, 64, endpoint=False)])
    triangles = []
    for i in range(len(rings)-1):
        for j in range(64):
            k = (j+1)%64
            triangles.extend([(rings[i][j], rings[i][k], rings[i+1][k]),
                              (rings[i][j], rings[i+1][k], rings[i+1][j])])
    for j in range(64):
        k = (j+1)%64
        triangles.extend([(centers[0], rings[0][k], rings[0][j]),
                          (tip, rings[-1][j], rings[-1][k])])
    with Path(path).open('wb') as out:
        out.write(b'Continuous display-only drill surface'.ljust(80, b'\0'))
        out.write(struct.pack('<I', len(triangles)))
        for triangle in triangles:
            a,b,c = triangle
            n = np.cross(b-a,c-a)
            n /= np.linalg.norm(n)
            out.write(struct.pack('<12fH', *n, *a, *b, *c, 0))
