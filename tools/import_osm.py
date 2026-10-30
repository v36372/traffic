#!/usr/bin/env python3
"""Bake a small OSM XML extract to the Phase A directed road CSV."""
import argparse
import csv
import math
import xml.etree.ElementTree as ET

# Keep roads usable by general motor traffic; this is not a full access interpreter.
HIGHWAYS = {'motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link',
            'secondary', 'secondary_link', 'tertiary', 'tertiary_link', 'unclassified',
            'residential', 'living_street', 'service', 'road'}
ORIGIN_LAT, ORIGIN_LON = 10.7725, 106.6995


def bake(source, destination, buildings=None):
    root = ET.parse(source).getroot()
    if root.tag != 'osm':
        raise ValueError('expected OSM XML')
    coords = {}
    for node in root.findall('node'):
        lat, lon = float(node.attrib['lat']), float(node.attrib['lon'])
        if not (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError(f"invalid coordinate on node {node.attrib['id']}")
        x = math.radians(lon - ORIGIN_LON) * 6371000 * math.cos(math.radians(ORIGIN_LAT))
        y = math.radians(lat - ORIGIN_LAT) * 6371000
        coords[node.attrib['id']] = (float(f'{x:.3f}'), float(f'{y:.3f}'))
    edges = []
    used = set()
    for way in root.findall('way'):
        tags = {t.attrib['k']: t.attrib['v'] for t in way.findall('tag')}
        if tags.get('highway') not in HIGHWAYS or tags.get('access') in ('no', 'private') or tags.get('motor_vehicle') in ('no', 'private'):
            continue
        direction = tags.get('oneway', '')
        if direction not in ('', 'yes', '1', 'true', '-1', 'no', '0', 'false'):
            raise ValueError(f"unsupported oneway={direction} on way {way.attrib['id']}")
        forward = direction != '-1'
        reverse = direction in ('no', '0', 'false') or (direction == '' and tags.get('junction') != 'roundabout' and tags['highway'] not in ('motorway', 'motorway_link'))
        refs = [n.attrib['ref'] for n in way.findall('nd')]
        if len(refs) < 2:
            raise ValueError(f"too few nodes on way {way.attrib['id']}")
        for ref in refs:
            if ref not in coords:
                raise ValueError(f"missing node on way {way.attrib['id']}: {ref}")
        for a, b in zip(refs, refs[1:]):
            if a == b or coords[a] == coords[b]:
                raise ValueError(f"zero-length segment after rounding on way {way.attrib['id']}: {a}, {b}")
            used.update((a, b))
            if forward:
                edges.append((a, b))
            if reverse or direction == '-1':
                edges.append((b, a))
    ids = sorted(used, key=int)
    index = {id_: i for i, id_ in enumerate(ids)}
    if not edges:
        raise ValueError('no usable road edges')
    with open(destination, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(['HCM_MAP', '1'])
        for id_ in ids:
            x, y = coords[id_]
            writer.writerow(['N', f'{x:.3f}', f'{y:.3f}'])
        for a, b in edges:
            writer.writerow(['E', index[a], index[b], '3.5', '8.33'])
    if buildings is not None:
        written, skipped = bake_buildings(root, coords, buildings)
        print(f'buildings={written} unsupported_outlines={skipped}')
    return len(ids), len(edges)


def simple_polygon(points):
    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def intersects(a, b, c, d):
        ab_c, ab_d, cd_a, cd_b = cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b)
        def on(a, b, p):
            return min(a[0], b[0]) <= p[0] <= max(a[0], b[0]) and min(a[1], b[1]) <= p[1] <= max(a[1], b[1])
        return ((ab_c == 0 and on(a, b, c)) or (ab_d == 0 and on(a, b, d)) or
                (cd_a == 0 and on(c, d, a)) or (cd_b == 0 and on(c, d, b)) or
                (ab_c * ab_d < 0 and cd_a * cd_b < 0))

    n = len(points)
    if not 3 <= n <= 256 or len(set(points)) != n:
        return False
    area = sum(points[i][0] * points[(i + 1) % n][1] - points[(i + 1) % n][0] * points[i][1] for i in range(n))
    if area == 0:
        return False
    for i in range(n):
        a, b, c = points[i - 1], points[i], points[(i + 1) % n]
        if cross(a, b, c) == 0 and (a[0] - b[0]) * (c[0] - b[0]) + (a[1] - b[1]) * (c[1] - b[1]) > 0:
            return False
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue
            if intersects(points[i], points[(i + 1) % n], points[j], points[(j + 1) % n]):
                return False
    return True


def bake_buildings(root, coords, destination):
    # All multipolygon way members are excluded, including separately tagged outers.
    relations = [r for r in root.findall('relation') if any(t.get('k') == 'type' and t.get('v') == 'multipolygon' for t in r.findall('tag'))]
    members = {m.get('ref') for r in relations for m in r.findall('member') if m.get('type') == 'way'}
    skipped = sum(1 for r in relations if any(t.get('k') == 'building' and t.get('v') not in ('no', 'roof', 'construction') for t in r.findall('tag')))
    rows = []
    height_counts = {'height': 0, 'levels': 0, 'default': 0}
    kind_counts = [0, 0, 0, 0]
    for way in root.findall('way'):
        tags = {t.get('k'): t.get('v') for t in way.findall('tag')}
        if tags.get('building') in (None, 'no', 'roof', 'construction') or 'construction' in tags:
            continue
        refs = [n.get('ref') for n in way.findall('nd')]
        if way.get('id') in members or len(refs) < 4 or refs[0] != refs[-1] or any(ref not in coords for ref in refs):
            skipped += 1
            continue
        points = [coords[ref] for ref in refs[:-1]]
        try:
            height = float(tags['height'].removesuffix(' m')) if 'height' in tags else float(tags['building:levels']) * 3 if 'building:levels' in tags else 9.0
        except ValueError:
            skipped += 1
            continue
        if not math.isfinite(height) or not 1e-6 <= height < 1000 or any(not (-1e7 < x < 1e7 and -1e7 < y < 1e7) for x, y in points) or not simple_polygon(points):
            skipped += 1
            continue
        building = tags['building']
        amenity = tags.get('amenity', '')
        if building in ('church', 'cathedral', 'mosque', 'temple', 'civic', 'public', 'government', 'marketplace') or amenity in ('place_of_worship', 'marketplace', 'townhall', 'courthouse', 'library'):
            kind = 3
        elif building in ('apartments', 'residential', 'dormitory') and height >= 6:
            kind = 1
        elif (building in ('office', 'commercial', 'retail') and height >= 6) or height >= 30:
            kind = 2
        else:
            kind = 0
        height_counts['height' if 'height' in tags else 'levels' if 'building:levels' in tags else 'default'] += 1
        kind_counts[kind] += 1
        rows.append(['B', str(height), kind, len(points), *(f'{v:.3f}' for p in points for v in p)])
    with open(destination, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(['HCM_BUILDINGS', '1'])
        writer.writerows(rows)
    print(f'height_sources={height_counts} kinds={kind_counts}')
    return len(rows), skipped


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('destination')
    parser.add_argument('--buildings', metavar='OUTPUT')
    args = parser.parse_args()
    print('nodes=%d directed_edges=%d' % bake(args.source, args.destination, args.buildings))
