#!/usr/bin/env python3
"""Compile an editable v1 XML map to road, building and object CSVs."""
import argparse
import csv
import math
import os
from pathlib import Path
import struct
import tempfile
import xml.etree.ElementTree as ET

from import_osm import simple_polygon

KINDS = {'lowrise': 0, 'apartment': 1, 'office': 2, 'landmark': 3}
VEHICLES = {'motorcycle': (0, 0.75), 'car': (1, 1.9), 'bus': (2, 2.5),
            'bicycle': (3, 0.65), 'truck': (4, 2.4), 'tricycle': (5, 1.35)}


def attrs(element, required, optional=()):
    keys = set(element.attrib)
    if not set(required) <= keys or keys - set(required) - set(optional):
        raise ValueError(f'invalid attributes on <{element.tag}>')
    if element.text and element.text.strip() or element.tail and element.tail.strip():
        raise ValueError(f'unexpected text in <{element.tag}>')


def number(value, label, lower, upper):
    try:
        n = float(value)
        # The loader stores values as f32; validate the actual geometry it sees.
        rounded = struct.unpack('f', struct.pack('f', n))[0]
    except (ValueError, OverflowError, struct.error):
        raise ValueError(f'invalid {label}: {value}') from None
    if not math.isfinite(n) or not (lower < n < upper and lower < rounded < upper):
        raise ValueError(f'invalid {label}: {value}')
    return rounded


def timing(value, label, minimum):
    n = number(value, label, -math.inf, math.inf)
    if not (minimum <= float(value) <= 86400 and minimum <= n <= 86400):
        raise ValueError(f'invalid {label}: {value}')
    return n


def coordinate(value):
    return number(value, 'coordinate', -1e7, 1e7)


def ident(value):
    if not value.strip():
        raise ValueError('empty id')
    return value


def compile_map(source):
    root = ET.parse(source).getroot()
    if root.tag != 'map':
        raise ValueError('expected <map>')
    attrs(root, ('version', 'name'))
    if root.get('version') != '1' or not root.get('name').strip():
        raise ValueError('unsupported version or empty map name')
    nodes, roads, buildings, signals, spawns = {}, [], [], [], []
    road_ids, building_ids, signal_ids, spawn_ids = set(), set(), set(), set()
    for child in root:
        if child.tag == 'node':
            attrs(child, ('id', 'x', 'y'))
            if len(child):
                raise ValueError('node cannot contain elements')
            id_ = ident(child.get('id'))
            if id_ in nodes:
                raise ValueError(f'duplicate node id: {id_}')
            nodes[id_] = (coordinate(child.get('x')), coordinate(child.get('y')))
        elif child.tag == 'road':
            attrs(child, ('from', 'to', 'width', 'speed', 'direction'), ('id',))
            if len(child):
                raise ValueError('road cannot contain elements')
            if 'id' in child.attrib:
                id_ = ident(child.get('id'))
                if id_ in road_ids:
                    raise ValueError(f'duplicate road id: {id_}')
                road_ids.add(id_)
            direction = child.get('direction')
            if direction not in ('oneway', 'twoway'):
                raise ValueError(f'invalid direction: {direction}')
            roads.append((child.get('id'), child.get('from'), child.get('to'),
                          number(child.get('width'), 'width', 0, 100),
                          number(child.get('speed'), 'speed', 0, 100), direction))
        elif child.tag in ('signal', 'spawn'):
            required = ('id', 'node', 'green', 'amber', 'clearance') if child.tag == 'signal' else ('id', 'road', 'destination', 'interval', 'kind')
            attrs(child, required, ('offset',) if child.tag == 'signal' else ())
            if len(child):
                raise ValueError(f'{child.tag} cannot contain elements')
            ids = signal_ids if child.tag == 'signal' else spawn_ids
            id_ = ident(child.get('id'))
            if id_ in ids:
                raise ValueError(f'duplicate {child.tag} id: {id_}')
            ids.add(id_)
            (signals if child.tag == 'signal' else spawns).append(child)
        elif child.tag == 'building':
            attrs(child, ('id', 'kind', 'height'))
            id_ = ident(child.get('id'))
            if id_ in building_ids:
                raise ValueError(f'duplicate building id: {id_}')
            building_ids.add(id_)
            kind = child.get('kind')
            if kind not in KINDS:
                raise ValueError(f'invalid building kind: {kind}')
            height = number(child.get('height'), 'height', 0, 1000)
            points = []
            for vertex in child:
                if vertex.tag != 'vertex':
                    raise ValueError(f'unexpected element: {vertex.tag}')
                attrs(vertex, ('x', 'y'))
                if len(vertex):
                    raise ValueError('vertex cannot contain elements')
                points.append((coordinate(vertex.get('x')), coordinate(vertex.get('y'))))
            if not simple_polygon(points):
                raise ValueError(f'invalid outline on building {id_}')
            area = sum(x * points[(i + 1) % len(points)][1] - points[(i + 1) % len(points)][0] * y
                       for i, (x, y) in enumerate(points))
            if area < 0:
                points.reverse()
            buildings.append(['B', height, KINDS[kind], len(points), *(v for p in points for v in p)])
        else:
            raise ValueError(f'unexpected element: {child.tag}')
    if not nodes or not roads:
        raise ValueError('map needs nodes and roads')
    indexes = {id_: i for i, id_ in enumerate(nodes)}
    edges = []
    road_indexes = {}
    for id_, a, b, width, speed, direction in roads:
        if a not in nodes or b not in nodes:
            raise ValueError(f'unknown road node: {a}, {b}')
        if a == b or nodes[a] == nodes[b]:
            raise ValueError(f'zero-length road: {a}, {b}')
        if id_ is not None:
            road_indexes[id_] = len(edges)
        edges.append(['E', indexes[a], indexes[b], width, speed])
        if direction == 'twoway':
            edges.append(['E', indexes[b], indexes[a], width, speed])
    objects = [['TRAFFIC_OBJECTS', '1']]
    controlled = set()
    for signal in signals:
        node = signal.get('node')
        if node not in indexes or node in controlled:
            raise ValueError(f'unknown or duplicate signal node: {node}')
        index = indexes[node]
        if sum(edge[2] == index for edge in edges) < 2:
            raise ValueError(f'signal needs at least two incoming roads: {node}')
        controlled.add(node)
        timings = [timing(signal.get(k), k, 0.05) for k in ('green', 'amber', 'clearance')]
        offset = timing(signal.get('offset', '0'), 'offset', 0)
        objects.append(['S', index, *timings, offset])
    for spawn in spawns:
        road, destination, kind = spawn.get('road'), spawn.get('destination'), spawn.get('kind')
        if road not in road_indexes or destination not in indexes or kind not in VEHICLES:
            raise ValueError('unknown spawn road, destination or kind')
        road_index, dest = road_indexes[road], indexes[destination]
        first = edges[road_index]
        kind_index, width = VEHICLES[kind]
        width = struct.unpack('f', struct.pack('f', width))[0]  # Match sim.kind_width's f32 values.
        if dest == first[1] or first[3] < width:
            raise ValueError('invalid spawn route')
        reached, pending = {first[2]}, [first[2]]
        while pending and dest not in reached:
            current = pending.pop()
            for edge in edges:
                if edge[1] == current and edge[3] >= width and edge[2] not in reached:
                    reached.add(edge[2])
                    pending.append(edge[2])
        if dest not in reached:
            raise ValueError('unreachable spawn destination')
        objects.append(['P', road_index, dest, timing(spawn.get('interval'), 'interval', 0.05), kind_index])
    return ([['HCM_MAP', '1'], *(['N', *p] for p in nodes.values()), *edges],
            [['HCM_BUILDINGS', '1'], *buildings], objects)


def build(source, output):
    source, output = Path(source), Path(output)
    sidecar = output.with_name(output.stem + '-buildings.csv')
    objects = output.with_name(output.stem + '-objects.csv')
    def same(a, b):
        return a.resolve() == b.resolve() or (a.exists() and b.exists() and os.path.samefile(a, b))
    paths = (source, output, sidecar, objects)
    if any(same(a, b) for i, a in enumerate(paths) for b in paths[i+1:]):
        raise ValueError('input and output paths must be distinct')
    road_rows, building_rows, object_rows = compile_map(source)
    output.parent.mkdir(parents=True, exist_ok=True)
    staged = []
    try:
        for destination, rows in ((output, road_rows), (sidecar, building_rows), (objects, object_rows)):
            with tempfile.NamedTemporaryFile(mode='w', newline='', encoding='utf-8',
                                             dir=destination.parent, delete=False) as f:
                staged.append(Path(f.name))
                csv.writer(f, lineterminator='\n').writerows(rows)
        # Each replacement is atomic, but the three replacements are not one transaction.
        for temporary, destination in zip(staged, (output, sidecar, objects)):
            os.replace(temporary, destination)
    finally:
        for temporary in staged:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    try:
        build(args.source, args.output)
    except (ValueError, ET.ParseError, OSError) as error:
        parser.exit(1, f'build_map: {error}\n')


if __name__ == '__main__':
    main()
