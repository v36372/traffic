import csv
import tempfile
import unittest
from pathlib import Path

from import_osm import bake, simple_polygon


class ImportTest(unittest.TestCase):
    def test_connectivity_and_direction(self):
        # Ways cross geometrically but use different OSM node IDs; no junction is invented.
        xml = '''<osm><node id="1" lat="10.0" lon="106.0"/><node id="2" lat="10.0" lon="106.1"/>
        <node id="3" lat="9.9" lon="106.05"/><node id="4" lat="10.1" lon="106.05"/>
        <node id="5" lat="10.0" lon="106.2"/>
        <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="5"/><tag k="highway" v="residential"/><tag k="oneway" v="-1"/></way>
        <way id="11"><nd ref="3"/><nd ref="4"/><tag k="highway" v="primary"/><tag k="bridge" v="yes"/></way>
        </osm>'''
        with tempfile.TemporaryDirectory() as d:
            source, output = Path(d) / 'input.osm', Path(d) / 'out.csv'
            source.write_text(xml)
            self.assertEqual(bake(source, output), (5, 4))
            with output.open() as f:
                rows = list(csv.reader(f))
            edges = [(int(r[1]), int(r[2])) for r in rows if r[0] == 'E']
            self.assertEqual(edges, [(1, 0), (4, 1), (2, 3), (3, 2)])

    def test_rejects_bad_coordinates_and_rounded_zero_segment(self):
        with tempfile.TemporaryDirectory() as d:
            source, output = Path(d) / 'input.osm', Path(d) / 'out.csv'
            source.write_text('<osm><node id="1" lat="nan" lon="0"/></osm>')
            with self.assertRaises(ValueError):
                bake(source, output)
            source.write_text('''<osm><node id="1" lat="10.7725" lon="106.6995"/>
                <node id="2" lat="10.7725" lon="106.6995000001"/>
                <node id="3" lat="10.7725" lon="106.6996"/>
                <way id="1"><nd ref="1"/><nd ref="2"/><nd ref="3"/>
                <tag k="highway" v="residential"/></way></osm>''')
            with self.assertRaisesRegex(ValueError, 'zero-length segment after rounding'):
                bake(source, output)

    def test_missing_node_preserves_existing_output(self):
        with tempfile.TemporaryDirectory() as d:
            source, output = Path(d) / 'input.osm', Path(d) / 'out.csv'
            output.write_text('existing output')
            source.write_text('''<osm><node id="1" lat="10" lon="106"/>
                <way id="42"><nd ref="1"/><nd ref="2"/>
                <tag k="highway" v="residential"/></way></osm>''')
            with self.assertRaisesRegex(ValueError, 'missing node on way 42'):
                bake(source, output)
            self.assertEqual(output.read_text(), 'existing output')

    def test_unknown_oneway_rejected(self):
        xml = '<osm><way id="1"><tag k="highway" v="primary"/><tag k="oneway" v="reversible"/></way></osm>'
        with tempfile.TemporaryDirectory() as d:
            source = Path(d) / 'input.osm'
            source.write_text(xml)
            with self.assertRaises(ValueError):
                bake(source, Path(d) / 'out.csv')


    def test_polygon_validation(self):
        self.assertTrue(simple_polygon([(0, 0), (2, 0), (2, 2), (1, 1), (0, 2)]))
        self.assertFalse(simple_polygon([(0, 0), (2, 2), (0, 2), (2, 0)]))
        self.assertFalse(simple_polygon([(0, 0), (2, 0), (1, 0), (0, 1)]))
        self.assertFalse(simple_polygon([(0, 0), (1, 0), (1, 0), (0, 1)]))

    def test_buildings_inference_and_unsupported(self):
        xml = '''<osm>
        <node id="1" lat="10.7725" lon="106.6995"/><node id="2" lat="10.7725" lon="106.6996"/>
        <node id="3" lat="10.7726" lon="106.6996"/><node id="4" lat="10.7726" lon="106.6995"/>
        <way id="road"><nd ref="1"/><nd ref="2"/><tag k="highway" v="residential"/></way>
        ''' + ''.join(f'<way id="{i}"><nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>{tags}</way>' for i, tags in [
            (10, '<tag k="building" v="yes"/><tag k="height" v="1 m"/><tag k="building:levels" v="8"/>'),
            (11, '<tag k="building" v="apartments"/><tag k="building:levels" v="5.5"/>'),
            (12, '<tag k="building" v="yes"/>'),
            (13, '<tag k="building" v="yes"/><tag k="height" v="nan"/>'),
            (14, '<tag k="building" v="yes"/>'),
            (15, '<tag k="building" v="roof"/>'),
        ]) + '''<relation id="30"><member type="way" ref="14" role="outer"/>
        <tag k="type" v="multipolygon"/><tag k="building" v="yes"/></relation></osm>'''
        with tempfile.TemporaryDirectory() as d:
            source, roads, buildings = (Path(d) / x for x in ('input.osm', 'roads.csv', 'buildings.csv'))
            source.write_text(xml)
            bake(source, roads, buildings)
            first = buildings.read_bytes()
            rows = list(csv.reader(buildings.read_text().splitlines()))
            self.assertEqual(rows[0], ['HCM_BUILDINGS', '1'])
            self.assertEqual([(r[1], r[2], r[3]) for r in rows[1:]], [('1.0', '0', '4'), ('16.5', '1', '4'), ('9.0', '0', '4')])
            bake(source, roads, buildings)
            self.assertEqual(first, buildings.read_bytes())

    def test_actual_artifacts_reproducible(self):
        root = Path(__file__).resolve().parent.parent
        with tempfile.TemporaryDirectory() as d:
            roads, buildings = Path(d) / 'roads.csv', Path(d) / 'buildings.csv'
            bake(root / 'data/hcm.osm', roads, buildings)
            self.assertEqual(roads.read_bytes(), (root / 'data/hcm.csv').read_bytes())
            self.assertEqual(buildings.read_bytes(), (root / 'data/hcm-buildings.csv').read_bytes())
            rows = list(csv.reader(buildings.read_text().splitlines()))
            self.assertEqual(len(rows) - 1, 123)
            self.assertEqual(max(float(row[1]) for row in rows[1:]), 193)


if __name__ == '__main__':
    unittest.main()
