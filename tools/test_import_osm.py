import csv
import io
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

from import_osm import bake, report_connectivity, report_routes, simple_polygon


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
            audit = io.StringIO()
            with redirect_stdout(audit):
                self.assertEqual(bake(source, output, routes=[('5', '1'), ('1', '5')]), (5, 4))
            self.assertIn('weak_components=2\n', audit.getvalue())
            self.assertIn('component: nodes=3 example_osm_node=1\n', audit.getvalue())
            self.assertIn('route 5 -> 1: reachable edges=2\n', audit.getvalue())
            self.assertIn('route 1 -> 5: unreachable\n', audit.getvalue())
            self.assertIn('ignored_bridge_ways=1 example_osm_ways=11\n', audit.getvalue())
            with output.open() as f:
                rows = list(csv.reader(f))
            edges = [(int(r[1]), int(r[2])) for r in rows if r[0] == 'E']
            self.assertEqual(edges, [(1, 0), (4, 1), (2, 3), (3, 2)])

    def test_connectivity_audit_reports_disconnected_components_deterministically(self):
        # One-way edges are weakly connected even though not every route is reachable.
        edges = [('3', '2'), ('2', '1'), ('4', '5'), ('5', '4')]
        expected = ('weak_components=2\n'
                    '  component: nodes=3 example_osm_node=1\n'
                    '  component: nodes=2 example_osm_node=4\n'
                    'strong_components=4\n'
                    '  strong_component: nodes=2 example_osm_node=4\n'
                    '  strong_component: nodes=1 example_osm_node=1\n'
                    '  strong_component: nodes=1 example_osm_node=2\n'
                    '  strong_component: nodes=1 example_osm_node=3\n')
        for ordering in (edges, list(reversed(edges))):
            output = io.StringIO()
            with redirect_stdout(output):
                report_connectivity(ordering)
            self.assertEqual(output.getvalue(), expected)

    def test_connectivity_audit_bounds_component_details(self):
        edges = [(str(i), str(i + 1)) for i in range(1, 15, 2)]
        output = io.StringIO()
        with redirect_stdout(output):
            report_connectivity(edges)
        self.assertIn('weak_components=7\n', output.getvalue())
        self.assertIn('  additional_components=2\n', output.getvalue())
        self.assertEqual(output.getvalue().count('  component:'), 5)

    def test_connectivity_audit_directed_cycles_and_long_chain(self):
        edges = [('1', '2'), ('2', '1'), ('2', '3'), ('3', '4'), ('4', '3')]
        output = io.StringIO()
        with redirect_stdout(output):
            report_connectivity(edges)
        self.assertEqual(output.getvalue(), 'weak_components=1\nstrong_components=2\n'
                         '  strong_component: nodes=2 example_osm_node=1\n'
                         '  strong_component: nodes=2 example_osm_node=3\n')
        output = io.StringIO()
        with redirect_stdout(output):
            report_connectivity([(str(i), str(i + 1)) for i in range(1500)])
        self.assertIn('weak_components=1\nstrong_components=1501\n', output.getvalue())
        self.assertIn('additional_strong_components=1496\n', output.getvalue())

    def test_directed_route_audit(self):
        output = io.StringIO()
        with redirect_stdout(output):
            report_routes([('1', '2'), ('2', '3'), ('3', '2'), ('4', '5')],
                          [('1', '3'), ('3', '1'), ('2', '2'), ('1', '5'), ('1', '6')])
        self.assertEqual(output.getvalue(), 'route 1 -> 3: reachable edges=2\n'
                         'route 3 -> 1: unreachable\n'
                         'route 2 -> 2: reachable edges=0\n'
                         'route 1 -> 5: unreachable\n'
                         'route 1 -> 6: missing_node\n')

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

    def test_unsupported_road_tags_are_reported_not_invented(self):
        xml = '''<osm><node id="1" lat="10" lon="106"/><node id="2" lat="10" lon="106.01"/>
        <way id="42"><nd ref="1"/><nd ref="2"/><tag k="highway" v="primary"/>
        <tag k="layer" v="-1"/><tag k="tunnel" v="yes"/>
        <tag k="turn:lanes:forward" v="left|through"/>
        <tag k="maxspeed:conditional" v="30 @ (wet)"/></way>
        <relation id="100"><member type="way" ref="42" role="from"/>
        <tag k="type" v="restriction"/><tag k="restriction" v="no_left_turn"/></relation>
        <relation id="101"><member type="way" ref="999" role="from"/>
        <tag k="type" v="restriction"/></relation></osm>'''
        with tempfile.TemporaryDirectory() as d:
            source, output = Path(d) / 'input.osm', Path(d) / 'out.csv'
            source.write_text(xml)
            audit = io.StringIO()
            with redirect_stdout(audit):
                bake(source, output)
            for category in ('layer', 'tunnel', 'turn:lanes', 'conditional'):
                self.assertIn(f'ignored_{category}_ways=1 example_osm_ways=42\n', audit.getvalue())
            self.assertNotIn('ignored_bridge_', audit.getvalue())
            self.assertIn('ignored_restriction_relations=1 example_osm_relations=100\n', audit.getvalue())

    def test_maxspeed_per_direction_and_inferred_default(self):
        xml = '''<osm>
        <node id="1" lat="10" lon="106"/><node id="2" lat="10" lon="106.01"/>
        <node id="3" lat="10" lon="106.02"/><node id="4" lat="10" lon="106.03"/>
        <way id="1"><nd ref="1"/><nd ref="2"/><tag k="highway" v="residential"/>
        <tag k="maxspeed" v="50"/><tag k="maxspeed:forward" v="30 mph"/>
        <tag k="maxspeed:backward" v="20 km/h"/></way>
        <way id="2"><nd ref="2"/><nd ref="3"/><tag k="highway" v="primary"/>
        <tag k="oneway" v="-1"/><tag k="maxspeed" v="60"/></way>
        <way id="3"><nd ref="3"/><nd ref="4"/><tag k="highway" v="residential"/></way>
        </osm>'''
        with tempfile.TemporaryDirectory() as d:
            source, output = Path(d) / 'input.osm', Path(d) / 'out.csv'
            source.write_text(xml)
            audit = io.StringIO()
            with redirect_stdout(audit):
                bake(source, output)
            edges = [r for r in csv.reader(output.read_text().splitlines()) if r[0] == 'E']
            self.assertEqual([r[4] for r in edges], ['13.411', '5.556', '16.667', '8.33', '8.33'])
            self.assertIn("speed_sources={'osm': 3, 'default': 2}", audit.getvalue())

    def test_invalid_maxspeed_does_not_overwrite_output(self):
        with tempfile.TemporaryDirectory() as d:
            source, output = Path(d) / 'input.osm', Path(d) / 'out.csv'
            output.write_text('existing output')
            for value in ('signals', '0', 'nan', '360', '0.001'):
                source.write_text(f'''<osm><node id="1" lat="10" lon="106"/>
                <node id="2" lat="10" lon="106.01"/><way id="42"><nd ref="1"/><nd ref="2"/>
                <tag k="highway" v="primary"/><tag k="maxspeed" v="{value}"/></way></osm>''')
                with self.assertRaisesRegex(ValueError, 'maxspeed=.* on way 42'):
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
            audit = io.StringIO()
            with redirect_stdout(audit):
                bake(root / 'data/hcm.osm', roads, buildings)
            self.assertIn('weak_components=1\n', audit.getvalue())
            self.assertIn('strong_components=107\n', audit.getvalue())
            self.assertEqual(roads.read_bytes(), (root / 'data/hcm.csv').read_bytes())
            self.assertEqual(buildings.read_bytes(), (root / 'data/hcm-buildings.csv').read_bytes())
            rows = list(csv.reader(buildings.read_text().splitlines()))
            self.assertEqual(len(rows) - 1, 123)
            self.assertEqual(max(float(row[1]) for row in rows[1:]), 193)


if __name__ == '__main__':
    unittest.main()
