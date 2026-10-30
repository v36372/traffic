import csv
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from build_map import build

ROOT = Path(__file__).resolve().parents[1]
BASE = '''<map version="1" name="test"><node id="a" x="0" y="0"/>
<node id="b" x="10" y="0"/><road from="a" to="b" width="3.5" speed="8" direction="twoway"/>
<building id="house" kind="office" height="10"><vertex x="0" y="1"/><vertex x="0" y="3"/><vertex x="2" y="3"/><vertex x="2" y="1"/></building></map>'''


class BuildMapTest(unittest.TestCase):
    def test_output_and_winding(self):
        with tempfile.TemporaryDirectory() as d:
            source, output = Path(d) / 'in.xml', Path(d) / 'out.csv'
            source.write_text(BASE)
            build(source, output)
            first = tuple((Path(d) / n).read_bytes() for n in ('out.csv', 'out-buildings.csv', 'out-objects.csv'))
            build(source, output)
            self.assertEqual(first, tuple((Path(d) / n).read_bytes() for n in ('out.csv', 'out-buildings.csv', 'out-objects.csv')))
            rows = list(csv.reader(output.read_text().splitlines()))
            self.assertEqual(rows[0], ['HCM_MAP', '1'])
            self.assertEqual([r[1:3] for r in rows if r[0] == 'E'], [['0', '1'], ['1', '0']])
            b = list(csv.reader((Path(d) / 'out-buildings.csv').read_text().splitlines()))[1]
            self.assertEqual(b[:4], ['B', '10.0', '2', '4'])
            p = list(zip(map(float, b[4::2]), map(float, b[5::2])))
            self.assertGreater(sum(x*p[(i+1)%len(p)][1]-p[(i+1)%len(p)][0]*y for i, (x, y) in enumerate(p)), 0)
            source.write_text(BASE.replace('twoway', 'oneway').replace(BASE[BASE.index('<building'):BASE.index('</map>')], ''))
            build(source, output)
            self.assertEqual(sum(r.startswith('E,') for r in output.read_text().splitlines()), 1)
            self.assertEqual((Path(d) / 'out-buildings.csv').read_text(), 'HCM_BUILDINGS,1\n')

    def test_invalid_preserves_outputs(self):
        bad = [
            BASE.replace('version="1"', 'version="2"'), BASE[:-5],
            BASE.replace('name="test"', 'name="test" extra="x"'),
            BASE.replace('</map>', '<signal/></map>'),
            BASE.replace('id="b"', 'id="a"'), BASE.replace('to="b"', 'to="missing"'),
            BASE.replace('to="b"', 'to="a"'), BASE.replace('twoway', 'both'),
            BASE.replace('width="3.5"', 'width="nan"'), BASE.replace('speed="8"', 'speed="inf"'),
            BASE.replace('speed="8"', 'speed="0"'), BASE.replace('width="3.5"', 'width="100"'),
            BASE.replace('height="10"', 'height="-1"'), BASE.replace('kind="office"', 'kind="tower"'),
            BASE.replace('x="10"', 'x="nan"'), BASE.replace('<vertex x="2" y="1"/>', '' ).replace('<vertex x="2" y="3"/>', ''),
            BASE.replace('<vertex x="0" y="3"/><vertex x="2" y="3"/>', '<vertex x="2" y="3"/><vertex x="0" y="3"/>'),
            BASE.replace('</building>', '<vertex x="0" y="1"/></building>'),
            BASE.replace('</building>', '<unknown/></building>'),
            BASE.replace('<road ', '<road id="r" ').replace('</map>', '<road id="r" from="a" to="b" width="1" speed="1" direction="oneway"/></map>'),
            BASE.replace('</map>', BASE[BASE.index('<building'):BASE.index('</map>')] + '</map>'),
            BASE.replace('</building>', '<vertex x="1" y="2"/>' * 254 + '</building>'),
        ]
        with tempfile.TemporaryDirectory() as d:
            source, output = Path(d) / 'in.xml', Path(d) / 'out.csv'
            sidecar = Path(d) / 'out-buildings.csv'
            objects = Path(d) / 'out-objects.csv'
            for xml in bad:
                with self.subTest(xml=xml[:120]):
                    source.write_text(xml)
                    output.write_text('roads preserved')
                    sidecar.write_text('buildings preserved')
                    objects.write_text('objects preserved')
                    with self.assertRaises((ValueError, ET.ParseError)):
                        build(source, output)
                    self.assertEqual(output.read_text(), 'roads preserved')
                    self.assertEqual(sidecar.read_text(), 'buildings preserved')
                    self.assertEqual(objects.read_text(), 'objects preserved')

    def test_collision(self):
        with tempfile.TemporaryDirectory() as d:
            source = Path(d) / 'map-buildings.csv'
            source.write_text(BASE)
            with self.assertRaises(ValueError):
                build(source, Path(d) / 'map.csv')
            with self.assertRaises(ValueError):
                build(source, source)
            other = Path(d) / 'map-objects.csv'
            other.write_text(BASE)
            with self.assertRaises(ValueError):
                build(other, Path(d) / 'map.csv')
            self.assertEqual(source.read_text(), BASE)

    def test_objects(self):
        xml = (ROOT / 'maps/intersection.xml').read_text()
        with tempfile.TemporaryDirectory() as d:
            source, output = Path(d) / 'in.xml', Path(d) / 'out.csv'
            source.write_text(xml)
            build(source, output)
            rows = list(csv.reader((Path(d) / 'out-objects.csv').read_text().splitlines()))
            self.assertEqual(rows, [['TRAFFIC_OBJECTS', '1'], ['S', '1', '8.0', '2.0', '1.0', '0.0'],
                                    ['P', '0', '2', '2.0', '0'], ['P', '4', '3', '4.0', '1']])
            bad = [
                xml.replace('node="center" green', 'node="missing" green'),
                xml.replace('</map>', '<signal id="other" node="center" green="1" amber="1" clearance="1"/></map>'),
                xml.replace('node="center" green', 'node="west" green'),
                xml.replace('green="8"', 'green="0.001"'),
                xml.replace('amber="2"', 'amber="86401"'),
                xml.replace('clearance="1"', 'clearance="nan"'),
                xml.replace('green="8"', 'green="8" offset="-0.00001"'),
                xml.replace('road="west-in"', 'road="missing"'),
                xml.replace('destination="east"', 'destination="west"'),
                xml.replace('kind="motorcycle"', 'kind="scooter"'),
                xml.replace('interval="2"', 'interval="0.001"'),
                xml.replace('interval="2"', 'interval="86401"'),
                xml.replace('width="3.5"', 'width="0.5"'),
                xml.replace('from="center" to="east" width="3.5" speed="10" direction="twoway"', 'from="east" to="center" width="3.5" speed="10" direction="oneway"'),
                xml.replace('from="center" to="east" width="3.5"', 'from="center" to="east" width="0.5"'),
                xml.replace('id="west-in"', 'id="west-in" extra="x"'),
                xml.replace('</map>', '<bus-stop id="fake"/></map>'),
            ]
            outputs = [output, Path(d) / 'out-buildings.csv', Path(d) / 'out-objects.csv']
            for case in bad:
                with self.subTest(case=case):
                    source.write_text(case)
                    for path in outputs: path.write_text('preserved')
                    with self.assertRaises(ValueError): build(source, output)
                    self.assertEqual([p.read_text() for p in outputs], ['preserved'] * 3)
            source.write_text(xml.replace('width="3.5"', 'width="1.9"').replace('kind="motorcycle"', 'kind="car"'))
            build(source, output)  # 1.9 rounds to the same f32 as sim's car width.

    def test_driver_configuration(self):
        driver = '<driver kind="motorcycle" speed_min="0.7" speed_max="1" headway_min="0.4" headway_max="2"/>'
        with tempfile.TemporaryDirectory() as d:
            source, output = Path(d) / 'in.xml', Path(d) / 'out.csv'
            source.write_text(BASE.replace('</map>', driver + '</map>'))
            build(source, output)
            rows = list(csv.reader((Path(d) / 'out-objects.csv').read_text().splitlines()))
            self.assertEqual(rows[1][0:2], ['D', '0'])
            self.assertAlmostEqual(float(rows[1][2]), 0.7)
            for bad in (driver + driver, driver.replace('1" headway', '1.1" headway'),
                        driver.replace('0.4', 'nan'), driver.replace('motorcycle', 'scooter'),
                        driver.replace('0.7', '0.499999999'), driver.replace('1" headway', '1.000000001" headway'),
                        driver.replace('0.4', '0.399999999'), driver.replace('2"/>', '2.000000001"/>'),
                        driver.replace('0.7', '0.700000001').replace('1" headway', '0.7" headway'),
                        driver.replace('headway_min="0.4"', 'headway_min="0.800000001"').replace('headway_max="2"', 'headway_max="0.8"')):
                source.write_text(BASE.replace('</map>', bad + '</map>'))
                with self.assertRaises(ValueError): build(source, output)

    def test_examples(self):
        sources = sorted((ROOT / 'maps').glob('*.xml'))
        self.assertTrue({'intersection', 'bottleneck', 'downtown', 'signalized', 'showcase'} <= {source.stem for source in sources})
        with tempfile.TemporaryDirectory() as d:
            for source in sources:
                with self.subTest(source=source):
                    output = Path(d) / (source.stem + '.csv')
                    build(source, output)
                    self.assertTrue(output.read_text().startswith('HCM_MAP,1\n'))
                    self.assertTrue(output.with_name(output.stem + '-buildings.csv').read_text().startswith('HCM_BUILDINGS,1\n'))
                    self.assertTrue(output.with_name(output.stem + '-objects.csv').read_text().startswith('TRAFFIC_OBJECTS,1\n'))


if __name__ == '__main__':
    unittest.main()
