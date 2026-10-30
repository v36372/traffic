import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import visual_reference as reference


class ReferenceTest(unittest.TestCase):
    def test_default_api_pin_and_refresh_guard(self):
        api = json.dumps({'255710': {'success': True, 'data': {'screenshots': [
            {'path_full': 'https://example.test/screenshot.jpg'}]}}}).encode()
        jpeg = b'\xff\xd8\xffexample'
        with tempfile.TemporaryDirectory() as d, patch.object(reference, 'fetch', side_effect=[api, jpeg]) as fetch:
            meta = reference.pin(d)
            self.assertEqual(meta['source'], 'https://example.test/screenshot.jpg')
            self.assertEqual(meta['source_api'], reference.STEAM_API)
            self.assertEqual((Path(d) / 'reference.jpg').read_bytes(), jpeg)
            self.assertEqual(json.loads((Path(d) / 'reference.json').read_text()), meta)
            with self.assertRaises(FileExistsError):
                reference.pin(d)
            self.assertEqual(fetch.call_count, 2)

    def test_failed_refresh_preserves_reference_offline(self):
        with tempfile.TemporaryDirectory() as d:
            source = Path(d) / 'input.png'
            source.write_bytes(b'\x89PNG\r\n\x1a\nimage')
            out = Path(d) / 'out'
            reference.pin(out, local=source)
            before = (out / 'reference.json').read_bytes()
            with patch.object(reference, 'fetch', side_effect=OSError('offline')):
                with self.assertRaises(OSError):
                    reference.pin(out, url='https://example.test/image', refresh=True)
            self.assertEqual((out / 'reference.json').read_bytes(), before)
            self.assertEqual((out / 'reference.png').read_bytes(), source.read_bytes())
            source.write_bytes(b'not an image')
            with self.assertRaises(ValueError):
                reference.pin(out, local=source, refresh=True)
            self.assertEqual((out / 'reference.json').read_bytes(), before)

    def test_rejects_bad_url_and_api_failure(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):
                reference.pin(d, url='file:///etc/passwd')
            with patch.object(reference, 'fetch', return_value=b'{"255710":{"success":false}}'):
                with self.assertRaises(ValueError):
                    reference.pin(d)
            self.assertEqual(list(Path(d).iterdir()), [])


if __name__ == '__main__':
    unittest.main()
