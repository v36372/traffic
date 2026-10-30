#!/usr/bin/env python3
"""Pin a visual reference image for offline, repeatable reviews."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import tempfile
from urllib.request import Request, urlopen

STEAM_API = 'https://store.steampowered.com/api/appdetails?appids=255710'
LIMIT = 20 * 1024 * 1024
RIGHTS = ('Third-party reference for visual review only; not a game asset. '
          'Copyright remains with its respective owner; do not redistribute as game content.')


def fetch(url):
    request = Request(url, headers={'User-Agent': 'TrafficGameVisualReview/1.0'})
    with urlopen(request, timeout=20) as response:
        data = response.read(LIMIT + 1)
    if len(data) > LIMIT:
        raise ValueError('response exceeds 20 MiB')
    return data


def image_extension(data):
    if data.startswith(b'\xff\xd8\xff'):
        return '.jpg'
    if data.startswith(b'\x89PNG\r\n\x1a\n'):
        return '.png'
    raise ValueError('reference must be a JPEG or PNG image')


def pin(output_dir, url=None, local=None, refresh=False):
    if url and local:
        raise ValueError('choose either URL or local image')
    output_dir = Path(output_dir)
    existing = [output_dir / ('reference' + ext) for ext in ('.jpg', '.png', '.json')]
    if not refresh and any(path.exists() for path in existing):
        raise FileExistsError('reference already exists; pass --refresh to replace it')
    source_api = None
    if local:
        source = str(Path(local).resolve())
        with open(local, 'rb') as f:
            data = f.read(LIMIT + 1)
        if len(data) > LIMIT:
            raise ValueError('image exceeds 20 MiB')
    else:
        if not url:
            source_api = STEAM_API
            payload = json.loads(fetch(STEAM_API))
            entry = payload['255710']
            if not entry.get('success'):
                raise ValueError('Steam API did not return app 255710')
            url = entry['data']['screenshots'][0]['path_full']
        source = url
        if not source.startswith(('https://', 'http://')):
            raise ValueError('reference URL must use HTTP or HTTPS')
        data = fetch(source)
    ext = image_extension(data)
    digest = hashlib.sha256(data).hexdigest()
    metadata = {'source': source, 'source_api': source_api, 'sha256': digest,
                'fetched_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                'file': 'reference' + ext, 'rights': RIGHTS}
    output_dir.mkdir(parents=True, exist_ok=True)
    # Stage both files before replacing the reference; network and validation failures
    # never overwrite the existing pin. Individual replaces are atomic, not the pair.
    staged = []
    try:
        for content in (data, (json.dumps(metadata, indent=2) + '\n').encode()):
            with tempfile.NamedTemporaryFile(dir=output_dir, delete=False) as f:
                staged.append(Path(f.name))
                f.write(content)
        os.replace(staged[0], output_dir / ('reference' + ext))
        os.replace(staged[1], output_dir / 'reference.json')
        other = output_dir / ('reference.png' if ext == '.jpg' else 'reference.jpg')
        if other.exists():
            other.unlink()
    finally:
        for path in staged:
            path.unlink(missing_ok=True)
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--url', help='HTTP(S) JPEG/PNG reference URL')
    source.add_argument('--local', help='local JPEG/PNG reference')
    parser.add_argument('--refresh', action='store_true', help='replace an existing pin')
    parser.add_argument('--output-dir', default='build/visual')
    args = parser.parse_args()
    try:
        result = pin(args.output_dir, args.url, args.local, args.refresh)
    except (OSError, ValueError, KeyError, IndexError, TypeError) as exc:
        parser.exit(1, f'visual reference: {exc}\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
