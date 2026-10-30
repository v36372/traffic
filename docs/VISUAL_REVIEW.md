# Visual review

1. Capture this game with `make capture`. Inspect the capture before judging; record the camera, viewport and scene used.
2. Run `python3 tools/visual_reference.py` once to pin the official Cities: Skylines Steam store app 255710 screenshot 0 to `build/visual/reference.jpg` and provenance to `build/visual/reference.json`. The pinned files are reused offline; rerunning the command refuses to overwrite them. Use `--refresh` only when deliberately replacing the reference.
3. Run this in Pi codemode (not Node). It sends both images to Pi's authenticated `cloudflare/clef-omni` vision classifier; no extra paid API key is required:

   ```js
   const script = await tools.read({path:'tools/visual_judge.js'});
   await eval('(async () => {' + script + '\n})()');
   ```

   The result is `build/visual/judgment.json`, with subjective ordinal 0–4 diagnostic scores, model, input hashes and a prioritized improvement—not an overall similarity percentage. If the classifier's comparability probability is below 0.6, the report marks `assessment_status=reference_alignment_needed`; do not treat those scores as a trustworthy progress comparison. Keep the same reference, capture conditions, model and prompt when comparing iterations; verify the recorded hashes. These are classifier judgments, not a claim of manual visual review.

Choose another reference with `python3 tools/visual_reference.py --url 'https://example.com/image.jpg'` or `python3 tools/visual_reference.py --local /path/to/image.png`. Add `--refresh` to replace an existing pin. The tool accepts JPEG or PNG signatures; PNG pins use `reference.png` and the metadata names the actual file. It validates signatures, not complete image decoding. A failed download or validation leaves the prior reference untouched. Network access is needed only to acquire or refresh a remote reference. Default Steam API metadata is fetched anew only when acquiring or refreshing; the store image can change over time. The metadata records the actual image URL, source API, SHA-256, acquisition UTC and rights notice. Local paths are recorded as absolute paths; inspect metadata before sharing.

The default reference is a third-party Steam store promotional screenshot, **not** a locally captured Cities: Skylines game. It is for visual review only, not a game asset; copyright remains with its owner. Do not redistribute it as game content. User-provided references remain subject to their own rights and permissions. Generated captures, downloaded references and judgments are review artifacts, not shipped assets.

A vision score is subjective feedback, not a validation of traffic behavior, performance, accessibility or correctness. The initial official screenshot is not an apples-to-apples camera match; use an aligned user-selected screenshot for trustworthy progress comparisons. Different camera elevation, angle, zoom, viewport aspect ratio, lighting, scene content and city scale can dominate the comparison; align these where practical and note mismatches. Compare broad qualities such as readability, composition and visual density, not exact buildings, assets or UI. Similarity is not a license to copy protected artwork or imply the games are equivalent.
