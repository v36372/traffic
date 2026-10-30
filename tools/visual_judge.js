// Run in Pi codemode, not Node: eval an async wrapper around this file's text.
// Uses Pi's authenticated vision classifier; no credentials are written to disk.
const manifest = await tools.bash({command: "python3 - <<'PY'\nimport hashlib, json\nfrom pathlib import Path\nmeta = json.loads(Path('build/visual/reference.json').read_text())\nfilename = meta['file']\nif filename not in ('reference.jpg', 'reference.png'):\n    raise ValueError('Invalid reference filename')\nitems = []\nfor name, path, mime in [('prototype', 'build/visual/current.png', 'image/png'), ('reference', 'build/visual/' + filename, 'image/png' if filename.endswith('.png') else 'image/jpeg')]:\n    data = Path(path).read_bytes()\n    digest = hashlib.sha256(data).hexdigest()\n    if name == 'reference' and digest != meta['sha256']:\n        raise ValueError('Reference checksum changed')\n    if len(data) > 5000000:\n        raise ValueError('Image exceeds 5 MB: ' + path)\n    items.append(dict(name=name, path=path, sha256=digest, type='image', mimeType=mime, bytes=len(data)))\nprint(json.dumps(items))\nPY", timeout: 10});
const inputs = JSON.parse(manifest);
// Read bounded chunks: this harness truncates large bash outputs even inside codemode.
for (const input of inputs) {
    const chunks = [];
    for (let offset = 0; offset < input.bytes; offset += 24000) {
        chunks.push(tools.bash({command: `python3 -c "import base64; f=open('${input.path}','rb'); f.seek(${offset}); print(base64.b64encode(f.read(24000)).decode()); f.close()"`,timeout:10}));
    }
    input.data = (await Promise.all(chunks)).map(x=>x.trim()).join("");
}
const model = await models.getModelOfType("classifier", "openrouter", "cloudflare/clef-omni");
if (!model || !model.input.includes("image")) throw new Error("Configured visual judge is unavailable or lacks image input.");
const scale = ["Absent or fundamentally unlike the reference", "Very weak resemblance; major gaps", "Partial resemblance; substantial gaps", "Strong resemblance with some visible gaps", "Comparable visual quality and design intent"];
const subjects = {
    camera: "God-view camera composition and scale; do not confuse different neighborhoods with defects",
    roads: "Road surface, junction, marking and curb readability",
    city_detail: "Buildings, vegetation and urban context rather than empty terrain",
    lighting: "Lighting, material depth and visual separation",
    vehicles: "Visible vehicle scale and traffic legibility; cannot infer traffic physics from still images"
};
const questions = {};
for (const [key, subject] of Object.entries(subjects)) questions[key] = {
    type: "score", criteria: scale,
    instructions: "Image 1 is our prototype, image 2 is the Cities: Skylines reference. Compare only visible evidence for: " + subject + ". This is stylistic progress, not pixel matching or permission to copy assets."
};
questions.comparable = {type: "bool", instructions: "Can these two images support a meaningful city/road visual comparison despite differences in scene and zoom?", criteria: {true: "Both visibly show comparable elevated city/road scenes", false: "One is missing, blank, not gameplay, or viewpoint/scale differs too much"}};
questions.ui = {type: "choice", instructions: "Compare management UI readability only if both screenshots show UI; otherwise abstain.", criteria: {not_comparable: "Reference or prototype lacks visible UI", worse: "Prototype UI is noticeably less legible or organized", similar: "Comparable readability and organization", better: "Prototype UI is clearer"}};
questions.priority = {type: "choice", instructions: "Choose the single highest-impact visible improvement to the prototype, based on both actual images. Do not invent hidden features.", criteria: {camera: "Match a more useful camera angle/zoom before comparing", roads: "Improve road/junction geometry and markings", city: "Add coherent buildings/vegetation/urban context", lighting: "Improve materials/light/shadow/depth", vehicles: "Improve vehicle silhouettes/scale/visibility", ui: "Improve visible UI layout/readability", none: "No supported improvement can be identified"}};
const result = await models.classify(model, {
    state: {task: "Independent visual progress review", image_order: inputs.map(x=>x.name), caveat: "Synthetic traffic on real HCMC geometry. Judge visible appearance only. Scores are subjective ordinal levels, not percent similarity or simulation validation."},
    images: inputs.map(({type,data,mimeType})=>({type,data,mimeType})), questions
});
if (result.stopReason !== "stop") throw new Error(result.errorMessage || "Visual judge did not finish");
const report = {
    schema: 1, recorded_at: new Date().toISOString(),
    assessment_status: result.answers.comparable.probability >= 0.6 ? "comparable_review" : "reference_alignment_needed",
    rubric: {version: 1, score_range: [0,4], score_levels: scale},
    inputs: inputs.map(({name,path,sha256})=>({name,path,sha256})),
    model: {provider:model.provider,id:model.id}, result,
    caveats: ["Subjective visual assessment, not simulation correctness", "Camera and reference content affect scores; do not accept a visual pass when assessment_status is reference_alignment_needed", "Low-confidence answers are uncertain, not objective measurements", "Do not reuse reference art as game assets", "Re-run after visual changes; report is tied to input hashes"]
};
await tools.write({path:"build/visual/judgment.json",content:JSON.stringify(report,null,2)+"\n"});
text(report);
