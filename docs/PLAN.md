# HCMC Traffic — research and development proposal

**Status:** Proposal for review; no game implementation yet.

## 1. Recommendation

Build a single-player, desktop, 3D traffic-management game in **Odin + raylib**, with a rotatable, zoomable god-view camera. Its identity is not “Cities: Skylines with Vietnamese scenery”: it is **managing motorcycle-dominated mixed traffic, constrained streets, public money, and public trust**.

Start with one recognizable, connected area of central Ho Chi Minh City, approximately **2–4 km²**, plus a surrounding traffic buffer. Use real street geometry and curated rules, with explicitly synthetic/calibrated travel demand. Expand only after the traffic model and management loop work.

The first product promise should be **real geography, plausible traffic, meaningful interventions**, not a live digital twin. A real map alone does not establish real traffic volumes, destinations, behavior, or signal timing.

All sizes, performance budgets, staffing, prices, and schedules below are **proposed targets**, not measured capabilities or local factual estimates.

## 2. Research findings and implications

### Geography and traffic data

| Finding | Design implication |
|---|---|
| Geofabrik distributes a Vietnam OpenStreetMap extract. [1] | Clip and preprocess a pinned extract offline; do not depend on live map servers during play. |
| OSM is ODbL-licensed and requires attribution; adapted databases carry obligations. [2] | Keep source provenance, licenses, map build scripts, and redistribution requirements explicit. |
| HCMC expanded administratively in 2025 through the merger with Bình Dương and Bà Rịa–Vũng Tàu. [3] | “The whole city” needs a defined boundary. Recommend the historic urban core first, not the entire enlarged municipality. |
| World Bank material reports motorcycles at 62% of daily trips in **2010**. [4] | Motorcycle dominance is well motivated, but this is not a current mode-share calibration. Do not confuse trip share, traffic counts, and vehicle registrations. |
| JICA's HOUTRANS provides historical surveys and planning context, dating to 2004. [5] | Useful background, not present-day ground truth. |

No current, openly licensed, citywide origin–destination matrix or public GTFS feed was verified in this research. Official portals and a traffic-flow dataset listing are leads, not yet production inputs. The traffic-flow listing did not establish a usable license. [6]

### Motorcycle behavior

HCMC-specific research studies non-lane-based motorcycle movement using observed trajectories. [7] SUMO's sublane model supports two-wheelers alongside each other, lateral movement, and virtual lanes; its documentation explicitly warns that finer lateral resolution costs runtime. [8]

**Decision:** use a lightweight lateral-space model rather than strict lane-center following. Use SUMO as an optional offline comparison tool, not the shipped simulation runtime initially. Comparison with SUMO is a consistency check, not proof of realism.

### Local restrictions

The city's official summary of Decision 23/2018 documents restricted entry into the defined urban area: light trucks 06:00–09:00 and 16:00–20:00; heavy trucks 06:00–22:00, with boundaries and exceptions. It defines those truck groups using **permitted cargo mass**, not simply actual gross mass. [9]

The official legal record lists subsequent changes. **These are historical reference rules, not a verified current legal configuration.** Freeze an explicit scenario date and review amendments, corridors, permits, and signage before calling a preset authentic.

**Decision:** restrictions must support time windows, geographic zones, exceptions, and distinct mass fields. “Ban trucks” cannot be just a road boolean.

### Infrastructure

FHWA guidance supports diagnosing a bottleneck before choosing a treatment; access management and operational improvements can matter as much as added capacity. Induced travel is relevant when evaluating widening. [10]

**Decision:** road upgrades are not a universal linear ladder. A flyover can move a queue downstream; widening can remove sidewalks or require land acquisition; a bus lane can improve person-throughput while slowing cars. This is transferable engineering guidance, not a substitute for Vietnamese design standards.

### Police and corruption

An IGC study of a Ghanaian police salary reform found that increased pay without equivalent anti-corruption enforcement did not reduce bribery. This is evidence against a universal salary-only rule, **not evidence about Vietnamese officers**. [11]

**Decision:** fictionalized systemic mechanics: pay adequacy, workload, supervision, accountability, and opportunities influence outcomes. Do not encode nationality or poverty as inherent dishonesty.

## 3. Player experience

The player manages a transport district with a starting infrastructure allocation and periodic public funding.

### Core loop

1. **Observe:** identify queues, unreliable buses, dangerous crossings, late deliveries, or inaccessible destinations.
2. **Diagnose:** inspect turning flows, demand, downstream blockage, curb activity, signal phases, and restrictions.
3. **Intervene:** change rules, retime signals, deploy staff, or commission construction.
4. **Evaluate:** compare the same demand period and seed against a baseline, including surrounding streets.
5. **Sustain:** pay payroll and maintenance, protect access and trust, and prepare for the next demand peak.

Winning means moving **people and goods safely and reliably**, not maximizing average car speed. Blocking every entrance must never count as success.

### Main interface

- **Map tools:** select roads/junctions, corridor selection, rule schedules, cross-section preview, project footprints, undo before confirmation.
- **Traffic overlays:** flow, queue length, travel-time reliability, restrictions, and pedestrian/conflict hotspots; use icons and patterns as well as color.
- **Inspector:** road dimensions, allowed movements, signal timing, observed demand, queue causes, and data confidence.
- **Management panels:** treasury/forecast, projects, police shifts/pay/training, and trust/complaints.
- **Controls:** pause, speed steps, orbit/pan/zoom, reset north, keyboard alternatives, readable Vietnamese street labels.

Provide explanations such as “queue cannot discharge because the next junction is blocked,” rather than a mysterious congestion percentage. Distinguish measured simulation facts from estimated intervention benefits.

## 4. Road users and demand

### Vehicle taxonomy

| Class | Distinct behavior or constraint |
|---|---|
| Motorcycles/scooters | Narrow bodies, lateral filtering, individual desired speed and gap preferences; delivery/taxi roles can share vehicle physics. |
| Mopeds/electric two-wheelers | Separate speed/power profiles when useful. “Motorbike” and “motorcycle” are not automatically different simulation classes. |
| Cars/taxis/vans | Wider occupancy, queues, curb stops, parking/drop-off demand. |
| Buses | Fixed stops/routes, dwell time, passenger load, turning needs, high person-throughput. |
| Freight vehicles | Light/medium/heavy gameplay profiles, separate permitted payload, actual total mass, dimensions, and bridge/zone eligibility. |
| Three-wheelers | Distinguish powered cargo tricycles from pedal cyclos; different speed, width, purpose, and access. |
| Bicycles | Low speeds, edge alignment, safe-passing space, vulnerability to heavy vehicles. |
| Pedestrians | Sidewalk/crossing graph, crossing waits, bus-stop access, conflict exposure. |

Vietnamese law distinguishes mopeds and motorcycles; legal classifications should be maintained separately from visual variants and physics profiles. [12] Actual three-wheeler restrictions and prevalence need local review; do not scatter them uniformly just because assets exist.

### Demand model

Use time-dependent origin–destination demand between homes, jobs, schools, markets, industrial/logistics sites, and boundary gateways. Land-use tags and buildings are imperfect proxies, not census counts.

Initial demand is synthetic and seedable. Calibrate route travel times, class counts, turning movements, and queue discharge against legally usable observations when available. Allow AM/PM peaks, school-release events, bus schedules, and freight delivery windows.

Track a trip's outcome: completed, waiting to enter, active, unreachable, or abandoned. Restriction-induced unmet demand remains visible and penalized. Never improve statistics by silently deleting stuck vehicles.

Initially keep mode shares fixed within a scenario so interventions are understandable. Add bounded departure-time/mode shifts and induced demand later, on a slower daily timescale. City growth and land-use simulation are out of initial scope.

Pedestrians are not decoration. They need continuous paths and crossing service; buses must carry modeled passengers rather than merely appearing as large cars. First-version passenger accounting can be aggregate per stop and bus, not an individually simulated population.

## 5. Traffic simulation

### Network representation

Use three linked representations:

| Layer | Purpose |
|---|---|
| Physical network | Road centerlines, widths, cross-sections, elevation layers, sidewalks, junctions, bridge spans. |
| Legal movement network | Directed travel edges, turn connectors, vehicle/time restrictions, crossing permissions. |
| Operational state | Occupancy, queues, signals, incidents, construction, staff assignments. |

A road crossing in map coordinates is not necessarily a junction. Bridges and tunnels must retain their connectivity and layer distinctions. Keep bridge structure limits separate from road access policies.

### Motion model

Longitudinal following uses speed, reaction gap, braking capability, and leader constraints. Laterally, agents occupy width across a small set of road strips while moving continuously across them. A car can block several strips; motorcycles can share space only where their bodies and safety gaps fit.

Use per-road longitudinal buckets or sorted occupancy to find nearby agents. Avoid all-to-all collision checks. Calibrate following and lateral decisions together; copying parameters from unrelated car traffic is insufficient.

Junctions need turn paths, conflict zones, signals or yield priorities, and downstream storage checks. These are simulation bookkeeping, not perfect cooperative reservations granted by real drivers. Controlled gap acceptance provides plausible interactions; explicit violation behavior is a later extension.

Normal vehicles should not overlap. Model incidents and near-miss exposure without a rigid-body crash engine. Conflict metrics are game safety indicators, not validated crash forecasts.

### Routing and policy changes

Use class-aware A* with a conservative distance/time heuristic. Route costs include expected delay and turn costs; mass/access prohibitions are eligibility checks, not merely large penalties.

Evaluate timed restrictions at projected traversal times. Revalidate at entry as actual conditions change. A later ban should not instantly invalidate physical occupancy: scenario policy defines whether existing occupants may exit, wait, or reroute. Handle windows crossing midnight and permit exemptions.

Cache routes by origin/destination zone, class, time bucket, and network version. Stagger rerouting to avoid every driver switching routes simultaneously. Expose unreachable trips and prevent road edits that strand agents without a deliberate handling policy.

Road conversion follows **preview → validation → funded project → closure/drain → rebuild → reopen**. Use network versioning and invalidate affected routes. Do not mutate occupied road geometry underneath moving vehicles.

### Time and scale

Start with a **20 Hz fixed simulation tick** and interpolated rendering. This is a benchmark hypothesis; braking/gap tests may require a smaller timestep or localized substeps. Economic time and traffic time must have an explicit relationship: changing game speed accelerates ticks, not vehicle physics constants.

Initial target: **5,000 active road users**, with load tests at 1,000/5,000/10,000, and 60 FPS at 1080p on an agreed reference machine. Count pedestrian and vehicle workloads separately. Record p50/p95 tick time, frame time, route work, and memory.

Rendering LOD is safe early; camera-dependent simulation shortcuts are not. Later, use a fixed detailed district surrounded by a mesoscopic network preserving flows, queues, vehicle classes, and travel time. Micro/meso boundaries must conserve demand and respect backpressure. Do not promise whole-city microscopic simulation before benchmarks.

## 6. Rules and infrastructure

### Rule tools

| Tool | Considerations |
|---|---|
| One-way/two-way and turn restrictions | Preserve local access; show detour effects and affected destinations. |
| Lane allocation | General traffic, bus priority, two-wheeler/bicycle allocation where scenario rules permit, turn pockets. |
| Class/time/mass restrictions | Road or zone scope, exemptions, signed schedule, enforcement and compliance. |
| Signal timing and priority | Phase sequence, green split, coordination, protected turns, pedestrian clearance; reject conflicting phases. |
| Speed, parking, loading and curb rules | Loading capacity and business access matter; enforcement can displace illegal parking elsewhere. |

Start with one-way conversion, class/time bans, a constrained signal editor, and curb restrictions. Add reversible lanes only after safe clearance and schedule transitions work.

### Infrastructure choices

| Intervention | What it can solve | Trade-off |
|---|---|---|
| Signal retiming/coordination | Poor green allocation, stop-start corridors | Limited by downstream capacity and pedestrian needs. |
| Turn pockets/channelization/median changes | Through traffic blocked by turns, conflicting movements | Needs physical space; can impose long detours. |
| Loading bays and bus-stop changes | Curb obstruction, buses blocking discharge | Uses scarce curb space; can worsen walking distance. |
| Sidewalks, crossings, refuge islands, bicycle facilities | Access, crossing safety, modal alternatives | Requires width and signal time, not just decoration. |
| Road widening | Genuine capacity deficit on a corridor | Land acquisition, displaced frontage, construction, maintenance, possible induced demand. |
| Bus priority lanes/signals | Low person-throughput and unreliable service | Reallocates space; needs adequate service and passenger demand. |
| Flyover/underpass | Specific crossing conflicts at a major junction | Ramp length/grade, clearance, turning radius, structures, drainage, visual impact, downstream queues. |
| New connection/bridge | Missing connectivity or overloaded river crossing | Expensive land/structural constraints, new traffic distribution, long construction. |

A **flyover** carries selected movements above an intersection; a **pedestrian overpass** serves a different purpose and must consider accessibility. Neither is a magic upgrade for every road.

Use cross-section templates for the first editor. Templates specify sidewalks, curb space, vehicle lanes, median and total width. Keep width conservation exact. Ordinary widening requires a feasible footprint; acquisition and compensation are explicit abstract costs, not claims about actual cadastral ownership.

Later grade-separated projects use constrained ramp/span templates, with clearance and connectivity validation. Start with one supported flyover configuration rather than an unrestricted Cities: Skylines-style spline editor. Exact engineering dimensions require standards review; the game must not present itself as a civil-design tool.

Projects have capital cost, construction duration, temporary capacity loss, upkeep, and commissioning. Starting a project reserves its budget. Inform the player about lifecycle cost before confirmation.

## 7. Public finance, police, and trust

### Budget

Use a configurable opening grant plus a periodic public allocation. Display VND with readable million/billion units, but identify initial prices as balancing values, not actual municipal procurement estimates.

`next_cash = cash + allocation - payroll - maintenance - construction_payments - training_and_oversight`

Prevent double spending by separating cash, committed funds, and available funds. Forecast recurring obligations. Avoid fine revenue as the primary positive feedback loop: it encourages policing for extraction rather than mobility and safety.

A constrained recovery grant, project postponement, and service reductions make failure recoverable without making insolvency meaningless. Do not automatically dismiss staff or erase infrastructure with no warning.

### Police force

Begin with squads and shifts, not hundreds of individual personnel records. Player decisions cover recruitment, compensation bands, training, assignment, and oversight.

| System | Gameplay effect |
|---|---|
| Pay and vacancies | Recruitment, retention, morale, financial pressure; recurring costs. |
| Staffing and workload | Fatigue, coverage, incident response, capacity to direct traffic. |
| Training and task assignment | Junction control versus parking enforcement versus incident response. |
| Oversight and rotation | Detection and deterrence of misconduct; costs staff time and money. |
| Public trust | Compliance, complaints, legitimacy, and willingness to cooperate. |

Directing an intersection can temporarily improve discharge; it cannot remove a physical bottleneck or stack infinitely with more officers. Manual control replaces or coordinates with signal control, never creates contradictory commands.

Model bribery as an uncertain institutional risk influenced by opportunity, pressure, supervision, accountability, and expected consequences. Poor pay can increase pressure, but **high pay does not guarantee integrity**. Discovered misconduct can trigger investigation, removal, retraining, and trust loss. Explain drivers to the player without labeling actual people or agencies.

Civil unrest is a late-stage, fictionalized consequence of sustained access failures, unfair burdens, and legitimacy loss—not an automatic reaction to a low salary slider. Start with complaints, falling compliance, and service disruption. Add demonstrations only if player testing supports a readable, respectful system; never reward suppression.

## 8. Map production pipeline

1. **Acquire and pin:** download Vietnam OSM PBF; record retrieval date, checksum, source URL, license, and chosen scenario boundary.
2. **Clip with buffer:** extract a connected district and surrounding routing area; preserve required relation members and connect boundary gateways.
3. **Normalize:** project coordinates to local meters, parse roads/restrictions/layers/buildings/water, split real junctions, generate directed movements.
4. **Audit and curate:** flag missing widths, lane counts, turn restrictions, sidewalks, and signal plans; apply documented assumptions and reviewed overrides.
5. **Bake:** generate road meshes and simulation data, check topology and permissions, package a versioned map with attribution and provenance.

Use osmium tooling rather than writing a PBF decoder. A small Python importer using pyosmium/projection tooling is a justified offline exception to the Odin runtime; confirm that mixed-language tooling is acceptable before implementation. Avoid building a general GIS platform.

Every important attribute gets an origin: **observed, OSM-tagged, inferred, or scenario-authored**. Imported topology is not automatically correct. Audit connected components, false bridge joins, duplicated edges, inaccessible stops, and disconnected footpaths.

Use procedural building footprints with approximate heights where reliable height tags are missing. A few landmark models and Vietnamese signage provide identity at much lower cost than a photorealistic city. Preserve diacritics and bundle an appropriately licensed font with tested glyph coverage.

OSM licensing does not automatically force unrelated game code to be open source. However, derivative database and produced-work distinctions need review, especially for redistributed baked maps and player exports. Keep game code and map licensing separate, preserve attribution, and plan a compliant data-release mechanism. Merely putting edits in another file does not settle their legal status. Do not copy Google Maps imagery, geometry, or traffic data without permission. [2]

## 9. Odin + raylib architecture

Odin supplies maintained `vendor:raylib` bindings, including mesh instancing and ray picking. [13] This supports the rendering foundation; it does not provide a traffic simulation, GIS importer, road editor, or complete management UI.

### Minimal package boundaries

| Package | Responsibility |
|---|---|
| `sim` | Network, demand, agents, routing, signals, rules; no raylib dependency. |
| `game` | Treasury, projects, police, objectives, clock, commands. |
| `render` | Chunk meshes, instanced vehicles, camera, picking, overlays. |
| `ui` | Inspectors, tools, graphs, interaction and accessibility. |
| `tools` / `tests` | Map baking, headless scenario runs, validation and benchmark fixtures. |

Use typed IDs with generations where needed, contiguous arrays for hot simulation state, and explicit ownership. Do not introduce a generic ECS, job framework, scripting language, or editor plugin system without a demonstrated need.

Start single-threaded, deterministic on the same build/platform. Use seeded random streams, stable iteration order, fixed ticks, and a command log. Do not promise cross-platform bit-identical floating-point replay. Thread routing or independent regions only after profiling, with stable result application.

Render procedural roads and buildings in spatial chunks with frustum culling. Instance repeated vehicles by mesh/material; use suitable instancing shaders and verify actual driver behavior. Use low-detail distant objects and limit shadows before sophisticated rendering work. Keep world/map coordinates in double precision during preprocessing and use local/chunk-relative floats for rendering.

Saving must capture network edits, simulation clock, treasury commitments, project phases, agents/routes, demand queues, RNG state, and map/schema version. Write to a temporary file and atomically replace after success. Reject incompatible saves clearly rather than silently losing state. Do not ship a public save-compatibility promise before the model stabilizes.

### Local environment verified

The workspace was empty and is not yet a Git repository. `odin version` returned `dev-2026-10:84bc3fc21`. Python 3 is on PATH; `osmium` and `sumo` are not. No raylib window, graphics driver, build, or throughput test has been run. Pin a tested compiler/library combination before development; installed does not mean validated.

## 10. Scope and delivery gates

Estimates assume **one experienced full-time engineer**, stylized assets, one desktop platform initially, and periodic local-domain/art help. They exclude major data licensing/procurement delays. They are planning ranges, not delivery commitments; re-estimate after the technical prototype.

| Phase | Estimated effort | Deliverable and exit condition |
|---|---:|---|
| A — data/technical spike | 2–3 weeks | Window/camera; headless sim; one synthetic junction with cars and side-by-side motorcycles; tiny real-map import; benchmark and provenance audit. Proceed only if model and tooling are viable. |
| B — traffic vertical slice | 5–8 weeks | Connected district, all requested road-user families at basic fidelity, buses/crossings, demand, queues, signals, timed restrictions, overlays. Bottlenecks and rerouting respond plausibly. |
| C — management MVP | 5–8 weeks | Treasury, projects, constrained widening, staff shifts/pay/oversight, objectives, save/load. A complete scenario is playable from opening grant to success/failure. |
| D — reviewable alpha | 6–10 weeks | Better district calibration, corruption/trust feedback, polished explanations, one validated flyover template, construction disruption, performance and accessibility pass. |
| E — expansion | Separately estimated | More districts, hybrid micro/meso scale, richer transit/logistics, weather/flood events, broader editing and scenario content. |

**Total to the proposed alpha: 18–29 engineer-weeks.** A polished commercial release is a separate undertaking involving content, QA, balancing, platform support, and user testing; this is not a whole-city shipping estimate.

### First scenario proposal

**Central market afternoon peak:** choose a connected patch around Bến Thành and neighboring streets after data audit. Familiarity is a selection reason, not a claim that this is today's worst bottleneck.

The player handles commuter traffic, a bus corridor, loading activity, pedestrian crossings, and a limited allocation. Retiming, loading windows, staff assignment, and a small physical upgrade compete for funds. Scenario-authored conditions must be labeled rather than advertised as an exact current reconstruction.

Run a baseline and intervention against the same peak demand. Require improvement in travel-time reliability and completed person-trips without unacceptable access loss, pedestrian delay, safety exposure, or insolvency. Set numerical thresholds only after the baseline is measured.

### Explicit initial exclusions

- Entire enlarged HCMC at full microscopic detail; live traffic prediction.
- General city-building, zoning, household life simulation, multiplayer.
- Photorealistic buildings and rigid-body vehicle crashes.
- Unrestricted interchange construction, terrain sculpting, and deep utility engineering.
- Detailed individual police careers, political simulation, or crowd-control gameplay.

## 11. Verification and acceptance

### Small deterministic scenarios

| Check | Must demonstrate |
|---|---|
| Narrow road with mixed widths | Motorcycles can travel abreast where feasible; cars cannot pass through occupied bodies. |
| Red signal and blocked downstream link | Queues form, spill back, and recover without normal collisions or unexplained teleportation. |
| Truck ban spanning midnight | Correct class/payload interpretation, exemptions, boundary time behavior, rerouting and unmet demand. |
| Occupied one-way conversion | Safe drain/closure, no opposing spawns, no invalid route references. |
| Crossings and bus stop | Pedestrians receive service, buses dwell and board, passenger throughput is counted. |
| Flyover above a crossing | No false ground-level connection; ramps and clearance validated. |
| Insufficient budget/project cancellation | No double spending; commitments and refunds follow explicit rules. |
| Save/reload and replay | Same-build state continuity, valid references, preserved RNG/demand and treasury. |

Add performance runs and invariant checks from the first simulation milestone. Important invariants include finite coordinates/speeds, valid graph IDs, population accounting, nonnegative occupied capacity, valid signal conflicts, and no recurring costs disappearing after reload.

For local calibration, measure class-specific counts, turning proportions, queue lengths, discharge rates, bus dwell, pedestrian wait, and travel-time distributions. Use separate observations for tuning and evaluation. Report coverage and error, not a vague “realistic” label.

For gameplay testing, ask whether players can explain why a queue exists, predict a trade-off, and distinguish temporary relief from long-term investment. Verify that wider roads, blanket bans, and maximum police deployment are not universal winning strategies.

## 12. Decisions to approve before implementation

1. **Scope:** historic central HCMC district first, with synthetic/calibrated demand; no whole-city digital-twin promise.
2. **Simulation:** custom Odin mixed-traffic model; optional offline SUMO comparison; all requested road-user families represented early.
3. **Management:** transparent public budget and squad-level staffing; pay plus accountability rather than a deterministic corruption slider.
4. **Editor:** constrained cross-sections first; one flyover template in alpha; freeform infrastructure later.
5. **Delivery:** Linux technical prototype on the current machine, then choose Windows/Linux release support; allow Python/GIS tools offline and approve the phase-A gate before committing to the full schedule.

## Sources and evidence boundaries

Sources below were researched for this proposal. Historical dates are called out where relevant. Documentation availability does not establish performance, and a dataset listing does not establish license, quality, or access.

[1] Geofabrik, Vietnam downloads: https://download.geofabrik.de/asia/vietnam.html

[2] OpenStreetMap copyright/license and linked attribution/legal requirements: https://www.openstreetmap.org/copyright and https://osmfoundation.org/wiki/Licence/Attribution_Guidelines

[3] Vietnam Government, Resolution 202/2025/QH15, provincial restructuring: https://chinhphu.vn/?classid=1&docid=213930&orggroupid=1&pageid=27160

[4] World Bank project document reporting historical 2010 modal split: https://documents1.worldbank.org/curated/en/099113023105535536/pdf/BOSIB04955d8ff0c208b8f0adb6e23b62a1.pdf

[5] JICA HOUTRANS report catalogue (2004): https://openjicareport.jica.go.jp/710/710/710_123_11764644.html

[6] Data discovery leads, not approved production sources: https://data.hochiminhcity.gov.vn/ ; https://www.giaothong.hochiminhcity.gov.vn/ ; https://ckan.transport-data.org/dataset/traffic-flow-data-in-ho-chi-minh-city-viet-nam-hochiminhcity/resource/9e5ba67b-cfb3-44f8-ac34-fdef7343e1a6

[7] TRID research abstracts: Nguyen et al., Traffic Conflict Assessment for Non-lane-based Movements of Motorcycles under Congested Conditions (2014), https://trid.trb.org/View/1305381 ; Describing Non–Lane-Based Motorcycle Movements in Motorcycle-Only Traffic Flow (2011), https://trid.trb.org/View/1130185 . Abstracts support the modeling direction; full papers must be reviewed before reproducing methods or calibration.

[8] SUMO sublane documentation, directly inspected: https://sumo.dlr.de/docs/Simulation/SublaneModel.html

[9] HCMC Official Gazette, summary of Decision 23/2018, directly inspected: https://congbao.hochiminhcity.gov.vn/tin-tuc-tong-hop/quy-%C4%91inh-ve-han-che-va-cap-phep-o-to-cho-hang-o-to-tai-luu-thong-trong-khu-vuc-noi-%C4%91o-thanh-pho-ho-chi-minh ; legal record and amendment links: https://www.congbao.hochiminhcity.gov.vn/cong-bao/van-ban/quyet-dinh/so/23-2018-qd-ubnd/ngay/19-07-2018/43178

[10] FHWA bottleneck guidance: https://ops.fhwa.dot.gov/bn/lbr.htm ; access management: https://highways.fhwa.dot.gov/safety/proven-safety-countermeasures/corridor-access-management ; travel/land-use and induced demand: https://www.environment.fhwa.dot.gov/nepa/Travel_LandUse/travel_landUse_rpt.aspx

[11] International Growth Centre, Ghana police-pay study summary, directly inspected: https://www.theigc.org/collections/can-raising-salaries-reduce-police-corruption-ghana

[12] Vietnam Road Traffic Order and Safety Law 36/2024/QH15, government text: https://xaydungchinhsach.chinhphu.vn/toan-van-luat-trat-tu-an-toan-giao-thong-duong-bo-119240909105718285.htm ; English translation: https://english.luatvietnam.vn/an-ninh-trat-tu/law-on-road-traffic-order-and-safety-2024-no-36-2024-qh15-360844-d1.html

[13] Odin raylib package reference, directly inspected for instancing and picking APIs: https://pkg.odin-lang.org/vendor/raylib/
