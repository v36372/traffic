ODIN ?= odin
LOCAL_ODIN = ODIN_ROOT="$(CURDIR)/build/odin-root" $(ODIN)
ODINFLAGS = -define:RAYLIB_SHARED=false
MAP ?= intersection
CAPTURE_MAP ?= build/maps/intersection.csv
EDITOR_ARGS ?=
CASE ?= build/debug-case.cbor
TIME ?= 250
DRIVER ?= 98

.PHONY: deps build run debug-build debug editor record-case replay-case maps run-map test check smoke capture clean

deps: build/vendor/raylib/.ready

build/vendor/raylib/.ready: tools/bootstrap_raylib.sh
	ODIN="$(ODIN)" bash tools/bootstrap_raylib.sh
	touch $@

build: deps maps
	mkdir -p build
	$(LOCAL_ODIN) build src/app -out:build/traffic-game $(ODINFLAGS) -o:speed

run: build
	./build/traffic-game

debug-build: deps
	$(LOCAL_ODIN) build src/replay/cli -out:build/replay-case -o:speed

debug: debug-build maps
	./build/replay-case --session "build/maps/$(MAP).csv"

editor: build
	./build/traffic-game --editor $(EDITOR_ARGS)

record-case: debug-build maps
	./build/replay-case --record "build/maps/$(MAP).csv" "$(TIME)" "$(DRIVER)" "$(CASE)"

replay-case: debug-build
	./build/replay-case "$(CASE)"

maps:
	mkdir -p build/maps
	@for source in maps/*.xml; do \
		python3 tools/build_map.py "$$source" --output "build/maps/$$(basename "$$source" .xml).csv" || exit $$?; \
	done

run-map: build maps
	./build/traffic-game --map "build/maps/$(MAP).csv"

test: deps maps
	# Match the native game's optimizer; retain bounds and test memory checks.
	mkdir -p build
	$(LOCAL_ODIN) test src/sim -out:build/sim-tests -o:speed
	$(LOCAL_ODIN) test src/sim/showcase_test -out:build/showcase-tests -o:speed
	$(LOCAL_ODIN) test src/mapdata -out:build/map-tests -o:speed
	$(LOCAL_ODIN) test src/replay -out:build/replay-tests -o:speed
	$(LOCAL_ODIN) test src/replay/cli -out:build/debug-cli-tests -o:speed
	$(LOCAL_ODIN) test src/integration -out:build/integration-tests -o:speed
	$(LOCAL_ODIN) test src/app -out:build/app-tests $(ODINFLAGS) -o:speed
	python3 -m unittest discover -s tools -p 'test_*.py'

check: deps
	$(LOCAL_ODIN) check src/app $(ODINFLAGS)

smoke: build
	./build/traffic-game --smoke-test

capture: build
	mkdir -p build/visual
	./build/traffic-game --map "$(CAPTURE_MAP)" --smoke-test --screenshot build/visual/current.png

clean:
	rm -f build/traffic-game build/sim-tests build/showcase-tests build/map-tests build/replay-tests build/replay-case build/debug-cli-tests build/integration-tests build/app-tests
