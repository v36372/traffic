ODIN ?= odin
LOCAL_ODIN = ODIN_ROOT="$(CURDIR)/build/odin-root" $(ODIN)
ODINFLAGS = -define:RAYLIB_SHARED=false
MAP ?= showcase
CAPTURE_MAP ?= build/maps/showcase.csv

.PHONY: deps build run maps run-map test check smoke capture clean

deps: build/vendor/raylib/.ready

build/vendor/raylib/.ready: tools/bootstrap_raylib.sh
	ODIN="$(ODIN)" bash tools/bootstrap_raylib.sh
	touch $@

build: deps maps
	mkdir -p build
	$(LOCAL_ODIN) build src/app -out:build/traffic-game $(ODINFLAGS)

run: build
	./build/traffic-game

maps:
	mkdir -p build/maps
	@for source in maps/*.xml; do \
		python3 tools/build_map.py "$$source" --output "build/maps/$$(basename "$$source" .xml).csv" || exit $$?; \
	done

run-map: build maps
	./build/traffic-game --map "build/maps/$(MAP).csv"

test: deps maps
	mkdir -p build
	$(LOCAL_ODIN) test src/sim -out:build/sim-tests
	$(LOCAL_ODIN) test src/sim/showcase_test -out:build/showcase-tests
	$(LOCAL_ODIN) test src/mapdata -out:build/map-tests
	$(LOCAL_ODIN) test src/integration -out:build/integration-tests
	$(LOCAL_ODIN) test src/app -out:build/app-tests $(ODINFLAGS)
	python3 -m unittest discover -s tools -p 'test_*.py'

check: deps
	$(LOCAL_ODIN) check src/app $(ODINFLAGS)

smoke: build
	./build/traffic-game --smoke-test

capture: build
	mkdir -p build/visual
	./build/traffic-game --map "$(CAPTURE_MAP)" --smoke-test --screenshot build/visual/current.png

clean:
	rm -f build/traffic-game build/sim-tests build/showcase-tests build/map-tests build/integration-tests build/app-tests
