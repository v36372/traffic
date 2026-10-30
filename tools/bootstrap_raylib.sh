#!/usr/bin/env bash
# Build pinned raylib 6.0 locally for the installed Odin 6.0 bindings; no system changes.
set -euo pipefail

cd "$(dirname "$0")/.."
case "$(uname -s):$(uname -m)" in
    Linux:x86_64) libdir=linux ;;
    Linux:aarch64) libdir=linux-arm64 ;;
    Darwin:arm64) libdir=macos ;;
    *) echo 'This bootstrap supports Linux x86_64/aarch64 and macOS arm64 only.' >&2; exit 1 ;;
esac

if [[ $libdir == macos ]]; then
    checksum=(shasum -a 256 -c)
    cmake_platform=()
else
    checksum=(sha256sum -c)
    cmake_platform=(-DGLFW_BUILD_WAYLAND=OFF -DGLFW_BUILD_X11=ON)
fi

odin=${ODIN:-odin}
# Ignore our generated ODIN_ROOT on repeat invocations.
odin_root=${ODIN_SOURCE_ROOT:-$(env -u ODIN_ROOT "$odin" root)}
odin_root=$(cd "$odin_root" && pwd -P)
bindings="$odin_root/vendor/raylib"
# Fail closed if the installed Odin bindings target another ABI.
grep -Eq '^VERSION_MAJOR :: 6$' "$bindings/raylib.odin"
grep -Eq '^VERSION_MINOR :: 0$' "$bindings/raylib.odin"
grep -Eq '^VERSION_PATCH :: 0$' "$bindings/raylib.odin"
test -f "$bindings/rlgl/rlgl.odin"
test -f "$bindings/LICENSE"

# Official 6.0 tag, commit dbc56a87da87d973a9c5baa4e7438a9d20121d28.
archive=build/deps/raylib-6.0.tar.gz
sha256=2b3ee1e2120c7a0796b33062c7e9a694dd8a8caa56a96319ac8c8ecf54a90d0b
mkdir -p build/deps
if [[ ! -f "$archive" ]]; then
    tmp=$(mktemp build/deps/raylib-download.XXXXXX)
    trap 'rm -f "$tmp"' EXIT
    curl --fail --location --retry 2 --output "$tmp" https://codeload.github.com/raysan5/raylib/tar.gz/refs/tags/6.0
    echo "$sha256  $tmp" | "${checksum[@]}" >/dev/null || { echo 'downloaded raylib archive checksum mismatch' >&2; exit 1; }
    mv "$tmp" "$archive"
    trap - EXIT
fi
echo "$sha256  $archive" | "${checksum[@]}" >/dev/null || { echo 'raylib archive checksum mismatch' >&2; exit 1; }

tar -xzf "$archive" -C build/deps
cmake -S build/deps/raylib-6.0 -B build/deps/raylib-6.0-build \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF \
    "${cmake_platform[@]}"
cmake --build build/deps/raylib-6.0-build --parallel

mkdir -p "build/vendor/raylib/$libdir" build/vendor/raylib/rlgl
cp "$bindings"/*.odin "$bindings/LICENSE" build/vendor/raylib/
cp "$bindings/rlgl/"*.odin build/vendor/raylib/rlgl/
cp build/deps/raylib-6.0-build/raylib/libraylib.a "build/vendor/raylib/$libdir/libraylib.a"
mkdir -p build/odin-root
ln -sfn "$odin_root/base" build/odin-root/base
ln -sfn "$odin_root/core" build/odin-root/core
ln -sfn ../vendor build/odin-root/vendor
printf '%s\n' 'raylib 6.0 ready. Build with ODIN_ROOT="$PWD/build/odin-root" odin build src/app -out:build/traffic-game -define:RAYLIB_SHARED=false'
