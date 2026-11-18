#!/usr/bin/env bash
# Builds the editor: the engine compiled to WebAssembly, its JavaScript bindings,
# and the page, all in editor/site/.
#
#     editor/build.sh
#
# Needs the wasm32-unknown-unknown target (`rustup target add wasm32-unknown-unknown`)
# and the `wasm-bindgen` command at exactly the version Cargo.lock pins the library
# to: the two have to agree to the patch release, so this reads the version out of
# the lock file and says so if the installed command is another.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
root=$(dirname "$here")
site="$here/site"

wanted=$(awk '/^name = "wasm-bindgen"$/ { getline; gsub(/"/, "", $3); print $3 }' "$root/Cargo.lock")
installed=$(wasm-bindgen --version 2>/dev/null | awk '{ print $2 }' || true)
if [ "$wanted" != "$installed" ]; then
  echo "wasm-bindgen $wanted is needed (found '${installed:-none}'):" >&2
  echo "    cargo install wasm-bindgen-cli --version $wanted --locked" >&2
  exit 1
fi

cargo build --manifest-path "$root/Cargo.toml" --release \
  -p roadgen-editor-wasm --target wasm32-unknown-unknown

rm -rf "$site"
mkdir -p "$site"
wasm-bindgen --target web --no-typescript --out-dir "$site/pkg" \
  "$root/target/wasm32-unknown-unknown/release/roadgen_editor_wasm.wasm"
cp "$here/index.html" "$here/editor.js" "$here/styles.css" "$site/"
ls -la "$site" "$site/pkg"
