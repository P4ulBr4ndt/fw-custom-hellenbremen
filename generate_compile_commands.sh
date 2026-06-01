#!/usr/bin/env bash
# Generates compile_commands.json for clangd with correct absolute paths for the current machine.
# Run once after cloning: bash generate_compile_commands.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sed "s|__PROJECT_ROOT__|$SCRIPT_DIR|g" \
    "$SCRIPT_DIR/compile_commands_template.json" \
    > "$SCRIPT_DIR/compile_commands.json"

echo "Generated compile_commands.json for $SCRIPT_DIR"
