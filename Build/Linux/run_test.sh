#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_TYPE="${BUILD_TYPE:-Release}"
BIN_DIR="${ROOT_DIR}/Binaries/Linux/${BUILD_TYPE}"
TEST_BIN="${BIN_DIR}/test"
LIB_DIR="${BIN_DIR}"

if [[ ! -x "${TEST_BIN}" || ! -f "${LIB_DIR}/libRiemann.so" ]]; then
	bash "${SCRIPT_DIR}/build_test.sh"
fi

export LD_LIBRARY_PATH="${LIB_DIR}:${LD_LIBRARY_PATH:-}"
exec "${TEST_BIN}" "$@"
