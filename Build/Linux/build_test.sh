#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SRC_DIR="${ROOT_DIR}/Src"
TEST_DIR="${ROOT_DIR}/Test"
CXX="${CXX:-g++}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
BIN_DIR="${ROOT_DIR}/Binaries/Linux/${BUILD_TYPE}"
LIB_DIR="${BIN_DIR}"
OUT_TEST="${BIN_DIR}/test"

if [[ "${SKIP_SO:-0}" != "1" ]]; then
	bash "${SCRIPT_DIR}/build_so.sh"
fi

if [[ ! -f "${LIB_DIR}/libRiemann.so" ]]; then
	echo "Missing ${LIB_DIR}/libRiemann.so. Run ${SCRIPT_DIR}/build_so.sh first." >&2
	exit 1
fi

COMMON_FLAGS=(
	-m64
	-std=c++14
	-I"${SRC_DIR}"
	-I"${TEST_DIR}"
	-Wall
	-Wextra
	-Wno-invalid-offsetof
	-pthread
)

case "${BUILD_TYPE}" in
	Debug)
		COMMON_FLAGS+=(-O0 -g)
		;;
		Release)
		COMMON_FLAGS+=(-O2 -DNDEBUG)
		;;
		*)
		echo "Unsupported BUILD_TYPE='${BUILD_TYPE}' (expected Debug or Release)" >&2
		exit 1
		;;
esac

mapfile -d '' TEST_SOURCES < <(find "${TEST_DIR}" -type f -name '*.cpp' -print0 | sort -z)

if ((${#TEST_SOURCES[@]} == 0)); then
	echo "No test source files found under ${TEST_DIR}" >&2
	exit 1
fi

mkdir -p "${BIN_DIR}"

echo "Building ${OUT_TEST}"
echo "  compiler: ${CXX}"
echo "  type:     ${BUILD_TYPE}"
echo "  sources:  ${#TEST_SOURCES[@]}"

"${CXX}" "${COMMON_FLAGS[@]}" ${CXXFLAGS:-} \
	-o "${OUT_TEST}" \
	"${TEST_SOURCES[@]}" \
	-L"${LIB_DIR}" \
	-Wl,-rpath,'$ORIGIN' \
	-lRiemann \
	${LDFLAGS:-} \
	-pthread

echo "Built ${OUT_TEST}"
