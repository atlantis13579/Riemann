#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SRC_DIR="${ROOT_DIR}/Src"
CXX="${CXX:-g++}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
OUT_DIR="${ROOT_DIR}/Binaries/Linux/${BUILD_TYPE}"
OUT_LIB="${OUT_DIR}/libRiemann.so"

COMMON_FLAGS=(
	-m64
	-std=c++14
	-fPIC
	-I"${SRC_DIR}"
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

mapfile -d '' SOURCES < <(find "${SRC_DIR}" -type f -name '*.cpp' -print0 | sort -z)

if ((${#SOURCES[@]} == 0)); then
	echo "No source files found under ${SRC_DIR}" >&2
	exit 1
fi

mkdir -p "${OUT_DIR}"

echo "Building ${OUT_LIB}"
echo "  compiler: ${CXX}"
echo "  type:     ${BUILD_TYPE}"
echo "  sources:  ${#SOURCES[@]}"

"${CXX}" "${COMMON_FLAGS[@]}" ${CXXFLAGS:-} \
	-shared \
	-o "${OUT_LIB}" \
	"${SOURCES[@]}" \
	${LDFLAGS:-} \
	-pthread

echo "Built ${OUT_LIB}"
