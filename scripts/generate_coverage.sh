#!/bin/bash
# Script to generate code coverage reports locally
# Usage: ./scripts/generate_coverage.sh [clean]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"
COVERAGE_DIR="${PROJECT_ROOT}/coverage-report"

echo "========================================"
echo "  ROS Project Coverage Generator"
echo "========================================"
echo ""

# Check for clean flag
if [ "$1" == "clean" ]; then
    echo "🧹 Cleaning previous coverage data..."
    cd "${PROJECT_ROOT}"
    catkin clean -y || true
    rm -rf "${COVERAGE_DIR}"
    rm -f "${PROJECT_ROOT}/coverage.info"
    rm -f "${PROJECT_ROOT}/coverage.xml"
    echo "✅ Clean complete"
    echo ""
fi

# Check dependencies
echo "📦 Checking dependencies..."
if ! command -v lcov &> /dev/null; then
    echo "❌ lcov not found. Installing..."
    sudo apt-get update
    sudo apt-get install -y lcov
fi

if ! command -v gcovr &> /dev/null; then
    echo "❌ gcovr not found. Installing..."
    sudo apt-get install -y gcovr
fi
echo "✅ Dependencies OK"
echo ""

# Source ROS
echo "🔧 Sourcing ROS..."
source /opt/ros/noetic/setup.bash
echo "✅ ROS sourced"
echo ""

# Configure for coverage
echo "⚙️  Configuring catkin for coverage..."
cd "${PROJECT_ROOT}"
catkin config \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Coverage \
    -DCMAKE_CXX_FLAGS="--coverage -fprofile-arcs -ftest-coverage" \
    -DCMAKE_C_FLAGS="--coverage -fprofile-arcs -ftest-coverage" \
    -DCMAKE_EXE_LINKER_FLAGS="--coverage"
echo "✅ Configuration complete"
echo ""

# Build
echo "🔨 Building packages..."
catkin build --no-status --summarize
echo "✅ Build complete"
echo ""

# Run tests
echo "🧪 Running tests..."
source "${PROJECT_ROOT}/devel/setup.bash"
catkin run_tests --no-status --summarize || true
echo "✅ Tests complete"
echo ""

# Generate coverage report
echo "📊 Generating coverage report..."

# Capture coverage data
echo "  → Capturing coverage data..."
lcov --capture \
    --directory "${BUILD_DIR}" \
    --output-file "${PROJECT_ROOT}/coverage.info" \
    --rc lcov_branch_coverage=1

# Remove system and external files
echo "  → Filtering system files..."
lcov --remove "${PROJECT_ROOT}/coverage.info" \
    '/opt/*' \
    '/usr/*' \
    '*/test/*' \
    '*/tests/*' \
    '*/_deps/*' \
    '*/CMakeFiles/*' \
    '*/msg/*' \
    '*/srv/*' \
    '*/action/*' \
    --output-file "${PROJECT_ROOT}/coverage.info"

# Generate summary
echo ""
echo "📈 Coverage Summary:"
echo "========================================"
lcov --summary "${PROJECT_ROOT}/coverage.info" 2>&1 | grep -E "(lines|functions|branches)"
echo "========================================"
echo ""

# Generate HTML report
echo "  → Generating HTML report..."
mkdir -p "${COVERAGE_DIR}"
genhtml "${PROJECT_ROOT}/coverage.info" \
    --output-directory "${COVERAGE_DIR}" \
    --demangle-cpp \
    --rc genhtml_branch_coverage=1 \
    --legend \
    --title "ROS Project Coverage Report"

# Generate XML report for tools
echo "  → Generating XML report..."
gcovr \
    --root "${PROJECT_ROOT}" \
    --build-dir "${BUILD_DIR}" \
    --xml-pretty \
    --output "${PROJECT_ROOT}/coverage.xml" \
    --filter 'src/.*' \
    --exclude '.*test.*' \
    --exclude '.*_deps/.*' \
    --exclude '.*CMakeFiles/.*'

echo ""
echo "========================================"
echo "  ✅ Coverage Report Generated!"
echo "========================================"
echo ""
echo "📁 Output files:"
echo "  • coverage.info       - LCOV format"
echo "  • coverage.xml        - Cobertura format"
echo "  • coverage-report/    - HTML report"
echo ""
echo "🌐 View HTML report:"
echo "  file://${COVERAGE_DIR}/index.html"
echo ""
echo "💡 Tips:"
echo "  • Open coverage-report/index.html in browser"
echo "  • Use 'genhtml' to regenerate HTML from coverage.info"
echo "  • Use './scripts/generate_coverage.sh clean' for fresh build"
echo ""
