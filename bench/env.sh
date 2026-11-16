#!/usr/bin/env bash
# Print the benchmark environment: machine, OS, compiler, flags, commit.
# Usage: bench/env.sh [build_dir] > env.txt
set -u
BUILD=${1:-build/release}
echo "date:      $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "commit:    $(git rev-parse --short HEAD)$(git diff --quiet HEAD -- core bench || echo ' (dirty)')"
if [ "$(uname)" = "Darwin" ]; then
  echo "cpu:       $(sysctl -n machdep.cpu.brand_string)"
  echo "cores:     $(sysctl -n hw.physicalcpu) physical / $(sysctl -n hw.logicalcpu) logical"
  echo "memory:    $(( $(sysctl -n hw.memsize) / 1073741824 )) GiB"
  echo "os:        macOS $(sw_vers -productVersion) ($(uname -m))"
else
  echo "cpu:       $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2 | sed 's/^ //')"
  echo "cores:     $(nproc) logical; threads/core $(lscpu | awk -F: '/Thread\(s\) per core/{gsub(/ /,"",$2);print $2}')"
  echo "caches:    $(lscpu | awk -F: '/L1d|L2|L3/{gsub(/^ +/,"",$2); printf "%s=%s; ", $1, $2}')"
  echo "memory:    $(awk '/MemTotal/{printf "%.1f GiB", $2/1048576}' /proc/meminfo)"
  echo "os:        $(. /etc/os-release; echo "$PRETTY_NAME"), kernel $(uname -r)"
  echo "virt:      $(systemd-detect-virt 2>/dev/null || echo unknown); hypervisor flag: $(grep -qw hypervisor /proc/cpuinfo && echo yes || echo no)"
  gov=/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
  echo "governor:  $( [ -r $gov ] && cat $gov || echo 'n/a (not exposed)')"
fi
echo "loadavg:   $(uptime | sed 's/.*load average[s]*: //')"
CXX=$(grep -m1 CMAKE_CXX_COMPILER:FILEPATH "$BUILD/CMakeCache.txt" 2>/dev/null | cut -d= -f2)
echo "compiler:  $($CXX --version 2>/dev/null | head -1)"
echo "flags:     $(grep -m1 '"command"' "$BUILD/compile_commands.json" 2>/dev/null | grep -o ' -[OWfgmsD][^ ]*' | tr -d '\n')"
echo "build:     $(grep -m1 CMAKE_BUILD_TYPE:STRING "$BUILD/CMakeCache.txt" | cut -d= -f2)"
