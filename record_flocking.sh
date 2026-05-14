#!/bin/bash
# Record flocking experiment data.
#
# Usage:
#   bash record_flocking.sh [num_drones] [experiment_name]
#
# Examples:
#   bash record_flocking.sh                 # 5 drones, auto name
#   bash record_flocking.sh 9               # 9 drones, auto name
#   bash record_flocking.sh 5 leader_test1  # 5 drones, named "leader_test1"

NUM_DRONES=${1:-5}
NAME=${2:-"run_$(date +%Y%m%d_%H%M%S)"}

OUT_DIR=~/flocking_logs
mkdir -p "$OUT_DIR"
cd "$OUT_DIR" || exit 1

# Build topic list
TOPICS=()
TOPICS+=("/leader/state")

for i in $(seq 0 $((NUM_DRONES - 1))); do
    if [ "$i" -eq 0 ]; then
        PREFIX="/fmu"
    else
        PREFIX="/px4_${i}/fmu"
    fi
    TOPICS+=("${PREFIX}/out/vehicle_local_position")
    TOPICS+=("${PREFIX}/out/vehicle_attitude")
    TOPICS+=("${PREFIX}/in/trajectory_setpoint")
done

echo "Recording ${NUM_DRONES} drones to: ${OUT_DIR}/${NAME}"
echo "Topics:"
for t in "${TOPICS[@]}"; do
    echo "    $t"
done
echo ""
echo "Press Ctrl+C to stop recording."
echo ""

ros2 bag record -o "$NAME" "${TOPICS[@]}"