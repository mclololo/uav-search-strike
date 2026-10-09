#!/usr/bin/env bash
# Source this file in the cloud desktop terminal before capture/training.
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "Use: source /home/ubuntu/catkin_ws/tools/fm9g_grounding/cloud_env.sh" >&2
    exit 2
fi
FM9G_EXPERIMENT=/home/ubuntu/model/experiments/fm9g4b_grounding
FM9G_DATA_MOUNT=/home/ubuntu/fm9g-data
PY=/home/ubuntu/.conda/envs/fm9g-train/bin/python
TOOL="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA="$FM9G_EXPERIMENT/data"
ANNO="$DATA/annotations"
if ! mountpoint -q "$FM9G_DATA_MOUNT"; then
    echo "Data disk is not mounted. Restore the bind mount via root SSH before capture/training." >&2
    return 1
fi
case "$(readlink -f "$DATA")" in
    "$FM9G_DATA_MOUNT"/*) ;;
    *) echo "Data path is outside the dedicated data mount: $DATA" >&2; return 1 ;;
esac
if [ ! -x "$PY" ] || [ ! -f "$TOOL/fcos_grounding.py" ]; then
    echo "Training environment or scripts are missing." >&2
    return 1
fi
if [ ! -w "$DATA/images" ] || [ ! -w "$ANNO" ] || [ ! -w "$FM9G_EXPERIMENT/checkpoints" ]; then
    echo "Images, annotations or checkpoints are not writable by the current user." >&2
    return 1
fi
FM9G_WORKSPACE="$(cd "$TOOL/../.." && pwd)"
source "$FM9G_WORKSPACE/devel/setup.bash" || return 1
cd "$FM9G_EXPERIMENT" || return 1
echo "FM9G_ENV_READY: PY=$PY TOOL=$TOOL DATA=$DATA"
