#!/bin/bash
# The QNN HTP stub talks to the NPU through the host's FastRPC library (libcdsprpc).
# docker-compose mounts the host library folder at /host-lib; expose libcdsprpc from there.
set -e
if [ -d /host-lib ]; then
    for lib in /host-lib/libcdsprpc.so*; do
        [ -e "$lib" ] || continue
        ln -sf "$lib" /usr/local/lib/libcdsprpc.so
        echo "Using FastRPC library: $lib"
        break
    done
    ldconfig
fi

# The DSP loads the QNN skel (libQnnHtpV68Skel.so) and, for an unsigned process, the shell
# (fastrpc_shell_unsigned_3) through FastRPC from ADSP_LIBRARY_PATH. The board's DSP folder comes
# from the host's /usr/share/qcom (e.g. qcs6490/radxa/dragon-q6a/dsp/cdsp); the image ships no
# /usr/share/qcom/conf.d that would let FastRPC find it by itself.
dsp_paths=()
qnn_libs=$(python3 -c 'import os, qai_appbuilder; print(os.path.join(os.path.dirname(qai_appbuilder.__file__), "libs"))' 2>/dev/null || true)
[ -d "$qnn_libs" ] && dsp_paths+=("$qnn_libs")
shell=$(find /usr/share/qcom /usr/lib/dsp /dsp -name fastrpc_shell_unsigned_3 -path '*cdsp*' 2>/dev/null | head -1 || true)
if [ -n "$shell" ]; then
    dsp_paths+=("$(dirname "$shell")")
else
    echo "WARNING: no fastrpc_shell_unsigned_3 from the host (/usr/share/qcom) - the NPU cannot start"
fi
if [ ${#dsp_paths[@]} -gt 0 ]; then
    ADSP_LIBRARY_PATH="$(IFS=';'; echo "${dsp_paths[*]}")${ADSP_LIBRARY_PATH:+;$ADSP_LIBRARY_PATH}"
    export ADSP_LIBRARY_PATH
    echo "ADSP_LIBRARY_PATH=$ADSP_LIBRARY_PATH"
fi

source /opt/ros/jazzy/setup.bash
exec python3 /app/depth_fusion_node.py --ros-args \
    -p model_path:="${OMNIMOW_MODEL_PATH:-/models/MowerAIn_seg_htp_context.bin}"
