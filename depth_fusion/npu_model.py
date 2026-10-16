#!/usr/bin/env python3
"""
Loads the segmentation model for OmniMow: the QNN context binary (.bin) on the Hexagon NPU
via Qualcomm QAI AppBuilder, or a .onnx file with onnxruntime on the CPU (for testing on a PC).
Used by depth_fusion_node.py.
"""
import os


NPU_UNREACHABLE = (
    "The model could not be created on the NPU. This is a system problem, not the model: the QNN "
    "runtime cannot open a FastRPC session on the CDSP (look for 'openSessionForPriority failed' / "
    "0x72 above). Check that 'fastrpc_test v68' passes first - CDSP running "
    "(/sys/class/remoteproc/*/state), its firmware loaded, the FastRPC daemon running and access "
    "to /dev/fastrpc-*.")


def init_qnn_runtime(log_level="ERROR"):
    """
    Configure QAI AppBuilder once per process (HTP backend). Call before loading a .bin.
    At WARN, QAI AppBuilder logs the timing of every inference (several lines per frame).
    """
    try:
        from qai_appbuilder import LogLevel, PerfProfile, ProfilingLevel, QNNConfig, Runtime
    except ImportError as e:
        raise SystemExit(
            "qai_appbuilder is not installed for this Python. On the Radxa (Ubuntu 24.04, Python 3.12):\n"
            "  sudo apt install python3-opencv python3-numpy python3-pip\n"
            "  pip3 install --user --break-system-packages qai-appbuilder==2.51.40\n"
            "(with sudo, run the script as 'sudo -E python3 ...' so root finds the user install)") from e
    QNNConfig.Config(Runtime.HTP, getattr(LogLevel, log_level), ProfilingLevel.OFF)


class NpuModel:
    """QNN context binary (.bin) on the Hexagon NPU. A .onnx file runs on CPU for testing on a PC."""

    def __init__(self, path):
        self.path = path
        self.mtime = os.path.getmtime(path)
        if path.endswith(".onnx"):
            import onnxruntime as ort
            self.session = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
            self.input_shape = list(self.session.get_inputs()[0].shape)
            self.backend = "onnxruntime-cpu"
        else:
            from qai_appbuilder import PerfProfile, QNNContext
            self.context = QNNContext("omnimow_seg", path)
            shapes = self.context.getInputShapes()
            if not shapes:
                self.context.release()
                raise RuntimeError(NPU_UNREACHABLE)
            self.input_shape = list(shapes[0])
            self.backend = "qnn-htp"
            # The perf profile can only be set once a model is loaded on the HTP
            PerfProfile.SetPerfProfileGlobal(PerfProfile.HIGH_PERFORMANCE)
        # The model export keeps the ONNX layout (NCHW) with --preserve_io, but support NHWC too
        self.nchw = self.input_shape[1] == 3
        self.size = int(self.input_shape[2] if self.nchw else self.input_shape[1])

    def infer(self, tensor):
        if self.backend == "onnxruntime-cpu":
            return self.session.run(None, {self.session.get_inputs()[0].name: tensor})
        return self.context.Inference([tensor])

    def release(self):
        if self.backend == "qnn-htp":
            self.context.release()
