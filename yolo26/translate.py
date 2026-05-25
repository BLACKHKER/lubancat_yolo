import torch
import torch.nn as nn

from ultralytics import YOLO
from ultralytics.nn.modules.head import Detect


PT_PATH = "yolo26n.pt"
ONNX_PATH = "yolo26n.onnx"
IMG_SIZE = 640
OPSET = 12


def _rknn_forward(self, x):
    """Return 3 raw branches [1, 84, H, W] with sigmoid applied to cls."""
    results = []
    for i in range(self.nl):
        box = self.cv2[i](x[i])              # [1, 4*reg_max, H, W]  (YOLO26: reg_max=1 -> 4)
        cls = self.cv3[i](x[i]).sigmoid()    # [1, nc, H, W]         (post-sigmoid for fp threshold)
        results.append(torch.cat((box, cls), 1))  # [1, 4+nc, H, W]
    return tuple(results)


Detect.forward = _rknn_forward


class RKNNWrapper(nn.Module):
    """Thin wrapper so torch.onnx.export sees 3 named outputs (not the YOLO export pipeline)."""

    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, x):
        return self.model(x)


if __name__ == "__main__":
    yolo = YOLO(PT_PATH)
    inner = yolo.model.eval()
    for m in inner.modules():
        if isinstance(m, Detect):
            m.export = True
            m.format = "onnx"
            if hasattr(m, "end2end"):
                m.end2end = False

    wrapper = RKNNWrapper(inner).eval()
    dummy = torch.zeros(1, 3, IMG_SIZE, IMG_SIZE)

    with torch.no_grad():
        outs = wrapper(dummy)
    print(f"forward outputs: {[tuple(o.shape) for o in outs]}")
    assert len(outs) == 3, f"expected 3 outputs, got {len(outs)}"
    for o in outs:
        assert o.ndim == 4 and o.shape[1] == 84, f"bad shape {tuple(o.shape)}"

    torch.onnx.export(
        wrapper,
        dummy,
        ONNX_PATH,
        opset_version=OPSET,
        input_names=["images"],
        output_names=["output0", "output1", "output2"],
        do_constant_folding=True,
        dynamic_axes=None,
    )
    print(f"Exported: {ONNX_PATH}")

    try:
        import onnx
        import onnxslim

        m = onnx.load(ONNX_PATH)
        m = onnxslim.slim(m)
        onnx.save(m, ONNX_PATH)
        print("Slimmed")
    except ImportError:
        pass
