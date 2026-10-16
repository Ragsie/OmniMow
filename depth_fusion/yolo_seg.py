#!/usr/bin/env python3
"""
YOLO segmentation pre- and post-processing without ROS or QNN dependencies.

Matches what Ultralytics does for an 800x800 model: letterbox resize with grey
(114) padding, RGB, 0-1 float input, and decoding of both output formats:
  * end-to-end (YOLO26 default, NMS-free): det (1, 300, 6 + nm) = x1 y1 x2 y2 score cls coeffs
  * classic (end2end=False):              det (1, 4 + nc + nm, N) = cx cy w h scores... coeffs
plus mask prototypes (1, nm, H/4, W/4) in NCHW or NHWC.
"""
import cv2
import numpy as np


class LetterboxMeta:
    """How the original image was placed into the square model input."""

    def __init__(self, orig_h, orig_w, size, scale, pad_x, pad_y):
        self.orig_h = orig_h
        self.orig_w = orig_w
        self.size = size
        self.scale = scale
        self.pad_x = pad_x
        self.pad_y = pad_y


def letterbox(img, size):
    """Same resize + padding as Ultralytics (and the export's calibration data)."""
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    new_w, new_h = int(round(w * r)), int(round(h * r))
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    top = (size - new_h) // 2
    left = (size - new_w) // 2
    padded = cv2.copyMakeBorder(resized, top, size - new_h - top, left, size - new_w - left,
                                cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return padded, LetterboxMeta(h, w, size, r, left, top)


def preprocess(img_bgr, size, nchw=True):
    """BGR image -> float32 RGB 0-1 tensor (1,3,S,S) or (1,S,S,3)."""
    padded, meta = letterbox(img_bgr, size)
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    if nchw:
        rgb = rgb.transpose(2, 0, 1)
    return np.ascontiguousarray(rgb[np.newaxis]), meta


def _split_outputs(outputs):
    """
    Find the detection tensor and the mask prototypes (as CHW) among the model outputs.

    Exports may split the detection head into several outputs over the same anchors, e.g.
    (4 + nc, N) + (nm, N) or (4, N) + (nc, N) + (nm, N); these are joined back into one
    (4 + nc + nm, N) tensor in that order.
    """
    protos, heads = None, []
    for out in outputs:
        arr = np.asarray(out, dtype=np.float32)
        if arr.ndim == 4:
            protos = arr[0]
        elif arr.ndim == 3:
            heads.append(arr[0])
        elif arr.ndim == 2:
            heads.append(arr)
    if not heads or protos is None:
        shapes = [np.asarray(o).shape for o in outputs]
        raise ValueError(f"Expected a detection and a prototype output, got shapes {shapes}")
    # Prototypes have few channels (32) and a large spatial size (200x200)
    if protos.shape[0] > protos.shape[2]:
        protos = protos.transpose(2, 0, 1)  # HWC -> CHW
    if len(heads) == 1:
        return heads[0], protos

    nm = protos.shape[0]
    # Channels first (C, N): the anchor count N is the larger dimension
    heads = [h if h.shape[1] >= h.shape[0] else h.T for h in heads]
    n = max(h.shape[1] for h in heads)
    heads = [h for h in heads if h.shape[1] == n]
    boxes = [h for h in heads if h.shape[0] == 4]
    coeffs = [h for h in heads if h.shape[0] == nm]
    rest = [h for h in heads if h.shape[0] not in (4, nm)]
    if len(coeffs) != 1:
        shapes = [np.asarray(o).shape for o in outputs]
        raise ValueError(f"Cannot find the {nm} mask coefficients among the outputs {shapes}")
    return np.concatenate(boxes + rest + coeffs, axis=0), protos


def _nms(boxes_xyxy, scores, classes, iou_thresh):
    """Class-aware NMS with OpenCV (offset boxes per class so classes never suppress each other)."""
    if len(boxes_xyxy) == 0:
        return np.zeros(0, dtype=int)
    offset = classes.astype(np.float32)[:, None] * 10000.0
    b = boxes_xyxy + offset
    xywh = np.column_stack([b[:, 0], b[:, 1], b[:, 2] - b[:, 0], b[:, 3] - b[:, 1]])
    keep = cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), 0.0, iou_thresh)
    return np.array(keep, dtype=int).reshape(-1)


def decode(outputs, meta, conf_thresh=0.25, iou_thresh=0.45, max_det=300):
    """
    Returns (detections, protos). Each detection is a dict with
    cls, score, box (x1,y1,x2,y2 in original image pixels), box_in (same in model input
    pixels) and coeffs (mask coefficients).
    """
    det, protos = _split_outputs(outputs)
    nm = protos.shape[0]

    rows_format = det.shape[0] > det.shape[1]
    if rows_format and det.shape[1] == 6 + nm and det.shape[0] <= 1000:
        # End-to-end head: already NMS-free and sorted
        boxes = det[:, :4]
        scores = det[:, 4]
        classes = det[:, 5].astype(int)
        coeffs = det[:, 6:6 + nm]
        keep = np.where(scores >= conf_thresh)[0][:max_det]
    else:
        pred = det if rows_format else det.T  # (N, 4 + nc + nm)
        nc = pred.shape[1] - 4 - nm
        if nc < 1:
            shapes = [np.asarray(o).shape for o in outputs]
            raise ValueError(f"Cannot infer number of classes from detection shape {det.shape} with {nm} "
                             f"mask channels (model outputs {shapes})")
        cls_scores = pred[:, 4:4 + nc]
        classes = cls_scores.argmax(axis=1)
        scores = cls_scores[np.arange(len(pred)), classes]
        cand = np.where(scores >= conf_thresh)[0]
        cxcywh = pred[cand, :4]
        boxes = np.empty_like(pred[:, :4])
        boxes[cand] = np.column_stack([
            cxcywh[:, 0] - cxcywh[:, 2] / 2, cxcywh[:, 1] - cxcywh[:, 3] / 2,
            cxcywh[:, 0] + cxcywh[:, 2] / 2, cxcywh[:, 1] + cxcywh[:, 3] / 2,
        ])
        coeffs = pred[:, 4 + nc:]
        keep = cand[_nms(boxes[cand], scores[cand], classes[cand], iou_thresh)]
        keep = keep[np.argsort(-scores[keep])][:max_det]

    detections = []
    for i in keep:
        x1, y1, x2, y2 = boxes[i]
        orig = np.array([
            (x1 - meta.pad_x) / meta.scale, (y1 - meta.pad_y) / meta.scale,
            (x2 - meta.pad_x) / meta.scale, (y2 - meta.pad_y) / meta.scale,
        ])
        orig[[0, 2]] = orig[[0, 2]].clip(0, meta.orig_w)
        orig[[1, 3]] = orig[[1, 3]].clip(0, meta.orig_h)
        detections.append({
            "cls": int(classes[i]),
            "score": float(scores[i]),
            "box": orig,
            "box_in": np.array([x1, y1, x2, y2]),
            "coeffs": coeffs[i],
        })
    return detections, protos


def class_mask(detections, protos, meta, class_ids, mask_thresh=0.5):
    """Union of the instance masks of the given classes, as uint8 0/255 in original image size."""
    nm, ph, pw = protos.shape
    selected = [d for d in detections if d["cls"] in class_ids]
    if not selected:
        return np.zeros((meta.orig_h, meta.orig_w), dtype=np.uint8)

    # Each instance mask only inside its own box (cropped like Ultralytics does): computing the
    # mask over the box region alone is far cheaper than full-size masks for every detection
    thresh = np.log(mask_thresh / (1 - mask_thresh))
    sx, sy = pw / meta.size, ph / meta.size
    union = np.zeros((ph, pw), dtype=bool)
    for d in selected:
        x1, y1, x2, y2 = d["box_in"]
        xa, xb = (int(np.clip(np.ceil(v * sx), 0, pw)) for v in (x1, x2))
        ya, yb = (int(np.clip(np.ceil(v * sy), 0, ph)) for v in (y1, y2))
        if xb <= xa or yb <= ya:
            continue
        logits = np.tensordot(d["coeffs"], protos[:, ya:yb, xa:xb], axes=1)
        union[ya:yb, xa:xb] |= logits > thresh

    # Remove the letterbox padding, then scale back to the camera resolution
    left = int(round(meta.pad_x * sx))
    top = int(round(meta.pad_y * sy))
    right = int(round((meta.size - meta.pad_x) * sx))
    bottom = int(round((meta.size - meta.pad_y) * sy))
    cropped = union[top:bottom, left:right].astype(np.uint8) * 255
    return cv2.resize(cropped, (meta.orig_w, meta.orig_h), interpolation=cv2.INTER_NEAREST)
