"""작은 ONNX MLP(정책)를 numpy만으로 계산한다 (분석 PC에 onnxruntime이 없어도 기록을 재생하려고).

policies/real_walk/policy.onnx는 Gemm과 Elu만 쓰는 MLP(opset 11)다. onnx 패키지 없이 protobuf를 직접 읽는다.
지원하는 연산: Gemm, Elu, Relu, Tanh, Sigmoid, Identity, Add, Sub, Mul, Div, MatMul, Clip(속성형)
그 밖의 연산이 있으면 OnnxMlp()가 ValueError를 낸다 (분석 도구는 재생을 건너뛴다).
"""
import struct

import numpy as np

_DTYPES = {1: np.float32, 6: np.int32, 7: np.int64, 11: np.float64}


def _varint(buf, i):
    shift = result = 0
    while True:
        b = buf[i]
        i += 1
        result |= (b & 0x7F) << shift
        if not b & 0x80:
            return result, i
        shift += 7


def _fields(buf):
    """protobuf 메시지 → [(field 번호, wire type, 값)]. 값은 int(varint, fixed) 또는 bytes(길이 지정)."""
    i, out = 0, []
    while i < len(buf):
        key, i = _varint(buf, i)
        num, wt = key >> 3, key & 7
        if wt == 0:
            v, i = _varint(buf, i)
        elif wt == 1:
            v = buf[i:i + 8]
            i += 8
        elif wt == 2:
            n, i = _varint(buf, i)
            v = buf[i:i + n]
            i += n
        elif wt == 5:
            v = buf[i:i + 4]
            i += 4
        else:
            raise ValueError(f"protobuf wire type {wt} 지원 안 함")
        out.append((num, wt, v))
    return out


def _packed_varints(v):
    out, i = [], 0
    while i < len(v):
        x, i = _varint(v, i)
        out.append(x)
    return out


def _tensor(buf):
    dims, dtype, name, raw, floats = [], 1, "", None, []
    for num, wt, v in _fields(buf):
        if num == 1:
            dims += _packed_varints(v) if wt == 2 else [v]
        elif num == 2:
            dtype = v
        elif num == 4:
            floats += list(struct.unpack(f"<{len(v) // 4}f", v)) if wt == 2 else [struct.unpack("<f", v)[0]]
        elif num == 8:
            name = v.decode()
        elif num == 9:
            raw = v
    if dtype not in _DTYPES:
        raise ValueError(f"텐서 {name}: data_type {dtype} 지원 안 함")
    arr = np.frombuffer(raw, dtype=_DTYPES[dtype]).copy() if raw is not None else np.array(floats, _DTYPES[dtype])
    return name, arr.reshape(dims) if dims else arr


def _attribute(buf):
    name, val = "", None
    for num, wt, v in _fields(buf):
        if num == 1:
            name = v.decode()
        elif num == 2:
            val = struct.unpack("<f", v)[0]
        elif num == 3:
            val = v - (1 << 64) if v >= 1 << 63 else v
    return name, val


def _node(buf):
    ins, outs, op, attrs = [], [], "", {}
    for num, wt, v in _fields(buf):
        if num == 1:
            ins.append(v.decode())
        elif num == 2:
            outs.append(v.decode())
        elif num == 4:
            op = v.decode()
        elif num == 5:
            k, a = _attribute(v)
            attrs[k] = a
    return op, ins, outs, attrs


def _value_name(buf):
    for num, wt, v in _fields(buf):
        if num == 1:
            return v.decode()
    return ""


class OnnxMlp:
    SUPPORTED = {"Gemm", "Elu", "Relu", "Tanh", "Sigmoid", "Identity", "Add", "Sub", "Mul", "Div", "MatMul", "Clip"}

    def __init__(self, path):
        with open(path, "rb") as f:
            model = f.read()
        graph = next(v for num, wt, v in _fields(model) if num == 7)
        self.nodes, self.init, inputs, outputs = [], {}, [], []
        for num, wt, v in _fields(graph):
            if num == 1:
                self.nodes.append(_node(v))
            elif num == 5:
                name, arr = _tensor(v)
                self.init[name] = arr
            elif num == 11:
                inputs.append(_value_name(v))
            elif num == 12:
                outputs.append(_value_name(v))
        self.input_name = next(n for n in inputs if n not in self.init)
        self.output_name = outputs[0]
        bad = sorted({op for op, *_ in self.nodes} - self.SUPPORTED)
        if bad:
            raise ValueError(f"지원하지 않는 ONNX 연산: {bad}")
        self.ops = [op for op, *_ in self.nodes]

    def __call__(self, x):
        """x: (N, 입력 수) → (N, 출력 수), float32 (onnxruntime과 같게 float32로 계산)."""
        env = dict(self.init)
        env[self.input_name] = np.asarray(x, dtype=np.float32)
        for op, ins, outs, at in self.nodes:
            a = [env[n] if n else None for n in ins]
            if op == "Gemm":
                A = a[0].T if at.get("transA", 0) else a[0]
                B = a[1].T if at.get("transB", 0) else a[1]
                y = at.get("alpha", 1.0) * (A @ B)
                if len(a) > 2 and a[2] is not None:
                    y = y + at.get("beta", 1.0) * a[2]
            elif op == "Elu":
                al = at.get("alpha", 1.0)
                y = np.where(a[0] > 0, a[0], al * (np.exp(np.minimum(a[0], 0)) - 1.0))
            elif op == "Relu":
                y = np.maximum(a[0], 0)
            elif op == "Tanh":
                y = np.tanh(a[0])
            elif op == "Sigmoid":
                y = 1.0 / (1.0 + np.exp(-a[0]))
            elif op == "Identity":
                y = a[0]
            elif op == "Add":
                y = a[0] + a[1]
            elif op == "Sub":
                y = a[0] - a[1]
            elif op == "Mul":
                y = a[0] * a[1]
            elif op == "Div":
                y = a[0] / a[1]
            elif op == "MatMul":
                y = a[0] @ a[1]
            elif op == "Clip":
                lo = a[1] if len(a) > 1 and a[1] is not None else at.get("min", -np.inf)
                hi = a[2] if len(a) > 2 and a[2] is not None else at.get("max", np.inf)
                y = np.clip(a[0], lo, hi)
            env[outs[0]] = np.asarray(y, dtype=np.float32)
        return env[self.output_name]


if __name__ == "__main__":       # 점검: Radxa walk_policy.check_policy()와 같은 기준 출력
    import os
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "radxa", "policies", "real_walk", "policy.onnx")
    m = OnnxMlp(path)
    stand = np.array([0, 0, 0.2457719, -0.4, 0.1542281, 0, 0, 0, -0.2457719, 0.4, -0.1542281, 0], np.float32)
    obs = np.zeros((1, 45), np.float32)
    obs[0, 8] = -1.0                                  # 중력 (0, 0, -1), 관절 = 서기 자세, 나머지 0
    ref = np.array([-0.08326, 0.03751, -0.06501, 0.26643, 0.15041, 0.20079,
                    0.03692, -0.06964, -0.04834, -0.17642, -0.24063, -0.11180])
    out = m(obs)[0]
    print("ops:", m.ops)
    print("weights:", {k: v.shape for k, v in m.init.items()})
    print("max diff from reference:", float(np.abs(out - ref).max()))
