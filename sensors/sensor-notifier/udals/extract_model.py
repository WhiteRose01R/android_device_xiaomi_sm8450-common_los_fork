#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 The LineageOS Project
# SPDX-License-Identifier: Apache-2.0
"""Extract schema-driven UDALS model parameters from a stock ELF using a device spec.

Reads only IEEE-754 double tables, never executable code. Device-specific ELF
digests, virtual addresses, and algorithm parameters live in each device tree's
spec rather than in sm8450-common.
"""

import argparse
import hashlib
import json
from pathlib import Path
import struct


class Elf:
    def __init__(self, path, image_base=0x100000):
        self.data = Path(path).read_bytes()
        if self.data[:6] != b"\x7fELF\x02\x01":
            raise ValueError("Expected a little-endian ELF64 file")
        self.sha256 = hashlib.sha256(self.data).hexdigest()
        self.image_base = image_base
        phoff = struct.unpack_from("<Q", self.data, 32)[0]
        entsize, count = struct.unpack_from("<HH", self.data, 54)
        self.segments = []
        for i in range(count):
            kind, _, offset, address, _, size, _, _ = struct.unpack_from(
                "<IIQQQQQQ", self.data, phoff + i * entsize
            )
            if kind == 1:
                self.segments.append((address, offset, size))

    def doubles(self, address, count=1):
        address -= self.image_base
        for base, offset, size in self.segments:
            if base <= address and address + count * 8 <= base + size:
                return list(
                    struct.unpack_from(f"<{count}d", self.data, offset + address - base)
                )
        raise ValueError(f"Unmapped model address {address:#x}")

    def scalar(self, address):
        return self.doubles(address)[0]


def _addr(value):
    return int(value, 0) if isinstance(value, str) else int(value)


def _resolve_scalar(elf, value):
    if isinstance(value, dict) and "addr" in value:
        return elf.scalar(_addr(value["addr"]))
    return float(value)


def _extract_network(elf, net_spec):
    base = _addr(net_spec["base"])
    widths = list(net_spec["layers"])
    outputs = widths[-1]
    layers = []
    inputs = 4
    for index, width in enumerate(widths):
        weights = elf.doubles(base, inputs * width)
        base += inputs * width * 8
        if index + 1 == len(widths) and "output_bias" in net_spec:
            biases = [_resolve_scalar(elf, v) for v in net_spec["output_bias"]]
        else:
            biases = elf.doubles(base, width)
            base += width * 8
        layers.append(
            {"inputs": inputs, "outputs": width, "weights": weights, "bias": biases}
        )
        inputs = width

    net = {
        "layers": layers,
        "exponent": [_resolve_scalar(elf, v) for v in net_spec["exponent"]],
        "divisor": (
            [_resolve_scalar(elf, v) for v in net_spec["divisor"]]
            if "divisor" in net_spec
            else [1.0] * outputs
        ),
        "negative": net_spec.get("negative", "allow"),
    }
    if "max_dbv" in net_spec and net_spec["max_dbv"] is not None:
        net["max_dbv"] = int(net_spec["max_dbv"])
    if "dc" in net_spec and net_spec["dc"] is not None:
        net["dc"] = bool(net_spec["dc"])
    return net


def _load_spec(spec):
    if isinstance(spec, (str, Path)):
        return json.loads(Path(spec).read_text())
    return spec


def extract(path, spec):
    spec = _load_spec(spec)
    elf = Elf(path, image_base=_addr(spec.get("image_base", 0x100000)))
    expected_sha = spec.get("sha256")
    if expected_sha and elf.sha256 != expected_sha:
        raise ValueError(
            f"Unsupported citsensor binary: got {elf.sha256}, expected {expected_sha}"
        )

    grid = int(spec["grid"])
    cells = grid * grid
    spatial = []
    for entry in spec["spatial"]:
        if isinstance(entry, dict):
            spatial.append(
                elf.doubles(_addr(entry["addr"]), int(entry.get("count", cells)))
            )
        else:
            spatial.append(elf.doubles(_addr(entry), cells))

    model = {
        "version": 1,
        "source_sha256": elf.sha256,
        "grid": grid,
        "block_size": int(spec.get("block_size", 5)),
        "gamma": _resolve_scalar(elf, spec["gamma"]),
        "spatial": spatial,
        "input_transform": spec["input_transform"],
        "routing": spec["routing"],
        "networks": [_extract_network(elf, n) for n in spec["networks"]],
    }
    if "sensor_owner" in spec:
        model["sensor_owner"] = spec["sensor_owner"]
    if "msg_unknown1" in spec:
        model["msg_unknown1"] = _resolve_scalar(elf, spec["msg_unknown1"])
    if "dc_curve_max_dbv" in spec:
        model["dc_curve_max_dbv"] = int(spec["dc_curve_max_dbv"])
    if "input_offset" in spec:
        model["input_offset"] = [
            _resolve_scalar(elf, v) for v in spec["input_offset"]
        ]
    if "input_divisor" in spec:
        model["input_divisor"] = [
            _resolve_scalar(elf, v) for v in spec["input_divisor"]
        ]
    if "lux" in spec:
        lux = {}
        for k, v in spec["lux"].items():
            lux[k] = _resolve_scalar(elf, v) if isinstance(v, dict) else v
        model["lux"] = lux
    if "filter" in spec:
        flt = {}
        for k, v in spec["filter"].items():
            if k == "deadband_tiers":
                tiers = []
                for tier in v:
                    out_tier = {"ratio": _resolve_scalar(elf, tier["ratio"])}
                    if "below" in tier and tier["below"] is not None:
                        out_tier["below"] = _resolve_scalar(elf, tier["below"])
                    tiers.append(out_tier)
                flt[k] = tiers
            else:
                flt[k] = _resolve_scalar(elf, v) if isinstance(v, dict) else v
        model["filter"] = flt
    return model


def fixup_model(ctx, file, file_path, spec, *args, **kwargs):
    model = extract(file_path, spec)
    Path(file_path).write_text(json.dumps(model, separators=(",", ":")) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("spec", help="Path to device UDALS extraction spec JSON")
    parser.add_argument("input", help="Path to stock citsensorservice ELF")
    parser.add_argument("output", help="Destination udals-model.json path")
    args = parser.parse_args()
    Path(args.output).write_text(
        json.dumps(extract(args.input, args.spec), separators=(",", ":")) + "\n"
    )
