#!/usr/bin/env python3
"""Rough register-to-register logic depth of a PolarFire netlist.

Input: yosys JSON written after `synth_microchip` (cells CFG1..4, ARI1, SLE,
RAM1K20, RAM64x12, MACC_PA, ...).  Every LUT counts 1 level, an ARI1 counts
1 level from its A..D inputs and CARRY from the carry input (fast carry
chain).  Flip-flops, RAMs, math blocks and ports start / end the paths.

  pf_depth.py netlist.json [top] [--paths N]

This is only a relative measure for finding long paths; the real timing
comes from Libero (SmartTime).
"""
import json
import sys
from collections import defaultdict

CARRY = 0.1      # FCI -> FCO / S, relative to one LUT level

SEQ_PREFIX = ("SLE", "RAM", "MACC", "CLKINT", "INBUF", "OUTBUF", "BUFG")


def main():
    argv = sys.argv[1:]
    npaths = 5
    if "--paths" in argv:
        i = argv.index("--paths")
        npaths = int(argv[i + 1])
        del argv[i:i + 2]
    args = argv
    d = json.load(open(args[0]))
    mods = d["modules"]
    top = args[1] if len(args) > 1 else next(
        m for m, v in mods.items() if v.get("attributes", {}).get("top"))
    mod = mods[top]
    edges = defaultdict(list)      # bit -> [(bit, delay, cell)]
    indeg = defaultdict(int)
    nodes = set()
    ends = {}      # bit -> sequential cell (or port) it ends at
    for name, c in mod["cells"].items():
        t = c["type"]
        dirs = c.get("port_directions", {})
        conns = c["connections"]
        ins = [(p, b) for p, bits in conns.items() if dirs.get(p) == "input" for b in bits if isinstance(b, int)]
        outs = [(p, b) for p, bits in conns.items() if dirs.get(p) == "output" for b in bits if isinstance(b, int)]
        if t.startswith(SEQ_PREFIX) or not outs:
            for _, b in ins:
                ends[b] = (name, [b2 for _, b2 in outs])
                nodes.add(b)
            for _, b in outs:
                nodes.add(b)
            continue
        for pi, bi in ins:
            for po, bo in outs:
                dl = 1.0
                if t == "ARI1" and pi == "FCI":
                    dl = CARRY
                edges[bi].append((bo, dl, name + ":" + t))
                indeg[bo] += 1
                nodes.add(bi)
                nodes.add(bo)
    for p, v in mod["ports"].items():
        if v["direction"] == "output":
            for b in v["bits"]:
                if isinstance(b, int):
                    ends[b] = (p, [])
    # longest path (DAG)
    arr = {n: 0.0 for n in nodes}
    prev = {}
    order = []
    deg = dict(indeg)
    stack = [n for n in nodes if deg.get(n, 0) == 0]
    while stack:
        n = stack.pop()
        order.append(n)
        for m, dl, cell in edges.get(n, ()):
            if arr[n] + dl > arr[m]:
                arr[m] = arr[n] + dl
                prev[m] = (n, cell)
            deg[m] -= 1
            if deg[m] == 0:
                stack.append(m)
    if len(order) != len(nodes):
        print("warning: combinational loop(s), %d nodes not ordered" % (len(nodes) - len(order)))
    names = {}
    for nm, v in mod["netnames"].items():
        for i, b in enumerate(v["bits"]):
            if isinstance(b, int) and (b not in names or nm.startswith("\\") or not nm.startswith("$")):
                names[b] = "%s[%d]" % (nm, i)
    best = sorted(((arr[e], e) for e in ends if e in arr), reverse=True)
    print("%s: max depth %.1f levels" % (top, best[0][0] if best else 0))
    shown = set()
    for dep, e in best:
        if len(shown) >= npaths:
            break
        path = []
        n = e
        while n in prev:
            p, cell = prev[n]
            path.append(names.get(n, str(n)))
            n = p
        start = names.get(n, str(n))
        cell, qs = ends[e]
        end = names.get(qs[0], cell) if qs else cell
        key = (start.split("[")[0], end.split("[")[0])
        if key in shown:
            continue
        shown.add(key)
        print("  %.1f  %s  ->  %s (%d cells)" % (dep, start, end, len(path)))


if __name__ == "__main__":
    main()
