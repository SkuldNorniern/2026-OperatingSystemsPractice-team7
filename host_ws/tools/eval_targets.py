#!/usr/bin/env python3
"""Score a run's targets.csv against the real marker positions.

usage: eval_targets.py TARGETS.csv TRUTH.csv [--tol 0.25]

TRUTH.csv: id,x,y  (real marker centres in the map frame, m; measured by tape from the map origin).
Reports per truth id: found or missed (FN), position error; and every confirmed id not in the truth
list as a false positive (FP). Ids in the truth file are the real targets; anything else confirmed is FP.
"""
import csv, math, sys

def read(path):
    with open(path) as f:
        return {int(r["id"]): (float(r["x"]), float(r["y"])) for r in csv.DictReader(f) if r.get("id")}

def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    tol = float(sys.argv[sys.argv.index("--tol") + 1]) if "--tol" in sys.argv else 0.25
    found, truth = read(args[0]), read(args[1])
    fn = fp = 0
    for i, (tx, ty) in sorted(truth.items()):
        if i in found:
            fx, fy = found[i]
            err = math.hypot(fx - tx, fy - ty)
            ok = err <= tol
            print(f"id {i:4d}: found at ({fx:.2f}, {fy:.2f}), error {err*100:.0f} cm {'OK' if ok else 'POSITION OFF'}")
        else:
            fn += 1
            print(f"id {i:4d}: MISSED (false negative)")
    for i in sorted(set(found) - set(truth)):
        fp += 1
        print(f"id {i:4d}: confirmed but not a real target (false positive) at ({found[i][0]:.2f}, {found[i][1]:.2f})")
    print(f"summary: {len(truth) - fn}/{len(truth)} real found, {fn} missed, {fp} false positives")

if __name__ == "__main__":
    main()
