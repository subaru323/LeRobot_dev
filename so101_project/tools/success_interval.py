"""Wilson 95% interval for a success count, to see what N evaluation trials can tell apart.

usage: python tools\\success_interval.py N [k ...]     (default: every k from 0 to N)
       python tools\\success_interval.py --compare N   (two runs of N trials: for each k1, the smallest k2
                                                       that Fisher's exact test, two-sided, puts below p = 0.05)
"""

import math
import sys


def fisher_two_sided(a, n1, c, n2):
    """p of a 2x2 table (a of n1 vs c of n2 successes), summing tables no more likely than the observed one."""
    total, m = n1 + n2, a + c

    def prob(x):
        return math.comb(n1, x) * math.comb(n2, m - x) / math.comb(total, m)

    observed = prob(a)
    lo, hi = max(0, m - n2), min(n1, m)
    return sum(prob(x) for x in range(lo, hi + 1) if prob(x) <= observed * (1 + 1e-9))


if sys.argv[1] == "--compare":
    n = int(sys.argv[2])
    print(f"N={n} vs N={n}  smallest k2 > k1 with Fisher exact two-sided p < 0.05")
    for k1 in range(n + 1):
        k2 = next((k for k in range(k1 + 1, n + 1) if fisher_two_sided(k1, n, k, n) < 0.05), None)
        print(f"  {k1:>3}/{n} -> " + (f"{k2}/{n} (p={fisher_two_sided(k1, n, k2, n):.3f})" if k2 is not None else "none"))
    sys.exit(0)

n = int(sys.argv[1])
ks = [int(k) for k in sys.argv[2:]] or list(range(n + 1))
z = 1.959964
print(f"N={n}  Wilson 95% interval of the success rate")
for k in ks:
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    print(f"  {k:>3}/{n}: {max(0.0, c - h):.2f} - {min(1.0, c + h):.2f}")
