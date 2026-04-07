# -*- coding: utf-8 -*-
"""
Standalone geometry tests -- no Fusion 360 / adsk required.

Run from the repo root:
    python tests/test_geometry.py

Optionally produces a profile plot if matplotlib is installed:
    python tests/test_geometry.py --plot
"""
import sys, os, math

# Allow importing lib without adsk by stubbing it out
sys.modules.setdefault('adsk', type(sys)('adsk'))
sys.modules.setdefault('adsk.core', type(sys)('adsk.core'))
sys.modules.setdefault('adsk.fusion', type(sys)('adsk.fusion'))

# Add parent dir to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lib import cycloid_geometry as cg


def check(name, condition, detail=''):
    marker = 'PASS' if condition else 'FAIL'
    print('[%s] %s%s' % (marker, name, ('  -- ' + detail) if detail else ''))
    return condition


def run_tests(m=4.0, N=12, d_pin=4.0, rack_len=100.0, verbose=True):
    r_pin  = d_pin / 2.0
    R_pc   = N * m / 2.0
    pitch  = math.pi * m

    print('\n=== Parameters ===')
    print('  module m       = %.2f mm' % m)
    print('  num pins N     = %d' % N)
    print('  pin diameter   = %.2f mm  (r_pin=%.2f)' % (d_pin, r_pin))
    print('  R_pc           = %.4f mm' % R_pc)
    print('  pitch          = %.4f mm' % pitch)
    print()

    # ---------------------------------------------------------------
    # 1. Validation
    # ---------------------------------------------------------------
    errors = cg.validate_params(m, N, r_pin)
    check('validate_params returns no errors', errors == [], str(errors))

    # ---------------------------------------------------------------
    # 2. phi_0: where trochoid returns to Y=0 after the cusp
    # ---------------------------------------------------------------
    phi_0 = 2.0 * math.asin(min(r_pin / (2.0 * R_pc), 1.0))
    x_at_phi0 = R_pc*(phi_0 - math.sin(phi_0)) + r_pin*math.cos(phi_0/2.0)
    y_at_phi0 = R_pc*(1.0 - math.cos(phi_0)) - r_pin*math.sin(phi_0/2.0)
    check('Y is ~0 at phi_0', abs(y_at_phi0) < 1e-6,
          'Y(phi_0)=%.8f mm' % y_at_phi0)
    check('X at phi_0 is close to r_pin', abs(x_at_phi0 - r_pin) < 0.01,
          'X(phi_0)=%.6f mm, r_pin=%.6f mm, gap=%.6f mm' % (x_at_phi0, r_pin, x_at_phi0 - r_pin))

    # Arc radius used in rack_generator (computed from trochoid, not r_pin*1.05)
    r_valley_actual = (R_pc*(phi_0 - math.sin(phi_0)) + r_pin*math.cos(phi_0/2.0))
    gap = abs(x_at_phi0 - r_valley_actual)
    check('Arc/spline junction gap == 0 with computed r_valley',
          gap < 1e-9,
          'x_junction=%.8f, r_valley_computed=%.8f, gap=%.2e mm' % (
              x_at_phi0, r_valley_actual, gap))
    print('  >> r_valley (computed) = %.6f mm  (r_pin = %.6f mm, diff = %.6f mm)' % (
        r_valley_actual, r_pin, r_valley_actual - r_pin))

    # ---------------------------------------------------------------
    # 3. Flank tangent at phi_0 should be nearly vertical
    # ---------------------------------------------------------------
    # dX/dphi = R_pc*(1-cos(phi)) - r_pin/2*sin(phi/2)
    # dY/dphi = R_pc*sin(phi) - r_pin/2*cos(phi/2)
    dX = R_pc*(1.0-math.cos(phi_0)) - (r_pin/2.0)*math.sin(phi_0/2.0)
    dY = R_pc*math.sin(phi_0) - (r_pin/2.0)*math.cos(phi_0/2.0)
    angle_from_vertical = math.degrees(math.atan2(abs(dX), abs(dY)))
    check('Flank tangent at phi_0 is nearly vertical (< 5 deg from vertical)',
          angle_from_vertical < 5.0,
          'angle from vertical = %.4f deg' % angle_from_vertical)

    # ---------------------------------------------------------------
    # 4. right_flank_points: all Y >= 0 when starting from phi_0
    # ---------------------------------------------------------------
    rflank = cg.right_flank_points(R_pc, r_pin, pitch, num_samples=48)
    min_y = min(y for x, y in rflank)
    check('All right flank points have Y >= 0',
          min_y >= -1e-9,
          'min Y = %.8f mm' % min_y)

    # Spline start near (r_pin, 0) -- arc must end there too
    sx0, sy0 = rflank[0]
    check('Spline start is close to (r_pin, 0)',
          abs(sx0 - r_pin) < 0.01 and abs(sy0) < 1e-4,
          'spline[0] = (%.6f, %.8f) mm' % (sx0, sy0))

    # ---------------------------------------------------------------
    # 5. Tooth tip: X should be at pitch/2
    # ---------------------------------------------------------------
    tip_x, tip_y = rflank[-1]
    check('Tooth tip X == pitch/2', abs(tip_x - pitch/2.0) < 0.01,
          'tip=(%.4f, %.4f), pitch/2=%.4f' % (tip_x, tip_y, pitch/2.0))
    check('Tooth tip Y > 0', tip_y > 0,
          'tip_y=%.4f mm' % tip_y)

    # ---------------------------------------------------------------
    # 6. Profile symmetry (left flank = mirror of right)
    # ---------------------------------------------------------------
    lflank = cg.left_flank_points(R_pc, r_pin, pitch, num_samples=48)
    for i, ((rx, ry), (lx, ly)) in enumerate(zip(rflank, reversed(lflank))):
        if abs((pitch - lx) - rx) > 0.001 or abs(ly - ry) > 0.001:
            check('Left flank is X-mirror of right flank', False,
                  'mismatch at i=%d: rflank=(%.4f,%.4f) lflank_m=(%.4f,%.4f)' % (
                      i, rx, ry, pitch-lx, ly))
            break
    else:
        check('Left flank is X-mirror of right flank', True)

    # ---------------------------------------------------------------
    # 7. Pinion wedge spec
    # ---------------------------------------------------------------
    spec = cg.pinion_wedge_spec(m, N, r_pin)
    check('R_rim > R_pc', spec['R_rim'] > spec['R_pc'],
          'R_pc=%.2f, R_rim=%.2f' % (spec['R_pc'], spec['R_rim']))
    check('R_hub < R_pc', spec['R_hub'] < spec['R_pc'],
          'R_hub=%.2f, R_pc=%.2f' % (spec['R_hub'], spec['R_pc']))
    check('wedge_angle == 2pi/N', abs(spec['wedge_angle'] - 2*math.pi/N) < 1e-9)

    # ---------------------------------------------------------------
    # 8. Rack tooth count
    # ---------------------------------------------------------------
    tc = cg.rack_tooth_count(rack_len, m)
    actual_len = tc * pitch
    check('Rack tooth count > 0', tc > 0, 'tc=%d' % tc)
    check('Actual rack <= requested length', actual_len <= rack_len + 1e-6,
          'actual=%.2f mm, requested=%.2f mm' % (actual_len, rack_len))

    # ---------------------------------------------------------------
    # 9. right_flank_arcs: arc chain properties
    # ---------------------------------------------------------------
    r_valley = R_pc*(phi_0 - math.sin(phi_0)) + r_pin*math.cos(phi_0/2.0)
    arcs = cg.right_flank_arcs(R_pc, r_pin, pitch, num_arcs=8)
    check('right_flank_arcs returns 8 arcs', len(arcs) == 8)

    # Chain starts at (r_valley, 0)
    ax0, ay0 = arcs[0][0]
    check('Arc chain starts at (r_valley, 0)',
          abs(ax0 - r_valley) < 1e-9 and abs(ay0) < 1e-9,
          'start=(%.6f, %.6f)' % (ax0, ay0))

    # Chain is C0: each arc's end == next arc's start
    c0_ok = True
    for i in range(len(arcs) - 1):
        ex, ey = arcs[i][2]
        sx, sy = arcs[i+1][0]
        if abs(ex - sx) > 1e-6 or abs(ey - sy) > 1e-6:
            c0_ok = False
            break
    check('Arc chain is C0 (consecutive arcs share endpoints)', c0_ok)

    # Chain ends near (pitch/2, y_tip)
    last_end = arcs[-1][2]
    check('Arc chain ends at tooth tip (X=pitch/2)',
          abs(last_end[0] - pitch/2.0) < 0.01,
          'end_x=%.4f, pitch/2=%.4f' % (last_end[0], pitch/2.0))

    # G1 at valley junction: first arc centre must lie on y=0
    # Compute centre from 3 arc points via circumscribed circle
    def circumcenter(p1, p2, p3):
        ax, ay = p1; bx, by = p2; cx, cy = p3
        D = 2*(ax*(by-cy) + bx*(cy-ay) + cx*(ay-by))
        if abs(D) < 1e-12:
            return None, None
        ux = ((ax**2+ay**2)*(by-cy) + (bx**2+by**2)*(cy-ay) + (cx**2+cy**2)*(ay-by)) / D
        uy = ((ax**2+ay**2)*(cx-bx) + (bx**2+by**2)*(ax-cx) + (cx**2+cy**2)*(bx-ax)) / D
        return ux, uy
    cx0, cy0 = circumcenter(*arcs[0])
    check('First arc centre lies on y=0 (G1 with valley arc)',
          cy0 is not None and abs(cy0) < 1e-6,
          'centre_y=%.9f' % (cy0 if cy0 is not None else float('nan')))

    return rflank, lflank, spec


def plot_profile(rflank, lflank, m, r_pin, pitch, r_valley, base_depth):
    try:
        import matplotlib
        matplotlib.use('Agg')   # non-interactive, no Tk required
        import matplotlib.pyplot as plt
    except ImportError:
        print('\nmatplotlib not available -- skipping plot')
        return

    R_pc = 12 * m / 2.0   # N=12 default

    def arc_pts(cx, cy, r, a0, a1, n=64):
        return [(cx + r*math.cos(a0 + (a1-a0)*i/(n-1)),
                 cy + r*math.sin(a0 + (a1-a0)*i/(n-1))) for i in range(n)]

    def circumcenter(p1, p2, p3):
        ax, ay = p1; bx, by = p2; cx, cy = p3
        D = 2*(ax*(by-cy) + bx*(cy-ay) + cx*(ay-by))
        if abs(D) < 1e-12:
            return None, None, None
        ux = ((ax**2+ay**2)*(by-cy) + (bx**2+by**2)*(cy-ay) + (cx**2+cy**2)*(ay-by)) / D
        uy = ((ax**2+ay**2)*(cx-bx) + (bx**2+by**2)*(ax-cx) + (cx**2+cy**2)*(bx-ax)) / D
        r  = math.hypot(ax-ux, ay-uy)
        return ux, uy, r

    def arc_chain_pts(arc_list, n_per=32):
        pts = []
        for idx, (p_s, p_m, p_e) in enumerate(arc_list):
            cx, cy, r = circumcenter(p_s, p_m, p_e)
            if cx is None:
                pts += ([p_s] if idx == 0 else []) + [p_e]
                continue
            a_s = math.atan2(p_s[1]-cy, p_s[0]-cx)
            a_e = math.atan2(p_e[1]-cy, p_e[0]-cx)
            # Cross product of (p_m-p_s, p_e-p_s) determines CCW vs CW
            cross = ((p_m[0]-p_s[0])*(p_e[1]-p_s[1])
                   - (p_m[1]-p_s[1])*(p_e[0]-p_s[0]))
            if cross >= 0:   # CCW
                if a_e < a_s:
                    a_e += 2*math.pi
            else:            # CW
                if a_s < a_e:
                    a_s += 2*math.pi
            seg = arc_pts(cx, cy, r, a_s, a_e, n=n_per)
            pts += seg if idx == 0 else seg[1:]
        return pts

    arcs_right = cg.right_flank_arcs(R_pc, r_pin, pitch, num_arcs=8)
    arcs_left  = [
        ((pitch - x_e, y_e), (pitch - x_m, y_m), (pitch - x_s, y_s))
        for (x_s, y_s), (x_m, y_m), (x_e, y_e) in reversed(arcs_right)
    ]

    lv = arc_pts(0, 0, r_valley, -math.pi/2, 0)
    rv = arc_pts(pitch, 0, r_valley, math.pi, 3*math.pi/2)
    rf_arc = arc_chain_pts(arcs_right)
    lf_arc = arc_chain_pts(arcs_left)

    # Arc-chain profile (what Fusion builds)
    prof_arc = ([(0, -base_depth), (0, -r_valley)]
                + lv[1:]
                + rf_arc[1:]
                + lf_arc[1:]
                + rv[1:]
                + [(pitch, -base_depth), (0, -base_depth)])
    xs, ys = zip(*prof_arc)

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.fill(xs, ys, alpha=0.3, color='steelblue')
    ax.plot(xs, ys, 'b-', linewidth=1.5, label='Arc-chain profile (8 arcs/flank)')

    # True trochoid overlay
    true_pts = (list(reversed(lflank)) + rflank[1:])
    tx, ty = zip(*true_pts)
    ax.plot(tx, ty, 'r--', linewidth=0.8, label='True trochoid (reference)')

    # Pitch line
    ax.axhline(0, color='grey', linestyle=':', linewidth=0.8, label='Pitch line Y=0')

    # Valley circle (pin seat)
    theta = [2*math.pi*i/60 for i in range(61)]
    ax.plot([r_valley*math.cos(t) for t in theta],
            [r_valley*math.sin(t) for t in theta], 'g--', linewidth=0.8, label='Valley (r_valley)')

    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title('Arc-chain approximation  m=%.1f  N=12  r_pin=%.2f mm' % (m, r_pin))
    plt.tight_layout()
    plt.savefig('tooth_profile.png', dpi=150)
    print('\nPlot saved to tooth_profile.png')
    plt.show()


if __name__ == '__main__':
    m, N, r_pin = 4.0, 12, 2.0
    R_pc  = N * m / 2.0
    pitch = math.pi * m
    phi_0 = 2.0 * math.asin(min(r_pin / (2.0 * R_pc), 1.0))
    r_valley   = R_pc*(phi_0 - math.sin(phi_0)) + r_pin*math.cos(phi_0/2.0)
    base_depth = r_pin * 2.0
    rflank, lflank, spec = run_tests(m=m, N=N, d_pin=r_pin*2)
    if '--plot' in sys.argv:
        plot_profile(rflank, lflank, m, r_pin, pitch, r_valley, base_depth)
    print()
