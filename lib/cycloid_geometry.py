# -*- coding: utf-8 -*-
"""
Cycloidal Rack-and-Pinion Geometry Math Library
================================================

System definition (all units in millimetres):

  - A ROLLER PINION has N cylindrical pins (radius r_pin) whose centres sit
    on a pitch circle of radius R_pc = N * module / 2.
  - The RACK tooth profile is the envelope generated as each pin rolls along
    the rack pitch line.  The envelope is an OFFSET TROCHOID (curtate cycloid).

Coordinate system (for a single rack tooth + two half-valleys):
  - Y increases upward (tooth tips point in +Y).
  - Rack pitch line is at Y = 0.
  - One tooth profile spans X = [0 ... pitch], valleys centred at X = 0 and X = pitch.
  - Tooth tip is at X = pitch/2, Y = y_tip.

Offset-trochoid flank equations
--------------------------------
As the pinion rolls to the right by angle phi (pinion rotates clockwise), pin #0
(starting at the valley at X=0, Y=0) traces the following path in the rack frame:

    x_c(phi) = R_pc * (phi - sin phi)
    y_c(phi) = R_pc * (1 - cos phi)

This is the standard cycloid, with the pin CENTER at y_c=0 (valley) when phi=0.

The rack TOOTH FLANK is the envelope of the family of circles (centres on the
cycloid, radius r_pin).  The outward unit normal of the cycloid at phi is:

    n^(phi) = (cos(phi/2), -sin(phi/2))

(Derived from the unit tangent T = (sin(phi/2), cos(phi/2)) rotated -90deg.)

Offset profile (right slope, valley at x=0 -> tooth tip at x=pitch/2):

    X(phi) = R_pc * (phi - sin phi) + r_pin * cos(phi/2)    [phi: 0 -> phi_top]
    Y(phi) = R_pc * (1 - cos phi)  - r_pin * sin(phi/2)

At phi=0:   (X, Y) = (r_pin, 0)          -- pitch-line level, right edge of valley
At phi=phi_top: X = pitch/2                -- tooth centerline (tip)

The left slope (tooth tip -> right valley) is the X-mirror about x = pitch/2:
    X_L = pitch - X(phi),   Y_L = Y(phi),   phi: phi_top -> 0

Valley arc
-----------
The valley beneath the pitch line is a circular arc:
    centre (0, 0), radius r_pin*(1 + clearance_frac)
    Spanning angle 270deg -> 360deg (= from bottom to right, i.e. from (0,-r) to (r, 0))
    [In Python math: from -pi/2 to 0, generating points CCW]

Validity constraint
--------------------
For a valid (non-undercutting) geometry: r_pin < pitch/4.
With pitch = pi * module: r_pin < pi*module/4 ~ 0.785 * module.
Typical: pin_diameter = 0.5 ... 1.5 * module  ->  r_pin = 0.25 ... 0.75 * module.
"""

import math


# ---------------------------------------------------------------------------
# Core parameter helpers
# ---------------------------------------------------------------------------

def derived_params(module_mm, num_pins, r_pin_mm):
    """Return a dict of all derived geometric parameters."""
    m = module_mm
    N = num_pins
    r = r_pin_mm
    R_pc = N * m / 2.0          # pitch-circle radius (pin centres)
    pitch = math.pi * m         # circular pitch (rack tooth spacing)
    return dict(module=m, num_pins=N, r_pin=r, R_pc=R_pc, pitch=pitch)


def validate_params(module_mm, num_pins, r_pin_mm):
    """
    Return a list of human-readable error strings.
    Empty list means everything is valid.
    """
    errors = []
    m = module_mm
    N = num_pins
    r = r_pin_mm
    if m <= 0:
        errors.append("Module must be > 0.")
    if N < 6:
        errors.append("Number of pins must be >= 6.")
    if r <= 0:
        errors.append("Pin radius must be > 0.")
    if m > 0 and r > 0:
        pitch = math.pi * m
        if r >= pitch / 4.0:
            errors.append(
                f"Pin radius ({r:.3f} mm) must be less than pitch/4 "
                f"({pitch/4:.3f} mm) to avoid valley/flank overlap.  "
                f"Try a smaller pin diameter or larger module."
            )
        R_pc = N * m / 2.0
        # Pins must not overlap on the pitch circle.
        min_gap = 2 * math.pi * R_pc / N - 2 * r
        if min_gap < 0:
            errors.append(
                "Pins overlap on the pitch circle.  "
                "Reduce pin diameter or increase the number of pins / module."
            )
    return errors


# ---------------------------------------------------------------------------
# phi_top solver  (where flank X-coordinate equals pitch/2)
# ---------------------------------------------------------------------------

def _flank_X(phi, R_pc, r_pin):
    return R_pc * (phi - math.sin(phi)) + r_pin * math.cos(phi / 2.0)


def _find_phi_top(R_pc, r_pin, pitch, tol=1e-8):
    """
    Find the parameter phi_top where the right flank reaches X = pitch/2.
    Uses bisection.  Returns phi_top in radians.
    """
    target = pitch / 2.0
    # Upper bound: phi will certainly be less than 2pi (well within one arch)
    lo, hi = 1e-6, math.pi  # pitch/2 < piR_pc/N < pi*R_pc
    # Sanity check: at hi, X should exceed target
    if _flank_X(hi, R_pc, r_pin) < target:
        # Extend search
        hi = math.pi * 3
    for _ in range(80):
        mid = (lo + hi) / 2.0
        if _flank_X(mid, R_pc, r_pin) < target:
            lo = mid
        else:
            hi = mid
        if hi - lo < tol:
            break
    return (lo + hi) / 2.0


# ---------------------------------------------------------------------------
# Flank profile sampling
# ---------------------------------------------------------------------------

def right_flank_points(R_pc, r_pin, pitch, num_samples=72):
    """
    Compute the RIGHT SLOPE of the VALLEY-at-X=0 going UP to the tooth tip.

    Returns a list of (x, y) tuples in mm, from (r_pin, 0) to (pitch/2, y_tip).
    phi ranges from 0 to phi_top (found by bisection).

    Note: at phi=0 the offset cycloid has a cusp, so the first returned point
    is at phi = small_epsilon to avoid the exact cusp.  The valley arc (computed
    separately) handles the region right at the cusp.
    """
    phi_top = _find_phi_top(R_pc, r_pin, pitch)
    # The trochoid has a cusp at phi=0 and dips below Y=0 until phi_0.
    # phi_0 is the first phi>0 where Y returns to 0:
    #   R_pc*(1-cos(phi)) - r_pin*sin(phi/2) = 0  =>  phi_0 = 2*asin(r_pin/(2*R_pc))
    # Start sampling from phi_0 so the spline only contains points with Y>=0.
    phi_0 = 2.0 * math.asin(min(r_pin / (2.0 * R_pc), 1.0))
    phi_start = phi_0
    phis = [phi_start + (phi_top - phi_start) * i / (num_samples - 1)
            for i in range(num_samples)]
    pts = []
    for phi in phis:
        x = R_pc * (phi - math.sin(phi)) + r_pin * math.cos(phi / 2.0)
        y = R_pc * (1.0 - math.cos(phi)) - r_pin * math.sin(phi / 2.0)
        pts.append((x, y))
    return pts


def left_flank_points(R_pc, r_pin, pitch, num_samples=72):
    """
    Compute the LEFT SLOPE from the tooth tip DOWN to the VALLEY-at-X=pitch.

    Returns a list of (x, y) tuples, from (pitch/2, y_tip) to (pitch-r_pin, 0).
    (This is the X-mirror of right_flank_points, centred on pitch/2.)
    """
    rpts = right_flank_points(R_pc, r_pin, pitch, num_samples)
    return [(pitch - x, y) for (x, y) in reversed(rpts)]


def right_flank_arcs(R_pc, r_pin, pitch, num_arcs=8):
    """
    Approximate the right trochoid flank as a chain of (num_arcs) circular arcs.

    The chain starts at (r_valley, 0) -- tangent-continuous (G1) with the valley
    arc whose tangent at that point is vertical (+Y direction).
    The chain ends at (pitch/2, y_tip).

    Returns a list of (p_start, p_mid, p_end) tuples; each p_* is an (x, y) pair
    in mm.  Consecutive arcs share endpoints (C0 chain).

    The FIRST arc is constructed so its centre lies on y=0, which guarantees G1
    with the concave valley arc (centre at (0,0), tangent vertical at the shared
    endpoint).  Remaining arcs use direct trochoid sample triplets.
    """
    phi_0   = 2.0 * math.asin(min(r_pin / (2.0 * R_pc), 1.0))
    phi_top = _find_phi_top(R_pc, r_pin, pitch)

    def _pt(phi):
        x = R_pc * (phi - math.sin(phi)) + r_pin * math.cos(phi / 2.0)
        y = R_pc * (1.0 - math.cos(phi)) - r_pin * math.sin(phi / 2.0)
        return (x, y)

    # Sample at 2*num_arcs+1 evenly-spaced phi values so each arc gets a
    # genuine trochoid midpoint.
    n    = 2 * num_arcs + 1
    phis = [phi_0 + (phi_top - phi_0) * i / (n - 1) for i in range(n)]
    pts  = list(map(_pt, phis))

    # Snap the start point to exact (r_valley, 0) -- Y is already 0 at phi_0.
    r_valley = pts[0][0]
    pts[0]   = (r_valley, 0.0)

    arcs = []

    # First arc: G1 with valley arc.
    # Valley centre (0, 0); its tangent at (r_valley, 0) is +Y (vertical).
    # => first flank arc centre must lie on y = 0 (perpendicular to +Y at start).
    # Find c_x such that C=(c_x, 0) is equidistant from pts[0] and pts[2]:
    #   (x_s - c_x)^2 = (x_e - c_x)^2 + y_e^2
    #   => c_x = (x_e^2 + y_e^2 - x_s^2) / (2*(x_e - x_s))
    x_s, y_s = pts[0]
    x_e, y_e = pts[2]
    denom = 2.0 * (x_e - x_s)
    c_x   = (x_e ** 2 + y_e ** 2 - x_s ** 2) / denom if abs(denom) > 1e-9 else x_s
    c_y   = 0.0
    R_a   = abs(x_s - c_x)
    # Arc midpoint at the angular midpoint around C (sweeps CCW from start to end)
    ang_s = math.atan2(y_s - c_y, x_s - c_x)
    ang_e = math.atan2(y_e - c_y, x_e - c_x)
    # Short-arc midpoint: signed angular difference in (-pi, pi]
    diff  = ((ang_e - ang_s) + math.pi) % (2.0 * math.pi) - math.pi
    ang_m = ang_s + diff / 2.0
    p_mid_0  = (c_x + R_a * math.cos(ang_m), c_y + R_a * math.sin(ang_m))
    arcs.append((pts[0], p_mid_0, pts[2]))

    # Remaining arcs: 3-point trochoid sampling (start, interior, end).
    for i in range(1, num_arcs):
        arcs.append((pts[2 * i], pts[2 * i + 1], pts[2 * i + 2]))

    return arcs


# ---------------------------------------------------------------------------
# Valley arc sampling
# ---------------------------------------------------------------------------

def valley_arc_points(cx, cy, r_valley, start_angle_deg, end_angle_deg,
                      num_samples=24, ccw=True):
    """
    Compute points along a circular arc.

    Parameters
    ----------
    cx, cy      : arc centre in mm
    r_valley    : arc radius in mm
    start_angle_deg, end_angle_deg : angles in degrees (standard math convention)
    num_samples : number of returned points (including endpoints)
    ccw         : True = counter-clockwise sweep

    Returns list of (x, y) tuples.
    """
    a0 = math.radians(start_angle_deg)
    a1 = math.radians(end_angle_deg)
    if ccw:
        if a1 < a0:
            a1 += 2 * math.pi
    else:
        if a1 > a0:
            a1 -= 2 * math.pi
    pts = []
    for i in range(num_samples):
        t = a0 + (a1 - a0) * i / (num_samples - 1)
        pts.append((cx + r_valley * math.cos(t),
                    cy + r_valley * math.sin(t)))
    return pts


# ---------------------------------------------------------------------------
# Full rack tooth profile assembly
# ---------------------------------------------------------------------------

def rack_tooth_profile(module_mm, num_pins, r_pin_mm,
                       clearance_frac=0.05,
                       base_depth_mm=None,
                       flank_samples=72,
                       arc_samples=24):
    """
    Build a complete closed 2D profile for ONE RACK TOOTH + TWO HALF-VALLEYS.

    The profile spans X = [0 ... pitch] and Y = [-base_depth ... y_tip].

    Layout (traversed counter-clockwise for correct Fusion 360 winding):
        1. Valley arc (left)  : (0, -r_val) -> (r_val, 0)    CCW
        2. Right flank spline : (r_pin, 0)  -> (pitch/2, y_tip)
        3. Left flank spline  : (pitch/2, y_tip) -> (pitch-r_pin, 0)
        4. Valley arc (right) : (pitch-r_val, 0) -> (pitch, -r_val)  CW
        5. Right vertical     : (pitch, -r_val) -> (pitch, -base_depth)
        6. Bottom horizontal  : (pitch, -base_depth) -> (0, -base_depth)
        7. Left vertical      : (0, -base_depth) -> (0, -r_val)

    Returns a dict with keys:
        'pitch'         : float (mm)
        'y_tip'         : float (mm) -- tooth tip height above pitch line
        'r_valley'      : float (mm)
        'base_depth'    : float (mm)
        'left_valley_arc'  : [(x,y), ...]   Points 1
        'right_flank'      : [(x,y), ...]   Points 2
        'left_flank'       : [(x,y), ...]   Points 3
        'right_valley_arc' : [(x,y), ...]   Points 4
        'right_vertical'   : [(x,y), (x,y)]
        'bottom_line'      : [(x,y), (x,y)]
        'left_vertical'    : [(x,y), (x,y)]
    """
    p = derived_params(module_mm, num_pins, r_pin_mm)
    R_pc  = p['R_pc']
    r     = p['r_pin']
    pitch = p['pitch']

    r_valley = r * (1.0 + clearance_frac)

    # Default rack base depth: 2x pin diameter below pitch line
    if base_depth_mm is None:
        base_depth_mm = 4.0 * r

    # Right flank: valley at x=0 up to tooth tip
    rflank = right_flank_points(R_pc, r, pitch, flank_samples)

    # Left flank: mirror of right flank
    lflank = left_flank_points(R_pc, r, pitch, flank_samples)

    # Tooth tip y (from the last point of right flank = first of left flank)
    y_tip = rflank[-1][1]

    # Left half-valley arc: centre (0, 0), radius r_valley, from -90deg -> 0deg CCW
    # = from (0, -r_valley) to (r_valley, 0)
    lv_arc = valley_arc_points(0.0, 0.0, r_valley, -90.0, 0.0,
                                arc_samples, ccw=True)

    # Right half-valley arc: centre (pitch, 0), radius r_valley,
    # from 180deg -> 270deg CCW = from (pitch-r_valley, 0) -> (pitch, -r_valley)
    rv_arc = valley_arc_points(pitch, 0.0, r_valley, 180.0, 270.0,
                                arc_samples, ccw=True)

    # Vertical and base lines (two endpoints each)
    right_vert  = [(pitch, -r_valley),  (pitch, -base_depth_mm)]
    bottom_line = [(pitch, -base_depth_mm), (0.0, -base_depth_mm)]
    left_vert   = [(0.0, -base_depth_mm), (0.0, -r_valley)]

    return {
        'pitch':             pitch,
        'y_tip':             y_tip,
        'r_valley':          r_valley,
        'base_depth':        base_depth_mm,
        'left_valley_arc':   lv_arc,
        'right_flank':       rflank,
        'left_flank':        lflank,
        'right_valley_arc':  rv_arc,
        'right_vertical':    right_vert,
        'bottom_line':       bottom_line,
        'left_vertical':     left_vert,
    }


# ---------------------------------------------------------------------------
# Pinion wedge geometry
# ---------------------------------------------------------------------------

def pinion_wedge_spec(module_mm, num_pins, r_pin_mm,
                      hub_diameter_mm=None,
                      bore_clearance_mm=0.1):
    """
    Return geometric specification for ONE pinion wedge sector.

    The wedge is centred on the +Y axis (pin at Y = R_pc, X = 0).
    Angular half-width = pi/N radians.

    Parameters
    ----------
    module_mm         : gear module [mm]
    num_pins          : N
    r_pin_mm          : pin radius [mm]
    hub_diameter_mm   : if None, defaults to 0.8 * R_pc * 2
    bore_clearance_mm : extra radius beyond r_pin for the bore

    Returns a dict:
        'N'             : number of pins
        'R_pc'          : pin-centre circle radius [mm]
        'wedge_angle'   : full wedge angle [radians]  = 2pi/N
        'half_angle'    : pi/N [radians]
        'R_hub'         : inner (hub) radius [mm]
        'R_rim'         : outer rim radius [mm]  = R_pc + 2*r_pin
        'pin_cx'        : pin bore centre X [mm]  (always 0)
        'pin_cy'        : pin bore centre Y [mm]  (= R_pc)
        'r_bore'        : bore radius [mm]  = r_pin + bore_clearance
        'left_angle'    : angle of left wedge edge from +X axis [degrees]
        'right_angle'   : angle of right wedge edge from +X axis [degrees]
    """
    N = num_pins
    R_pc = num_pins * module_mm / 2.0
    r    = r_pin_mm

    half_angle = math.pi / N        # half wedge angle
    wedge_angle = 2.0 * math.pi / N  # full wedge angle

    R_hub = hub_diameter_mm / 2.0 if hub_diameter_mm is not None \
        else 0.4 * R_pc
    R_rim = R_pc + 2.0 * r          # outer rim just clears pin top

    r_bore = r + bore_clearance_mm

    # Angles from +X axis for the two radial edges of the wedge
    # Wedge centred on +Y (= 90deg).  Left edge at 90deg + half_angle_deg,
    # right edge at 90deg - half_angle_deg.
    half_deg = math.degrees(half_angle)
    left_angle  = 90.0 + half_deg
    right_angle = 90.0 - half_deg

    return {
        'N':           N,
        'R_pc':        R_pc,
        'wedge_angle': wedge_angle,
        'half_angle':  half_angle,
        'R_hub':       R_hub,
        'R_rim':       R_rim,
        'pin_cx':      0.0,
        'pin_cy':      R_pc,
        'r_bore':      r_bore,
        'left_angle':  left_angle,
        'right_angle': right_angle,
    }


# ---------------------------------------------------------------------------
# Tooth-count helper
# ---------------------------------------------------------------------------

def rack_tooth_count(rack_length_mm, module_mm):
    """
    Return the number of complete teeth that fit within rack_length_mm.
    Uses floor(), so the rack will be <= the requested length.
    """
    pitch = math.pi * module_mm
    return max(1, int(math.floor(rack_length_mm / pitch)))
