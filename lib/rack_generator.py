# -*- coding: utf-8 -*-
"""Rack Generator - cycloidal rack using arcs + spline sketch entities."""
import math
import adsk.core
import adsk.fusion

from . import cycloid_geometry as cg
from . import fusion_geometry as fg


def generate_rack(component, params):
    """Build the rack in component. params dict keys:
    module_mm, num_pins, pin_diameter_mm, rack_length_mm,
    extrude_depth_mm, clearance_frac (default 0.05).
    Returns the seed BRepBody."""
    m        = params['module_mm']
    N        = params['num_pins']
    r_pin    = params['pin_diameter_mm'] / 2.0
    rack_len = params['rack_length_mm']
    depth    = params['extrude_depth_mm']
    c_frac   = params.get('clearance_frac', 0.05)

    pitch       = math.pi * m
    tooth_count = cg.rack_tooth_count(rack_len, m)
    R_pc        = N * m / 2.0

    # r_valley: X coordinate where the trochoid flank meets Y=0 (at phi_0).
    # This is derived from the trochoid equation, not a fudge factor, so the
    # arc endpoint and spline start are exactly coincident.
    phi_0    = 2.0 * math.asin(min(r_pin / (2.0 * R_pc), 1.0))
    r_valley = (R_pc * (phi_0 - math.sin(phi_0))
                + r_pin * math.cos(phi_0 / 2.0))

    base_depth = r_pin * 2.0

    # Arc chains for both flanks.
    # right_flank_arcs: (r_valley,0) -> tip (pitch/2, y_tip), 8 arcs, G1 at valley.
    arcs_right = cg.right_flank_arcs(R_pc, r_pin, pitch, num_arcs=8)
    # Left flank: mirror about x=pitch/2, traversed tip -> (pitch-r_valley, 0).
    arcs_left = [
        ((pitch - x_e, y_e), (pitch - x_m, y_m), (pitch - x_s, y_s))
        for (x_s, y_s), (x_m, y_m), (x_e, y_e) in reversed(arcs_right)
    ]

    # Valley arc midpoints (concave arcs at pitch-line level).
    S        = math.sqrt(2.0) / 2.0
    lv_start = (0.0,               -r_valley)
    lv_mid   = (r_valley * S,      -r_valley * S)
    lv_end   = (r_valley,           0.0)
    rv_start = (pitch - r_valley,   0.0)
    rv_mid   = (pitch - r_valley * S, -r_valley * S)
    rv_end   = (pitch,             -r_valley)

    # Draw sketch: left vertical -> left valley arc -> right flank arcs ->
    #              left flank arcs -> right valley arc -> right vertical -> bottom
    sk          = fg.create_xy_sketch(component, 'RackTooth')
    fg.draw_sketch_line(sk, (0.0, -base_depth), (0.0, -r_valley))
    fg.draw_sketch_arc(sk, lv_start, lv_mid, lv_end)
    for (p_s, p_m, p_e) in arcs_right:
        fg.draw_sketch_arc(sk, p_s, p_m, p_e)
    for (p_s, p_m, p_e) in arcs_left:
        fg.draw_sketch_arc(sk, p_s, p_m, p_e)
    fg.draw_sketch_arc(sk, rv_start, rv_mid, rv_end)
    fg.draw_sketch_line(sk, rv_end, (pitch, -base_depth))
    fg.draw_sketch_line(sk, (pitch, -base_depth), (0.0, -base_depth))

    # Select tooth profile and extrude
    # Note: Fusion 360 auto-merges sketch curves drawn at the same coordinates,
    # so no explicit coincident constraints are needed.
    cx = pitch / 2.0
    cy = -base_depth / 2.0
    profile = fg.profile_containing_point(sk, cx, cy)
    if profile is None:
        profile = fg.largest_profile(sk)
    if profile is None:
        raise RuntimeError(
            'Rack sketch produced no closed profile. '
            'Sketch %s has %d profiles. '
            'Check r_pin (%.3f mm) < pitch/4 (%.3f mm).' % (
                sk.name, sk.profiles.count, r_pin, pitch / 4.0)
        )

    extrude = fg.extrude_profile(component, profile, depth)

    # Rectangular pattern along +X
    if tooth_count > 1:
        fg.rectangular_pattern_body(component, extrude, tooth_count, pitch)

    return extrude.bodies.item(0)
