# -*- coding: utf-8 -*-
"""Pinion Generator - carrier disc with solid cylindrical pins."""
import math
import adsk.core
import adsk.fusion

from . import cycloid_geometry as cg
from . import fusion_geometry as fg


def generate_pinion(component, params):
    """Build the pinion carrier in component. params dict keys:
    module_mm, num_pins, pin_diameter_mm, extrude_depth_mm,
    hub_diameter_mm (None = auto), bore_clearance_mm (unused, kept for compat).
    Returns the carrier disc BRepBody."""
    m       = params['module_mm']
    N       = params['num_pins']
    r_pin   = params['pin_diameter_mm'] / 2.0
    depth   = params['extrude_depth_mm']
    hub_dia = params.get('hub_diameter_mm', None)

    spec  = cg.pinion_wedge_spec(m, N, r_pin, hub_diameter_mm=hub_dia)
    R_pc  = spec['R_pc']
    R_hub = spec['R_hub']
    R_rim = spec['R_rim']

    # Step 1: carrier disc -- annular ring (outer circle + hub bore)
    sk_disc = fg.create_xy_sketch(component, 'PinionDisc')
    fg.draw_sketch_circle(sk_disc, 0.0, 0.0, R_rim)
    fg.draw_sketch_circle(sk_disc, 0.0, 0.0, R_hub)

    r_mid    = (R_hub + R_rim) / 2.0
    ann_prof = fg.profile_containing_point(sk_disc, 0.0, r_mid)
    if ann_prof is None:
        ann_prof = fg.largest_profile(sk_disc)
    if ann_prof is None:
        raise RuntimeError(
            'Pinion disc sketch produced no annular profile. '
            'R_hub=%.2f mm, R_rim=%.2f mm' % (R_hub, R_rim)
        )

    disc_extrude = fg.extrude_profile(component, ann_prof, depth)

    # Step 2: draw ALL N pin circles in ONE sketch then extrude together.
    # This avoids circular-pattern API limitations with Join operations.
    sk_pins = fg.create_xy_sketch(component, 'PinionPins')
    for i in range(N):
        angle = 2.0 * math.pi * i / N
        cx    = R_pc * math.cos(angle)
        cy    = R_pc * math.sin(angle)
        fg.draw_sketch_circle(sk_pins, cx, cy, r_pin)

    # Collect all N pin profiles into an ObjectCollection
    pin_profiles = adsk.core.ObjectCollection.create()
    for j in range(sk_pins.profiles.count):
        pin_profiles.add(sk_pins.profiles.item(j))

    if pin_profiles.count == 0:
        raise RuntimeError(
            'PinionPins sketch produced no profiles. N=%d R_pc=%.2f r_pin=%.2f' % (N, R_pc, r_pin)
        )

    # Extrude all pin profiles in one operation, joining to the disc.
    # Sketch is at Z=0 (disc base). Disc top face is at Z=+depth.
    # Offset the start to Z=+depth so Fusion extrudes downward through the disc.
    feats    = component.features.extrudeFeatures
    ext_inp  = feats.createInput(
        pin_profiles,
        adsk.fusion.FeatureOperations.JoinFeatureOperation
    )
    depth_cm = depth / 10.0   # mm -> cm
    ext_inp.startExtent = adsk.fusion.OffsetStartDefinition.create(
        adsk.core.ValueInput.createByReal(depth_cm)
    )
    ext_inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(depth_cm))
    feats.add(ext_inp)

    return disc_extrude.bodies.item(0)
