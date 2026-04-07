# -*- coding: utf-8 -*-
"""
Fusion 360 Geometry Helpers
============================

Thin wrappers around the Fusion 360 Python API for creating sketch geometry,
extrude features, and pattern features.

Unit convention
---------------
* All helper functions accept measurements in **millimetres**.
* Internally they convert to centimetres before calling the API (Fusion 360
  stores everything in cm).

Sketch strategy -- POLYLINES ONLY
---------------------------------
Fitted splines created programmatically do NOT automatically form coincident
constraints with adjacent curves, so Fusion 360 cannot detect a closed profile
from mixed splines+lines+arcs.  Instead, we approximate all curves (flanks,
arcs) as sequences of short line segments.  With 60-80 segments per flank the
result is visually identical to a smooth spline at normal zoom levels, and
profile detection is 100% reliable.

Sketch plane
------------
All rack and pinion sketches are drawn on the XY plane of the supplied
component, with:
    X+ -> along the rack
    Y+ -> tooth tips / outward radial
    Z+ -> extrude depth
"""

import math
import adsk.core
import adsk.fusion


def _pt(x_mm, y_mm, z_mm=0.0):
    """Create a Point3D from mm coordinates."""
    return adsk.core.Point3D.create(x_mm / 10.0, y_mm / 10.0, z_mm / 10.0)


def _val(value_cm):
    """Create a ValueInput from a real value (already in cm)."""
    return adsk.core.ValueInput.createByReal(value_cm)


def _val_mm(value_mm):
    """Create a ValueInput from a mm value (converts to cm)."""
    return adsk.core.ValueInput.createByReal(value_mm / 10.0)


# ---------------------------------------------------------------------------
# Sketch creation
# ---------------------------------------------------------------------------

def create_xy_sketch(component, name='Profile'):
    """Return a new Sketch on the component's XY construction plane."""
    sketches = component.sketches
    xy_plane = component.xYConstructionPlane
    sk = sketches.add(xy_plane)
    sk.name = name
    return sk


# ---------------------------------------------------------------------------
# Polyline drawing (the ONLY sketch curve method we use)
# ---------------------------------------------------------------------------

def draw_closed_polyline(sketch, points_mm):
    """
    Draw a closed polygon through the given 2-D points (mm).

    Adds one SketchLine for each consecutive pair of points, plus one closing
    segment from the last point back to the first.  All lines share endpoint
    coordinates, so Fusion 360 auto-merges them and detects a closed profile.

    Parameters
    ----------
    sketch     : adsk.fusion.Sketch
    points_mm  : list of (x, y) tuples in mm -- must have >= 3 entries

    Returns list of SketchLine objects created.
    """
    lines_api = sketch.sketchCurves.sketchLines
    n = len(points_mm)
    created = []
    for i in range(n):
        p1 = points_mm[i]
        p2 = points_mm[(i + 1) % n]
        # Skip zero-length segments (can happen at duplicate points).
        if math.hypot(p2[0] - p1[0], p2[1] - p1[1]) < 1e-6:
            continue
        line = lines_api.addByTwoPoints(_pt(p1[0], p1[1]), _pt(p2[0], p2[1]))
        created.append(line)
    return created


def draw_sketch_line(sketch, p1_mm, p2_mm):
    """Draw a single SketchLine from p1 to p2 (mm). Returns the SketchLine."""
    return sketch.sketchCurves.sketchLines.addByTwoPoints(
        _pt(p1_mm[0], p1_mm[1]), _pt(p2_mm[0], p2_mm[1])
    )


def draw_sketch_arc(sketch, p_start_mm, p_mid_mm, p_end_mm):
    """
    Draw a SketchArc through three points (all in mm).
    The arc passes through start->mid->end in that order.
    Returns the SketchArc; .startSketchPoint = p_start, .endSketchPoint = p_end.
    """
    return sketch.sketchCurves.sketchArcs.addByThreePoints(
        _pt(p_start_mm[0], p_start_mm[1]),
        _pt(p_mid_mm[0],   p_mid_mm[1]),
        _pt(p_end_mm[0],   p_end_mm[1]),
    )


def draw_sketch_spline(sketch, points_mm):
    """
    Draw an open fitted spline through (x, y) mm points.
    Returns SketchFittedSpline; endpoints via .startSketchPoint / .endSketchPoint.
    """
    coll = adsk.core.ObjectCollection.create()
    for x, y in points_mm:
        coll.add(_pt(x, y))
    return sketch.sketchCurves.sketchFittedSplines.add(coll)


def draw_sketch_circle(sketch, cx_mm, cy_mm, r_mm):
    """Draw a full circle. Returns SketchCircle."""
    return sketch.sketchCurves.sketchCircles.addByCenterRadius(
        _pt(cx_mm, cy_mm), r_mm / 10.0
    )


def add_coincident(sketch, skpoint1, skpoint2):
    """Add a coincident constraint between two SketchPoints."""
    sketch.geometricConstraints.addCoincident(skpoint1, skpoint2)


def arc_polyline_points(cx_mm, cy_mm, r_mm, start_angle_rad, end_angle_rad,
                        num_samples=24, ccw=True):
    """
    Return a list of (x, y) mm points along a circular arc -- no Fusion calls.

    Parameters
    ----------
    cx_mm, cy_mm     : arc centre
    r_mm             : arc radius
    start_angle_rad  : start angle (standard math convention, measured from +X)
    end_angle_rad    : end angle
    num_samples      : number of points including both endpoints
    ccw              : True  -> sweep counter-clockwise (increasing angle)
                       False -> sweep clockwise (decreasing angle)
    """
    a0, a1 = start_angle_rad, end_angle_rad
    if ccw:
        while a1 < a0:
            a1 += 2 * math.pi
    else:
        while a1 > a0:
            a1 -= 2 * math.pi
    pts = []
    for i in range(num_samples):
        t = a0 + (a1 - a0) * i / (num_samples - 1)
        pts.append((cx_mm + r_mm * math.cos(t),
                    cy_mm + r_mm * math.sin(t)))
    return pts


def circle_polyline_points(cx_mm, cy_mm, r_mm, num_samples=48):
    """Return (x, y) mm points for a full circle polygon (for bore holes)."""
    pts = []
    for i in range(num_samples):
        a = 2 * math.pi * i / num_samples
        pts.append((cx_mm + r_mm * math.cos(a),
                    cy_mm + r_mm * math.sin(a)))
    return pts


# ---------------------------------------------------------------------------
# Feature helpers
# ---------------------------------------------------------------------------

def extrude_profile(component, profile, depth_mm,
                    operation=adsk.fusion.FeatureOperations.NewBodyFeatureOperation):
    """
    Extrude a sketch profile in +Z by depth_mm.

    Returns the new ExtrudeFeature.
    """
    feats = component.features.extrudeFeatures
    inp = feats.createInput(profile, operation)
    dist = _val_mm(depth_mm)
    inp.setDistanceExtent(False, dist)
    return feats.add(inp)


def rectangular_pattern_body(component, feature, quantity, spacing_mm):
    """
    Create a rectangular pattern of *feature* along the +X axis.

    Parameters
    ----------
    component  : adsk.fusion.Component
    feature    : adsk.fusion.ExtrudeFeature  -- the seed feature to pattern
    quantity   : int  -- total number of instances (including the original)
    spacing_mm : float  -- distance between instances (pitch) in mm

    Returns the RectangularPatternFeature.
    """
    feats = component.features.rectangularPatternFeatures
    entities = adsk.core.ObjectCollection.create()
    entities.add(feature)

    x_axis = component.xConstructionAxis

    inp = feats.createInput(
        entities,
        x_axis,
        adsk.core.ValueInput.createByString(str(int(quantity))),
        _val_mm(spacing_mm),
        adsk.fusion.PatternDistanceType.SpacingPatternDistanceType
    )
    inp.patternComputeOption = adsk.fusion.PatternComputeOptions.IdenticalPatternCompute
    return feats.add(inp)


def circular_pattern_body(component, feature, quantity):
    """
    Create a full (360deg) circular pattern of *feature* around the Z axis.

    Returns the CircularPatternFeature.
    """
    feats = component.features.circularPatternFeatures
    entities = adsk.core.ObjectCollection.create()
    entities.add(feature)

    z_axis = component.zConstructionAxis

    inp = feats.createInput(entities, z_axis)
    inp.quantity = adsk.core.ValueInput.createByString(str(int(quantity)))
    inp.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    inp.isSymmetric = False
    inp.patternComputeOption = adsk.fusion.PatternComputeOptions.IdenticalPatternCompute
    return feats.add(inp)


# ---------------------------------------------------------------------------
# Component / occurrence helpers
# ---------------------------------------------------------------------------

def new_component(root_component, name):
    """
    Create a new empty sub-component under root_component.

    Returns (occurrence, component).
    """
    matrix = adsk.core.Matrix3D.create()
    occ = root_component.occurrences.addNewComponent(matrix)
    comp = occ.component
    comp.name = name
    return occ, comp


# ---------------------------------------------------------------------------
# Profile selection helpers  (find a specific loop after sketching)
# ---------------------------------------------------------------------------

def largest_profile(sketch):
    """
    Return the sketch Profile with the largest actual area (via areaProperties).
    """
    best = None
    best_area = -1.0
    for i in range(sketch.profiles.count):
        prof = sketch.profiles.item(i)
        area = prof.areaProperties().area
        if area > best_area:
            best_area = area
            best = prof
    return best


def profile_containing_point(sketch, x_mm, y_mm):
    """
    Return the sketch Profile whose bounding box contains the point (x_mm, y_mm).
    If multiple profiles contain the point, returns the smallest one (innermost).
    Coordinates in mm (converted to cm internally).
    """
    tx, ty = x_mm / 10.0, y_mm / 10.0
    best = None
    best_area = float('inf')
    for i in range(sketch.profiles.count):
        prof = sketch.profiles.item(i)
        bb = prof.boundingBox
        if (bb.minPoint.x <= tx <= bb.maxPoint.x and
                bb.minPoint.y <= ty <= bb.maxPoint.y):
            w = bb.maxPoint.x - bb.minPoint.x
            h = bb.maxPoint.y - bb.minPoint.y
            area = w * h
            if area < best_area:
                best_area = area
                best = prof
    return best
