"""
Cycloidal Rack and Pinion - Command Dialog
Uses class-based event handlers so exceptions surface via messageBox.
"""
import math
import traceback
import os
import adsk.core
import adsk.fusion
from ... import config
from ...lib import fusionAddInUtils as futil

# Surface import errors immediately so they show in Fusion's UI
try:
    from ...lib import cycloid_geometry as cg
    from ...lib import rack_generator
    from ...lib import pinion_generator
    from ...lib.fusion_geometry import new_component as _new_component
    _IMPORT_ERROR = None
except Exception:
    _IMPORT_ERROR = traceback.format_exc()
    cg = rack_generator = pinion_generator = _new_component = None

_app = adsk.core.Application.get()
_ui  = _app.userInterface

CMD_ID            = f'{config.COMPANY_NAME}_{config.ADDIN_NAME}_gearGen'
CMD_NAME          = 'Cycloidal Rack & Pinion Generator'
CMD_Description   = (
    'Generate a cycloidal rack and roller-pinion carrier.\n'
    'The rack tooth profile is a trochoid (offset cycloid); '
    'the pinion is an N-pin carrier with cylindrical bore holes.'
)
IS_PROMOTED       = True
WORKSPACE_ID      = 'FusionSolidEnvironment'
PANEL_ID          = 'SolidCreatePanel'
COMMAND_BESIDE_ID = 'SolidcycloidalRackAndPinionCommand'
ICON_FOLDER       = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', '')

# Keep references alive.
# _handlers  : permanent (holds _CreatedHandler for the lifetime of the add-in)
# _cmd_handlers : per-dialog (cleared after each command dialog is destroyed)
_handlers     = []
_cmd_handlers = []


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _units():
    """Return the active document's default length unit string (e.g. 'mm')."""
    return _app.activeProduct.unitsManager.defaultLengthUnits


def _vi(expr):
    """Create a ValueInput from a string expression (with units)."""
    return adsk.core.ValueInput.createByString(expr)


def _read_mm(inputs, input_id):
    """
    Read a ValueCommandInput and return its value in mm.
    Fusion stores values internally in cm; multiply by 10 to get mm.
    """
    inp = inputs.itemById(input_id)
    return inp.value * 10.0   # cm -> mm


def _build_info_text(inputs):
    """
    Compute derived info string from current dialog values.
    Returns a multi-line HTML string for the info TextBox.
    """
    try:
        if cg is None:
            return 'Import error -- see startup message.'
        m   = _read_mm(inputs, 'module')
        N   = inputs.itemById('num_pins').value
        d_p = _read_mm(inputs, 'pin_diameter')
        r_p = d_p / 2.0
        gen_rack   = inputs.itemById('generate_rack').value
        rack_len   = _read_mm(inputs, 'rack_length') if gen_rack else 0.0

        if m <= 0 or N < 1 or r_p <= 0:
            return 'Enter valid parameters above.'

        R_pc  = N * m / 2.0
        pitch = math.pi * m
        errors = cg.validate_params(m, N, r_p)

        lines = [
            f'Pitch circle R: {R_pc:.2f} mm',
            f'Circular pitch: {pitch:.3f} mm',
        ]
        if gen_rack:
            tc = cg.rack_tooth_count(rack_len, m)
            actual_len = tc * pitch
            lines.append(f'Tooth count: {tc}  (actual length ~ {actual_len:.1f} mm)')

        if errors:
            lines.append('ERRORS: ' + ' | '.join(errors))

        return '\n'.join(lines)
    except Exception:
        return ''


# ---------------------------------------------------------------------------
# Add-in lifecycle
# ---------------------------------------------------------------------------

def start():
    try:
        if _IMPORT_ERROR:
            _ui.messageBox(
                f'{CMD_NAME} - import error:\n\n{_IMPORT_ERROR}', CMD_NAME
            )
            return

        # Clean up any leftover definition/control from a previous run
        existing_ctrl = _ui.allToolbarPanels.itemById(PANEL_ID).controls.itemById(CMD_ID)
        if existing_ctrl:
            existing_ctrl.deleteMe()
        existing_def = _ui.commandDefinitions.itemById(CMD_ID)
        if existing_def:
            existing_def.deleteMe()
        _handlers.clear()

        cmd_def = _ui.commandDefinitions.addButtonDefinition(
            CMD_ID, CMD_NAME, CMD_Description, ICON_FOLDER
        )
        handler = _CreatedHandler()
        cmd_def.commandCreated.add(handler)
        _handlers.append(handler)

        panel   = _ui.allToolbarPanels.itemById(PANEL_ID)
        control = panel.controls.addCommand(cmd_def, COMMAND_BESIDE_ID, False)
        control.isPromotedByDefault = True
    except Exception:
        _ui.messageBox(
            f'{CMD_NAME} - startup error:\n\n{traceback.format_exc()}', CMD_NAME
        )


def stop():
    panel = _ui.allToolbarPanels.itemById(PANEL_ID)
    ctrl  = panel.controls.itemById(CMD_ID)
    cmd_def   = _ui.commandDefinitions.itemById(CMD_ID)
    if ctrl:
        ctrl.deleteMe()
    if cmd_def:
        cmd_def.deleteMe()
    _handlers.clear()


# ---------------------------------------------------------------------------
# Class-based event handlers
# ---------------------------------------------------------------------------

class _CreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            if _IMPORT_ERROR:
                _ui.messageBox(
                    f'{CMD_NAME} - import error:\n\n{_IMPORT_ERROR}', CMD_NAME
                )
                return

            cmd    = adsk.core.CommandCreatedEventArgs.cast(args).command
            inputs = cmd.commandInputs
            u      = _units()

            # ----- Gear parameters ------------------------------------------
            inputs.addTextBoxCommandInput(
                'section_gear', '', '<b>Gear Parameters</b>', 1, True
            )
            inputs.addValueInput('module',       'Module (m)',      u, _vi('4 mm'))
            inputs.addIntegerSpinnerCommandInput(
                'num_pins', 'Number of Pins (N)', 6, 60, 1, 12
            )
            inputs.addValueInput('pin_diameter', 'Pin Diameter',    u, _vi('4 mm'))

            # ----- Rack parameters ------------------------------------------
            inputs.addTextBoxCommandInput(
                'section_rack', '', '<b>Rack Parameters</b>', 1, True
            )
            inputs.addBoolValueInput('generate_rack', 'Generate Rack', True, '', True)
            inputs.addValueInput('rack_length', 'Rack Length', u, _vi('100 mm'))

            # ----- Pinion parameters ----------------------------------------
            inputs.addTextBoxCommandInput(
                'section_pinion', '', '<b>Pinion Parameters</b>', 1, True
            )
            inputs.addBoolValueInput(
                'generate_pinion', 'Generate Pinion', True, '', True
            )
            inputs.addValueInput('hub_diameter',   'Hub Diameter (0 = auto)', u, _vi('0 mm'))
            inputs.addValueInput('bore_clearance', 'Bore Clearance',          u, _vi('0.1 mm'))

            # ----- Common ---------------------------------------------------
            inputs.addTextBoxCommandInput(
                'section_common', '', '<b>Common</b>', 1, True
            )
            inputs.addValueInput('extrude_depth', 'Extrude Depth', u, _vi('10 mm'))

            # ----- Info display ---------------------------------------------
            inputs.addTextBoxCommandInput(
                'info', 'Derived Info', _build_info_text(inputs), 4, True
            )

            # ----- Attach further handlers ----------------------------------
            _cmd_handlers.clear()
            for ev, handler in [
                (cmd.execute,        _ExecuteHandler()),
                (cmd.inputChanged,   _InputChangedHandler()),
                (cmd.validateInputs, _ValidateHandler()),
                (cmd.destroy,        _DestroyHandler()),
            ]:
                ev.add(handler)
                _cmd_handlers.append(handler)

        except Exception:
            _ui.messageBox(
                f'{CMD_NAME} - dialog error:\n\n{traceback.format_exc()}', CMD_NAME
            )


class _InputChangedHandler(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            inputs  = adsk.core.InputChangedEventArgs.cast(args).inputs
            info_box = inputs.itemById('info')
            if info_box:
                info_box.text = _build_info_text(inputs)
        except Exception:
            pass   # info update is non-critical; swallow silently


class _ValidateHandler(adsk.core.ValidateInputsEventHandler):
    def notify(self, args):
        try:
            ev     = adsk.core.ValidateInputsEventArgs.cast(args)
            inputs = ev.inputs

            m          = _read_mm(inputs, 'module')
            N          = inputs.itemById('num_pins').value
            d_p        = _read_mm(inputs, 'pin_diameter')
            r_p        = d_p / 2.0
            depth      = _read_mm(inputs, 'extrude_depth')
            gen_rack   = inputs.itemById('generate_rack').value
            gen_pinion = inputs.itemById('generate_pinion').value

            if not gen_rack and not gen_pinion:
                ev.areInputsValid = False
                return
            if cg is None or cg.validate_params(m, N, r_p):
                ev.areInputsValid = False
                return
            if depth <= 0:
                ev.areInputsValid = False
                return
            if gen_rack and _read_mm(inputs, 'rack_length') <= 0:
                ev.areInputsValid = False
                return

            ev.areInputsValid = True
        except Exception:
            adsk.core.ValidateInputsEventArgs.cast(args).areInputsValid = False


class _ExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            inputs = adsk.core.CommandEventArgs.cast(args).command.commandInputs

            m        = _read_mm(inputs, 'module')
            N        = inputs.itemById('num_pins').value
            d_pin    = _read_mm(inputs, 'pin_diameter')
            depth    = _read_mm(inputs, 'extrude_depth')
            rack_len = _read_mm(inputs, 'rack_length')
            hub_dia  = _read_mm(inputs, 'hub_diameter')
            bore_clr = _read_mm(inputs, 'bore_clearance')
            gen_rack   = inputs.itemById('generate_rack').value
            gen_pinion = inputs.itemById('generate_pinion').value

            hub_dia_arg = hub_dia if hub_dia > 0.0 else None

            design    = adsk.fusion.Design.cast(_app.activeProduct)
            root_comp = design.rootComponent

            params = {
                'module_mm':         m,
                'num_pins':          N,
                'pin_diameter_mm':   d_pin,
                'extrude_depth_mm':  depth,
                'rack_length_mm':    rack_len,
                'hub_diameter_mm':   hub_dia_arg,
                'bore_clearance_mm': bore_clr,
                'clearance_frac':    0.05,
            }

            if gen_rack:
                _, rack_comp = _new_component(
                    root_comp, f'Rack_M{m:.1f}_N{N}'
                )
                rack_generator.generate_rack(rack_comp, params)

            if gen_pinion:
                _, pin_comp = _new_component(
                    root_comp, f'Pinion_M{m:.1f}_N{N}'
                )
                pinion_generator.generate_pinion(pin_comp, params)

        except Exception:
            _ui.messageBox(
                f'{CMD_NAME} - generation error:\n\n{traceback.format_exc()}',
                CMD_NAME
            )


class _DestroyHandler(adsk.core.CommandEventHandler):
    def notify(self, _args):
        _cmd_handlers.clear()   # release per-dialog handlers; _handlers keeps _CreatedHandler alive
