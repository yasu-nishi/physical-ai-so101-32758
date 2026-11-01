"""MCP boundary. No LLM client, credentials, perception or object-state API."""
import base64
import json
from typing import Literal
from mcp.server.fastmcp import FastMCP
from mcp.types import ImageContent, TextContent, ToolAnnotations


def make_server(controller, port=8765):
    server = FastMCP('SO-101 Physical AI', host='127.0.0.1', port=port,
                     instructions='Observe the actual image and select the requested color. '
                     'For calibrated continuous picking, select the TOP FACE CENTER pixel, prepare_pick, '
                     'inspect the preview, and execute the generated Python program or prepared_pick. '
                     'For fixed slots use pick_place. Poll completion, observe again and report a visual verdict. '
                     'For iterative control, prepare_step generates only one small joint-relative step; '
                     'execute it, inspect the new RGB, then decide the next step. Never batch future visual decisions. '
                     'Command completion does not prove task success. Never infer a block from a preset layout. '
                     'Stop is latched. Real motion can only be armed locally by the operator.')

    @server.tool(structured_output=False, annotations=ToolAnnotations(readOnlyHint=True))
    def observe() -> list:
        """Fresh RGB and available depth PNGs, measured joints, gripper, tool pose and last result.

        Read the returned capabilities: fixed slots or continuous image picking.
        This operation does not move the robot.
        If a wrist-only camera needs a taught viewpoint, call observe_pose explicitly.
        """
        meta, images = controller.observe_bundle()
        return [TextContent(type='text', text=json.dumps(meta, ensure_ascii=False)),
                *[ImageContent(type='image', mimeType='image/png', data=base64.b64encode(png).decode())
                  for png in images]]

    @server.tool(annotations=ToolAnnotations(readOnlyHint=True))
    def sample_depth(observation_id: str, pixel_x: int, pixel_y: int) -> dict:
        """Return metric depth, work XYZ and surface height at an AI-selected RGB pixel.

        Use integer coordinates in the original image, not a resized display.
        Requires depth_sampling capability; no motion and no object identification.
        Samples the exact observation's aligned depth, not current object state.
        Height is relative to calibrated table Z=0, not the object's total size.
        Missing/invalid/stale depth raises an error, never a guessed measurement.
        prepare_pick still supports only the configured known-height upright block.
        """
        return controller.sample_depth(observation_id, pixel_x, pixel_y)

    @server.tool(annotations=ToolAnnotations(readOnlyHint=True))
    def locate_on_plane(observation_id: str, pixel_x: float, pixel_y: float, plane_z_m: float) -> dict:
        """Locate an AI-selected RGB pixel on an explicitly KNOWN height plane; no depth sensor.

        Works on calibrated sim/real RGB observations. plane_z_m must come from a
        measured task fixture/object dimension, not an inferred claim of height.
        Returns XYZ with height_measured=false. For an unknown height, this tool
        cannot determine a unique 3D point. No movement or skill generation.
        """
        return controller.locate_on_plane(observation_id, pixel_x, pixel_y, plane_z_m)

    @server.tool(structured_output=False, annotations=ToolAnnotations(readOnlyHint=True))
    def measure_height(observation_id: str, bottom_pixel: list[float], top_pixel: list[float],
                       center_pixel: list[float] | None, support_z_m: float, visual_reason: str) -> list:
        """Estimate UNKNOWN upright-object height from one calibrated RGB, without a depth sensor.

        Select bottom and top endpoints of the SAME VISIBLE vertical edge, plus the
        centre of the horizontal top face (null for height only when occluded).
        support_z_m is the known support plane
        (table=0), not object height. For a raised support measure it first. Returns
        estimated height, pixel-error interval, top centre XYZ and marked PNG.
        Hidden endpoints, tilted/curved edges, or wrong calibration invalidate this
        method. Do not guess an invisible edge. Works on calibrated real RGB too.
        """
        meta,png=controller.measure_height(observation_id,bottom_pixel,top_pixel,center_pixel,support_z_m,visual_reason)
        return [TextContent(type='text',text=json.dumps(meta,ensure_ascii=False)),
                ImageContent(type='image',mimeType='image/png',data=base64.b64encode(png).decode())]

    @server.tool(structured_output=False, annotations=ToolAnnotations(readOnlyHint=False,destructiveHint=False))
    def prepare_cartesian_step(observation_id: str, target_xyz_m: list[float] | None,
                               gripper_open: float | None, instruction: str, visual_reason: str,
                               expected_visual_change: str,
                               tool_orientation: Literal['down','preserve_tilt','position_only'] = 'down',
                               locked_joints: list[Literal['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex']] | None = None) -> list:
        """Generate ONE bounded next Cartesian OR gripper move, never a whole task.

        Exactly one of target_xyz_m and gripper_open must be non-null. Use measured
        RGB geometry and current robot-only FK from observe. A new RGB must be
        inspected after each Python run before deciding the next stage. Separate
        approach, descend, close, test-lift, transfer, lower and release decisions.
        Does not prove contact or task success. Real kinematics must be independently
        verified locally; no simulated object state is used. Limits are in observe.
        tool_orientation=down keeps the original downward-tool behavior.
        preserve_tilt keeps the current inclination and wrist-roll joint, allowing
        an angled real arm to translate without first rotating the tool downward.
        Pan can turn during XY translation; it is not fixed full orientation.
        position_only holds wrist-roll but permits at most 5 degrees tool-axis
        change (or a stricter local limit). Only in that mode, locked_joints may
        hold ONE additional joint at its measured nominal angle. No motor is
        disabled; all actual joint and Cartesian tracking checks remain active.
        Inspect clearance and orientation before choosing this option.
        """
        meta,png=controller.prepare_cartesian_step(observation_id,target_xyz_m,gripper_open,instruction,visual_reason,expected_visual_change,tool_orientation,locked_joints)
        return [TextContent(type='text',text=json.dumps(meta,ensure_ascii=False)),
                ImageContent(type='image',mimeType='image/png',data=base64.b64encode(png).decode())]

    @server.tool(annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=True, idempotentHint=True))
    def execute_skill(name: str, args: dict, observation_id: str, instruction: str, request_id: str) -> dict:
        """Start a bounded local skill and return immediately with action_id.

        name=pick_place, args={"slot":"slot_a" or "slot_b","destination":"box"};
        or name=observe_pose, args={}; or name=prepared_pick / prepared_step /
        prepared_cartesian_step, args={"program_id":"..."} returned by the corresponding
        prepare tool. Select using the recent observe image and
        user instruction, not hidden simulation state. Unique request_id prevents
        duplicate execution on transport retries. Poll get_status; then observe.
        """
        return controller.execute_skill(name, args, observation_id, instruction, request_id)

    @server.tool(structured_output=False, annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=False))
    def prepare_pick(observation_id: str, pixel_x: float, pixel_y: float,
                     instruction: str, visual_reason: str) -> list:
        """Localize the AI-selected top-face center, generate/validate a motion program, without motion.

        Use original observe PNG pixel coordinates (origin top-left, u right, v down),
        NOT the screen's resized dimensions or a preset object location. Known upright
        30 mm cube on table only. Returns estimated work coordinates, joint trajectory,
        executable Python/JSON paths and an image marking your selected pixel.
        Requires continuous_image_pick capability. No color detection or hidden object
        poses here: the connected AI must select the pixel from the image.
        """
        meta, png = controller.prepare_pick(observation_id, pixel_x, pixel_y, instruction, visual_reason)
        return [TextContent(type='text', text=json.dumps(meta, ensure_ascii=False)),
                ImageContent(type='image', mimeType='image/png', data=base64.b64encode(png).decode())]

    @server.tool(annotations=ToolAnnotations(readOnlyHint=True))
    def get_status() -> dict:
        """Non-blocking command lifecycle, local mode, last action and latched stop state."""
        return controller.get_status()

    @server.tool(structured_output=False, annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=False))
    def prepare_step(observation_id: str, joint_delta_deg: dict[str, float], gripper_delta: float,
                     instruction: str, visual_reason: str, expected_visual_change: str) -> list:
        """Generate ONE small joint-relative Python motion program without moving.

        Read incremental_step_limits in observe. joint_delta_deg uses the named arm
        joints in degrees; gripper_delta uses opening fraction, not an angle. Move
        either the arm or the gripper, then reobserve. State the visible evidence
        and the change you expect to see. Run the returned Python or prepared_step,
        inspect the new image, and only then generate another step. No image/depth
        localization or IK is assumed. Joint deltas do not guarantee Cartesian
        clearance, height or contact. If the scene is ambiguous, stop, not blind descent.
        Real default is dry_run; actual incremental motion needs local verification + ARM.
        """
        meta, png = controller.prepare_step(observation_id, joint_delta_deg, gripper_delta,
                                             instruction, visual_reason, expected_visual_change)
        return [TextContent(type='text', text=json.dumps(meta, ensure_ascii=False)),
                ImageContent(type='image', mimeType='image/png', data=base64.b64encode(png).decode())]

    @server.tool(annotations=ToolAnnotations(readOnlyHint=False, idempotentHint=True))
    def stop() -> dict:
        """Latch stop without waiting for the action. Local restart is required to resume.

        Real backend holds measured position. This is software stop, not a hardware emergency stop.
        The operator can also create the local STOP file or press Ctrl-C.
        """
        return controller.stop()

    @server.tool(annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=False))
    def report_result(action_id: str, observation_id: str, verdict: str, reason: str) -> dict:
        """Record your post-action image judgment: success, failure, or uncertain.

        Describe the visible evidence. This AI claim is separate from command completion
        and the independent offline simulation evaluator.
        """
        return controller.report_result(action_id, observation_id, verdict, reason)

    return server
