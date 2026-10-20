"""Exercise actual MuJoCo camera bindings without opening a graphics window."""
import unittest
from unittest.mock import patch

import glfw
import mujoco
from mujoco_viewer import callbacks


class CameraCallbacksTest(unittest.TestCase):
    def setUp(self):
        self.viewer = callbacks.Callbacks(hide_menus=True)
        self.viewer.model = mujoco.MjModel.from_xml_string(
            '<mujoco><worldbody><geom type="sphere" size="0.1"/></worldbody></mujoco>')
        self.viewer.data = mujoco.MjData(self.viewer.model)
        self.viewer.cam = mujoco.MjvCamera()
        self.viewer.cam.distance = 3
        self.viewer.scn = mujoco.MjvScene(self.viewer.model, maxgeom=10)
        self.viewer.pert = mujoco.MjvPerturb()
        self.viewer._scale = 1

    def test_mouse_drag_rotation_and_pan_with_current_bindings(self):
        for right in (False, True):
            for shift in (False, True):
                with self.subTest(right=right, shift=shift):
                    self.viewer._button_left_pressed = not right
                    self.viewer._button_right_pressed = right
                    with patch.object(callbacks.glfw, 'get_key', return_value=glfw.PRESS if shift else glfw.RELEASE), \
                            patch.object(callbacks.glfw, 'get_framebuffer_size', return_value=(1200, 900)):
                        self.viewer._cursor_pos_callback(None, 40, 20)
                    self.assertEqual(self.viewer._last_mouse_x, 40)
                    self.assertEqual(self.viewer._last_mouse_y, 20)
                    self.viewer._last_mouse_x = self.viewer._last_mouse_y = 0

    def test_scroll_changes_camera_distance(self):
        before = self.viewer.cam.distance
        self.viewer._scroll_callback(None, 0, 1)
        self.assertNotEqual(self.viewer.cam.distance, before)
        self.viewer._scroll_callback(None, 0, -1)
        self.assertGreater(self.viewer.cam.distance, 0)

    def test_legacy_signature_retains_scene(self):
        with patch.object(callbacks, 'MUJOCO_VERSION', (3, 10, 0)), \
                patch.object(callbacks.mujoco, 'mjv_moveCamera') as move:
            self.viewer._scroll_callback(None, 0, 1)
        move.assert_called_once_with(self.viewer.model, mujoco.mjtMouse.mjMOUSE_ZOOM,
                                     0, -0.05, self.viewer.scn, self.viewer.cam)

    def test_perturb_keeps_original_scene_argument(self):
        self.viewer._button_left_pressed = True
        self.viewer.pert.active = mujoco.mjtPertBit.mjPERT_ROTATE
        with patch.object(callbacks.glfw, 'get_key', return_value=glfw.RELEASE), \
                patch.object(callbacks.glfw, 'get_framebuffer_size', return_value=(1200, 900)), \
                patch.object(callbacks.mujoco, 'mjv_movePerturb') as move:
            self.viewer._cursor_pos_callback(None, 12, 9)
        self.assertEqual(len(move.call_args.args), 7)
        self.assertIs(move.call_args.args[-2], self.viewer.scn)
        self.assertIs(move.call_args.args[-1], self.viewer.pert)


if __name__ == '__main__':
    unittest.main()
