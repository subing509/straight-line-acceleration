"""Check GUI refresh budgeting without importing ROS, OpenCV or NumPy."""
import ast
import os
import unittest


def load_visualization_wait_ms():
    source_path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                               'src', 'acceleration_event', 'src', 'jiantu.py')
    with open(source_path, 'rb') as source:
        tree = ast.parse(source.read(), filename=source_path)
    helpers = [node for node in tree.body
               if isinstance(node, ast.FunctionDef)
               and node.name == 'visualization_wait_ms']
    if len(helpers) != 1:
        raise AssertionError('Expected one visualization_wait_ms helper')
    isolated = ast.parse('')
    isolated.body = helpers
    namespace = {}
    eval(compile(isolated, source_path, 'exec'), namespace)
    return namespace['visualization_wait_ms']


class VisualizationBudgetTests(unittest.TestCase):
    def setUp(self):
        self.wait_ms = load_visualization_wait_ms()

    def test_default_five_hz_reserves_remaining_frame_time(self):
        self.assertEqual(200, self.wait_ms(0.0, 0.0, 5.0))
        self.assertEqual(150, self.wait_ms(0.0, 0.05, 5.0))

    def test_refresh_frequency_changes_only_the_wait_budget(self):
        self.assertEqual(100, self.wait_ms(0.0, 0.0, 10.0))
        self.assertEqual(50, self.wait_ms(0.0, 0.05, 10.0))

    def test_full_budget_or_render_overrun_never_blocks_for_a_key(self):
        self.assertEqual(1, self.wait_ms(0.0, 0.2, 5.0))
        self.assertEqual(1, self.wait_ms(0.0, 0.25, 5.0))

    def test_backward_clock_uses_normal_frame_budget(self):
        self.assertEqual(200, self.wait_ms(100.0, 99.0, 5.0))

    def test_submillisecond_budget_is_at_least_one_millisecond(self):
        self.assertEqual(1, self.wait_ms(0.0, 0.1999, 5.0))
        self.assertEqual(1, self.wait_ms(0.0, 0.0, 2000.0))


if __name__ == '__main__':
    unittest.main()
