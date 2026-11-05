"""Dual-arm recording contract: 12 joints plus one gripper width per arm (14 dims)."""
import unittest

from embodied.data.episode.contract import (
    feature_names, gripper_contracts, runtime_contract, validate_contract, vector,
)

DUAL = {
    'joint_count': 12,
    'arm_count': 2,
    'gripper_max_widths': [0.082, 0.082],
    'joint_names': [f'left_joint{i}' for i in range(1, 7)] + [f'right_joint{i}' for i in range(1, 7)],
    'parameters': {'effective_robot_config': {
        'left': {'park_q': [0.] * 6, 'gripper_max_width': 0.082},
        'right': {'park_q': [0.] * 6, 'gripper_max_width': 0.082},
    }},
}


class DualContractTests(unittest.TestCase):
    def test_dual_descriptor_declares_one_gripper_per_arm(self):
        contract = runtime_contract(DUAL)
        self.assertEqual([g['name'] for g in gripper_contracts(contract)], ['gripper_left', 'gripper_right'])
        self.assertIsNone(contract['gripper'])
        self.assertEqual(len(feature_names(contract)), 14)

    def test_vector_carries_both_gripper_widths(self):
        contract = runtime_contract(DUAL)
        row = {'q': [0.1] * 12, 'gripper_widths': [0.08, 0.01]}
        values = vector(row, contract)
        self.assertEqual(len(values), 14)
        self.assertEqual(values[-2:], [0.08, 0.01])

    def test_vector_requires_one_width_per_gripper(self):
        contract = runtime_contract(DUAL)
        with self.assertRaises(ValueError):
            vector({'q': [0.1] * 12, 'gripper_widths': [0.08]}, contract)
        with self.assertRaises(ValueError):
            vector({'q': [0.1] * 12, 'gripper_width': 0.08}, contract)

    def test_out_of_range_width_is_rejected(self):
        contract = runtime_contract(DUAL)
        with self.assertRaises(ValueError):
            vector({'q': [0.1] * 12, 'gripper_widths': [0.08, -0.001]}, contract)
        with self.assertRaises(ValueError):
            vector({'q': [0.1] * 12, 'gripper_widths': [0.08, 0.09]}, contract)

    def test_single_arm_contract_is_unchanged(self):
        contract = runtime_contract({
            'joint_count': 6,
            'parameters': {'effective_robot_config': {'park_q': [0.] * 6, 'gripper_max_width': 0.082}},
        })
        self.assertEqual(len(feature_names(contract)), 7)
        self.assertEqual(vector({'q': [0.1] * 6, 'gripper_width': 0.03}, contract)[-1], 0.03)

    def test_contract_rejects_mixing_single_and_per_arm_grippers(self):
        with self.assertRaises(ValueError):
            validate_contract({
                'schema': 'embodied.joint_vectors/1', 'representation': 'absolute_joint_position',
                'joint_names': ['a', 'b'], 'joint_unit': 'rad',
                'gripper': {'name': 'gripper_m', 'unit': 'm', 'range': [0., .08]},
                'grippers': [{'name': 'gripper_left', 'unit': 'm', 'range': [0., .08]}],
            })

    def test_gripper_names_must_be_unique(self):
        with self.assertRaises(ValueError):
            validate_contract({
                'schema': 'embodied.joint_vectors/1', 'representation': 'absolute_joint_position',
                'joint_names': ['a', 'b'], 'joint_unit': 'rad', 'gripper': None,
                'grippers': [{'name': 'gripper_x', 'unit': 'm', 'range': [0., .08]},
                             {'name': 'gripper_x', 'unit': 'm', 'range': [0., .08]}],
            })


if __name__ == '__main__':
    unittest.main()
