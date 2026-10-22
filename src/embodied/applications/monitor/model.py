"""URDF display model selected by the observed runtime, independent of a robot SDK."""
import hashlib
from pathlib import Path

import numpy as np

from embodied.data.episode.contract import runtime_contract


class RuntimeModel:
    def __init__(self, descriptor):
        import pinocchio as pin
        source = descriptor.get('parameters', {}).get('model', {})
        self.data_contract = runtime_contract(descriptor)
        path = source.get('urdf_path')
        if not path:
            raise ValueError('运行时未声明显示 URDF；等待完整模型描述')
        self.urdf = Path(path).resolve(strict=True)
        expected = source.get('urdf')
        if isinstance(expected, str) and '<robot' in expected and self.urdf.read_text() != expected:
            raise ValueError('运行时 URDF 内容已变化')
        self.model = pin.buildModelFromUrdf(str(self.urdf))
        self.names = list(self.model.names)[1:]
        if (self.model.nq != self.model.nv or self.model.nq != len(self.data_contract['joint_names'])
                or self.names != self.data_contract['joint_names']):
            raise ValueError('URDF 关节顺序/数量与运行时 data_contract 不一致')
        self.eef = source.get('manifest', {}).get('tcp_link')
        if not self.eef:
            raise ValueError('运行时模型未声明 TCP frame')
        self.frame = self.model.getFrameId(self.eef)
        if self.frame >= len(self.model.frames):
            raise ValueError('运行时 TCP frame 不存在于 URDF')
        self.lower, self.upper = self.model.lowerPositionLimit.copy(), self.model.upperPositionLimit.copy()

    def metadata(self):
        return {'urdf_sha256': hashlib.sha256(self.urdf.read_bytes()).hexdigest(),
                'eef_frame': self.eef, 'base_frame': self.model.frames[1].name,
                'joint_names': self.names, 'joint_unit': 'rad', 'length_unit': 'm'}

    def joints(self, q):
        values = np.asarray(q, dtype=float)
        if values.shape != (self.model.nq,) or not np.isfinite(values).all():
            raise ValueError('显示关节向量与模型维度不一致')
        return values
