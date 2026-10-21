from embodied.phone.core.mapping import AxisMapping, MappingScales, PoseMapping
from embodied.phone.runtime.mapping_settings import MappingSettingsStore


def test_mapping_settings_update_scales_without_changing_axes() -> None:
    axes = AxisMapping(robot_from_input=(2, 0, 1), signs=(-1, -1, 1))
    store = MappingSettingsStore(
        PoseMapping(axes=axes, position_scale=0.7, rotation_scale=0.8)
    )

    applied = store.update(MappingScales(1.25, 0.5))

    assert applied == MappingScales(1.25, 0.5)
    assert store.load() == PoseMapping(
        axes=axes,
        position_scale=1.25,
        rotation_scale=0.5,
    )
