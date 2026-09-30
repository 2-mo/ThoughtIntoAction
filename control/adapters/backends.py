"""Lazy factories around the original arx implementations."""
def simulation(config, **options):
    from arx5_vr.sim import MujocoBackend
    return MujocoBackend(config, **options)


def hardware(config, **options):
    from arx5_vr.real import RealBackend
    return RealBackend(config, **options)
