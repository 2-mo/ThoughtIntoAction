"""B601 components; offline model imports never initialize motorbridge."""
from importlib import import_module

__all__ = ["actuator", "kinematics", "dynamics"]

def __getattr__(name):
    if name not in __all__:
        raise AttributeError(name)
    value = import_module("." + name, __name__)
    globals()[name] = value
    return value
