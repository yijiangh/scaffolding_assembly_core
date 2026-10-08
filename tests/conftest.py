"""Skip the test modules whose extras are not installed: compas and compas_fab (`mirrors`), rs_data_structure."""

from importlib.util import find_spec

#: Test module -> the packages it imports beyond the core's own dependencies.
_NEEDS = {
    "test_geometry.py": ("compas", "compas_fab"),
    "test_compas_planning_group.py": ("compas", "compas_fab", "compas_robots"),
    "test_legacy_guards.py": ("compas", "compas_robots", "compas_fab", "rs_data_structure"),
    "test_legacy_conversion.py": ("compas", "compas_fab", "rs_data_structure"),
}

collect_ignore = [name for name, packages in _NEEDS.items() if any(find_spec(p) is None for p in packages)]
