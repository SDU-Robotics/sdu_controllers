#!/usr/bin/env python3
"""Generate type stubs for sdu_controllers and its submodules using nanobind.stubgen"""

import sys
import os
import pathlib

# Add current directory to path to find _sdu_controllers extension module
# This is important for cibuildwheel builds where the .pyd/.so might not be in standard paths
sys.path.insert(0, os.getcwd())
# Also add the directory of this script to the path, just in case
sys.path.insert(0, pathlib.Path(__file__).parent.resolve())

from nanobind.stubgen import StubGen
try:
    import _sdu_controllers
except ImportError as e:
    print(f"ERROR: Could not import _sdu_controllers: {e}")
    print(f"Current directory: {os.getcwd()}")
    print(f"sys.path: {sys.path}")
    print(f"Files in current directory: {os.listdir('.')}")
    raise

print("Generating stubs for sdu_controllers...")

# Generate stubs for main module
sg_main = StubGen(_sdu_controllers)
sg_main.put(_sdu_controllers)
main_stubs = sg_main.get().replace("_sdu_controllers", "sdu_controllers")

# Try to generate stubs for submodules
submodules = {}
for attr_name in dir(_sdu_controllers):
    if not attr_name.startswith('_'):
        try:
            attr = getattr(_sdu_controllers, attr_name)
            # Check if it's a module-like object
            if hasattr(attr, '__dict__') and not callable(attr):
                print(f"  - Found submodule: {attr_name}")
                try:
                    sg = StubGen(attr)
                    sg.put(attr)
                    submodules[attr_name] = sg.get().replace("_sdu_controllers", "sdu_controllers")
                except Exception as e:
                    print(f"    Warning: Could not generate stubs for {attr_name}: {e}")
        except Exception as e:
            pass

# Save to files
with open("sdu_controllers.pyi", "w", encoding="utf-8") as f:
    f.write(main_stubs)

for name, stubs_content in submodules.items():
    with open(f"{name}.pyi", "w", encoding="utf-8") as f:
        f.write(stubs_content)
