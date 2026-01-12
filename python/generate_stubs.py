#!/usr/bin/env python3
"""Generate type stubs for sdu_controllers and its submodules using nanobind.stubgen"""

from nanobind.stubgen import StubGen
import _sdu_controllers

print("Generating stubs for sdu_controllers...")

# Generate stubs for main module
sg_main = StubGen(_sdu_controllers)
sg_main.put(_sdu_controllers)
main_stubs = sg_main.get()

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
                    submodules[attr_name] = sg.get()
                except Exception as e:
                    print(f"    Warning: Could not generate stubs for {attr_name}: {e}")
        except Exception as e:
            pass

# Print main stubs
print("\n" + "="*80)
print("Main module stubs:")
print("="*80)
print(main_stubs if main_stubs.strip() else "[Empty - bindings may not be fully exposed]")

# Print submodule stubs
for name, stubs_content in submodules.items():
    print("\n" + "="*80)
    print(f"Stubs for {name}:")
    print("="*80)
    print(stubs_content if stubs_content.strip() else f"[Empty for {name}]")

# Save to files
with open("sdu_controllers.pyi", "w") as f:
    f.write(main_stubs)

for name, stubs_content in submodules.items():
    with open(f"{name}.pyi", "w") as f:
        f.write(stubs_content)

print("\n" + "="*80)
print("Stubs written to:")
print(f"  - sdu_controllers.pyi (main module)")
for name in submodules.keys():
    print(f"  - sdu_controllers.{name}.pyi (submodule)")
print("="*80)
