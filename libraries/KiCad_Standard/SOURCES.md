# Bundled KiCad standard assets

These assets were copied from the installed KiCad 10.0 standard libraries on September 29, 2026. They are provided by the KiCad library community under the license in `LICENSE.md`.

Upstream sources:

- Symbols: https://gitlab.com/kicad/libraries/kicad-symbols
- Footprints: https://gitlab.com/kicad/libraries/kicad-footprints
- 3D models: https://gitlab.com/kicad/libraries/kicad-packages3D
- Library license: https://gitlab.com/kicad/libraries/kicad-footprints/-/raw/master/LICENSE.md

The five symbol libraries are copied intact. The five footprint libraries contain only footprints assigned in the student starter. Their geometry is unchanged; their 3D model paths were changed from the KiCad installation variable to `${KIPRJMOD}/libraries/KiCad_Standard/3dmodels/...` for portability. The referenced STEP models are copied unchanged.

The project library tables point these library nicknames to the bundled copies. To use additional standard footprints, extend the local subsets or assign a distinct nickname to the full installed library.
