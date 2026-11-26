"""Migrate lerobot/diffusion_pusht to the v0.7.0 processor format, locally (no hub push).

Workaround: the old config stores tuple-typed fields (e.g. crop_shape) as JSON lists, which
draccus 0.11 cannot re-encode when saving. Convert such lists to tuples before the stock script runs.
"""

import dataclasses
import runpy
import sys
import typing

from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig

_orig_post_init = DiffusionConfig.__post_init__
_hints = typing.get_type_hints(DiffusionConfig)


def _post_init(self):
    for f in dataclasses.fields(self):
        value = getattr(self, f.name)
        if isinstance(value, list) and "tuple" in str(_hints.get(f.name, "")):
            setattr(self, f.name, tuple(value))
    _orig_post_init(self)


DiffusionConfig.__post_init__ = _post_init

sys.argv = [
    "migrate_policy_normalization",
    "--pretrained-path",
    "lerobot/diffusion_pusht",
    "--output-dir",
    sys.argv[1] if len(sys.argv) > 1 else "remote_e2e/diffusion_pusht_migrated",
]
runpy.run_module("lerobot.processor.migrate_policy_normalization", run_name="__main__")
