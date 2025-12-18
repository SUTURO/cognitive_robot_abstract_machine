from __future__ import annotations

from typing import get_args, get_origin, Any

import krrood.lubm_with_predicates as lubm_pd_module
from krrood.lubm_with_predicates import Person, Software


def _eval_if_str(ann):
    if isinstance(ann, str):
        try:
            return eval(ann, vars(lubm_pd_module))
        except Exception:
            return ann
    return ann
