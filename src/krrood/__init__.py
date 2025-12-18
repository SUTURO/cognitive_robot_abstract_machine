__version__ = "0.1.0"

import logging

logger = logging.Logger("krrood")
logger.setLevel(logging.INFO)

# Regenerate generated module on package import to ensure latest ontology mapping
try:
    from .helpers import generate_lubm_with_predicates

    generate_lubm_with_predicates()
except Exception:
    # Non-fatal: generation is a build-time helper
    pass
