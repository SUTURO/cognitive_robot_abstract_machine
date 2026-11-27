"""
The plans for an instruction, read from and written back to the JSON the model writes.
"""

from __future__ import annotations

import json

from cramspeech.inference.plan import InstructionPlans

from ..dataset.dataset_files import ModelOutputFile


# %% JSON round trip
def test_plans_write_back_the_json_they_were_read_from() -> None:
    """
    Writing the plans read from the model's JSON gives that JSON again, with the summary
    as the opening ``say`` step and without the keys of unset fields.
    """
    payload = json.loads(ModelOutputFile.TWO_PLANS.read_text())

    assert InstructionPlans.from_json(payload).to_json() == payload
