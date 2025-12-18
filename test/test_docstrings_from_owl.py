from __future__ import annotations

import krrood.lubm_with_predicates as lubm


def test_class_docstring_from_label():
    # In LUBM, rdfs:label for Organization is "organization"
    assert lubm.Organization.__doc__ is not None
    assert "organization" in lubm.Organization.__doc__


def test_property_descriptor_docstring_from_label():
    # rdfs:label for worksFor is "Works For" and descriptor is WorksFor
    assert lubm.WorksFor.__doc__ is not None
    assert "Works For" in lubm.WorksFor.__doc__
