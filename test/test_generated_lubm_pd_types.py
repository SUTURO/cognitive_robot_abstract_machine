from __future__ import annotations

from typing import get_args, get_origin, List

import pytest

import krrood.lubm_with_predicates as lubm_pd_module
from krrood.lubm_with_predicates import (
    Person,
    Employee,
    Student,
    Organization,
    Course,
)


def _eval_if_str(ann):
    if isinstance(ann, str):
        try:
            return eval(ann, vars(lubm_pd_module))
        except Exception:
            return ann
    return ann


def _is_list_of(annotation, expected_inner):
    ann = _eval_if_str(annotation)
    return get_origin(ann) is list and get_args(ann)[0] is expected_inner


def test_properties_declared_on_most_specific_classes():
    # works_for should first appear on Employee, not Person
    assert 'works_for' not in Person.__annotations__
    assert 'works_for' in Employee.__annotations__
    assert _is_list_of(Employee.__annotations__['works_for'], Organization)

    # takes_course should first appear on Student, not Person
    assert 'takes_course' not in Person.__annotations__
    assert 'takes_course' in Student.__annotations__
    assert _is_list_of(Student.__annotations__['takes_course'], Course)


@pytest.mark.parametrize("cls, attr", [
    (Organization, 'works_for'),
    (Organization, 'takes_course'),
])
def test_organization_does_not_have_person_specific_properties(cls, attr):
    assert attr not in cls.__annotations__
