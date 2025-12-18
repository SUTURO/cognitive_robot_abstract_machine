from __future__ import annotations

from typing import get_args, get_origin

import krrood.lubm_with_predicates as lubm_pd_module
from krrood.lubm_with_predicates import (
    Person,
    Chair,
    Dean,
    Director,
    Department,
    College,
    Program,
    HeadOfDepartment,
    HeadOfCollege,
    HeadOfProgram,
    ResearchAssistant,
    ResearchGroup,
    WorksForResearchGroup,
    Student,
    GraduateStudent,
    Course,
    GraduateCourse,
    TakesCourseCourse,
    TakesCourseGraduateCourse,
)


def _eval_if_str(ann):
    if isinstance(ann, str):
        try:
            return eval(ann, vars(lubm_pd_module))
        except Exception:
            return ann
    return ann


def test_person_has_no_head_of():
    assert 'head_of' not in Person.__annotations__


def test_head_of_specialized_descriptors_and_types():
    # Chair heads a Department
    chair_ann = _eval_if_str(Chair.__annotations__['head_of'])
    assert get_origin(chair_ann) in (list, getattr(__import__('typing'), 'List'))
    (inner_chair,) = get_args(chair_ann)
    assert inner_chair is Department
    assert isinstance(Chair.__dict__['head_of'], HeadOfDepartment)

    # Dean heads a College
    dean_ann = _eval_if_str(Dean.__annotations__['head_of'])
    assert get_origin(dean_ann) in (list, getattr(__import__('typing'), 'List'))
    (inner_dean,) = get_args(dean_ann)
    assert inner_dean is College
    assert isinstance(Dean.__dict__['head_of'], HeadOfCollege)

    # Director heads a Program
    director_ann = _eval_if_str(Director.__annotations__['head_of'])
    assert get_origin(director_ann) in (list, getattr(__import__('typing'), 'List'))
    (inner_director,) = get_args(director_ann)
    assert inner_director is Program
    assert isinstance(Director.__dict__['head_of'], HeadOfProgram)


def test_works_for_and_takes_course_specializations():
    # ResearchAssistant works for ResearchGroup
    ra_ann = _eval_if_str(ResearchAssistant.__annotations__['works_for'])
    assert get_origin(ra_ann) in (list, getattr(__import__('typing'), 'List'))
    (inner_ra,) = get_args(ra_ann)
    assert inner_ra is ResearchGroup
    assert isinstance(ResearchAssistant.__dict__['works_for'], WorksForResearchGroup)

    # Student takes Course
    st_ann = _eval_if_str(Student.__annotations__['takes_course'])
    assert get_origin(st_ann) in (list, getattr(__import__('typing'), 'List'))
    (inner_st,) = get_args(st_ann)
    assert inner_st is Course
    assert isinstance(Student.__dict__['takes_course'], TakesCourseCourse)

    # GraduateStudent takes GraduateCourse
    gst_ann = _eval_if_str(GraduateStudent.__annotations__['takes_course'])
    assert get_origin(gst_ann) in (list, getattr(__import__('typing'), 'List'))
    (inner_gst,) = get_args(gst_ann)
    assert inner_gst is GraduateCourse
    assert isinstance(GraduateStudent.__dict__['takes_course'], TakesCourseGraduateCourse)
