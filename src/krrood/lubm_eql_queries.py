import os
from typing import List

from entity_query_language import symbolic_mode, in_, a, flatten
from entity_query_language.symbolic import ResultQuantifier

from krrood import lubm_with_predicates
from krrood.helpers import evaluate_eql
from krrood.lubm_with_predicates import (
    GraduateStudent,
    GraduateCourse,
    Person,
    Publication,
    Professor,
    AssistantProfessor,
    AssociateProfessor,
    Department,
    University,
    Student,
    Course,
    Faculty,
    ResearchGroup,
    Chair,
    UndergraduateStudent,
    MemberOf,
    SubOrganizationOf,
    UndergraduateDegreeFrom,
    PublicationAuthor,
    TeacherOf,
    WorksFor,
    Advisor,
    HasAlumnus,
    TakesCourse,
)
from krrood.owl_instances_loader import load_instances


def get_eql_queries() -> List[ResultQuantifier]:
    # 1
    with symbolic_mode():
        q1 = a(
            x := GraduateStudent(),
            TakesCourse(x, GraduateCourse(name="GraduateCourse0")),
        )

    # 2
    with symbolic_mode():
        q2 = a(
            x := GraduateStudent(),
            MemberOf(x, z := Department()),
            SubOrganizationOf(z, y := University()),
            UndergraduateDegreeFrom(x, y),
        )

    # 3
    with symbolic_mode():
        q3 = a(
            x := Publication(),
            PublicationAuthor(x, AssistantProfessor(name="AssistantProfessor0")),
        )

    # 4
    with symbolic_mode():
        q4 = a(x := Professor(), WorksFor(x, Department(name="Department0")))

    # 5
    with symbolic_mode():
        q5 = a(
            x := Person(),
            MemberOf(x, Department(name="Department0")),
        )

    # 6
    with symbolic_mode():
        q6 = a(x := Student())

    # 7
    with symbolic_mode():
        q7 = a(
            x := Student(),
            TakesCourse(x, y := Course()),
            TeacherOf(AssociateProfessor(name="AssociateProfessor0"), y),
        )

    # 8
    with symbolic_mode():
        q8 = a(
            x := Student(),
            MemberOf(x, y := Department()),
            SubOrganizationOf(y, University(name="University0")),
        )

    # 9
    with symbolic_mode():
        q9 = a(
            x := Student(),
            Advisor(x, y := Faculty()),
            TeacherOf(y, z := Course()),
            TakesCourse(x, z),
        )

    # 10
    with symbolic_mode():
        q10 = a(x := Student(), TakesCourse(x, GraduateCourse(name="GraduateCourse0")))

    # 11
    with symbolic_mode():
        q11 = a(
            x := ResearchGroup(),
            SubOrganizationOf(x, University(name="University0")),
        )

    # 12
    with symbolic_mode():
        q12 = a(
            x := Chair(),
            WorksFor(x, y := Department()),
            SubOrganizationOf(y, University(name="University0")),
        )

    # 13
    with symbolic_mode():
        q13 = a(
            x := Person(),
            HasAlumnus(University(name="University0"), x),
        )

    # 14
    with symbolic_mode():
        q14 = a(x := UndergraduateStudent())

    eql_queries = [q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, q13, q14]
    return eql_queries


if __name__ == "__main__":
    instances_path = os.path.join("..", "..", "resources", "lubm_instances.owl")
    load_instances(
        instances_path,
        model_module=lubm_with_predicates,
    )
    counts = evaluate_eql(get_eql_queries())
    for i, n in enumerate(counts, 1):
        print(f"{i}:{n}")
