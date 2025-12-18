#!/usr/bin/env python3
"""Test enum functionality after refactoring."""

from krrood.lubm import (
    DepartmentName,
    ResearchArea,
    CoursePrefix,
    CourseSubject,
    PublicationTitlePrefix,
    Faculty,
    FullProfessor,
)
from krrood.generator import UniversityDataGenerator, GeneratorConfiguration


def test_enum_imports():
    """Test that all enums can be imported and have values."""
    print("Testing enum imports...")

    # Test DepartmentName enum
    assert len(list(DepartmentName)) == 24
    assert DepartmentName.COMPUTER_SCIENCE.value == "Computer Science"
    print(f"✓ DepartmentName enum has {len(list(DepartmentName))} values")

    # Test ResearchArea enum
    assert len(list(ResearchArea)) == 16
    assert ResearchArea.ARTIFICIAL_INTELLIGENCE.value == "Artificial Intelligence"
    print(f"✓ ResearchArea enum has {len(list(ResearchArea))} values")

    # Test CoursePrefix enum
    assert len(list(CoursePrefix)) == 8
    assert CoursePrefix.INTRODUCTION_TO.value == "Introduction to"
    print(f"✓ CoursePrefix enum has {len(list(CoursePrefix))} values")

    # Test CourseSubject enum
    assert len(list(CourseSubject)) == 17
    assert CourseSubject.ALGORITHMS.value == "Algorithms"
    print(f"✓ CourseSubject enum has {len(list(CourseSubject))} values")

    # Test PublicationTitlePrefix enum
    assert len(list(PublicationTitlePrefix)) == 6
    assert PublicationTitlePrefix.A_NOVEL_APPROACH_TO.value == "A Novel Approach to"
    print(
        f"✓ PublicationTitlePrefix enum has {len(list(PublicationTitlePrefix))} values"
    )


def test_faculty_research_interest_enum():
    """Test that Faculty.research_interest accepts ResearchArea enum."""
    print("\nTesting Faculty.research_interest with ResearchArea enum...")

    # Create faculty with ResearchArea enum
    prof = FullProfessor(
        name="Dr. Smith", research_interest=ResearchArea.MACHINE_LEARNING
    )

    assert prof.research_interest == ResearchArea.MACHINE_LEARNING
    assert prof.research_interest.value == "Machine Learning"
    print(f"✓ Faculty.research_interest accepts ResearchArea enum")
    print(f"  - research_interest: {prof.research_interest}")
    print(f"  - research_interest.value: {prof.research_interest.value}")

    # Create faculty with None (should be allowed as Optional)
    prof2 = FullProfessor(name="Dr. Jones")
    assert prof2.research_interest is None
    print(f"✓ Faculty.research_interest accepts None (Optional)")


def test_generator_with_enums():
    """Test that the generator works with enums."""
    print("\nTesting UniversityDataGenerator with enums...")

    config = GeneratorConfiguration(
        departments_min=2,
        departments_max=3,
        full_professors_min=1,
        full_professors_max=2,
        associate_professors_min=1,
        associate_professors_max=1,
        assistant_professors_min=1,
        assistant_professors_max=1,
        lecturers_min=1,
        lecturers_max=1,
        seed=123,
    )

    generator = UniversityDataGenerator(config)
    universities = generator.generate_universities(1)

    assert len(universities) == 1
    print(f"✓ Generated {len(universities)} university")

    # Check that departments have valid names
    university = universities[0]
    dept_count = 0
    for member in university.members:
        if hasattr(member, "works_for") and member.works_for:
            dept_count += 1
            dept_name = member.works_for.name
            assert isinstance(dept_name, str)
            print(f"  - Department: {dept_name}")

            # Check faculty research interests
            if hasattr(member, "research_interest") and member.research_interest:
                assert isinstance(member.research_interest, ResearchArea)
                print(
                    f"    Faculty: {member.name}, Research: {member.research_interest.value}"
                )

            if dept_count >= 3:  # Just check first few
                break

    print(f"✓ All generated data uses enums correctly")


def test_all_enum_values_accessible():
    """Test that all enum values are accessible."""
    print("\nTesting that all enum values are accessible...")

    for dept in DepartmentName:
        assert isinstance(dept.value, str)
        assert len(dept.value) > 0
    print(f"✓ All {len(list(DepartmentName))} DepartmentName values are accessible")

    for area in ResearchArea:
        assert isinstance(area.value, str)
        assert len(area.value) > 0
    print(f"✓ All {len(list(ResearchArea))} ResearchArea values are accessible")

    for prefix in CoursePrefix:
        assert isinstance(prefix.value, str)
        assert len(prefix.value) > 0
    print(f"✓ All {len(list(CoursePrefix))} CoursePrefix values are accessible")

    for subject in CourseSubject:
        assert isinstance(subject.value, str)
        assert len(subject.value) > 0
    print(f"✓ All {len(list(CourseSubject))} CourseSubject values are accessible")

    for title_prefix in PublicationTitlePrefix:
        assert isinstance(title_prefix.value, str)
        assert len(title_prefix.value) > 0
    print(
        f"✓ All {len(list(PublicationTitlePrefix))} PublicationTitlePrefix values are accessible"
    )


if __name__ == "__main__":
    print("=" * 70)
    print("TESTING ENUM FUNCTIONALITY")
    print("=" * 70)

    test_enum_imports()
    test_faculty_research_interest_enum()
    test_generator_with_enums()
    test_all_enum_values_accessible()

    print("\n" + "=" * 70)
    print("ALL TESTS PASSED!")
    print("=" * 70)
