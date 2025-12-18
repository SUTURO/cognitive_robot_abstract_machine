#!/usr/bin/env python3
"""Simple standalone test script for the LUBM data generator."""

from krrood.generator import UniversityDataGenerator, GeneratorConfiguration
from krrood.lubm import (
    University,
    Department,
    FullProfessor,
    AssociateProfessor,
    AssistantProfessor,
    Lecturer,
    UndergraduateStudent,
    GraduateStudent,
    Course,
    GraduateCourse,
    Professor,
)


def test_basic_generation():
    """Test basic generator functionality."""

    config = GeneratorConfiguration(seed=42)
    generator = UniversityDataGenerator(config)
    universities = generator.generate_universities(1)

    assert len(universities) == 1
    assert universities[0].name == "University0"


def test_with_small_config():
    """Test with a small, controlled configuration."""

    config = GeneratorConfiguration(
        seed=42,
        departments_min=2,
        departments_max=2,
        full_professors_min=2,
        full_professors_max=2,
        associate_professors_min=2,
        associate_professors_max=2,
        assistant_professors_min=2,
        assistant_professors_max=2,
        lecturers_min=1,
        lecturers_max=1,
        undergraduate_per_faculty_min=5,
        undergraduate_per_faculty_max=5,
        graduate_per_faculty_min=2,
        graduate_per_faculty_max=2,
    )
    generator = UniversityDataGenerator(config)
    universities = generator.generate_universities(1)

    assert len(universities) == 1


def test_reproducibility():
    """Test that same seed produces same results."""
    print("\nTesting reproducibility...")

    config1 = GeneratorConfiguration(seed=42, departments_min=1, departments_max=1)
    generator1 = UniversityDataGenerator(config1)
    universities1 = generator1.generate_universities(1)
    courses1 = len(generator1.all_courses)

    config2 = GeneratorConfiguration(seed=42, departments_min=1, departments_max=1)
    generator2 = UniversityDataGenerator(config2)
    universities2 = generator2.generate_universities(1)
    courses2 = len(generator2.all_courses)

    assert universities1[0].name == universities2[0].name
    assert courses1 == courses2


def test_default_configuration():
    """Test with default LUBM configuration."""

    config = GeneratorConfiguration(seed=42)
    generator = UniversityDataGenerator(config)
    universities = generator.generate_universities(1)

    assert len(universities) == 1


def test_multiple_universities():
    """Test generating multiple universities."""
    config = GeneratorConfiguration(seed=42, departments_min=1, departments_max=2)
    generator = UniversityDataGenerator(config)
    universities = generator.generate_universities(3)

    assert len(universities) == 3


def test_configuration_customization():
    """Test that configuration parameters are customizable."""

    configs = [
        GeneratorConfiguration(departments_min=5, departments_max=10),
        GeneratorConfiguration(full_professors_min=3, full_professors_max=5),
        GeneratorConfiguration(
            undergraduate_per_faculty_min=10, undergraduate_per_faculty_max=15
        ),
    ]

    for i, config in enumerate(configs):
        config.seed = 42
        config.departments_min = 1
        config.departments_max = 1
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)
        assert len(universities) == 1


def verify_data_structure():
    """Verify the generated data has proper structure."""
    print("\nVerifying data structure...")
    config = GeneratorConfiguration(
        seed=42,
        departments_min=1,
        departments_max=1,
        full_professors_min=1,
        full_professors_max=1,
        courses_per_faculty_min=1,
        courses_per_faculty_max=1,
    )
    generator = UniversityDataGenerator(config)
    universities = generator.generate_universities(1)

    university = universities[0]
    assert isinstance(university, University)
    assert university.name is not None
