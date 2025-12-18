#!/usr/bin/env python3
"""
Example usage of the LUBM Data Generator.

This script demonstrates how to use the customizable LUBM data generator
to create university data following the LUBM benchmark specifications.
"""

from krrood.generator import UniversityDataGenerator, GeneratorConfiguration


def example_basic_usage():
    """Basic usage with default configuration."""
    print("=" * 70)
    print("Example 1: Basic Usage with Default Configuration")
    print("=" * 70)

    # Create generator with default LUBM benchmark configuration
    generator = UniversityDataGenerator()

    # Generate 2 universities
    universities = generator.generate_universities(2)

    print(f"Generated {len(universities)} universities")
    for university in universities:
        print(f"  - {university.name}")

    print(f"Total courses generated: {len(generator.all_courses)}")
    print(f"Total graduate courses: {len(generator.all_graduate_courses)}")
    print()


def example_custom_configuration():
    """Using custom configuration for smaller datasets."""
    print("=" * 70)
    print("Example 2: Custom Configuration for Small Dataset")
    print("=" * 70)

    # Create a custom configuration for a smaller university
    config = GeneratorConfiguration(
        seed=42,  # For reproducibility
        departments_min=5,
        departments_max=8,
        full_professors_min=3,
        full_professors_max=5,
        associate_professors_min=5,
        associate_professors_max=8,
        assistant_professors_min=4,
        assistant_professors_max=6,
        lecturers_min=2,
        lecturers_max=4,
        undergraduate_per_faculty_min=10,
        undergraduate_per_faculty_max=12,
        graduate_per_faculty_min=3,
        graduate_per_faculty_max=4,
    )

    generator = UniversityDataGenerator(config)
    universities = generator.generate_universities(1)

    print(f"Generated {len(universities)} university with custom configuration")
    print(f"Seed: {config.seed}")
    print(
        f"Departments: {config.departments_min}-{config.departments_max} per university"
    )
    print(f"Courses generated: {len(generator.all_courses)}")
    print()


def example_reproducible_generation():
    """Demonstrate reproducible generation with seeds."""
    print("=" * 70)
    print("Example 3: Reproducible Generation with Seeds")
    print("=" * 70)

    # Generate with seed 42
    config1 = GeneratorConfiguration(seed=42, departments_min=3, departments_max=3)
    generator1 = UniversityDataGenerator(config1)
    universities1 = generator1.generate_universities(1)
    courses1 = len(generator1.all_courses)

    # Generate again with same seed
    config2 = GeneratorConfiguration(seed=42, departments_min=3, departments_max=3)
    generator2 = UniversityDataGenerator(config2)
    universities2 = generator2.generate_universities(1)
    courses2 = len(generator2.all_courses)

    print(f"Run 1 with seed=42: Generated {courses1} courses")
    print(f"Run 2 with seed=42: Generated {courses2} courses")
    print(f"Results are identical: {courses1 == courses2}")
    print()


def example_large_scale():
    """Generate a large-scale university system."""
    print("=" * 70)
    print("Example 4: Large-Scale University System")
    print("=" * 70)

    # Use default LUBM benchmark configuration
    config = GeneratorConfiguration(seed=12345)
    generator = UniversityDataGenerator(config)

    # Generate 5 universities
    universities = generator.generate_universities(5)

    print(f"Generated {len(universities)} universities")
    print(f"Total courses: {len(generator.all_courses)}")
    print(f"Total graduate courses: {len(generator.all_graduate_courses)}")
    print("\nThis follows the LUBM benchmark rules:")
    print("  - 15-25 departments per university")
    print("  - 7-10 Full Professors per department")
    print("  - 10-14 Associate Professors per department")
    print("  - 8-11 Assistant Professors per department")
    print("  - 5-7 Lecturers per department")
    print("  - Student-to-faculty ratios maintained")
    print("  - Publications assigned by faculty type")
    print("  - Teaching and research assistants assigned")
    print()


def example_configuration_options():
    """Show all available configuration options."""
    print("=" * 70)
    print("Example 5: Available Configuration Options")
    print("=" * 70)

    config = GeneratorConfiguration()

    print("Department Configuration:")
    print(f"  departments_min: {config.departments_min}")
    print(f"  departments_max: {config.departments_max}")

    print("\nFaculty Configuration:")
    print(
        f"  full_professors_min/max: {config.full_professors_min}-{config.full_professors_max}"
    )
    print(
        f"  associate_professors_min/max: {config.associate_professors_min}-{config.associate_professors_max}"
    )
    print(
        f"  assistant_professors_min/max: {config.assistant_professors_min}-{config.assistant_professors_max}"
    )
    print(f"  lecturers_min/max: {config.lecturers_min}-{config.lecturers_max}")

    print("\nStudent Configuration:")
    print(
        f"  undergraduate_per_faculty_min/max: {config.undergraduate_per_faculty_min}-{config.undergraduate_per_faculty_max}"
    )
    print(
        f"  graduate_per_faculty_min/max: {config.graduate_per_faculty_min}-{config.graduate_per_faculty_max}"
    )

    print("\nCourse Configuration:")
    print(
        f"  courses_per_faculty_min/max: {config.courses_per_faculty_min}-{config.courses_per_faculty_max}"
    )
    print(
        f"  graduate_courses_per_faculty_min/max: {config.graduate_courses_per_faculty_min}-{config.graduate_courses_per_faculty_max}"
    )

    print("\nPublication Configuration:")
    print(
        f"  full_professor_publications_min/max: {config.full_professor_publications_min}-{config.full_professor_publications_max}"
    )
    print(
        f"  associate_professor_publications_min/max: {config.associate_professor_publications_min}-{config.associate_professor_publications_max}"
    )
    print(
        f"  assistant_professor_publications_min/max: {config.assistant_professor_publications_min}-{config.assistant_professor_publications_max}"
    )
    print(
        f"  lecturer_publications_min/max: {config.lecturer_publications_min}-{config.lecturer_publications_max}"
    )

    print("\nOther Configuration:")
    print(
        f"  research_groups_min/max: {config.research_groups_min}-{config.research_groups_max}"
    )
    print(
        f"  teaching_assistant_ratio: {config.teaching_assistant_ratio_min}-{config.teaching_assistant_ratio_max}"
    )
    print(
        f"  research_assistant_ratio: {config.research_assistant_ratio_min}-{config.research_assistant_ratio_max}"
    )
    print(f"  seed: {config.seed} (None = random)")
    print()


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("LUBM DATA GENERATOR - USAGE EXAMPLES")
    print("=" * 70)
    print()

    example_basic_usage()
    example_custom_configuration()
    example_reproducible_generation()
    example_large_scale()
    example_configuration_options()

    print("=" * 70)
    print("For more information, see the GeneratorConfiguration class")
    print("and the UniversityDataGenerator class in src/krrood/generator.py")
    print("=" * 70)
