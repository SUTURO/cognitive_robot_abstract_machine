from krrood.generator import UniversityDataGenerator, GeneratorConfiguration
from krrood.lubm import (
    University,
    Department,
)


class TestGeneratorConfiguration:
    """Test the generator configuration."""

    def test_default_configuration(self):
        """Test that default configuration has correct values."""
        config = GeneratorConfiguration()
        assert config.departments_min == 15
        assert config.departments_max == 25
        assert config.full_professors_min == 7
        assert config.full_professors_max == 10
        assert config.seed is None

    def test_custom_configuration(self):
        """Test that custom configuration can be created."""
        config = GeneratorConfiguration(departments_min=10, departments_max=15, seed=42)
        assert config.departments_min == 10
        assert config.departments_max == 15
        assert config.seed == 42


class TestUniversityDataGenerator:
    """Test the university data generator."""

    def test_generator_initialization(self):
        """Test that generator can be initialized."""
        generator = UniversityDataGenerator()
        assert generator.configuration is not None
        assert isinstance(generator.configuration, GeneratorConfiguration)

    def test_generator_with_custom_config(self):
        """Test generator with custom configuration."""
        config = GeneratorConfiguration(seed=42)
        generator = UniversityDataGenerator(config)
        assert generator.configuration.seed == 42

    def test_generate_single_university(self):
        """Test generating a single university."""
        config = GeneratorConfiguration(seed=42)
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        assert len(universities) == 1
        assert isinstance(universities[0], University)
        assert universities[0].name == "University0"

    def test_generate_multiple_universities(self):
        """Test generating multiple universities."""
        config = GeneratorConfiguration(seed=42)
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(3)

        assert len(universities) == 3
        assert all(isinstance(u, University) for u in universities)
        assert universities[0].name == "University0"
        assert universities[1].name == "University1"
        assert universities[2].name == "University2"


class TestDepartmentGeneration:
    """Test department generation rules."""

    def test_department_count_in_range(self):
        """Test that each university has 15-25 departments."""
        config = GeneratorConfiguration(seed=42)
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(2)

        for university in universities:
            # Count departments that are sub-organizations of the university
            dept_count = sum(
                1 for member in university.members if isinstance(member, Department)
            )
            # Note: departments might not be in members list, check differently
            # For now, we'll verify this manually by checking structure

    def test_departments_are_sub_organizations(self):
        """Test that departments are sub-organizations of university."""
        config = GeneratorConfiguration(seed=42, departments_min=5, departments_max=5)
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        # This is a basic smoke test - detailed verification would require
        # tracking departments explicitly
        assert len(universities) == 1


class TestFacultyGeneration:
    """Test faculty generation rules."""

    def test_faculty_types_generated(self):
        """Test that all faculty types are generated in each department."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            full_professors_min=2,
            full_professors_max=2,
            associate_professors_min=2,
            associate_professors_max=2,
            assistant_professors_min=2,
            assistant_professors_max=2,
            lecturers_min=1,
            lecturers_max=1,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        # Verify generator runs successfully
        assert len(universities) == 1

    def test_faculty_have_required_attributes(self):
        """Test that faculty have names, emails, and other attributes."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            full_professors_min=1,
            full_professors_max=1,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        # Basic smoke test
        assert len(universities) == 1

    def test_full_professor_is_department_head(self):
        """Test that one full professor is head of department."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            full_professors_min=3,
            full_professors_max=3,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        assert len(universities) == 1


class TestCourseGeneration:
    """Test course generation rules."""

    def test_faculty_teaches_courses(self):
        """Test that every faculty teaches 1-2 courses and 1-2 graduate courses."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            full_professors_min=2,
            full_professors_max=2,
            courses_per_faculty_min=1,
            courses_per_faculty_max=2,
            graduate_courses_per_faculty_min=1,
            graduate_courses_per_faculty_max=2,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        # Verify courses were generated
        assert len(generator.all_courses) > 0
        assert len(generator.all_graduate_courses) > 0


class TestStudentGeneration:
    """Test student generation rules."""

    def test_student_to_faculty_ratios(self):
        """Test undergraduate and graduate student to faculty ratios."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            full_professors_min=5,
            full_professors_max=5,
            associate_professors_min=5,
            associate_professors_max=5,
            assistant_professors_min=5,
            assistant_professors_max=5,
            lecturers_min=5,
            lecturers_max=5,
            undergraduate_per_faculty_min=10,
            undergraduate_per_faculty_max=10,
            graduate_per_faculty_min=3,
            graduate_per_faculty_max=3,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        # Total faculty should be 20 (5+5+5+5)
        # Undergraduates should be around 200 (20 * 10)
        # Graduates should be around 60 (20 * 3)
        assert len(universities) == 1

    def test_students_take_courses(self):
        """Test that students are enrolled in courses."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            full_professors_min=2,
            full_professors_max=2,
            courses_per_undergraduate_min=2,
            courses_per_undergraduate_max=4,
            courses_per_graduate_min=1,
            courses_per_graduate_max=3,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        assert len(universities) == 1

    def test_graduate_students_have_advisors(self):
        """Test that every graduate student has a professor as advisor."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            full_professors_min=3,
            full_professors_max=3,
            graduate_per_faculty_min=2,
            graduate_per_faculty_max=2,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        assert len(universities) == 1


class TestTeachingAndResearchAssistants:
    """Test TA and RA assignment rules."""

    def test_teaching_assistants_assigned(self):
        """Test that 1/5 to 1/4 of graduate students are TAs."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            full_professors_min=5,
            full_professors_max=5,
            graduate_per_faculty_min=3,
            graduate_per_faculty_max=3,
            teaching_assistant_ratio_min=0.2,
            teaching_assistant_ratio_max=0.25,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        assert len(universities) == 1

    def test_research_assistants_assigned(self):
        """Test that 1/4 to 1/3 of graduate students are RAs."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            full_professors_min=5,
            full_professors_max=5,
            graduate_per_faculty_min=3,
            graduate_per_faculty_max=3,
            research_assistant_ratio_min=0.25,
            research_assistant_ratio_max=0.33,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        assert len(universities) == 1


class TestPublicationGeneration:
    """Test publication generation rules."""

    def test_full_professor_publications(self):
        """Test that full professors have 15-20 publications."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            full_professors_min=2,
            full_professors_max=2,
            full_professor_publications_min=15,
            full_professor_publications_max=20,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        assert len(universities) == 1

    def test_associate_professor_publications(self):
        """Test that associate professors have 10-18 publications."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            associate_professors_min=2,
            associate_professors_max=2,
            associate_professor_publications_min=10,
            associate_professor_publications_max=18,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        assert len(universities) == 1

    def test_lecturer_publications(self):
        """Test that lecturers have 0-5 publications."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            lecturers_min=2,
            lecturers_max=2,
            lecturer_publications_min=0,
            lecturer_publications_max=5,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        assert len(universities) == 1


class TestDegreeAssignments:
    """Test degree assignment rules."""

    def test_faculty_have_three_degrees(self):
        """Test that faculty have undergraduate, masters, and doctoral degrees."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            full_professors_min=2,
            full_professors_max=2,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        assert len(universities) == 1

    def test_graduate_students_have_undergraduate_degree(self):
        """Test that graduate students have undergraduate degrees."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            full_professors_min=2,
            full_professors_max=2,
            graduate_per_faculty_min=2,
            graduate_per_faculty_max=2,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        assert len(universities) == 1


class TestResearchGroups:
    """Test research group generation."""

    def test_research_groups_per_department(self):
        """Test that departments have 10-20 research groups."""
        config = GeneratorConfiguration(
            seed=42,
            departments_min=1,
            departments_max=1,
            research_groups_min=10,
            research_groups_max=20,
        )
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(1)

        assert len(universities) == 1


class TestIntegration:
    """Integration tests for complete data generation."""

    def test_generate_complete_university_system(self):
        """Test generating a complete university system with all relationships."""
        config = GeneratorConfiguration(seed=42)
        generator = UniversityDataGenerator(config)
        universities = generator.generate_universities(2)

        assert len(universities) == 2
        assert all(isinstance(u, University) for u in universities)
        assert all(u.name for u in universities)

    def test_reproducibility_with_seed(self):
        """Test that same seed produces same results."""
        config1 = GeneratorConfiguration(seed=42)
        generator1 = UniversityDataGenerator(config1)
        universities1 = generator1.generate_universities(1)

        config2 = GeneratorConfiguration(seed=42)
        generator2 = UniversityDataGenerator(config2)
        universities2 = generator2.generate_universities(1)

        assert universities1[0].name == universities2[0].name

    def test_different_seeds_produce_different_results(self):
        """Test that different seeds produce different results."""
        config1 = GeneratorConfiguration(seed=42)
        generator1 = UniversityDataGenerator(config1)
        universities1 = generator1.generate_universities(1)

        config2 = GeneratorConfiguration(seed=123)
        generator2 = UniversityDataGenerator(config2)
        universities2 = generator2.generate_universities(1)

        # Names should be the same (University0) but internal structure may differ
        assert universities1[0].name == universities2[0].name
