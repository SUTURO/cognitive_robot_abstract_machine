from dataclasses import dataclass, field
from typing import List, Optional
import random
from datetime import datetime, timedelta

import tqdm

from .lubm import (
    University,
    Department,
    ResearchGroup,
    FullProfessor,
    AssociateProfessor,
    AssistantProfessor,
    Lecturer,
    UndergraduateStudent,
    GraduateStudent,
    Course,
    GraduateCourse,
    Publication,
    TeachingAssistant,
    ResearchAssistant,
    Person,
    Professor,
    Faculty,
    DepartmentName,
    ResearchArea,
    CoursePrefix,
    CourseSubject,
    PublicationTitlePrefix,
    Student,
    Organization,
    Schedule,
    Work,
)


@dataclass
class GeneratorConfiguration:
    """Configuration for customizing the LUBM data generator.

    Controls all parameters for generating university benchmark data according
    to the LUBM specification, including faculty counts, student ratios,
    course assignments, and publication counts.
    """

    departments_min: int = 15
    departments_max: int = 25

    full_professors_min: int = 7
    full_professors_max: int = 10

    associate_professors_min: int = 10
    associate_professors_max: int = 14

    assistant_professors_min: int = 8
    assistant_professors_max: int = 11

    lecturers_min: int = 5
    lecturers_max: int = 7

    research_groups_min: int = 10
    research_groups_max: int = 20

    undergraduate_per_faculty_min: int = 8
    undergraduate_per_faculty_max: int = 14

    graduate_per_faculty_min: int = 3
    graduate_per_faculty_max: int = 4

    courses_per_faculty_min: int = 1
    courses_per_faculty_max: int = 2

    graduate_courses_per_faculty_min: int = 1
    graduate_courses_per_faculty_max: int = 2

    courses_per_undergraduate_min: int = 2
    courses_per_undergraduate_max: int = 4

    courses_per_graduate_min: int = 1
    courses_per_graduate_max: int = 3

    teaching_assistant_ratio_min: float = 0.2  # 1/5
    teaching_assistant_ratio_max: float = 0.25  # 1/4

    research_assistant_ratio_min: float = 0.25  # 1/4
    research_assistant_ratio_max: float = 0.33  # 1/3

    undergraduate_with_advisor_ratio: float = 0.2  # 1/5

    full_professor_publications_min: int = 15
    full_professor_publications_max: int = 20

    associate_professor_publications_min: int = 10
    associate_professor_publications_max: int = 18

    assistant_professor_publications_min: int = 5
    assistant_professor_publications_max: int = 10

    lecturer_publications_min: int = 0
    lecturer_publications_max: int = 5

    graduate_coauthor_publications_min: int = 0
    graduate_coauthor_publications_max: int = 5

    seed: Optional[int] = None


class RandomDataHelper:
    """Helper class for generating random data for the LUBM benchmark.

    This class provides static methods and data lists for generating realistic
    random values used throughout the university data generation process.
    """

    FIRST_NAMES = [
        "James",
        "Mary",
        "John",
        "Patricia",
        "Robert",
        "Jennifer",
        "Michael",
        "Linda",
        "William",
        "Barbara",
        "David",
        "Elizabeth",
        "Richard",
        "Susan",
        "Joseph",
        "Jessica",
        "Thomas",
        "Sarah",
        "Charles",
        "Karen",
        "Christopher",
        "Nancy",
        "Daniel",
        "Lisa",
        "Matthew",
        "Betty",
        "Anthony",
        "Margaret",
        "Mark",
        "Sandra",
        "Donald",
        "Ashley",
        "Steven",
        "Kimberly",
        "Paul",
        "Emily",
        "Andrew",
        "Donna",
        "Joshua",
        "Michelle",
    ]

    LAST_NAMES = [
        "Smith",
        "Johnson",
        "Williams",
        "Brown",
        "Jones",
        "Garcia",
        "Miller",
        "Davis",
        "Rodriguez",
        "Martinez",
        "Hernandez",
        "Lopez",
        "Gonzalez",
        "Wilson",
        "Anderson",
        "Thomas",
        "Taylor",
        "Moore",
        "Jackson",
        "Martin",
        "Lee",
        "Perez",
        "Thompson",
        "White",
        "Harris",
        "Sanchez",
        "Clark",
        "Ramirez",
        "Lewis",
        "Robinson",
        "Walker",
        "Young",
        "Allen",
        "King",
        "Wright",
        "Scott",
        "Torres",
        "Nguyen",
        "Hill",
        "Flores",
    ]

    DEPARTMENT_NAMES = list(DepartmentName)

    RESEARCH_AREAS = list(ResearchArea)

    COURSE_PREFIXES = list(CoursePrefix)

    COURSE_SUBJECTS = list(CourseSubject)

    @staticmethod
    def generate_name() -> str:
        """Generate a random person name by combining a first and last name."""
        first = random.choice(RandomDataHelper.FIRST_NAMES)
        last = random.choice(RandomDataHelper.LAST_NAMES)
        return f"{first} {last}"

    @staticmethod
    def generate_email(name: str, organization: str) -> str:
        """Generate an email address using first initial, last name, and organization."""
        parts = name.lower().split()
        username = f"{parts[0][0]}{parts[-1]}" if len(parts) > 1 else parts[0]
        org_part = organization.lower().replace(" ", "")[:10]
        return f"{username}@{org_part}.edu"

    @staticmethod
    def generate_department_name(existing_names: List[str]) -> str:
        """Generate a unique department name, avoiding those already in use."""
        available = [
            name
            for name in RandomDataHelper.DEPARTMENT_NAMES
            if name.value not in existing_names
        ]
        if not available:
            return f"Department of Science {len(existing_names)}"
        return random.choice(available).value

    @staticmethod
    def generate_course_name() -> str:
        """Generate a course name by combining a prefix and subject."""
        prefix = random.choice(RandomDataHelper.COURSE_PREFIXES)
        subject = random.choice(RandomDataHelper.COURSE_SUBJECTS)
        return f"{prefix.value} {subject.value}"

    @staticmethod
    def generate_research_interest() -> ResearchArea:
        """Generate a random research area."""
        return random.choice(RandomDataHelper.RESEARCH_AREAS)

    @staticmethod
    def generate_publication_title() -> str:
        """Generate a publication title by combining a prefix and research area."""
        prefix = random.choice(list(PublicationTitlePrefix))
        subject = random.choice(RandomDataHelper.RESEARCH_AREAS)
        return f"{prefix.value} {subject.value}"

    @staticmethod
    def generate_publication_date() -> str:
        """Generate a random publication date within the last 10 years."""
        end_date = datetime.now()
        start_date = end_date - timedelta(days=365 * 10)
        random_date = start_date + timedelta(
            days=random.randint(0, (end_date - start_date).days)
        )
        return random_date.strftime("%Y-%m-%d")


class UniversityDataGenerator:
    """Main generator for creating university data following LUBM benchmark rules.

    This class orchestrates the generation of complete university datasets including
    departments, faculty, students, courses, publications, and all their relationships
    according to the LUBM (Lehigh University Benchmark) specification.
    """

    def __init__(self, configuration: Optional[GeneratorConfiguration] = None):
        """Initialize the generator with optional configuration and random seed."""
        self.configuration = configuration or GeneratorConfiguration()
        if self.configuration.seed is not None:
            random.seed(self.configuration.seed)

        self.all_universities: List[University] = []
        self.all_departments: List[Department] = []
        self.all_research_groups: List[ResearchGroup] = []
        self.all_courses: List[Course] = []
        self.all_graduate_courses: List[GraduateCourse] = []
        self.all_publications: List[Publication] = []

    def generate_universities(self, count: int) -> List[University]:
        """Generate universities with complete departments, faculty, students, and courses."""
        universities = []

        for i in range(count):
            university = self._generate_single_university(i)
            universities.append(university)

        self.all_universities = universities
        return universities

    def _generate_single_university(self, index: int) -> University:
        """Generate a single university with all its departments."""
        university_name = f"University{index}"
        university = University(name=university_name)

        department_count = random.randint(
            self.configuration.departments_min, self.configuration.departments_max
        )

        existing_dept_names = []

        for dept_index in range(department_count):
            department = self._generate_department(
                university, dept_index, existing_dept_names
            )
            existing_dept_names.append(department.name)

        return university

    def _generate_department(
        self, university: University, index: int, existing_names: List[str]
    ) -> Department:
        """Generate a complete department with faculty, students, courses, and publications."""
        dept_name = RandomDataHelper.generate_department_name(existing_names)
        department = Department(name=dept_name, sub_organization_of=university)
        self.all_departments.append(department)

        # Generate faculty
        full_professors = self._generate_faculty_list(
            FullProfessor,
            department,
            self.configuration.full_professors_min,
            self.configuration.full_professors_max,
        )

        associate_professors = self._generate_faculty_list(
            AssociateProfessor,
            department,
            self.configuration.associate_professors_min,
            self.configuration.associate_professors_max,
        )

        assistant_professors = self._generate_faculty_list(
            AssistantProfessor,
            department,
            self.configuration.assistant_professors_min,
            self.configuration.assistant_professors_max,
        )

        lecturers = self._generate_faculty_list(
            Lecturer,
            department,
            self.configuration.lecturers_min,
            self.configuration.lecturers_max,
        )

        all_faculty = (
            full_professors + associate_professors + assistant_professors + lecturers
        )
        all_professors = full_professors + associate_professors + assistant_professors

        # Set department head
        if full_professors:
            head = random.choice(full_professors)
            head.head_of = department

        # Assign degrees to all faculty
        for faculty_member in all_faculty:
            self._assign_faculty_degrees(faculty_member)

        # Generate courses
        regular_courses, grad_courses = self._generate_courses_for_faculty(
            all_faculty, department
        )
        self.all_courses.extend(regular_courses)
        self.all_graduate_courses.extend(grad_courses)

        # Generate students
        total_faculty = len(all_faculty)
        undergraduate_count = random.randint(
            total_faculty * self.configuration.undergraduate_per_faculty_min,
            total_faculty * self.configuration.undergraduate_per_faculty_max,
        )
        graduate_count = random.randint(
            total_faculty * self.configuration.graduate_per_faculty_min,
            total_faculty * self.configuration.graduate_per_faculty_max,
        )

        undergraduates = self._generate_undergraduate_students(
            undergraduate_count, department, regular_courses, all_professors
        )

        graduates = self._generate_graduate_students(
            graduate_count, department, grad_courses, all_professors
        )

        # Assign teaching assistants
        self._assign_teaching_assistants(graduates, regular_courses)

        # Assign research assistants
        self._assign_research_assistants(graduates, department)

        # Generate publications
        self._generate_publications_for_faculty(
            full_professors,
            graduates,
            self.configuration.full_professor_publications_min,
            self.configuration.full_professor_publications_max,
        )

        self._generate_publications_for_faculty(
            associate_professors,
            graduates,
            self.configuration.associate_professor_publications_min,
            self.configuration.associate_professor_publications_max,
        )

        self._generate_publications_for_faculty(
            assistant_professors,
            graduates,
            self.configuration.assistant_professor_publications_min,
            self.configuration.assistant_professor_publications_max,
        )

        self._generate_publications_for_faculty(
            lecturers,
            graduates,
            self.configuration.lecturer_publications_min,
            self.configuration.lecturer_publications_max,
        )

        # Generate research groups
        research_group_count = random.randint(
            self.configuration.research_groups_min,
            self.configuration.research_groups_max,
        )

        for rg_index in range(research_group_count):
            research_group = ResearchGroup(
                name=f"{dept_name} Research Group {rg_index + 1}",
                sub_organization_of=department,
            )
            self.all_research_groups.append(research_group)

        return department

    def _generate_faculty_list(
        self, faculty_class, department: Department, min_count: int, max_count: int
    ) -> List[Faculty]:
        """Generate a list of faculty members of a specific type for a department."""
        count = random.randint(min_count, max_count)
        faculty_list = []

        for i in range(count):
            name = RandomDataHelper.generate_name()
            email = RandomDataHelper.generate_email(name, department.name)

            faculty = faculty_class(
                name=name,
                email_address=email,
                works_for=department,
                office_number=f"{department.name[:3].upper()}-{random.randint(100, 999)}",
                research_interest=RandomDataHelper.generate_research_interest(),
                member_of=[department],
            )

            if isinstance(faculty, Professor):
                faculty.tenured = random.choice([True, False])

            faculty_list.append(faculty)
            department.members.append(faculty)

        return faculty_list

    def _assign_faculty_degrees(self, faculty: Faculty) -> None:
        """Assign placeholder degree universities to a faculty member."""
        # Pick random universities for degrees
        available_universities = self.all_universities if self.all_universities else []

        # For now, create placeholder universities
        undergrad_univ = University(name=f"University_UG_{random.randint(1, 100)}")
        masters_univ = University(name=f"University_MS_{random.randint(1, 100)}")
        doctoral_univ = University(name=f"University_PhD_{random.randint(1, 100)}")

        faculty.undergraduate_degree_from = [undergrad_univ]
        faculty.masters_degree_from = [masters_univ]
        faculty.doctoral_degree_from = [doctoral_univ]

    def _generate_courses_for_faculty(
        self, faculty_list: List[Faculty], department: Department
    ) -> tuple[List[Course], List[GraduateCourse]]:
        """Generate regular and graduate courses for faculty members."""
        regular_courses = []
        graduate_courses = []

        for faculty in faculty_list:
            # Regular courses
            course_count = random.randint(
                self.configuration.courses_per_faculty_min,
                self.configuration.courses_per_faculty_max,
            )

            for _ in range(course_count):
                course = Course(
                    name=f"{RandomDataHelper.generate_course_name()} ({department.name})",
                    teacher=faculty,
                )
                regular_courses.append(course)
                faculty.teaches.append(course)

            # Graduate courses
            grad_course_count = random.randint(
                self.configuration.graduate_courses_per_faculty_min,
                self.configuration.graduate_courses_per_faculty_max,
            )

            for _ in range(grad_course_count):
                grad_course = GraduateCourse(
                    name=f"{RandomDataHelper.generate_course_name()} (Graduate) ({department.name})",
                    teacher=faculty,
                )
                graduate_courses.append(grad_course)
                faculty.teaches.append(grad_course)

        return regular_courses, graduate_courses

    def _generate_undergraduate_students(
        self,
        count: int,
        department: Department,
        available_courses: List[Course],
        professors: List[Professor],
    ) -> List[UndergraduateStudent]:
        """Generate undergraduate students with course enrollments and optional advisors."""
        students = []

        for i in range(count):
            name = RandomDataHelper.generate_name()
            email = RandomDataHelper.generate_email(name, department.name)

            student = UndergraduateStudent(
                name=name, email_address=email, member_of=[department]
            )

            # Assign courses
            if available_courses:
                course_count = random.randint(
                    self.configuration.courses_per_undergraduate_min,
                    self.configuration.courses_per_undergraduate_max,
                )
                student.takes_courses = random.sample(
                    available_courses, min(course_count, len(available_courses))
                )

            # Assign advisor (1/5 of undergraduates)
            if (
                professors
                and random.random()
                < self.configuration.undergraduate_with_advisor_ratio
            ):
                student.advisor = random.choice(professors)

            # Assign undergraduate degree from a university
            undergrad_univ = University(
                name=f"Previous_University_{random.randint(1, 50)}"
            )
            student.undergraduate_degree_from = [undergrad_univ]

            students.append(student)
            department.members.append(student)

        return students

    def _generate_graduate_students(
        self,
        count: int,
        department: Department,
        available_courses: List[GraduateCourse],
        professors: List[Professor],
    ) -> List[GraduateStudent]:
        """Generate graduate students with course enrollments and required advisors."""
        students = []

        for i in range(count):
            name = RandomDataHelper.generate_name()
            email = RandomDataHelper.generate_email(name, department.name)

            student = GraduateStudent(
                name=name, email_address=email, member_of=[department]
            )

            # Assign courses
            if available_courses:
                course_count = random.randint(
                    self.configuration.courses_per_graduate_min,
                    self.configuration.courses_per_graduate_max,
                )
                student.takes_courses = random.sample(
                    available_courses, min(course_count, len(available_courses))
                )

            # Every graduate student has an advisor
            if professors:
                student.advisor = random.choice(professors)

            # Assign undergraduate degree
            undergrad_univ = University(
                name=f"Previous_University_{random.randint(1, 50)}"
            )
            student.undergraduate_degree_from = [undergrad_univ]

            students.append(student)
            department.members.append(student)

        return students

    def _assign_teaching_assistants(
        self, graduate_students: List[GraduateStudent], courses: List[Course]
    ) -> None:
        """Assign a portion of graduate students as teaching assistants for courses."""
        if not graduate_students or not courses:
            return

        ta_count = int(
            len(graduate_students)
            * random.uniform(
                self.configuration.teaching_assistant_ratio_min,
                self.configuration.teaching_assistant_ratio_max,
            )
        )

        selected_students = random.sample(
            graduate_students, min(ta_count, len(graduate_students))
        )
        available_courses = courses.copy()
        random.shuffle(available_courses)

        for i, student in enumerate(selected_students):
            if i < len(available_courses):
                course = available_courses[i]
                # Create a teaching assistant role (reusing the student with TA attributes)
                student.teaching_assistant_of = [course]
                course.teaching_assistants.append(student)

    def _assign_research_assistants(
        self, graduate_students: List[GraduateStudent], department: Department
    ) -> None:
        """Assign a portion of graduate students as research assistants."""
        if not graduate_students:
            return

        ra_count = int(
            len(graduate_students)
            * random.uniform(
                self.configuration.research_assistant_ratio_min,
                self.configuration.research_assistant_ratio_max,
            )
        )

        selected_students = random.sample(
            graduate_students, min(ra_count, len(graduate_students))
        )

        for student in selected_students:
            # Create a research group if needed
            research_group = ResearchGroup(
                name=f"{department.name} Research Group", sub_organization_of=department
            )
            self.all_research_groups.append(research_group)
            student.works_for = research_group

    def _generate_publications_for_faculty(
        self,
        faculty_list: List[Faculty],
        graduate_students: List[GraduateStudent],
        min_pubs: int,
        max_pubs: int,
    ) -> None:
        """Generate publications for faculty members with optional graduate student co-authors."""
        for faculty in faculty_list:
            pub_count = random.randint(min_pubs, max_pubs)

            for _ in range(pub_count):
                pub = Publication(
                    name=RandomDataHelper.generate_publication_title(),
                    publication_date=RandomDataHelper.generate_publication_date(),
                    authors=[faculty],
                )

                # Some graduate students co-author with faculty
                if graduate_students and random.random() < 0.3:
                    coauthor = random.choice(graduate_students)
                    if coauthor not in pub.authors:
                        pub.authors.append(coauthor)

                # Track the publication
                self.all_publications.append(pub)


@dataclass
class Dataset:
    organizations: List[Organization]
    publications: List[Publication]
    works: List[Work]
    schedules: List[Schedule]
    persons: List[Person]

    @classmethod
    def from_generator(cls, generator: UniversityDataGenerator):
        """
        Create a dataset from a UniversityDataGenerator.
        """
        organizations = []
        persons = []
        works = []
        schedules = []

        visited_persons = set()

        # Collect all organizations from the generator's tracking lists
        organizations.extend(generator.all_universities)
        organizations.extend(generator.all_departments)
        organizations.extend(generator.all_research_groups)

        # Collect all persons from all organizations
        for org in tqdm.tqdm(organizations):
            for member in tqdm.tqdm(org.members):
                if id(member) not in visited_persons:
                    visited_persons.add(id(member))
                    persons.append(member)

        # Collect all works from the generator
        works.extend(generator.all_courses)
        works.extend(generator.all_graduate_courses)

        # Collect all publications from the generator
        publications = list(generator.all_publications)

        # Schedules are empty as the generator doesn't create them

        return cls(
            organizations=organizations,
            publications=publications,
            works=works,
            schedules=schedules,
            persons=persons,
        )
