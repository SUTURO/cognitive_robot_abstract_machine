#!/usr/bin/env python3
"""Simple test to verify the LUBM class hierarchy implementation."""

from krrood.lubm import (
    Person,
    Organization,
    University,
    Department,
    College,
    Professor,
    AssistantProfessor,
    FullProfessor,
    Faculty,
    Student,
    UndergraduateStudent,
    GraduateStudent,
    Course,
    GraduateCourse,
    Publication,
    Article,
    Book,
    ResearchGroup,
    Research,
)


def test_basic_instantiation():
    """Test that basic classes can be instantiated."""
    person = Person(name="John Doe", age=30, email_address="john@example.com")
    assert person.name == "John Doe"
    assert person.age == 30
    assert person.email_address == "john@example.com"


def test_organization_hierarchy():
    """Test organization hierarchy."""
    university = University(name="MIT")
    department = Department(name="Computer Science", sub_organization_of=university)
    assert isinstance(university, Organization)
    assert isinstance(department, Organization)
    assert department.sub_organization_of == university


def test_person_hierarchy():
    """Test person hierarchy and inheritance."""
    professor = AssistantProfessor(
        name="Dr. Smith", email_address="smith@university.edu", tenured=False
    )
    assert isinstance(professor, AssistantProfessor)
    assert isinstance(professor, Professor)
    assert isinstance(professor, Faculty)
    assert isinstance(professor, Person)
    assert professor.tenured is False


def test_student_types():
    """Test student types."""
    undergrad = UndergraduateStudent(name="Alice")
    grad = GraduateStudent(name="Bob")

    assert isinstance(undergrad, Student)
    assert isinstance(undergrad, Person)
    assert isinstance(grad, Person)


def test_course_and_teaching():
    """Test course and teaching relationships."""
    professor = FullProfessor(name="Dr. Johnson")
    course = GraduateCourse(name="Advanced Algorithms", teacher=professor)

    assert isinstance(course, Course)
    assert course.teacher == professor


def test_publication_hierarchy():
    """Test publication hierarchy."""
    author = Person(name="Jane Doe")
    article = Article(name="My Research Paper", authors=[author])
    book = Book(name="My Book", authors=[author])

    assert isinstance(article, Publication)
    assert isinstance(book, Publication)
    assert author in article.authors


def test_research_group():
    """Test research group and relationships."""
    research = Research(name="AI Research Project")
    group = ResearchGroup(name="AI Lab", research_projects=[research])

    assert research in group.research_projects


def test_degree_relationships():
    """Test degree relationships."""
    university = University(name="Stanford")
    person = Person(
        name="John Smith",
        undergraduate_degree_from=[university],
        doctoral_degree_from=[university],
    )

    assert university in person.undergraduate_degree_from
    assert university in person.doctoral_degree_from
