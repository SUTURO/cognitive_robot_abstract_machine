from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, List
from enum import Enum


class DepartmentName(Enum):
    """Enumeration of available department names."""
    COMPUTER_SCIENCE = "Computer Science"
    MATHEMATICS = "Mathematics"
    PHYSICS = "Physics"
    CHEMISTRY = "Chemistry"
    BIOLOGY = "Biology"
    ELECTRICAL_ENGINEERING = "Electrical Engineering"
    MECHANICAL_ENGINEERING = "Mechanical Engineering"
    CIVIL_ENGINEERING = "Civil Engineering"
    ECONOMICS = "Economics"
    PSYCHOLOGY = "Psychology"
    HISTORY = "History"
    PHILOSOPHY = "Philosophy"
    ENGLISH_LITERATURE = "English Literature"
    BUSINESS_ADMINISTRATION = "Business Administration"
    POLITICAL_SCIENCE = "Political Science"
    SOCIOLOGY = "Sociology"
    ANTHROPOLOGY = "Anthropology"
    STATISTICS = "Statistics"
    MATERIALS_SCIENCE = "Materials Science"
    ENVIRONMENTAL_SCIENCE = "Environmental Science"
    AEROSPACE_ENGINEERING = "Aerospace Engineering"
    CHEMICAL_ENGINEERING = "Chemical Engineering"
    BIOMEDICAL_ENGINEERING = "Biomedical Engineering"
    INFORMATION_SYSTEMS = "Information Systems"


class ResearchArea(Enum):
    """Enumeration of research areas."""
    ARTIFICIAL_INTELLIGENCE = "Artificial Intelligence"
    MACHINE_LEARNING = "Machine Learning"
    DATA_MINING = "Data Mining"
    COMPUTER_VISION = "Computer Vision"
    NATURAL_LANGUAGE_PROCESSING = "Natural Language Processing"
    ROBOTICS = "Robotics"
    COMPUTER_NETWORKS = "Computer Networks"
    CYBERSECURITY = "Cybersecurity"
    QUANTUM_COMPUTING = "Quantum Computing"
    BIOINFORMATICS = "Bioinformatics"
    COMPUTATIONAL_BIOLOGY = "Computational Biology"
    OPTIMIZATION = "Optimization"
    ALGORITHM_DESIGN = "Algorithm Design"
    DATABASE_SYSTEMS = "Database Systems"
    SOFTWARE_ENGINEERING = "Software Engineering"
    HUMAN_COMPUTER_INTERACTION = "Human-Computer Interaction"


class CoursePrefix(Enum):
    """Enumeration of course prefixes."""
    INTRODUCTION_TO = "Introduction to"
    ADVANCED = "Advanced"
    FUNDAMENTALS_OF = "Fundamentals of"
    TOPICS_IN = "Topics in"
    THEORY_OF = "Theory of"
    APPLIED = "Applied"
    COMPUTATIONAL = "Computational"
    EXPERIMENTAL = "Experimental"


class CourseSubject(Enum):
    """Enumeration of course subjects."""
    ALGORITHMS = "Algorithms"
    DATA_STRUCTURES = "Data Structures"
    PROGRAMMING = "Programming"
    SOFTWARE_DESIGN = "Software Design"
    COMPUTER_ARCHITECTURE = "Computer Architecture"
    OPERATING_SYSTEMS = "Operating Systems"
    DISTRIBUTED_SYSTEMS = "Distributed Systems"
    MACHINE_LEARNING = "Machine Learning"
    ARTIFICIAL_INTELLIGENCE = "Artificial Intelligence"
    COMPUTER_GRAPHICS = "Computer Graphics"
    DATABASE_MANAGEMENT = "Database Management"
    WEB_DEVELOPMENT = "Web Development"
    MOBILE_COMPUTING = "Mobile Computing"
    NETWORK_SECURITY = "Network Security"
    CRYPTOGRAPHY = "Cryptography"
    COMPILERS = "Compilers"
    ANALYSIS = "Analysis"


class PublicationTitlePrefix(Enum):
    """Enumeration of publication title prefixes."""
    A_NOVEL_APPROACH_TO = "A Novel Approach to"
    ON_THE_THEORY_OF = "On the Theory of"
    ADVANCES_IN = "Advances in"
    AN_EMPIRICAL_STUDY_OF = "An Empirical Study of"
    TOWARDS = "Towards"
    UNDERSTANDING = "Understanding"


@dataclass
class Person:
    """Represents a person in the university benchmark ontology.
    
    Base class for all people including students, faculty, and staff members.
    """

    name: Optional[str] = None
    email_address: Optional[str] = None
    telephone: Optional[str] = None
    age: Optional[int] = None
    title: Optional[str] = None
    advisor: Optional[Professor] = None
    degree_from: List[University] = field(default_factory=list)
    undergraduate_degree_from: List[University] = field(default_factory=list)
    masters_degree_from: List[University] = field(default_factory=list)
    doctoral_degree_from: List[University] = field(default_factory=list)
    member_of: List[Organization] = field(default_factory=list)
    affiliate_of: List[Organization] = field(default_factory=list)


@dataclass
class Organization:
    """Represents an organization in the university benchmark ontology.
    
    Base class for universities, departments, research groups, and other institutions.
    """

    name: Optional[str] = None
    sub_organization_of: Optional[Organization] = None
    affiliated_organization_of: List[Organization] = field(default_factory=list)
    members: List[Person] = field(default_factory=list)
    affiliates: List[Person] = field(default_factory=list)
    publications: List[Publication] = field(default_factory=list)


@dataclass
class University(Organization):
    """Represents a university.
    
    A university is a higher education institution that contains departments,
    colleges, and other sub-organizations.
    
    :ivar alumni: List of people who are alumni of this university
    :vartype alumni: List[Person]
    """

    alumni: List[Person] = field(default_factory=list)


@dataclass
class College(Organization):
    """Represents a school or college within a university.
    
    A college is an organizational unit within a university that typically
    groups related academic departments together.
    """

    pass


@dataclass
class Department(Organization):
    """Represents a university department.
    
    A department is an academic unit within a university or college that
    focuses on a specific field of study and employs faculty members.
    """

    pass


@dataclass
class Institute(Organization):
    """Represents an institute.
    
    An institute is a research or educational organization that may be
    affiliated with or part of a university.
    """

    pass


@dataclass
class Program(Organization):
    """Represents a program.
    
    A program is an organized course of study leading to a degree or
    certificate within an academic institution.
    """

    pass


@dataclass
class ResearchGroup(Organization):
    """Represents a research group.
    
    A research group is a team of researchers working together on specific
    research topics or projects within a department or institution.
    
    :ivar research_projects: List of research projects this group is working on
    :vartype research_projects: List[Research]
    """

    research_projects: List[Research] = field(default_factory=list)


@dataclass
class Employee(Person):
    """Represents an employee who works for an organization.
    
    An employee is a person who has a formal employment relationship with
    an organization such as a university or department.
    
    :ivar works_for: The organization this person is employed by
    :vartype works_for: Optional[Organization]
    """

    works_for: Optional[Organization] = None


@dataclass
class AdministrativeStaff(Employee):
    """Represents an administrative staff worker.
    
    Administrative staff members provide support services and management
    functions within an organization.
    """

    pass


@dataclass
class ClericalStaff(AdministrativeStaff):
    """Represents a clerical staff worker.
    
    Clerical staff members handle office duties such as filing, typing,
    and general administrative tasks.
    """

    pass


@dataclass
class SystemsStaff(AdministrativeStaff):
    """Represents a systems staff worker.
    
    Systems staff members manage and maintain technical infrastructure
    such as computers, networks, and information systems.
    """

    pass


@dataclass
class Faculty(Employee):
    """Represents a faculty member.
    
    Faculty members are academic staff who teach courses and conduct research
    at a university or college.
    
    :ivar office_number: The office number or location of the faculty member
    :vartype office_number: Optional[str]
    :ivar research_interest: The primary research area of interest for this faculty member
    :vartype research_interest: Optional[ResearchArea]
    :ivar teaches: List of courses taught by this faculty member
    :vartype teaches: List[Course]
    """

    office_number: Optional[str] = None
    research_interest: Optional[ResearchArea] = None
    teaches: List[Course] = field(default_factory=list)


@dataclass
class Lecturer(Faculty):
    """Represents a lecturer.
    
    A lecturer is a faculty member focused primarily on teaching rather than
    research, typically without tenure-track status.
    """

    pass


@dataclass
class PostDoc(Faculty):
    """Represents a post doctorate.
    
    A postdoctoral researcher is a temporary research position for individuals
    who have recently completed their doctoral degree.
    """

    pass


@dataclass
class Professor(Faculty):
    """Represents a professor.
    
    A professor is a senior faculty member who typically holds tenure or is
    on a tenure-track and engages in both teaching and research.
    
    :ivar tenured: Whether the professor has tenure
    :vartype tenured: Optional[bool]
    """

    tenured: Optional[bool] = None


@dataclass
class AssistantProfessor(Professor):
    """Represents an assistant professor.
    
    An assistant professor is an entry-level tenure-track faculty position,
    typically held by faculty members in the early stages of their academic career.
    """

    pass


@dataclass
class AssociateProfessor(Professor):
    """Represents an associate professor.
    
    An associate professor is a mid-level tenured faculty position, typically
    achieved after demonstrating excellence in teaching, research, and service.
    """

    pass


@dataclass
class FullProfessor(Professor):
    """Represents a full professor.
    
    A full professor is the highest academic rank for faculty members, achieved
    through sustained excellence in research, teaching, and service.
    """

    pass


@dataclass
class VisitingProfessor(Professor):
    """Represents a visiting professor.
    
    A visiting professor is a temporary faculty position for scholars from other
    institutions who teach or conduct research for a limited period.
    """

    pass


@dataclass
class Chair(Professor):
    """Represents a chair who heads a department.
    
    A chair is a professor who serves as the administrative head of an
    academic department.
    
    :ivar head_of: The department this chair is heading
    :vartype head_of: Optional[Department]
    """

    head_of: Optional[Department] = None


@dataclass
class Dean(Professor):
    """Represents a dean who heads a college.
    
    A dean is a senior administrator who oversees an entire college or
    school within a university.
    
    :ivar head_of: The college this dean is heading
    :vartype head_of: Optional[College]
    """

    head_of: Optional[College] = None


@dataclass
class Director(Person):
    """Represents a director who heads a program.
    
    A director is an administrator who manages and oversees an academic
    or research program.
    
    :ivar head_of: The program this director is heading
    :vartype head_of: Optional[Program]
    """

    head_of: Optional[Program] = None


@dataclass
class Student(Person):
    """Represents a student who takes courses.
    
    A student is a person enrolled in an educational institution to pursue
    a degree or certificate program.
    
    :ivar takes_courses: List of courses the student is enrolled in
    :vartype takes_courses: List[Course]
    """

    takes_courses: List[Course] = field(default_factory=list)


@dataclass
class UndergraduateStudent(Student):
    """Represents an undergraduate student.
    
    An undergraduate student is pursuing a bachelor's degree or lower-level
    academic credential.
    """

    pass


@dataclass
class GraduateStudent(Person):
    """Represents a graduate student.
    
    A graduate student is pursuing an advanced degree such as a master's
    or doctoral degree.
    
    :ivar takes_courses: List of graduate courses the student is enrolled in
    :vartype takes_courses: List[GraduateCourse]
    """

    takes_courses: List[GraduateCourse] = field(default_factory=list)


@dataclass
class ResearchAssistant(Person):
    """Represents a university research assistant.
    
    A research assistant is typically a graduate student or postdoc who assists
    with research projects in exchange for funding or academic credit.
    
    :ivar works_for: The research group this assistant works for
    :vartype works_for: Optional[ResearchGroup]
    """

    works_for: Optional[ResearchGroup] = None


@dataclass
class TeachingAssistant(Person):
    """Represents a university teaching assistant.
    
    A teaching assistant is typically a graduate student who helps faculty
    members with teaching duties such as grading and leading discussion sections.
    
    :ivar teaching_assistant_of: List of courses this person is a teaching assistant for
    :vartype teaching_assistant_of: List[Course]
    """

    teaching_assistant_of: List[Course] = field(default_factory=list)


@dataclass
class Work:
    """Represents work in the university benchmark ontology.
    
    Work is the base class for academic activities including courses and research.
    
    :ivar name: The name or title of the work
    :vartype name: Optional[str]
    """

    name: Optional[str] = None


@dataclass
class Course(Work):
    """Represents a teaching course.
    
    A course is an organized unit of instruction typically taught by faculty
    members and taken by students for academic credit.
    
    :ivar teacher: The faculty member teaching this course
    :vartype teacher: Optional[Faculty]
    :ivar teaching_assistants: List of teaching assistants helping with this course
    :vartype teaching_assistants: List[TeachingAssistant]
    """

    teacher: Optional[Faculty] = None
    teaching_assistants: List[TeachingAssistant] = field(default_factory=list)


@dataclass
class GraduateCourse(Course):
    """Represents graduate level courses.
    
    Graduate courses are advanced courses designed for graduate students
    pursuing master's or doctoral degrees.
    """

    pass


@dataclass
class Research(Work):
    """Represents research work.
    
    Research represents scholarly investigation and study activities conducted
    by faculty and students.
    """

    pass


@dataclass
class Schedule:
    """Represents a schedule.
    
    A schedule contains a list of courses offered during a specific term or period.
    
    :ivar listed_courses: List of courses included in this schedule
    :vartype listed_courses: List[Course]
    """

    listed_courses: List[Course] = field(default_factory=list)


@dataclass
class Publication:
    """Represents a publication.
    
    A publication is a scholarly work such as an article, book, or software
    that has been made publicly available.
    
    :ivar name: The title of the publication
    :vartype name: Optional[str]
    :ivar publication_date: The date the publication was published
    :vartype publication_date: Optional[str]
    :ivar authors: List of people who authored this publication
    :vartype authors: List[Person]
    :ivar publication_research: The research work associated with this publication
    :vartype publication_research: Optional[Research]
    """

    name: Optional[str] = None
    publication_date: Optional[str] = None
    authors: List[Person] = field(default_factory=list)
    publication_research: Optional[Research] = None


@dataclass
class Article(Publication):
    """Represents an article.
    
    An article is a written work published in a journal, conference proceedings,
    or as a technical report.
    """

    pass


@dataclass
class ConferencePaper(Article):
    """Represents a conference paper.
    
    A conference paper is an article presented and published in the proceedings
    of an academic conference.
    """

    pass


@dataclass
class JournalArticle(Article):
    """Represents a journal article.
    
    A journal article is a scholarly article published in an academic journal,
    typically peer-reviewed.
    """

    pass


@dataclass
class TechnicalReport(Article):
    """Represents a technical report.
    
    A technical report is a document describing research findings or technical
    work, often published by institutions or research groups.
    """

    pass


@dataclass
class Book(Publication):
    """Represents a book.
    
    A book is a published work of substantial length covering a topic or
    topics in depth.
    """

    pass


@dataclass
class Manual(Publication):
    """Represents a manual.
    
    A manual is an instructional document providing guidance on how to use
    or operate something, such as software or equipment.
    """

    pass


@dataclass
class Software(Publication):
    """Represents a software program.
    
    Software represents a published computer program or application that
    may be used for research or educational purposes.
    
    :ivar software_version: The version number of the software
    :vartype software_version: Optional[str]
    :ivar software_documentation: List of documentation publications for this software
    :vartype software_documentation: List[Publication]
    """

    software_version: Optional[str] = None
    software_documentation: List[Publication] = field(default_factory=list)


@dataclass
class Specification(Publication):
    """Represents a published specification.
    
    A specification is a formal document that describes technical requirements,
    standards, or protocols.
    """

    pass


@dataclass
class UnofficialPublication(Publication):
    """Represents an unofficial publication.
    
    An unofficial publication is a work that has not been formally published
    through traditional channels, such as preprints or drafts.
    """

    pass
