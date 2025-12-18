"""
Auto-generated Python classes from OWL ontology
Generated using custom converter
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Union, Any

from entity_query_language.property_descriptor import Thing, PropertyDescriptor


# Property descriptor classes (object properties)
@dataclass(frozen=True)
class Advisor(PropertyDescriptor):
    """is being advised by"""


@dataclass(frozen=True)
class AffiliateOf(PropertyDescriptor):
    """is affiliated with"""


@dataclass(frozen=True)
class AffiliatedOrganizationOf(PropertyDescriptor):
    """is affiliated with"""


@dataclass(frozen=True)
class DegreeFrom(PropertyDescriptor):
    """has a degree from"""


@dataclass(frozen=True)
class HasAlumnus(PropertyDescriptor):
    """has as an alumnus"""
    # Inverse of degreeFrom
    inverse_of = DegreeFrom


@dataclass(frozen=True)
class ListedCourse(PropertyDescriptor):
    """lists as a course"""


@dataclass(frozen=True)
class Member(PropertyDescriptor):
    """has as a member"""


@dataclass(frozen=True)
class MemberOf(PropertyDescriptor):
    """member of"""
    # Inverse of member
    inverse_of = Member


@dataclass(frozen=True)
class OrgPublication(PropertyDescriptor):
    """publishes"""


@dataclass(frozen=True)
class PublicationAuthor(PropertyDescriptor):
    """was written by"""


@dataclass(frozen=True)
class PublicationResearch(PropertyDescriptor):
    """is about"""


@dataclass(frozen=True)
class ResearchProject(PropertyDescriptor):
    """has as a research project"""


@dataclass(frozen=True)
class SoftwareDocumentation(PropertyDescriptor):
    """is documented in"""


@dataclass(frozen=True)
class SubOrganizationOf(PropertyDescriptor):
    """is part of"""
    transitive = True


@dataclass(frozen=True)
class TakesCourse(PropertyDescriptor):
    """is taking"""


@dataclass(frozen=True)
class TeacherOf(PropertyDescriptor):
    """teaches"""


@dataclass(frozen=True)
class TeachingAssistantOf(PropertyDescriptor):
    """is a teaching assistant for"""


@dataclass(frozen=True)
class DoctoralDegreeFrom(DegreeFrom):
    """has a doctoral degree from"""


@dataclass(frozen=True)
class MastersDegreeFrom(DegreeFrom):
    """has a masters degree from"""


@dataclass(frozen=True)
class TakesCourseCourse(TakesCourse):
    """is taking"""


@dataclass(frozen=True)
class TakesCourseGraduateCourse(TakesCourse):
    """is taking"""


@dataclass(frozen=True)
class TeachingAssistantOfCourse(TeachingAssistantOf):
    """is a teaching assistant for"""


@dataclass(frozen=True)
class UndergraduateDegreeFrom(DegreeFrom):
    """has an undergraduate degree from"""


@dataclass(frozen=True)
class WorksFor(MemberOf):
    """Works For"""


@dataclass(frozen=True)
class HeadOf(WorksFor):
    """is the head of"""


@dataclass(frozen=True)
class WorksForOrganization(WorksFor):
    """Works For"""


@dataclass(frozen=True)
class WorksForResearchGroup(WorksFor):
    """Works For"""


@dataclass(frozen=True)
class HeadOfCollege(HeadOf):
    """is the head of"""


@dataclass(frozen=True)
class HeadOfDepartment(HeadOf):
    """is the head of"""


@dataclass(frozen=True)
class HeadOfProgram(HeadOf):
    """is the head of"""



# Generated classes
@dataclass(eq=False)
class UnivBenchOntology(Thing):
    """Base class for Univ-bench Ontology"""
    # name
    name: Optional[str] = None
    # office room No.
    office_number: Optional[int] = None
    # is researching
    research_interest: Optional[str] = None


@dataclass(eq=False)
class Organization(UnivBenchOntology):
    """organization"""
    # is affiliated with
    affiliated_organization_of: List[Organization] = AffiliatedOrganizationOf(default_factory=list)
    # is affiliated with
    affiliate_of: List[Person] = AffiliateOf(default_factory=list)
    # has as a member
    member: List[Person] = Member(default_factory=list)
    # publishes
    org_publication: List[Publication] = OrgPublication(default_factory=list)
    # is part of
    sub_organization_of: List[Organization] = SubOrganizationOf(default_factory=list)


@dataclass(eq=False)
class Person(UnivBenchOntology):
    """person"""
    # is being advised by
    advisor: List[Professor] = Advisor(default_factory=list)
    # has a degree from
    degree_from: List[University] = DegreeFrom(default_factory=list)
    # has a doctoral degree from
    doctoral_degree_from: List[University] = DoctoralDegreeFrom(default_factory=list)
    # has a masters degree from
    masters_degree_from: List[University] = MastersDegreeFrom(default_factory=list)
    # has an undergraduate degree from
    undergraduate_degree_from: List[University] = UndergraduateDegreeFrom(default_factory=list)
    # is age
    age: Optional[int] = None
    # can be reached at
    email_address: Optional[str] = None
    # telephone number
    telephone: Optional[str] = None
    # title
    title: Optional[str] = None


@dataclass(eq=False)
class Publication(UnivBenchOntology):
    """publication"""
    # was written by
    publication_author: List[Person] = PublicationAuthor(default_factory=list)
    # was written on
    publication_date: Optional[str] = None
    # is about
    publication_research: List[Research] = PublicationResearch(default_factory=list)


@dataclass(eq=False)
class Schedule(UnivBenchOntology):
    """schedule"""
    # lists as a course
    listed_course: List[Course] = ListedCourse(default_factory=list)


@dataclass(eq=False)
class Work(UnivBenchOntology):
    """Work"""
    ...


@dataclass(eq=False)
class Article(Publication):
    """article"""
    ...


@dataclass(eq=False)
class Book(Publication):
    """book"""
    ...


@dataclass(eq=False)
class College(Organization):
    """school"""
    ...


@dataclass(eq=False)
class Course(Work):
    """teaching course"""
    ...


@dataclass(eq=False)
class Department(Organization):
    """university department"""
    ...


@dataclass(eq=False)
class Director(Person):
    """director"""
    # is the head of
    head_of: List[Program] = HeadOfProgram(default_factory=list)


@dataclass(eq=False)
class Employee(Person):
    """Employee"""
    # is the head of
    head_of: List[Organization] = HeadOf(default_factory=list)
    # Works For
    works_for: List[Organization] = WorksForOrganization(default_factory=list)


@dataclass(eq=False)
class GraduateStudent(Person):
    """graduate student"""
    # is taking
    takes_course: List[GraduateCourse] = TakesCourseGraduateCourse(default_factory=list)


@dataclass(eq=False)
class Institute(Organization):
    """institute"""
    ...


@dataclass(eq=False)
class Manual(Publication):
    """manual"""
    ...


@dataclass(eq=False)
class Program(Organization):
    """program"""
    ...


@dataclass(eq=False)
class Research(Work):
    """research work"""
    ...


@dataclass(eq=False)
class ResearchAssistant(Person):
    """university research assistant"""
    # Works For
    works_for: List[ResearchGroup] = WorksForResearchGroup(default_factory=list)


@dataclass(eq=False)
class ResearchGroup(Organization):
    """research group"""
    # has as a research project
    research_project: List[Research] = ResearchProject(default_factory=list)


@dataclass(eq=False)
class Software(Publication):
    """software program"""
    # is documented in
    software_documentation: List[Publication] = SoftwareDocumentation(default_factory=list)
    # is version
    software_version: Optional[str] = None


@dataclass(eq=False)
class Specification(Publication):
    """published specification"""
    ...


@dataclass(eq=False)
class Student(Person):
    """student"""
    # is taking
    takes_course: List[Course] = TakesCourseCourse(default_factory=list)


@dataclass(eq=False)
class TeachingAssistant(Person):
    """university teaching assistant"""
    # is a teaching assistant for
    teaching_assistant_of: List[Course] = TeachingAssistantOfCourse(default_factory=list)


@dataclass(eq=False)
class University(Organization):
    """university"""
    # has as an alumnus
    has_alumnus: List[Person] = HasAlumnus(default_factory=list)


@dataclass(eq=False)
class UnofficialPublication(Publication):
    """unnoficial publication"""
    ...


@dataclass(eq=False)
class AdministrativeStaff(Employee):
    """administrative staff worker"""
    ...


@dataclass(eq=False)
class ConferencePaper(Article):
    """conference paper"""
    ...


@dataclass(eq=False)
class Faculty(Employee):
    """faculty member"""
    # teaches
    teacher_of: List[Course] = TeacherOf(default_factory=list)


@dataclass(eq=False)
class GraduateCourse(Course):
    """Graduate Level Courses"""
    ...


@dataclass(eq=False)
class JournalArticle(Article):
    """journal article"""
    ...


@dataclass(eq=False)
class TechnicalReport(Article):
    """technical report"""
    ...


@dataclass(eq=False)
class UndergraduateStudent(Student):
    """undergraduate student"""
    ...


@dataclass(eq=False)
class ClericalStaff(AdministrativeStaff):
    """clerical staff worker"""
    ...


@dataclass(eq=False)
class Lecturer(Faculty):
    """lecturer"""
    ...


@dataclass(eq=False)
class PostDoc(Faculty):
    """post doctorate"""
    ...


@dataclass(eq=False)
class Professor(Faculty):
    """professor"""
    # is tenured:
    tenured: Optional[bool] = None


@dataclass(eq=False)
class SystemsStaff(AdministrativeStaff):
    """systems staff worker"""
    ...


@dataclass(eq=False)
class AssistantProfessor(Professor):
    """assistant professor"""
    ...


@dataclass(eq=False)
class AssociateProfessor(Professor):
    """associate professor"""
    ...


@dataclass(eq=False)
class Chair(Professor):
    """chair"""
    # is the head of
    head_of: List[Department] = HeadOfDepartment(default_factory=list)


@dataclass(eq=False)
class Dean(Professor):
    """dean"""
    # is the head of
    head_of: List[College] = HeadOfCollege(default_factory=list)


@dataclass(eq=False)
class FullProfessor(Professor):
    """full professor"""
    ...


@dataclass(eq=False)
class VisitingProfessor(Professor):
    """visiting professor"""
    ...


