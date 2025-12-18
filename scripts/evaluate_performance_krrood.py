from ormatic.dao import to_dao
from sqlalchemy import create_engine, select
from sqlalchemy.orm import Session

from entity_query_language import an, entity, let, contains
from krrood.lubm import Student
from krrood.orm.ormatic_interface import Base, PersonDAO
from krrood.generator import UniversityDataGenerator, GeneratorConfiguration, Dataset


def evaluate_eql():
    engine = create_engine("sqlite:///:memory:")
    session = Session(bind=engine)
    Base.metadata.create_all(engine)
    session.commit()

    # Generate data here instead
    config = GeneratorConfiguration(seed=69)
    generator = UniversityDataGenerator(config)
    generator.generate_universities(1)
    dataset = Dataset.from_generator(generator)
    session.add(to_dao(dataset))
    session.commit()
    print("INSERTED")
    q = select(PersonDAO)
    print(session.scalars(q).all())


if __name__ == "__main__":
    evaluate_eql()
