import pytest
import rclpy
import time

from pycram.external_interfaces.nlp_interface import (
    FilterOptions,
    NlpInterface,
    NlpNode,
)

# ROS2 Test Setup
@pytest.fixture(scope="session", autouse=True)
def ros_context():
    """
    Initialize and shutdown ROS2 exactly once for the whole test session.
    """
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def nlp_node():
    """
    Provides a real ROS2 NlpNode and cleans it up after the test.
    """
    node = NlpNode()
    yield node
    node.destroy_node()


# Dummy NLP Interface for testing abstract base class
class DummyNlpFilter(NlpInterface):
    def filter_response(self, response, filter_for):
        return super().filter_response(response, filter_for)


# FilterResponse Tests
@pytest.fixture
def sample_response():
    return [
        "Bring me the water.",
        "deliver",
        [
            ["Drink", "water", "Drink", [], [], []],
            ["Person", "me", "NaturalPerson", [], [], []],
        ]
    ]


def test_filter_sentence(sample_response):
    nlp = DummyNlpFilter()
    assert nlp.filter_response(sample_response, FilterOptions.SENTENCE) == "Bring me the water."


def test_filter_intent(sample_response):
    nlp = DummyNlpFilter()
    assert nlp.filter_response(sample_response, FilterOptions.INTENT) == "deliver"


def test_filter_entity_found(sample_response):
    nlp = DummyNlpFilter()
    assert nlp.filter_response(sample_response, FilterOptions.DRINK) == "water"


def test_filter_entity_not_found(sample_response):
    nlp = DummyNlpFilter()
    assert nlp.filter_response(sample_response, FilterOptions.ROOM) is None


def test_filter_none_response():
    nlp = DummyNlpFilter()
    assert nlp.filter_response(None, FilterOptions.INTENT) is None


def test_filter_none_filter(sample_response):
    nlp = DummyNlpFilter()
    assert nlp.filter_response(sample_response, None) == sample_response


# JSON Parsing Tests
def test_parse_json_string_valid(nlp_node):
    json_string = """
    {
        "sentence": "Bring water",
        "intent": "deliver",
        "entities": [
            {
                "role": "Drink",
                "value": "water",
                "entity": "Drink",
                "propertyAttribute": [],
                "actionAttribute": [],
                "numberAttribute": []
            }
        ]
    }
    """

    nlp_node.parse_json_string(json_string)

    assert nlp_node.response is not None
    assert nlp_node.response[0] == "Bring water"
    assert nlp_node.response[1] == "deliver"
    assert nlp_node.response[2][0][0] == "Drink"
    assert nlp_node.response[2][0][1] == "water"


def test_parse_json_string_invalid(nlp_node):
    nlp_node.parse_json_string("{this is not json")
    assert nlp_node.response is None


def test_parse_json_string_empty(nlp_node):
    nlp_node.parse_json_string("")
    assert nlp_node.response is None



# Confirmation Logic Tests
def test_confirm_last_response_affirm(monkeypatch):
    nlp = DummyNlpFilter()
    nlp.last_output = ["go to kitchen", "go", []]

    def fake_talk_nlp(self, timeout):
        return ["yes", "affirm", []]

    monkeypatch.setattr(NlpNode, "talk_nlp", fake_talk_nlp)

    assert nlp.confirm_last_response() is True


def test_confirm_last_response_deny(monkeypatch):
    nlp = DummyNlpFilter()
    nlp.last_output = ["go to kitchen", "go", []]

    def fake_talk_nlp(self, timeout):
        return ["no", "deny", []]

    monkeypatch.setattr(NlpNode, "talk_nlp", fake_talk_nlp)

    assert nlp.confirm_last_response() is False


def test_confirm_last_response_no_previous():
    nlp = DummyNlpFilter()
    nlp.last_output = None
    assert nlp.confirm_last_response() is False


# talk_nlp() Behaviour Tests (no real NLP pipeline)
def test_talk_nlp_timeout(nlp_node):
    """
    No message arrives → should timeout and return None.
    """
    start = time.time()
    result = nlp_node.talk_nlp(timeout=0.2)
    duration = time.time() - start

    assert result is None
    assert duration >= 0.2


def test_talk_nlp_returns_existing_response(nlp_node):
    """
    If response already exists, talk_nlp should return it immediately.
    """
    nlp_node.response = ["Test sentence", "test_intent", []]

    result = nlp_node.talk_nlp(timeout=1)

    assert result[0] == "Test sentence"
    assert nlp_node.response is None  # response must be reset
