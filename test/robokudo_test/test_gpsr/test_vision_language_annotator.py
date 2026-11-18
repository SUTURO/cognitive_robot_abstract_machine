"""
Tests for asking a vision language model about every detected person.
"""

from __future__ import annotations

# %% Imports

import pytest
from py_trees.common import Status
from robokudo.annotators.gpsr import vision_language_annotator
from robokudo.annotators.gpsr.vision_language_annotator import (
    GpuLayers,
    VisionLanguageAnnotator,
)
from robokudo.cas import CASViews

from ._vision_language_mock import DownloadRecorder, ModelLoader, ProjectorLoader
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from ._gpsr_scene import GpsrScene
    from PIL import Image
    from pathlib import Path

# %% model stand-in


@pytest.fixture(autouse=True)
def hub_downloads(monkeypatch: pytest.MonkeyPatch) -> DownloadRecorder:
    """
    Record the downloads from Hugging Face in every test instead of downloading.

    :param monkeypatch: Replaces the download.
    :return: The recorder of the downloads.
    """
    recorder = DownloadRecorder()
    monkeypatch.setattr(vision_language_annotator, "hf_hub_download", recorder)
    return recorder


@pytest.fixture
def model_loader(monkeypatch: pytest.MonkeyPatch) -> ModelLoader:
    """
    Make the annotator load the recording model instead of the model files.

    :param monkeypatch: Replaces the model loader.
    :return: The loader.
    """
    loader = ModelLoader()
    monkeypatch.setattr(vision_language_annotator, "Llama", loader)
    monkeypatch.setattr(
        vision_language_annotator, "Qwen25VLChatHandler", ProjectorLoader()
    )
    return loader


def asking_annotator() -> VisionLanguageAnnotator:
    """
    :return: A vision language annotator with its own default parameters.
    """
    return VisionLanguageAnnotator(descriptor=VisionLanguageAnnotator.Descriptor())


# %% answers about persons


@pytest.mark.parametrize(
    "blue_green_red, red_green_blue",
    [
        ((0, 0, 255), (255, 0, 0)),
        ((255, 0, 0), (0, 0, 255)),
        ((0, 255, 0), (0, 255, 0)),
    ],
    ids=["red", "blue", "green"],
)
def test_person_image_preserves_camera_colors(
    scene: GpsrScene,
    monkeypatch: pytest.MonkeyPatch,
    blue_green_red: tuple[int, int, int],
    red_green_blue: tuple[int, int, int],
) -> None:
    """
    Person image preserves camera colors.
    """
    annotator = asking_annotator()
    cas = scene.place_in_pipeline(annotator)
    image = scene.color_image()
    image[:] = blue_green_red
    cas.set(CASViews.COLOR_IMAGE, image)
    scene.add_human()
    received_pixels: list[int | tuple[int, ...]] = []

    def record_pixel(image: Image.Image) -> str:
        """
        Stand in for the model and remember the first pixel of the image it receives.

        :param image: The image of the model.
        :return: The answer of the stand-in.
        """
        received_pixels.append(image.getpixel((0, 0)))
        return "person"

    monkeypatch.setattr(annotator, "ask", record_pixel)
    annotator.compute()

    assert received_pixels == [red_green_blue]


def test_answer_of_the_model_is_annotated_on_the_person(
    scene: GpsrScene, model_loader: ModelLoader
) -> None:
    """
    Answer of the model is annotated on the person.
    """
    annotator = asking_annotator()
    scene.place_in_pipeline(annotator)
    human_hypothesis = scene.add_human()

    status = annotator.compute()

    assert status == Status.SUCCESS
    assert scene.activities_of(human_hypothesis, VisionLanguageAnnotator) == [
        model_loader.loaded.answer
    ]


def test_answer_is_stripped_of_surrounding_spaces(
    scene: GpsrScene, model_loader: ModelLoader
) -> None:
    """
    Answer is stripped of surrounding spaces.
    """
    model_loader.loaded.answer = "  red coat \n"
    annotator = asking_annotator()
    scene.place_in_pipeline(annotator)
    human_hypothesis = scene.add_human()

    annotator.compute()

    assert scene.activities_of(human_hypothesis, VisionLanguageAnnotator) == [
        "red coat"
    ]


def test_empty_answer_annotates_nothing(
    scene: GpsrScene, model_loader: ModelLoader
) -> None:
    """
    Empty answer annotates nothing.
    """
    model_loader.loaded.answer = " "
    annotator = asking_annotator()
    scene.place_in_pipeline(annotator)
    human_hypothesis = scene.add_human()

    annotator.compute()

    assert scene.activities_of(human_hypothesis, VisionLanguageAnnotator) == []


def test_every_person_is_asked_about(
    scene: GpsrScene, model_loader: ModelLoader
) -> None:
    """
    Every person is asked about.
    """
    annotator = asking_annotator()
    scene.place_in_pipeline(annotator)
    scene.add_human()
    scene.add_human()

    annotator.compute()

    assert len(model_loader.loaded.received_messages) == 2


def test_without_persons_nothing_is_asked(
    scene: GpsrScene, model_loader: ModelLoader
) -> None:
    """
    Without persons nothing is asked.
    """
    annotator = asking_annotator()
    scene.place_in_pipeline(annotator)

    status = annotator.compute()

    assert status == Status.FAILURE
    assert model_loader.loaded.received_messages == []


# %% conversation with the model


def test_model_is_asked_the_configured_question_with_the_system_prompt(
    scene: GpsrScene, model_loader: ModelLoader
) -> None:
    """
    Model is asked the configured question with the system prompt.
    """
    annotator = asking_annotator()
    parameters = annotator.descriptor.parameters
    parameters.question = "What is this person holding?"
    scene.place_in_pipeline(annotator)
    scene.add_human()

    annotator.compute()

    [[system, user]] = model_loader.loaded.received_messages
    assert system == {"role": "system", "content": parameters.system_prompt}
    [image, question] = user["content"]
    assert image["type"] == "image_url"
    assert image["image_url"]["url"].startswith("data:image/jpeg;base64,")
    assert question == {"type": "text", "text": parameters.question}


def test_model_answers_with_the_configured_temperature(
    scene: GpsrScene, model_loader: ModelLoader
) -> None:
    """
    Model answers with the configured temperature.
    """
    annotator = asking_annotator()
    scene.place_in_pipeline(annotator)
    scene.add_human()

    annotator.compute()

    assert model_loader.loaded.received_temperatures == [
        annotator.descriptor.parameters.temperature
    ]


# %% loading the model


def test_model_is_loaded_once(scene: GpsrScene, model_loader: ModelLoader) -> None:
    """
    Model is loaded once.
    """
    annotator = asking_annotator()
    scene.place_in_pipeline(annotator)
    scene.add_human()

    annotator.compute()
    annotator.compute()

    assert len(model_loader.load_arguments) == 1


def test_model_is_loaded_from_the_models_directory(
    scene: GpsrScene, model_loader: ModelLoader
) -> None:
    """
    Model is loaded from the models directory.
    """
    annotator = asking_annotator()
    parameters = annotator.descriptor.parameters
    scene.place_in_pipeline(annotator)
    scene.add_human()

    annotator.compute()

    [arguments] = model_loader.load_arguments
    directory = parameters.models_directory
    assert arguments["model_path"] == str(directory / parameters.model_file)
    assert arguments["chat_handler"] == str(directory / parameters.projector_file)
    assert arguments["n_ctx"] == parameters.context_length


def test_missing_model_files_are_downloaded_into_the_models_directory(
    hub_downloads: DownloadRecorder, tmp_path: Path
) -> None:
    """
    Before the analysis engine runs, the model files the models directory lacks are
    fetched from the Hugging Face repository.
    """
    annotator = asking_annotator()
    parameters = annotator.descriptor.parameters
    parameters.models_directory = tmp_path

    annotator.setup()

    assert hub_downloads.downloads == [
        {
            "repo_id": parameters.huggingface_repository,
            "filename": file_name,
            "local_dir": tmp_path,
        }
        for file_name in (parameters.model_file, parameters.projector_file)
    ]


def test_model_files_in_the_models_directory_are_not_downloaded_again(
    hub_downloads: DownloadRecorder, tmp_path: Path
) -> None:
    """
    Once its files are downloaded, the model starts without the network.
    """
    annotator = asking_annotator()
    parameters = annotator.descriptor.parameters
    parameters.models_directory = tmp_path
    for file_name in (parameters.model_file, parameters.projector_file):
        (tmp_path / file_name).touch()

    annotator.setup()

    assert hub_downloads.downloads == []


@pytest.mark.parametrize(
    "use_gpu, gpu_layers",
    [(True, GpuLayers.ALL), (False, GpuLayers.NONE)],
    ids=["gpu", "cpu"],
)
def test_gpu_setting_decides_the_layers_on_the_gpu(
    scene: GpsrScene, model_loader: ModelLoader, use_gpu: bool, gpu_layers: GpuLayers
) -> None:
    """
    The GPU setting decides how many layers run on the GPU.
    """
    annotator = asking_annotator()
    annotator.descriptor.parameters.use_gpu = use_gpu
    scene.place_in_pipeline(annotator)
    scene.add_human()

    annotator.compute()

    [arguments] = model_loader.load_arguments
    assert arguments["n_gpu_layers"] == gpu_layers
