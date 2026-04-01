from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
from urllib.parse import urlparse
from zipfile import ZipFile
import json
import requests

from semantic_digital_twin.adapters.sage_10k_dataset.schema import Sage10kScene


@dataclass
class Sage10kDatasetLoader:
    """
    Loader for scenes from the Sage10k dataset.

    """

    scene_url: str
    """
    The URL where the scene is located.
    """

    directory: Path = field(default_factory=lambda: Path.home() / "sage-10k-scenes")
    """
    The directory where the scene should be downloaded to.
    """

    def __post_init__(self):
        if not self.directory.exists():
            ...

    def _download_scene(self) -> Path:
        """
        Download the scene from the Sage10k dataset and return it as a Path.
        :return: The path to the scene downloaded.
        """
        self.directory.mkdir(parents=True, exist_ok=True)

        filename = Path(urlparse(self.scene_url).path).name
        target_path = self.directory / filename

        # check if the target file already exists
        if target_path.exists():
            return target_path

        with requests.get(self.scene_url, stream=True, timeout=60) as response:
            response.raise_for_status()
            with target_path.open("wb") as f:
                for chunk in response.iter_content(chunk_size=8192):
                    if chunk:
                        f.write(chunk)

        return target_path

    def _unzip_scene(self, scene_path: Path) -> Path:
        """
        Extracts the contents of a zip file to a directory. If the directory already exists,
        the function will return the existing directory path without performing any operations.

        :param scene_path: Path to the zip file that needs to be extracted.
        :return: The path to the directory where the contents of the zip file have been extracted.
        """
        extract_dir = self.directory / scene_path.stem

        if extract_dir.exists():
            return extract_dir

        extract_dir.mkdir(parents=True, exist_ok=True)

        with ZipFile(scene_path, "r") as zip_ref:
            zip_ref.extractall(extract_dir)

        return extract_dir

    def _parse_json(self, extracted_dir: Path) -> Sage10kScene:
        """
        Parses the extracted directory to locate and load a specific JSON file, ensuring there is
        exactly one valid file matching the naming pattern. Load the JSON into a Sage10kScene object.

        :param extracted_dir: The directory containing the extracted files to be parsed.
        :return: A Sage10kScene object created from the parsed JSON content. The object's
            `directory_path` attribute is also updated to the given `extracted_dir`.
        """
        json_files = list(extracted_dir.glob("layout_*.json"))
        if not json_files:
            raise ValueError(f"JSON file not found in {extracted_dir}")
        elif len(json_files) > 1:
            raise ValueError(f"Multiple JSON files found in {extracted_dir}")
        json_file = json_files[0]

        raw_json = json_file.read_text()
        json_dict = json.loads(raw_json)
        result = Sage10kScene._from_json(json_dict)
        result.directory_path = extracted_dir
        return result

    def create_scene(self) -> Sage10kScene:
        """
        Create a scene from the given URL by downloading it and loading it into the memory.

        :return: The Sage10kScene object.
        """
        target_path = self._download_scene()
        unzipped = self._unzip_scene(target_path)
        scene = self._parse_json(unzipped)
        return scene
