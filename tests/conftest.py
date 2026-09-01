import os
from pathlib import Path

import pytest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

REPO = Path(__file__).resolve().parents[1]
VEHICLES = REPO / "vehicles"


@pytest.fixture
def repo() -> Path:
    return REPO
