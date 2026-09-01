from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
VEHICLES = REPO / "vehicles"


@pytest.fixture
def repo() -> Path:
    return REPO
