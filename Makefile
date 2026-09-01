PYTHON ?= python3
ifneq ($(wildcard .venv/bin/python),)
PYTHON := .venv/bin/python
endif
PIP ?= $(PYTHON) -m pip
VEHICLE ?= vehicles/uk_solenoid_octagon.json

.PHONY: install test run compare-actuators new-vehicle assets

install:
	$(PIP) install -e ".[dev]"

test:
	$(PYTHON) -m pytest

run:
	$(PYTHON) -m airbearing run --vehicle $(VEHICLE)

compare-actuators:
	$(PYTHON) -m airbearing compare-actuators

new-vehicle:
	$(PYTHON) -m airbearing new-vehicle

assets:
	$(PYTHON) -m airbearing run --vehicle vehicles/uk_solenoid_octagon.json --assets
	$(PYTHON) -m airbearing compare-actuators --assets
