PYTHON ?= python3
ifneq ($(wildcard .venv/bin/python),)
PYTHON := .venv/bin/python
endif
PIP ?= $(PYTHON) -m pip
VEHICLE ?= examples/vehicles/fan_plus.json

.PHONY: install test run view edit-vehicle new-vehicle check lab1 lab2 lab3 lab4 identify compare report compare-actuators assets

install:
	$(PIP) install -e ".[dev,viz]"

test:
	$(PYTHON) -m pytest

run:
	$(PYTHON) -m airbearing run --vehicle $(VEHICLE)

view:
	$(PYTHON) -m airbearing view --vehicle $(VEHICLE)

edit-vehicle:
	$(PYTHON) -m airbearing edit-vehicle

new-vehicle:
	$(PYTHON) -m airbearing new-vehicle

check:
	$(PYTHON) -m airbearing check $(VEHICLE)

lab1:
	$(PYTHON) -m airbearing lab 1

lab2:
	$(PYTHON) -m airbearing lab 2

lab3:
	$(PYTHON) -m airbearing lab 3

lab4:
	$(PYTHON) -m airbearing lab 4

identify:
	$(PYTHON) -m airbearing identify $(LOG) --vehicle $(VEHICLE)

compare:
	$(PYTHON) -m airbearing compare --sim $(SIM) --real $(REAL) --mismatch-delay 1

report:
	$(PYTHON) -m airbearing report $(RUN)

compare-actuators:
	$(PYTHON) -m airbearing compare-actuators

assets:
	$(PYTHON) -m airbearing run --vehicle examples/vehicles/solenoid_octagon.json --assets
	$(PYTHON) -m airbearing compare-actuators --assets
	$(PYTHON) -m airbearing view --record --duration 3 --vehicle examples/vehicles/fan_plus.json
