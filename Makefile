PYTHON ?= python3
ifneq ($(wildcard .venv/bin/python),)
PYTHON := .venv/bin/python
endif
PIP ?= $(PYTHON) -m pip
VEHICLE ?= vehicles/uk_solenoid_octagon.json

.PHONY: install test run compare-actuators new-vehicle edit-vehicle view check assets lab1 lab2 lab3 lab4

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

edit-vehicle:
	$(PYTHON) -m airbearing edit-vehicle

view:
	$(PYTHON) -m airbearing view --vehicle $(VEHICLE)

check:
	$(PYTHON) -m airbearing check $(VEHICLE)

assets:
	$(PYTHON) -m airbearing run --vehicle vehicles/uk_solenoid_octagon.json --assets
	$(PYTHON) -m airbearing compare-actuators --assets
	$(PYTHON) -m airbearing view --record --duration 3 --vehicle vehicles/fan_quadrotor_plus.json

lab1:
	$(PYTHON) -m airbearing lab 1

lab2:
	$(PYTHON) -m airbearing lab 2

lab3:
	$(PYTHON) -m airbearing lab 3

lab4:
	$(PYTHON) -m airbearing lab 4
