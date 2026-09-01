"""Planar air-bearing satellite GNC kit (Generation 4). Not flight software."""

__version__ = "4.0.0"
__author__ = "Ævar Öfjörð"

from airbearing.spec import SatelliteSpec, load_vehicle

__all__ = ["SatelliteSpec", "load_vehicle", "__version__"]
