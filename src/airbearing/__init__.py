"""Planar (x, y, yaw) GNC for cubesats on air-bearing tables. Not flight software."""

__version__ = "1.0.0"
__author__ = "Ævar Öfjörð"

from airbearing.spec import SatelliteSpec, load_vehicle

__all__ = ["SatelliteSpec", "load_vehicle", "__version__"]
