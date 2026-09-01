import json
import urllib.request

from airbearing.dashboard import Dashboard


def test_dashboard_estop():
    state = {"estop": False, "armed": True, "t": 0.0, "not_flight_software": True}

    def status():
        return dict(state)

    def estop():
        state["estop"] = True

    d = Dashboard(status, estop, port=0)
    url = d.start()
    try:
        with urllib.request.urlopen(url, timeout=2) as r:
            html = r.read().decode()
        assert "E-STOP" in html
        with urllib.request.urlopen(url + "status.json", timeout=2) as r:
            j = json.loads(r.read().decode())
        assert j["armed"] is True
        req = urllib.request.Request(url + "estop", method="POST")
        urllib.request.urlopen(req, timeout=2).read()
        assert state["estop"] is True
    finally:
        d.stop()
