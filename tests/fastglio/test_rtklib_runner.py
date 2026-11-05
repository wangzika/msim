#!/usr/bin/env python3
import importlib.util
import pathlib
import os
import sys
import tempfile
import time
from unittest import mock


HEADER = "% program   : RTKLIB ver.demo5 b34c\n% obs start : 2021/06/19 08:58:48.0 GPST\n"


def row(time, latitude="37.0", status="5"):
    return f"2021/06/19 {time} {latitude} -121.0 20.0 {status} 8 1 1 1 1 1 1 0 3.1\n"


def main():
    spec = importlib.util.spec_from_file_location("rtklib_runner", pathlib.Path(sys.argv[1]))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with tempfile.TemporaryDirectory(prefix="p4-runner-test-") as tmp:
        root = pathlib.Path(tmp)
        good = root / "good.pos"
        good.write_text(HEADER + row("08:58:48.000") + row("08:58:48.200"))
        parsed = module.parse_pos(good)
        assert len(parsed) == 2 and parsed[0]["status_code"] == 5
        bad_cases = {
            "nan": HEADER + row("08:58:48.000", "nan"),
            "status": HEADER + row("08:58:48.000", status="99"),
            "backwards": HEADER + row("08:58:48.200") + row("08:58:48.000"),
            "duplicate": HEADER + row("08:58:48.000") + row("08:58:48.000"),
            "malformed": HEADER + "broken row\n",
            "no-gpst": "% program : RTKLIB\n" + row("08:58:48.000"),
        }
        for name, content in bad_cases.items():
            path = root / f"{name}.pos"
            path.write_text(content)
            try:
                module.parse_pos(path)
                raise AssertionError(f"invalid case accepted: {name}")
            except ValueError:
                pass
        helper = root / "helper.py"
        pidfile = root / "grandchild.pid"
        helper.write_text("import subprocess,sys,time\n"
                          "p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'])\n"
                          "open(sys.argv[1],'w').write(str(p.pid))\n"
                          "time.sleep(60)\n")
        try:
            module.run_process_group([sys.executable, str(helper), str(pidfile)], root,
                                     os.environ.copy(), root / "timeout.log", 0.2)
            raise AssertionError("timeout helper unexpectedly succeeded")
        except RuntimeError as exc:
            assert "timed out" in str(exc) and (root / "timeout.log").is_file()
        grandchild = int(pidfile.read_text())
        for _ in range(20):
            try:
                os.kill(grandchild, 0)
            except ProcessLookupError:
                break
            time.sleep(0.05)
        else:
            # A killed orphan may briefly remain as an init-owned zombie; it
            # must no longer be a live non-zombie task.
            stat = pathlib.Path(f"/proc/{grandchild}/stat")
            assert stat.is_file() and stat.read_text().split()[2] == "Z"
        try:
            module.run_process_group([sys.executable, "-c", "raise SystemExit(7)"], root,
                                     os.environ.copy(), root / "failure.log", 5)
            raise AssertionError("non-zero child unexpectedly succeeded")
        except RuntimeError as exc:
            assert "exited 7" in str(exc) and (root / "failure.log").is_file()
        fake = mock.Mock(pid=123456)
        fake.wait.side_effect = [KeyboardInterrupt(), 0]
        fake.poll.return_value = None
        with mock.patch.object(module.subprocess, "Popen", return_value=fake), \
                mock.patch.object(module.os, "killpg") as killpg:
            try:
                module.run_process_group(["unused"], root, os.environ.copy(),
                                         root / "cancel.log", 5)
                raise AssertionError("cancellation was swallowed")
            except KeyboardInterrupt:
                pass
            killpg.assert_called_once_with(123456, module.signal.SIGTERM)
    config = module.config_text("relative_fixed", "continuous", 0.25)
    assert "stats-errsnr=0.25" in config and "out-timesys=gpst" in config
    assert "pos1-navsys=45" in module.config_text("relative_fixed", "continuous", 0.25,
                                                 navsys=45)
    print("RTKLIB solution parser contract passed")


if __name__ == "__main__":
    main()
