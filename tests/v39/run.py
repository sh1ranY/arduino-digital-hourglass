#!/usr/bin/env python3
"""Run full-sketch host scenarios; never connects to or uploads to a board."""
from pathlib import Path
import hashlib, json, os, subprocess, tempfile
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SOURCE = ROOT / "firmware/Hourglass_v39"
SUITES = [
 ("baseline-native", "host_tests.cpp", [], 32),
 ("baseline-calibration", "host_tests.cpp", ["-DHOURGLASS_CORNER_CALIBRATION=1"], 1),
 ("baseline-avr-api", "host_tests.cpp", ["-DARDUINO_ARCH_AVR=1"], 32),
 ("baseline-r4-bridge", "host_tests.cpp", ["-DARDUINO_UNOR4_WIFI=1", "-DNO_USB=1"], 32),
 ("recovery", "recovery_fault_tests.cpp", ["-DARDUINO_ARCH_RENESAS_UNO=1"], 10),
 ("bus-native", "bus_recovery_tests.cpp", [], None),
 ("bus-bridge", "bus_recovery_tests.cpp", ["-DARDUINO_UNOR4_WIFI=1", "-DNO_USB=1"], None),
 *[(name, name + "_tests.cpp", [], None) for name in
   ["direction", "compact_upper", "finish_flash", "preset_behavior", "aux_clock_behavior"]]
]
def hashes():
 return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
         for p in sorted(list(SOURCE.glob("*")) + list(HERE.glob("*.cpp")) + list(HERE.glob("*.h")) + [Path(__file__).resolve()]) if p.is_file()}
before=hashes()
compiler=os.environ.get("CXX", "clang++")
report={"kind":"host software scenarios with Arduino/I2C doubles; not physical tests",
        "compiler":subprocess.check_output([compiler,"--version"],text=True).splitlines()[0],
        "files_sha256":before,"runs":[]}
with tempfile.TemporaryDirectory(prefix="hourglass-v39-") as work:
 for name, file, flags, expected in SUITES:
  binary=Path(work)/name
  subprocess.run([compiler,"-std=c++11","-O1","-Wall","-Wextra","-Werror","-pedantic",
                  "-I",str(HERE),*flags,str(HERE/file),"-o",str(binary)],check=True)
  discovery=subprocess.run([str(binary)],capture_output=True,text=True,timeout=30)
  # Test executables intentionally return 2 when listing available scenarios.
  if discovery.returncode not in (0,2) or discovery.stderr:
   raise RuntimeError(f"Scenario discovery failed: {name}: {discovery.stderr}")
  names=discovery.stdout.splitlines()
  if file=="host_tests.cpp":
   names=[n for n in names if (n=="calibration_silent")==("calibration" in name)]
  if not names or (expected is not None and len(names)!=expected):
   raise RuntimeError(f"Unexpected scenario count for {name}: {len(names)}")
  cases=[]
  for case in names:
   result=subprocess.run([str(binary),case],capture_output=True,text=True,timeout=90)
   cases.append({"name":case,"exit":result.returncode,"output":result.stdout+result.stderr})
  report["runs"].append({"suite":name,"flags":flags,"cases":cases})
  print(f"{name}: {sum(c['exit']==0 for c in cases)}/{len(cases)}",flush=True)
report["inputs_unchanged"]=before==hashes()
report["executions"]=sum(len(r["cases"]) for r in report["runs"])
report["passed"]=report["inputs_unchanged"] and all(c["exit"]==0 for r in report["runs"] for c in r["cases"])
output=ROOT/"build/v39-host-results.json"
output.parent.mkdir(exist_ok=True)
output.write_text(json.dumps(report,indent=2)+"\n")
if not report["passed"]:raise SystemExit("Host tests failed; inspect build/v39-host-results.json")
print(f"PASS {report['executions']} host scenario executions")
