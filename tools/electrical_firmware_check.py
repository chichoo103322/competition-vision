"""Run supplied STM32 C host tests against our Python codec, without hardware.

Mac: python3 -m tools.electrical_firmware_check /path/to/car_code1
Reads the supplied firmware, compiles in a temporary directory, never edits it.
Requires clang and the supplied tests/route host mocks; no STM32 flashing.
"""
import argparse
import hashlib
from pathlib import Path
import re
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('firmware', type=Path)
    args = parser.parse_args()
    root = args.firmware.resolve()
    sources = [root / 'Core/Src/route.c', root / 'Core/Src/route_uart.c']
    if (root / 'Core/Src/route_keys.c').exists():
        sources.append(root / 'Core/Src/route_keys.c')
    with tempfile.TemporaryDirectory(prefix='electrical-host-') as directory:
        temporary = Path(directory)
        harness = (root / 'tests/route/harness.c').read_text()
        harness = harness.replace('#define EX __declspec(dllexport)', '#define EX')
        # Windows freestanding mocks supply libc; use host libc on macOS/Linux.
        harness = re.sub(r'void \*mem(?:set|cpy)\([^\n]*\)\n\{[^}]*\}\n', '', harness)
        (temporary / 'harness.c').write_text(harness)
        library = temporary / 'route_test.so'
        subprocess.run(['clang', '-shared', '-fPIC', '-std=c99', '-Wall', '-Wextra', '-Werror', '-U__weak',
                        '-include', str(root / 'tests/route/mock.h'),
                        '-I' + str(root / 'Core/Inc'), '-I' + str(root / 'imu'),
                        *(str(s) for s in sources), str(temporary / 'harness.c'),
                        '-o', str(library)], check=True)
        tests = (root / 'tests/route/run_tests.py').read_text()
        tests = tests.replace('ROOT = Path(__file__).resolve().parents[2]', f'ROOT = Path({str(root)!r})')
        tests = tests.replace('import route_client as P', 'from pi_vision import electrical_protocol as P')
        tests = tests.replace("dll = C.CDLL(str(ROOT / 'tests/route/route_test.dll'))",
                              f'dll = C.CDLL({str(library)!r})')
        # Run the 12 production-C cases. The supplied Python Client has a
        # different API; our client is covered by tests/test_electrical_communication.py.
        (temporary / 'run_tests.py').write_text(tests)
        subprocess.run([sys.executable, '-c',
                        'import runpy,sys; sys.path.insert(0,sys.argv[1]); '
                        'sys.argv=[sys.argv[2],"RouteTests"]; runpy.run_path(sys.argv[0],run_name="__main__")',
                        str(Path(__file__).resolve().parents[1]), str(temporary / 'run_tests.py')], check=True)
    for source in sources:
        print(f'{source.name} sha256={hashlib.sha256(source.read_bytes()).hexdigest()}')
    print('C host simulation only; no real UART or motors tested. Area/key cases run when present in supplied firmware.')


if __name__ == '__main__':
    main()
