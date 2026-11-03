"""Formal competition entry: same physical-key standby, real work feedback only.

The commissioned backend is supplied by explicit local configuration. No mock
backend or chassis-test fallback is available from this entry point.
"""
import argparse
import json
from pathlib import Path

from tools.chassis_button_service import main as button_main


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--reset-round', action='store_true')
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    backend = config.get('competition_backend')
    if not args.reset_round and (not isinstance(backend, str) or ':' not in backend):
        parser.error('正式扫码/抓放后端尚未配置；不能启动测试驻留来替代正式作业')
    options = ['--config', str(args.config)]
    if args.reset_round:
        options.append('--reset-round')
    else:
        options.extend(['--competition-backend', backend])
    return button_main(options)


if __name__ == '__main__':
    main()
