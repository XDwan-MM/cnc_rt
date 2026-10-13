#!/usr/bin/env python3
"""Isolated simulator regression: sudo python3 test/rt_offline_orient.py.

Use --scenario spindle for the existing M3/M5/M19 sequence, or
--csv-reached 0 to check the original simulator status-word behavior.
"""
import argparse
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--scenario', choices=('cold', 'spindle'), default='cold')
parser.add_argument('--csv-reached', choices=('0', '1'), default='1')
parser.add_argument('--isolated', action='store_true', help=argparse.SUPPRESS)
args = parser.parse_args()
if os.geteuid() != 0:
    parser.error('Run as root: the shared-memory library requires it.')
if not args.isolated:
    raise SystemExit(subprocess.call([
        'unshare', '--ipc', '--fork', sys.executable, str(Path(__file__).resolve()),
        *sys.argv[1:], '--isolated']))

work = Path(tempfile.mkdtemp(prefix='rt-orient-'))
print('Logs:', work, flush=True)
shutil.copytree(ROOT / 'config', work / 'config')
helper = work / 'helper'
subprocess.run([
    'gcc', str(ROOT / 'test/rt_offline_helper.c'), '-o', str(helper),
    '-I' + str(ROOT / 'lib/cncshm/api'), '-I' + str(ROOT / 'lib/cncshm'),
    '-I/usr/include/x86_64-linux-gnu/qt5',
    '-I/usr/include/x86_64-linux-gnu/qt5/QtCore',
    str(ROOT / 'lib/cncshm/bin/libshm.a'),
    '-lQt5Core', '-lstdc++', '-lpthread', '-lm'], check=True)

env = {k: v for k, v in os.environ.items() if not k.startswith('CNC_RT_')}
env.update(TMPDIR=str(work), CNC_RT_SIM_CSV_REACHED=args.csv_reached)
children = []
try:
    with (work / 'helper.log').open('w') as hs_log, (work / 'stdout.log').open('w') as rt_log:
        hs = subprocess.Popen([
            str(helper), '--cold-orient' if args.scenario == 'cold' else '--spindle'],
            cwd=work, env=env, stdout=hs_log, stderr=subprocess.STDOUT)
        children.append(hs)
        time.sleep(0.5)
        rt = subprocess.Popen([str(ROOT / 'bin/cnc_rt_d')], cwd=work, env=env,
                              stdout=rt_log, stderr=subprocess.STDOUT)
        children.append(rt)
        hs_rc = hs.wait(timeout=65)
        early_exit = rt.poll() is not None
        if not early_exit:
            rt.send_signal(signal.SIGINT)
        rt_rc = rt.wait(timeout=10)
    report = (work / 'helper.log').read_text(errors='replace')
    output = (work / 'stdout.log').read_text(errors='replace')
    print(report)
    failures = ('主轴定向请求一直不被受理', '主轴定向超时', '主轴定向释放超时',
                'PDO 循环意外退出')
    assert hs_rc == 0 and 'FAIL' not in report, 'Helper assertions failed'
    assert not early_exit and rt_rc == 0, 'Unexpected RT exit'
    assert not any(message in output for message in failures), 'RT fault in log'
    assert '发送实时关闭完成信号' in output, 'Shutdown did not finish'
    print('PASS:', args.scenario, 'CSV bit10 simulation:', args.csv_reached)
finally:
    for child in children:
        if child.poll() is None:
            child.kill()
        child.wait()
