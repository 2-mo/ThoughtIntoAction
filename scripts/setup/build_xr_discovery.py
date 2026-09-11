"""Build a native XR discovery switch without changing the Qt runtime or firewall.

Downloads the pinned upstream sources and extracts Ubuntu Qt development headers
locally. Only DeviceConnectionManager is rebuilt, against the installed PC service.
Use --install after validation to atomically install the library with a backup;
the running XR service must then be restarted once to load it.
"""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / 'outputs/build/xr-discovery'
REVISION = '85bac4dbc1fd5cef42c74a160d9c30aa3491f122'
URL = 'https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git'


def run(args, **kwargs):
    return subprocess.run(list(map(str, args)), check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--service-dir', type=Path, default=Path('/opt/apps/roboticsservice'))
    parser.add_argument('--source-checkout', type=Path)
    parser.add_argument('--install', action='store_true')
    args = parser.parse_args()
    service = args.service_dir.resolve()
    BUILD.mkdir(parents=True, exist_ok=True)
    repo = args.source_checkout or BUILD / 'upstream'
    if not repo.exists():
        run(['git', 'init', repo])
        run(['git', '-C', repo, 'fetch', '--depth', '1', URL, REVISION])
    source = BUILD / 'source'
    source.mkdir(exist_ok=True)
    archive = subprocess.check_output(['git', '-C', str(repo), 'archive', REVISION,
                                      'RoboticsService/DeviceConnectionManager', 'RoboticsService/CommonUtils'])
    run(['tar', '-x', '-C', source], input=archive)
    module = source / 'RoboticsService/DeviceConnectionManager'
    worker = module / 'Manage/TCPConnectionManage/tcpserverworker.cpp'
    text = worker.read_text(encoding='utf-8-sig')
    anchor = 'void DevConnSDK::TcpServerWorker::onBraodCastTCPServer()\n{'
    if text.count(anchor) != 1:
        raise RuntimeError('Upstream broadcast function differs from the audited source')
    text = text.replace(anchor, anchor + '\n    if (!questDiscoveryEnabled()) return;')
    worker.write_text('#include "discovery_control.h"\n' + text)
    shutil.copy2(ROOT / 'scripts/setup/xr-discovery/discovery_control.h', module)
    qt = BUILD / 'qt'
    moc = qt / 'usr/lib/qt6/libexec/moc'
    if not moc.exists():
        debs = BUILD / 'qt-debs'
        debs.mkdir(exist_ok=True)
        run(['apt-get', 'download', 'qt6-base-dev', 'qt6-base-dev-tools'], cwd=debs)
        for deb in debs.glob('*.deb'):
            run(['dpkg-deb', '-x', deb, qt])
    includes = [module, module.parent / 'CommonUtils', qt / 'usr/include/x86_64-linux-gnu/qt6']
    includes += [includes[-1] / name for name in ('QtCore', 'QtNetwork')]
    flags = ['-DDEVICECONNECTIONSDK_LIBRARY', '-DLINUX_x86', *['-I' + str(p) for p in includes]]
    env = {**os.environ, 'LD_LIBRARY_PATH': str(service / 'lib') + ':' + str(service)}
    generated = BUILD / 'moc'
    generated.mkdir(exist_ok=True)
    mocs = []
    for header in module.rglob('*.h'):
        if 'Q_OBJECT' in header.read_text(encoding='utf-8-sig'):
            output = generated / ('moc_' + header.stem + '.cpp')
            run([moc, *flags, header, '-o', output], env=env)
            mocs.append(output)
    output = BUILD / 'libDeviceConnectionManager.so'
    temporary = output.with_suffix('.so.building')
    run(['g++', '-std=c++17', '-O2', '-shared', '-fPIC', '-pthread', *flags,
         *module.rglob('*.cpp'), *mocs, '-L' + str(service), '-L' + str(service / 'lib'),
         '-lCommonUtils', '-lQt6Network', '-lQt6Core', '-Wl,-soname,libDeviceConnectionManager.so',
         '-Wl,-z,defs', '-o', temporary], env=env)
    temporary.replace(output)
    print('Built:', output, flush=True)
    test = BUILD / 'discovery-test'
    run(['g++', '-std=c++17', '-fPIC', *flags,
         ROOT / 'scripts/setup/xr-discovery/test_discovery.cpp',
         '-L' + str(BUILD), '-L' + str(service / 'lib'),
         '-lDeviceConnectionManager', '-lQt6Network', '-lQt6Core',
         '-Wl,--export-dynamic', '-Wl,-rpath-link,' + str(service), '-o', test], env=env)
    run([test], env={**env, 'LD_LIBRARY_PATH': str(BUILD) + ':' + env['LD_LIBRARY_PATH']})
    if args.install:
        target = service / output.name
        digest = hashlib.sha256(target.read_bytes()).hexdigest()[:16]
        backup = target.with_name(target.name + '.backup-' + digest)
        if not backup.exists():
            shutil.copy2(target, backup)
        pending = target.with_name(target.name + '.quest-new')
        shutil.copy2(output, pending)
        pending.chmod(0o755)
        pending.replace(target)
        print('Installed:', target, '\nBackup:', backup,
              '\nRestart XR service once to load the switch. USB tracking reconnects during that restart.')


if __name__ == '__main__':
    main()
