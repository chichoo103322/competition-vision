"""Build the double-clickable macOS app using the current Python environment."""
from pathlib import Path
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    build_root = Path.home()/'Library/Caches/CompetitionRobot/desktop_build'
    bundle = build_root/'dist/小车调试台.app'
    command = [
        sys.executable, '-m', 'PyInstaller', '--noconfirm', '--windowed', '--onedir',
        '--name', '小车调试台', '--osx-bundle-identifier', 'org.competition.robotconsole',
        '--specpath', str(build_root/'spec'), '--distpath', str(build_root/'dist'),
        '--workpath', str(build_root/'work'),
        '--add-data', str(root/'tools/desktop_remote.py')+':.',
        '--add-data', str(root/'tools/camera_console.html')+':.',
        str(root/'tools/robot_desktop.py'),
    ]
    subprocess.run(command, cwd=root, check=True)
    # Sign outside file-provider managed folders, where Finder metadata can reappear.
    subprocess.run(['/usr/bin/xattr', '-cr', str(bundle)], check=True)
    subprocess.run(['/usr/bin/codesign', '--force', '--deep', '--sign', '-', str(bundle)], check=True)
    subprocess.run(['/usr/bin/codesign', '--verify', '--deep', '--strict', str(bundle)], check=True)
    print(bundle)


if __name__ == '__main__':
    main()
