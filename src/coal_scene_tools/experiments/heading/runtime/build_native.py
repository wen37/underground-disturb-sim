"""Build the same native package for either supported source layout."""
import subprocess
from paths import ROOT, NATIVE_SOURCE
if __name__ == '__main__':
    raise SystemExit(subprocess.call(['colcon','--log-base',str(ROOT/'evidence/native_log'),
        'build','--base-paths',str(NATIVE_SOURCE),'--packages-select','coal_face_sim',
        '--build-base',str(ROOT/'evidence/native_build'),
        '--install-base',str(ROOT/'evidence/native_install'),
        '--cmake-args','-DBUILD_TESTING=OFF']))
