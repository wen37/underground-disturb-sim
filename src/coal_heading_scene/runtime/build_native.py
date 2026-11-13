"""Build the same native package for either supported source layout."""
import subprocess
from paths import NATIVE_SOURCE, NATIVE_BUILD, NATIVE_INSTALL, NATIVE_LOG
# 按当前工作区位置构建原生插件，使用普通复制安装。
if __name__ == '__main__':
    raise SystemExit(subprocess.call(['colcon','--log-base',str(NATIVE_LOG),
        'build','--base-paths',str(NATIVE_SOURCE),'--packages-select','coal_face_sim',
        '--build-base',str(NATIVE_BUILD),
        '--install-base',str(NATIVE_INSTALL),
        '--cmake-args','-DBUILD_TESTING=OFF']))
