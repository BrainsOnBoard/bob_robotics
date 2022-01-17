import os.path
import subprocess as sp
from warnings import warn

def get_git_version():
    mydir = os.path.dirname(__file__)

    # Try to make a version string based on status of git tree
    try:
        output = sp.check_output(["git", "log", "-1", '--format=%cd.%h', "--date=format:%Y%m%d"], cwd=mydir)
        version = "1.0+" + output.decode('utf-8').rstrip()

        ret = sp.run(["git", "diff", "--no-ext-diff", "--quiet", "--exit-code"], cwd=mydir)
        if ret.returncode != 0:
            version += '.dirty'
        return version

    except sp.CalledProcessError:
        warn('Could not get current git commit; version of code is unknown')
        return '1.0+(unknown)'
