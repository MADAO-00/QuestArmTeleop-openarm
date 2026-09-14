from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SOURCE_HELPER = ROOT / "scripts" / "source_openarm_v1_overlay.sh"


def write_source_file(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def prefix_setup(log_event: str) -> str:
    return f'''printf '%s\\n' {log_event!r} >>"$OPENARM_TEST_LOG"
export COLCON_PREFIX_PATH="$OPENARM_TEST_CURRENT_PREFIX${{COLCON_PREFIX_PATH:+:$COLCON_PREFIX_PATH}}"
export AMENT_PREFIX_PATH="$OPENARM_TEST_CURRENT_PREFIX${{AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}}"
export CMAKE_PREFIX_PATH="$OPENARM_TEST_CURRENT_PREFIX${{CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}}"
export PYTHONPATH="$OPENARM_TEST_CURRENT_PREFIX/python${{PYTHONPATH:+:$PYTHONPATH}}"
'''


def write_python_stub(tmp_path: Path) -> Path:
    python = tmp_path / "openarm-venv" / "bin" / "python"
    write_source_file(
        python,
        '#!/usr/bin/bash\nprintf "openarm-python\\n" >>"$OPENARM_TEST_LOG"\n',
    )
    python.chmod(0o755)
    return python


def base_source_environment(
    tmp_path: Path,
    *,
    ros_setup: Path,
    python: Path,
    openarm_ws: Path,
    questarm_dir: Path,
    overlay_prefix: Path,
) -> dict[str, str]:
    return {
        "PATH": "/usr/bin:/bin",
        "ROS_DISTRO": "humble",
        "OPENARM_TEST_LOG": str(tmp_path / "source-events.log"),
        "OPENARM_V1_ROS_SETUP": str(ros_setup),
        "OPENARM_VT_PYTHON": str(python),
        "OPENARM_WS": str(openarm_ws),
        "QUESTARMTELEOP_DIR": str(questarm_dir),
        "OPENARM_V1_OVERLAY_PREFIX": str(overlay_prefix),
    }


@pytest.mark.parametrize("inherited_skip", [None, "0", "1"])
def test_sources_local_setup_without_replaying_setup_chain(
    tmp_path: Path, inherited_skip: str | None,
) -> None:
    log = tmp_path / "source-events.log"
    ros_prefix = tmp_path / "reviewed-ros"
    openarm_ws = tmp_path / "reviewed-openarm"
    openarm_prefix = openarm_ws / "install"
    questarm_dir = tmp_path / "legacy-questarm"
    overlay_prefix = tmp_path / "target-overlay"
    legacy_piper_hook = tmp_path / "legacy-piper" / "local_setup.bash"
    legacy_workspace_hook = tmp_path / "legacy-workspace" / "local_setup.bash"
    python = write_python_stub(tmp_path)

    write_source_file(
        ros_prefix / "setup.bash",
        prefix_setup("ros-setup").replace(
            "$OPENARM_TEST_CURRENT_PREFIX", "$OPENARM_TEST_ROS_PREFIX"
        ),
    )
    write_source_file(
        openarm_prefix / "setup.bash",
        prefix_setup("openarm-setup").replace(
            "$OPENARM_TEST_CURRENT_PREFIX", "$OPENARM_TEST_OPENARM_PREFIX"
        ),
    )
    write_source_file(
        questarm_dir / "install" / "setup.bash",
        "printf 'legacy-workspace-direct\\n' >>\"$OPENARM_TEST_LOG\"\n",
    )
    write_source_file(
        legacy_piper_hook,
        "printf 'legacy-piper-chain\\n' >>\"$OPENARM_TEST_LOG\"\n",
    )
    write_source_file(
        legacy_workspace_hook,
        "printf 'legacy-workspace-chain\\n' >>\"$OPENARM_TEST_LOG\"\n",
    )
    write_source_file(
        overlay_prefix / "setup.bash",
        '''printf 'target-setup\\n' >>"$OPENARM_TEST_LOG"
source "$OPENARM_TEST_LEGACY_PIPER_HOOK"
source "$OPENARM_TEST_LEGACY_WORKSPACE_HOOK"
return 97
''',
    )
    write_source_file(
        overlay_prefix / "local_setup.bash",
        '''printf 'target-local\\n' >>"$OPENARM_TEST_LOG"
export OPENARM_TEST_LOCAL_SETUP=1
export COLCON_PREFIX_PATH="$OPENARM_V1_OVERLAY_PREFIX${COLCON_PREFIX_PATH:+:$COLCON_PREFIX_PATH}"
export AMENT_PREFIX_PATH="$OPENARM_V1_OVERLAY_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$OPENARM_V1_OVERLAY_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
''',
    )

    legacy_prefixes = [tmp_path / "inherited-piper", tmp_path / "inherited-workspace"]
    for prefix in legacy_prefixes:
        prefix.mkdir()
    env = base_source_environment(
        tmp_path,
        ros_setup=ros_prefix / "setup.bash",
        python=python,
        openarm_ws=openarm_ws,
        questarm_dir=questarm_dir,
        overlay_prefix=overlay_prefix,
    )
    env.update(
        {
            "OPENARM_TEST_ROS_PREFIX": str(ros_prefix),
            "OPENARM_TEST_OPENARM_PREFIX": str(openarm_prefix),
            "OPENARM_TEST_LEGACY_PIPER_HOOK": str(legacy_piper_hook),
            "OPENARM_TEST_LEGACY_WORKSPACE_HOOK": str(legacy_workspace_hook),
            "COLCON_PREFIX_PATH": os.pathsep.join(map(str, legacy_prefixes)),
            "AMENT_PREFIX_PATH": os.pathsep.join(map(str, legacy_prefixes)),
            "CMAKE_PREFIX_PATH": os.pathsep.join(map(str, legacy_prefixes)),
            "PYTHONPATH": os.pathsep.join(map(str, legacy_prefixes)),
        }
    )
    if inherited_skip is not None:
        env["OPENARM_V1_SKIP_LEGACY_OVERLAY"] = inherited_skip
    expected_prefix_path = os.pathsep.join(
        (str(overlay_prefix), str(openarm_prefix), str(ros_prefix))
    )

    result = subprocess.run(
        [
            "/usr/bin/bash",
            "--noprofile",
            "--norc",
            "-c",
            '''set -euo pipefail
source "$1"
[[ ${OPENARM_TEST_LOCAL_SETUP:-0} == 1 ]]
[[ $COLCON_PREFIX_PATH == "$2" ]]
[[ $AMENT_PREFIX_PATH == "$2" ]]
[[ $CMAKE_PREFIX_PATH == "$2" ]]
[[ $PYTHONPATH == "$3" ]]
[[ ${OPENARM_V1_SKIP_LEGACY_OVERLAY} == 1 ]]
[[ $(command -v python) == "$OPENARM_VT_PYTHON" ]]
[[ $- == *u* ]]
python
''',
            "bash",
            str(SOURCE_HELPER),
            expected_prefix_path,
            os.pathsep.join((str(openarm_prefix / "python"), str(ros_prefix / "python"))),
        ],
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    events = log.read_text(encoding="utf-8").splitlines()
    assert events == [
        "ros-setup",
        "openarm-setup",
        "target-local",
        "openarm-python",
    ]
    for forbidden_event in (
        "target-setup",
        "legacy-piper-chain",
        "legacy-workspace-chain",
        "legacy-workspace-direct",
    ):
        assert forbidden_event not in events


def test_build_environment_generates_clean_colcon_setup_chain(
    tmp_path: Path,
) -> None:
    colcon = shutil.which("colcon", path="/usr/bin:/bin")
    assert colcon is not None

    ros_prefix = tmp_path / "reviewed-ros"
    openarm_ws = tmp_path / "reviewed-openarm"
    openarm_prefix = openarm_ws / "install"
    questarm_dir = tmp_path / "legacy-questarm"
    python = write_python_stub(tmp_path)
    legacy_prefixes = [tmp_path / "legacy-piper", tmp_path / "legacy-workspace"]
    for prefix in (ros_prefix, openarm_prefix, *legacy_prefixes):
        write_source_file(prefix / "local_setup.bash", ":\n")
        write_source_file(prefix / ".catkin", "\n")
    write_source_file(
        ros_prefix / "setup.bash",
        prefix_setup("ros-setup").replace(
            "$OPENARM_TEST_CURRENT_PREFIX", "$OPENARM_TEST_ROS_PREFIX"
        ),
    )
    write_source_file(
        openarm_prefix / "setup.bash",
        prefix_setup("openarm-setup").replace(
            "$OPENARM_TEST_CURRENT_PREFIX", "$OPENARM_TEST_OPENARM_PREFIX"
        ),
    )
    source_tree = tmp_path / "empty-src"
    source_tree.mkdir()
    install_prefix = tmp_path / "generated-overlay" / "install"

    env = base_source_environment(
        tmp_path,
        ros_setup=ros_prefix / "setup.bash",
        python=python,
        openarm_ws=openarm_ws,
        questarm_dir=questarm_dir,
        overlay_prefix=tmp_path / "not-built-yet",
    )
    env.update(
        {
            "OPENARM_TEST_ROS_PREFIX": str(ros_prefix),
            "OPENARM_TEST_OPENARM_PREFIX": str(openarm_prefix),
            "COLCON_PREFIX_PATH": os.pathsep.join(map(str, legacy_prefixes)),
            "AMENT_PREFIX_PATH": os.pathsep.join(map(str, legacy_prefixes)),
            "CMAKE_PREFIX_PATH": os.pathsep.join(map(str, legacy_prefixes)),
            # Even an inherited request to use legacy prefixes cannot enable them.
            "OPENARM_V1_SKIP_LEGACY_OVERLAY": "0",
        }
    )
    result = subprocess.run(
        [
            "/usr/bin/bash",
            "--noprofile",
            "--norc",
            "-c",
            '''set -euo pipefail
source "$1"
"$2" --log-base "$3" build --base-paths "$4" --build-base "$5" --install-base "$6" --event-handlers console_direct+
''',
            "bash",
            str(SOURCE_HELPER),
            colcon,
            str(tmp_path / "colcon-log"),
            str(source_tree),
            str(tmp_path / "colcon-build"),
            str(install_prefix),
        ],
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    for setup_name in ("setup.bash", "setup.sh", "setup.zsh"):
        setup_chain = (install_prefix / setup_name).read_text(encoding="utf-8")
        assert str(ros_prefix) in setup_chain
        assert str(openarm_prefix) in setup_chain
        for legacy_prefix in legacy_prefixes:
            assert str(legacy_prefix) not in setup_chain
