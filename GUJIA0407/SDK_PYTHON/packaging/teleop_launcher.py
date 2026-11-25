# -*- coding: utf-8 -*-
"""GujiaTeleop.exe 打包入口。

同一个 exe 两种模式：
- 无参数：启动骨架摇操前端界面
- --run-control：运行控制脚本（由前端以子进程方式调用）
"""
import os
import sys


def _ensure_std_streams():
    """windowed 模式下 stdout/stderr 可能为 None，这里恢复到继承的管道/句柄。"""
    if sys.stdout is None:
        try:
            sys.stdout = open(1, "w", encoding="utf-8", errors="replace", buffering=1, closefd=False)
        except Exception:
            pass
    if sys.stderr is None:
        try:
            sys.stderr = open(2, "w", encoding="utf-8", errors="replace", buffering=1, closefd=False)
        except Exception:
            pass
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(line_buffering=True)
        except Exception:
            pass


def _setup_source_path():
    """源码运行时把 SDK_PYTHON 和其父目录加入 sys.path。"""
    here = os.path.dirname(os.path.abspath(__file__))
    sdk_dir = os.path.dirname(here)
    project_root = os.path.dirname(sdk_dir)
    for path in (sdk_dir, project_root):
        if path not in sys.path:
            sys.path.insert(0, path)


def main():
    if "--pack-probe" in sys.argv:
        _ensure_std_streams()
        print("PROBE-OK", flush=True)
        return 0

    if "--sdk-probe" in sys.argv:
        _ensure_std_streams()
        if not getattr(sys, "frozen", False):
            _setup_source_path()
        from gujiayaocao_0817_frontend import read_sdk_version
        version, error = read_sdk_version()
        print(f"SDK-VERSION={version} ERROR={error}", flush=True)
        return 0

    if "--run-control" in sys.argv:
        _ensure_std_streams()
        if not getattr(sys, "frozen", False):
            _setup_source_path()
        import gujiayaocao_0817 as control
        control.main()
        return 0

    if not getattr(sys, "frozen", False):
        _setup_source_path()
    import gujiayaocao_0817_frontend as frontend
    app = frontend.TeleopFrontend()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
