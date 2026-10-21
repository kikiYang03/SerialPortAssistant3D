#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键打包：编译 -> windeployqt -> 生成 .evb 工程 -> Enigma Virtual Box 打成单文件 exe

用法（在工程根目录）：
    python tools/package.py                  # Release，完整流程（推荐对外发布用）
    python tools/package.py --debug          # Debug，自己验证用
    python tools/package.py --no-build       # 跳过编译，直接用现有产物重新打包
    python tools/package.py --no-config      # config.ini 不打进盒子，随 exe 单独放

产物：build/dist/luaPort1.exe  <- 单文件，拷到没装 Qt / 没装 VC 运行库的机器上直接跑

为什么不用手点 Enigma Virtual Box GUI：
    windeployqt 每次可能产出不同的 DLL 集合（换了 Qt 模块、换了版本都会变），
    手工维护 .evb 里的文件列表迟早漏文件，且漏的往往正是插件目录下的 dll，
    表现是程序在开发机好好的、到别人机器上起不来。这里改成扫描暂存目录自动生成。
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

# ---------------------------------------------------------------- 环境路径
ROOT = Path(__file__).resolve().parent.parent
QT_BIN = Path(r"D:\apps\codeApp\Qt5.15.2\5.15.2\msvc2019_64\bin")
EVB_CONSOLE = Path(r"D:\apps\codeApp\Enigma Virtual Box\enigmavbconsole.exe")
BUILD_BAT = ROOT / ".vscode" / "build.bat"
# VC142 运行时（msvcp140 / vcruntime140 / concrt140 ...）。windeployqt 的
# --compiler-runtime 找不到 VS 安装信息时会静默跳过，那时用这里的作为兜底。
CRT_REDIST = (Path(r"D:\apps\codeApp\VisualStudioDownload\Professional")
              / "VC" / "Redist" / "MSVC" / "14.29.30133" / "x64" / "Microsoft.VC142.CRT")

TARGET = "luaPort1"
STAGE = ROOT / "build" / "exe"          # windeployqt 的落地目录，也是 .evb 的根
DIST = ROOT / "build" / "dist"          # 最终单文件输出
EVB_PROJECT = ROOT / "build" / "package.evb"

# Enigma Virtual Box 工程里固定不变的尾部配置（取自手工建好的工程，行为一致）
EVB_TAIL = """  <Registries>
    <Enabled>False</Enabled>
    <Registries>
      <Registry>
        <Type>1</Type>
        <Virtual>True</Virtual>
        <Name>Classes</Name>
        <ValueType>0</ValueType>
        <Value/>
        <Registries/>
      </Registry>
      <Registry>
        <Type>1</Type>
        <Virtual>True</Virtual>
        <Name>User</Name>
        <ValueType>0</ValueType>
        <Value/>
        <Registries/>
      </Registry>
      <Registry>
        <Type>1</Type>
        <Virtual>True</Virtual>
        <Name>Machine</Name>
        <ValueType>0</ValueType>
        <Value/>
        <Registries/>
      </Registry>
      <Registry>
        <Type>1</Type>
        <Virtual>True</Virtual>
        <Name>Users</Name>
        <ValueType>0</ValueType>
        <Value/>
        <Registries/>
      </Registry>
      <Registry>
        <Type>1</Type>
        <Virtual>True</Virtual>
        <Name>Config</Name>
        <ValueType>0</ValueType>
        <Value/>
        <Registries/>
      </Registry>
    </Registries>
  </Registries>
  <Packaging>
    <Enabled>False</Enabled>
  </Packaging>
  <Options>
    <ShareVirtualSystem>False</ShareVirtualSystem>
    <MapExecutableWithTemporaryFile>True</MapExecutableWithTemporaryFile>
    <TemporaryFileMask/>
    <AllowRunningOfVirtualExeFiles>True</AllowRunningOfVirtualExeFiles>
    <ProcessesOfAnyPlatforms>False</ProcessesOfAnyPlatforms>
  </Options>
  <Storage>
    <Files>
      <Enabled>False</Enabled>
      <Folder>%DEFAULT FOLDER%\\</Folder>
      <RandomFileNames>False</RandomFileNames>
      <EncryptContent>False</EncryptContent>
    </Files>
  </Storage>
</>
"""


def log(msg: str) -> None:
    print(f"[package] {msg}", flush=True)


def die(msg: str) -> None:
    print(f"[package] 失败: {msg}", file=sys.stderr, flush=True)
    raise SystemExit(1)


def run(cmd, cwd=None, env=None) -> None:
    log("执行: " + " ".join(str(c) for c in cmd))
    proc = subprocess.run([str(c) for c in cmd], cwd=str(cwd) if cwd else None, env=env)
    if proc.returncode != 0:
        die(f"命令返回 {proc.returncode}")


def qt_env() -> dict:
    """windeployqt / qmake 需要 Qt 的 bin 在 PATH 里。"""
    env = dict(os.environ)
    env["PATH"] = str(QT_BIN) + os.pathsep + env.get("PATH", "")
    return env


# ---------------------------------------------------------------- 1. 编译
def build(build_type: str) -> Path:
    if not BUILD_BAT.exists():
        die(f"找不到构建脚本 {BUILD_BAT}")
    run(["cmd", "/c", str(BUILD_BAT), build_type], cwd=ROOT)

    exe = ROOT / "build" / "vscode" / build_type / f"{TARGET}.exe"
    if not exe.exists():
        die(f"编译完成但没找到 {exe}")
    return exe


def find_existing_exe(build_type: str) -> Path:
    exe = ROOT / "build" / "vscode" / build_type / f"{TARGET}.exe"
    if not exe.exists():
        die(f"--no-build 但 {exe} 不存在，先编译一次")
    return exe


# ---------------------------------------------------------------- 2. 暂存 + windeployqt
def prepare_stage(exe: Path, build_type: str, with_config: bool) -> None:
    if STAGE.exists():
        log(f"清空暂存目录 {STAGE}")
        shutil.rmtree(STAGE)
    STAGE.mkdir(parents=True, exist_ok=True)

    staged_exe = STAGE / f"{TARGET}.exe"
    log(f"拷贝 exe -> {staged_exe}")
    shutil.copy2(exe, staged_exe)

    args = [
        QT_BIN / "windeployqt.exe",
        "--debug" if build_type == "debug" else "--release",
        "--no-translations",          # 不需要 Qt 的多语言翻译，省 30 个文件
    ]
    if build_type == "release":
        # 把 VC 运行库一起带上，目标机没装 VS 也能跑
        args.append("--compiler-runtime")
    args.append(staged_exe)
    run(args, env=qt_env())

    if with_config:
        src = ROOT / "config" / "config.ini"
        if src.exists():
            log("拷贝 config.ini（打进盒子，程序按 exe 同目录读取）")
            shutil.copy2(src, STAGE / "config.ini")
        else:
            log("警告: config/config.ini 不存在，跳过")

    if build_type == "release":
        ensure_crt()


def ensure_crt() -> None:
    """确保 VC142 运行时 DLL 在暂存目录里。

    windeployqt --compiler-runtime 依赖注册表/vswhere 找到 VS；
    找不到时只打一行 warning 就过去了，目标机就会缺 msvcp140.dll。
    这里做一次显式检查，缺了就从 VC\Redist 直接拷。
    """
    if (STAGE / "msvcp140.dll").exists():
        log("VC 运行库: windeployqt 已带上")
        return
    if not CRT_REDIST.is_dir():
        log(f"警告: 没找到 VC 运行库目录 {CRT_REDIST}，目标机需自装 VC++ 2015-2022 运行库")
        return

    copied = 0
    for dll in sorted(CRT_REDIST.glob("*.dll")):
        shutil.copy2(dll, STAGE / dll.name)
        copied += 1
    log(f"VC 运行库: windeployqt 没带，已从 Redist 补 {copied} 个 DLL")


# ---------------------------------------------------------------- 3. 生成 .evb
def _esc(text: str) -> str:
    return (text.replace("&", "&amp;").replace("<", "&lt;")
                .replace(">", "&gt;").replace('"', "&quot;"))


def _file_entry(name: str, src: Path, depth: int) -> str:
    pad = "  " * depth
    return (
        f"{pad}<File>\n"
        f"{pad}  <Type>2</Type>\n"
        f"{pad}  <Name>{_esc(name)}</Name>\n"
        f"{pad}  <File>{_esc(str(src))}</File>\n"
        f"{pad}  <ActiveX>False</ActiveX>\n"
        f"{pad}  <ActiveXInstall>False</ActiveXInstall>\n"
        f"{pad}  <Action>0</Action>\n"
        f"{pad}  <OverwriteDateTime>False</OverwriteDateTime>\n"
        f"{pad}  <OverwriteAttributes>False</OverwriteAttributes>\n"
        f"{pad}  <PassCommandLine>False</PassCommandLine>\n"
        f"{pad}  <HideFromDialogs>0</HideFromDialogs>\n"
        f"{pad}</File>\n"
    )


def _folder_entry(name: str, folder: Path, depth: int) -> str:
    pad = "  " * depth
    head = (
        f"{pad}<File>\n"
        f"{pad}  <Type>3</Type>\n"
        f"{pad}  <Name>{_esc(name)}</Name>\n"
        f"{pad}  <Action>0</Action>\n"
        f"{pad}  <OverwriteDateTime>False</OverwriteDateTime>\n"
        f"{pad}  <OverwriteAttributes>False</OverwriteAttributes>\n"
        f"{pad}  <HideFromDialogs>0</HideFromDialogs>\n"
        f"{pad}  <Files>\n"
    )
    children = []
    # 目录在前、文件在后，各自按名字排序，保证每次生成结果稳定，便于 diff
    for sub in sorted(p for p in folder.iterdir() if p.is_dir()):
        children.append(_folder_entry(sub.name, sub, depth + 2))
    for f in sorted(p for p in folder.iterdir() if p.is_file()):
        children.append(_file_entry(f.name, f, depth + 2))
    body = "".join(children)
    return head + body + f"{pad}  </Files>\n{pad}</File>\n"


def write_evb(input_exe: Path, output_exe: Path) -> None:
    files_block = _folder_entry("%DEFAULT FOLDER%", STAGE, 4)
    xml = (
        '<?xml version="1.0" encoding="windows-1252"?>\n'
        "<>\n"
        f"  <InputFile>{_esc(str(input_exe))}</InputFile>\n"
        f"  <OutputFile>{_esc(str(output_exe))}</OutputFile>\n"
        "  <Files>\n"
        "    <Enabled>True</Enabled>\n"
        "    <DeleteExtractedOnExit>False</DeleteExtractedOnExit>\n"
        "    <CompressFiles>False</CompressFiles>\n"
        "    <Files>\n"
        f"{files_block}"
        "    </Files>\n"
        "  </Files>\n"
        + EVB_TAIL
    )

    EVB_PROJECT.parent.mkdir(parents=True, exist_ok=True)
    try:
        data = xml.encode("cp1252")
    except UnicodeEncodeError:
        # 路径里出现中文时退回 UTF-8，声明同步改掉
        xml = xml.replace('encoding="windows-1252"', 'encoding="utf-8"', 1)
        data = xml.encode("utf-8")
    EVB_PROJECT.write_bytes(data)
    log(f"生成工程文件 {EVB_PROJECT}")


# ---------------------------------------------------------------- 4. 打包
def pack(input_exe: Path, output_exe: Path) -> None:
    if not EVB_CONSOLE.exists():
        die(f"找不到 {EVB_CONSOLE}")
    output_exe.parent.mkdir(parents=True, exist_ok=True)
    if output_exe.exists():
        output_exe.unlink()

    write_evb(input_exe, output_exe)
    run([EVB_CONSOLE, EVB_PROJECT], cwd=EVB_CONSOLE.parent)

    if not output_exe.exists():
        die(f"EVB 执行完了但没生成 {output_exe}")
    size_mb = output_exe.stat().st_size / 1024 / 1024
    log(f"完成: {output_exe}  ({size_mb:.1f} MB)")


def main() -> None:
    parser = argparse.ArgumentParser(description="Qt 工程一键单文件打包")
    parser.add_argument("--debug", action="store_true", help="打 Debug 版（默认 Release）")
    parser.add_argument("--no-build", action="store_true", help="跳过编译，用现有产物")
    parser.add_argument("--no-config", action="store_true",
                        help="不把 config.ini 打进盒子，改为随 exe 单独放")
    args = parser.parse_args()

    build_type = "debug" if args.debug else "release"
    log(f"目标: {build_type}")

    if args.no_build:
        exe = find_existing_exe(build_type)
    else:
        exe = build(build_type)

    prepare_stage(exe, build_type, with_config=not args.no_config)
    pack(STAGE / f"{TARGET}.exe", DIST / f"{TARGET}.exe")


if __name__ == "__main__":
    main()
