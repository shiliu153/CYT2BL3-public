#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""发布前凭据自检 / Pre-publication credential scan.

扫描范围：指定 git 仓库的**全部对象**（含历史版本）与工作区文件，
查找 WiFi 口令、SSID、API key、私钥、邮箱、手机号等可能被误提交的凭据。

用法 / Usage:
    python tools/scan_secrets.py            # 扫描 HEAD 树 + 工作区
    python tools/scan_secrets.py --all      # 扫描全部 git 对象（更慢，覆盖历史）
    python tools/scan_secrets.py --root DIR # 指定仓库路径

退出码 / Exit codes:
    0  未命中
    1  有命中（详见输出）
    2  执行错误（不在 git 仓库内、git 不可用等）

注意 / Note:
    禁止清单中的字面量以**十六进制**存储，而不是直接写在源码里。
    否则脚本会把自己扫出来，形成永远无法通过的自检。
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys

# ---------------------------------------------------------------------------
# 禁止清单：额外需要拦截的字面量。
# 一律用十六进制表示，避免脚本匹配到自身。
# 需要新增时：python -c "print('你的字符串'.encode().hex())"
# ---------------------------------------------------------------------------
_FORBIDDEN_HEX = [
    # 这里放需要硬拦截的字面量的十六进制。示例格式（注释里也不能写明文）：
    #   "<明文的十六进制>"，
    "6578616d706c655f70617373776f7264",   # 占位示例，可按需替换或删除
]

FORBIDDEN = [bytes.fromhex(h) for h in _FORBIDDEN_HEX]

# ---------------------------------------------------------------------------
# 模式清单
# ---------------------------------------------------------------------------
PATTERNS = [
    ("WiFi 口令赋值",
     re.compile(rb"(?i)wifi[a-z_]*\s*(password|passwd|pwd|key)\s*[:=]\s*[\"']?[^\s\"';,]{4,}")),
    ("SSID 赋值",
     re.compile(rb"(?i)\bssid\s*[:=]\s*[\"']?[A-Za-z0-9_\-\.]{3,}")),
    ("口令赋值",
     re.compile(rb"(?i)\b(password|passwd)\s*[:=]\s*[\"'][^\"'\s]{4,}[\"']")),
    ("API key / token",
     re.compile(rb"(?i)\b(api[_-]?key|secret[_-]?key|access[_-]?token|auth[_-]?token)"
                rb"\s*[:=]\s*[\"']?[A-Za-z0-9_\-]{12,}")),
    ("私钥块",
     re.compile(rb"-----BEGIN [A-Z ]*PRIVATE KEY-----")),
    ("中国手机号",
     re.compile(rb"\b1[3-9]\d{9}\b")),
    ("邮箱地址",
     re.compile(rb"[A-Za-z0-9._%+\-]+@[A-Za-z0-9.\-]+\.[A-Za-z]{2,}")),
]

# 允许出现的例外
ALLOWED_EMAIL_PARTS = (
    b"@example", b"@gnu.org", b"@apache.org", b"@arm.com", b"@freertos.org",
    b"@cypress.com", b"@infineon.com", b"@seekfree", b"@version", b"@file",
    b"@brief", b"@param", b"@return", b"@defgroup", b"@note", b"@see",
    b"@addtogroup", b"@details", b"@{\\", b"@}",
)

MAX_BLOB_BYTES = 8 * 1024 * 1024
SKIP_DIRS = {".git", "__pycache__", "node_modules"}


def _run(args, **kw):
    return subprocess.run(args, capture_output=True, check=True, **kw)


def is_git_repo(root: str) -> bool:
    try:
        out = _run(["git", "-C", root, "rev-parse", "--is-inside-work-tree"]).stdout
        return out.strip() == b"true"
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False


def git_objects(root: str):
    """遍历全部 blob 对象（含历史）。"""
    listing = _run(["git", "-C", root, "cat-file", "--batch-all-objects",
                    "--batch-check=%(objectname) %(objecttype) %(objectsize)"]).stdout
    for line in listing.decode("utf-8", "replace").splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[1] == "blob":
            yield parts[0], int(parts[2])


def head_blobs(root: str):
    """只遍历 HEAD 树中的 blob（快得多）。"""
    listing = _run(["git", "-C", root, "ls-tree", "-r", "-z", "HEAD"]).stdout
    for entry in listing.split(b"\0"):
        if not entry:
            continue
        meta, _, path = entry.partition(b"\t")
        fields = meta.split()
        if len(fields) >= 3 and fields[1] == b"blob":
            yield fields[2].decode(), path.decode("utf-8", "replace")


def check(label: str, data: bytes, extra: tuple = ()):
    """返回 [(模式名, 匹配文本), ...]"""
    hits = []
    for name, pat in PATTERNS:
        for m in pat.finditer(data):
            text = m.group(0)
            if name == "邮箱地址" and any(s in text.lower() for s in ALLOWED_EMAIL_PARTS):
                continue
            hits.append((name, text))
    for literal in FORBIDDEN:
        if literal in data:
            hits.append(("禁止字面量", literal))
    return hits


def scan_blob(root: str, sha: str, label: str, report: dict):
    data = _run(["git", "-C", root, "cat-file", "blob", sha]).stdout
    for name, text in check(label, data):
        report.setdefault(name, []).append((label, text[:120]))


def scan_worktree(root: str, report: dict, files_only=None):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in filenames:
            path = os.path.join(dirpath, fn)
            rel = os.path.relpath(path, root)
            if files_only is not None and rel not in files_only:
                continue
            try:
                if os.path.getsize(path) > MAX_BLOB_BYTES:
                    continue
                with open(path, "rb") as fh:
                    data = fh.read()
            except OSError:
                continue
            for name, text in check(rel, data):
                report.setdefault(name, []).append((rel, text[:120]))


def main() -> int:
    ap = argparse.ArgumentParser(description="发布前凭据自检")
    ap.add_argument("--root", default=".", help="仓库路径（默认当前目录）")
    ap.add_argument("--all", action="store_true",
                    help="扫描全部 git 对象（含历史），默认只扫 HEAD 树 + 工作区")
    ap.add_argument("--quiet", action="store_true", help="只输出结论")
    args = ap.parse_args()

    root = os.path.abspath(args.root)
    if not is_git_repo(root):
        print("错误：%s 不是 git 仓库" % root, file=sys.stderr)
        return 2

    report: dict = {}
    scanned = 0

    try:
        if args.all:
            for sha, size in git_objects(root):
                if size > MAX_BLOB_BYTES:
                    continue
                scan_blob(root, sha, "obj:%s" % sha[:10], report)
                scanned += 1
        else:
            try:
                for sha, path in head_blobs(root):
                    scan_blob(root, sha, "HEAD:%s" % path, report)
                    scanned += 1
            except subprocess.CalledProcessError:
                # 尚无提交（unborn HEAD）——只扫工作区
                if not args.quiet:
                    print("提示：仓库尚无提交，跳过 HEAD 扫描，只检查工作区")
        scan_worktree(root, report)
    except subprocess.CalledProcessError as exc:
        print("错误：git 命令失败：%s" % exc, file=sys.stderr)
        return 2

    if not args.quiet:
        print("扫描对象：%d 个 blob + 工作区文件" % scanned)

    if not report:
        print("结果：未发现疑似凭据")
        return 0

    print("结果：发现 %d 类疑似凭据" % len(report))
    for name in sorted(report):
        uniq = sorted(set(report[name]))
        print("\n=== %s : %d 处 ===" % (name, len(uniq)))
        for where, text in uniq[:40]:
            print("   %-46s %s" % (where, text.decode("utf-8", "replace")))
        if len(uniq) > 40:
            print("   ... 另有 %d 处" % (len(uniq) - 40))
    print("\n若确有凭据被提交：请先**吊销并更换**该凭据，再清理仓库历史。"
          "\n仅删除文件是不够的——已推送的内容可能已被缓存或抓取。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
