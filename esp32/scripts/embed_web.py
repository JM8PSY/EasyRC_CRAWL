"""web/ 以下のファイルを gzip してファームウェアに埋め込む（ビルド前に自動実行）。

生成物: src/web_assets.h（git 管理外）
"""
import gzip
import os

Import("env")  # noqa: F821  (PlatformIO が提供)

MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
}

project = env["PROJECT_DIR"]  # noqa: F821
web_dir = os.path.join(project, "web")
out_path = os.path.join(project, "src", "web_assets.h")

lines = [
    "// 自動生成ファイル（scripts/embed_web.py）。編集しないでください",
    "#pragma once",
    "#include <cstddef>",
    "#include <cstdint>",
    "",
    "struct WebAsset { const char* path; const char* mime; const uint8_t* data; size_t length; };",
    "",
]
entries = []
for i, name in enumerate(sorted(os.listdir(web_dir))):
    ext = os.path.splitext(name)[1]
    if ext not in MIME:
        continue
    with open(os.path.join(web_dir, name), "rb") as f:
        data = gzip.compress(f.read(), compresslevel=9, mtime=0)
    lines.append(f"static const uint8_t WEB_ASSET_{i}[] = {{")
    for j in range(0, len(data), 20):
        lines.append("  " + ",".join(str(b) for b in data[j : j + 20]) + ",")
    lines.append("};")
    path = "/" if name == "index.html" else "/" + name
    entries.append(f'  {{"{path}", "{MIME[ext]}", WEB_ASSET_{i}, sizeof(WEB_ASSET_{i})}},')

lines.append("static const WebAsset WEB_ASSETS[] = {")
lines += entries
lines.append("};")
lines.append("static const size_t WEB_ASSET_COUNT = sizeof(WEB_ASSETS) / sizeof(WEB_ASSETS[0]);")
content = "\n".join(lines) + "\n"

old = None
if os.path.exists(out_path):
    with open(out_path, encoding="utf-8") as f:
        old = f.read()
if old != content:  # 内容が同じなら書き換えない（不要な再コンパイル防止）
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(content)
