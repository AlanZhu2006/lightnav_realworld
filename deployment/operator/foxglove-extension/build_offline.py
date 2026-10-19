#!/usr/bin/env python3
"""Build this dependency-free panel when Node is unavailable on the Jetson.

This deliberately supports only the explicit type annotations used by these
sources. It is not a TypeScript compiler or type checker. Source hashes are
recorded, generated JavaScript must parse in GJS, and existing model tests run
against the exact erased model before packaging. Prefer npm's normal build on
the development workstation.
"""
from __future__ import annotations

import hashlib
import json
import re
import subprocess
import tempfile
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def erase(source: str) -> str:
    source = re.sub(r"^import .*?;\n", "", source, flags=re.M)
    source = re.sub(r"^export type .*?;\n", "", source, flags=re.M)
    # Interfaces contain no executable expressions. Balance nested object types.
    while match := re.search(r"^export interface \w+\s*\{", source, re.M):
        level, end = 1, match.end()
        while level:
            if source[end] == "{":
                level += 1
            elif source[end] == "}":
                level -= 1
            end += 1
        source = source[:match.start()] + source[end:]
    exact = {
        "<K extends keyof HTMLElementTagNameMap>": "",
        ": K": "",
        ": HTMLElementTagNameMap[K]": "",
        ": {box:HTMLDivElement;body:HTMLDivElement;head:HTMLDivElement}": "",
        ": {box:HTMLDivElement;img:HTMLImageElement;area:HTMLDivElement;meta:HTMLSpanElement;footer:HTMLDivElement}": "",
        ": {snapshot?: boolean}": "",
        ": () => void": "",
        ":{bytes:Uint8Array;source:PreviewSource;receivedAt:number}|undefined": "",
        ": (Pending & {superseded:boolean}) | undefined": "",
        ":Record<string,[string,boolean]>": "",
        ":[string,string,boolean][]": "",
        ":{disconnected?:boolean}": "",
        ":[number,number]": "",
        ":typeof TOPICS.rgb": "",
        ":typeof camera": "",
        ":'rgb'|'goal'": "",
        " as const": "",
    }
    for before, after in exact.items():
        source = source.replace(before, after)
    source = re.sub(r":\s*Record<(?:string|Action),\s*string>", "", source)
    source = re.sub(r"(new (?:Map|Set))<[^>]+>", r"\1", source)
    source = re.sub(r":Promise<(?:void|HTMLImageElement)>", "", source)
    source = re.sub(r" as (?:JsonMap|number|Point\[\])\b", "", source)
    # Word boundary must precede [] in assertions.
    source = source.replace(" as Point[]", "")
    names = r"(?:unknown|void|number|string|boolean|JsonMap|Action|TaskMode|Point|PathView|PreviewSource|GoalSelection|Command|Pending|Uint8Array|HTMLElement|HTMLButtonElement|HTMLImageElement|HTMLCanvasElement|File|PanelExtensionContext|ExtensionContext)"
    annotation = rf"\??:\s*{names}(?:\[\])?(?:\s*\|\s*undefined)?(?=\s*[,)=;{{>])"
    source = re.sub(annotation, "", source)
    source = re.sub(r"\)!\.", ").", source)
    source = re.sub(r"\bexport ", "", source)
    return source


def run_gjs(script: str) -> None:
    with tempfile.NamedTemporaryFile(mode="w", suffix=".js", encoding="utf-8") as output:
        output.write(script)
        output.flush()
        subprocess.run(["gjs", output.name], check=True)


def build() -> Path:
    paths = [ROOT / "src" / f"{name}.ts" for name in ["model", "styles", "panel", "index"]]
    parts = {path.stem: erase(path.read_text()) for path in paths}
    bundle = '/* Built from src/*.ts by build_offline.py; see offline-build.json. */\n(() => {\n"use strict";\n' + "\n".join(parts.values()) + "\nmodule.exports={activate};\n})();\n"
    run_gjs("new Function(" + json.dumps(bundle) + "); print('Generated extension JavaScript parses.');")
    tests = (ROOT / "tests/model.test.mjs").read_text()
    tests = re.sub(r"^import .*?;\n", "", tests, flags=re.M)
    assertions = (ROOT / "tests/offline-assert.js").read_text()
    run_gjs(assertions + "\n" + parts["model"] + "\n" + tests + "\nprint('Model regression tests passed.');\n")
    panel_tests = (ROOT / "tests/offline-panel.js").read_text()
    run_gjs(assertions + "\n" + panel_tests + "\n" + bundle + "\nrunPanelTests(module.exports);\n")
    out = ROOT / "dist"
    out.mkdir(exist_ok=True)
    (out / "extension.js").write_text(bundle)
    metadata = {"builder": "build_offline.py", "typecheck": "not run: Node/npm/TypeScript unavailable", "validation": ["GJS JavaScript parse", "existing model tests", "native panel mock interaction tests"], "sources": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}, "extension_sha256": hashlib.sha256(bundle.encode()).hexdigest()}
    (out / "offline-build.json").write_text(json.dumps(metadata, indent=2) + "\n")
    package = json.loads((ROOT / "package.json").read_text())
    target = ROOT / f"{package['publisher']}.{package['name']}-{package['version']}.foxe"
    with zipfile.ZipFile(target, "w", zipfile.ZIP_DEFLATED) as archive:
        for name in ["package.json", "README.md", "CHANGELOG.md", "LICENSE", "dist/extension.js", "dist/offline-build.json"]:
            archive.write(ROOT / name, name)
    print(target)
    return target


if __name__ == "__main__":
    build()
