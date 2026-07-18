#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Excel(.xlsx) -> CSV 转换工具，无需 pandas/openpyxl 等第三方库。

为什么需要这个脚本？
  ROS/C++ 直接读取 .xlsx 文件需要较重的依赖；为了让训练节点保持轻量、稳定，
  本包采用“Excel 先转 CSV，C++ 训练节点读取 CSV”的方式。

重要特性：
  1. 保留 Excel 中的空列位置，例如 A、C、D、F 等列不会被压缩；
     因此 YAML 配置中的列号仍然按 Excel 原始列号 0-based 计算。
  2. 支持共享字符串 sharedStrings、普通数字、inlineStr。
  3. 默认读取第一个 sheet，也可以通过第三个参数指定 sheet 名称或 0-based sheet 索引。

用法：
  python3 scripts/xlsx_to_csv.py input.xlsx output.csv
  python3 scripts/xlsx_to_csv.py input.xlsx output.csv Sheet1
  python3 scripts/xlsx_to_csv.py input.xlsx output.csv 0
"""

import csv
import re
import sys
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path

NS = {
    "a": "http://schemas.openxmlformats.org/spreadsheetml/2006/main",
    "r": "http://schemas.openxmlformats.org/officeDocument/2006/relationships",
}


def col_letters_to_index(letters: str) -> int:
    """Excel 列字母转 0-based 列号：A->0, B->1, C->2。"""
    idx = 0
    for ch in letters:
        idx = idx * 26 + (ord(ch.upper()) - ord('A') + 1)
    return idx - 1


def cell_ref_to_rc(ref: str):
    """单元格引用转 0-based 行列号，例如 C4 -> (3,2)。"""
    m = re.match(r"([A-Z]+)([0-9]+)", ref)
    if not m:
        return None, None
    col = col_letters_to_index(m.group(1))
    row = int(m.group(2)) - 1
    return row, col


def read_shared_strings(zf):
    """读取 sharedStrings.xml，返回字符串表。"""
    if "xl/sharedStrings.xml" not in zf.namelist():
        return []
    root = ET.fromstring(zf.read("xl/sharedStrings.xml"))
    strings = []
    for si in root.findall("a:si", NS):
        # 一个字符串可能由多个 <t> 组成，例如富文本。
        text = "".join((t.text or "") for t in si.findall(".//a:t", NS))
        strings.append(text)
    return strings


def get_sheet_targets(zf):
    """返回 [(sheet_name, xml_path), ...]。"""
    wb_root = ET.fromstring(zf.read("xl/workbook.xml"))
    rel_root = ET.fromstring(zf.read("xl/_rels/workbook.xml.rels"))
    rels = {rel.attrib["Id"]: rel.attrib["Target"] for rel in rel_root}

    sheets = []
    for sh in wb_root.findall("a:sheets/a:sheet", NS):
        name = sh.attrib["name"]
        rid = sh.attrib["{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id"]
        target = rels[rid].lstrip("/")
        if not target.startswith("xl/"):
            target = "xl/" + target
        sheets.append((name, target))
    return sheets


def choose_sheet(sheets, selector=None):
    if selector is None:
        return sheets[0]
    # 如果 selector 是数字，按 0-based sheet index 选择。
    if re.fullmatch(r"\d+", selector):
        idx = int(selector)
        if idx < 0 or idx >= len(sheets):
            raise RuntimeError(f"sheet index out of range: {idx}")
        return sheets[idx]
    # 否则按 sheet 名称选择。
    for name, target in sheets:
        if name == selector:
            return name, target
    raise RuntimeError(f"cannot find sheet: {selector}; available sheets: {[s[0] for s in sheets]}")


def cell_value(cell, shared_strings):
    """提取单元格显示值。"""
    cell_type = cell.attrib.get("t", "")
    if cell_type == "inlineStr":
        t = cell.find(".//a:t", NS)
        return t.text if t is not None and t.text is not None else ""

    v = cell.find("a:v", NS)
    if v is None or v.text is None:
        return ""
    raw = v.text
    if cell_type == "s":
        try:
            return shared_strings[int(raw)]
        except Exception:
            return ""
    return raw


def xlsx_to_rows(xlsx_path, sheet_selector=None):
    with zipfile.ZipFile(xlsx_path) as zf:
        shared = read_shared_strings(zf)
        sheets = get_sheet_targets(zf)
        sheet_name, sheet_xml = choose_sheet(sheets, sheet_selector)
        root = ET.fromstring(zf.read(sheet_xml))

        rows_dict = {}
        max_row = -1
        max_col = -1
        for row in root.findall("a:sheetData/a:row", NS):
            for cell in row.findall("a:c", NS):
                ref = cell.attrib.get("r", "")
                r, c = cell_ref_to_rc(ref)
                if r is None:
                    continue
                val = cell_value(cell, shared)
                rows_dict[(r, c)] = val
                max_row = max(max_row, r)
                max_col = max(max_col, c)

        table = []
        for r in range(max_row + 1):
            table.append([rows_dict.get((r, c), "") for c in range(max_col + 1)])
        return sheet_name, table


def main():
    if len(sys.argv) not in (3, 4):
        print("Usage: python3 xlsx_to_csv.py input.xlsx output.csv [sheet_name_or_index]")
        sys.exit(1)
    inp = Path(sys.argv[1]).expanduser()
    out = Path(sys.argv[2]).expanduser()
    selector = sys.argv[3] if len(sys.argv) == 4 else None

    sheet_name, rows = xlsx_to_rows(str(inp), selector)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerows(rows)
    print(f"Saved CSV: {out}")
    print(f"Sheet: {sheet_name}, rows={len(rows)}, cols={len(rows[0]) if rows else 0}")
    print("Column index is 0-based and blank columns are preserved.")


if __name__ == "__main__":
    main()
