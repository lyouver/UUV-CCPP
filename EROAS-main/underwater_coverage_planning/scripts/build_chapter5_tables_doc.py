#!/usr/bin/env python3
"""Build chapter-5 table documents from generated CSV data."""

import csv
import os
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from xml.sax.saxutils import escape

import yaml


ROOT = Path("/home/tb/dave_ws")
OUTPUT_DIR = ROOT / "chapter5_outputs" / "deliverables"
TABLE_5_2_CSV = ROOT / "chapter5_outputs" / "revised_global" / "tables" / "table_5_2.csv"
TABLE_5_3_CSV = ROOT / "chapter5_outputs" / "revised_global" / "tables" / "table_5_3.csv"
TABLE_5_3_ITERS_CSV = ROOT / "chapter5_outputs" / "revised_global" / "tables" / "table_5_3_iterations.csv"
TABLE_5_4_CSV = ROOT / "chapter5_outputs" / "revised_global_batch" / "tables" / "table_5_4.csv"
LOCAL_MPC_CFG = ROOT / "src" / "EROAS-main" / "example" / "src" / "config" / "local_mpc_adapter.yaml"
DETECTOR_CFG = ROOT / "src" / "onboard_detector" / "cfg" / "detector_param.yaml"
PREDICTOR_CFG = ROOT / "src" / "dynamic_predictor" / "cfg" / "predictor_param.yaml"


def read_single_row_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f"No rows found in {path}")
    return rows[0]


def read_rows_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f"No rows found in {path}")
    return rows


def read_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def pct(value):
    return f"{float(value) * 100:.2f}%"


def meters(value):
    return f"{float(value):.3f} m"


def seconds(value):
    return f"{float(value):.3f} s"


def plain_num(value):
    if isinstance(value, int):
        return str(value)
    fv = float(value)
    if abs(fv - round(fv)) < 1e-9:
        return str(int(round(fv)))
    return f"{fv:.3f}"


def rtf_escape(text):
    out = []
    for ch in str(text):
        code = ord(ch)
        if ch == "\\":
            out.append(r"\\")
        elif ch == "{":
            out.append(r"\{")
        elif ch == "}":
            out.append(r"\}")
        elif ch == "\n":
            out.append(r"\line ")
        elif code > 127:
            if code > 32767:
                code -= 65536
            out.append(rf"\u{code}?")
        else:
            out.append(ch)
    return "".join(out)


def paragraph(text="", align="l", bold=False, font_size=24, space_before=0, space_after=120):
    align_map = {"l": r"\ql", "c": r"\qc", "r": r"\qr"}
    bold_tag = r"\b" if bold else ""
    bold_end = r"\b0" if bold else ""
    align_tag = align_map.get(align, r"\ql")
    return (
        rf"\pard{align_tag}\sb{space_before}\sa{space_after}\f1\fs{font_size}"
        rf"{bold_tag} {rtf_escape(text)}{bold_end}\par"
    )


def table_row(cells, widths, header=False, merged=False):
    parts = [r"\trowd\trgaph108\trleft0"]
    cumul = 0
    for idx, width in enumerate(widths):
        cumul += width
        cell_def = r"\clbrdrt\brdrs\brdrw10\clbrdrl\brdrs\brdrw10\clbrdrb\brdrs\brdrw10\clbrdrr\brdrs\brdrw10"
        if merged:
            if idx == 0:
                cell_def += r"\clmgf"
            else:
                cell_def += r"\clmrg"
        parts.append(cell_def + rf"\cellx{cumul}")
    line = "".join(parts)
    content = []
    for idx, cell in enumerate(cells):
        bold_tag = r"\b" if header or (merged and idx == 0) else ""
        bold_end = r"\b0" if header or (merged and idx == 0) else ""
        align = r"\qc" if header or merged else r"\ql"
        text = cell if (idx == 0 or not merged) else ""
        content.append(rf"\pard\intbl{align}\f1\fs22 {bold_tag}{rtf_escape(text)}{bold_end}\cell")
    return line + "".join(content) + r"\row"


def build_table(caption, headers, rows, widths):
    lines = [table_row([caption] + [""] * (len(headers) - 1), widths, merged=True)]
    lines.append(table_row(headers, widths, header=True))
    for row in rows:
        lines.append(table_row(row, widths))
    return "\n".join(lines)


def xml_run(text, bold=False, size=21):
    bold_xml = "<w:b/>" if bold else ""
    return (
        "<w:r>"
        "<w:rPr>"
        '<w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman" w:eastAsia="宋体"/>'
        f"{bold_xml}"
        f'<w:sz w:val="{size}"/><w:szCs w:val="{size}"/>'
        "</w:rPr>"
        f'<w:t xml:space="preserve">{escape(str(text))}</w:t>'
        "</w:r>"
    )


def xml_paragraph(text, align="left", bold=False, size=24):
    jc = {"left": "left", "center": "center", "right": "right"}[align]
    return (
        "<w:p>"
        f'<w:pPr><w:jc w:val="{jc}"/></w:pPr>'
        f"{xml_run(text, bold=bold, size=size)}"
        "</w:p>"
    )


def xml_table(caption, headers, rows, widths):
    total_width = sum(widths)
    borders = (
        "<w:tblBorders>"
        '<w:top w:val="single" w:sz="8" w:space="0" w:color="000000"/>'
        '<w:left w:val="single" w:sz="8" w:space="0" w:color="000000"/>'
        '<w:bottom w:val="single" w:sz="8" w:space="0" w:color="000000"/>'
        '<w:right w:val="single" w:sz="8" w:space="0" w:color="000000"/>'
        '<w:insideH w:val="single" w:sz="8" w:space="0" w:color="000000"/>'
        '<w:insideV w:val="single" w:sz="8" w:space="0" w:color="000000"/>'
        "</w:tblBorders>"
    )
    grid = "".join(f'<w:gridCol w:w="{w}"/>' for w in widths)
    out = [
        "<w:tbl>",
        (
            "<w:tblPr>"
            '<w:tblStyle w:val="TableGrid"/>'
            '<w:tblW w:w="0" w:type="auto"/>'
            f"{borders}"
            '<w:tblLayout w:type="fixed"/>'
            "</w:tblPr>"
        ),
        f"<w:tblGrid>{grid}</w:tblGrid>",
        (
            "<w:tr><w:tc><w:tcPr>"
            f'<w:tcW w:w="{total_width}" w:type="dxa"/>'
            f'<w:gridSpan w:val="{len(headers)}"/>'
            "</w:tcPr>"
            f"{xml_paragraph(caption, align='center', bold=False, size=21)}"
            "</w:tc></w:tr>"
        ),
    ]
    header_cells = []
    for head, width in zip(headers, widths):
        header_cells.append(
            "<w:tc><w:tcPr>"
            f'<w:tcW w:w="{width}" w:type="dxa"/>'
            "</w:tcPr>"
            f"{xml_paragraph(head, align='center', bold=False, size=21)}"
            "</w:tc>"
        )
    out.append("<w:tr>" + "".join(header_cells) + "</w:tr>")
    for row in rows:
        cells = []
        for idx, (cell, width) in enumerate(zip(row, widths)):
            align = "center" if idx == 1 and len(widths) <= 3 else "left"
            if len(widths) >= 5 and idx in (1, 2, 3, 4):
                align = "center"
            cells.append(
                "<w:tc><w:tcPr>"
                f'<w:tcW w:w="{width}" w:type="dxa"/>'
                "</w:tcPr>"
                f"{xml_paragraph(cell, align=align, size=21)}"
                "</w:tc>"
            )
        out.append("<w:tr>" + "".join(cells) + "</w:tr>")
    out.append("</w:tbl>")
    return "".join(out)


def build_docx(docx_path, title, tables, note):
    created = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    body = [xml_paragraph(title, align="center", bold=True, size=28)]
    body.append(xml_paragraph(" ", align="left", size=10))
    for caption, headers, rows, widths in tables:
        body.append(xml_table(caption, headers, rows, widths))
        body.append(xml_paragraph(" ", align="left", size=10))
    body.append(xml_paragraph(note, align="left", size=20))
    body.append(
        "<w:sectPr>"
        '<w:pgSz w:w="11906" w:h="16838"/>'
        '<w:pgMar w:top="1440" w:right="1440" w:bottom="1440" w:left="1440" w:header="720" w:footer="720" w:gutter="0"/>'
        "</w:sectPr>"
    )
    document_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<w:document xmlns:wpc="http://schemas.microsoft.com/office/word/2010/wordprocessingCanvas" '
        'xmlns:mc="http://schemas.openxmlformats.org/markup-compatibility/2006" '
        'xmlns:o="urn:schemas-microsoft-com:office:office" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" '
        'xmlns:m="http://schemas.openxmlformats.org/officeDocument/2006/math" '
        'xmlns:v="urn:schemas-microsoft-com:vml" '
        'xmlns:wp14="http://schemas.microsoft.com/office/word/2010/wordprocessingDrawing" '
        'xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing" '
        'xmlns:w10="urn:schemas-microsoft-com:office:word" '
        'xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main" '
        'xmlns:w14="http://schemas.microsoft.com/office/word/2010/wordml" '
        'xmlns:wpg="http://schemas.microsoft.com/office/word/2010/wordprocessingGroup" '
        'xmlns:wpi="http://schemas.microsoft.com/office/word/2010/wordprocessingInk" '
        'xmlns:wne="http://schemas.microsoft.com/office/word/2006/wordml" '
        'xmlns:wps="http://schemas.microsoft.com/office/word/2010/wordprocessingShape" '
        'mc:Ignorable="w14 wp14">'
        f"<w:body>{''.join(body)}</w:body></w:document>"
    )
    styles_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">'
        '<w:docDefaults><w:rPrDefault><w:rPr>'
        '<w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman" w:eastAsia="宋体"/>'
        '<w:sz w:val="21"/><w:szCs w:val="21"/>'
        '</w:rPr></w:rPrDefault></w:docDefaults>'
        '<w:style w:type="paragraph" w:default="1" w:styleId="Normal"><w:name w:val="Normal"/></w:style>'
        '<w:style w:type="table" w:styleId="TableGrid"><w:name w:val="Table Grid"/></w:style>'
        "</w:styles>"
    )
    content_types_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
        '<Default Extension="xml" ContentType="application/xml"/>'
        '<Override PartName="/word/document.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>'
        '<Override PartName="/word/styles.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>'
        '<Override PartName="/docProps/core.xml" '
        'ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>'
        '<Override PartName="/docProps/app.xml" '
        'ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>'
        "</Types>"
    )
    rels_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" '
        'Target="word/document.xml"/>'
        '<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" '
        'Target="docProps/core.xml"/>'
        '<Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" '
        'Target="docProps/app.xml"/>'
        "</Relationships>"
    )
    document_rels_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"></Relationships>'
    )
    core_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties" '
        'xmlns:dc="http://purl.org/dc/elements/1.1/" '
        'xmlns:dcterms="http://purl.org/dc/terms/" '
        'xmlns:dcmitype="http://purl.org/dc/dcmitype/" '
        'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">'
        '<dc:title>第五章表格数据汇总</dc:title>'
        '<dc:creator>Codex</dc:creator>'
        '<cp:lastModifiedBy>Codex</cp:lastModifiedBy>'
        f'<dcterms:created xsi:type="dcterms:W3CDTF">{created}</dcterms:created>'
        f'<dcterms:modified xsi:type="dcterms:W3CDTF">{created}</dcterms:modified>'
        "</cp:coreProperties>"
    )
    app_xml = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties" '
        'xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">'
        '<Application>Codex</Application><DocSecurity>0</DocSecurity><ScaleCrop>false</ScaleCrop>'
        "</Properties>"
    )
    with zipfile.ZipFile(docx_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("[Content_Types].xml", content_types_xml)
        zf.writestr("_rels/.rels", rels_xml)
        zf.writestr("word/document.xml", document_xml)
        zf.writestr("word/styles.xml", styles_xml)
        zf.writestr("word/_rels/document.xml.rels", document_rels_xml)
        zf.writestr("docProps/core.xml", core_xml)
        zf.writestr("docProps/app.xml", app_xml)


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    table_5_2 = read_single_row_csv(TABLE_5_2_CSV)
    table_5_3 = read_rows_csv(TABLE_5_3_CSV)
    table_5_3_iters = read_rows_csv(TABLE_5_3_ITERS_CSV)
    table_5_4 = read_rows_csv(TABLE_5_4_CSV)
    local_mpc = read_yaml(LOCAL_MPC_CFG)
    detector = read_yaml(DETECTOR_CFG)
    predictor = read_yaml(PREDICTOR_CFG)

    rows_5_2 = [
        ["候选视点数量", plain_num(table_5_2["candidate_viewpoints"]), "SIP采样后得到的候选覆盖视点总数"],
        ["最终航点数量", plain_num(table_5_2["final_waypoints"]), "优化后导出的全局覆盖航点数量"],
        ["有效覆盖单元数", plain_num(table_5_2["covered_cells"]), "至少被一个视点有效覆盖的地形单元数量"],
        ["覆盖率", pct(table_5_2["coverage_rate"]), "有效覆盖单元数占目标覆盖单元总数的比例"],
        ["开放路径长度", meters(table_5_2["open_path_length_m"]), "按航点顺序累加得到的执行路径长度"],
        ["闭合路径长度", meters(table_5_2["closed_path_length_m"]), "考虑首尾闭合后的路径长度"],
        ["平均航段长度", meters(table_5_2["mean_segment_length_m"]), "相邻航点间平均距离"],
        ["最大航段长度", meters(table_5_2["max_segment_length_m"]), "用于识别是否存在异常跨区连接"],
        ["路径生成总时间", seconds(table_5_2["generation_time_sec"]), "从视点采样到航点导出的总耗时"],
    ]

    rows_5_3 = []
    for row in table_5_3:
        improve = "基准" if row["stage"] == "initial_order" else pct(row["relative_improvement"])
        rows_5_3.append(
            [
                row["planning_stage"],
                meters(row["path_length_m"]),
                plain_num(row["tour_cost"]),
                seconds(row["compute_time_sec"]),
                improve,
            ]
        )

    rows_5_4 = []
    for row in table_5_4:
        rows_5_4.append(
            [
                row["comparison"],
                pct(row["coverage_rate"]),
                meters(row["path_length_m"]),
                meters(row["max_segment_length_m"]),
                seconds(row["planning_time_sec"]),
                row["conclusion_role"],
            ]
        )

    rows_5_3_iters = []
    for row in table_5_3_iters:
        rows_5_3_iters.append(
            [
                f"第{int(row['iteration'])}轮",
                meters(row["path_length_m"]),
                seconds(row["compute_time_sec"]),
                pct(row["relative_improvement_vs_iter1"]),
                f"{int(row['feasible_edges'])}/{int(row['total_edges'])}",
            ]
        )

    rows_5_5 = [
        ["深度观测范围下限", "depth_min_value", f"{detector['depth_min_value']} m"],
        ["深度观测范围上限", "depth_max_value", f"{detector['depth_max_value']:.0f} m"],
        ["图像列数", "image_cols", str(detector["image_cols"])],
        ["图像行数", "image_rows", str(detector["image_rows"])],
        ["DBSCAN最小点数", "dbscan_min_points_cluster", str(detector["dbscan_min_points_cluster"])],
        ["DBSCAN搜索半径", "dbscan_search_range_epsilon", f"{detector['dbscan_search_range_epsilon']} m"],
        ["动态速度阈值", "dynamic_velocity_threshold", f"{detector['dynamic_velocity_threshold']} m/s"],
        ["预测步数", "prediction_size", str(predictor["prediction_size"])],
        ["预测时间间隔", "prediction_time_step", f"{predictor['prediction_time_step']} s"],
        ["MPC规划时域", "horizon", str(local_mpc["mpc_planner/horizon"])],
        ["静态安全距离", "static_safety_dist", f"{local_mpc['mpc_planner/static_safety_dist']:.0f} m"],
        ["动态安全距离", "dynamic_safety_dist", f"{local_mpc['mpc_planner/dynamic_safety_dist']:.1f} m"],
    ]

    rtf = []
    rtf.append(r"{\rtf1\ansi\deff0")
    rtf.append(r"{\fonttbl{\f0 Times New Roman;}{\f1 SimSun;}}")
    rtf.append(r"\viewkind4\uc1")
    rtf.append(paragraph("第5章表格数据汇总", align="c", bold=True, font_size=28, space_after=220))
    rtf.append(paragraph("依据原论文中的表5.2至表5.5结构整理，已将当前实验输出和配置参数填入。", align="l", font_size=22))
    rtf.append(build_table("表5.2 全局覆盖路径生成结果统计", ["统计项", "结果", "说明"], rows_5_2, [2200, 2200, 4600]))
    rtf.append(paragraph(space_after=80))
    rtf.append(build_table("表5.3 LKH与2-opt优化效果对比", ["规划阶段", "路径长度", "路径代价", "计算时间", "相对改善率"], rows_5_3, [2500, 1800, 1800, 1400, 1500]))
    rtf.append(paragraph(space_after=80))
    rtf.append(build_table("表5.4 全局规划消融实验结果对比", ["对比方案", "覆盖率", "路径长度", "最大航段长度", "规划时间", "结论作用"], rows_5_4, [2200, 1200, 1600, 1700, 1400, 1900]))
    rtf.append(paragraph(space_after=80))
    rtf.append(build_table("表5.5 局部感知、预测与规划关键参数表", ["参数含义", "代码参数名", "取值"], rows_5_5, [2400, 3000, 3600]))
    rtf.append(paragraph("数据来源：table_5_2.csv、table_5_3.csv、table_5_4.csv 以及检测器/预测器/MPC 参数配置文件。", align="l", font_size=20, space_before=120, space_after=0))
    rtf.append("}")

    title = "第5章表格数据汇总"
    note = "数据来源：table_5_2.csv、table_5_3.csv、table_5_4.csv 以及检测器、预测器、MPC 参数配置文件。"
    tables = [
        ("表5.2 全局覆盖路径生成结果统计", ["统计项", "结果", "说明"], rows_5_2, [2200, 2200, 4600]),
        ("表5.3 LKH与2-opt优化效果对比", ["规划阶段", "路径长度", "路径代价", "计算时间", "相对改善率"], rows_5_3, [2500, 1800, 1800, 1400, 1500]),
        ("表5.3-补充 迭代重采样各轮结果", ["迭代轮次", "开放路径长度", "轮次耗时", "相对第1轮改善率", "可行边数"], rows_5_3_iters, [1800, 2200, 1800, 2200, 1600]),
        ("表5.4 全局规划消融实验结果对比", ["对比方案", "覆盖率", "路径长度", "最大航段长度", "规划时间", "结论作用"], rows_5_4, [2200, 1200, 1600, 1700, 1400, 1900]),
        ("表5.5 局部感知、预测与规划关键参数表", ["参数含义", "代码参数名", "取值"], rows_5_5, [2400, 3000, 3600]),
    ]

    rtf_path = OUTPUT_DIR / "第五章表格数据汇总.rtf"
    docx_path = OUTPUT_DIR / "第五章表格数据汇总.docx"
    rtf_path.write_text("\n".join(rtf), encoding="utf-8")
    build_docx(docx_path, title, tables, note)
    if not docx_path.exists():
        raise RuntimeError(f"Expected output not found: {docx_path}")
    print(rtf_path)
    print(docx_path)


if __name__ == "__main__":
    main()
