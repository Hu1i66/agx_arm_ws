#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple Markdown to PDF renderer using reportlab.
Supports basic headings, paragraphs, bullet lists and fenced code blocks.
"""
import os
import sys
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Preformatted
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfbase import pdfmetrics

# Prefer system Noto Sans CJK if available
CANDIDATE_FONTS = [
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc',
    '/usr/share/fonts/truetype/arphic/ukai.ttc',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
]
FONT_PATH = None
for p in CANDIDATE_FONTS:
    if os.path.exists(p):
        FONT_PATH = p
        break

if FONT_PATH is None:
    print('未找到可用字体，请在系统中安装中文字体（如 Noto Sans CJK）')
    sys.exit(1)

FONT_NAME = 'CJKFONT'
pdfmetrics.registerFont(TTFont(FONT_NAME, FONT_PATH))

styles = getSampleStyleSheet()
styles.add(ParagraphStyle(name='NormalCJK', parent=styles['Normal'], fontName=FONT_NAME, fontSize=11, leading=14))
styles.add(ParagraphStyle(name='Heading1CJK', parent=styles['Heading1'], fontName=FONT_NAME, fontSize=18, leading=22))
styles.add(ParagraphStyle(name='Heading2CJK', parent=styles['Heading2'], fontName=FONT_NAME, fontSize=14, leading=18))
styles.add(ParagraphStyle(name='Code', parent=styles['Code'], fontName='Courier', fontSize=9, leading=12))


def md_to_flowables(md_text):
    lines = md_text.splitlines()
    flowables = []
    para_lines = []
    in_code = False
    code_lines = []
    for line in lines:
        if line.startswith('```'):
            if not in_code:
                in_code = True
                code_lines = []
            else:
                in_code = False
                code_text = '\n'.join(code_lines)
                flowables.append(Preformatted(code_text, styles['Code']))
            continue
        if in_code:
            code_lines.append(line)
            continue
        if line.strip() == '':
            if para_lines:
                para_text = ' '.join(para_lines)
                flowables.append(Paragraph(para_text, styles['NormalCJK']))
                para_lines = []
            flowables.append(Spacer(1,6))
            continue
        if line.startswith('# '):
            if para_lines:
                para_text = ' '.join(para_lines)
                flowables.append(Paragraph(para_text, styles['NormalCJK']))
                para_lines = []
            txt = line[2:].strip()
            flowables.append(Paragraph(txt, styles['Heading1CJK']))
            continue
        if line.startswith('## '):
            if para_lines:
                para_text = ' '.join(para_lines)
                flowables.append(Paragraph(para_text, styles['NormalCJK']))
                para_lines = []
            txt = line[3:].strip()
            flowables.append(Paragraph(txt, styles['Heading2CJK']))
            continue
        if line.lstrip().startswith('- '):
            if para_lines:
                para_text = ' '.join(para_lines)
                flowables.append(Paragraph(para_text, styles['NormalCJK']))
                para_lines = []
            bullet = line.lstrip()[2:].strip()
            flowables.append(Paragraph('• ' + bullet, styles['NormalCJK']))
            continue
        para_lines.append(line.strip())
    if para_lines:
        para_text = ' '.join(para_lines)
        flowables.append(Paragraph(para_text, styles['NormalCJK']))
    return flowables


def main():
    md_path = 'robot_kinematics.md'
    pdf_path = 'robot_kinematics.pdf'
    if len(sys.argv) > 1:
        md_path = sys.argv[1]
    if len(sys.argv) > 2:
        pdf_path = sys.argv[2]
    if not os.path.exists(md_path):
        print('Markdown file not found:', md_path)
        sys.exit(1)
    with open(md_path, 'r', encoding='utf-8') as f:
        md_text = f.read()
    doc = SimpleDocTemplate(pdf_path, pagesize=A4, leftMargin=20*mm, rightMargin=20*mm, topMargin=20*mm, bottomMargin=20*mm)
    flowables = md_to_flowables(md_text)
    doc.build(flowables)
    print('Generated', pdf_path)

if __name__ == '__main__':
    main()
