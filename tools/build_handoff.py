#!/usr/bin/env python3
"""Build a portable source handoff and an offline guide using only stdlib.

The tiny renderer supports the headings, fenced code, lists, links and tables
used in these two project documents. This is not a general Markdown renderer.
"""
import hashlib
import html
from pathlib import Path
import re
from zipfile import ZipFile, ZIP_DEFLATED

ROOT = Path(__file__).resolve().parent.parent
PACKAGE = 'LumbarLab_Hardware_Handoff_2026-10-24'
DOCS = [('HANDOFF.md', 'handoff', '01 · 接手与分工'),
        ('docs/HARDWARE_INTEGRATION.md', 'implementation', '02 · 接入与自动驱动')]


def link_target(target, source):
    if target.startswith(('https://', 'http://', '#', 'mailto:')):
        return target
    path, separator, fragment = target.partition('#')
    resolved = (ROOT / source).parent.joinpath(path).resolve()
    if not resolved.is_relative_to(ROOT):
        return '#'
    relative = resolved.relative_to(ROOT).as_posix()
    for doc, anchor, _ in DOCS:
        if relative == doc:
            return '#' + anchor
    return relative + (separator + fragment if separator else '')


def inline(value, source):
    pattern = r'`([^`]+)`|\[([^\]]+)\]\(([^)]+)\)|\*\*([^*]+)\*\*|<(https?://[^>]+)>'
    result = []
    start = 0
    for match in re.finditer(pattern, value):
        result.append(html.escape(value[start:match.start()]))
        code, label, href, bold, bare_url = match.groups()
        if code is not None:
            result.append('<code>' + html.escape(code) + '</code>')
        elif href is not None:
            result.append('<a href="' + html.escape(link_target(href, source), quote=True) + '">' + html.escape(label) + '</a>')
        elif bold is not None:
            result.append('<strong>' + inline(bold, source) + '</strong>')
        else:
            result.append('<a href="' + html.escape(bare_url, quote=True) + '">' + html.escape(bare_url) + '</a>')
        start = match.end()
    result.append(html.escape(value[start:]))
    return ''.join(result)


def table_cells(line):
    cells, current, in_code = [], [], False
    for char in line.strip().strip('|'):
        if char == '`':
            in_code = not in_code
        if char == '|' and not in_code:
            cells.append(''.join(current).strip())
            current = []
        else:
            current.append(char)
    cells.append(''.join(current).strip())
    return cells


def render(source, anchor):
    lines = (ROOT / source).read_text(encoding='utf-8-sig').splitlines()
    blocks, toc = [], []
    index = 0
    heading_count = 0
    while index < len(lines):
        line = lines[index].strip()
        if not line:
            index += 1
            continue
        if line.startswith('```'):
            language = line[3:].strip()
            code = []
            index += 1
            while index < len(lines) and not lines[index].strip().startswith('```'):
                code.append(lines[index])
                index += 1
            if index == len(lines):
                raise ValueError(f'Unclosed code block in {source}')
            blocks.append('<div class="code-label">' + html.escape(language or 'text') + '</div><pre><code>' + html.escape('\n'.join(code)) + '</code></pre>')
            index += 1
            continue
        heading = re.match(r'^(#{1,6})\s+(.+)$', line)
        if heading:
            heading_count += 1
            level = len(heading.group(1))
            target = f'{anchor}-{heading_count}'
            title = heading.group(2)
            blocks.append(f'<h{min(level+1,6)} id="{target}">' + inline(title, source) + f'</h{min(level+1,6)}>')
            if level == 2:
                toc.append(f'<a href="#{target}">' + html.escape(title) + '</a>')
            index += 1
            continue
        if line.startswith('|') and index+1 < len(lines) and re.fullmatch(r'[\s|:\-]+', lines[index+1]):
            rows = [table_cells(line)]
            index += 2
            while index < len(lines) and lines[index].strip().startswith('|'):
                rows.append(table_cells(lines[index]))
                index += 1
            content = ['<div class="table-wrap"><table><thead><tr>']
            content += ['<th>' + inline(cell, source) + '</th>' for cell in rows[0]]
            content.append('</tr></thead><tbody>')
            for row in rows[1:]:
                if len(row) != len(rows[0]):
                    raise ValueError(f'Unequal table columns in {source}: {row}')
                content.append('<tr>' + ''.join('<td>'+inline(cell,source)+'</td>' for cell in row) + '</tr>')
            content.append('</tbody></table></div>')
            blocks.append(''.join(content))
            continue
        list_item = re.match(r'^([-*]|\d+[.)])\s+(.+)$', line)
        if list_item:
            ordered = list_item.group(1)[0].isdigit()
            tag = 'ol' if ordered else 'ul'
            items = []
            while index < len(lines):
                item = re.match(r'^([-*]|\d+[.)])\s+(.+)$', lines[index].strip())
                if not item or item.group(1)[0].isdigit() != ordered:
                    break
                items.append('<li>'+inline(item.group(2),source)+'</li>')
                index += 1
            blocks.append(f'<{tag}>' + ''.join(items) + f'</{tag}>')
            continue
        if line.startswith('>'):
            blocks.append('<aside>'+inline(line[1:].strip(), source)+'</aside>')
            index += 1
            continue
        if line in ('---', '***'):
            blocks.append('<hr>')
            index += 1
            continue
        paragraph = [line]
        index += 1
        while index < len(lines) and lines[index].strip() and not re.match(r'^(#{1,6}\s|```|\||>|[-*]\s|\d+[.)]\s)', lines[index].strip()):
            paragraph.append(lines[index].strip())
            index += 1
        blocks.append('<p>'+inline(' '.join(paragraph),source)+'</p>')
    return ''.join(blocks), ''.join(toc)


def build_html():
    sections, navigation = [], []
    for source, anchor, title in DOCS:
        content, toc = render(source, anchor)
        sections.append(f'<section id="{anchor}"><div class="section-label">{title}</div>{content}</section>')
        navigation.append(f'<details open><summary><a href="#{anchor}">{title}</a></summary>{toc}</details>')
    css = '''
    :root{color-scheme:light;--ink:#233833;--muted:#5f746b;--line:#dce5dd;--accent:#205d4b}
    *{box-sizing:border-box}html{scroll-behavior:smooth;scroll-padding-top:24px}body{margin:0;background:#f5f6f1;color:var(--ink);font:16px/1.8 system-ui,-apple-system,"Segoe UI","Microsoft YaHei",sans-serif}
    header{background:#193d33;color:#f8faf6;padding:36px max(28px,calc((100vw - 1360px)/2));border-bottom:5px solid #aec6a2}header small{letter-spacing:.15em;color:#c5d6c4}header h1{font-size:32px;margin:10px 0}header p{color:#dce7d8;margin:0;max-width:900px}.badge{display:inline-block;border:1px solid #7f9d8b;border-radius:20px;padding:3px 12px;font-size:13px;margin-top:15px}
    .layout{max-width:1400px;margin:30px auto;display:grid;grid-template-columns:264px minmax(0,1fr);gap:28px;padding:0 24px}nav{position:sticky;top:18px;align-self:start;max-height:92vh;overflow:auto;font-size:13px;padding-right:12px}nav a{display:block;padding:5px 8px;text-decoration:none;color:var(--muted)}nav summary a{display:inline;color:var(--accent);font-weight:700}nav details{margin-bottom:18px}main{min-width:0}section{background:white;border:1px solid var(--line);border-radius:14px;padding:30px 36px;margin-bottom:30px;box-shadow:0 6px 24px #163c2505}.section-label{color:var(--accent);font-weight:700;letter-spacing:.07em;border-bottom:1px solid var(--line);padding-bottom:15px;margin-bottom:18px}
    h2{font-size:27px;line-height:1.4;margin:20px 0}h3{font-size:22px;border-top:1px solid var(--line);padding-top:26px;margin-top:34px;line-height:1.5}h4{font-size:18px;margin:26px 0 10px}p{margin:13px 0}a{color:var(--accent);text-underline-offset:3px;overflow-wrap:anywhere}code{font-family:Consolas,"SFMono-Regular",monospace;font-size:.9em;background:#f0f4ee;padding:2px 5px;border-radius:4px;overflow-wrap:anywhere}pre{white-space:pre;overflow:auto;background:#1c302b;color:#ecf1e7;padding:18px;border-radius:0 0 8px 8px;margin:0 0 20px;font-size:13px;line-height:1.65}pre code{background:none;padding:0;color:inherit;overflow-wrap:normal;font-size:inherit}.code-label{background:#e9efe6;color:#496152;padding:4px 13px;border-radius:8px 8px 0 0;font:12px/1.8 Consolas,monospace;margin-top:14px}
    .table-wrap{overflow:auto;margin:18px 0}table{border-collapse:collapse;width:100%;font-size:14px}th,td{padding:10px 12px;text-align:left;vertical-align:top;border:1px solid var(--line);min-width:100px}th{background:#edf3e8;color:#365546}tr:nth-child(even){background:#fafbf8}li{margin:7px 0}aside{border-left:4px solid #8eab78;padding:12px 16px;background:#f3f7ec}footer{text-align:center;padding:16px 24px 35px;color:var(--muted);font-size:13px}
    @media(max-width:920px){.layout{display:block;padding:0 14px}nav{position:static;max-height:none;display:none}section{padding:22px 18px}header{padding:25px 22px}header h1{font-size:26px}}
    @media print{body{background:white;font-size:10pt}header{background:white;color:black;padding:0;border:0}header p,header small{color:#444}nav{display:none}.layout{display:block;margin:0;padding:0}section{border:0;box-shadow:none;padding:0;page-break-before:always}pre{white-space:pre-wrap;background:#f3f3f3;color:black}h3,h4{page-break-after:avoid}tr{break-inside:avoid}footer{display:none}}
    '''
    document = '<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>智能腰靠 · 硬件接手与自动驱动指南</title><style>'+css+'</style></head><body>'
    document += '<header><small>LUMBAR LAB / HARDWARE HANDOFF</small><h1>智能腰靠 · 硬件接手与自动驱动指南</h1><p>从真实压力输入、八区映射，到电机适配、板端保护与自动闭环验收。电脑端软件已具备，硬件侧按本指南接入。</p><span class="badge">2026-10-24 · 硬件联调开发包 · 离线可读</span></header>'
    document += '<div class="layout"><nav aria-label="交接文档目录">'+''.join(navigation)+'</nav><main>'+''.join(sections)+'</main></div><footer>文档由 HANDOFF.md 与 docs/HARDWARE_INTEGRATION.md 生成。源码与实际硬件验证共同构成接入依据。</footer></body></html>'
    return document.encode('utf-8')


def main():
    document = build_html()
    (ROOT / '交接说明.html').write_bytes(document)
    files = {}
    for name in ['run.py','start_windows.bat','start_unix.sh','requirements-serial.txt','README.md','HANDOFF.md','.gitignore']:
        files[name] = (ROOT/name).read_bytes()
    for directory in ['lumbar','web','config','firmware','docs','tests','integrations','tools']:
        for path in sorted((ROOT/directory).rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix not in ('.pyc','.pyo','.log','.tmp'):
                files[path.relative_to(ROOT).as_posix()] = path.read_bytes()
    files['交接说明.html'] = document
    files['data/sessions/.gitkeep'] = b''
    # No session data, old archives, local environments or machine-specific caches.
    inventory = ''.join(hashlib.sha256(content).hexdigest()+'  '+name+'\n' for name,content in sorted(files.items()))
    files['MANIFEST.sha256'] = inventory.encode('utf-8')
    destination = ROOT/(PACKAGE+'.zip')
    with ZipFile(destination,'w',compression=ZIP_DEFLATED) as archive:
        for name,content in sorted(files.items()):
            archive.writestr(PACKAGE+'/'+name,content)
    with ZipFile(destination) as archive:
        if archive.testzip() is not None:
            raise RuntimeError('Archive CRC verification failed')
    checksum = hashlib.sha256(destination.read_bytes()).hexdigest()
    (ROOT/(PACKAGE+'.zip.sha256')).write_text(checksum+'  '+destination.name+'\n',encoding='utf-8')
    print(f'{destination.name}: {len(files)} files, {destination.stat().st_size} bytes')
    print('SHA256: '+checksum)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
