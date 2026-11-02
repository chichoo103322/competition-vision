"""Build a precise nominal floor-plan SVG and standalone viewer, without hardware."""
from __future__ import annotations
import argparse
import html
import json
from pathlib import Path


def build(config, output):
    output.mkdir(parents=True, exist_ok=True)
    shapes = []
    # Plot in SVG millimetres, world origin at bottom-left, screen origin top-left.
    def rect(r, fill, stroke="#657176", dash=""):
        x, y, w, h = r
        shapes.append(f'<rect x="{x}" y="{2400-y-h}" width="{w}" height="{h}" fill="{fill}" stroke="{stroke}" stroke-width="3" {dash}/>')

    def line(x1, y1, x2, y2, color="#526b82", extra=""):
        shapes.append(f'<path d="M{x1},{2400-y1} L{x2},{2400-y2}" stroke="{color}" fill="none" stroke-width="3" {extra}/>')

    def label(t, x, y, size=37, color="#1b2b33", extra=""):
        shapes.append(f'<text x="{x}" y="{2400-y}" text-anchor="middle" font-size="{size}" fill="{color}" {extra}>{html.escape(str(t))}</text>')

    def dimension(x1,y1,x2,y2,t,tx,ty):
        line(x1,y1,x2,y2,extra='marker-start="url(#arrow)" marker-end="url(#arrow)"')
        label(t,tx,ty,34,"#235180")

    rect(config["boundary_mm"], "#dfdfdf", "#172b38")
    for r in config["floor_exclusions"]:
        rect(r["rect_mm"], "#fff8c5", "#bcb16a")
        x,y,w,h=r["rect_mm"]
        label("450 × 450",x+w/2,y+h/2-12,32,"#695d23")
    for z in config["start_zones"]:
        rect(z["rect_mm"],"#2457cb","#2457cb")
        x,y,w,h=z["rect_mm"]
        label("启停区 "+z["id"][-1],x+w/2,y+h/2,37,"white")
        label("300 × 300",x+w/2,y+h/2-60,28,"white")
    for z in config["work_zones"]:
        rect(z["rect_mm"],"#fff","#777")
        for x,y in z["placement_reference_points_mm"]:
            shapes.append(f'<circle cx="{x}" cy="{2400-y}" r="27" fill="none" stroke="#333" stroke-width="5"/>')
            line(x-38,y,x+38,y,"#999")
            line(x,y-38,x,y+38,"#999")
    label("暂存区",-200,1250,39)
    label("580 × 150",-200,1180,32)
    label("粗加工区 · 580 × 150",1200,-160,39)
    label("两端放置参考点间距 300",1200,-225,30,"#46565e")
    line(1200,160,1200,2280,"#7d8990",'stroke-dasharray="28 14 5 14"')
    line(190,1200,2250,1200,"#7d8990",'stroke-dasharray="28 14 5 14"')
    dimension(1000,2050,1400,2050,"400",1200,2085)
    line(1000,1850,1000,2100);line(1400,1850,1400,2100)
    dimension(2020,1000,2020,1400,"400",2130,1185)
    line(1850,1000,2070,1000);line(1850,1400,2070,1400)
    dimension(0,-360,2400,-360,"2400 mm",1200,-330)
    line(0,0,0,-420);line(2400,0,2400,-420)
    dimension(2740,0,2740,2400,"2400 mm",2910,1200)
    line(2400,0,2790,0);line(2400,2400,2790,2400)
    # Variable references deliberately shown as bands, not a made-up exact device pose.
    rect([1100,2315,200,10],"#f1ad52","#a8650a")
    line(1100,2320,1100,2500,"#a8650a",'stroke-dasharray="10 10"')
    line(1300,2320,1300,2500,"#a8650a",'stroke-dasharray="10 10"')
    dimension(1100,2540,2400,2540,"距右边 1100–1300",1750,2580)
    label("原料区 · 转盘 Ø300",1200,2740,40)
    label("横向位置可变；顶边图示参考内距 75–85",1200,2670,31,"#9b5900")
    line(2400,1100,2400,1300,"#c4780c")
    label("二维码板",2660,1550,38)
    label("距上边",2660,1480,30,"#9b5900")
    label("1100–1300",2660,1420,30,"#9b5900")
    label("图示参考位置范围",2660,1360,27,"#9b5900")
    line(0,0,330,0,"#1b6e5d",'marker-end="url(#axis)"')
    line(0,0,0,330,"#1b6e5d",'marker-end="url(#axis)"')
    label("(0, 0)",-100,-100,32,"#1b6e5d");label("+X",350,-70,32,"#1b6e5d");label("+Y",-85,340,32,"#1b6e5d")
    label("2027 全国命题 · 初赛静态底图",1200,3020,62)
    label("尺寸单位 mm · 按规则尺寸构建，非现场测量",1200,2930,34,"#4d606b")
    label("灰色：名义车道　浅黄：禁行区　蓝色：启停区　橙色：位置可变参考",1200,-560,31)
    label("功能区按图示名义居中；真实场地有偏差。障碍物、机器人、雷达位置均未预填。",1200,-640,31,"#5a6870")
    svg='''<svg xmlns="http://www.w3.org/2000/svg" viewBox="-470 -760 3650 3900" role="img" aria-label="2027全国初赛名义静态场地图，尺寸毫米">
<defs><marker id="arrow" markerWidth="9" markerHeight="9" refX="4" refY="4" orient="auto-start-reverse"><path d="M0,0 L8,4 L0,8 Z" fill="#526b82"/></marker><marker id="axis" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="#1b6e5d"/></marker></defs>
<rect x="-470" y="-760" width="3650" height="3900" fill="#fff"/>
<g font-family="PingFang SC,Microsoft YaHei,sans-serif">'''+''.join(shapes)+'</g></svg>'
    (output/'field_map.svg').write_text(svg)
    (output/'field_map.json').write_text(json.dumps(config,ensure_ascii=False,indent=2))
    title='比赛静态底图'
    page=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{title}</title>
<style>body{{margin:0;background:#111;color:#f2f4f4;font:16px/1.7 -apple-system,BlinkMacSystemFont,"PingFang SC",sans-serif}}header,main{{max-width:1160px;margin:auto;padding:24px}}header{{display:flex;align-items:center;justify-content:space-between;gap:20px}}h1{{margin:0;font-size:25px}}p{{color:#bfcbcf;margin:8px 0}}a{{color:#75e6c0;text-underline-offset:4px}}a:focus-visible{{outline:2px solid #75e6c0;outline-offset:4px}}img{{width:100%;display:block;background:white}}article{{max-width:76ch;padding:20px 0}}table{{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}}td,th{{padding:10px 14px;text-align:left;border-bottom:1px solid #495257}}nav{{display:flex;gap:18px;flex-wrap:wrap}}@media(max-width:650px){{header{{display:block}}main,header{{padding:16px}}h1{{font-size:23px}}td,th{{padding:8px;font-size:14px}}}}</style>
<header><div><h1>{title}</h1><p>2027 全国命题初赛 · 左下原点 · 单位毫米</p></div><nav><a href="field_map.svg" download>下载矢量图</a><a href="field_map.json" download>下载坐标文件</a></nav></header><main><img src="field_map.svg" alt="2.4米比赛场地，400毫米十字通道和四块450毫米禁行区"><article><h2>固定尺寸与名义坐标</h2><table><tr><th>区域</th><th>左下角 (X,Y)</th><th>宽 × 高 / mm</th></tr>
<tr><td>场地外框</td><td>(0, 0)</td><td>2400 × 2400</td></tr><tr><td>左下浅黄区</td><td>(550, 550)</td><td>450 × 450</td></tr><tr><td>右下浅黄区</td><td>(1400, 550)</td><td>450 × 450</td></tr><tr><td>左上浅黄区</td><td>(550, 1400)</td><td>450 × 450</td></tr><tr><td>右上浅黄区</td><td>(1400, 1400)</td><td>450 × 450</td></tr><tr><td>启停区1</td><td>(2100, 2100)</td><td>300 × 300</td></tr><tr><td>启停区2</td><td>(2100, 0)</td><td>300 × 300</td></tr><tr><td>暂存区（名义）</td><td>(0, 910)</td><td>150 × 580</td></tr><tr><td>粗加工区（名义）</td><td>(910, 0)</td><td>580 × 150</td></tr></table>
<h2>图里没有固定的内容</h2><p>原料区与二维码板按规则保留位置范围，暂存和粗加工区的实际偏差待现场确认。障碍物数量、位置随机，不填假坐标；放置点的1/2/3身份也不由这张总图推断。</p><p>此图为二维地面布局。浅黄色是地面禁行区，不是雷达能测到的墙。尚未加入车体尺寸和安全余量，不能把图上的点路径直接用作车辆轨迹。</p><p>雷达、机器人初始位置均为空。后续若在启停区随机摆放，要另做初始位置和方向确认；静态底图本身不提供自动定位。</p><p>依据：全国命题文件第3页图1、第4页表2。适用于初赛名义场地，决赛以现场公布为准。</p></article></main></html>'''
    (output/'index.html').write_text(page)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config',type=Path,default=Path('config/field_map_national_2027.json'))
    ap.add_argument('--output',type=Path,default=Path('output/field_map_20261101'))
    a=ap.parse_args()
    build(json.loads(a.config.read_text()),a.output)
    print(a.output.resolve())


if __name__=='__main__':
    main()
