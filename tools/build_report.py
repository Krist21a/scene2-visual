"""Create a Chinese PDF, talk notes and indexed evidence for Scene2."""
import csv,json,shutil
import sys
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.colors import HexColor
from PIL import Image,ImageDraw,ImageFont
sys.path.insert(0,'/tmp/scene2-report-deps')
from pptx import Presentation
from pptx.util import Inches,Pt
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import PP_ALIGN, MSO_AUTO_SIZE, MSO_ANCHOR
from pptx.oxml.xmlchemy import OxmlElement

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'reports/scene2-20261111'
for sub in ('images','evidence','videos'):(OUT/sub).mkdir(parents=True,exist_ok=True)
SOURCE=ROOT/'evidence/six-20261111-01'
events=[json.loads(line) for line in (SOURCE/'events.jsonl').read_text().splitlines()]
completed=[row for row in events if row['event']=='completed']
def receipt(label):
    path=ROOT/'evidence'/label/'summary.json'
    return json.loads(path.read_text()) if path.exists() else None
RUN='report-six-20261111-02';VERIFY='verify-report-20261111-02'
supplement=receipt(RUN);verified=receipt(VERIFY)
def copy(src,dst):shutil.copy2(ROOT/src,OUT/dst)
for name in ['acceptance-20261111.json','audit-six-20261111-01.json','tests-20261111-07.log','final-replay-20261111.log']:
    copy('evidence/'+name,'evidence/'+name)
for folder in ['six-20261111-01','final-ledger-six-20261111-02','verify-six-20261111-04',RUN,VERIFY]:
    for name in ('summary.json','source-manifest.json'):
        path=ROOT/'evidence'/folder/name
        if path.exists():
            dest=OUT/'evidence'/folder;dest.mkdir(exist_ok=True);shutil.copy2(path,dest/name)
copy('HANDOFF-20261111.txt','evidence/开发交接记录.txt')
copy('evidence/six-20261111-01/003-initial_occupancy.png','images/01-原成功回合-初始空箱.png')
copy('evidence/verify-six-20261111-04/05.png','images/02-原成功回合-最终验收.png')
milestones=[('B1','022'),('B2','041'),('A1','060'),('A2','079'),('C1','105'),('C2','140')]
font='/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'
sheet=Image.new('RGB',(1800,840),'#edf2f7');draw=ImageDraw.Draw(sheet)
for i,((label,index),row) in enumerate(zip(milestones,completed)):
    source=SOURCE/f'{index}-placement_check.png'
    shutil.copy2(source,OUT/'images'/f'{i+3:02d}-原成功回合-{label}.png')
    image=Image.open(source);image.thumbnail((580,330))
    x=10+(i%3)*600;y=10+(i//3)*420;sheet.paste(image,(x,y+45))
    seconds=row['elapsed_s'];stamp=f'{int(seconds)//60:02d}:{int(seconds)%60:02d}'
    draw.text((x,y),f'{i+1}/6  {label} 已确认  |  {stamp}',font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',26),fill='#17324d')
sheet.save(OUT/'images/09-原成功回合-六件里程碑.png')
with (OUT/'指标与时间线.csv').open('w',encoding='utf-8-sig',newline='') as file:
    writer=csv.writer(file);writer.writerow(['回合','序号','物体','确认时间_秒','本件及前置阶段耗时_秒'])
    for label,rows in [('原成功回合 six-20261111-01',completed)]:
        previous=0
        for i,row in enumerate(rows):
            writer.writerow([label,i+1,row['ledger'][-1]['kind'],round(row['elapsed_s'],3),round(row['elapsed_s']-previous,3)])
            previous=row['elapsed_s']
    runlog=ROOT/'evidence'/RUN/'events.jsonl'
    if runlog.exists():
        previous=0
        for row in map(json.loads,runlog.read_text().splitlines()):
            if row['event']=='completed':
                writer.writerow(['汇报补录 '+RUN,row['count'],row['ledger'][-1]['kind'],round(row['elapsed_s'],3),round(row['elapsed_s']-previous,3)])
                previous=row['elapsed_s']

pdfmetrics.registerFont(TTFont('CN',font))
pdfmetrics.registerFont(TTFont('Latin','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'))
W,H=960,540;c=canvas.Canvas(str(OUT/'Scene2开发与验收汇报.pdf'),pagesize=(W,H))
prs=Presentation();prs.slide_width=Inches(W/72);prs.slide_height=Inches(H/72)
slide=None
c.setTitle('Scene2 视觉分拣：开发、验收与任务录像');c.setAuthor('Scene2 项目')
NAVY='#17324d';TEAL='#087f8c';GRAY='#526477';page=0
def text(x,y,value,size=15,color=NAVY):
    c.setFillColor(HexColor(color));obj=c.beginText(x,H-y)
    for char in value:
        obj.setFont('CN' if ord(char)>=0x2e80 else 'Latin',size);obj.textOut(char)
    c.drawText(obj)
    width=sum(pdfmetrics.stringWidth(char,'CN' if ord(char)>=0x2e80 else 'Latin',size) for char in value)+4
    box=slide.shapes.add_textbox(Inches(x/72),Inches((y-size*1.05)/72),Inches(width/72),Inches((size*1.6)/72))
    frame=box.text_frame;frame.margin_left=frame.margin_right=frame.margin_top=frame.margin_bottom=0
    frame.word_wrap=False;frame.paragraphs[0].alignment=PP_ALIGN.LEFT
    frame.auto_size=MSO_AUTO_SIZE.NONE;frame.vertical_anchor=MSO_ANCHOR.TOP
    run=frame.paragraphs[0].add_run();run.text=value
    run.font.name='Noto Sans CJK SC';run.font.size=Pt(size);run.font.color.rgb=RGBColor.from_string(color[1:])
def ppt_rect(x,y,w,h,color):
    shape=slide.shapes.add_shape(MSO_SHAPE.RECTANGLE,Inches(x/72),Inches(y/72),Inches(w/72),Inches(h/72))
    shape.fill.solid();shape.fill.fore_color.rgb=RGBColor.from_string(color[1:]);shape.line.fill.background()
    shape._element.spPr.append(OxmlElement('a:effectLst'))
def new(title):
    global page,slide
    if page:c.showPage()
    page+=1;slide=prs.slides.add_slide(prs.slide_layouts[6]);slide.background.fill.solid()
    slide.background.fill.fore_color.rgb=RGBColor.from_string('F3F6FA');ppt_rect(0,0,W,9,TEAL)
    c.setFillColor(HexColor('#f3f6fa'));c.rect(0,0,W,H,fill=1,stroke=0)
    c.setFillColor(HexColor(TEAL));c.rect(0,H-9,W,9,fill=1,stroke=0)
    text(42,40,'SCENE2 / DEVELOPMENT & VALIDATION',10,TEAL);text(42,85,title,28)
    c.setStrokeColor(HexColor('#d6e1e9'));c.line(42,H-106,918,H-106)
    text(42,520,'2026-11-11  |  官方 scene2 seed0  |  视觉验收，不等同官方评分',10,GRAY);text(900,520,str(page),10,GRAY)
def lines(x,y,values,size=16,step=31):
    for i,value in enumerate(values):text(x,y+i*step,value,size)
def picture(path,x,y,w,h):
    c.drawImage(str(path),x,H-y-h,width=w,height=h,preserveAspectRatio=True,anchor='c',mask='auto')
    iw,ih=Image.open(path).size;scale=min(w/iw,h/ih);pw,ph=iw*scale,ih*scale
    slide.shapes.add_picture(str(path),Inches((x+(w-pw)/2)/72),Inches((y+(h-ph)/2)/72),width=Inches(pw/72),height=Inches(ph/72))
def metric(x,value,label,detail):
    c.setFillColor(HexColor('#ffffff'));c.roundRect(x,H-245,268,120,10,fill=1,stroke=0)
    ppt_rect(x,125,268,120,'#ffffff')
    text(x+18,174,value,34,TEAL);text(x+18,205,label,16);text(x+18,230,detail,11,GRAY)
new('Scene2 开发汇报：成功证据与补录结果')
metric(42,'6 / 6','原回合连续分拣','原成功回合：B → B → A → A → C → C')
metric(346,'A2 / B2 / C2','原回合终态五帧复核','生产检查 + 另一组独立 RGB-D 采集')
metric(650,'69 项','最终版本单元测试通过','另有 47 帧终态图像回放通过')
lines(48,295,['任务：识别、抓取六件零件，并放入对应的三个料箱。','方法：RGB-D 感知、约束规划、反馈执行与视觉验收闭环。','原成功回合用时 3011.9 秒（约 50.2 分钟）。'],18,38)
text(48,443,'本次主录像：六件均执行抓放，验收 5/6；不是 6/6 成功录像。',17,TEAL)
text(48,475,'原 6/6 成功回合没有连续录像。两轮补录分别为 2/6、5/6，结果单独列示。',15)
new('任务范围与执行闭环')
for i,label in enumerate(['视觉观测','抓取规划','闭环抓取','运输 / 释放','多帧验收']):
    x=44+i*177;c.setFillColor(HexColor(TEAL));c.roundRect(x,H-205,154,58,8,fill=1,stroke=0)
    ppt_rect(x,147,154,58,TEAL);text(x+17,183,label,17,'#ffffff')
    if i<4:text(x+160,184,'›',21,TEAL)
lines(48,258,['输入：头部与腕部 RGB-D、公开 URDF / 标定、关节、夹爪和 IMU。','执行：正常 MPC → WBC 通道；10 Hz；臂关节步长上限 0.012 rad。','成功：完整观测包络在正确箱内、数量增加、源位置消失、稳定多帧。','结束：所有料箱同时连续五个不同时间戳画面满足预期数量。'],17,44)
text(48,465,'策略不读取物体真值或 MuJoCo qpos；未改动场景物理。',15,TEAL)
new('开发重点：从失败证据修正策略')
rows=[('接近阶段净空不足','Cartesian 候选步长回退；保留实际姿态硬检查。'),('A 运输腕关节饱和','释放规划保留 0.08 rad 腕关节余量。'),('C 把手重叠 / 杆体混淆','独立金属杆与端点关联；第二件增加方向及间距偏好。'),('终态检测间歇漏件','识别 B 倾斜可见侧面；以旋转箱腔多边形检验 C。'),('历史账本与最终场景不一致','正常结束前重新检查所有箱子，失败则不报告成功。')]
for i,(issue,change) in enumerate(rows):
    y=148+i*65;text(48,y,issue,17,TEAL);text(48,y+27,change,15)
text(48,484,'C 验收保留箱腔外边界内缩 15 mm 与物体边距 3 mm；越界样本仍拒绝。',12,GRAY)
new('原成功回合：六件里程碑')
picture(OUT/'images/09-原成功回合-六件里程碑.png',42,119,876,354)
text(48,492,'时间从策略启动计算；截图为关键帧，不是连续录像。',12,GRAY)
new('时间分布：C 是后续提速重点')
previous=0
for i,row in enumerate(completed):
    duration=row['elapsed_s']-previous;previous=row['elapsed_s'];y=148+i*48
    text(48,y+13,milestones[i][0],17);c.setFillColor(HexColor(TEAL if i<4 else '#d38526'))
    c.roundRect(100,H-y-20,duration/1152*650,27,4,fill=1,stroke=0)
    ppt_rect(100,y-7,duration/1152*650,27,TEAL if i<4 else '#d38526');text(120+duration/1152*650,y+13,f'{duration/60:.1f} 分钟',14)
text(48,465,'按相邻完成事件划分阶段；首段包含初始化，非纯机械动作耗时。',13,GRAY)
text(48,489,'C 两件约占总用时 57%；可优化持物姿态估计、释放修正与规划效率。',13,GRAY)
new('终态验收与证据边界')
picture(OUT/'images/02-原成功回合-最终验收.png',42,128,500,300)
lines(572,158,['五帧全部 A2 / B2 / C2','完整观测包络检查','原位置消失 + 数量增加','原近壁失败样本仍被拒绝'],16,47)
lines(48,443,['原回合运动结束后修正终态感知，并在同一场景重新采集验证。','同种子补录出现过 2/6 失败；稳定性还需同种子与多种子重复验证。','task_success=null，未接入官方评分；不作通用成功率声明。'],13,24)
new('完整任务录像：汇报补录回合')
if supplement:
    success=bool(supplement.get('visual_success'))
    lines(48,153,[f"补录回合：{RUN}；完成 {len(supplement['completed'])}/6。",f"执行结果：{'视觉验收通过' if success else '未通过，详见停止证据'}；用时 {supplement['elapsed_s']/60:.1f} 分钟。",f"独立新帧复核：{'通过' if verified and verified.get('verified') else '未通过（五帧均 A2/B2/C1）' if verified else '未采集'}。"],18,44)
else:lines(48,153,['补录回合进行中；本页在结束后按真实结果更新。'],18)
text(48,286,'补录尝试 1：完成 2/6，A 下降受净空限制超时；失败录像与日志单独保留。',13,GRAY)
lines(48,322,['原速版：保留连续动作、等待与纠正过程，无删段。','加速版：明确标注倍速，用于现场演示。','仅采集 MuJoCo 场景窗口，无音轨；录像不提供给策略使用。','配套时间线、关键帧与原始验收 JSON 可交叉核对。'],16,37)
end_image=OUT/'images/15-补录尝试2-终态.png'
if end_image.exists():
    new('补录画面：同一回合的开始与结束')
    picture(OUT/'images/14-补录尝试2-初始.png',42,125,425,266)
    picture(end_image,493,125,425,266)
    text(48,419,'开始：桌面六件物体，料箱为空。',16)
    text(499,419,'结束：验收 5/6，最后一件 C 边距不合格。',16)
    text(48,465,'两张画面来自同一份连续原速录像；具体完成数量见上一页及验收 JSON。',13,GRAY)
c.save()
prs.save(OUT/'Scene2开发与验收汇报.pptx')
(OUT/'汇报讲稿.txt').write_text('''Scene2 汇报讲稿（约 3–5 分钟）

任务是在官方 seed0 场景里，将桌面 A、B、C 三类物体各两件放入对应料箱。原成功回合从空箱开始，不做跨回合拼接，连续完成六件，用时约 50.2 分钟。

系统采用感知、规划、反馈执行与验收的闭环。策略只使用头部和腕部 RGB-D、公开标定与 URDF，以及关节、夹爪和 IMU 反馈，经正常 MPC 到 WBC 执行；没有读取物体真值，也没有改动物理场景。

开发重点是处理实际反馈与可见证据。A 的腕关节饱和通过规划余量修正。C 的长杆和重叠把手通过独立金属杆、端点关联和放置方向偏好处理。接近阶段加入参考步长回退，同时保留硬净空检查；成功回合没有实际触发回退，不能把成功直接归因于它。

验收曾发现逐件账本显示六件，但独立复核漏检。我们没有直接沿用成功结论，而是检查 RGB-D。问题分别是 B 倾斜侧面可见厚度变化，以及 C 旋转箱腔被轴对齐矩形过度裁角。修正后保持原有边距，旧越界样本仍被拒绝。最终两组五帧检查均确认 A2/B2/C2，69 项测试及 47 帧终态回放通过。正常结束也要求所有箱子同时连续五帧合格。

录像说明：原成功回合未保存连续录像。资料中的录像来自另一次空场汇报补录，真实结果见 PDF 最后一页和对应 summary.json，不能冒充原成功回合录像。原速版完整保留过程，加速版用于现场展示。

本次主录像已完整执行六件抓放，但只有五件通过验收。最后一件 C 的完整观测包络不满足箱边距，独立五帧复核也未通过，不能当作 6/6 成功演示。该失败反映当前系统的复现稳定性不足。

第一轮补录在 A 下降时因净空限制超时，完成 2/6；失败录像和日志单独保留。说明当前成功不能直接解释为稳定成功率。

结论是已取得 seed0 六件连续视觉验收证据。后续需优化速度，开展同种子及多种子重复试验，并接入官方评分。task_success 为 null，不作官方成绩或通用成功率声明。
''')
(OUT/'阅读说明.txt').write_text('''Scene2 汇报材料

先读 Scene2开发与验收汇报.pdf；讲述可参考 汇报讲稿.txt。
images/ 包含原成功回合初始画面、终态验收和六件关键帧。
指标与时间线.csv 区分原成功回合与汇报补录，可制作图表。
evidence/ 保存验收、测试、源代码清单和录像采集信息。
videos/ 为本次补录；与原成功回合不是同一回合。最终媒体属性见 媒体清单.json。

所有数字以证据为准。关键帧拼图不是完整录像；加速视频不是原速执行。
原成功回合最终感知修正在运动结束后完成；补录结果单独列示。

本次主录像为完整过程补录，验收结果 5/6，不是六件成功录像。
此前原成功回合有 6/6 及独立复核证据，但没有连续录像。
两轮补录结果分别为 2/6、5/6；失败不隐去，未放宽验收条件。
''')
print(OUT/'Scene2开发与验收汇报.pdf')

