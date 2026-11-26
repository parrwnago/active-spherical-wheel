"""Build a source-linked teaching PDF; consumes saved evidence, never physics."""
from pathlib import Path
import sys,json,hashlib,io,re,html,shutil,traceback
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'build/presentation-pdf-packages'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,PageBreak,Table,TableStyle,Image,KeepTogether,Flowable
from reportlab.lib.styles import getSampleStyleSheet,ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.utils import ImageReader
FAMILY=ROOT/'results/presentation_preparation_20261126';HERE=Path(__file__).resolve().parent
OUT=ROOT/'docs/iterations/2026-11-26/presentation-preparation/control-and-observer-study-guide.pdf'
FONT=Path(matplotlib.get_data_path())/'fonts/ttf'
for name,file in [('Body','DejaVuSans.ttf'),('Bold','DejaVuSans-Bold.ttf'),('Italic','DejaVuSans-Oblique.ttf'),('Mono','DejaVuSansMono.ttf')]:pdfmetrics.registerFont(TTFont(name,str(FONT/file)))
pdfmetrics.registerFontFamily('Body',normal='Body',bold='Bold',italic='Italic',boldItalic='Bold')
NAVY=colors.HexColor('#17334D');BLUE=colors.HexColor('#256F98');TEAL=colors.HexColor('#16796F');MUTED=colors.HexColor('#526170');LIGHT=colors.HexColor('#EEF4F7');INK=colors.HexColor('#202F3D')
styles={
'body':ParagraphStyle('body',fontName='Body',fontSize=10.1,leading=14.7,textColor=INK,spaceAfter=8),
'bullet':ParagraphStyle('bullet',fontName='Body',fontSize=10.0,leading=14.4,leftIndent=12,firstLineIndent=-9,textColor=INK,spaceAfter=6),
'h1':ParagraphStyle('h1',fontName='Bold',fontSize=21,leading=26,textColor=NAVY,spaceAfter=12),
'h2':ParagraphStyle('h2',fontName='Bold',fontSize=11.4,leading=15,textColor=TEAL,spaceBefore=8,spaceAfter=6),
'kicker':ParagraphStyle('kicker',fontName='Bold',fontSize=8.5,leading=12,textColor=BLUE,spaceAfter=7),
'small':ParagraphStyle('small',fontName='Body',fontSize=8.1,leading=11.4,textColor=MUTED,spaceAfter=5),
'caption':ParagraphStyle('caption',fontName='Body',fontSize=8.4,leading=11.5,textColor=MUTED,spaceAfter=8),
'cell':ParagraphStyle('cell',fontName='Body',fontSize=8.9,leading=12.2,textColor=INK),
'cellhead':ParagraphStyle('cellhead',fontName='Bold',fontSize=8.8,leading=12,textColor=colors.white),
'callout':ParagraphStyle('callout',fontName='Body',fontSize=10.1,leading=14.8,textColor=NAVY),
'code':ParagraphStyle('code',fontName='Mono',fontSize=8.0,leading=11,textColor=INK,spaceAfter=7),
}

def para(text,style='body'):
 text=re.sub(r'&(?!#?\w+;)', '&amp;', text)
 return Paragraph(text,styles[style])

def equation(expr,height=.47):
 key=hashlib.sha256(expr.encode()).hexdigest()[:12];path=FAMILY/'figures'/('equation_'+key+'.png')
 if not path.exists():
  fig=plt.figure(figsize=(7.0,height),dpi=240);fig.text(.01,.5,'$'+expr+'$',fontsize=13,ha='left',va='center',color='#17334D');fig.savefig(path,transparent=True,dpi=240,bbox_inches='tight',pad_inches=.035);plt.close(fig)
 w,h=ImageReader(str(path)).getSize(); scale=min(72/240,475/w)
 return Image(str(path),width=w*scale,height=h*scale,hAlign='LEFT')

class Bookmark(Flowable):
 def __init__(self,key,title):super().__init__();self.key=key;self.title=title;self.width=self.height=0
 def draw(self):self.canv.bookmarkPage(self.key);self.canv.addOutlineEntry(self.title,self.key,0,False)

def table(rows,widths=None):
 data=[]
 for i,row in enumerate(rows):data.append([para(str(c),'cellhead' if i==0 else 'cell') for c in row])
 if widths is None:widths=[475/len(rows[0])]*len(rows[0])
 t=Table(data,colWidths=widths,hAlign='LEFT',repeatRows=1)
 t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),NAVY),('BACKGROUND',(0,1),(-1,-1),colors.HexColor('#F4F7F9')),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),8),('RIGHTPADDING',(0,0),(-1,-1),8),('TOPPADDING',(0,0),(-1,-1),7),('BOTTOMPADDING',(0,0),(-1,-1),7),('LINEBELOW',(0,0),(-1,0),.6,colors.white),('LINEBELOW',(0,1),(-1,-1),.4,colors.HexColor('#D5E0E7'))]))
 return t

def image_block(path,maxheight=220):
 p=ROOT/path
 w,h=ImageReader(str(p)).getSize();scale=min(475/w,maxheight/h)
 return Image(str(p),width=w*scale,height=h*scale,hAlign='LEFT')

def footer(c,doc):
 c.saveState();w,h=doc.pagesize;c.setStrokeColor(colors.HexColor('#C9D8E2'));c.line(48,39,w-48,39);c.setFont('Body',8);c.setFillColor(MUTED);c.drawString(48,26,'Active spherical wheel | Study guide | 26 November 2026');c.drawRightString(w-48,26,str(doc.page));c.restoreState()

def build(content):
 story=[];page_map={p['id']:i+1 for i,p in enumerate(content['pages'])}
 for i,page in enumerate(content['pages']):
  if i:story.append(PageBreak())
  story.extend([Bookmark(page['id'],page['title']),para(page.get('section','PRESENTATION PREPARATION').upper(),'kicker'),para(page['title'],'h1')])
  for b in page['blocks']:
   k=b['type']
   if k=='p':story.append(para(b['text']))
   elif k=='small':story.append(para(b['text'],'small'))
   elif k=='h':story.append(para(b['text'],'h2'))
   elif k=='bullets':
    for t in b['items']:story.append(para('• '+t,'bullet'))
   elif k=='eq':story.extend([equation(b['text'],b.get('height',.47)),Spacer(1,7)])
   elif k=='table':story.extend([table(b['rows'],b.get('widths')),Spacer(1,9)])
   elif k=='figure':
    story.append(image_block(b['path'],b.get('height',220)))
    if b.get('caption'):story.append(para(b['caption'],'caption'))
   elif k=='callout':
    t=Table([[para(b['text'],'callout')]],colWidths=[475]);t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,-1),LIGHT),('BOX',(0,0),(-1,-1),.5,colors.HexColor('#CCDDE8')),('LEFTPADDING',(0,0),(-1,-1),10),('RIGHTPADDING',(0,0),(-1,-1),10),('TOPPADDING',(0,0),(-1,-1),9),('BOTTOMPADDING',(0,0),(-1,-1),9)]));story.extend([t,Spacer(1,9)])
   elif k=='toc':
    for ident,label in b['items']:
     story.append(para(f'<link href="#{ident}" color="#256F98">{label}</link> <font color="#526170">— page {page_map[ident]}</font>'))
   elif k=='code':story.append(para(html.escape(b['text']).replace('\n','<br/>'),'code'))
  if page.get('sources'):story.append(para('<b>Sources:</b> '+page['sources'],'small'))
 doc=SimpleDocTemplate(str(OUT),pagesize=(612,792),rightMargin=65,leftMargin=65,topMargin=46,bottomMargin=53,title='Active Spherical Wheel — Dynamics, Control and Contact Estimation',author='Project study guide, prepared with Codex assistance',subject='Current implementation and saved-log investigation; no new simulations')
 doc.build(story,onFirstPage=footer,onLaterPages=footer)
 sys.path.insert(0,str(ROOT/'build/presentation-pdf-packages'));import pymupdf
 pdf=pymupdf.open(OUT);titles=[]
 for p in pdf:titles.append(p.get_text()[:110])
 report={'expected_pages':len(content['pages']),'actual_pages':len(pdf),'page_titles':titles,'bytes':OUT.stat().st_size,'sha256':hashlib.sha256(OUT.read_bytes()).hexdigest(),'path':str(OUT.relative_to(ROOT)),'flow_pages_match':len(pdf)==len(content['pages'])}
 (FAMILY/'authoring/build_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
 if len(pdf)!=len(content['pages']):raise RuntimeError('Page overflow: adjust content or layout before delivery')
 return report

if __name__=='__main__':
 stages=HERE/'build_stages';stages.mkdir(exist_ok=True)
 index=len([p for p in stages.iterdir() if p.is_dir()])+1
 stage=stages/f'stage{index:02d}';stage.mkdir(exist_ok=False)
 for name in ['build_guide.py','guide_content.json','make_content.py']:
  source=HERE/name
  if source.exists():shutil.copy2(source,stage/name)
 try:
  report=build(json.loads((HERE/'guide_content.json').read_text()))
  shutil.copy2(OUT,stage/'output.pdf');(stage/'report.json').write_text(json.dumps(report,indent=2)+'\n')
 except Exception:
  (stage/'failure.txt').write_text(traceback.format_exc())
  if OUT.exists():shutil.copy2(OUT,stage/'partial_or_overflow.pdf')
  raise
