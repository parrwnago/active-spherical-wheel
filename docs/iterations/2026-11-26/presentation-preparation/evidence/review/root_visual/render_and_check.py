from pathlib import Path
import sys,json,hashlib,re
ROOT=Path.cwd();sys.path.insert(0,str(ROOT/'build/presentation-pdf-packages'))
import pymupdf
from PIL import Image,ImageOps,ImageDraw,ImageFont
F=ROOT/'results/presentation_preparation_20261126';OUT=Path(__file__).resolve().parent
p=ROOT/'docs/iterations/2026-11-26/presentation-preparation/control-and-observer-study-guide.pdf'
d=pymupdf.open(p);toc=d.get_toc();issues=[];pages=[]
content=json.loads((F/'authoring/guide_content.json').read_text())['pages']
for i,page in enumerate(d):
 pix=page.get_pixmap(matrix=pymupdf.Matrix(1.1,1.1),alpha=False);path=OUT/f'page_{i+1:02d}.png';pix.save(path)
 txt=page.get_text();blocks=page.get_text('dict')['blocks'];bad=[]
 for b in blocks:
  if b['type']!=0:continue
  for line in b['lines']:
   for span in line['spans']:
    x0,y0,x1,y1=span['bbox']
    if x0<30 or x1>590 or y0<10 or y1>777:bad.append(span['text'])
 if bad:issues.append({'page':i+1,'out_of_page_text':bad})
 pages.append({'page':i+1,'title':content[i]['title'],'text_characters':len(txt),'text_blocks_outside_safe_page':bad,'image':str(path.relative_to(ROOT)),'links':len(page.get_links())})
 if len(txt)<200:issues.append({'page':i+1,'too_little_extractable_text':len(txt)})
for start in range(0,len(d),9):
 sheet=Image.new('RGB',(1368,1848),'#DEE5EA');draw=ImageDraw.Draw(sheet)
 for j in range(start,min(start+9,len(d))):
  im=Image.open(OUT/f'page_{j+1:02d}.png');im.thumbnail((432,560));x=12+(j-start)%3*456;y=34+(j-start)//3*616;sheet.paste(im,(x,y));draw.text((x,y-22),f'Page {j+1}: {content[j]["id"]}',fill='#17334D')
 sheet.save(OUT/f'contact_sheet_{start//9+1}.jpg',quality=92)
checks={'pages':len(d)==33,'bookmarks':len(toc)==33,'bookmark_pages_match':all(row[2]==i+1 for i,row in enumerate(toc)),'all_pages_have_text':all(v['text_characters']>=200 for v in pages),'no_out_of_page_text':not issues,'has_external_citations':any(l.get('uri','').startswith('https://') for page in d for l in page.get_links())}
r={'passed':all(checks.values()),'checks':checks,'pdf_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'pages':pages,'issues':issues,'visual_review':'Contact sheets/full pages are rendered for root human-like visual inspection; script only checks bounds/text/bookmarks.'}
(OUT/'render_report.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({'checks':checks,'issues':issues,'sha256':r['pdf_sha256']},indent=2));assert r['passed']
