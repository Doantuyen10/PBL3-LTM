from pathlib import Path
import sys,zipfile,json,xml.etree.ElementTree as ET
from html import escape as esc
from PIL import Image as PILImage
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,PageBreak,Table,TableStyle,Image,KeepTogether
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER,TA_JUSTIFY,TA_LEFT
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from noi_dung import O,PAGES,TITLE
for n,p in [('Serif','/usr/share/fonts/truetype/dejavu/DejaVuSerif.ttf'),('Sans','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'),('Bold','/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf')]:pdfmetrics.registerFont(TTFont(n,p))
styles={
'p':ParagraphStyle('p',fontName='Serif',fontSize=10.8,leading=15.8,spaceAfter=8,alignment=TA_JUSTIFY),
'h':ParagraphStyle('h',fontName='Bold',fontSize=12,leading=17,spaceBefore=7,spaceAfter=7,textColor=colors.HexColor('#136c78'),keepWithNext=True),
'chapter':ParagraphStyle('chapter',fontName='Bold',fontSize=16,leading=22,spaceAfter=16,textColor=colors.HexColor('#15324f'),keepWithNext=True),
'eq':ParagraphStyle('eq',fontName='Sans',fontSize=10.3,leading=16,spaceBefore=3,spaceAfter=7,alignment=TA_CENTER),
'caption':ParagraphStyle('caption',fontName='Sans',fontSize=9,leading=13,spaceAfter=8,textColor=colors.HexColor('#4a5a68')),
'cell':ParagraphStyle('cell',fontName='Sans',fontSize=9.1,leading=12.8),
'cover':ParagraphStyle('cover',fontName='Sans',fontSize=12,leading=21,spaceAfter=14,alignment=TA_CENTER),
'coverTitle':ParagraphStyle('coverTitle',fontName='Bold',fontSize=19,leading=28,spaceAfter=18,alignment=TA_CENTER,textColor=colors.HexColor('#15324f')),
}
WID=470
class Doc(SimpleDocTemplate):
 def afterFlowable(self,f):
  if hasattr(f,'section_title'):self.pages_seen[f.section_title]=self.page

def tableflow(heads,rows):
 n=len(heads)
 if n==2:width=[425,45] if heads==['Nội dung','Trang'] else [160,310]
 elif n==3:width=[155,157.5,157.5]
 else:width=[110,195,70,95]
 vals=[[Paragraph(esc(str(x)) or ' ',styles['cell']) for x in row] for row in [heads]+rows]
 t=Table(vals,colWidths=width,repeatRows=1,hAlign='LEFT',minRowHeights=[22]*(len(rows)+1))
 t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e2edf2')),('GRID',(0,0),(-1,-1),.35,colors.HexColor('#afbec9')),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6)]))
 if heads==['Nội dung','Trang']:t.setStyle(TableStyle([('TOPPADDING',(0,0),(-1,-1),3),('BOTTOMPADDING',(0,0),(-1,-1),3)]))
 return t

def coverflows():return [Spacer(1,22),Paragraph('TRƯỜNG: ______________________________',styles['cover']),Paragraph('KHOA / BỘ MÔN: ________________________',styles['cover']),Spacer(1,42),Paragraph('BÁO CÁO DỰ ÁN CUỐI KỲ<br/>PBL3–LTM',styles['coverTitle']),Spacer(1,15),Paragraph(TITLE,styles['coverTitle']),Paragraph('PPO relay phục vụ ba e-puck<br/>Huấn luyện và tích hợp trong Webots',styles['cover']),Spacer(1,32),Paragraph('Giảng viên hướng dẫn: ____________________',styles['cover']),Paragraph('Nhóm / lớp: ___________________________',styles['cover']),Paragraph('Sinh viên thực hiện: _____________________',styles['cover']),Spacer(1,30),Paragraph('Địa điểm, tháng/năm nộp: __________________',styles['cover'])]
figure_list=[b[2] for pg in PAGES for b in pg['blocks'] if b[0]=='figure'];table_list=[b[1] for pg in PAGES for b in pg['blocks'] if b[0]=='table']
def makeflows(page_map):
 flow=coverflows()
 flow.extend([PageBreak(),Paragraph('MỤC LỤC',styles['chapter'])])
 rows=[]
 for pg in PAGES:rows.append([pg['title'],str(page_map.get(pg['title'],''))])
 flow.append(tableflow(['Nội dung','Trang'],rows))
 flow.extend([PageBreak(),Paragraph('DANH MỤC HÌNH',styles['chapter'])])
 for s in figure_list:flow.append(Paragraph(esc(s),styles['p']))
 flow.extend([PageBreak(),Paragraph('DANH MỤC BẢNG',styles['chapter'])])
 for s in table_list:flow.append(Paragraph(esc(s),styles['caption']))
 for pg in PAGES:
  flow.append(PageBreak());t=Paragraph(esc(pg['title']),styles['chapter']);t.section_title=pg['title'];flow.append(t)
  for b in pg['blocks']:
   if b[0] in ['p','h','eq']:flow.append(Paragraph(esc(b[1]),styles[b[0]]))
   elif b[0]=='table':flow.append(Paragraph(esc(b[1]),styles['caption']));flow.append(tableflow(b[2],b[3]));flow.append(Spacer(1,10))
   elif b[0]=='figure':
    path=O/'hinh'/b[1];im=PILImage.open(path);height=WID*im.height/im.width
    flow.append(KeepTogether([Image(str(path),width=WID,height=height),Spacer(1,5),Paragraph(esc(b[2]),styles['caption'])]))
 return flow

def foot(c,doc):
 c.setStrokeColor(colors.HexColor('#c7d4df'));c.line(64,40,534,40);c.setFont('Sans',8);c.setFillColor(colors.HexColor('#5b6f80'));c.drawString(64,28,'PBL3–LTM | Báo cáo dự án cuối kỳ');c.drawRightString(534,28,str(doc.page))
page_map={}
for i in range(3):
 doc=Doc(str(O/'Bao_cao_cuoi_ky_PBL3_LTM.pdf'),pagesize=(595.28,841.89),leftMargin=64,rightMargin=61.28,topMargin=48,bottomMargin=55,title=TITLE,author='')
 doc.pages_seen={};doc.build(makeflows(page_map),onFirstPage=foot,onLaterPages=foot)
 if doc.pages_seen==page_map:break
 page_map=doc.pages_seen
(O/'minh_chung/vi_tri_muc_pdf.json').write_text(json.dumps(page_map,ensure_ascii=False,indent=2))
# OOXML editable Word document.
e=lambda s:esc(str(s),quote=True)
NSW='http://schemas.openxmlformats.org/wordprocessingml/2006/main';NSR='http://schemas.openxmlformats.org/officeDocument/2006/relationships'
def wp(text,style='Normal',extra=''):
 return '<w:p><w:pPr><w:pStyle w:val="'+style+'"/>'+extra+'</w:pPr><w:r><w:t xml:space="preserve">'+e(text)+'</w:t></w:r></w:p>'
def wt(heads,rows):
 n=len(heads); widths=[3150,6100] if n==2 else [3000,3125,3125] if n==3 else [2200,3800,1400,1850]
 out='<w:tbl><w:tblPr><w:tblW w:w="9250" w:type="dxa"/><w:tblLayout w:type="fixed"/><w:tblBorders>'+''.join('<w:'+x+' w:val="single" w:sz="4" w:color="B7C6D2"/>' for x in ['top','left','bottom','right','insideH','insideV'])+'</w:tblBorders><w:tblCellMar>'+''.join('<w:'+x+' w:w="90" w:type="dxa"/>' for x in ['top','left','bottom','right'])+'</w:tblCellMar></w:tblPr><w:tblGrid>'+''.join(f'<w:gridCol w:w="{x}"/>' for x in widths)+'</w:tblGrid>'
 for idx,row in enumerate([heads]+rows):
  out+='<w:tr><w:trPr><w:cantSplit/>'+('<w:tblHeader/>' if idx==0 else '')+'</w:trPr>'
  for w,t in zip(widths,row):out+=f'<w:tc><w:tcPr><w:tcW w:w="{w}" w:type="dxa"/>'+('<w:shd w:fill="E2EDF2"/>' if idx==0 else '')+'</w:tcPr>'+wp(t or ' ','TableHead' if idx==0 else 'TableText')+'</w:tc>'
  out+='</w:tr>'
 return out+'</w:tbl>'+wp('','Spacer')
parts=[]
for txt,sty in [('TRƯỜNG: ______________________________','Cover'),('KHOA / BỘ MÔN: ________________________','Cover'),('','Cover'),('','Cover'),('BÁO CÁO DỰ ÁN CUỐI KỲ','Title'),('PBL3–LTM','Title'),('', 'Cover'),(TITLE,'Title'),('PPO relay phục vụ ba e-puck','Subtitle'),('Huấn luyện và tích hợp trong Webots','Subtitle'),('','Cover'),('Giảng viên hướng dẫn: ____________________','Cover'),('Nhóm / lớp: ___________________________','Cover'),('Sinh viên thực hiện: _____________________','Cover'),('','Cover'),('Địa điểm, tháng/năm nộp: __________________','Cover')]:parts.append(wp(txt,sty))
parts.append(wp('MỤC LỤC','Heading1','<w:pageBreakBefore/>'))
parts.append(wp('Cập nhật mục lục trong Word để lấy số trang theo bố cục hiển thị trên máy.','Caption'))
# Cached TOC entries without fixed page numbers; Word updates the field.
parts.append('<w:p><w:r><w:fldChar w:fldCharType="begin" w:dirty="true"/></w:r><w:r><w:instrText xml:space="preserve"> TOC \\o "1-2" \\h \\z \\u </w:instrText></w:r><w:r><w:fldChar w:fldCharType="separate"/></w:r></w:p>')
for pg in PAGES:parts.append(wp(pg['title'],'TOC1'))
parts.append('<w:p><w:r><w:fldChar w:fldCharType="end"/></w:r></w:p>')
for title,items in [('DANH MỤC HÌNH',figure_list),('DANH MỤC BẢNG',table_list)]:
 parts.append(wp(title,'Heading1','<w:pageBreakBefore/>'));parts.extend(wp(t,'Caption') for t in items)
images=[]
for pg in PAGES:
 parts.append(wp(pg['title'],'Heading1','<w:pageBreakBefore/>'))
 for b in pg['blocks']:
  if b[0] in ['p','h','eq']:parts.append(wp(b[1],{'p':'Normal','h':'Heading2','eq':'Equation'}[b[0]]))
  elif b[0]=='table':parts.append(wp(b[1],'Caption','<w:keepNext/>'));parts.append(wt(b[2],b[3]))
  elif b[0]=='figure':
   images.append(b[1]);num=len(images);im=PILImage.open(O/'hinh'/b[1]);cx=5800000;cy=int(cx*im.height/im.width)
   parts.append(f'<w:p><w:pPr><w:jc w:val="center"/><w:keepNext/></w:pPr><w:r><w:drawing><wp:inline distT="0" distB="0" distL="0" distR="0"><wp:extent cx="{cx}" cy="{cy}"/><wp:docPr id="{num}" name="Hình {num}"/><a:graphic><a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture"><pic:pic><pic:nvPicPr><pic:cNvPr id="0" name="{e(b[1])}"/><pic:cNvPicPr/></pic:nvPicPr><pic:blipFill><a:blip r:embed="img{num}"/><a:stretch><a:fillRect/></a:stretch></pic:blipFill><pic:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="{cx}" cy="{cy}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></pic:spPr></pic:pic></a:graphicData></a:graphic></wp:inline></w:drawing></w:r></w:p>');parts.append(wp(b[2],'Caption'))
sect='<w:sectPr><w:footerReference w:type="default" r:id="footer"/><w:pgSz w:w="11906" w:h="16838"/><w:pgMar w:top="1050" w:right="1200" w:bottom="1100" w:left="1450" w:header="500" w:footer="500"/></w:sectPr>'
xml=f'<w:document xmlns:w="{NSW}" xmlns:r="{NSR}" xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing" xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture"><w:body>'+''.join(parts)+sect+'</w:body></w:document>'
stylesxml=f'<w:styles xmlns:w="{NSW}"><w:docDefaults><w:rPrDefault><w:rPr><w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman"/><w:sz w:val="24"/><w:lang w:val="vi-VN"/></w:rPr></w:rPrDefault><w:pPrDefault><w:pPr><w:spacing w:after="140" w:line="290" w:lineRule="auto"/><w:jc w:val="both"/></w:pPr></w:pPrDefault></w:docDefaults>'
for name,size,color,bold,align in [('Normal',24,'172B40',False,'both'),('Heading1',34,'15324F',True,'left'),('Heading2',27,'136C78',True,'left'),('Title',38,'15324F',True,'center'),('Subtitle',28,'136C78',False,'center'),('Cover',26,'172B40',False,'center'),('Caption',20,'4A5A68',False,'left'),('Equation',23,'172B40',False,'center'),('TableText',21,'172B40',False,'left'),('TableHead',21,'172B40',True,'left'),('Spacer',8,'172B40',False,'left'),('TOC1',22,'172B40',False,'left')]:
 pp='<w:jc w:val="'+align+'"/>'
 if name.startswith('Heading'):pp+='<w:keepNext/><w:outlineLvl w:val="'+('0' if name=='Heading1' else '1')+'"/>'
 if name in ['Caption','TableText','TableHead','TOC1']:pp+='<w:spacing w:after="80" w:line="250" w:lineRule="auto"/>'
 stylesxml+=f'<w:style w:type="paragraph" w:styleId="{name}"><w:name w:val="{name}"/><w:pPr>{pp}</w:pPr><w:rPr><w:sz w:val="{size}"/><w:color w:val="{color}"/>'+('<w:b/>' if bold else '')+'</w:rPr></w:style>'
stylesxml+='</w:styles>'
REL='http://schemas.openxmlformats.org/package/2006/relationships'
def rels(items):return '<Relationships xmlns="'+REL+'">'+''.join(f'<Relationship Id="{i}" Type="{NSR}/{typ}" Target="{target}"/>' for i,typ,target in items)+'</Relationships>'
ct='<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Default Extension="png" ContentType="image/png"/>'+''.join(f'<Override PartName="/word/{p}.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.{t}+xml"/>' for p,t in [('document','document.main'),('styles','styles'),('footer1','footer'),('settings','settings')])+'</Types>'
with zipfile.ZipFile(O/'Bao_cao_cuoi_ky_PBL3_LTM.docx','w',zipfile.ZIP_DEFLATED) as z:
 z.writestr('[Content_Types].xml',ct);z.writestr('_rels/.rels',rels([('root','officeDocument','word/document.xml')]))
 z.writestr('word/document.xml',xml);z.writestr('word/styles.xml',stylesxml);z.writestr('word/settings.xml',f'<w:settings xmlns:w="{NSW}"><w:updateFields w:val="true"/></w:settings>')
 z.writestr('word/footer1.xml',f'<w:ftr xmlns:w="{NSW}"><w:p><w:pPr><w:jc w:val="center"/></w:pPr><w:r><w:t>PBL3–LTM  |  </w:t></w:r><w:fldSimple w:instr="PAGE"/></w:p></w:ftr>')
 items=[('styles','styles','styles.xml'),('footer','footer','footer1.xml'),('settings','settings','settings.xml')]
 for i,name in enumerate(images,1):items.append((f'img{i}','image','media/'+name));z.write(O/'hinh'/name,'word/media/'+name)
 z.writestr('word/_rels/document.xml.rels',rels(items))
# Plain text/Markdown source for quick review; no code listings in the report.
md=['# '+TITLE]
for pg in PAGES:
 md.append('\n## '+pg['title'])
 for b in pg['blocks']:
  if b[0] in ['p','h','eq']:md.append(('### ' if b[0]=='h' else '')+b[1])
  elif b[0]=='figure':md.append(f'![{b[2]}](hinh/{b[1]})')
  elif b[0]=='table':md.extend([b[1],'| '+' | '.join(b[2])+' |','| '+' | '.join(['---']*len(b[2]))+' |']+['| '+' | '.join(str(x) for x in row)+' |' for row in b[3]])
(O/'Bao_cao_cuoi_ky_PBL3_LTM.md').write_text('\n\n'.join(md))
print('Exported',len(PAGES),'sections;',len(images),'figures;',len(table_list),'tables; PDF last section starts at',max(page_map.values()))
