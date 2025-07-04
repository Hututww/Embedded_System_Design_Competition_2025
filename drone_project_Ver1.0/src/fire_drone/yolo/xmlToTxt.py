import os
import xml.etree.ElementTree as ET

# VOC标签路径
xml_dir = 'data/labels/val'
# YOLO标签输出路径
txt_dir = 'data/labels/val'
os.makedirs(txt_dir, exist_ok=True)

# 图片尺寸可从xml中读取
def convert(size, box):
    dw = 1. / size[0]
    dh = 1. / size[1]
    x = (box[0] + box[1]) / 2.0
    y = (box[2] + box[3]) / 2.0
    w = box[1] - box[0]
    h = box[3] - box[2]
    return (x * dw, y * dh, w * dw, h * dh)

for xml_file in os.listdir(xml_dir):
    if not xml_file.endswith('.xml'):
        continue
    tree = ET.parse(os.path.join(xml_dir, xml_file))
    root = tree.getroot()
    size = root.find('size')
    w = int(size.find('width').text)
    h = int(size.find('height').text)
    txt_path = os.path.join(txt_dir, xml_file.replace('.xml', '.txt'))
    with open(txt_path, 'w') as f:
        for obj in root.iter('object'):
            cls = obj.find('name').text
            if cls != 'fire':
                continue
            xmlbox = obj.find('bndbox')
            b = (float(xmlbox.find('xmin').text), float(xmlbox.find('xmax').text),
                 float(xmlbox.find('ymin').text), float(xmlbox.find('ymax').text))
            bb = convert((w, h), b)
            f.write(f"0 {' '.join([str(a) for a in bb])}\n")