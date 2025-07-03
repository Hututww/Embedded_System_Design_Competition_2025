import os

ann_dir = 'data/Annotations'
img_dir = 'data/JPEGImages'

ann_files = {os.path.splitext(f)[0] for f in os.listdir(ann_dir) if f.endswith('.xml')}
img_files = {os.path.splitext(f)[0] for f in os.listdir(img_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png'))}

common = ann_files & img_files

for f in os.listdir(ann_dir):
    name, ext = os.path.splitext(f)
    if ext == '.xml' and name not in common:
        os.remove(os.path.join(ann_dir, f))

for f in os.listdir(img_dir):
    name, ext = os.path.splitext(f)
    if ext.lower() in ('.jpg', '.jpeg', '.png') and name not in common:
        os.remove(os.path.join(img_dir, f))

print(f"已完成清理，只保留了{len(common)}对图片和标注。")