"""Export the editable SVG to existing iOS slots; requires ImageMagick convert.

Development helper only. Xcode builds use the committed PNGs.
"""
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

resources = Path(__file__).resolve().parent / 'RobotTeleop/Resources'
source = (resources / 'AppIcon.svg').read_bytes()
catalog = resources / 'Assets.xcassets'


def render(svg, size, target, *, opaque):
    subprocess.run(['convert', '-background', 'none', 'svg:-', '-resize', f'{size}x{size}',
                    f'{"PNG24" if opaque else "PNG32"}:{target}'], input=svg, check=True)


if __name__ == '__main__':
    slots = catalog / 'AppIcon.appiconset'
    for item in json.loads((slots / 'Contents.json').read_text())['images']:
        size = int(float(item['size'].split('x')[0]) * int(item['scale'][:-1]))
        render(source, size, slots / item['filename'], opaque=True)
    ET.register_namespace('', 'http://www.w3.org/2000/svg')
    icon = ET.fromstring(source)
    mark = next(child for child in icon if child.get('id') == 'mark')
    icon[:] = [mark]
    icon.set('viewBox', '196 196 632 632')
    icon.set('width', '632')
    icon.set('height', '632')
    slots = catalog / 'BrandMark.imageset'
    for item in json.loads((slots / 'Contents.json').read_text())['images']:
        render(ET.tostring(icon), 34 * int(item['scale'][:-1]), slots / item['filename'], opaque=False)
