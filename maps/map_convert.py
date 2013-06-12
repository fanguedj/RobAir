#!/usr/bin/env python
import sys
import ImageDraw

from PIL import Image

if __name__ == '__main__':
    im = Image.open(sys.argv[1])
    im = im.convert('RGB')    
    #draw = ImageDraw.Draw(im)
    #draw.ellipse((20,20,50,50), fill=(0,200,20))
    #del draw
    im.save(sys.argv[1]+'.jpg')
