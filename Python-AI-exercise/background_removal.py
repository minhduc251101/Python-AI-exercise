from rembg import remove

with open('/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/Python-AI-exercise/—Pngtree—cute wallpapers cats wallpapers hd_2615259.png',"rb") as i:
    with open('output.png', 'wb') as o:
        input = i.read()
        output = remove(input, force_return_bytes=True)
        o.write(output)