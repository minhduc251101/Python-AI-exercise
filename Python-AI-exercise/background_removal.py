from rembg import remove
# input_path = Path("images") / "vecteezy_smartphone-and-mobile-phone_11047522.png"
with open('/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/Python-AI-exercise/vecteezy_smartphone-and-mobile-phone_11047522.png',"rb") as i:
    with open('output1.png', 'wb') as o:
        input = i.read()
        output = remove(input, force_return_bytes=True)
        o.write(output)