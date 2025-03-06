filename = "test.bin"

with open(filename, "wb") as f:
    # 20 MB file
    for i in range(256) :
        for j in range(128 * 1024) :
            f.write(i.to_bytes(1, byteorder='big'))
