while True:
    try:
        h = float(input())
        if (h > 0):
            s = h
            a = h
            for i in range(5):
                s = s + a
                a = a/2.0
            print round(s,6)
            print round(a,6)
    except:
        break


