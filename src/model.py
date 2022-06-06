def inputFilter(value):
    if value < 60:
        return 50

    elif value > 120:
        return 120

    else:
        return int(value)


def loadToDegree(load: float, a=0.9447612500347906, b=-6.5668370833003396):
    return a * load + b


def degreeToLoad(degree: int, a=1.0462126454570502, b=7.931256436573569):
    return a * degree + b
