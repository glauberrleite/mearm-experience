def mapping(value,  anglesLimits, limits):
        'A function that translates a value within a range of angles to a value within the DutyCycle Limits'
        span = anglesLimits[1] - anglesLimits[0]
        scaled = float(value - anglesLimits[0])/float(span)
        return limits[0] + (scaled * (limits[1] - limits[0]))

print mapping(-30, [-75, 0], [5, 10])
