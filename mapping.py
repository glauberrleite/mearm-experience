def mapping(value,  anglesLimits, limits):
        'A function that translates a value within a range of angles to a value within the DutyCycle Limits'
        span = angleLimits[1] - angleLimits[0]
        scaled = float(value - angleLimits[0])/float(span)
        return limits[0] + (scaled * (limits[1] - limits[0]))

