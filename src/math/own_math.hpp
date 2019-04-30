//
// Created by johannes on 4/18/19.
//

#ifndef BABOCAM_OWN_MATH_HPP
#define BABOCAM_OWN_MATH_HPP

static double range(double pref, double min, double max) {
    if(pref < min) return min;
    if(pref > max) return max;
    return pref;
}

static double absRange(double val, double abs_min, double abs_max) {
    if(val > 0) {
        return range(val, abs_min, abs_max);
    } else {
        return range(val, -abs_max, -abs_min);
    }
}

#endif //BABOCAM_OWN_MATH_HPP
