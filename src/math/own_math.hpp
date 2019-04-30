//
// Created by johannes on 4/18/19.
//

#ifndef BABOCAM_OWN_MATH_HPP
#define BABOCAM_OWN_MATH_HPP

/**
 * mathematische Methode, der einen Wert in einem Bereich verifiziert.
 * Verlässt dieser den angegebenen Bereich, wird das Minimum/Maximum zurück gegeben.
 * @param pref Der gewünschte Wert
 * @param min Minimale Grenze
 * @param max Maximale Grenze
 * @return Wenn min < wert < max, dann wert, ansonsten min bzw. max
 */
static double range(double pref, double min, double max) {
    if(pref < min) return min;
    if(pref > max) return max;
    return pref;
}

/**
 * verifiziert Wert in einem Bereich wie @see { range }, allerdings auch für negative Zahlen.
 * der absolute Wert der Zahl muss in dem Bereich liegen.
 *
 * @param val
 * @param abs_min
 * @param abs_max
 * @return
 */
static double absRange(double val, double abs_min, double abs_max) {
    if(val > 0) {
        return range(val, abs_min, abs_max);
    } else {
        return range(val, -abs_max, -abs_min);
    }
}

#endif //BABOCAM_OWN_MATH_HPP
