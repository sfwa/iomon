/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "coord.h"

#define SCALE 0.0000001

enum pt_poly_status_t is_latlng_in_poly(int32_t pt_lat, int32_t pt_lng,
        uint8_t num_points, int32_t lats[], int32_t lngs[]) {
    /*
    Check lat/long is within boundary.
    See http://msdn.microsoft.com/en-us/library/cc451895.aspx

    Scaling of all lat and lng points is 1/10,000,000 of a degree.

    Does not work at the poles, and probably also doesn't work across the
    -180/180 split. Also doesn't do any correction for spherical surfaces so
    won't work for very large polygons (more than a few degrees across).
    */
    uint8_t in_bounds = 0, i, j;
    double p_lat, p_lng, i_lat, i_lng, j_lat, j_lng;

    p_lat = (double)pt_lat * SCALE;
    p_lng = (double)pt_lng * SCALE;

    if (p_lat < -90.0 || p_lat > 90.0 || p_lng < -180.0 || p_lng > 180.0) {
        return INVALID_POINT;
    } else if (num_points == 0 || lats == 0 || lngs == 0) {
        return IN_BOUNDS;
    }

    for (i = 0, j = num_points - 1; i < num_points; i++) {
        /* Only validate i points because each point in the polygon is used as
        both j and i */
        i_lat = (double)lats[i] * SCALE;
        i_lng = (double)lngs[i] * SCALE;
        j_lat = (double)lats[j] * SCALE;
        j_lng = (double)lngs[j] * SCALE;

        if (i_lat < -90.0 || i_lat > 90.0 || i_lng < -180.0 || i_lng > 180.0) {
            return INVALID_POLY;
        }

        /* Check if line segment points are either side of the POI,
        longitudinally */
        if ((i_lng < p_lng && j_lng >= p_lng) || (j_lng < p_lng && i_lng >= p_lng)) {
            /* See which side of the line segment the point is on */
            double ij_grad = (p_lng - i_lng) * (j_lat - i_lat) / (j_lng - i_lng);

            if (i_lat + ij_grad < p_lat) {
                in_bounds = ~in_bounds;
            }
        }
        j = i;
    }

    return in_bounds != 0 ? IN_BOUNDS : OUT_OF_BOUNDS;
}
