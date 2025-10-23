#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H
#include <stdint.h>
#include <types.h>

void dp_run_doppler(radarcube_t data, detmatrix_t out);
void dp_run_anglefft(radarcube_t data, struct detected_point *points, size_t len, int16imre_t *out);

int dp_run_cfar(detmatrix_t detmatrix);
int dp_run_anglecfar(int16imre_t *angles, size_t len);

#endif /* DATAPROCESSING_H */
