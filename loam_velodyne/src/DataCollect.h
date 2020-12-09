#ifndef DATACOLLECT_H
#define DATACOLLECT_H

double transformLat(double x, double y);
double transformLon(double x, double y);
void transformFromWGSToGCJ(double longitude, double latitude, double* newLng, double* newLat);
double getEuclideanDistance(double this_x, double this_y, double other_x, double other_y);
double getEuclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2);
double getHeadingDifferenceRadian(double headingAngle);
void rotateGPSCoordinate(double x, double y, double* x_after, double* y_after, double x_diff, double y_diff, double heading_diff);

#endif