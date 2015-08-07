#ifndef LADAR_DISTANCE_FILTER_H
#define LADAR_DISTANCE_FILTER_H

// Definition of the distances used for ladar filtering
// ordered as minX, maxX, minY, maxY, minZ, maxZ

// Close distances
static double closeDistances[] = {-0.5, 1.4, // X
				  -1.4, 1.3, // Y
				  -0.4, 2.4  // Z
};

// Far distances
static double farDistances[] = {-9.5, 9.5, // X
				-9.5, 9.5, // Y
				-9.5, 9.5  // Z
};


#endif
