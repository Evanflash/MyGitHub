#include "common.h"

namespace ALOAM{

    int N_SCANS = 32;
    float MINMUM_RANGE = 0.1;
    double SCAN_PERIOD = 0.1;
    double DISTANCE_SQ_THRESHOLD = 25;
    double NEARBY_SCAN = 2.5;
    int DISTORTION = 0;

    
    string cornerPointsSharpName = "cornerPointsSharp";
    string cornerPointsLessSharpName = "cornerPointsLessSharp";
    string surfPointsFlatName = "surfPointsFlat";
    string surfPointsLessFlatName = "surfPointsLessFlat";
    string laserCloudFullResName = "laserCloudFullRes";

    double para_q[4] = {0, 0, 0, 1};
    double para_t[3] = {0, 0, 0};

    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};

    string cornerLast = "cornerLast";
    string surfLast = "surfLast";
    string fullRes = "fullRes";

    int laserCloudCenWidth = 10;
    int laserCloudCenHeight = 10;
    int laserCloudCenDepth = 5;

    int laserCloudWidth = 21;
    int laserCloudHeight = 21;
    int laserCloudDepth = 11;
}