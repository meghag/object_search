#ifndef __VAN_DER_COPUT_H__
#define __VAN_DER_COPUT_H__

#include <tf/tf.h>

class VanDerCorput {
    public :
    static double vdc(int n, double base = 2);

    //int bases[] = {2,3,5,7,11,13,17,19,23,29};

    //get the nth element of a dense sequence of poses filling 0..1 in x, y, z and the SO(3) for rotation
    // based on van der corput sequence with relatively prime bases
    static tf::Pose vdc_pose(int n);

    static tf::Pose vdc_pose_bound(tf::Vector3 min, tf::Vector3 max, int n);

};



#endif
